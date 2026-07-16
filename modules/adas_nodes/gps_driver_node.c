/**
 * gps_driver_node.c — GPS 串口驱动节点插件 (NMEA 0183 真车输入)
 *
 * 从真实 GPS 串口读取 NMEA 0183 报文，解析为 GpsData 发布到 sensor/gps 话题。
 *
 * 这是 FlowEngine 接入真实 GNSS 硬件的第一环，替代仿真用的 sensor_model_node
 * （后者从 vehicle/state 真值合成带噪声的假 GPS）。真车上由本节点接管 sensor/gps，
 * 下游 fusion_node / control_node 拿到的是真实定位而非仿真真值。
 *
 * 工作流程：
 *   串口(/dev/ttyUSB0) → NMEA 0183 文本行 → nmea_parser → GpsData → sensor/gps
 *
 * 常见 GPS 模块（u-blox NEO-M8N、ATGM336H、中科微 AT6558、Quectel L76 等）
 * 默认都输出 NMEA 0183 语句（$GPGGA/$GPRMC 等），波特率 9600，可直接对接，
 * 无需额外驱动或厂商 SDK。
 *
 * 设计要点：
 *   - serial_port.h 封装 POSIX termios，按行读取（NMEA 是 \n 结尾的 ASCII 行协议），
 *     9600 波特率下一组 GGA+RMC 约 1Hz，read_line 超时 1000ms 足够覆盖。
 *   - 无串口设备时（沙箱/开发机/CI）serial_open 返回 NULL，自动降级为 dry-run 模式：
 *     只按 publish_hz 打印假的 NMEA 行到日志提示无硬件，不发任何数据，不阻塞流水线
 *     其余节点运行。
 *   - NmeaParser 内部累积 GGA/RMC 字段，单条语句字段缺失时合并上次状态，所以
 *     只要有 GGA 或 RMC 任一解析成功即可发布一次 GpsData。
 *
 * 话题契约：
 *   输入: 无（从串口硬件读）
 *   输出: sensor/gps (GpsData 二进制序列化, type_id=0x0596b0b7)
 *
 * 典型 pipeline.json 配置:
 *   {
 *     "name": "gps_driver",
 *     "library": "libgps_driver_node.so",
 *     "params": {
 *       "serial_port": "/dev/ttyUSB0",
 *       "baud_rate": 9600,
 *       "enable": true,
 *       "publish_hz": 10
 *     }
 *   }
 *
 * 树莓派部署前置（USB GPS 模块示例）:
 *   # 插入 USB GPS 后会出现 /dev/ttyUSB0，权限不足时:
 *   sudo usermod -aG dialout $USER
 *   # 或临时:
 *   sudo chmod 666 /dev/ttyUSB0
 *   # 验证数据（直接 cat 串口）:
 *   stty -F /dev/ttyUSB0 9600 raw -echo
 *   cat /dev/ttyUSB0
 *
 * 编译依赖: serial_port.h / nmea_parser.h（项目自带，无需外部库）
 */

#include "node_plugin.h"
#include "adas_msgs_gen.h"
#include "transport.h"
#include "discovery.h"
#include "logger.h"
#include "serial_port.h"
#include "nmea_parser.h"

#include <math.h>
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

/* ── 节点状态 ─────────────────────────────────────────────── */

static struct {
    Transport*        transport;
    DiscoveryManager* discovery;

    /* 串口句柄，NULL 表示未打开（dry-run 模式） */
    SerialPort*       serial;

    /* NMEA 解析器（累积 GGA/RMC 字段） */
    NmeaParser        parser;

    /* 配置参数（pipeline.json 注入） */
    char     serial_port[64];   /* "/dev/ttyUSB0" */
    int      baud_rate;         /* 9600 等常见值 */
    int      enabled;           /* 总开关，false=不读串口也不发布 */
    int      publish_hz;        /* 期望发布频率（dry-run 心跳 + discovery 通告用） */
    int      dry_run;           /* true=只日志不发数据（无硬件或调试） */

    /* 统计 */
    uint64_t sentences_parsed;  /* 成功解析的 NMEA 语句数 */
    uint64_t sentences_failed;  /* 解析失败的语句数 */
    uint64_t gps_published;     /* 发布到 sensor/gps 的帧数 */

    /* 读取线程 */
    pthread_t     thread;
    volatile int  thread_running;
    volatile int  should_stop;
} g;

/* ── 参数解析（手写 JSON 字符串解析，项目节点里不用 cJSON） ──── */

static double parse_double(const char* json, const char* key, double default_val) {
    if (!json || !key) return default_val;
    char pat[64];
    snprintf(pat, sizeof(pat), "\"%s\"", key);
    const char* p = strstr(json, pat);
    if (!p) return default_val;
    p = strchr(p + strlen(pat), ':');
    if (!p) return default_val;
    p++;
    while (*p == ' ' || *p == '\t') p++;
    return strtod(p, NULL);
}

static int parse_int(const char* json, const char* key, int default_val) {
    return (int)parse_double(json, key, (double)default_val);
}

static void parse_string(const char* json, const char* key, char* out, size_t out_sz, const char* default_val) {
    if (!json || !key || !out || out_sz == 0) { if (default_val) snprintf(out, out_sz, "%s", default_val); return; }
    char pat[64];
    snprintf(pat, sizeof(pat), "\"%s\"", key);
    const char* p = strstr(json, pat);
    if (!p) { if (default_val) snprintf(out, out_sz, "%s", default_val); return; }
    p = strchr(p + strlen(pat), ':');
    if (!p) { if (default_val) snprintf(out, out_sz, "%s", default_val); return; }
    p++;
    while (*p == ' ' || *p == '\t') p++;
    if (*p != '"') { if (default_val) snprintf(out, out_sz, "%s", default_val); return; }
    p++;
    size_t n = 0;
    while (*p && *p != '"' && n < out_sz - 1) out[n++] = *p++;
    out[n] = '\0';
}

/* ── 读取线程：循环 serial_read_line → nmea_parse_line → publish ─
 *
 * 真实模式：从串口逐行读 NMEA，解析成功则序列化发布到 sensor/gps。
 * dry-run 模式：无硬件，按 publish_hz 打印假 NMEA 行到日志，不发数据。
 */
static void* gps_reader_thread(void* arg) {
    (void)arg;
    pthread_setname_np(pthread_self(), "gps_reader");
    char line[256];
    long dry_period_us = 1000000L / (g.publish_hz > 0 ? g.publish_hz : 10);

    while (!g.should_stop) {
        if (g.dry_run || !g.serial) {
            /* 降级模式：按 publish_hz 打印假的 NMEA 行，提示无硬件，不发数据 */
            usleep((unsigned long)dry_period_us);
            if (g.should_stop) break;
            LOG_INFO("gps_driver", "[dry-run] 无串口硬件，模拟 NMEA: "
                     "$GPRMC,092750.000,A,2237.4960,N,11402.9740,E,0.02,31.66,280511,,,A*7E "
                     "(请接入真实 GPS 模块如 u-blox NEO-M8N / ATGM336H)");
            continue;
        }

        /* 真实模式：从串口读一行 NMEA，超时 1000ms */
        int n = serial_read_line(g.serial, line, sizeof(line), 1000);
        if (g.should_stop) break;
        if (n < 0) {
            /* 设备错误/关闭 */
            g.sentences_failed++;
            LOG_WARN("gps_driver", "serial_read_line error (n=%d), 设备可能断开", n);
            usleep(100000);  /* 避免错误时忙轮询 */
            continue;
        }
        if (n == 0) {
            /* 超时，无数据，正常情况（GPS 可能未定位） */
            continue;
        }

        /* 解析 NMEA 行 */
        GpsData gps;
        int rc = nmea_parse_line(&g.parser, line, &gps);
        if (rc == NMEA_OK) {
            g.sentences_parsed++;
            /* 序列化 + 发布到 sensor/gps */
            uint8_t buf[64];
            size_t  len = 0;
            if (GpsData_serialize(&gps, buf, &len) == 0 && len > 0) {
                transport_publish(g.transport, "sensor/gps", buf, (uint32_t)len);
                g.gps_published++;
                /* 周期性日志（避免刷屏，每 50 条打一次） */
                if (g.gps_published % 50 == 1) {
                    LOG_INFO("gps_driver", "GPS #%lu lat=%.6f lon=%.6f spd=%.2f hdg=%.1f acc=%.1f "
                             "(parsed=%lu fail=%lu)",
                             (unsigned long)g.gps_published,
                             gps.latitude, gps.longitude, gps.speed_mps,
                             gps.heading_deg, gps.accuracy_m,
                             (unsigned long)g.sentences_parsed,
                             (unsigned long)g.sentences_failed);
                }
            }
        } else {
            g.sentences_failed++;
            /* 非 GGA/RMC 语句（如 GSV/GSA）返回 UNSUPPORTED，不算错误，降低日志级别 */
            if (rc != NMEA_ERR_UNSUPPORTED) {
                LOG_DEBUG("gps_driver", "nmea_parse_line rc=%d: %s", rc, line);
            }
        }
    }
    return NULL;
}

/* ── NodePlugin 实现 ─────────────────────────────────────── */

static const char* s_inputs[]  = { NULL };
static const char* s_outputs[] = { "sensor/gps", NULL };

static NodePlugin s_plugin;

static int gps_driver_init(MessageBus* bus, Transport* transport,
                           DiscoveryManager* discovery, Scheduler* scheduler,
                           const char* params_json) {
    (void)bus; (void)scheduler;
    memset(&g, 0, sizeof(g));
    g.transport = transport;
    g.discovery = discovery;
    g.serial    = NULL;

    /* 默认参数 */
    snprintf(g.serial_port, sizeof(g.serial_port), "/dev/ttyUSB0");
    g.baud_rate  = 9600;
    g.enabled    = 1;
    g.publish_hz = 10;
    g.dry_run    = 0;

    if (params_json) {
        parse_string(params_json, "serial_port", g.serial_port, sizeof(g.serial_port), "/dev/ttyUSB0");
        g.baud_rate  = parse_int(params_json, "baud_rate", 9600);
        g.enabled    = parse_int(params_json, "enable", 1);
        g.publish_hz = parse_int(params_json, "publish_hz", 10);
        g.dry_run    = parse_int(params_json, "dry_run", 0);
    }

    /* 初始化 NMEA 解析器（清零累积状态） */
    nmea_parser_init(&g.parser);

    if (!g.enabled) {
        LOG_INFO("gps_driver", "disabled by config (enable=0), will not read serial");
        return 0;
    }

    /* 打开串口（失败则自动降级为 dry_run，不阻塞） */
    if (!g.dry_run) {
        g.serial = serial_open(g.serial_port, g.baud_rate);
        if (!g.serial) {
            LOG_WARN("gps_driver", "serial_open('%s', %d) failed, falling back to dry-run "
                     "(真实硬件部署时检查: ls /dev/ttyUSB* && sudo chmod 666 %s)",
                     g.serial_port, g.baud_rate, g.serial_port);
            g.dry_run = 1;
        }
    }

    /* 向 discovery 通告本节点发布 sensor/gps（GpsData, type_id=0x0596b0b7） */
    discovery_advertise(discovery, "sensor/gps", 0x0596b0b7u, CAP_PUBLISHER, 10.0);

    LOG_INFO("gps_driver", "initialized: dev=%s baud=%d hz=%d %s",
             g.serial_port, g.baud_rate, g.publish_hz,
             g.dry_run ? "[DRY-RUN]" : "[LIVE-SERIAL]");

    if (g.dry_run) {
        LOG_INFO("gps_driver", "DRY-RUN 模式：只打印假 NMEA 不发数据 "
                 "(无 %s 设备或 dry_run=true)。树莓派部署: "
                 "插入 USB GPS 模块后 ls /dev/ttyUSB*，权限不足加 dialout 组",
                 g.serial_port);
    }
    return 0;
}

static int gps_driver_start(void) {
    if (!g.enabled) return 0;
    g.thread_running = 1;
    g.should_stop = 0;
    if (pthread_create(&g.thread, NULL, gps_reader_thread, NULL) != 0) {
        LOG_WARN("gps_driver", "reader thread create failed");
        return -1;
    }
    LOG_INFO("gps_driver", "started (%s, %dHz target)",
             g.dry_run ? "dry-run" : "live serial", g.publish_hz);
    node_announce_self(g.transport, &s_plugin);
    return 0;
}

static void gps_driver_stop(void) {
    g.should_stop = 1;
    g.thread_running = 0;
}

static void gps_driver_cleanup(void) {
    g.thread_running = 0;
    g.should_stop = 1;
    if (g.thread) { pthread_join(g.thread, NULL); g.thread = 0; }
    if (g.serial) { serial_close(g.serial); g.serial = NULL; }
    LOG_INFO("gps_driver", "cleanup: parsed=%lu failed=%lu published=%lu",
             (unsigned long)g.sentences_parsed,
             (unsigned long)g.sentences_failed,
             (unsigned long)g.gps_published);
}

static int gps_driver_health(void) {
    /* 健康判定：解析失败数明显多于成功数且失败超过阈值，视为异常 */
    if (!g.enabled) return 0;
    if (g.sentences_failed > g.sentences_parsed && g.sentences_failed > 100) return -1;
    return 0;
}

static NodePlugin s_plugin = {
    .api_version   = NODE_PLUGIN_API_VERSION,
    .name          = "gps_driver",
    .version       = "1.0.0",
    .description   = "GPS serial driver (NMEA 0183 → GpsData → sensor/gps)",
    .input_topics  = s_inputs,
    .output_topics = s_outputs,
    .init          = gps_driver_init,
    .start         = gps_driver_start,
    .stop          = gps_driver_stop,
    .cleanup       = gps_driver_cleanup,
    .health        = gps_driver_health,
};

NodePlugin* node_get_plugin(void) { return &s_plugin; }
