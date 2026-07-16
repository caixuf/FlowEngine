/**
 * actuator_node.c — 执行器节点插件 (SocketCAN 真车输出)
 *
 * 把 control/raw_cmd (ControlRaw) 转成 SocketCAN 帧发到 CAN 总线，
 * 驱动真实 ESC（电子调速器）+ 转向舵机。
 *
 * 这是 FlowEngine 接入真实硬件的最后一环：
 *   control_node → control/raw_cmd → safety_control_node (限幅) → control/cmd
 *     → [actuator_node] → CAN 总线 → ESC/舵机
 *
 * 注意：订阅的是 safety_control_node 限幅后的 control/cmd（ControlCmd），
 * 不是 control_node 原始的 control/raw_cmd（ControlRaw）。safety_control_node
 * 负责 TTC/行人/横向交叉防护和限幅，真车上必须经过它，否则裸 raw_cmd 直发
 * CAN 无任何安全闸。
 *
 * 设计要点：
 *   - SocketCAN 是 Linux 内核标准 CAN 抽象：把 CAN 控制器映射成网络接口
 *     (can0)，用 socket(PF_CAN, SOCK_RAW, CAN_RAW) + write() 发帧，
 *     和发以太网包一样。树莓派 + MCP2515 SPI-CAN 扩展板或 USB-CAN 都走这条路。
 *   - CAN 帧格式（ID + data 字节）完全取决于具体 ESC/舵机控制器协议，
 *     本节点把编码做成参数化，默认用一个常见 RC 小车布局，社区用户
 *     按自己硬件改 can_throttle_id / can_steering_id 和 encode 函数即可。
 *   - 无 CAN 设备时（沙箱/开发机/CI）socket 创建失败，自动降级为
 *     日志输出模式，不阻塞流水线其余节点运行。
 *
 * 话题契约：
 *   输入: control/cmd  (ControlCmd 二进制，safety_control_node 限幅后发布)
 *   无输出 topic（输出是物理 CAN 帧）
 *
 * 典型 pipeline.json 配置:
 *   {
 *     "name": "actuator",
 *     "library": "libactuator_node.so",
 *     "params": {
 *       "can_interface": "can0",
 *       "can_throttle_id": "0x100",
 *       "can_steering_id": "0x101",
 *       "throttle_scale": 1000.0,
 *       "steering_scale": 1000.0,
 *       "enable": true,
 *       "dry_run": false
 *     }
 *   }
 *
 * 树莓派部署前置（MCP2515 SPI-CAN 扩展板示例）:
 *   # dtoverlay 配置 /boot/firmware/config.txt:
 *   dtparam=spi=on
 *   dtoverlay=mcp2515-can0,oscillator=16000000,interrupt=25
 *   # 启动接口:
 *   sudo ip link set can0 type can bitrate 500000
 *   sudo ip link set can0 up
 *
 * 编译依赖: linux/can.h (Linux 3.x+ 内核头文件，树莓派/Jetson/Ubuntu 自带)
 */

#include "node_plugin.h"
#include "adas_msgs_gen.h"
#include "transport.h"
#include "discovery.h"
#include "logger.h"

#include <math.h>
#include <pthread.h>
#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

/* SocketCAN 头文件 — 仅 Linux 可用。非 Linux 环境（CI/沙箱）降级为 dry-run。 */
#ifdef __linux__
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#define HAVE_SOCKETCAN 1
#else
#define HAVE_SOCKETCAN 0
/* 非 Linux 占位类型，让下面代码能编译过（实际运行走 dry_run 路径） */
struct can_frame { uint32_t can_id; uint8_t can_dlc; uint8_t data[8]; };
#define CAN_RAW 0
#define PF_CAN  0
#define AF_CAN  0
#endif

/* ── 节点状态 ─────────────────────────────────────────────── */

static struct {
    Transport*        transport;
    DiscoveryManager* discovery;

    /* SocketCAN 句柄，<0 表示未连接（dry-run 模式） */
    int               can_sock;

    /* 配置参数（pipeline.json 注入） */
    char     can_interface[16];    /* "can0" */
    uint32_t can_throttle_id;      /* 油门/刹车帧 ID，默认 0x100 */
    uint32_t can_steering_id;      /* 转向帧 ID，默认 0x101 */
    uint32_t can_status_id;        /* 心跳/状态帧 ID，默认 0x102 */
    float    throttle_scale;       /* [0,1] → [0, scale] 量化值 */
    float    steering_scale;       /* [-1,1] → [-scale, scale] 量化值 */
    int      bitrate;              /* CAN 波特率（仅日志提示，实际由 ip link 设置） */
    int      enabled;              /* 总开关，false=不接 CAN 也不订阅 */
    int      dry_run;              /* true=只日志不发真实 CAN 帧（开发调试） */

    /* 统计 */
    uint64_t frames_sent;          /* 成功 write 的 CAN 帧数 */
    uint64_t frames_failed;        /* write 失败计数 */
    uint64_t cmds_received;        /* 收到的 control/raw_cmd 数 */
    uint32_t last_seq;             /* 最近一次指令 seq */

    /* 心跳线程：定期发 status 帧让 ESC 知道控制器还活着 */
    pthread_t hb_thread;
    volatile int hb_running;
    volatile int should_stop;
    int      heartbeat_hz;         /* 心跳频率，默认 10Hz */
} g;

/* ── SocketCAN 工具函数 ──────────────────────────────────── */

/* 打开 CAN socket 并绑定到指定接口（如 can0）。成功返回 fd，失败返回 -1。 */
static int can_open(const char* ifname) {
#if HAVE_SOCKETCAN
    int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (s < 0) {
        LOG_WARN("actuator", "socket(PF_CAN) failed: %s", strerror(errno));
        return -1;
    }
    struct ifreq ifr;
    memset(&ifr, 0, sizeof(ifr));
    snprintf(ifr.ifr_name, sizeof(ifr.ifr_name), "%s", ifname);
    if (ioctl(s, SIOCGIFINDEX, &ifr) < 0) {
        LOG_WARN("actuator", "CAN interface '%s' not found: %s (ip link set %s up?)",
                 ifname, strerror(errno), ifname);
        close(s);
        return -1;
    }
    struct sockaddr_can addr;
    memset(&addr, 0, sizeof(addr));
    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(s, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        LOG_WARN("actuator", "bind CAN socket to %s failed: %s", ifname, strerror(errno));
        close(s);
        return -1;
    }
    LOG_INFO("actuator", "CAN socket opened on %s (ifindex=%d)", ifname, ifr.ifr_ifindex);
    return s;
#else
    (void)ifname;
    LOG_INFO("actuator", "SocketCAN not available on this platform (non-Linux), dry-run only");
    return -1;
#endif
}

/* 发送一帧 CAN。dry_run 或 socket 未连接时只日志。返回 0 成功。 */
static int can_send(uint32_t can_id, const uint8_t* data, uint8_t dlc) {
    if (dlc > 8) dlc = 8;
    if (g.dry_run || g.can_sock < 0) {
        /* 降级模式：只打印，不发真实帧 */
        char hex[32] = {0};
        for (int i = 0; i < dlc; i++) snprintf(hex + i*3, 4, "%02x ", data[i]);
        LOG_INFO("actuator", "[dry-run] CAN 0x%03X [%d] %s", can_id, dlc, hex);
        return 0;
    }
#if HAVE_SOCKETCAN
    struct can_frame frame;
    memset(&frame, 0, sizeof(frame));
    frame.can_id  = can_id;
    frame.can_dlc = dlc;
    memcpy(frame.data, data, dlc);
    ssize_t n = write(g.can_sock, &frame, sizeof(frame));
    if (n != sizeof(frame)) {
        g.frames_failed++;
        LOG_WARN("actuator", "CAN write failed (id=0x%03X): %s", can_id, strerror(errno));
        return -1;
    }
    g.frames_sent++;
    return 0;
#else
    g.frames_sent++;
    return 0;
#endif
}

/* ── 控制指令 → CAN 帧编码 ──────────────────────────────────
 * 默认编码方案（RC 小车常见布局，社区用户按自己硬件改这里）：
 *
 * 油门/刹车帧 (can_throttle_id, 8 bytes):
 *   [0-1] throttle 量化值 (uint16 LE, 0..throttle_scale)
 *   [2-3] brake    量化值 (uint16 LE, 0..throttle_scale)
 *   [4]   gear     (-1=REV, 0=N, 1=DRIVE)
 *   [5]   e_stop   (0/1)
 *   [6-7] reserved
 *
 * 转向帧 (can_steering_id, 4 bytes):
 *   [0-1] steering 量化值 (int16 LE, -steering_scale..+steering_scale)
 *   [2-3] seq (uint16 LE)
 */
static void encode_throttle_frame(float throttle, float brake,
                                  int gear, int e_stop, uint8_t* out) {
    memset(out, 0, 8);
    /* 钳位到 [0,1] */
    if (throttle < 0) throttle = 0;
    if (throttle > 1) throttle = 1;
    if (brake    < 0) brake    = 0;
    if (brake    > 1) brake    = 1;
    uint16_t t_q = (uint16_t)(throttle * g.throttle_scale);
    uint16_t b_q = (uint16_t)(brake    * g.throttle_scale);
    out[0] = t_q & 0xFF; out[1] = (t_q >> 8) & 0xFF;
    out[2] = b_q & 0xFF; out[3] = (b_q >> 8) & 0xFF;
    out[4] = (uint8_t)gear;
    out[5] = e_stop ? 1 : 0;
}

static void encode_steering_frame(float steering, uint32_t seq, uint8_t* out) {
    if (steering < -1) steering = -1;
    if (steering >  1) steering =  1;
    int16_t s_q = (int16_t)(steering * g.steering_scale);
    out[0] = s_q & 0xFF; out[1] = (s_q >> 8) & 0xFF;
    out[2] = seq & 0xFF; out[3] = (seq >> 8) & 0xFF;
}

/* ── 订阅回调：收到 control/cmd（safety_control 限幅后） ─────── */

static void on_control_cmd(const Message* msg, void* user_data) {
    (void)user_data;
    if (!msg || !g.enabled) return;
    if (msg->data_size == 0) return;

    /* 二进制路径：ControlCmd 反序列化（safety_control_node 发布） */
    ControlCmd cmd;
    if (ControlCmd_deserialize(&cmd, (const uint8_t*)msg->data, msg->data_size) != 0) {
        LOG_WARN("actuator", "ControlCmd deserialize failed (size=%u)", msg->data_size);
        return;
    }

    g.cmds_received++;
    g.last_seq = cmd.seq;

    /* 紧急停车：ControlCmd 自带 emergency_stop 标志（safety_control 设置） */
    int e_stop = cmd.emergency_stop ? 1 : 0;

    /* 编码 + 发送油门/刹车帧（gear 直接用 ControlCmd 的 gear 字段） */
    uint8_t tbuf[8];
    encode_throttle_frame(cmd.throttle, cmd.brake, (int)cmd.gear, e_stop, tbuf);
    can_send(g.can_throttle_id, tbuf, 8);

    /* 编码 + 发送转向帧 */
    uint8_t sbuf[4];
    encode_steering_frame(cmd.steering, cmd.seq, sbuf);
    can_send(g.can_steering_id, sbuf, 4);

    /* 周期性日志（避免刷屏，每 50 条打一次） */
    if (g.cmds_received % 50 == 1) {
        LOG_INFO("actuator", "cmd #%u thr=%.2f brk=%.2f steer=%.3f gear=%d estop=%d → CAN sent=%lu fail=%lu",
                 cmd.seq, cmd.throttle, cmd.brake, cmd.steering, (int)cmd.gear, e_stop,
                 (unsigned long)g.frames_sent, (unsigned long)g.frames_failed);
    }
}

/* ── 心跳线程：定期发 status 帧让 ESC 知道控制器在线 ─────────── */

static void* heartbeat_thread(void* arg) {
    (void)arg;
    pthread_setname_np(pthread_self(), "act_hb");
    long period_us = 1000000L / (g.heartbeat_hz > 0 ? g.heartbeat_hz : 10);
    while (g.hb_running) {
        usleep((unsigned long)period_us);
        if (!g.hb_running || !g.enabled) break;
        /* status 帧: [0-3] frames_sent, [4-7] cmds_received */
        uint8_t buf[8];
        memcpy(buf,     &g.frames_sent,    4);
        memcpy(buf + 4, &g.cmds_received,  4);
        can_send(g.can_status_id, buf, 8);
    }
    return NULL;
}

/* ── 参数解析 ─────────────────────────────────────────────── */

static uint32_t parse_hex_int(const char* json, const char* key, uint32_t default_val) {
    if (!json || !key) return default_val;
    char pat[64];
    snprintf(pat, sizeof(pat), "\"%s\"", key);
    const char* p = strstr(json, pat);
    if (!p) return default_val;
    p = strchr(p + strlen(pat), ':');
    if (!p) return default_val;
    p++;
    while (*p == ' ' || *p == '\t') p++;
    if (p[0] == '0' && (p[1] == 'x' || p[1] == 'X')) {
        return (uint32_t)strtoul(p + 2, NULL, 16);
    }
    return (uint32_t)strtoul(p, NULL, 10);
}

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

/* ── NodePlugin 实现 ─────────────────────────────────────── */

static const char* s_inputs[]  = { "control/cmd", NULL };
static const char* s_outputs[] = { NULL };

static NodePlugin s_plugin;

static int actuator_init(MessageBus* bus, Transport* transport,
                         DiscoveryManager* discovery, Scheduler* scheduler,
                         const char* params_json) {
    (void)bus; (void)scheduler;
    memset(&g, 0, sizeof(g));
    g.transport = transport;
    g.discovery = discovery;
    g.can_sock  = -1;

    /* 默认参数 */
    snprintf(g.can_interface, sizeof(g.can_interface), "can0");
    g.can_throttle_id  = 0x100;
    g.can_steering_id  = 0x101;
    g.can_status_id    = 0x102;
    g.throttle_scale   = 1000.0f;
    g.steering_scale   = 1000.0f;
    g.bitrate          = 500000;
    g.enabled          = 1;
    g.dry_run          = 0;
    g.heartbeat_hz     = 10;

    if (params_json) {
        parse_string(params_json, "can_interface", g.can_interface, sizeof(g.can_interface), "can0");
        g.can_throttle_id = parse_hex_int(params_json, "can_throttle_id", 0x100);
        g.can_steering_id = parse_hex_int(params_json, "can_steering_id", 0x101);
        g.can_status_id   = parse_hex_int(params_json, "can_status_id",   0x102);
        g.throttle_scale  = (float)parse_double(params_json, "throttle_scale", 1000.0);
        g.steering_scale  = (float)parse_double(params_json, "steering_scale", 1000.0);
        g.bitrate         = parse_int(params_json, "bitrate", 500000);
        g.enabled         = parse_int(params_json, "enable", 1);
        g.dry_run         = parse_int(params_json, "dry_run", 0);
        g.heartbeat_hz    = parse_int(params_json, "heartbeat_hz", 10);
    }

    if (!g.enabled) {
        LOG_INFO("actuator", "disabled by config (enable=0), will not subscribe control/raw_cmd");
        return 0;
    }

    /* 打开 SocketCAN（失败则自动降级为 dry_run） */
    if (!g.dry_run) {
        g.can_sock = can_open(g.can_interface);
        if (g.can_sock < 0) {
            LOG_WARN("actuator", "CAN open failed on '%s', falling back to dry-run "
                     "(真实硬件部署时检查: ip link set %s up)",
                     g.can_interface, g.can_interface);
            g.dry_run = 1;
        }
    }

    /* 订阅 control/cmd（safety_control_node 限幅后的安全指令） */
    transport_subscribe(transport, "control/cmd", on_control_cmd, NULL);
    discovery_advertise(discovery, "control/cmd", CONTROLCMD_TYPE_ID, CAP_SUBSCRIBER, 0);

    LOG_INFO("actuator", "initialized: if=%s thr_id=0x%03X steer_id=0x%03X "
             "thr_scale=%.0f steer_scale=%.0f bitrate=%d %s",
             g.can_interface, g.can_throttle_id, g.can_steering_id,
             g.throttle_scale, g.steering_scale, g.bitrate,
             g.dry_run ? "[DRY-RUN]" : "[LIVE-CAN]");

    if (g.dry_run) {
        LOG_INFO("actuator", "DRY-RUN 模式：CAN 帧只打印不发真实总线 "
                 "(无 %s 接口或 dry_run=true)。树莓派部署: "
                 "sudo ip link set %s type can bitrate %d && sudo ip link set %s up",
                 g.can_interface, g.can_interface, g.bitrate, g.can_interface);
    }
    return 0;
}

static int actuator_start(void) {
    if (!g.enabled) return 0;
    g.hb_running = 1;
    g.should_stop = 0;
    if (pthread_create(&g.hb_thread, NULL, heartbeat_thread, NULL) != 0) {
        LOG_WARN("actuator", "heartbeat thread create failed");
        return -1;
    }
    LOG_INFO("actuator", "started (heartbeat %dHz, %s)",
             g.heartbeat_hz, g.dry_run ? "dry-run" : "live CAN");
    node_announce_self(g.transport, &s_plugin);
    return 0;
}

static void actuator_stop(void) {
    g.should_stop = 1;
    g.hb_running = 0;
}

static void actuator_cleanup(void) {
    g.hb_running = 0;
    if (g.hb_thread) { pthread_join(g.hb_thread, NULL); g.hb_thread = 0; }
#if HAVE_SOCKETCAN
    if (g.can_sock >= 0) { close(g.can_sock); g.can_sock = -1; }
#endif
    LOG_INFO("actuator", "cleanup: cmds=%lu sent=%lu fail=%lu",
             (unsigned long)g.cmds_received,
             (unsigned long)g.frames_sent,
             (unsigned long)g.frames_failed);
}

static int actuator_health(void) {
    /* 健康判定：最近 10 秒内若收到过指令且无连续失败，视为健康 */
    if (!g.enabled) return 0;
    if (g.frames_failed > g.frames_sent && g.frames_failed > 100) return -1;
    return 0;
}

static NodePlugin s_plugin = {
    .api_version   = NODE_PLUGIN_API_VERSION,
    .name          = "actuator",
    .version       = "1.0.0",
    .description   = "SocketCAN actuator driver (control/raw_cmd → CAN bus → ESC/servo)",
    .input_topics  = s_inputs,
    .output_topics = s_outputs,
    .init          = actuator_init,
    .start         = actuator_start,
    .stop          = actuator_stop,
    .cleanup       = actuator_cleanup,
    .health        = actuator_health,
};

NodePlugin* node_get_plugin(void) { return &s_plugin; }
