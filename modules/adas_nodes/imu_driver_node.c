/**
 * imu_driver_node.c — IMU 串口驱动节点插件 (六轴惯性测量单元真车输入)
 *
 * 从真实 IMU 串口读取加速度计/陀螺仪数据，解析为 ImuData 发布到 sensor/imu 话题。
 *
 * IMU 是 FlowEngine 感知/定位栈的高频基础输入：
 *   - 为 fusion_node 的 EKF 提供 predict 增量：两帧 GPS/定位之间用陀螺仪 + 加速度计
 *     推演位姿，显著平滑定位输出，并在 GPS 短时丢星时维持连续性；
 *   - 为 SLAM（LiDAR/视觉）提供运动先验，辅助点云/特征帧间配准；
 *   - 为 control_node 提供 yaw_rate / accel 反馈，做横摆稳定与纵向平滑。
 *
 * 工作流程：
 *   串口(/dev/ttyUSB2) → IMU ASCII 行 → parse_imu_line → ImuData → sensor/imu
 *
 * 常见 IMU 模块与接入方式：
 *   - MPU6050：原生 I2C，社区多用串口转接板（CH340/CP2102 + MCU）输出 ASCII；
 *     若直接走 I2C，需把 read 方式从 serial_read_line 改为 I2C 块读。
 *   - ICM-42688 / BMI088：SPI 或 I2C，常配 MCU 转串口输出 ASCII/二进制。
 *   - Xsens MTi-300 / SBG Ellipse：商用 AHRS，串口二进制协议（需把 parse_imu_line
 *     改为二进制帧解析，并把 serial_read_line 换成 serial_read 按帧长读取）。
 *   - 维特智能 WIT-Motion、Bosch BNO085 串口模块：直接输出 ASCII 行，本实现兼容。
 * 本默认实现兼容 "ax,ay,az,gx,gy,gz,temp\n" 格式的 ASCII 串口 IMU 模块。
 *
 * 设计要点：
 *   - serial_port.h 封装 POSIX termios，按行读取（ASCII 行协议友好）。IMU 通常高频
 *     （100~1000Hz），read_line 超时设 50ms，避免单次读阻塞拖慢采样循环。
 *   - 无串口设备时（沙箱/开发机/CI）serial_open 返回 NULL，自动降级为 dry-run 模式：
 *     按 sample_hz 生成静止状态模拟 IMU（accel_z≈gravity，其余≈0，加小噪声），
 *     走完整 序列化 + 发布 链路，不阻塞流水线其余节点运行。
 *   - parse_imu_line 是硬件适配点：社区用户按自己 IMU 协议改这一个函数即可（见注释）。
 *
 * 话题契约：
 *   输入: 无（从串口硬件读）
 *   输出: sensor/imu (ImuData 二进制序列化, type_id=0x7dc626af, IMUDATA_TYPE_ID)
 *
 * 典型 pipeline.json 配置:
 *   {
 *     "name": "imu_driver",
 *     "library": "libimu_driver_node.so",
 *     "params": {
 *       "serial_port": "/dev/ttyUSB2",
 *       "baud_rate": 115200,
 *       "enable": true,
 *       "sample_hz": 100,
 *       "gravity": 9.80665,
 *       "dry_run": false
 *     }
 *   }
 *
 * 树莓派部署前置（USB-TTL + IMU 模块示例）:
 *   # 插入 USB-TTL+IMU 后出现 /dev/ttyUSB2，权限不足时:
 *   sudo usermod -aG dialout $USER
 *   # 或临时:
 *   sudo chmod 666 /dev/ttyUSB2
 *   # 验证数据（直接 cat 串口，确认输出 "ax,ay,az,gx,gy,gz,temp" 行）:
 *   stty -F /dev/ttyUSB2 115200 raw -echo
 *   cat /dev/ttyUSB2
 *
 * 编译依赖: serial_port.h（项目自带，无需外部库）
 */

#include "node_plugin.h"
#include "adas_msgs_gen.h"
#include "transport.h"
#include "discovery.h"
#include "logger.h"
#include <cjson/cJSON.h>
#include "clock_service.h"
#include "serial_port.h"

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

    /* 配置参数（pipeline.json 注入） */
    char     serial_port[64];   /* "/dev/ttyUSB2" */
    int      baud_rate;         /* 115200 等常见值 */
    int      enabled;           /* 总开关，false=不读串口也不发布 */
    int      sample_hz;         /* 期望采样频率（dry-run 节拍 + discovery 通告用） */
    int      dry_run;           /* true=只发模拟数据（无硬件或调试） */
    double   gravity;           /* 重力加速度(m/s²)，加速度计校准参考 + dry-run 静止 z 轴 */

    /* 统计 */
    uint64_t samples_read;      /* 成功读取并解析的 IMU 样本数 */
    uint64_t samples_failed;    /* 读取/解析失败的样本数 */
    uint64_t imu_published;     /* 发布到 sensor/imu 的帧数 */

    /* 读取线程 */
    pthread_t     thread;
    volatile int  thread_running;
    volatile int  should_stop;
} g;

/* ── 硬件适配点：解析一行 IMU 数据 ───────────────────────────
 *
 * ⚠ 这是硬件适配点。社区用户按自己 IMU 模块的输出协议改本函数即可，无需动其它代码。
 *
 * 默认实现：兼容 ASCII 行协议，假设 IMU 模块每行输出
 *   "ax,ay,az,gx,gy,gz,temp\n"
 * 即 6 个浮点（三轴加速度 + 三轴角速度）+ 1 个温度，逗号分隔，\n 结尾。
 *
 * 单位约定（默认假设输入已是 SI 单位，若不符请在赋值处做转换）：
 *   - 加速度计 ax/ay/az：m/s²。很多模块原始输出 g 或 mg，需 ×gravity 转 m/s²；
 *   - 陀螺仪 gx/gy/gz：rad/s。很多模块输出 °/s，需 ×(M_PI/180.0) 转 rad/s；
 *   - 温度 temp：℃。
 * 例如某 MPU6050 串口模块输出 g 与 °/s，则在赋值前：
 *   out->accel_x = v[0] * (float)g.gravity;        // g → m/s²
 *   out->gyro_x  = v[3] * (float)(M_PI / 180.0);   // °/s → rad/s
 *
 * 常见 IMU 协议迁移提示：
 *   - MPU6050：I2C 器件，本节点串口方式需配串口转接板输出 ASCII，或改 read 为 I2C；
 *   - ICM-42688：SPI/I2C，常经 MCU 转串口 ASCII/二进制；
 *   - Xsens MTi-300：串口二进制协议（MTData2），需把本函数改成二进制帧解析，
 *     并把调用处的 serial_read_line 换成 serial_read 按帧长读取；
 *   - 一些串口 IMU 模块（WIT-Motion/BNO085 串口版）直接吐 ASCII，本实现可直接用。
 *
 * @param line  串口读到的一行（不含末尾 '\n' 也兼容）
 * @param out   解析结果写入
 * @return 0 成功，-1 解析失败（字段不足/格式错误）
 */
static int parse_imu_line(const char* line, ImuData* out) {
    if (!line || !out) return -1;

    const char* p = line;
    while (*p == ' ' || *p == '\t') p++;                 /* 跳过前导空白 */
    if (*p == '\0' || *p == '\r' || *p == '\n') return -1;

    /* 顺序解析 7 个浮点：ax, ay, az, gx, gy, gz, temp */
    float  v[7];
    int    cnt = 0;
    char*  end = NULL;
    for (cnt = 0; cnt < 7; cnt++) {
        v[cnt] = strtof(p, &end);
        if (end == p) return -1;                         /* 当前位置无数值 */
        p = end;
        /* 跳过分隔符（逗号/空白/回车）到下一字段 */
        while (*p == ',' || *p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') p++;
    }

    /* 默认假设输入已是 m/s² 与 rad/s；若模块输出 g/°/s，在此处乘换算系数 */
    out->accel_x = v[0];
    out->accel_y = v[1];
    out->accel_z = v[2];
    out->gyro_x  = v[3];
    out->gyro_y  = v[4];
    out->gyro_z  = v[5];
    out->temperature = v[6];
    return 0;
}

/* ── dry-run：生成静止状态模拟 IMU 数据 ──────────────────────
 *
 * 静止时 z 轴承受重力（accel_z ≈ gravity），其余轴 ≈ 0；叠加小噪声模拟传感器抖动。
 * 走完整 publish 链路，便于在无硬件环境下联调下游 fusion/SLAM。
 */
static void make_synthetic_imu(ImuData* out) {
    if (!out) return;
    /* 噪声幅度：加速度 ±0.01 m/s²，角速度 ±0.001 rad/s，温度 ±0.1 ℃ */
    float na = (float)((rand() % 2001) - 1000) / 100000.0f;   /* ±0.01 */
    float ng = (float)((rand() % 2001) - 1000) / 1000000.0f;  /* ±0.001 */
    out->accel_x = na;
    out->accel_y = na * 0.5f;
    out->accel_z = (float)g.gravity + na;                     /* 静止 z 轴 = 重力 */
    out->gyro_x  = ng;
    out->gyro_y  = ng * 0.5f;
    out->gyro_z  = ng;
    out->temperature = 25.0f + na * 10.0f;
}

/* ── 读取线程：循环 serial_read_line → parse_imu_line → publish ─
 *
 * 真实模式：从串口逐行读 IMU 数据，解析成功则序列化发布到 sensor/imu。
 *           IMU 自身按其输出频率推数，serial_read_line 阻塞等待下一行（超时 50ms），
 *           故真实采样率由硬件决定；sample_hz 主要用于 dry-run 节拍与 discovery 通告。
 * dry-run 模式：无硬件，按 sample_hz 生成模拟 IMU 数据，走完整 publish 链路。
 */
static void* imu_reader_thread(void* arg) {
    (void)arg;
    pthread_setname_np(pthread_self(), "imu_reader");
    char line[256];
    long period_us = 1000000L / (g.sample_hz > 0 ? g.sample_hz : 100);

    while (!g.should_stop) {
        ImuData imu;
        int     ok = 0;

        if (g.dry_run || !g.serial) {
            /* 降级模式：按 sample_hz 生成模拟 IMU，走完整发布链路 */
            usleep((unsigned long)period_us);
            if (g.should_stop) break;
            make_synthetic_imu(&imu);
            ok = 1;
        } else {
            /* 真实模式：从串口读一行 IMU 数据，超时 50ms（IMU 高频不能等太久） */
            int n = serial_read_line(g.serial, line, sizeof(line), 50);
            if (g.should_stop) break;
            if (n < 0) {
                /* 设备错误/关闭 */
                g.samples_failed++;
                LOG_WARN("imu_driver", "serial_read_line error (n=%d), 设备可能断开", n);
                usleep(100000);  /* 避免错误时忙轮询 */
                continue;
            }
            if (n == 0) {
                /* 超时，无数据，正常情况 */
                continue;
            }

            /* 解析 IMU 数据行 */
            if (parse_imu_line(line, &imu) == 0) {
                g.samples_read++;
                ok = 1;
            } else {
                g.samples_failed++;
                LOG_DEBUG("imu_driver", "parse_imu_line failed: %s", line);
                continue;
            }
        }

        if (!ok) continue;

        /* 填充时间戳 + 序列化 + 发布到 sensor/imu */
        imu.timestamp_us = (uint32_t)(clock_now_us() & 0xFFFFFFFFu);
        uint8_t buf[64];
        size_t  len = 0;
        if (ImuData_serialize(&imu, buf, &len) == 0 && len > 0) {
            transport_publish(g.transport, "sensor/imu", buf, (uint32_t)len);
            g.imu_published++;
            /* 周期性日志（IMU 高频，每 200 条打一次，避免刷屏） */
            if (g.imu_published % 200 == 1) {
                LOG_INFO("imu_driver", "IMU #%lu ax=%.3f ay=%.3f az=%.3f "
                         "gx=%.4f gy=%.4f gz=%.4f temp=%.1f (read=%lu fail=%lu)",
                         (unsigned long)g.imu_published,
                         imu.accel_x, imu.accel_y, imu.accel_z,
                         imu.gyro_x, imu.gyro_y, imu.gyro_z, imu.temperature,
                         (unsigned long)g.samples_read,
                         (unsigned long)g.samples_failed);
            }
        }
    }
    return NULL;
}

/* ── NodePlugin 实现 ─────────────────────────────────────── */

static const char* s_inputs[]  = { NULL };
static const char* s_outputs[] = { "sensor/imu", NULL };

static NodePlugin s_plugin;

static int imu_driver_init(MessageBus* bus, Transport* transport,
                           DiscoveryManager* discovery, Scheduler* scheduler,
                           const char* params_json) {
    (void)bus; (void)scheduler;
    memset(&g, 0, sizeof(g));
    g.transport = transport;
    g.discovery = discovery;
    g.serial    = NULL;

    /* 默认参数 */
    snprintf(g.serial_port, sizeof(g.serial_port), "/dev/ttyUSB2");
    g.baud_rate  = 115200;
    g.enabled    = 1;
    g.sample_hz  = 100;
    g.dry_run    = 0;
    g.gravity    = 9.80665;

    /* cJSON 参数解析（替代手写 parse_*，CLAUDE.md 规范唯一合法入口） */
    if (params_json) {
        cJSON* root = cJSON_Parse(params_json);
        if (root) {
            cJSON* j;
            snprintf(g.serial_port, sizeof(g.serial_port), "%s", "/dev/ttyUSB2");
            if ((j = cJSON_GetObjectItem(root, "serial_port")) && cJSON_IsString(j))
                snprintf(g.serial_port, sizeof(g.serial_port), "%s", j->valuestring);
            g.baud_rate  = 115200;
            if ((j = cJSON_GetObjectItem(root, "baud_rate")) && cJSON_IsNumber(j))
                g.baud_rate = j->valueint;
            g.enabled    = 1;
            if ((j = cJSON_GetObjectItem(root, "enable")) && cJSON_IsNumber(j))
                g.enabled = j->valueint;
            g.sample_hz  = 100;
            if ((j = cJSON_GetObjectItem(root, "sample_hz")) && cJSON_IsNumber(j))
                g.sample_hz = j->valueint;
            g.dry_run    = 0;
            if ((j = cJSON_GetObjectItem(root, "dry_run")) && cJSON_IsNumber(j))
                g.dry_run = j->valueint;
            g.gravity    = 9.80665;
            if ((j = cJSON_GetObjectItem(root, "gravity")) && cJSON_IsNumber(j))
                g.gravity = j->valuedouble;
            cJSON_Delete(root);
        }
    }

    /* dry-run 噪声种子 */
    srand((unsigned)time(NULL));

    if (!g.enabled) {
        LOG_INFO("imu_driver", "disabled by config (enable=0), will not read serial");
        return 0;
    }

    /* 打开串口（失败则自动降级为 dry_run，不阻塞） */
    if (!g.dry_run) {
        g.serial = serial_open(g.serial_port, g.baud_rate);
        if (!g.serial) {
            LOG_WARN("imu_driver", "serial_open('%s', %d) failed, falling back to dry-run "
                     "(真实硬件部署时检查: ls /dev/ttyUSB* && sudo chmod 666 %s)",
                     g.serial_port, g.baud_rate, g.serial_port);
            g.dry_run = 1;
        }
    }

    /* 向 discovery 通告本节点发布 sensor/imu (ImuData, type_id=0x7dc626af, IMUDATA_TYPE_ID) */
    discovery_advertise(discovery, "sensor/imu", 0x7dc626afu, CAP_PUBLISHER, (double)g.sample_hz);

    LOG_INFO("imu_driver", "initialized: dev=%s baud=%d hz=%d gravity=%.4f %s",
             g.serial_port, g.baud_rate, g.sample_hz, g.gravity,
             g.dry_run ? "[DRY-RUN]" : "[LIVE-SERIAL]");

    if (g.dry_run) {
        LOG_INFO("imu_driver", "DRY-RUN 模式：发布模拟 IMU 数据 (静止: accel_z≈%.4f, 其余≈0) "
                 "(无 %s 设备或 dry_run=true)。树莓派部署: "
                 "插入 USB-TTL+IMU 模块后 ls /dev/ttyUSB*，权限不足加 dialout 组",
                 g.gravity, g.serial_port);
    }
    return 0;
}

static int imu_driver_start(void) {
    if (!g.enabled) return 0;
    g.thread_running = 1;
    g.should_stop = 0;
    if (pthread_create(&g.thread, NULL, imu_reader_thread, NULL) != 0) {
        LOG_WARN("imu_driver", "reader thread create failed");
        return -1;
    }
    LOG_INFO("imu_driver", "started (%s, %dHz target)",
             g.dry_run ? "dry-run" : "live serial", g.sample_hz);
    node_announce_self(g.transport, &s_plugin);
    return 0;
}

static void imu_driver_stop(void) {
    g.should_stop = 1;
    g.thread_running = 0;
}

static void imu_driver_cleanup(void) {
    g.thread_running = 0;
    g.should_stop = 1;
    if (g.thread) { pthread_join(g.thread, NULL); g.thread = 0; }
    if (g.serial) { serial_close(g.serial); g.serial = NULL; }
    LOG_INFO("imu_driver", "cleanup: read=%lu failed=%lu published=%lu",
             (unsigned long)g.samples_read,
             (unsigned long)g.samples_failed,
             (unsigned long)g.imu_published);
}

static int imu_driver_health(void) {
    /* 健康判定：解析失败数明显多于成功数且失败超过阈值，视为异常。
     * IMU 高频（百 Hz 级），失败阈值比 GPS 放大（1000），避免短时抖动误报。 */
    if (!g.enabled) return 0;
    if (g.samples_failed > g.samples_read && g.samples_failed > 1000) return -1;
    return 0;
}

static NodePlugin s_plugin = {
    .api_version   = NODE_PLUGIN_API_VERSION,
    .name          = "imu_driver",
    .version       = "1.0.0",
    .description   = "IMU serial driver (6-axis IMU → ImuData → sensor/imu)",
    .input_topics  = s_inputs,
    .output_topics = s_outputs,
    .init          = imu_driver_init,
    .start         = imu_driver_start,
    .stop          = imu_driver_stop,
    .cleanup       = imu_driver_cleanup,
    .health        = imu_driver_health,
};

NodePlugin* node_get_plugin(void) { return &s_plugin; }
