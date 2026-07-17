/**
 * actuator_pwm_node.c — RC 小车 PWM 执行器节点 (control/cmd → PCA9685 / GPIO PWM)
 *
 * 为 RC 小车（舵机 + 电调）提供执行器输出，替代 actuator_node 的 SocketCAN 方案：
 *
 *   safety_control → control/cmd (ControlCmd) → [本节点] → PWM → 舵机/ESC
 *
 * ── 与 actuator_node 的关系 ──
 *   actuator_node 用 SocketCAN 输出 CAN 帧，适合真车（ESC 带 CAN 接口）。
 *   RC 小车用舵机（PWM 信号）+ 电调（PWM 信号），不接 CAN，用 PCA9685
 *   I2C-PWM 扩展板（16 路 PWM，¥10）或树莓派 GPIO 硬件 PWM 输出。
 *   pipeline_car.json 里二选一：
 *     - 真车 ESC: actuator_node (libactuator_node.so)
 *     - RC 小车 : actuator_pwm_node (libactuator_pwm_node.so)
 *
 * ── 硬件方案 ──
 *   方案 A: PCA9685 I2C-PWM 扩展板（推荐）
 *     - 16 路 12-bit PWM，I2C 控制地址 0x40-0x7F
 *     - 树莓派 GPIO2(SDA)+GPIO3(SCL) → PCA9685
 *     - 输出通道 0 接电调（ESC），通道 1 接转向舵机
 *     - 优点：16 路独立 PWM，频率/占空比独立可调，不占 GPIO
 *     - 缺点：需 i2c-tools / libi2c
 *
 *   方案 B: 树莓派 GPIO 硬件 PWM（piozero/servo）
 *     - 用 GPIO12/13 (PWM0/PWM1) 硬件 PWM
 *     - 优点：无需额外硬件
 *     - 缺点：只有 2 路 PWM；和音频共用时钟，需 disable audio
 *
 *   方案 C: sysfs 软 PWM（gpiochip）
 *     - 通过 /sys/class/pwm/ 输出
 *     - 优点：跨平台
 *     - 缺点：抖动大，舵机会颤抖
 *
 *   本节点实现方案 A（PCA9685 I2C）为主，方案 B（GPIO）作 fallback。
 *   通过 backend 参数选择："pca9685" / "gpio" / "dry_run"。
 *
 * ── PWM 信号标准（舵机/ESC 通用） ──
 *   频率 50Hz（周期 20ms）
 *   脉宽 1000-2000μs（1-2ms）
 *     1500μs = 中位（舵机居中 / ESC 停转）
 *     1000μs = 全左 / 全倒车
 *     2000μs = 全右 / 全油门
 *   PCA9685 12-bit 分辨率：0-4095 对应 0-20ms
 *     1500μs → 4096 * 1500/20000 ≈ 307
 *     1000μs → 4096 * 1000/20000 ≈ 205
 *     2000μs → 4096 * 2000/20000 ≈ 410
 *
 * ── ControlCmd → PWM 映射 ──
 *   ControlCmd.throttle ∈ [-1, 1]  → ESC:  1500 + throttle*500 μs
 *   ControlCmd.steering ∈ [-1, 1]  → 舵机: 1500 + steering*500 μs
 *   ControlCmd.brake ∈ [0, 1]      → ESC:  1500 - brake*500 μs（反向）
 *   ControlCmd.emergency_stop      → ESC:  1500 μs（停转）
 *
 * ── 降级机制 ──
 *   - dry_run=1: 只日志不发硬件
 *   - I2C 打开失败 / GPIO 不可用: 自动降级 dry_run + LOG_WARN
 *
 * 话题契约：
 *   输入: control/cmd (ControlCmd 二进制，safety_control_node 限幅后发布)
 *   无输出 topic（输出是物理 PWM 信号）
 *
 * 典型 pipeline_car.json 配置:
 *   {
 *     "name": "actuator",
 *     "library": "libactuator_pwm_node.so",
 *     "subscribe": ["control/cmd"],
 *     "params": {
 *       "backend": "pca9685",
 *       "i2c_bus": 1,
 *       "i2c_addr": 64,
 *       "esc_channel": 0,
 *       "steer_channel": 1,
 *       "pwm_freq_hz": 50,
 *       "throttle_scale": 500,
 *       "steering_scale": 500,
 *       "enable": true,
 *       "dry_run": false
 *     }
 *   }
 */

#include "node_plugin.h"
#include "adas_msgs_gen.h"
#include "transport.h"
#include "discovery.h"
#include "logger.h"
#include <cjson/cJSON.h>

#include <fcntl.h>
#include <math.h>
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>

#ifdef __linux__
#include <linux/i2c-dev.h>
#define HAVE_I2C 1
#else
#define HAVE_I2C 0
#endif

/* PCA9685 寄存器 */
#define PCA9685_MODE1       0x00
#define PCA9685_PRESCALE    0xFE
#define PCA9685_LED0_ON_L   0x06
#define PCA9685_LED_OFF_L(ch) (PCA9685_LED0_ON_L + 4 * (ch))

/* PWM 频率/分辨率常量 */
#define PWM_FREQ_HZ_DEFAULT  50
#define PCA9685_OSC_HZ       25000000ULL   /* PCA9685 内部振荡器 25MHz */
#define PCA9685_RESOLUTION   4096          /* 12-bit */
#define PWM_CENTER_US        1500          /* 中位脉宽 μs */
#define PWM_RANGE_US         500           /* ±500μs 对应 ±1.0 控制 */

#define BACKEND_DRY_RUN  0
#define BACKEND_PCA9685 1
#define BACKEND_GPIO    2

/* ── 节点状态 ─────────────────────────────────────────────── */

static struct {
    Transport*        transport;
    DiscoveryManager* discovery;

    /* 配置 */
    int    enabled;
    int    dry_run;
    int    backend;             /* BACKEND_* 枚举 */
    int    i2c_bus;             /* I2C 总线号（/dev/i2c-N） */
    int    i2c_addr;            /* PCA9685 I2C 地址（0x40-0x7F，十进制 64-127） */
    int    esc_channel;         /* PCA9685 输出通道号（接 ESC） */
    int    steer_channel;       /* PCA9685 输出通道号（接舵机） */
    int    pwm_freq_hz;         /* PWM 频率，默认 50Hz */
    double throttle_scale;      /* throttle → μs 比例，默认 500 */
    double steering_scale;      /* steering → μs 比例，默认 500 */
    int    gpio_esc_pin;        /* GPIO 模式的 ESC 引脚 */
    int    gpio_steer_pin;      /* GPIO 模式的舵机引脚 */

    /* PCA9685 句柄 */
    int    i2c_fd;
    int    pca9685_inited;

    /* 统计 */
    uint64_t cmds_received;
    uint64_t cmds_applied;
    uint64_t e_stop_count;
    uint32_t last_seq;

    /* 安全看门狗：超过 watchdog_timeout_s 未收到 cmd → 自动停转 */
    time_t   last_cmd_time;
    int      watchdog_timeout_s;
    pthread_t watchdog_thread;
    volatile int watchdog_running;
    volatile int should_stop;
} g;

/* ── PCA9685 操作 ─────────────────────────────────────────── */

#if HAVE_I2C

static int pca9685_write_reg(int fd, int addr, uint8_t reg, uint8_t val) {
    if (ioctl(fd, I2C_SLAVE, addr) < 0) return -1;
    uint8_t buf[2] = { reg, val };
    if (write(fd, buf, 2) != 2) return -1;
    return 0;
}

static int pca9685_read_reg(int fd, int addr, uint8_t reg, uint8_t* val) {
    if (ioctl(fd, I2C_SLAVE, addr) < 0) return -1;
    if (write(fd, &reg, 1) != 1) return -1;
    if (read(fd, val, 1) != 1) return -1;
    return 0;
}

/* 设置 PWM 频率（PCA9685 PRESCALE 寄存器） */
static int pca9685_set_freq(int fd, int addr, int freq_hz) {
    /* prescale = round(25MHz / (4096 * freq)) - 1 */
    float prescaleval = (float)PCA9685_OSC_HZ / (float)(PCA9685_RESOLUTION * freq_hz) - 1.0f;
    int prescale = (int)(prescaleval + 0.5f);
    if (prescale < 3) prescale = 3;
    if (prescale > 255) prescale = 255;

    uint8_t oldmode;
    if (pca9685_read_reg(fd, addr, PCA9685_MODE1, &oldmode) != 0) return -1;

    /* 进入 sleep 模式才能写 prescale */
    uint8_t newmode = (oldmode & 0x7F) | 0x10;
    if (pca9685_write_reg(fd, addr, PCA9685_MODE1, newmode) != 0) return -1;
    if (pca9685_write_reg(fd, addr, PCA9685_PRESCALE, (uint8_t)prescale) != 0) return -1;
    if (pca9685_write_reg(fd, addr, PCA9685_MODE1, oldmode) != 0) return -1;
    usleep(5000);
    /* 重启振荡器：MODE1 |= 0x80 */
    if (pca9685_write_reg(fd, addr, PCA9685_MODE1, oldmode | 0x80) != 0) return -1;
    return 0;
}

/* 设置指定通道的 PWM 脉宽（μs） */
static int pca9685_set_pulse_us(int fd, int addr, int channel, int pulse_us) {
    if (channel < 0 || channel > 15) return -1;
    /* tick = pulse_us / period_us * 4096 */
    int period_us = 1000000 / g.pwm_freq_hz;
    int tick = (int)((float)pulse_us / (float)period_us * (float)PCA9685_RESOLUTION + 0.5f);
    if (tick < 0) tick = 0;
    if (tick >= PCA9685_RESOLUTION) tick = PCA9685_RESOLUTION - 1;

    if (ioctl(fd, I2C_SLAVE, addr) < 0) return -1;
    uint8_t buf[5];
    buf[0] = PCA9685_LED_OFF_L(channel);
    buf[1] = 0;            /* ON_L */
    buf[2] = 0;            /* ON_H */
    buf[3] = tick & 0xFF;  /* OFF_L */
    buf[4] = (tick >> 8) & 0x0F;  /* OFF_H */
    if (write(fd, buf, 5) != 5) return -1;
    return 0;
}

#endif /* HAVE_I2C */

/* ── GPIO sysfs PWM 操作（fallback） ────────────────────────
 * 用 /sys/class/pwm/pwmchipN/ 输出。需要 dtoverlay 配置。
 * 简化：仅记录目标脉宽，实际硬件 PWM 由用户预先 dtoverlay 配好。
 */
static int gpio_set_pulse_us(int channel_unused, int pin, int pulse_us) {
    (void)channel_unused;
    char path[128];
    snprintf(path, sizeof(path), "/sys/class/pwm/pwmchip0/pwm%d/duty_cycle", pin);
    int fd = open(path, O_WRONLY);
    if (fd < 0) return -1;
    /* duty_cycle 单位 ns */
    char val[16];
    snprintf(val, sizeof(val), "%d", pulse_us * 1000);
    ssize_t n = write(fd, val, strlen(val));
    close(fd);
    return (n > 0) ? 0 : -1;
}

/* ── 通用 PWM 输出接口 ──────────────────────────────────────
 * @param channel  PCA9685 通道号 或 GPIO pin
 * @param pulse_us 目标脉宽（μs）
 * @return 0 成功, -1 失败
 */
static int pwm_set_pulse(int channel, int pin_fallback, int pulse_us) {
    /* 钳位脉宽到安全范围 [1000, 2000]μs */
    if (pulse_us < 1000) pulse_us = 1000;
    if (pulse_us > 2000) pulse_us = 2000;

    if (g.dry_run || g.backend == BACKEND_DRY_RUN) {
        return 0;  /* dry-run 模式不写硬件 */
    }

    if (g.backend == BACKEND_PCA9685) {
#if HAVE_I2C
        if (g.i2c_fd < 0) return -1;
        return pca9685_set_pulse_us(g.i2c_fd, g.i2c_addr, channel, pulse_us);
#else
        return -1;
#endif
    } else if (g.backend == BACKEND_GPIO) {
        return gpio_set_pulse_us(channel, pin_fallback, pulse_us);
    }
    return -1;
}

/* ── ControlCmd → PWM 转换 + 输出 ─────────────────────────── */
static void apply_control_cmd(const ControlCmd* cmd) {
    int e_stop = cmd->emergency_stop ? 1 : 0;

    /* ESC 脉宽计算: throttle ∈ [-1, 1] → 1500 + throttle*scale
     * brake 单独处理: brake > 0 时反向拉低 ESC（电调刹车） */
    int esc_us;
    if (e_stop) {
        esc_us = PWM_CENTER_US;  /* 紧急停转：中位 */
    } else if (cmd->brake > 0.01f) {
        esc_us = (int)(PWM_CENTER_US - (double)cmd->brake * g.throttle_scale);
    } else {
        esc_us = (int)(PWM_CENTER_US + (double)cmd->throttle * g.throttle_scale);
    }

    /* 舵机脉宽计算: steering 是弧度(±0.22 rad)，先归一化到 [-1,1] 再映射 PWM */
    const double MAX_STEER_RAD = 0.22;
    double steer_norm = (double)cmd->steering / MAX_STEER_RAD;
    if (steer_norm > 1.0) steer_norm = 1.0;
    if (steer_norm < -1.0) steer_norm = -1.0;
    int steer_us = (int)(PWM_CENTER_US + steer_norm * (double)g.steering_scale);

    /* 输出 PWM */
    if (g.backend == BACKEND_PCA9685) {
        pwm_set_pulse(g.esc_channel,   g.gpio_esc_pin,   esc_us);
        pwm_set_pulse(g.steer_channel, g.gpio_steer_pin, steer_us);
    } else if (g.backend == BACKEND_GPIO) {
        pwm_set_pulse(g.esc_channel,   g.gpio_esc_pin,   esc_us);
        pwm_set_pulse(g.steer_channel, g.gpio_steer_pin, steer_us);
    }

    g.cmds_applied++;
    if (e_stop) g.e_stop_count++;

    /* 周期性日志 */
    if (g.cmds_received % 50 == 1 || e_stop) {
        LOG_INFO("actuator_pwm", "seq=%u thr=%.2f brk=%.2f steer=%.2f gear=%d estop=%d "
                 "→ esc=%dμs steer=%dμs [%s]",
                 cmd->seq, cmd->throttle, cmd->brake, cmd->steering,
                 (int)cmd->gear, e_stop, esc_us, steer_us,
                 g.dry_run ? "dry" : (g.backend == BACKEND_PCA9685 ? "i2c" : "gpio"));
    }
}

/* ── 订阅回调：收到 control/cmd ────────────────────────────── */
static void on_control_cmd(const Message* msg, void* user_data) {
    (void)user_data;
    if (!msg || !msg->data || !g.enabled) return;

    /* 二进制反序列化 ControlCmd */
    ControlCmd cmd;
    if (ControlCmd_deserialize(&cmd, (const uint8_t*)msg->data, msg->data_size) != 0) {
        LOG_WARN("actuator_pwm", "ControlCmd deserialize failed (size=%u)", msg->data_size);
        return;
    }

    g.cmds_received++;
    g.last_seq = cmd.seq;
    g.last_cmd_time = time(NULL);

    apply_control_cmd(&cmd);
}

/* ── 安全看门狗线程 ─────────────────────────────────────────
 * 超过 watchdog_timeout_s 未收到 cmd → 强制 ESC 中位（停转）。
 * 防止 control_node 崩溃时小车失控狂奔。
 */
static void* watchdog_thread(void* arg) {
    (void)arg;
    pthread_setname_np(pthread_self(), "act_pwm_wd");
    while (g.watchdog_running && !g.should_stop) {
        sleep(1);
        if (g.should_stop) break;
        time_t now = time(NULL);
        if (g.last_cmd_time > 0 && (now - g.last_cmd_time) > g.watchdog_timeout_s) {
            /* 超时：ESC 强制中位 */
            int esc_us = PWM_CENTER_US;
            int steer_us = PWM_CENTER_US;
            if (g.backend == BACKEND_PCA9685) {
                pwm_set_pulse(g.esc_channel,   g.gpio_esc_pin,   esc_us);
                pwm_set_pulse(g.steer_channel, g.gpio_steer_pin, steer_us);
            } else if (g.backend == BACKEND_GPIO) {
                pwm_set_pulse(g.esc_channel,   g.gpio_esc_pin,   esc_us);
                pwm_set_pulse(g.steer_channel, g.gpio_steer_pin, steer_us);
            }
            LOG_WARN("actuator_pwm", "WATCHDOG: %ds 无 cmd，强制 ESC 中位", g.watchdog_timeout_s);
            g.last_cmd_time = now;  /* 避免每秒重复告警 */
        }
    }
    return NULL;
}

/* ── NodePlugin 实现 ─────────────────────────────────────── */

static const char* s_inputs[]  = { "control/cmd", NULL };
static const char* s_outputs[] = { NULL };

static NodePlugin s_plugin;

static int actuator_pwm_init(MessageBus* bus, Transport* transport,
                              DiscoveryManager* discovery, Scheduler* scheduler,
                              const char* params_json) {
    (void)bus; (void)scheduler;
    memset(&g, 0, sizeof(g));
    g.transport = transport;
    g.discovery = discovery;

    /* 默认参数 */
    g.enabled           = 1;
    g.dry_run           = 0;
    g.backend           = BACKEND_PCA9685;
    g.i2c_bus           = 1;
    g.i2c_addr          = 0x40;     /* 64 */
    g.esc_channel       = 0;
    g.steer_channel     = 1;
    g.pwm_freq_hz       = PWM_FREQ_HZ_DEFAULT;
    g.throttle_scale    = PWM_RANGE_US;
    g.steering_scale    = PWM_RANGE_US;
    g.gpio_esc_pin      = 12;       /* PWM0 */
    g.gpio_steer_pin    = 13;       /* PWM1 */
    g.watchdog_timeout_s = 3;
    g.i2c_fd            = -1;

    cJSON* root = cJSON_Parse(params_json);
    if (root) {
        cJSON* j;

        g.enabled           = 1;
        if ((j = cJSON_GetObjectItem(root, "enable")) && cJSON_IsNumber(j)) g.enabled = j->valueint;
        g.dry_run           = 0;
        if ((j = cJSON_GetObjectItem(root, "dry_run")) && cJSON_IsNumber(j)) g.dry_run = j->valueint;
        g.i2c_bus           = 1;
        if ((j = cJSON_GetObjectItem(root, "i2c_bus")) && cJSON_IsNumber(j)) g.i2c_bus = j->valueint;
        g.i2c_addr          = 0x40;
        if ((j = cJSON_GetObjectItem(root, "i2c_addr")) && cJSON_IsNumber(j)) g.i2c_addr = j->valueint;
        g.esc_channel       = 0;
        if ((j = cJSON_GetObjectItem(root, "esc_channel")) && cJSON_IsNumber(j)) g.esc_channel = j->valueint;
        g.steer_channel     = 1;
        if ((j = cJSON_GetObjectItem(root, "steer_channel")) && cJSON_IsNumber(j)) g.steer_channel = j->valueint;
        g.pwm_freq_hz       = PWM_FREQ_HZ_DEFAULT;
        if ((j = cJSON_GetObjectItem(root, "pwm_freq_hz")) && cJSON_IsNumber(j)) g.pwm_freq_hz = j->valueint;
        g.throttle_scale    = PWM_RANGE_US;
        if ((j = cJSON_GetObjectItem(root, "throttle_scale")) && cJSON_IsNumber(j)) g.throttle_scale = j->valuedouble;
        g.steering_scale    = PWM_RANGE_US;
        if ((j = cJSON_GetObjectItem(root, "steering_scale")) && cJSON_IsNumber(j)) g.steering_scale = j->valuedouble;
        g.gpio_esc_pin      = 12;
        if ((j = cJSON_GetObjectItem(root, "gpio_esc_pin")) && cJSON_IsNumber(j)) g.gpio_esc_pin = j->valueint;
        g.gpio_steer_pin    = 13;
        if ((j = cJSON_GetObjectItem(root, "gpio_steer_pin")) && cJSON_IsNumber(j)) g.gpio_steer_pin = j->valueint;
        g.watchdog_timeout_s = 3;
        if ((j = cJSON_GetObjectItem(root, "watchdog_timeout_s")) && cJSON_IsNumber(j)) g.watchdog_timeout_s = j->valueint;

        /* 解析 backend 字符串 */
        char backend[32] = {0};
        snprintf(backend, sizeof(backend), "%s", "pca9685");
        if ((j = cJSON_GetObjectItem(root, "backend")) && cJSON_IsString(j))
            snprintf(backend, sizeof(backend), "%s", j->valuestring);
        if (strcmp(backend, "gpio") == 0)       g.backend = BACKEND_GPIO;
        else if (strcmp(backend, "dry_run") == 0) g.backend = BACKEND_DRY_RUN;
        else                                    g.backend = BACKEND_PCA9685;

        cJSON_Delete(root);
    }

    if (!g.enabled) {
        LOG_INFO("actuator_pwm", "disabled by config");
        return 0;
    }

    /* 初始化 PCA9685（如果 backend 选了） */
    if (g.backend == BACKEND_PCA9685 && !g.dry_run) {
#if HAVE_I2C
        char i2c_path[32];
        snprintf(i2c_path, sizeof(i2c_path), "/dev/i2c-%d", g.i2c_bus);
        g.i2c_fd = open(i2c_path, O_RDWR);
        if (g.i2c_fd < 0) {
            LOG_WARN("actuator_pwm", "I2C 打开失败 %s: %m → 自动降级 dry_run", i2c_path);
            g.dry_run = 1;
        } else {
            /* 软复位 */
            if (pca9685_write_reg(g.i2c_fd, g.i2c_addr, PCA9685_MODE1, 0x00) != 0) {
                LOG_WARN("actuator_pwm", "PCA9685 reset 失败 (addr=0x%02x) → 降级 dry_run", g.i2c_addr);
                close(g.i2c_fd); g.i2c_fd = -1;
                g.dry_run = 1;
            } else if (pca9685_set_freq(g.i2c_fd, g.i2c_addr, g.pwm_freq_hz) != 0) {
                LOG_WARN("actuator_pwm", "PCA9685 设频率 %dHz 失败 → 降级 dry_run", g.pwm_freq_hz);
                close(g.i2c_fd); g.i2c_fd = -1;
                g.dry_run = 1;
            } else {
                /* 初始中位输出（防止舵机上电跳到极限位置） */
                pca9685_set_pulse_us(g.i2c_fd, g.i2c_addr, g.esc_channel,   PWM_CENTER_US);
                pca9685_set_pulse_us(g.i2c_fd, g.i2c_addr, g.steer_channel, PWM_CENTER_US);
                g.pca9685_inited = 1;
                LOG_INFO("actuator_pwm", "PCA9685 初始化成功: bus=%d addr=0x%02x freq=%dHz "
                         "esc=ch%d steer=ch%d",
                         g.i2c_bus, g.i2c_addr, g.pwm_freq_hz, g.esc_channel, g.steer_channel);
            }
        }
#else
        LOG_WARN("actuator_pwm", "非 Linux 平台无 I2C → 降级 dry_run");
        g.dry_run = 1;
#endif
    }

    /* 订阅 control/cmd */
    transport_subscribe(transport, "control/cmd", on_control_cmd, NULL);
    discovery_advertise(discovery, "control/cmd", CONTROLCMD_TYPE_ID, CAP_SUBSCRIBER, 0);

    LOG_INFO("actuator_pwm", "initialized: backend=%s dry_run=%d thr_scale=%.0f str_scale=%.0f "
             "watchdog=%ds",
             g.backend == BACKEND_PCA9685 ? "pca9685" :
             (g.backend == BACKEND_GPIO ? "gpio" : "dry_run"),
             g.dry_run, g.throttle_scale, g.steering_scale, g.watchdog_timeout_s);
    return 0;
}

static int actuator_pwm_start(void) {
    if (!g.enabled) return 0;
    g.watchdog_running = 1;
    g.should_stop = 0;
    g.last_cmd_time = time(NULL);
    if (pthread_create(&g.watchdog_thread, NULL, watchdog_thread, NULL) != 0) {
        LOG_WARN("actuator_pwm", "watchdog thread create failed");
        g.watchdog_running = 0;
        /* 看门狗失败不阻塞启动，主功能仍可用 */
    }
    LOG_INFO("actuator_pwm", "started");
    node_announce_self(g.transport, &s_plugin);
    return 0;
}

static void actuator_pwm_stop(void) {
    g.should_stop = 1;
    g.watchdog_running = 0;
}

static void actuator_pwm_cleanup(void) {
    g.should_stop = 1;
    g.watchdog_running = 0;
    if (g.watchdog_thread) { pthread_join(g.watchdog_thread, NULL); g.watchdog_thread = 0; }

    /* 停机：ESC 强制中位（防失控） */
    if (g.enabled && !g.dry_run) {
        if (g.backend == BACKEND_PCA9685) {
            pwm_set_pulse(g.esc_channel,   g.gpio_esc_pin,   PWM_CENTER_US);
            pwm_set_pulse(g.steer_channel, g.gpio_steer_pin, PWM_CENTER_US);
        } else if (g.backend == BACKEND_GPIO) {
            pwm_set_pulse(g.esc_channel,   g.gpio_esc_pin,   PWM_CENTER_US);
            pwm_set_pulse(g.steer_channel, g.gpio_steer_pin, PWM_CENTER_US);
        }
    }

#if HAVE_I2C
    if (g.i2c_fd >= 0) { close(g.i2c_fd); g.i2c_fd = -1; }
#endif

    LOG_INFO("actuator_pwm", "cleanup: cmds=%lu applied=%lu e_stops=%lu",
             (unsigned long)g.cmds_received,
             (unsigned long)g.cmds_applied,
             (unsigned long)g.e_stop_count);
}

static int actuator_pwm_health(void) {
    if (!g.enabled) return 0;
    /* 5 秒未收到 cmd 视为异常（control 链路断） */
    time_t now = time(NULL);
    if (g.cmds_received > 0 && (now - g.last_cmd_time) > 5) return -1;
    return 0;
}

static NodePlugin s_plugin = {
    .api_version   = NODE_PLUGIN_API_VERSION,
    .name          = "actuator_pwm",
    .version       = "1.0.0",
    .description   = "RC car PWM actuator (PCA9685 I2C / GPIO PWM, replaces CAN actuator)",
    .input_topics  = s_inputs,
    .output_topics = s_outputs,
    .init          = actuator_pwm_init,
    .start         = actuator_pwm_start,
    .stop          = actuator_pwm_stop,
    .cleanup       = actuator_pwm_cleanup,
    .health        = actuator_pwm_health,
};

NodePlugin* node_get_plugin(void) { return &s_plugin; }
