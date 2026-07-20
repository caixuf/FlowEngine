/**
 * manual_drive_node.c — 终端 WASD 键盘侧车工具（游戏模式）
 *
 * 用键盘直接驾驶 ego 车辆。把按键映射成 ControlCmd 发布到 control/cmd，
 * flowsim_node 订阅它驱动 ego，无感知地替代 control_node + safety_control_node。
 *
 * 设计要点：
 *   - init() 用 termios 把 stdin 设为 raw + 非阻塞模式，保存原状态供 shutdown 恢复
 *   - execute() 主循环 20Hz 调用 manual_tick()，每帧 read(STDIN_FILENO) 非阻塞读键盘
 *   - 按键映射（连续按住式，靠 raw 终端的按键重复事件维持"按住"语义）：
 *       w/W  throttle 斜坡升到 1.0，松开衰减到 0
 *       s/S  brake    斜坡升到 1.0，松开衰减到 0
 *       a/A  steering 斜坡到 -0.25 rad，松开自动回中
 *       d/D  steering 斜坡到 +0.25 rad，松开自动回中
 *       空格  手刹：立即 brake=1.0, throttle=0
 *       q/Q  退出节点（返回 -1，shutdown 恢复终端）
 *   - 每帧用 ControlCmd_serialize 序列化 + transport_publish("control/cmd")
 *   - 每帧打印 [manual] 状态行
 *
 * 话题契约：
 *   无输入 topic
 *   输出: control/cmd (ControlCmd 二进制)
 *
 * 典型 pipeline_manual.json 配置:
 *   {
 *     "name": "manual_drive",
 *     "library_path": "modules/adas_nodes/manual_drive_node.so",
 *     "auto_start": true,
 *     "subscribe": [],
 *     "publish": [{"topic": "control/cmd", "type": "ControlCmd"}]
 *   }
 *
 * 编译依赖: adas_msgs_gen.h (ControlCmd_serialize)，随构建生成。
 */

#include "node_plugin.h"
#include "adas_msgs_gen.h"
#include "transport.h"
#include "discovery.h"
#include "logger.h"

#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

/* ── 常量 ─────────────────────────────────────────────────── */

#define MANUAL_TICK_HZ        20
#define MANUAL_TICK_US        (1000000u / MANUAL_TICK_HZ)
#define MANUAL_ACCEL_RATE     1.5f   /* throttle 升/衰减率 (1/s) */
#define MANUAL_BRAKE_RATE     3.5f   /* brake    升/衰减率 (1/s) */
#define MANUAL_STEER_RATE     2.0f   /* steering 升/回中率 (rad/s) */
#define MANUAL_STEER_MAX      0.25f  /* 物理层钳位 (rad) */
#define MANUAL_DT             (1.0f / (float)MANUAL_TICK_HZ)
#define CONTROLCMD_TYPE_ID    0x2D95C6D2u

/* ── 节点状态 ─────────────────────────────────────────────── */

static struct {
    Transport*        transport;
    DiscoveryManager* discovery;
    Scheduler*        scheduler;

    /* 托管模式：嵌入 TaskBase，由 node_start_managed 派生线程跑 manual_execute */
    TaskBase          taskbase;

    /* 控制状态（持续按住式斜坡） */
    float    throttle;     /* [0,1] */
    float    brake;        /* [0,1] */
    float    steering;     /* [-0.25, 0.25] rad */
    uint32_t seq;

    /* 当前 tick 按键状态（按住=1，松开=0） */
    int key_w, key_s, key_a, key_d;
} g;

/* 终端原状态保存（shutdown 时恢复） */
static struct termios g_orig_termios;
static int            g_termios_saved = 0;

/* ── 终端 raw 模式 ──────────────────────────────────────────── */

static int terminal_set_raw(void) {
    if (tcgetattr(STDIN_FILENO, &g_orig_termios) != 0) {
        LOG_WARN("manual_drive", "tcgetattr failed: %s", strerror(errno));
        return -1;
    }
    g_termios_saved = 1;

    struct termios raw = g_orig_termios;
    /* 关闭规范模式 + 回显 + 信号生成（防止 Ctrl-C 等误触 SIGINT 退出 pipeline） */
    raw.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHOK | ECHONL | ISIG);
    raw.c_iflag &= ~(IXON | IXOFF | ICRNL | INLCR | IGNCR);
    raw.c_oflag &= ~OPOST;
    raw.c_cc[VMIN]  = 0;   /* 非阻塞：无数据立即返回 0 */
    raw.c_cc[VTIME] = 0;

    if (tcsetattr(STDIN_FILENO, TCSAFLUSH, &raw) != 0) {
        LOG_WARN("manual_drive", "tcsetattr(raw) failed: %s", strerror(errno));
        return -1;
    }
    return 0;
}

static void terminal_restore(void) {
    if (!g_termios_saved) return;
    if (tcsetattr(STDIN_FILENO, TCSAFLUSH, &g_orig_termios) != 0) {
        fprintf(stderr, "[manual] WARNING: failed to restore terminal\n");
    }
    g_termios_saved = 0;
}

/* ── 单帧 tick：读键盘 + 更新状态 + 发布 ControlCmd ─────────── */
/* 返回 -1 表示请求节点退出（'q' 按下），0 表示继续。 */
static int manual_tick(void) {
    /* 每帧重置按键状态，靠本帧 read 命中重新置位。
     * raw 终端下按住键会产生重复输入事件，松开后停止，故此模式可表达"按住"。 */
    g.key_w = 0; g.key_s = 0; g.key_a = 0; g.key_d = 0;

    /* 非阻塞读所有可用按键事件 */
    uint8_t buf[64];
    ssize_t n;
    while ((n = read(STDIN_FILENO, buf, sizeof(buf))) > 0) {
        for (ssize_t i = 0; i < n; i++) {
            switch (buf[i]) {
                case 'w': case 'W': g.key_w = 1; break;
                case 's': case 'S': g.key_s = 1; break;
                case 'a': case 'A': g.key_a = 1; break;
                case 'd': case 'D': g.key_d = 1; break;
                case ' ':  /* 手刹：立即满刹 + 零油门 */
                    g.brake    = 1.0f;
                    g.throttle = 0.0f;
                    break;
                case 'q': case 'Q':
                    LOG_INFO("manual_drive", "quit key pressed, exiting");
                    return -1;
                default: break;
            }
        }
    }

    /* 油门斜坡：按住升到 1.0，松开衰减到 0 */
    if (g.key_w) {
        g.throttle += MANUAL_ACCEL_RATE * MANUAL_DT;
        if (g.throttle > 1.0f) g.throttle = 1.0f;
    } else {
        g.throttle -= MANUAL_ACCEL_RATE * MANUAL_DT;
        if (g.throttle < 0.0f) g.throttle = 0.0f;
    }

    /* 刹车斜坡：按住升到 1.0，松开衰减到 0（手刹 brake=1 也按此衰减回 0） */
    if (g.key_s) {
        g.brake += MANUAL_BRAKE_RATE * MANUAL_DT;
        if (g.brake > 1.0f) g.brake = 1.0f;
    } else {
        g.brake -= MANUAL_BRAKE_RATE * MANUAL_DT;
        if (g.brake < 0.0f) g.brake = 0.0f;
    }

    /* 转向斜坡：a → -0.25，d → +0.25，松开（或同时按 a+d）回中 */
    if (g.key_a && !g.key_d) {
        g.steering -= MANUAL_STEER_RATE * MANUAL_DT;
        if (g.steering < -MANUAL_STEER_MAX) g.steering = -MANUAL_STEER_MAX;
    } else if (g.key_d && !g.key_a) {
        g.steering += MANUAL_STEER_RATE * MANUAL_DT;
        if (g.steering > MANUAL_STEER_MAX) g.steering = MANUAL_STEER_MAX;
    } else {
        if (g.steering > 0.0f) {
            g.steering -= MANUAL_STEER_RATE * MANUAL_DT;
            if (g.steering < 0.0f) g.steering = 0.0f;
        } else if (g.steering < 0.0f) {
            g.steering += MANUAL_STEER_RATE * MANUAL_DT;
            if (g.steering > 0.0f) g.steering = 0.0f;
        }
    }

    /* 钳位保险（防数值漂移） */
    if (g.throttle < 0.0f) g.throttle = 0.0f;
    if (g.throttle > 1.0f) g.throttle = 1.0f;
    if (g.brake    < 0.0f) g.brake    = 0.0f;
    if (g.brake    > 1.0f) g.brake    = 1.0f;
    if (g.steering < -MANUAL_STEER_MAX) g.steering = -MANUAL_STEER_MAX;
    if (g.steering >  MANUAL_STEER_MAX) g.steering =  MANUAL_STEER_MAX;

    /* 序列化 + 发布 ControlCmd */
    ControlCmd bin;
    memset(&bin, 0, sizeof(bin));
    bin.seq            = g.seq++;
    bin.throttle       = g.throttle;
    bin.brake          = g.brake;
    bin.steering       = g.steering;
    bin.gear           = GEAR_DRIVE;
    bin.emergency_stop = (g.brake > 0.95f) ? true : false;

    uint8_t sbuf[32];
    size_t  slen = sizeof(sbuf);
    ControlCmd_serialize(&bin, sbuf, &slen);
    transport_publish(g.transport, "control/cmd", sbuf, (uint32_t)slen);

    fprintf(stderr, "[manual] throttle=%.2f brake=%.2f steer=%.3f\n",
            g.throttle, g.brake, g.steering);
    return 0;
}

/* ── 托管模式主循环（execute） ─────────────────────────────── */

static int manual_execute(TaskBase* task) {
    pthread_setname_np(pthread_self(), "manual_drv");
    while (!task->should_stop) {
        usleep(MANUAL_TICK_US);
        if (task->should_stop) break;
        if (manual_tick() != 0) break;  /* 'q' 触发退出 */
    }
    return 0;
}

/* 托管模式虚函数表：仅实现 execute()（完整主循环）。 */
static const TaskInterface manual_vtable = {
    .execute = manual_execute,
};

/* ── NodePlugin 生命周期 ──────────────────────────────────── */

static const char* s_inputs[]  = { NULL };
static const char* s_outputs[] = { "control/cmd", NULL };
static NodePlugin  s_plugin;

static int manual_init(MessageBus* bus, Transport* transport,
                       DiscoveryManager* discovery, Scheduler* scheduler,
                       const char* params_json) {
    (void)bus; (void)params_json;

    memset(&g, 0, sizeof(g));
    g.transport = transport;
    g.discovery = discovery;
    g.scheduler = scheduler;

    /* 把 stdin 设为 raw + 非阻塞模式（保存原状态供 cleanup 恢复） */
    if (terminal_set_raw() != 0) {
        LOG_WARN("manual_drive", "failed to set raw terminal; "
                 "keyboard input may not work in this environment");
    }

    /* 注册为 control/cmd 发布者 */
    transport_advertise(transport, "control/cmd", CONTROLCMD_TYPE_ID);
    discovery_advertise(discovery, "control/cmd", CONTROLCMD_TYPE_ID,
                        CAP_PUBLISHER, MANUAL_TICK_HZ);

    /* 托管模式：初始化嵌入的 TaskBase 并挂上 vtable */
    TaskConfig cfg;
    memset(&cfg, 0, sizeof(cfg));
    snprintf(cfg.name, sizeof(cfg.name), "manual_drive");
    cfg.priority         = TASK_PRIORITY_NORMAL;
    cfg.max_frequency_hz = (double)MANUAL_TICK_HZ;
    cfg.enable_stats     = true;
    if (task_base_init(&g.taskbase, &manual_vtable, &cfg) != 0) {
        LOG_WARN("manual_drive", "task_base_init failed");
        return -1;
    }

    LOG_INFO("manual_drive",
             "initialized: WASD drive @ %dHz (w=throttle s=brake "
             "a/d=steer space=handbrake q=quit)", MANUAL_TICK_HZ);
    return 0;
}

static int manual_start(void) {
    /* 托管模式：node_start_managed 注册 taskbase 到调度器并派生工作线程跑
     * manual_execute()。 */
    int rc = node_start_managed(&s_plugin, g.scheduler);
    if (rc != 0) {
        LOG_WARN("manual_drive", "node_start_managed failed: %d", rc);
        return rc;
    }
    LOG_INFO("manual_drive", "started (managed, %dHz tick)", MANUAL_TICK_HZ);
    node_announce_self(g.transport, &s_plugin);
    return 0;
}

static void manual_stop(void) {
    /* task_stop 置 should_stop=true 并 join 工作线程（manual_execute 随即退出）。
     * launcher 保证 stop() 在 cleanup() 前调用，故此处阻塞 join 是安全的。 */
    task_stop(&g.taskbase);
}

static void manual_cleanup(void) {
    /* stop() 已 join 线程；此处再 task_stop 一次作幂等保险，随后释放 TaskBase 资源。 */
    task_stop(&g.taskbase);
    task_base_destroy(&g.taskbase);
    /* 必须无条件恢复原 termios 状态，否则用户终端会停留在 raw 模式 */
    terminal_restore();
    LOG_INFO("manual_drive", "cleanup done");
}

static int manual_health(void) {
    return 0;
}

static NodePlugin s_plugin = {
    .api_version   = NODE_PLUGIN_API_VERSION,
    .name          = "manual_drive",
    .version       = "1.0.0",
    .description   = "Terminal WASD keyboard driver (publishes control/cmd)",
    .input_topics  = s_inputs,
    .output_topics = s_outputs,
    .init          = manual_init,
    .start         = manual_start,
    .stop          = manual_stop,
    .cleanup       = manual_cleanup,
    .health        = manual_health,
    .taskbase      = &g.taskbase,   /* v2: 托管模式钩子 */
};

NodePlugin* node_get_plugin(void) { return &s_plugin; }
