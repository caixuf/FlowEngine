/**
 * inference_node.c — 车端模型推理节点 (车端学习闭环 · Stage 2)
 *
 * 订阅 fusion/localization → 用内置 tiny-MLP 推理 → 影子模式发布 inference/trajectory。
 * v2 升级: 支持 16 维特征（含障碍物 + 控制状态），支持直接控制模式。
 *
 * 设计要点:
 *   1. 影子模式 (shadow mode): 本节点与 planning_node 并行运行，但**不**接入
 *      control 链路。它只发布 inference/trajectory 供监控/对比，安全永远由
 *      planning → control → safety_control 兜底。这样即使模型是随机权重/未训练，
 *      也不会影响真实控制。
 *   2. 零重型依赖: 推理内核是 tiny_mlp.h（纯 C，单隐层 MLP）。当 model_path 指向的
 *      权重文件存在时加载训练好的模型；否则回退到一个可解释的启发式策略，保证
 *      "模型跑进 pipeline" 这条链路始终可运行。
 *   3. 可替换: 后续把 run_inference() 内部替换为 ONNX Runtime / TensorRT 调用即可，
 *      数据契约 (输入特征 / 输出语义) 保持不变。
 *   4. 三种控制模式:
 *      - shadow:   只发布 inference/trajectory 供监控对比（默认安全模式）
 *      - plan_assist: 影子轨迹附带额外字段，planning_node 可选择性消费
 *      - direct_ctrl: 额外发布 inference/raw_cmd，安全由 safety_control 兜底
 *
 * 数据契约 (v2，16 维特征):
 *   输入 x[16] = {
 *       ego_v, ego_y, ego_heading, ego_yaw_rate,        // 0-3: 自车状态
 *       front0_x, front0_y, front0_vx,                   // 4-6: 前车 0
 *       front0_type(0/1/2), front0_confidence,            // 7-8
 *       front1_x, front1_y, front1_vx,                   // 9-11: 前车 1
 *       front1_type(0/1/2), front1_confidence,            // 12-13
 *       ctrl_brake, ctrl_emergency_stop(0/1)              // 14-15: 控制状态
 *   }
 *   输出 y[1..4] = { target_speed, lateral_d, throttle, steer }
 *     影子模式只用前 2 维（target_speed + lateral_d）；
 *     direct_ctrl 模式需要 4 维输出（含 throttle/steer）。
 */

#include "node_plugin.h"
#include "state_machine.h"
#include "adas_msgs_gen.h"
#include "logger.h"
#include "tiny_mlp.h"

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <pthread.h>
#include <unistd.h>

/* ── 控制模式枚举 ──────────────────────────────────────────────── */

enum CtrlMode {
    CTRL_MODE_SHADOW      = 0,  /* 只发 inference/trajectory（默认安全）*/
    CTRL_MODE_PLAN_ASSIST = 1,  /* 影子轨迹附带额外辅助字段 */
    CTRL_MODE_DIRECT      = 2,  /* 额外发 inference/raw_cmd 直接控制 */
};

/* ── 障碍物类映射 ────────────────────────────────────────────── */

#define OBJ_TYPE_UNKNOWN   0
#define OBJ_TYPE_VEHICLE   1
#define OBJ_TYPE_PEDESTRIAN 2

/* ── 时序滑窗 ────────────────────────────────────────────────── */

#define V2_DIM          16    /* 每帧特征维度 */
#define TEMPORAL_WINDOW  5    /* 时序滑窗帧数 (共 5×16 = 80 维) */

/* ── 节点本地状态 ───────────────────────────────────────────── */

static struct {
    Transport*        transport;
    DiscoveryManager* discovery;
    Scheduler*        scheduler;

    pthread_t    thread;
    volatile int running;
    volatile int should_stop;

    ReflectiveStateMachine sm;

    TinyMLP         model;
    pthread_mutex_t model_mutex;  /* 保护 model 的并发读写 */
    char    model_path[256];
    int     control_mode;  /* enum CtrlMode */

    /* OTA 热重载: model_ota/active 收到 "reload" 信号时置 1 */
    volatile int reload_flag;

    /* v1 特征: 最新 ego 状态（来自 fusion/localization） */
    double ego_x, ego_y, ego_v, ego_heading, ego_yaw_rate;
    volatile int has_fusion;

    /* v2 特征: 前向障碍物（来自 perception/obstacles） */
    double front0_x, front0_y, front0_vx;
    double front0_type, front0_confidence;
    double front1_x, front1_y, front1_vx;
    double front1_type, front1_confidence;
    volatile int has_obstacles;

    /* v2 特征: 控制状态（来自 control/cmd） */
    double ctrl_brake;
    int    ctrl_emergency_stop;
    volatile int has_control;

    /* 影子对比: planning 发布的 target_speed（若存在） */
    double planning_target_speed;
    volatile int has_planning;

    /* 配置 */
    double cfg_max_speed;
    double cfg_frequency_hz;

    /* 时序滑窗缓冲: 保存最近 TEMPORAL_WINDOW 帧的 16 维特征 */
    float  frame_buf[TEMPORAL_WINDOW][V2_DIM];
    int    frame_head;  /* 当前写入位置（环形） */
    int    frame_count; /* 已写入帧数 */

    int infer_count;
    int reload_count;
} g;

/* ── 订阅回调 ────────────────────────────────────────────────── */

static void on_fusion(const Message* msg, void* user_data) {
    (void)user_data;
    if (!msg || !msg->data) return;

    Localization loc;
    if (Localization_deserialize(&loc, (const uint8_t*)msg->data, msg->data_size) == 0) {
        g.ego_x        = loc.x;
        g.ego_y        = loc.y;
        g.ego_v        = loc.v;
        g.ego_heading  = loc.heading;
        g.ego_yaw_rate = loc.yaw_rate;
        g.has_fusion   = 1;
        return;
    }

    /* Fallback: 文本 JSON */
    const char* d = (const char*)msg->data;
    const char* p;
    if ((p = strstr(d, "\"x\":")))       sscanf(p + 4, "%lf", &g.ego_x);
    if ((p = strstr(d, "\"y\":")))       sscanf(p + 4, "%lf", &g.ego_y);
    if ((p = strstr(d, "\"v\":")))       sscanf(p + 4, "%lf", &g.ego_v);
    if ((p = strstr(d, "\"heading\":"))) sscanf(p + 10, "%lf", &g.ego_heading);
    g.has_fusion = 1;
}

static void on_planning(const Message* msg, void* user_data) {
    (void)user_data;
    if (!msg || !msg->data) return;
    const char* d = (const char*)msg->data;
    const char* p;
    if ((p = strstr(d, "\"target_speed\":")))
        sscanf(p + 15, "%lf", &g.planning_target_speed);
    else if ((p = strstr(d, "speed=")))
        sscanf(p + 6, "%lf", &g.planning_target_speed);
    g.has_planning = 1;
}

/* ── v2: 障碍物订阅回调 ─────────────────────────────────────── */

static void on_obstacles(const Message* msg, void* user_data) {
    (void)user_data;
    if (!msg || !msg->data) return;

    /* 尝试二进制反序列化 */
    {
        ObstacleList list;
        if (ObstacleList_deserialize(&list, (const uint8_t*)msg->data, msg->data_size) == 0) {
            /* 取前 2 个障碍物作为 front0/front1 */
            int n = list.count > 2 ? 2 : list.count;
            int fi = 0;
            for (int i = 0; i < n && fi < 2; i++) {
                /* 跳过后方障碍物 */
                if (list.obstacles[i].x - g.ego_x < 0) continue;
                double type_f = OBJ_TYPE_UNKNOWN;
                if (list.obstacles[i].type == 3) type_f = OBJ_TYPE_VEHICLE;
                else if (list.obstacles[i].type == 4) type_f = OBJ_TYPE_PEDESTRIAN;
                if (fi == 0) {
                    g.front0_x = list.obstacles[i].x;
                    g.front0_y = list.obstacles[i].y;
                    g.front0_vx = list.obstacles[i].vx;
                    g.front0_type = type_f;
                    g.front0_confidence = list.obstacles[i].confidence;
                } else {
                    g.front1_x = list.obstacles[i].x;
                    g.front1_y = list.obstacles[i].y;
                    g.front1_vx = list.obstacles[i].vx;
                    g.front1_type = type_f;
                    g.front1_confidence = list.obstacles[i].confidence;
                }
                fi++;
            }
            /* 未满的 slot 置零 */
            if (fi <= 0) { g.front0_x = 0; g.front0_y = 0; g.front0_vx = 0; g.front0_type = 0; g.front0_confidence = 0; }
            if (fi <= 1) { g.front1_x = 0; g.front1_y = 0; g.front1_vx = 0; g.front1_type = 0; g.front1_confidence = 0; }
            g.has_obstacles = 1;
            return;
        }
    }

    /* Fallback: 文本 JSON 解析 */
    const char* d = (const char*)msg->data;
    g.front0_x = 0; g.front0_y = 0; g.front0_vx = 0; g.front0_type = 0; g.front0_confidence = 0;
    g.front1_x = 0; g.front1_y = 0; g.front1_vx = 0; g.front1_type = 0; g.front1_confidence = 0;
    const char* p;
    for (int i = 0; i < 2; i++) {
        char key[16];
        snprintf(key, sizeof(key), "\"x\":");
        p = strstr(d, key);
        double *ox = (i == 0) ? &g.front0_x : &g.front1_x;
        double *oy = (i == 0) ? &g.front0_y : &g.front1_y;
        if (p) sscanf(p + 3, "%lf", ox);
        snprintf(key, sizeof(key), "\"y\":");
        if ((p = strstr(d, key))) sscanf(p + 3, "%lf", oy);
        /* 跳过后方 */
        if (*ox - g.ego_x < 0) continue;
    }
    g.has_obstacles = 1;
}

/* ── v2: 控制状态订阅回调 ────────────────────────────────────── */

static void on_control_cmd(const Message* msg, void* user_data) {
    (void)user_data;
    if (!msg || !msg->data) return;

    /* 二进制反序列化 */
    {
        ControlCmd cmd;
        if (ControlCmd_deserialize(&cmd, (const uint8_t*)msg->data, msg->data_size) == 0) {
            g.ctrl_brake = cmd.brake;
            g.ctrl_emergency_stop = cmd.emergency_stop ? 1 : 0;
            g.has_control = 1;
            return;
        }
    }

    /* 文本回退 */
    const char* d = (const char*)msg->data;
    const char* p;
    if ((p = strstr(d, "brake="))) sscanf(p + 6, "%lf", &g.ctrl_brake);
    if ((p = strstr(d, "mode="))) {
        g.ctrl_emergency_stop = (strstr(p, "AEB") != NULL
                                 || strstr(p, "BRAKE") != NULL) ? 1 : 0;
    }
    g.has_control = 1;
}

/* ── OTA 热重载回调 ──────────────────────────────────────────── */

/*
 * model_ota_node 在激活新版本后发布 model_ota/active。
 * 收到 "reload" 信号时，设置 reload_flag；inference_thread 在下一个周期
 * 检测到 flag 后加锁重载模型（保证推理线程内原子更新）。
 */
static void on_model_ota_active(const Message* msg, void* user_data) {
    (void)user_data;
    if (!msg || !msg->data) return;
    const char* d = (const char*)msg->data;
    if (strstr(d, "\"reload\"")) {
        g.reload_flag = 1;
    }
}



/* 将当前时刻 ego + obstacles + control 打包成 16 维帧 */
static void build_frame(float frame[V2_DIM]) {
    memset(frame, 0, sizeof(float) * V2_DIM);
    frame[0]  = (float)g.ego_v;
    frame[1]  = (float)g.ego_y;
    frame[2]  = (float)g.ego_heading;
    frame[3]  = (float)g.ego_yaw_rate;
    frame[4]  = (float)g.front0_x;
    frame[5]  = (float)g.front0_y;
    frame[6]  = (float)g.front0_vx;
    frame[7]  = (float)g.front0_type;
    frame[8]  = (float)g.front0_confidence;
    frame[9]  = (float)g.front1_x;
    frame[10] = (float)g.front1_y;
    frame[11] = (float)g.front1_vx;
    frame[12] = (float)g.front1_type;
    frame[13] = (float)g.front1_confidence;
    frame[14] = (float)g.ctrl_brake;
    frame[15] = (float)g.ctrl_emergency_stop;
}

/* 将当前帧压入环形缓冲 */
static void push_frame(void) {
    build_frame(g.frame_buf[g.frame_head]);
    g.frame_head = (g.frame_head + 1) % TEMPORAL_WINDOW;
    if (g.frame_count < TEMPORAL_WINDOW) g.frame_count++;
}

/* ── 推理核心 ────────────────────────────────────────────────── */

/*
 * 计算模型输出。
 *
 * 输入特征维度 (in_dim) 由 model.txt 自动决定：
 *   in_dim=4   → v1: [ego_v, ego_y, ego_heading, ego_yaw_rate]（单帧）
 *   in_dim=16  → v2: v1 + front0/1 + control（单帧）
 *   in_dim=80  → v3: 5帧×16维时序滑窗（可预测行人轨迹/变道意图）
 *
 * 输出维度 (out_dim) 由 model.txt 决定：
 *   out_dim=1 → target_speed
 *   out_dim=2 → target_speed + lateral_d
 *   out_dim=4 → target_speed + lateral_d + throttle + steer
 *   out_dim=5 → throttle + brake + steer + lane_change + confidence（direct_ctrl 完整输出）
 */
static void run_inference(double* out_speed, double* out_d,
                          double* out_throttle, double* out_brake,
                          double* out_steer, double* out_lc, double* out_conf) {
    float y[TINY_MLP_MAX_OUT];
    *out_lc = 0.0;
    *out_conf = 0.0;

    if (g.model.loaded) {
        float x[TINY_MLP_MAX_IN] = {0};

        if (g.model.in_dim >= 80 && g.frame_count >= TEMPORAL_WINDOW) {
            /* v3: 时序滑窗 5×16 = 80 维 */
            int idx = 0;
            for (int w = 0; w < TEMPORAL_WINDOW; w++) {
                int fi = (g.frame_head + w) % TEMPORAL_WINDOW;
                for (int d = 0; d < V2_DIM; d++) {
                    x[idx++] = g.frame_buf[fi][d];
                }
            }
        } else if (g.model.in_dim >= 16) {
            /* v2: 单帧 16 维 */
            build_frame(x);
        } else {
            /* v1: 单帧 4 维 */
            x[0] = (float)g.ego_v;
            x[1] = (float)g.ego_y;
            x[2] = (float)g.ego_heading;
            x[3] = (float)g.ego_yaw_rate;
        }

        int n = tiny_mlp_forward(&g.model, x, y);
        if (n >= 5) {
            /* direct_ctrl 完整输出 */
            *out_throttle = y[0];
            *out_brake    = y[1];
            *out_steer    = y[2];
            *out_lc       = y[3];
            *out_conf     = y[4];
            *out_speed    = g.ego_v + (y[0] - y[1]) * 5.0;  /* 从 thr/brk 推算参考速度 */
            *out_d        = 0.0;
        } else if (n >= 4) {
            *out_speed    = y[0];
            *out_d        = y[1];
            *out_throttle = y[2];
            *out_steer    = y[3];
        } else if (n >= 2) {
            *out_speed = y[0];
            *out_d     = y[1];
            *out_throttle = 0.5;  /* 回退保守油门 */
            *out_steer = atan2(0.5 * (float)y[1], fmax((float)g.ego_v, 3.0f));
        } else if (n == 1) {
            *out_speed = y[0];
            *out_d     = 0.0;
        } else {
            *out_speed = g.ego_v;
            *out_d     = 0.0;
        }
    } else {
        /* 回退启发式 */
        double target = g.ego_v + 1.0;
        if (target > g.cfg_max_speed) target = g.cfg_max_speed;
        *out_speed    = target;
        *out_d        = 0.0;
        *out_throttle = 0.0;
        *out_brake    = 0.0;
        *out_steer    = 0.0;
    }

    /* 安全夹紧 */
    if (*out_speed < 0.0)              *out_speed = 0.0;
    if (*out_speed > g.cfg_max_speed)  *out_speed = g.cfg_max_speed;
    if (*out_d >  6.0)                 *out_d = 6.0;
    if (*out_d < -6.0)                 *out_d = -6.0;
}

/* ── 从推理输出生成直接控制指令 ───────────────────────────────── */

static void build_control_raw(double pred_speed, double pred_d,
                              double pred_throttle, double pred_brake,
                              double pred_steer, double pred_lc, double pred_conf,
                              uint8_t* buf, size_t* len, char* mode_tag) {
    double throttle = pred_throttle, brake = pred_brake, steer = pred_steer;

    /* 模型未输出有效控制时回退到 PD 推算 */
    if (throttle == 0.0 && brake == 0.0 && steer == 0.0) {
        double error = pred_speed - g.ego_v;
        if (error > 0) {
            throttle = fmin(error / 5.0, 1.0);
            brake = 0.0;
        } else {
            throttle = 0.0;
            brake = fmin((-error) / 8.0, 1.0);
        }
        steer = atan2(0.5 * pred_d, fmax(g.ego_v, 3.0));
    }

    if (steer > 0.22) steer = 0.22;
    if (steer < -0.22) steer = -0.22;
    if (throttle > 1.0) throttle = 1.0;
    if (brake > 1.0) brake = 1.0;

    ControlRaw raw;
    memset(&raw, 0, sizeof(raw));
    raw.seq      = (uint32_t)g.infer_count;
    raw.throttle = (float)throttle;
    raw.brake    = (float)brake;
    raw.steering = (float)steer;
    raw.speed    = (float)g.ego_v;
    raw.target   = (float)pred_speed;
    raw.error    = (float)(pred_speed - g.ego_v);
    snprintf(raw.mode, sizeof(raw.mode), "%s", mode_tag);

    ControlRaw_serialize(&raw, buf, len);
}

/* ── 任务线程 ────────────────────────────────────────────────── */

static void* inference_thread(void* arg) {
    (void)arg;
    pthread_setname_np(pthread_self(), "inference");

    double period = g.cfg_frequency_hz > 0.0 ? 1.0 / g.cfg_frequency_hz : 0.1;
    useconds_t sleep_us = (useconds_t)(period * 1e6);

    while (!g.should_stop) {
        usleep(sleep_us);
        if (g.should_stop) break;

        /* OTA 热重载: 检测 model_ota_node 发来的 reload 信号 */
        if (g.reload_flag) {
            g.reload_flag = 0;
            pthread_mutex_lock(&g.model_mutex);
            TinyMLP new_model;
            if (tiny_mlp_load(&new_model, g.model_path) == 0) {
                g.model = new_model;
                g.reload_count++;
                LOG_INFO("inference", "OTA hot-reload #%d from %s (in=%d hid=%d out=%d)",
                         g.reload_count, g.model_path,
                         g.model.in_dim, g.model.hid_dim, g.model.out_dim);
            } else {
                LOG_WARN("inference", "OTA hot-reload failed: %s", g.model_path);
            }
            pthread_mutex_unlock(&g.model_mutex);
        }

        if (!g.has_fusion) continue;

        /* 将当前帧压入时序缓冲 */
        push_frame();

        double pred_speed = 0.0, pred_d = 0.0;
        double pred_throttle = 0.0, pred_brake = 0.0, pred_steer = 0.0;
        double pred_lc = 0.0, pred_conf = 0.0;
        run_inference(&pred_speed, &pred_d,
                      &pred_throttle, &pred_brake, &pred_steer,
                      &pred_lc, &pred_conf);

        /* 影子对比: 与 planning 输出的目标速度差 */
        double shadow_delta = g.has_planning
            ? (pred_speed - g.planning_target_speed) : 0.0;

        const char* model_name = g.model.loaded ? "tiny-mlp" : "heuristic";

        /* 所有模式下都发布 inference/trajectory 供监控 */
        {
            char traj[1280];
            int off = snprintf(traj, sizeof(traj),
                "{\"type\":\"inference\",\"model\":\"%s\",\"infer\":%d,"
                "\"shadow\":true,\"target_speed\":%.2f,\"lateral_d\":%.2f,"
                "\"shadow_delta\":%.2f,\"throttle\":%.3f,\"brake\":%.3f,"
                "\"steer\":%.4f,\"lane_change\":%.1f,\"confidence\":%.2f,"
                "\"ego\":{\"x\":%.2f,\"y\":%.2f,\"v\":%.2f}",
                model_name, g.infer_count,
                pred_speed, pred_d, shadow_delta,
                pred_throttle, pred_brake, pred_steer, pred_lc, pred_conf,
                g.ego_x, g.ego_y, g.ego_v);

            if (g.has_obstacles) {
                off += snprintf(traj + off, sizeof(traj) - (size_t)off,
                    ",\"front0\":{\"x\":%.1f,\"y\":%.1f,\"vx\":%.1f,\"type\":%.0f}",
                    g.front0_x, g.front0_y, g.front0_vx, g.front0_type);
            }
            off += snprintf(traj + off, sizeof(traj) - (size_t)off, "}");

            transport_publish(g.transport, "inference/trajectory",
                              (const uint8_t*)traj, (uint32_t)off + 1);
        }

        /* plan_assist 模式: 额外发布结构化轨迹供 planning 消费 */
        if (g.control_mode == CTRL_MODE_PLAN_ASSIST) {
            char assist[512];
            snprintf(assist, sizeof(assist),
                "{\"type\":\"assist\",\"speed\":%.2f,\"d\":%.2f,"
                "\"throttle\":%.3f,\"steer\":%.4f,\"infer\":%d}",
                pred_speed, pred_d, pred_throttle, pred_steer, g.infer_count);
            transport_publish(g.transport, "inference/assist",
                              (const uint8_t*)assist, (uint32_t)strlen(assist) + 1);
        }

        /* direct_ctrl 模式: 发布推理控制指令（safety_control 兜底） */
        if (g.control_mode == CTRL_MODE_DIRECT) {
            uint8_t raw_buf[64];
            size_t  raw_len = sizeof(raw_buf);
            build_control_raw(pred_speed, pred_d,
                              pred_throttle, pred_brake, pred_steer,
                              pred_lc, pred_conf,
                              raw_buf, &raw_len, "INFER");
            transport_publish(g.transport, "inference/raw_cmd",
                              raw_buf, (uint32_t)raw_len);
        }

        g.infer_count++;

        if (g.infer_count % 25 == 1) {
            const char* mode_str = "shadow";
            if (g.control_mode == CTRL_MODE_PLAN_ASSIST) mode_str = "plan_assist";
            else if (g.control_mode == CTRL_MODE_DIRECT) mode_str = "direct_ctrl";
            LOG_INFO("inference",
                "#%d [%s] mode=%s ego_v=%.1f → speed=%.1f d=%.2f (shadow Δ=%.2f vs planning)",
                g.infer_count, model_name, mode_str,
                g.ego_v, pred_speed, pred_d, shadow_delta);
        }
    }

    LOG_INFO("inference", "stopped (%d inferences, state=%s)",
             g.infer_count, statem_state_name(&g.sm, g.sm.current));
    statem_send_event(&g.sm, SM_EVENT_STOP, NULL);
    statem_send_event(&g.sm, SM_EVENT_DONE, NULL);
    return NULL;
}

/* ── NodePlugin 实现 ─────────────────────────────────────────── */

static const char* s_inputs[]  = {
    "fusion/localization",
    "planning/trajectory",
    "perception/obstacles",   /* v2 */
    "control/cmd",            /* v2 */
    "model_ota/active",       /* OTA 热重载信号 */
    NULL
};
static const char* s_outputs[] = {
    "inference/trajectory",
    "inference/assist",       /* plan_assist 模式 */
    "inference/raw_cmd",      /* direct_ctrl 模式 */
    NULL
};

static NodePlugin s_plugin;  /* forward decl */

static int inference_init(MessageBus* bus, Transport* transport,
                          DiscoveryManager* discovery, Scheduler* scheduler,
                          const char* params_json) {
    (void)bus;

    memset(&g, 0, sizeof(g));
    g.transport   = transport;
    g.discovery   = discovery;
    g.scheduler   = scheduler;
    g.should_stop = 0;
    g.control_mode = CTRL_MODE_SHADOW;  /* 默认安全模式 */

    /* 默认参数 */
    g.cfg_max_speed    = 20.0;
    g.cfg_frequency_hz = 20.0;  /* 和 control 对齐，时序滑窗需要更高帧率 */
    strncpy(g.model_path, "tools/train/model.txt", sizeof(g.model_path) - 1);

    if (params_json) {
        const char* p;
        if ((p = strstr(params_json, "\"max_speed\":")))
            sscanf(p + 12, "%lf", &g.cfg_max_speed);
        if ((p = strstr(params_json, "\"frequency_hz\":")))
            sscanf(p + 15, "%lf", &g.cfg_frequency_hz);
        if ((p = strstr(params_json, "\"model_path\":\""))) {
            const char* start = p + 14;
            const char* end = strchr(start, '"');
            if (end && (size_t)(end - start) < sizeof(g.model_path)) {
                size_t len = (size_t)(end - start);
                memcpy(g.model_path, start, len);
                g.model_path[len] = '\0';
            }
        }
        /* v2: 控制模式 */
        if ((p = strstr(params_json, "\"control_mode\":\""))) {
            const char* mode_start = p + 16;
            if (strncmp(mode_start, "plan_assist", 11) == 0)
                g.control_mode = CTRL_MODE_PLAN_ASSIST;
            else if (strncmp(mode_start, "direct_ctrl", 11) == 0)
                g.control_mode = CTRL_MODE_DIRECT;
        }
    }

    pthread_mutex_init(&g.model_mutex, NULL);

    /* 尝试加载训练好的模型 */
    if (tiny_mlp_load(&g.model, g.model_path) == 0) {
        LOG_INFO("inference", "model loaded from %s (in=%d hid=%d out=%d)",
                 g.model_path, g.model.in_dim, g.model.hid_dim, g.model.out_dim);
    } else {
        g.model.loaded = 0;
        LOG_INFO("inference",
                 "no model at %s — using heuristic policy (train via tools/train/)",
                 g.model_path);
    }

    transport_subscribe(transport, "fusion/localization", on_fusion, NULL);
    transport_subscribe(transport, "planning/trajectory", on_planning, NULL);
    transport_subscribe(transport, "perception/obstacles", on_obstacles, NULL);  /* v2 */
    transport_subscribe(transport, "control/cmd", on_control_cmd, NULL);         /* v2 */
    transport_subscribe(transport, "model_ota/active", on_model_ota_active, NULL); /* OTA */
    transport_advertise(transport, "inference/trajectory", 0x1F5E2A10u);
    if (g.control_mode >= CTRL_MODE_PLAN_ASSIST)
        transport_advertise(transport, "inference/assist", 0x1F5E2A11u);
    if (g.control_mode >= CTRL_MODE_DIRECT)
        transport_advertise(transport, "inference/raw_cmd", 0x2D95C6E0u);

    discovery_advertise(discovery, "fusion/localization", 0xF0ED10C0u,
                        CAP_SUBSCRIBER, 0);
    discovery_advertise(discovery, "inference/trajectory", 0x1F5E2A10u,
                        CAP_PUBLISHER, g.cfg_frequency_hz);

    statem_init(&g.sm, NULL, SM_STATE_INITIALIZED, "inference");
    statem_send_event(&g.sm, SM_EVENT_START, NULL);

    const char* mode_str = "shadow";
    if (g.control_mode == CTRL_MODE_PLAN_ASSIST) mode_str = "plan_assist";
    else if (g.control_mode == CTRL_MODE_DIRECT) mode_str = "direct_ctrl";
    LOG_INFO("inference", "initialized (mode=%s, %.0f Hz, max=%.0f m/s, %s)",
             mode_str, g.cfg_frequency_hz, g.cfg_max_speed,
             g.model.loaded ? "model loaded" : "heuristic fallback");
    return 0;
}

static int inference_start(void) {
    g.running = 1; g.should_stop = 0;
    if (pthread_create(&g.thread, NULL, inference_thread, NULL) != 0) return -1;
    LOG_INFO("inference", "started [state=%s]", statem_state_name(&g.sm, g.sm.current));
    node_announce_self(g.transport, &s_plugin);
    return 0;
}

static void inference_stop(void) { g.should_stop = 1; }

static void inference_cleanup(void) {
    if (g.running) { g.should_stop = 1; pthread_join(g.thread, NULL); g.running = 0; }
    pthread_mutex_destroy(&g.model_mutex);
    LOG_INFO("inference", "cleanup done (reloads=%d)", g.reload_count);
}

static int inference_health(void) { return 0; }

static NodePlugin s_plugin = {
    .api_version   = NODE_PLUGIN_API_VERSION,
    .name          = "inference",
    .version       = "2.1.0",
    .description   = "On-vehicle MLP inference (v2: 16-dim + OTA hot-reload)",
    .input_topics  = s_inputs,
    .output_topics = s_outputs,
    .init          = inference_init,
    .start         = inference_start,
    .stop          = inference_stop,
    .cleanup       = inference_cleanup,
    .health        = inference_health,
};

NodePlugin* node_get_plugin(void) { return &s_plugin; }
