/**
 * inference_node.c — 车端模型推理节点 (车端学习闭环 · Stage 2)
 *
 * 订阅 fusion/localization → 用内置 tiny-MLP 推理 → 影子模式发布 inference/trajectory。
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
 *
 * 数据契约:
 *   输入特征 x[4] = { ego_v, ego_y, ego_heading, ego_yaw_rate }
 *   输出     y[1..2] = { target_speed (m/s)[, lateral_d (m)] }
 *     本节点支持 1 或 2 维输出：内置演示模型 (tools/train/model.txt) 只输出
 *     target_speed（1 维），此时 lateral_d 回退为 0；若后续训练出 2 维模型，
 *     第二维即为相对参考线的横向偏移 lateral_d。
 *
 * NodePlugin 接口，编译为 libinference_node.so。
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

/* ── 节点本地状态 ───────────────────────────────────────────── */

static struct {
    Transport*        transport;
    DiscoveryManager* discovery;
    Scheduler*        scheduler;

    pthread_t    thread;
    volatile int running;
    volatile int should_stop;

    ReflectiveStateMachine sm;

    TinyMLP model;
    char    model_path[256];

    /* 最新 ego 状态（来自 fusion/localization） */
    double ego_x, ego_y, ego_v, ego_heading, ego_yaw_rate;
    volatile int has_fusion;

    /* 影子对比: planning 发布的 target_speed（若存在） */
    double planning_target_speed;
    volatile int has_planning;

    /* 配置 */
    double cfg_max_speed;
    double cfg_frequency_hz;

    int infer_count;
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

/* ── 推理核心 ────────────────────────────────────────────────── */

/*
 * 计算模型输出。若已加载训练模型则走 tiny_mlp_forward()，否则回退到可解释启发式：
 *   - target_speed: 朝 max_speed 平滑加速（每步 +1 m/s 上限）
 *   - lateral_d:    保持在参考线上 (0)
 * 输出会被夹紧到 [0, max_speed] 与合理横向范围。
 */
static void run_inference(double* out_speed, double* out_d) {
    float y[TINY_MLP_MAX_OUT];
    if (g.model.loaded) {
        float x[4];
        x[0] = (float)g.ego_v;
        x[1] = (float)g.ego_y;
        x[2] = (float)g.ego_heading;
        x[3] = (float)g.ego_yaw_rate;
        int n = tiny_mlp_forward(&g.model, x, y);
        if (n >= 2) {
            *out_speed = y[0];
            *out_d     = y[1];
        } else if (n == 1) {
            *out_speed = y[0];
            *out_d     = 0.0;
        } else {
            *out_speed = g.ego_v;
            *out_d     = 0.0;
        }
    } else {
        double target = g.ego_v + 1.0;
        if (target > g.cfg_max_speed) target = g.cfg_max_speed;
        *out_speed = target;
        *out_d     = 0.0;
    }

    /* 安全夹紧（即便模型输出异常也不越界） */
    if (*out_speed < 0.0)              *out_speed = 0.0;
    if (*out_speed > g.cfg_max_speed)  *out_speed = g.cfg_max_speed;
    if (*out_d >  6.0)                 *out_d = 6.0;
    if (*out_d < -6.0)                 *out_d = -6.0;
}

/* ── 任务线程 ────────────────────────────────────────────────── */

static void* inference_thread(void* arg) {
    (void)arg;
    pthread_setname_np(pthread_self(), "inference");

    double period = g.cfg_frequency_hz > 0.0 ? 1.0 / g.cfg_frequency_hz : 0.1;
    useconds_t sleep_us = (useconds_t)(period * 1e6);

    while (!g.should_stop) {
        usleep(sleep_us);
        if (g.should_stop || !g.has_fusion) continue;

        double pred_speed = 0.0, pred_d = 0.0;
        run_inference(&pred_speed, &pred_d);

        /* 影子对比: 与 planning 输出的目标速度差 */
        double shadow_delta = g.has_planning
            ? (pred_speed - g.planning_target_speed) : 0.0;

        char traj[768];
        snprintf(traj, sizeof(traj),
            "{\"type\":\"inference\",\"model\":\"%s\",\"infer\":%d,"
            "\"shadow\":true,\"target_speed\":%.2f,\"lateral_d\":%.2f,"
            "\"shadow_delta\":%.2f,\"ego\":{\"x\":%.2f,\"y\":%.2f,\"v\":%.2f}}",
            g.model.loaded ? "tiny-mlp" : "heuristic",
            g.infer_count, pred_speed, pred_d, shadow_delta,
            g.ego_x, g.ego_y, g.ego_v);

        transport_publish(g.transport, "inference/trajectory",
                          (const uint8_t*)traj, (uint32_t)strlen(traj) + 1);
        g.infer_count++;

        if (g.infer_count % 25 == 1) {
            LOG_INFO("inference",
                "#%d [%s] ego_v=%.1f → speed=%.1f d=%.2f (shadow Δ=%.2f vs planning)",
                g.infer_count, g.model.loaded ? "model" : "heuristic",
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

static const char* s_inputs[]  = { "fusion/localization", "planning/trajectory", NULL };
static const char* s_outputs[] = { "inference/trajectory", NULL };

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

    /* 默认参数 */
    g.cfg_max_speed    = 20.0;
    g.cfg_frequency_hz = 10.0;
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
    }

    /* 尝试加载训练好的模型；失败则用启发式回退（不视为错误） */
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
    transport_advertise(transport, "inference/trajectory", 0x1F5E2A10u);

    discovery_advertise(discovery, "fusion/localization", 0xF0ED10C0u,
                        CAP_SUBSCRIBER, 0);
    discovery_advertise(discovery, "inference/trajectory", 0x1F5E2A10u,
                        CAP_PUBLISHER, g.cfg_frequency_hz);

    statem_init(&g.sm, NULL, SM_STATE_INITIALIZED, "inference");
    statem_send_event(&g.sm, SM_EVENT_START, NULL);

    LOG_INFO("inference", "initialized (shadow mode, %.0f Hz, max=%.0f m/s)",
             g.cfg_frequency_hz, g.cfg_max_speed);
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
    LOG_INFO("inference", "cleanup done");
}

static int inference_health(void) { return 0; }

static NodePlugin s_plugin = {
    .api_version   = NODE_PLUGIN_API_VERSION,
    .name          = "inference",
    .version       = "1.0.0",
    .description   = "On-vehicle tiny-MLP inference (shadow mode)",
    .input_topics  = s_inputs,
    .output_topics = s_outputs,
    .init          = inference_init,
    .start         = inference_start,
    .stop          = inference_stop,
    .cleanup       = inference_cleanup,
    .health        = inference_health,
};

NodePlugin* node_get_plugin(void) { return &s_plugin; }
