/**
 * data_recorder_node.c — 训练样本采集节点 (车端学习闭环 · Stage 0)
 *
 * 订阅 fusion/localization + planning/trajectory，按固定频率把"最新对齐"的
 * (特征, 标签) 样本以 JSONL (每行一个 JSON) 落盘，供 tools/train/ 离线训练。
 *
 * 设计要点:
 *   - 时间对齐: 采用"最新值锁存"(latest-value latch) 的简单对齐策略，采样时刻
 *     取各 topic 最近一次的值。对 10~50Hz 的同源仿真链路足够；真实车端可换成
 *     基于时间戳的插值对齐。
 *   - 数据契约 (与 inference_node / tools/train 一致):
 *       特征 features = [ego_v, ego_y, ego_heading, ego_yaw_rate]
 *       标签 label    = planning_target_speed (模仿学习: 学 planning 的目标速度)
 *   - 轻量: 只写文本 JSONL，不引入 Bag v2 二进制依赖，便于 Python 直接解析。
 *
 * NodePlugin 接口，编译为 libdata_recorder_node.so。
 */

#include "node_plugin.h"
#include "state_machine.h"
#include "adas_msgs_gen.h"
#include "logger.h"

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <pthread.h>
#include <unistd.h>
#include <time.h>

static struct {
    Transport*        transport;
    DiscoveryManager* discovery;
    Scheduler*        scheduler;

    pthread_t    thread;
    volatile int running;
    volatile int should_stop;

    ReflectiveStateMachine sm;

    FILE* out;
    char  out_path[256];

    /* 最新锁存值 */
    double ego_x, ego_y, ego_v, ego_heading, ego_yaw_rate;
    double planning_target_speed;
    volatile int has_fusion;
    volatile int has_planning;

    double cfg_frequency_hz;
    int    sample_count;
} g;

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
    }
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

static long long now_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    return (long long)ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
}

static void* recorder_thread(void* arg) {
    (void)arg;
    pthread_setname_np(pthread_self(), "recorder");

    double period = g.cfg_frequency_hz > 0.0 ? 1.0 / g.cfg_frequency_hz : 0.1;
    useconds_t sleep_us = (useconds_t)(period * 1e6);

    while (!g.should_stop) {
        usleep(sleep_us);
        if (g.should_stop) break;
        /* 只在两路数据都到齐时采样，保证 (特征,标签) 完整 */
        if (!g.has_fusion || !g.has_planning || !g.out) continue;

        fprintf(g.out,
            "{\"t\":%lld,\"features\":[%.4f,%.4f,%.4f,%.4f],"
            "\"label\":%.4f,\"ego\":{\"x\":%.3f,\"y\":%.3f}}\n",
            now_ms(),
            g.ego_v, g.ego_y, g.ego_heading, g.ego_yaw_rate,
            g.planning_target_speed,
            g.ego_x, g.ego_y);
        fflush(g.out);
        g.sample_count++;

        if (g.sample_count % 50 == 1) {
            LOG_INFO("recorder", "#%d sample: v=%.1f y=%.2f → label(target_speed)=%.1f",
                     g.sample_count, g.ego_v, g.ego_y, g.planning_target_speed);
        }
    }

    LOG_INFO("recorder", "stopped (%d samples → %s)", g.sample_count, g.out_path);
    statem_send_event(&g.sm, SM_EVENT_STOP, NULL);
    statem_send_event(&g.sm, SM_EVENT_DONE, NULL);
    return NULL;
}

static const char* s_inputs[]  = { "fusion/localization", "planning/trajectory", NULL };
static const char* s_outputs[] = { NULL };

static NodePlugin s_plugin;  /* forward decl */

static int recorder_init(MessageBus* bus, Transport* transport,
                         DiscoveryManager* discovery, Scheduler* scheduler,
                         const char* params_json) {
    (void)bus;

    memset(&g, 0, sizeof(g));
    g.transport   = transport;
    g.discovery   = discovery;
    g.scheduler   = scheduler;
    g.should_stop = 0;

    g.cfg_frequency_hz = 10.0;
    strncpy(g.out_path, "/tmp/flow_train_samples.jsonl", sizeof(g.out_path) - 1);

    if (params_json) {
        const char* p;
        if ((p = strstr(params_json, "\"frequency_hz\":")))
            sscanf(p + 15, "%lf", &g.cfg_frequency_hz);
        if ((p = strstr(params_json, "\"output_path\":\""))) {
            const char* start = p + 15;
            const char* end = strchr(start, '"');
            if (end && (size_t)(end - start) < sizeof(g.out_path)) {
                size_t len = (size_t)(end - start);
                memcpy(g.out_path, start, len);
                g.out_path[len] = '\0';
            }
        }
    }

    g.out = fopen(g.out_path, "w");
    if (!g.out) {
        LOG_ERROR("recorder", "cannot open output file %s", g.out_path);
        return -1;
    }

    transport_subscribe(transport, "fusion/localization", on_fusion, NULL);
    transport_subscribe(transport, "planning/trajectory", on_planning, NULL);

    discovery_advertise(discovery, "fusion/localization", 0xF0ED10C0u,
                        CAP_SUBSCRIBER, 0);
    discovery_advertise(discovery, "planning/trajectory", 0x3A7B1C2Du,
                        CAP_SUBSCRIBER, 0);

    statem_init(&g.sm, NULL, SM_STATE_INITIALIZED, "recorder");
    statem_send_event(&g.sm, SM_EVENT_START, NULL);

    LOG_INFO("recorder", "initialized (%.0f Hz → %s)",
             g.cfg_frequency_hz, g.out_path);
    return 0;
}

static int recorder_start(void) {
    g.running = 1; g.should_stop = 0;
    if (pthread_create(&g.thread, NULL, recorder_thread, NULL) != 0) return -1;
    LOG_INFO("recorder", "started [state=%s]", statem_state_name(&g.sm, g.sm.current));
    node_announce_self(g.transport, &s_plugin);
    return 0;
}

static void recorder_stop(void) { g.should_stop = 1; }

static void recorder_cleanup(void) {
    if (g.running) { g.should_stop = 1; pthread_join(g.thread, NULL); g.running = 0; }
    if (g.out) { fclose(g.out); g.out = NULL; }
    LOG_INFO("recorder", "cleanup done");
}

static int recorder_health(void) { return g.out ? 0 : -1; }

static NodePlugin s_plugin = {
    .api_version   = NODE_PLUGIN_API_VERSION,
    .name          = "data_recorder",
    .version       = "1.0.0",
    .description   = "Training sample recorder (imitation learning JSONL)",
    .input_topics  = s_inputs,
    .output_topics = s_outputs,
    .init          = recorder_init,
    .start         = recorder_start,
    .stop          = recorder_stop,
    .cleanup       = recorder_cleanup,
    .health        = recorder_health,
};

NodePlugin* node_get_plugin(void) { return &s_plugin; }
