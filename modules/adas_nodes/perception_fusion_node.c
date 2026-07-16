/**
 * perception_fusion_node.c — 感知融合节点（LiDAR + Stereo 两路 ObstacleList 去重合并）
 *
 * 解决 lidar_driver 和 stereo_vision 都发布 perception/obstacles 时的"后写者覆盖
 * 先写者"问题。两源各发到独立 topic（perception/obstacles_lidar / _stereo），
 * 本节点订阅两路，按 (x,y) 距离去重合并成更稳的 ObstacleList，输出 perception/obstacles。
 *
 * 链路:
 *   lidar_driver   ──▶ perception/obstacles_lidar  ─┐
 *   stereo_vision  ──▶ perception/obstacles_stereo ─┼──▶ [本节点] ──▶ perception/obstacles
 *                                                    │
 * 合并策略:
 *   1. 收集两路障碍物到池（最多 16 = 8+8）
 *   2. 按前向距离 x 排序（近的优先保留）
 *   3. 遍历，与已选障碍物距离 < merge_dist 则合并（位置取置信度高的，尺寸取大的），
 *      否则选入
 *   4. 取最近 8 个，id 重新编号 0..n-1
 *
 * 新鲜度: 某一路超过 max_age_ms 未更新则视为失效，只用另一路。
 *
 * 话题契约:
 *   输入: perception/obstacles_lidar  (ObstacleList, 可配 input_a_topic)
 *         perception/obstacles_stereo (ObstacleList, 可配 input_b_topic)
 *   输出: perception/obstacles        (ObstacleList, 可配 output_topic)
 *
 * 典型 pipeline.json 配置:
 *   // lidar_driver 和 stereo_vision 的 output_topic 改成各自的别名 topic:
 *   "lidar_driver":  { "params": "{\"output_topic\":\"perception/obstacles_lidar\",...}" }
 *   "stereo_vision": { "params": "{\"output_topic\":\"perception/obstacles_stereo\",...}" }
 *   "perception_fusion": {
 *     "library": "libperception_fusion_node.so",
 *     "subscribe": ["perception/obstacles_lidar", "perception/obstacles_stereo"],
 *     "publish":  [{ "topic": "perception/obstacles", "type": "ObstacleList" }],
 *     "params": "{\"merge_dist\":1.0,\"publish_hz\":10,\"max_age_ms\":500}"
 *   }
 *
 * 不用融合时（单源直发）: lidar_driver/stereo_vision 的 output_topic 保持默认
 * perception/obstacles，不挂本节点即可，完全向后兼容。
 *
 * 编译依赖: adas_msgs_gen.h (ObstacleList 反序列化/序列化)，随构建生成。
 */

#include "node_plugin.h"
#include "adas_msgs_gen.h"
#include "transport.h"
#include "discovery.h"
#include "logger.h"

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

    /* 两路输入 topic 名（可配） */
    char input_a_topic[64];   /* 默认 perception/obstacles_lidar */
    char input_b_topic[64];   /* 默认 perception/obstacles_stereo */
    char output_topic[64];    /* 默认 perception/obstacles */

    /* 融合参数 */
    double merge_dist;        /* 去重距离阈值 (m)，默认 1.0 */
    int    publish_hz;        /* 发布频率，默认 10 */
    int    max_age_ms;        /* 输入最大新鲜期 (ms)，超期视为失效，默认 500 */
    int    enabled;

    /* 两路输入缓存（回调写，工作线程读，各自 mutex 保护） */
    ObstacleList  a_list;
    uint64_t      a_ts_us;
    int           a_ready;
    pthread_mutex_t a_lock;

    ObstacleList  b_list;
    uint64_t      b_ts_us;
    int           b_ready;
    pthread_mutex_t b_lock;

    /* 统计 */
    uint64_t frames_in_a;
    uint64_t frames_in_b;
    uint64_t frames_out;
    uint64_t merges_done;

    /* 线程 */
    pthread_t thread;
    volatile int thread_running;
    volatile int should_stop;
} g;

/* ── 时间工具 ─────────────────────────────────────────────── */

static uint64_t now_us(void) {
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    return (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;
}

/* ── 参数解析 ─────────────────────────────────────────────── */

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

/* ── 订阅回调：两路 ObstacleList ──────────────────────────── */

static void on_input_a(const Message* msg, void* user_data) {
    (void)user_data;
    if (!msg || !msg->data || !g.enabled) return;
    ObstacleList list;
    if (ObstacleList_deserialize(&list, (const uint8_t*)msg->data, msg->data_size) != 0) {
        LOG_WARN("perception_fusion", "input_a deserialize failed (size=%u)", msg->data_size);
        return;
    }
    pthread_mutex_lock(&g.a_lock);
    g.a_list = list;
    g.a_ts_us = now_us();
    g.a_ready = 1;
    g.frames_in_a++;
    pthread_mutex_unlock(&g.a_lock);
}

static void on_input_b(const Message* msg, void* user_data) {
    (void)user_data;
    if (!msg || !msg->data || !g.enabled) return;
    ObstacleList list;
    if (ObstacleList_deserialize(&list, (const uint8_t*)msg->data, msg->data_size) != 0) {
        LOG_WARN("perception_fusion", "input_b deserialize failed (size=%u)", msg->data_size);
        return;
    }
    pthread_mutex_lock(&g.b_lock);
    g.b_list = list;
    g.b_ts_us = now_us();
    g.b_ready = 1;
    g.frames_in_b++;
    pthread_mutex_unlock(&g.b_lock);
}

/* ── 融合核心：两路 ObstacleList → 去重合并 ──────────────────
 *
 * 算法:
 *   1. 收集两路障碍物到 pool（最多 16）
 *   2. 按前向距离 x 升序排序（近的优先保留）
 *   3. 遍历 pool，对每个障碍物检查是否与已选集合中任一障碍物距离 < merge_dist:
 *      - 是 → 合并（位置取置信度高的，尺寸取大的，置信度取大的），不新增
 *      - 否 → 加入已选集合（最多 8 个）
 *   4. id 重新编号 0..n-1
 */
typedef struct { Obstacle obs; } ObsEntry;

static int cmp_obs_by_x(const void* a, const void* b) {
    const ObsEntry* ea = (const ObsEntry*)a;
    const ObsEntry* eb = (const ObsEntry*)b;
    if (ea->obs.x < eb->obs.x) return -1;
    if (ea->obs.x > eb->obs.x) return  1;
    return 0;
}

static void fuse_obstacles(const ObstacleList* a, const ObstacleList* b,
                           double merge_dist, ObstacleList* out) {
    ObsEntry pool[16];
    int n = 0;

    if (a) {
        for (uint32_t i = 0; i < a->count && n < 16; i++) {
            pool[n++].obs = a->obstacles[i];
        }
    }
    if (b) {
        for (uint32_t i = 0; i < b->count && n < 16; i++) {
            pool[n++].obs = b->obstacles[i];
        }
    }

    if (n == 0) {
        memset(out, 0, sizeof(*out));
        return;
    }

    /* 按前向距离排序 */
    qsort(pool, (size_t)n, sizeof(ObsEntry), cmp_obs_by_x);

    /* 去重合并 */
    Obstacle selected[8];
    int sel_n = 0;
    int merges = 0;

    for (int i = 0; i < n; i++) {
        const Obstacle* cand = &pool[i].obs;
        int merged = 0;
        for (int j = 0; j < sel_n; j++) {
            double dx = (double)cand->x - (double)selected[j].x;
            double dy = (double)cand->y - (double)selected[j].y;
            double dist = sqrt(dx * dx + dy * dy);
            if (dist < merge_dist) {
                /* 合并：位置取置信度高的 */
                if (cand->confidence > selected[j].confidence) {
                    selected[j].x = cand->x;
                    selected[j].y = cand->y;
                    selected[j].confidence = cand->confidence;
                    /* 速度也跟随高置信度源 */
                    selected[j].vx = cand->vx;
                    selected[j].vy = cand->vy;
                }
                /* 尺寸取大的（更保守的安全包络） */
                if (cand->width  > selected[j].width)  selected[j].width  = cand->width;
                if (cand->length > selected[j].length) selected[j].length = cand->length;
                /* 类型取置信度高的（已在上面条件里换了位置，这里同步类型） */
                if (cand->confidence > selected[j].confidence) {
                    selected[j].type = cand->type;
                }
                merges++;
                merged = 1;
                break;
            }
        }
        if (!merged && sel_n < 8) {
            selected[sel_n++] = *cand;
        }
    }

    /* 输出：id 重新编号 */
    memset(out, 0, sizeof(*out));
    out->count = (uint32_t)sel_n;
    for (int i = 0; i < sel_n; i++) {
        selected[i].id = (uint32_t)i;
        out->obstacles[i] = selected[i];
    }
    g.merges_done += (uint64_t)merges;
}

/* ── 工作线程：定时融合 + 发布 ────────────────────────────── */

static void* fusion_thread(void* arg) {
    (void)arg;
    pthread_setname_np(pthread_self(), "perc_fuse");
    long period_us = 1000000L / (g.publish_hz > 0 ? g.publish_hz : 10);

    while (!g.should_stop) {
        usleep((useconds_t)period_us);
        if (g.should_stop || !g.enabled) break;

        uint64_t now = now_us();
        uint64_t max_age_us = (uint64_t)g.max_age_ms * 1000ULL;

        /* 拷贝两路缓存（加锁，快拷贝） */
        ObstacleList a_copy, b_copy;
        int has_a = 0, has_b = 0;

        pthread_mutex_lock(&g.a_lock);
        if (g.a_ready && (now - g.a_ts_us) < max_age_us) {
            a_copy = g.a_list;
            has_a = 1;
        }
        pthread_mutex_unlock(&g.a_lock);

        pthread_mutex_lock(&g.b_lock);
        if (g.b_ready && (now - g.b_ts_us) < max_age_us) {
            b_copy = g.b_list;
            has_b = 1;
        }
        pthread_mutex_unlock(&g.b_lock);

        if (!has_a && !has_b) continue;  /* 两路都无数据/过期，跳过 */

        /* 融合 */
        ObstacleList fused;
        fuse_obstacles(has_a ? &a_copy : NULL,
                       has_b ? &b_copy : NULL,
                       g.merge_dist, &fused);

        if (fused.count == 0) continue;

        /* 序列化 + 发布 */
        fused.frame_id     = (uint32_t)(g.frames_out & 0xFFFFFFFFu);
        fused.timestamp_us = now;

        uint8_t buf[280];
        size_t len = 0;
        if (ObstacleList_serialize(&fused, buf, &len) == 0 && len > 0) {
            transport_publish(g.transport, g.output_topic, buf, (uint32_t)len);
            g.frames_out++;
        }

        /* 周期性日志（每 50 次一次） */
        if (g.frames_out % 50 == 1) {
            LOG_INFO("perception_fusion", "out #%lu: a=%d b=%d → fused=%d (merges=%lu)",
                     (unsigned long)g.frames_out, has_a ? (int)a_copy.count : -1,
                     has_b ? (int)b_copy.count : -1, (int)fused.count,
                     (unsigned long)g.merges_done);
        }
    }
    return NULL;
}

/* ── NodePlugin 实现 ─────────────────────────────────────── */

static const char* s_inputs[]  = { "perception/obstacles_lidar",
                                    "perception/obstacles_stereo", NULL };
static const char* s_outputs[] = { "perception/obstacles", NULL };

static int fusion_init(MessageBus* bus, Transport* transport,
                       DiscoveryManager* discovery, Scheduler* scheduler,
                       const char* params_json) {
    (void)bus; (void)scheduler;
    memset(&g, 0, sizeof(g));
    g.transport = transport;
    g.discovery = discovery;

    pthread_mutex_init(&g.a_lock, NULL);
    pthread_mutex_init(&g.b_lock, NULL);

    /* 默认参数 */
    snprintf(g.input_a_topic, sizeof(g.input_a_topic), "perception/obstacles_lidar");
    snprintf(g.input_b_topic, sizeof(g.input_b_topic), "perception/obstacles_stereo");
    snprintf(g.output_topic,  sizeof(g.output_topic),  "perception/obstacles");
    g.merge_dist  = 1.0;
    g.publish_hz  = 10;
    g.max_age_ms  = 500;
    g.enabled     = 1;

    if (params_json) {
        parse_string(params_json, "input_a_topic", g.input_a_topic,
                     sizeof(g.input_a_topic), "perception/obstacles_lidar");
        parse_string(params_json, "input_b_topic", g.input_b_topic,
                     sizeof(g.input_b_topic), "perception/obstacles_stereo");
        parse_string(params_json, "output_topic", g.output_topic,
                     sizeof(g.output_topic), "perception/obstacles");
        g.merge_dist = parse_double(params_json, "merge_dist", 1.0);
        g.publish_hz = parse_int(params_json, "publish_hz", 10);
        g.max_age_ms = parse_int(params_json, "max_age_ms", 500);
        g.enabled    = parse_int(params_json, "enable", 1);
    }

    if (!g.enabled) {
        LOG_INFO("perception_fusion", "disabled by config (enable=0)");
        return 0;
    }

    /* 订阅两路输入 */
    transport_subscribe(transport, g.input_a_topic, on_input_a, NULL);
    transport_subscribe(transport, g.input_b_topic, on_input_b, NULL);
    discovery_advertise(discovery, g.input_a_topic, OBSTACLELIST_TYPE_ID, CAP_SUBSCRIBER, 0);
    discovery_advertise(discovery, g.input_b_topic, OBSTACLELIST_TYPE_ID, CAP_SUBSCRIBER, 0);

    /* 发布融合结果 */
    discovery_advertise(discovery, g.output_topic, OBSTACLELIST_TYPE_ID, CAP_PUBLISHER,
                        (double)g.publish_hz);

    LOG_INFO("perception_fusion", "initialized: a=%s b=%s → out=%s "
             "merge_dist=%.2fm hz=%d max_age=%dms",
             g.input_a_topic, g.input_b_topic, g.output_topic,
             g.merge_dist, g.publish_hz, g.max_age_ms);
    return 0;
}

static int fusion_start(void) {
    if (!g.enabled) return 0;
    g.should_stop = 0;
    g.thread_running = 1;
    if (pthread_create(&g.thread, NULL, fusion_thread, NULL) != 0) {
        LOG_ERROR("perception_fusion", "failed to create fusion thread");
        g.thread_running = 0;
        return -1;
    }
    return 0;
}

static void fusion_stop(void) {
    g.should_stop = 1;
}

static void fusion_cleanup(void) {
    if (g.thread_running) {
        g.should_stop = 1;
        pthread_join(g.thread, NULL);
        g.thread_running = 0;
    }
    pthread_mutex_destroy(&g.a_lock);
    pthread_mutex_destroy(&g.b_lock);
    LOG_INFO("perception_fusion", "cleanup: in_a=%lu in_b=%lu out=%lu merges=%lu",
             (unsigned long)g.frames_in_a, (unsigned long)g.frames_in_b,
             (unsigned long)g.frames_out, (unsigned long)g.merges_done);
}

/* ── 插件导出 ─────────────────────────────────────────────── */

static NodePlugin s_plugin = {
    .api_version   = NODE_PLUGIN_API_VERSION,
    .name          = "perception_fusion",
    .version       = "1.0.0",
    .description   = "Perception fusion (LiDAR + Stereo ObstacleList merge → perception/obstacles)",
    .input_topics  = s_inputs,
    .output_topics = s_outputs,
    .init          = fusion_init,
    .start         = fusion_start,
    .stop          = fusion_stop,
    .cleanup       = fusion_cleanup,
};
