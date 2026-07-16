/**
 * perception_fusion_node.c — 感知融合节点（LiDAR + Stereo 两路 ObstacleList 去重合并
 *                            + 跨帧时序追踪 Phase 2）
 *
 * 解决 lidar_driver 和 stereo_vision 都发布 perception/obstacles 时的"后写者覆盖
 * 先写者"问题。两源各发到独立 topic（perception/obstacles_lidar / _stereo），
 * 本节点订阅两路，按 (x,y) 距离去重合并后做跨帧最近邻关联，输出带持久 ID 和
 * 差分速度(vx/vy)的 ObstacleList 到 perception/obstacles。
 *
 * 链路:
 *   lidar_driver   ──▶ perception/obstacles_lidar  ─┐
 *   stereo_vision  ──▶ perception/obstacles_stereo ─┼──▶ [本节点] ──▶ perception/obstacles
 *                                                    │              (持久ID + vx/vy)
 * 算法:
 *   Stage 1 — 空间去重合并（fuse_obstacles, 同 Phase 1）:
 *     1. 收集两路障碍物到池（最多 16）
 *     2. 按前向距离 x 排序（近的优先保留）
 *     3. 遍历，与已选障碍物距离 < merge_dist 则合并
 *     4. 输出去重后的检测列表（最多 8 个）
 *
 *   Stage 2 — 跨帧时序追踪（associate_and_track, Phase 2 新增）:
 *     1. 对每个 fused detection 找最近的 track（Euclidean ≤ assoc_dist, 3m）
 *     2. 匹配成功 → 更新 track 位置，差分测速 + EMA 平滑
 *     3. 未匹配 detection → 新建 track，vx=vy=0（首帧无速度）
 *     4. 连续未匹配的 track → missed++，>max_missed 删除
 *     5. 输出活跃 track 作为 ObstacleList，id=持久 track_id
 *
 * 新鲜度: 某一路超过 max_age_ms 未更新则视为失效，只用另一路。
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

/* Phase 2: 跨帧追踪 track */
#define PF_MAX_TRACKS 16

typedef struct {
    int      id;              /* 持久 track ID（单调递增） */
    int      active;          /* 此槽位是否在使用 */
    double   x, y;            /* 当前位置（车体坐标系） */
    double   vx, vy;          /* EMA 平滑速度 (m/s) */
    double   width, length;   /* 尺寸 */
    int8_t   type;            /* ObstacleType */
    float    confidence;
    uint64_t last_update_us;  /* 上次关联时间戳 */
    int      age;             /* 总存活帧数 */
    int      missed;          /* 连续未匹配帧数 */
    /* Phase 6: 远距外推 */
    int      extrapolated;    /* 是否处于外推模式(超出传感器覆盖) */
    int      extrap_frames;   /* 连续外推帧数 */
} Track;

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

    /* Phase 2: 时序追踪参数 */
    double track_assoc_dist;  /* 最近邻关联距离阈值 (m)，默认 3.0 */
    int    track_max_missed;  /* 连续未匹配帧数上限，超此删除，默认 3 */
    double track_ema_alpha;   /* 速度 EMA 平滑因子 (0-1)，默认 0.3 */
    int    tracking_enabled;  /* 是否启用跨帧追踪，默认 1 */

    /* Phase 6: 远距外推参数 */
    int    far_enabled;          /* 是否启用远距外推,默认 1 */
    int    far_persist_frames;   /* 离开传感器覆盖后持续外推帧数,默认 5 */
    double far_max_range;        /* 外推最远距离 (m),默认 30.0 */
    double far_conf_decay;       /* 每帧置信度衰减系数,默认 0.85 */

    /* Phase 2: Track 池 */
    Track  tracks[PF_MAX_TRACKS];
    int    track_count;       /* 当前活跃 track 数 */
    int    next_track_id;     /* 下一个新 track 的 ID */

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
    uint64_t tracks_created;     /* Phase 2: 累计创建的 track 数 */
    uint64_t tracks_matched;     /* Phase 2: 累计成功关联次数 */
    uint64_t tracks_killed;      /* Phase 2: 累计因未匹配删除的 track 数 */

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

/* ── Phase 2: 跨帧时序追踪 ──────────────────────────────────
 *
 * 对 fuse_obstacles 产出的空间去重检测列表做最近邻跨帧关联:
 *   1. 每个检测找最近的活跃 track（Euclidean ≤ assoc_dist）
 *   2. 匹配成功 → 差分测速 + EMA 平滑,更新 track 位置/尺寸
 *   3. 未匹配 detection → 新建 track (vx=vy=0, 首帧无速度)
 *   4. 本轮未关联的 track → missed++, 超过 max_missed 则删除
 *   5. 按 x 排序输出活跃 track → ObstacleList
 *
 * @param detections  输入: 空间融合后的检测列表
 * @param now_us      当前时间戳 (微秒)
 * @param out         输出: 带持久 ID + 速度的 ObstacleList
 */

static void associate_and_track(const ObstacleList* detections,
                                 uint64_t now_us, ObstacleList* out) {
    int nd = (int)detections->count;
    int matched_det[8] = {0};  /* detection i 是否已关联到 track */
    int matched_trk[PF_MAX_TRACKS] = {0};  /* track j 本轮是否已关联 */

    /* ── Pass 1: 对每个 detection 找最近 track ──────────── */
    for (int i = 0; i < nd; i++) {
        const Obstacle* d = &detections->obstacles[i];
        int best_j = -1;
        double best_d2 = g.track_assoc_dist * g.track_assoc_dist;

        for (int j = 0; j < PF_MAX_TRACKS; j++) {
            if (!g.tracks[j].active || matched_trk[j]) continue;
            double dx = (double)d->x - g.tracks[j].x;
            double dy = (double)d->y - g.tracks[j].y;
            double d2 = dx * dx + dy * dy;
            if (d2 < best_d2) {
                best_d2 = d2;
                best_j = j;
            }
        }

        if (best_j >= 0) {
            /* 匹配成功: 更新 track */
            Track* t = &g.tracks[best_j];
            uint64_t dt_us = now_us - t->last_update_us;
            double dt_s = (dt_us > 0) ? (double)dt_us / 1000000.0 : 0.1;

            /* 差分测速 */
            double raw_vx = (dt_s > 1e-6) ? ((double)d->x - t->x) / dt_s : 0.0;
            double raw_vy = (dt_s > 1e-6) ? ((double)d->y - t->y) / dt_s : 0.0;

            /* EMA 平滑 */
            double a = g.track_ema_alpha;
            t->vx = a * raw_vx + (1.0 - a) * t->vx;
            t->vy = a * raw_vy + (1.0 - a) * t->vy;

            t->x = (double)d->x;
            t->y = (double)d->y;
            t->width  = (double)d->width;
            t->length = (double)d->length;
            t->type   = d->type;
            t->confidence    = d->confidence;
            t->last_update_us = now_us;
            t->age++;
            t->missed = 0;

            matched_det[i] = 1;
            matched_trk[best_j] = 1;
            g.tracks_matched++;
        }
    }

    /* ── Pass 2: 未匹配的 detection → 新建 track ───────── */
    for (int i = 0; i < nd; i++) {
        if (matched_det[i]) continue;

        /* 找空闲槽位 */
        int slot = -1;
        for (int j = 0; j < PF_MAX_TRACKS; j++) {
            if (!g.tracks[j].active) { slot = j; break; }
        }
        if (slot < 0) continue;  /* track 池满，丢弃 */

        const Obstacle* d = &detections->obstacles[i];
        Track* t = &g.tracks[slot];
        memset(t, 0, sizeof(*t));
        t->id   = g.next_track_id++;
        t->active = 1;
        t->x = (double)d->x;
        t->y = (double)d->y;
        t->vx = 0.0;  /* 首帧无速度估计 */
        t->vy = 0.0;
        t->width  = (double)d->width;
        t->length = (double)d->length;
        t->type   = d->type;
        t->confidence    = d->confidence;
        t->last_update_us = now_us;
        t->age    = 1;
        t->missed = 0;

        matched_trk[slot] = 1;
        g.tracks_created++;
    }

    /* ── Pass 3: 本轮未匹配的 track → missed++ / 远距外推 / 删除 ── */
    for (int j = 0; j < PF_MAX_TRACKS; j++) {
        if (!g.tracks[j].active) continue;
        if (matched_trk[j]) {
            /* 刚被匹配: 清除外推状态 */
            g.tracks[j].extrapolated  = 0;
            g.tracks[j].extrap_frames = 0;
            continue;
        }

        g.tracks[j].missed++;

        /* Phase 6: 远距外推。
         * track 离开传感器覆盖但有有效速度时，用常速度模型外推位置，
         * 保持 far_persist_frames 帧后才删除。外推期间置信度逐帧衰减。 */
        int can_extrapolate = g.far_enabled &&
            g.tracks[j].extrap_frames < g.far_persist_frames &&
            (fabs(g.tracks[j].vx) + fabs(g.tracks[j].vy)) > 0.05;

        if (can_extrapolate) {
            /* 估算 dt: publish_hz 的倒数 */
            double dt_s = 1.0 / (g.publish_hz > 0 ? g.publish_hz : 10);
            g.tracks[j].x += g.tracks[j].vx * dt_s;
            g.tracks[j].y += g.tracks[j].vy * dt_s;
            g.tracks[j].extrapolated  = 1;
            g.tracks[j].extrap_frames++;
            g.tracks[j].confidence   *= (float)g.far_conf_decay;

            /* 超出远距上限 → 不再外推,正常走删除逻辑 */
            if (g.tracks[j].x > g.far_max_range ||
                g.tracks[j].x < -g.far_max_range) {
                can_extrapolate = 0;
            }
        }

        if (!can_extrapolate && g.tracks[j].missed > g.track_max_missed) {
            g.tracks[j].active = 0;
            g.tracks_killed++;
        }
    }

    /* ── 收集活跃 track → 输出 ─────────────────────────── */
    int n_out = 0;
    Obstacle sorted[8];
    for (int j = 0; j < PF_MAX_TRACKS && n_out < 8; j++) {
        if (!g.tracks[j].active) continue;
        /* Phase 6: 外推 track 置信度打折,type 标记为 UNKNOWN(远距无法分类) */
        Obstacle o;
        memset(&o, 0, sizeof(o));
        o.id         = (uint32_t)g.tracks[j].id;
        o.x          = (float)g.tracks[j].x;
        o.y          = (float)g.tracks[j].y;
        o.vx         = (float)g.tracks[j].vx;
        o.vy         = (float)g.tracks[j].vy;
        o.width      = (float)g.tracks[j].width;
        o.length     = (float)g.tracks[j].length;
        o.type       = g.tracks[j].extrapolated ? OBJ_TYPE_UNKNOWN : g.tracks[j].type;
        o.confidence = g.tracks[j].confidence;  /* 外推时已逐帧衰减 */
        sorted[n_out++] = o;
    }

    /* 按前向距离排序 */
    for (int i = 0; i < n_out - 1; i++) {
        for (int j = i + 1; j < n_out; j++) {
            if (sorted[i].x > sorted[j].x) {
                Obstacle tmp = sorted[i];
                sorted[i] = sorted[j];
                sorted[j] = tmp;
            }
        }
    }

    g.track_count = n_out;
    memset(out, 0, sizeof(*out));
    out->count = (uint32_t)n_out;
    for (int i = 0; i < n_out; i++) {
        out->obstacles[i] = sorted[i];
    }
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

        /* Stage 1: 空间融合（两路去重） */
        ObstacleList fused;
        fuse_obstacles(has_a ? &a_copy : NULL,
                       has_b ? &b_copy : NULL,
                       g.merge_dist, &fused);

        /* Stage 2 (Phase 2): 跨帧时序追踪
         * 对融合检测做最近邻关联 → 持久 ID + 差分速度。
         * 融合结果为空时仍跑一遍追踪以清理过期 track。 */
        ObstacleList tracked;
        if (g.tracking_enabled) {
            associate_and_track(&fused, now, &tracked);
        } else {
            tracked = fused;  /* 不追踪，直接透传（向后兼容） */
        }

        /* 序列化 + 发布 */
        tracked.frame_id     = (uint32_t)(g.frames_out & 0xFFFFFFFFu);
        tracked.timestamp_us = now;

        uint8_t buf[280];
        size_t len = 0;
        if (ObstacleList_serialize(&tracked, buf, &len) == 0 && len > 0) {
            transport_publish(g.transport, g.output_topic, buf, (uint32_t)len);
            g.frames_out++;
        }

        /* 周期性日志（每 50 次,含追踪统计） */
        if (g.frames_out % 50 == 1) {
            LOG_INFO("perception_fusion", "out #%lu: a=%d b=%d fused=%d "
                     "trk=%d(created=%lu matched=%lu killed=%lu) merges=%lu",
                     (unsigned long)g.frames_out,
                     has_a ? (int)a_copy.count : -1,
                     has_b ? (int)b_copy.count : -1,
                     (int)fused.count,
                     g.track_count,
                     (unsigned long)g.tracks_created,
                     (unsigned long)g.tracks_matched,
                     (unsigned long)g.tracks_killed,
                     (unsigned long)g.merges_done);
        }
    }
    return NULL;
}

/* ── NodePlugin 实现 ─────────────────────────────────────── */

static const char* s_inputs[]  = { "perception/obstacles_lidar",
                                    "perception/obstacles_stereo", NULL };
static const char* s_outputs[] = { "perception/obstacles", NULL };

static NodePlugin s_plugin;  /* forward decl for node_announce_self in fusion_start */

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

    /* Phase 2: 追踪默认参数 */
    g.track_assoc_dist = 3.0;
    g.track_max_missed = 3;
    g.track_ema_alpha  = 0.3;
    g.tracking_enabled = 1;
    g.next_track_id    = 1;  /* ID 从 1 开始,0 保留 */

    /* Phase 6: 远距外推默认参数 */
    g.far_enabled        = 1;
    g.far_persist_frames = 5;
    g.far_max_range      = 30.0;
    g.far_conf_decay     = 0.85;

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

        /* Phase 2 追踪参数 */
        g.track_assoc_dist = parse_double(params_json, "track_assoc_dist", 3.0);
        g.track_max_missed = parse_int(params_json, "track_max_missed", 3);
        g.track_ema_alpha  = parse_double(params_json, "track_ema_alpha", 0.3);
        g.tracking_enabled = parse_int(params_json, "tracking_enabled", 1);

        /* Phase 6 远距外推参数 */
        g.far_enabled        = parse_int(params_json, "far_enabled", 1);
        g.far_persist_frames = parse_int(params_json, "far_persist_frames", 5);
        g.far_max_range      = parse_double(params_json, "far_max_range", 30.0);
        g.far_conf_decay     = parse_double(params_json, "far_conf_decay", 0.85);
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
             "merge=%.2fm hz=%d max_age=%dms tracking=%s(assoc=%.1fm "
             "max_missed=%d ema=%.2f) far=%s(persist=%d max_range=%.0fm "
             "conf_decay=%.2f)",
             g.input_a_topic, g.input_b_topic, g.output_topic,
             g.merge_dist, g.publish_hz, g.max_age_ms,
             g.tracking_enabled ? "on" : "off",
             g.track_assoc_dist, g.track_max_missed, g.track_ema_alpha,
             g.far_enabled ? "on" : "off",
             g.far_persist_frames, g.far_max_range, g.far_conf_decay);
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
    LOG_INFO("perception_fusion", "started");
    node_announce_self(g.transport, &s_plugin);
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
    LOG_INFO("perception_fusion", "cleanup: in_a=%lu in_b=%lu out=%lu merges=%lu "
             "trk_created=%lu trk_matched=%lu trk_killed=%lu",
             (unsigned long)g.frames_in_a, (unsigned long)g.frames_in_b,
             (unsigned long)g.frames_out, (unsigned long)g.merges_done,
             (unsigned long)g.tracks_created, (unsigned long)g.tracks_matched,
             (unsigned long)g.tracks_killed);
}

/* ── 插件导出 ─────────────────────────────────────────────── */

static NodePlugin s_plugin = {
    .api_version   = NODE_PLUGIN_API_VERSION,
    .name          = "perception_fusion",
    .version       = "1.0.0",
    .description   = "Perception fusion + tracking (LiDAR+Stereo merge → NN association → persistent IDs + vx/vy)",
    .input_topics  = s_inputs,
    .output_topics = s_outputs,
    .init          = fusion_init,
    .start         = fusion_start,
    .stop          = fusion_stop,
    .cleanup       = fusion_cleanup,
};
