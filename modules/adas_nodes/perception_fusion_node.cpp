/**
 * perception_fusion_node.cpp — 感知融合节点 (FlowCoro 协程版)
 *
 * 从 perception_fusion_node.c 迁移而来，参照 fusion_node.cpp 范本：
 *   - CoroutineTask 替代 pthread_create(fusion_thread) + 手写 usleep 轮询
 *   - MessageBuffer + message_buffer_latest 替代手写 a_list/b_list + mutex + max_age
 *   - co_await select_for({a,b}, period_us) 替代 usleep(period_us) 定时轮询
 *   - cJSON 替代手写 parse_double/parse_int/parse_string（CLAUDE.md 规范唯一合法入口）
 *
 * 保留不动：fuse_obstacles() / associate_and_track() 业务算法（与执行模型无关）。
 *
 * select_for 超时语义 vs 旧 usleep 等价性（迁移风险点，已核对）：
 *   旧 usleep(period) 是固定 period 节奏；新 select_for(period) 是"有数据即醒、
 *   无数据 period 超时醒"。对 perception_fusion 更合理：有数据时融合延迟最低，
 *   无数据时每 period 超时醒一次（与旧 usleep 等价）。不做额外节流，与范本
 *   fusion_node.cpp 一致。
 *
 * 链路:
 *   lidar_driver   ──▶ perception/obstacles_lidar  ─┐
 *   stereo_vision  ──▶ perception/obstacles_stereo ─┼──▶ [本节点] ──▶ perception/obstacles
 *                                                    │              (持久ID + vx/vy)
 * 算法:
 *   Stage 1 — 空间去重合并（fuse_obstacles）: 两路 ObstacleList → 按距离去重合并
 *   Stage 2 — 跨帧时序追踪（associate_and_track）: 最近邻关联 → 持久 ID + 差分速度
 *
 * 编译依赖: adas_msgs_gen.h (ObstacleList 反序列化/序列化)，随构建生成。
 */

#include "node_plugin.h"
#include "adas_msgs_gen.h"
#include "transport.h"
#include "discovery.h"
#include "coroutine_task.h"
#include "fusion.h"        /* MessageBuffer: 替代手写 a_list/b_list + mutex */
#include "clock_service.h"
#undef LOG_TRACE
#undef LOG_DEBUG
#undef LOG_INFO
#undef LOG_WARN
#undef LOG_ERROR
#undef LOG_FATAL
#include "logger.h"
#include <cjson/cJSON.h>

#include <errno.h>
#include <math.h>
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <atomic>
#include <memory>

namespace {

/* ── Phase 2: 跨帧追踪 track（原样保留） ─────────────────── */
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
    int      extrapolated;    /* 是否处于外推模式(超出传感器覆盖) */
    int      extrap_frames;   /* 连续外推帧数 */
} Track;

/* ── 节点状态 ─────────────────────────────────────────────── */

struct PerceptionFusionContext {
    Transport*        transport{nullptr};
    DiscoveryManager* discovery{nullptr};

    /* 协程宿主线程 */
    pthread_t         thread{};
    bool              running{false};
    std::atomic<bool> should_stop{false};

    /* 两路输入 topic 名（可配） */
    char input_a_topic[64];   /* 默认 perception/obstacles_lidar */
    char input_b_topic[64];   /* 默认 perception/obstacles_stereo */
    char output_topic[64];    /* 默认 perception/obstacles */

    /* 融合参数 */
    double merge_dist{1.0};        /* 去重距离阈值 (m) */
    int    publish_hz{10};         /* 发布频率 */
    int    max_age_ms{500};        /* 输入最大新鲜期 (ms)，超期视为失效 */
    int    enabled{1};

    /* 时序追踪参数 */
    double track_assoc_dist{3.0};  /* 最近邻关联距离阈值 (m) */
    int    track_max_missed{3};    /* 连续未匹配帧数上限 */
    double track_ema_alpha{0.3};   /* 速度 EMA 平滑因子 (0-1) */
    int    tracking_enabled{1};

    /* 远距外推参数 */
    int    far_enabled{1};
    int    far_persist_frames{5};
    double far_max_range{30.0};
    double far_conf_decay{0.85};

    /* Track 池 */
    Track  tracks[PF_MAX_TRACKS];
    int    track_count{0};
    int    next_track_id{1};       /* ID 从 1 开始,0 保留 */

    /* 两路输入缓冲：复用 core/fusion.c 的 MessageBuffer（与 fusion_node.cpp 一致） */
    MessageBuffer* a_buf{nullptr};
    MessageBuffer* b_buf{nullptr};

    /* 统计 */
    uint64_t frames_in_a{0};
    uint64_t frames_in_b{0};
    uint64_t frames_out{0};
    uint64_t merges_done{0};
    uint64_t tracks_created{0};
    uint64_t tracks_matched{0};
    uint64_t tracks_killed{0};

    /* 协程任务 */
    std::unique_ptr<class PerceptionFusionTask> task;
};

PerceptionFusionContext g;

/* ── 订阅回调：仅 push 到 MessageBuffer（与 fusion_node.cpp 一致） ──
 * 反序列化推迟到协程体做，避免在 transport 回调线程里做重活。 */

static void on_input_a(const Message* msg, void* user_data) {
    (void)user_data;
    if (!msg || msg->data_size == 0 || !g.enabled) return;
    if (message_buffer_push(g.a_buf, msg) == 0) g.frames_in_a++;
}

static void on_input_b(const Message* msg, void* user_data) {
    (void)user_data;
    if (!msg || msg->data_size == 0 || !g.enabled) return;
    if (message_buffer_push(g.b_buf, msg) == 0) g.frames_in_b++;
}

/* ── 融合核心：两路 ObstacleList → 去重合并（原样保留，操作 g.merges_done） ──
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

/* ── 跨帧时序追踪（原样保留，操作 g.tracks/g.track_*） ──────
 *
 * 对 fuse_obstacles 产出的空间去重检测列表做最近邻跨帧关联:
 *   1. 每个检测找最近的活跃 track（Euclidean ≤ assoc_dist）
 *   2. 匹配成功 → 差分测速 + EMA 平滑,更新 track 位置/尺寸
 *   3. 未匹配 detection → 新建 track (vx=vy=0, 首帧无速度)
 *   4. 本轮未关联的 track → missed++, 超过 max_missed 则删除
 *   5. 按 x 排序输出活跃 track → ObstacleList
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

        /* 远距外推：track 离开传感器覆盖但有有效速度时，用常速度模型外推位置，
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
        /* 外推 track 置信度打折,type 标记为 UNKNOWN(远距无法分类) */
        Obstacle o;
        memset(&o, 0, sizeof(o));
        o.id         = (uint32_t)g.tracks[j].id;
        o.x          = (float)g.tracks[j].x;
        o.y          = (float)g.tracks[j].y;
        o.vx         = (float)g.tracks[j].vx;
        o.vy         = (float)g.tracks[j].vy;
        o.width      = (float)g.tracks[j].width;
        o.length     = (float)g.tracks[j].length;
        o.type       = g.tracks[j].extrapolated ? OBJ_TYPE_UNKNOWN
                                                : (ObstacleType)g.tracks[j].type;
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

/* ── 协程任务 ────────────────────────────────────────────── */

class PerceptionFusionTask : public CoroutineTask {
public:
    PerceptionFusionTask(MessageBus* bus, Transport* transport,
                         MessageBuffer* a_buf, MessageBuffer* b_buf)
        : CoroutineTask(bus), transport_(transport), a_buf_(a_buf), b_buf_(b_buf) {}

protected:
    Task run() override {
        LOG_INFO("perception_fusion", "FlowCoro perception fusion started (thread-pool resume)");
        long period_us = 1000000L / (g.publish_hz > 0 ? g.publish_hz : 10);

        while (!should_stop()) {
            /* select_for 替代 usleep(period_us)：等任一输入或 period 超时。
             * 超时作 watchdog（无数据时定期醒来，与旧 usleep 等价）；
             * 有数据即醒，融合延迟比旧 usleep 更低。自动注入 CancelToken，
             * stop() 可立即唤醒。 */
            auto res = co_await select_for(bus(), {g.input_a_topic, g.input_b_topic},
                                           (uint64_t)period_us);
            (void)res;  /* 唤醒即可，数据从 MessageBuffer 读取 */
            if (should_stop() || !g.enabled) break;

            uint64_t now = clock_now_us();
            uint64_t max_age_us = (uint64_t)g.max_age_ms * 1000ULL;

            /* 从 MessageBuffer 取最新一条，按 max_age 判断新鲜度。
             * 反序列化在协程体做（回调只 push，与 fusion_node.cpp 一致）。 */
            ObstacleList a_copy, b_copy;
            int has_a = 0, has_b = 0;

            const Message* a_msg = message_buffer_latest(a_buf_);
            if (a_msg && (now - a_msg->timestamp_us) < max_age_us &&
                ObstacleList_deserialize(&a_copy, a_msg->data, a_msg->data_size) == 0) {
                has_a = 1;
            }

            const Message* b_msg = message_buffer_latest(b_buf_);
            if (b_msg && (now - b_msg->timestamp_us) < max_age_us &&
                ObstacleList_deserialize(&b_copy, b_msg->data, b_msg->data_size) == 0) {
                has_b = 1;
            }

            if (!has_a && !has_b) continue;  /* 两路都无数据/过期，跳过 */

            /* Stage 1: 空间融合（两路去重） */
            ObstacleList fused;
            fuse_obstacles(has_a ? &a_copy : nullptr,
                           has_b ? &b_copy : nullptr,
                           g.merge_dist, &fused);

            /* Stage 2: 跨帧时序追踪 → 持久 ID + 差分速度 */
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
                transport_publish(transport_, g.output_topic, buf, (uint32_t)len);
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

        LOG_INFO("perception_fusion", "FlowCoro perception fusion stopped (out=%lu)",
                 (unsigned long)g.frames_out);
    }

private:
    Transport*     transport_;
    MessageBuffer* a_buf_;
    MessageBuffer* b_buf_;
};

/* ── 协程宿主线程 ─────────────────────────────────────────── */

void* fusion_thread(void*) {
    pthread_setname_np(pthread_self(), "p_fusion");
    try {
        flowcoro::rt::RtExecutor ex{{ .pin_cpu=-1, .idle_sleep_us=200 }};
        g_node_exec = &ex;
        CoroutineTask& ct = *g.task;
        ex.spawn(ct.run(), "p_fusion");
        while (!g.should_stop) ex.run();
        ex.shutdown();
        g_node_exec = nullptr;
    } catch (...) {
        LOG_ERROR("perception_fusion", "FlowCoro task failed");
    }
    return nullptr;
}

/* ── NodePlugin 实现 ─────────────────────────────────────── */

static const char* s_inputs[]  = { "perception/obstacles_lidar",
                                    "perception/obstacles_stereo", nullptr };
static const char* s_outputs[] = { "perception/obstacles", nullptr };

extern NodePlugin s_plugin;  /* 前向声明：定义在文件末尾，供 init/start 引用 */

static int fusion_init(MessageBus* bus, Transport* transport,
                       DiscoveryManager* discovery, Scheduler* scheduler,
                       const char* params_json) {
    (void)scheduler;

    /* 逐字段重置状态（含 std::atomic<bool>/unique_ptr，不可整体赋值，参照
     * fusion_node.cpp 同样的字段式重置）。默认值与结构体默认成员初始化一致。 */
    g.transport = transport;
    g.discovery = discovery;
    g.should_stop = false;
    g.running    = false;
    snprintf(g.input_a_topic, sizeof(g.input_a_topic), "%s", "perception/obstacles_lidar");
    snprintf(g.input_b_topic, sizeof(g.input_b_topic), "%s", "perception/obstacles_stereo");
    snprintf(g.output_topic,  sizeof(g.output_topic),  "%s", "perception/obstacles");
    g.merge_dist        = 1.0;
    g.publish_hz        = 10;
    g.max_age_ms        = 500;
    g.enabled           = 1;
    g.track_assoc_dist  = 3.0;
    g.track_max_missed  = 3;
    g.track_ema_alpha   = 0.3;
    g.tracking_enabled  = 1;
    g.far_enabled       = 1;
    g.far_persist_frames= 5;
    g.far_max_range     = 30.0;
    g.far_conf_decay    = 0.85;
    memset(g.tracks, 0, sizeof(g.tracks));
    g.track_count   = 0;
    g.next_track_id = 1;
    g.frames_in_a = g.frames_in_b = g.frames_out = 0;
    g.merges_done = g.tracks_created = g.tracks_matched = g.tracks_killed = 0;

    /* cJSON 参数解析（替代手写 parse_*，CLAUDE.md 规范唯一合法入口） */
    if (params_json) {
        cJSON* root = cJSON_Parse(params_json);
        if (root) {
            cJSON* j;
            if ((j = cJSON_GetObjectItem(root, "input_a_topic")) && cJSON_IsString(j))
                snprintf(g.input_a_topic, sizeof(g.input_a_topic), "%s", j->valuestring);
            if ((j = cJSON_GetObjectItem(root, "input_b_topic")) && cJSON_IsString(j))
                snprintf(g.input_b_topic, sizeof(g.input_b_topic), "%s", j->valuestring);
            if ((j = cJSON_GetObjectItem(root, "output_topic")) && cJSON_IsString(j))
                snprintf(g.output_topic, sizeof(g.output_topic), "%s", j->valuestring);
            if ((j = cJSON_GetObjectItem(root, "merge_dist")) && cJSON_IsNumber(j))
                g.merge_dist = j->valuedouble;
            if ((j = cJSON_GetObjectItem(root, "publish_hz")) && cJSON_IsNumber(j))
                g.publish_hz = j->valueint;
            if ((j = cJSON_GetObjectItem(root, "max_age_ms")) && cJSON_IsNumber(j))
                g.max_age_ms = j->valueint;
            if ((j = cJSON_GetObjectItem(root, "enable")) && cJSON_IsNumber(j))
                g.enabled = j->valueint;
            if ((j = cJSON_GetObjectItem(root, "track_assoc_dist")) && cJSON_IsNumber(j))
                g.track_assoc_dist = j->valuedouble;
            if ((j = cJSON_GetObjectItem(root, "track_max_missed")) && cJSON_IsNumber(j))
                g.track_max_missed = j->valueint;
            if ((j = cJSON_GetObjectItem(root, "track_ema_alpha")) && cJSON_IsNumber(j))
                g.track_ema_alpha = j->valuedouble;
            if ((j = cJSON_GetObjectItem(root, "tracking_enabled")) && cJSON_IsNumber(j))
                g.tracking_enabled = j->valueint;
            if ((j = cJSON_GetObjectItem(root, "far_enabled")) && cJSON_IsNumber(j))
                g.far_enabled = j->valueint;
            if ((j = cJSON_GetObjectItem(root, "far_persist_frames")) && cJSON_IsNumber(j))
                g.far_persist_frames = j->valueint;
            if ((j = cJSON_GetObjectItem(root, "far_max_range")) && cJSON_IsNumber(j))
                g.far_max_range = j->valuedouble;
            if ((j = cJSON_GetObjectItem(root, "far_conf_decay")) && cJSON_IsNumber(j))
                g.far_conf_decay = j->valuedouble;
            cJSON_Delete(root);
        }
    }

    if (!g.enabled) {
        LOG_INFO("perception_fusion", "disabled by config (enable=0)");
        return 0;
    }

    /* 创建两路 MessageBuffer（capacity=4 足够最新值轮询；window_us 对齐 max_age） */
    g.a_buf = message_buffer_create(g.input_a_topic, OBSTACLELIST_TYPE_ID, 4,
                                    (uint64_t)g.max_age_ms * 1000ULL);
    g.b_buf = message_buffer_create(g.input_b_topic, OBSTACLELIST_TYPE_ID, 4,
                                    (uint64_t)g.max_age_ms * 1000ULL);
    if (!g.a_buf || !g.b_buf) {
        LOG_ERROR("perception_fusion", "message_buffer_create failed (OOM?)");
        return -1;
    }

    /* 订阅两路输入 — 回调仅 push 到 buf，协程 select_for 负责唤醒 */
    transport_subscribe(transport, g.input_a_topic, on_input_a, nullptr);
    transport_subscribe(transport, g.input_b_topic, on_input_b, nullptr);
    discovery_advertise(discovery, g.input_a_topic, OBSTACLELIST_TYPE_ID, CAP_SUBSCRIBER, 0);
    discovery_advertise(discovery, g.input_b_topic, OBSTACLELIST_TYPE_ID, CAP_SUBSCRIBER, 0);

    /* 发布融合结果 */
    discovery_advertise(discovery, g.output_topic, OBSTACLELIST_TYPE_ID, CAP_PUBLISHER,
                        (double)g.publish_hz);

    /* 构造协程任务 */
    g.task = std::make_unique<PerceptionFusionTask>(bus, transport, g.a_buf, g.b_buf);

    LOG_INFO("perception_fusion", "initialized (FlowCoro): a=%s b=%s → out=%s "
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
    g.should_stop = false;
    if (pthread_create(&g.thread, nullptr, fusion_thread, nullptr) != 0) {
        LOG_WARN("perception_fusion", "failed to create thread: %s", strerror(errno));
        return -1;
    }
    g.running = true;
    LOG_INFO("perception_fusion", "started");
    node_announce_self(g.transport, &s_plugin);
    return 0;
}

static void fusion_stop(void) {
    g.should_stop = true;
    if (g.task) g.task->set_stop();  /* 翻 stop flag；select_for 超时醒来查 should_stop */
}

static void fusion_cleanup(void) {
    fusion_stop();
    if (g.running) {
        pthread_join(g.thread, nullptr);
        g.running = false;
    }
    g.task.reset();
    if (g.a_buf) { message_buffer_destroy(g.a_buf); g.a_buf = nullptr; }
    if (g.b_buf) { message_buffer_destroy(g.b_buf); g.b_buf = nullptr; }
    LOG_INFO("perception_fusion", "cleanup: in_a=%lu in_b=%lu out=%lu merges=%lu "
             "trk_created=%lu trk_matched=%lu trk_killed=%lu",
             (unsigned long)g.frames_in_a, (unsigned long)g.frames_in_b,
             (unsigned long)g.frames_out, (unsigned long)g.merges_done,
             (unsigned long)g.tracks_created, (unsigned long)g.tracks_matched,
             (unsigned long)g.tracks_killed);
}

/* ── 导出入口（同 fusion_node.cpp 风格，手工构造 s_plugin） ──
 * s_plugin 定义在匿名命名空间内（与 forward decl / fusion_start 引用同实体），
 * 匿名命名空间对文件作用域隐式 using-directive，故文件尾的 node_get_plugin
 * 可见 s_plugin。 */

NodePlugin s_plugin = {
    NODE_PLUGIN_API_VERSION,
    "perception_fusion",
    "1.0.0",
    "Perception fusion + tracking (LiDAR+Stereo merge → NN association → persistent IDs + vx/vy) [FlowCoro]",
    s_inputs,
    s_outputs,
    fusion_init,
    fusion_start,
    fusion_stop,
    fusion_cleanup,
    nullptr,  /* health_check */
};

} // namespace

extern "C" NodePlugin* node_get_plugin(void) { return &s_plugin; }
