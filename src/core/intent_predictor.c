/**
 * intent_predictor.c — 意图预测 + 交互预测引擎实现
 *
 * 算法流程：
 *   1. 历史更新：环形缓冲维护每个目标最近 8 帧历史
 *   2. 特征提取：从历史轨迹提取横/纵向速度变化、车道位置偏差、航向角偏差
 *   3. 意图分类：基于特征向量计算每种意图的似然得分
 *   4. 交互修正：考虑 ego 未来轨迹对目标意图的影响
 *   5. 轨迹生成：为每种高概率意图生成物理可行的轨迹
 */

#include "intent_predictor.h"
#include <string.h>
#include <math.h>
#include <stdio.h>

/* ── Helpers ────────────────────────────────────────────────── */

static inline double clamp(double v, double lo, double hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

static inline double sigmoid(double x) {
    return 1.0 / (1.0 + exp(-x));
}

static inline double sqr(double x) { return x * x; }

/* 归一化角度差到 [-π, π] */
static double normalize_angle(double a) {
    while (a >  M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

/* ── History management ─────────────────────────────────────── */

static void history_push(IntentHistory* h, const IntentObject* obj) {
    h->history[h->head] = *obj;
    h->head = (h->head + 1) % INTENT_HISTORY_LEN;
    if (h->count < INTENT_HISTORY_LEN) h->count++;
}

static const IntentObject* history_get(const IntentHistory* h, int i) {
    if (i < 0 || i >= h->count) return NULL;
    int idx = (h->head - 1 - i + INTENT_HISTORY_LEN) % INTENT_HISTORY_LEN;
    return &h->history[idx];
}

/* ── Feature extraction ─────────────────────────────────────── */

typedef struct {
    double avg_vx, avg_vy;       /**< 平均速度 */
    double avg_ax, avg_ay;       /**< 平均加速度 */
    double lateral_speed;         /**< 横向速度 (绝对值) */
    double lateral_accel;         /**< 横向加速度 (绝对值) */
    double heading_change;        /**< 航向角变化 (历史帧间) */
    double lane_offset;           /**< 相对车道中心的偏移 */
    double lane_offset_rate;      /**< 车道偏移变化率 */
    double speed_ratio;           /**< 当前速度/历史平均速度 */
    double time_to_lane_edge;     /**< 按当前横向速度到达车道边缘的时间 */
    double heading_to_road;       /**< 航向角与道路方向的偏差 */
    int    history_len;           /**< 有效历史帧数 */
} IntentFeatures;

static void extract_features(const IntentHistory* h,
                             double lane_center_y, double lane_width,
                             IntentFeatures* f) {
    memset(f, 0, sizeof(*f));
    if (h->count < 2) { f->history_len = h->count; return; }

    const IntentObject* cur = history_get(h, 0);
    if (!cur) return;

    f->history_len = h->count;

    /* 速度统计 */
    double sum_vx = 0, sum_vy = 0;
    for (int i = 0; i < h->count; i++) {
        const IntentObject* o = history_get(h, i);
        if (!o) continue;
        sum_vx += o->vx;
        sum_vy += o->vy;
    }
    f->avg_vx = sum_vx / h->count;
    f->avg_vy = sum_vy / h->count;

    /* 加速度: 从速度差分估算 */
    const IntentObject* prev = history_get(h, 1);
    if (prev) {
        f->avg_ax = (cur->vx - prev->vx) / INTENT_WAYPOINT_DT_S;
        f->avg_ay = (cur->vy - prev->vy) / INTENT_WAYPOINT_DT_S;
    }

    /* 横向特征 */
    f->lateral_speed = fabs(cur->vy);
    f->lateral_accel = fabs(f->avg_ay);

    /* 航向角变化 */
    if (prev) {
        f->heading_change = normalize_angle(cur->heading - prev->heading);
    }

    /* 车道偏移 */
    f->lane_offset = cur->y - lane_center_y;
    if (prev) {
        f->lane_offset_rate = (f->lane_offset - (prev->y - lane_center_y)) / INTENT_WAYPOINT_DT_S;
    }

    /* 速度比 */
    if (fabs(f->avg_vx) > 0.1) {
        f->speed_ratio = cur->vx / f->avg_vx;
    } else {
        f->speed_ratio = 1.0;
    }

    /* 到达车道边缘时间 */
    if (f->lateral_speed > 0.1) {
        double half_lane = lane_width * 0.5;
        f->time_to_lane_edge = fabs(half_lane - fabs(f->lane_offset)) / f->lateral_speed;
    } else {
        f->time_to_lane_edge = 999.0;
    }

    /* 航向与道路方向偏差 */
    f->heading_to_road = fabs(f->heading_change);
}

/* ── Intent classifier ──────────────────────────────────────── */

/**
 * 计算每种意图的得分。
 * 得分越高表示该意图越可能。
 *
 * 特征权重基于驾驶行为先验知识：
 * - Lane Keep: 横向速度小 + 车道偏移小 + 航向角变化小
 * - Lane Change L/R: 横向速度大 + 向目标方向移动 + 靠近车道边缘
 * - Stop: 速度递减 + 减速度大
 * - Accel: 加速 + 速度比 > 1
 * - Decel: 减速 + 速度比 < 1
 */
static void compute_intent_scores(const IntentFeatures* f,
                                  double lane_width,
                                  double* scores) {
    /* 初始化所有得分为 0 */
    memset(scores, 0, sizeof(double) * 7);

    if (f->history_len < 2) {
        scores[INTENT_UNKNOWN] = 1.0;
        return;
    }

    const double half_lane = lane_width * 0.5;

    /* ── Lane Keep 得分 ── */
    double lk_score = 0.0;
    /* 横向速度小 → 高得分 */
    lk_score += 1.0 - clamp(f->lateral_speed / 2.0, 0.0, 1.0);
    /* 车道偏移小 → 高得分 */
    lk_score += 1.0 - clamp(fabs(f->lane_offset) / half_lane, 0.0, 1.0);
    /* 航向角变化小 → 高得分 */
    lk_score += 1.0 - clamp(fabs(f->heading_change) / 0.2, 0.0, 1.0);
    /* 车道偏移变化率小 → 高得分 */
    lk_score += 1.0 - clamp(fabs(f->lane_offset_rate) / 1.0, 0.0, 1.0);
    scores[INTENT_LANE_KEEP] = lk_score / 4.0;

    /* ── Lane Change 得分 ── */
    /* 左变道: 横向速度为正 (y增加) + 车道偏移为负 (在中心线左侧) */
    double lc_left = 0.0;
    if (f->lateral_speed > 0.1) {
        lc_left += clamp(f->lateral_speed / 2.0, 0.0, 1.0);
    }
    /* 靠近左车道边缘 */
    if (f->lane_offset < -half_lane * 0.3) {
        lc_left += clamp(fabs(f->lane_offset) / half_lane, 0.0, 1.0) * 0.5;
    }
    /* 横向速度方向与车道偏移符号一致 (正在离开当前车道) */
    if (f->lane_offset_rate * f->lateral_speed > 0) {
        lc_left += 0.5;
    }
    scores[INTENT_LANE_CHANGE_L] = lc_left / 3.0;

    /* 右变道: 横向速度为负 + 车道偏移为正 */
    double lc_right = 0.0;
    if (f->lateral_speed > 0.1) {
        lc_right += clamp(f->lateral_speed / 2.0, 0.0, 1.0);
    }
    if (f->lane_offset > half_lane * 0.3) {
        lc_right += clamp(fabs(f->lane_offset) / half_lane, 0.0, 1.0) * 0.5;
    }
    if (f->lane_offset_rate * f->avg_vy < 0) {
        lc_right += 0.5;
    }
    scores[INTENT_LANE_CHANGE_R] = lc_right / 3.0;

    /* ── Stop 得分 ── */
    double stop_score = 0.0;
    /* 速度递减 */
    if (f->avg_ax < -0.5) stop_score += clamp(-f->avg_ax / 5.0, 0.0, 1.0);
    /* 速度比 < 1 (正在减速) */
    if (f->speed_ratio < 0.9) stop_score += 1.0 - f->speed_ratio;
    /* 低速 */
    if (fabs(f->avg_vx) < 3.0) stop_score += 1.0 - fabs(f->avg_vx) / 3.0;
    scores[INTENT_STOP] = stop_score / 3.0;

    /* ── Accel 得分 ── */
    double accel_score = 0.0;
    if (f->avg_ax > 0.5) accel_score += clamp(f->avg_ax / 3.0, 0.0, 1.0);
    if (f->speed_ratio > 1.05) accel_score += clamp((f->speed_ratio - 1.0) * 10.0, 0.0, 1.0);
    scores[INTENT_ACCEL] = accel_score / 2.0;

    /* ── Decel 得分 ── */
    double decel_score = 0.0;
    if (f->avg_ax < -0.5) decel_score += clamp(-f->avg_ax / 3.0, 0.0, 1.0);
    if (f->speed_ratio < 0.95) decel_score += clamp((1.0 - f->speed_ratio) * 10.0, 0.0, 1.0);
    scores[INTENT_DECEL] = decel_score / 2.0;
}

/**
 * 对得分做 softmax 归一化得到概率。
 */
static void scores_to_prob(double* scores, int n) {
    double max_score = scores[0];
    for (int i = 1; i < n; i++) {
        if (scores[i] > max_score) max_score = scores[i];
    }
    double sum = 0.0;
    for (int i = 0; i < n; i++) {
        scores[i] = exp(scores[i] - max_score);
        sum += scores[i];
    }
    if (sum > 1e-12) {
        for (int i = 0; i < n; i++) scores[i] /= sum;
    }
}

/* ── Interaction prediction ─────────────────────────────────── */

/**
 * 交互预测修正：ego 的轨迹影响目标意图。
 *
 * 关键交互：
 *   - ego 在目标前方加速 → 目标减速概率增加
 *   - ego 从后方逼近目标 → 目标可能加速或变道避让
 *   - ego 和目标在同一车道 → 目标可能变道
 */
static void apply_interaction(const IntentEgoState* ego,
                              const IntentObject* obj,
                              const IntentFeatures* f,
                              double lane_width,
                              double* scores) {
    double dx = obj->x - ego->x;  /* 正=目标在前方 */
    double dy = obj->y - ego->y;
    double half_lane = lane_width * 0.5;

    /* 同车道判断 */
    int same_lane = (fabs(dy) < half_lane);

    /* 前方近距离 (< 50m) */
    double distance = sqrt(dx * dx + dy * dy);
    int close_range = (distance < 50.0);

    if (!close_range) return;  /* 远距离无交互 */

    if (dx > 0 && same_lane) {
        /* 目标在 ego 前方同车道 → ego 逼近 */
        double closing_speed = ego->speed - obj->vx;
        double ttc = (closing_speed > 0.5) ? dx / closing_speed : 999.0;

        if (ttc < 5.0) {
            /* 快速逼近 → 目标可能加速 或 变道避让 */
            double urgency = 1.0 - clamp(ttc / 5.0, 0.0, 1.0);
            scores[INTENT_ACCEL] += urgency * 0.3;
            scores[INTENT_LANE_CHANGE_L] += urgency * 0.15;
            scores[INTENT_LANE_CHANGE_R] += urgency * 0.15;
        }
    } else if (dx < 0 && same_lane) {
        /* 目标在 ego 后方同车道 → ego 在前方加速 */
        if (ego->speed > obj->vx + 1.0) {
            /* ego 快速拉开距离 → 目标无压力 */
            scores[INTENT_LANE_KEEP] += 0.2;
        }
    } else if (dx < 0 && fabs(dy) < half_lane * 2.0) {
        /* 目标在 ego 后方相邻车道 → ego 可能变道到目标车道 */
        double ego_lateral = ego->vy;
        if (fabs(ego_lateral) > 0.1) {
            /* ego 横向移动 → 目标可能减速让行 */
            scores[INTENT_DECEL] += 0.2;
        }
    }

    /* 重归一化 */
    double sum = 0.0;
    for (int i = 0; i < 7; i++) sum += scores[i];
    if (sum > 1e-12) {
        for (int i = 0; i < 7; i++) scores[i] /= sum;
    }
}

/* ── Trajectory generation ──────────────────────────────────── */

/**
 * 为指定意图生成轨迹。
 * 考虑物理约束（加减速限制、最大转向角）。
 */
static void generate_trajectory(const IntentObject* obj,
                                IntentMode intent,
                                IntentTrajectory* traj) {
    memset(traj, 0, sizeof(*traj));
    traj->intent = intent;

    double x = obj->x, y = obj->y;
    double vx = obj->vx, vy = obj->vy;
    const double dt = INTENT_WAYPOINT_DT_S;
    const double max_accel = 3.0, max_decel = 5.0;
    const double max_lat_speed = 2.0; /* 最大横向速度 (m/s) */

    traj->count = INTENT_MAX_WAYPOINTS;

    for (int step = 0; step < INTENT_MAX_WAYPOINTS; step++) {
        double ax = 0.0, ay = 0.0;

        switch (intent) {
        case INTENT_LANE_KEEP:
            /* 保持当前速度，横向归零 */
            ax = -vy * 0.3;  /* 阻尼横向速度 */
            ay = -vy * 0.5;  /* 横向归零 */
            break;

        case INTENT_LANE_CHANGE_L:
            /* 向左加速横向 */
            ax = 0.0;
            ay = clamp(2.0 - vy, -max_lat_speed, max_lat_speed) * 0.3;
            break;

        case INTENT_LANE_CHANGE_R:
            /* 向右加速横向 */
            ax = 0.0;
            ay = clamp(-2.0 - vy, -max_lat_speed, max_lat_speed) * 0.3;
            break;

        case INTENT_STOP:
            /* 减速到 0 */
            ax = clamp(-vx / dt, -max_decel, 0.0);
            ay = clamp(-vy / dt, -max_lat_speed, max_lat_speed);
            break;

        case INTENT_ACCEL:
            /* 加速 */
            ax = clamp(2.0, 0.0, max_accel);
            ay = clamp(-vy * 0.3, -max_lat_speed, max_lat_speed);
            break;

        case INTENT_DECEL:
            /* 减速 */
            ax = clamp(-2.0, -max_decel, 0.0);
            ay = clamp(-vy * 0.5, -max_lat_speed, max_lat_speed);
            break;

        default:
            break;
        }

        vx += ax * dt;
        vy += ay * dt;
        x  += vx * dt;
        y  += vy * dt;

        traj->waypoints[step][0] = x;
        traj->waypoints[step][1] = y;
        traj->waypoints[step][2] = sqrt(vx * vx + vy * vy);
    }
}

/* ── Public API ──────────────────────────────────────────────── */

void intent_predictor_init(IntentPredictor* pred) {
    memset(pred, 0, sizeof(*pred));
    pred->lane_width = 3.5;
    pred->lane_center_y = 0.0;
    pred->lane_count = 2;
}

void intent_predictor_set_lane(IntentPredictor* pred,
                               double lane_width,
                               double lane_center_y,
                               int lane_count) {
    pred->lane_width = lane_width;
    pred->lane_center_y = lane_center_y;
    pred->lane_count = lane_count;
}

void intent_predictor_feed(IntentPredictor* pred,
                           const IntentEgoState* ego,
                           const IntentObject* objects,
                           int n) {
    if (!pred || !objects || n <= 0) return;

    if (ego) {
        pred->ego = *ego;
    }

    if (n > INTENT_MAX_OBJECTS) n = INTENT_MAX_OBJECTS;

    /* 更新历史：按 object_id 匹配 */
    for (int i = 0; i < n; i++) {
        const IntentObject* obj = &objects[i];
        int slot = -1;

        /* 查找匹配的历史槽位 */
        for (int j = 0; j < pred->history_count; j++) {
            if (pred->histories[j].count > 0) {
                const IntentObject* h = history_get(&pred->histories[j], 0);
                if (h && h->x == obj->x && h->y == obj->y) {
                    slot = j;
                    break;
                }
            }
        }

        /* 未找到匹配 → 新槽位 */
        if (slot < 0 && pred->history_count < INTENT_MAX_OBJECTS) {
            slot = pred->history_count++;
        }

        if (slot >= 0) {
            history_push(&pred->histories[slot], obj);
        }
    }
}

void intent_predictor_predict(IntentPredictor* pred,
                              IntentPredictionList* out) {
    memset(out, 0, sizeof(*out));

    for (int i = 0; i < pred->history_count && i < INTENT_MAX_OBJECTS; i++) {
        IntentHistory* h = &pred->histories[i];
        if (h->count < 2) continue;

        const IntentObject* cur = history_get(h, 0);
        if (!cur || cur->is_static) continue;

        IntentFeatures f;
        extract_features(h, pred->lane_center_y, pred->lane_width, &f);

        /* 计算意图得分 */
        double scores[7];
        compute_intent_scores(&f, pred->lane_width, scores);

        /* 交互修正 */
        if (pred->ego.speed > 0.5) {
            apply_interaction(&pred->ego, cur, &f, pred->lane_width, scores);
        }

        /* 只保留概率最高的 3 个意图 */
        /* 找 top 3 */
        typedef struct { double prob; IntentMode mode; } Ranked;
        Ranked ranked[7];
        for (int m = 0; m < 7; m++) {
            ranked[m].prob = scores[m];
            ranked[m].mode = (IntentMode)m;
        }
        /* 简单选择排序 top 3 */
        for (int a = 0; a < 3; a++) {
            int best = a;
            for (int b = a + 1; b < 7; b++) {
                if (ranked[b].prob > ranked[best].prob) best = b;
            }
            if (best != a) {
                Ranked tmp = ranked[a];
                ranked[a] = ranked[best];
                ranked[best] = tmp;
            }
        }

        /* 归一化 top 3 */
        double sum = ranked[0].prob + ranked[1].prob + ranked[2].prob;
        if (sum < 1e-12) {
            ranked[0].prob = 1.0;
            ranked[0].mode = INTENT_LANE_KEEP;
            sum = 1.0;
        }

        IntentPrediction* pred_out = &out->predictions[out->count];
        pred_out->object_id = (uint32_t)(i + 1); /* 使用历史槽位 ID */
        pred_out->confidence = clamp(f.history_len / (double)INTENT_HISTORY_LEN, 0.3, 1.0);
        pred_out->horizon_s = INTENT_PRED_HORIZON_S;
        pred_out->primary_intent = ranked[0].mode;
        pred_out->trajectory_count = 0;

        for (int t = 0; t < 3 && ranked[t].prob > 0.02; t++) {
            IntentTrajectory* traj = &pred_out->trajectories[pred_out->trajectory_count];
            generate_trajectory(cur, ranked[t].mode, traj);
            traj->probability = ranked[t].prob / sum;
            pred_out->trajectory_count++;
        }

        out->count++;
    }
}