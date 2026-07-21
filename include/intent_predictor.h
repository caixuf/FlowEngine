#ifndef INTENT_PREDICTOR_H
#define INTENT_PREDICTOR_H

/**
 * @file intent_predictor.h
 * @brief 基于历史轨迹的意图预测 + 交互预测引擎
 *
 * 相比原始 prediction_node 的纯规则 CV/LK/LC 三模式：
 *   1. 意图分类：基于横纵向速度、相对车道位置、航向角偏差推断意图
 *   2. 置信度：从固定 0.7/0.2/0.1 改为基于特征的成本函数
 *   3. 交互预测：考虑 ego 未来轨迹对目标车辆的影响
 *       - 若 ego 在目标前方加速 → 目标可能减速
 *       - 若 ego 在目标后方逼近 → 目标可能加速
 *       - 若 ego 和目标在同车道 → 目标可能变道避让
 *   4. 轨迹生成：考虑加减速物理约束（非恒定速度）
 *
 * 整体架构：
 *   IntentPredictor → IntentFeatures → IntentClassifier → TrajectoryGenerator
 *
 * 用法：
 *   IntentPredictor pred;
 *   intent_predictor_init(&pred);
 *   intent_predictor_feed(&pred, &ego, objects, n);
 *   intent_predictor_predict(&pred, &output);
 */

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Constants ──────────────────────────────────────────────── */

#define INTENT_MAX_OBJECTS      32
#define INTENT_MAX_TRAJECTORIES 3     /**< 每个目标最多 3 条轨迹 */
#define INTENT_MAX_WAYPOINTS    10    /**< 每条轨迹最多 10 个航点 */
#define INTENT_HISTORY_LEN      8     /**< 历史轨迹长度 */
#define INTENT_PRED_HORIZON_S   5.0   /**< 预测时长 (s) */
#define INTENT_WAYPOINT_DT_S    0.5   /**< 航点间隔 (s) */

/* ── Enums ──────────────────────────────────────────────────── */

typedef enum {
    INTENT_LANE_KEEP     = 0,  /**< 保持当前车道 */
    INTENT_LANE_CHANGE_L = 1,  /**< 向左变道 */
    INTENT_LANE_CHANGE_R = 2,  /**< 向右变道 */
    INTENT_STOP          = 3,  /**< 停车 */
    INTENT_ACCEL         = 4,  /**< 加速 */
    INTENT_DECEL         = 5,  /**< 减速 */
    INTENT_UNKNOWN       = 6,
} IntentMode;

/* ── Types ──────────────────────────────────────────────────── */

/** 观测到的目标状态（一帧） */
typedef struct {
    double x, y;           /**< 世界坐标 (m) */
    double vx, vy;         /**< 速度 (m/s) */
    double ax, ay;         /**< 加速度 (m/s²) */
    double heading;        /**< 航向角 (rad) */
    double width, length;  /**< 尺寸 (m) */
    int32_t lane_id;       /**< 车道 ID (-1=未知) */
    bool    is_static;     /**< 是否静止 */
    int     type;          /**< 类型: 0=unknown, 1=vehicle, 2=pedestrian, 3=cyclist */
} IntentObject;

/** 历史轨迹缓冲区 */
typedef struct {
    IntentObject history[INTENT_HISTORY_LEN];
    int          count;        /**< 有效历史帧数 */
    int          head;         /**< 环形缓冲写入位置 */
} IntentHistory;

/** 自车状态 */
typedef struct {
    double x, y, vx, vy;
    double heading, speed;
    double lane_y;             /**< ego 所在车道中心 y */
    double lane_width;
    double predicted_path[INTENT_MAX_WAYPOINTS][3]; /**< ego 未来路径 [x,y,v] */
    int    path_len;
} IntentEgoState;

/** 预测轨迹 */
typedef struct {
    double waypoints[INTENT_MAX_WAYPOINTS][3]; /**< [step][x, y, v] */
    int    count;
    double probability;       /**< 该轨迹概率 [0, 1] */
    IntentMode intent;        /**< 对应意图 */
} IntentTrajectory;

/** 单个目标的预测结果 */
typedef struct {
    uint32_t object_id;
    double   confidence;      /**< 整体预测置信度 */
    double   horizon_s;
    IntentTrajectory trajectories[INTENT_MAX_TRAJECTORIES];
    int      trajectory_count;
    IntentMode primary_intent; /**< 最可能的意图 */
} IntentPrediction;

/** 预测输出 */
typedef struct {
    IntentPrediction predictions[INTENT_MAX_OBJECTS];
    int              count;
    uint32_t         frame_id;
    uint64_t         timestamp_us;
} IntentPredictionList;

/* ── Engine ─────────────────────────────────────────────────── */

typedef struct {
    IntentHistory  histories[INTENT_MAX_OBJECTS];
    int            history_count;
    IntentEgoState ego;
    double         lane_width;
    double         lane_center_y;  /**< 道路中心线 y 坐标 */
    int            lane_count;
} IntentPredictor;

/* ── API ────────────────────────────────────────────────────── */

/**
 * 初始化意图预测器。
 */
void intent_predictor_init(IntentPredictor* pred);

/**
 * 喂入 ego 状态和观测目标列表。
 * @param objects  观测目标数组
 * @param n        目标数量
 */
void intent_predictor_feed(IntentPredictor* pred,
                           const IntentEgoState* ego,
                           const IntentObject* objects,
                           int n);

/**
 * 执行预测，输出所有目标的预测轨迹。
 * @param out  输出预测列表
 */
void intent_predictor_predict(IntentPredictor* pred,
                              IntentPredictionList* out);

/**
 * 设置车道参数（用于意图分类）。
 */
void intent_predictor_set_lane(IntentPredictor* pred,
                               double lane_width,
                               double lane_center_y,
                               int lane_count);

#ifdef __cplusplus
}
#endif

#endif /* INTENT_PREDICTOR_H */