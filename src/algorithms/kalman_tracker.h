/**
 * kalman_tracker.h — Multi-Object Kalman Filter Tracker
 *
 * Each track maintains a 4D Kalman filter: [x, y, vx, vy]
 * with a constant velocity (CV) motion model.
 *
 * Data association uses the Hungarian algorithm to find the optimal
 * assignment between detections and existing tracks.
 *
 * Track lifecycle:
 *   TENTATIVE  →  (N consecutive matches)  →  CONFIRMED
 *   CONFIRMED  →  (M consecutive misses)   →  COASTING
 *   COASTING   →  (K more misses)           →  DELETED
 *   Any state  →  (match)                   →  CONFIRMED
 *
 * Usage:
 *   KalmanTracker kt;
 *   ktracker_init(&kt, dt);
 *   // Each frame:
 *   ktracker_predict(&kt);
 *   ktracker_associate_and_update(&kt, detections, n_dets);
 *   // Read confirmed tracks:
 *   for (int i = 0; i < kt.n_tracks; i++) {
 *       if (kt.tracks[i].state == TRACK_CONFIRMED) { ... }
 *   }
 *
 * Reference:
 *   Weng, et al. "AB3DMOT: A 3D Multi-Object Tracking Baseline."
 *   Kalman, R.E. "A New Approach to Linear Filtering and Prediction Problems." (1960)
 *   Kuhn, H.W. "The Hungarian Method for the Assignment Problem." (1955)
 */

#ifndef KALMAN_TRACKER_H
#define KALMAN_TRACKER_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── 常量 ────────────────────────────────────────────────────── */
#define KTRACKER_MAX_TRACKS   32      /**< 最大跟踪目标数 */
#define KTRACKER_MAX_DETS     32      /**< 每帧最大检测数 */
#define KTRACKER_STATE_DIM    4       /**< [x, y, vx, vy] */

/* ── 跟踪状态 ────────────────────────────────────────────────── */
typedef enum {
    TRACK_TENTATIVE = 0,  /**< 新建，待确认 */
    TRACK_CONFIRMED = 1,  /**< 已确认（连续匹配 N 帧） */
    TRACK_COASTING  = 2,  /**< 正在丢失（连续未匹配 M 帧） */
} TrackState;

/* ── 检测 (来自感知) ──────────────────────────────────────────── */
typedef struct {
    float x, y;           /**< 位置 (m, 自车坐标系) */
    float vx, vy;         /**< 速度 (m/s), 或 0 表示未知 */
    float width, length;  /**< 包围盒 */
    int   cls;            /**< 类型: 0=unknown, 1=vehicle, 2=pedestrian, 3=cyclist */
    float confidence;     /**< 检测置信度 [0, 1] */
} KTrackDetection;

/* ── 单条跟踪轨迹 ────────────────────────────────────────────── */
typedef struct {
    int     id;                 /**< 全局唯一 ID */
    TrackState state;           /**< 状态 */
    double  x[KTRACKER_STATE_DIM]; /**< 状态均值: [x, y, vx, vy] */
    double  P[KTRACKER_STATE_DIM * KTRACKER_STATE_DIM]; /**< 协方差 4×4 */

    int     age;                /**< 存活帧数 */
    int     hits;               /**< 累计匹配次数 */
    int     misses;             /**< 连续未匹配次数 */
    int     hits_streak;        /**< 连续匹配次数 */

    float   width, length;      /**< 包围盒 (从最近一次检测复制) */
    int     cls;                /**< 类型 */
    float   confidence;         /**< 平均置信度 */
} KTrack;

/* ── 跟踪器实例 ──────────────────────────────────────────────── */
typedef struct {
    double  dt;                 /**< 时间步长 (s) */
    KTrack  tracks[KTRACKER_MAX_TRACKS];
    int     n_tracks;
    int     next_id;

    /* 参数 */
    int     tent_hits_to_confirm;   /**< 连续命中 N 次确认 (default 3) */
    int     max_misses_before_delete; /**< 连续未命中 M 次删除 (default 10) */
    float   max_assoc_dist;         /**< 最大关联距离 (m), Mahalanobis 门限 */
    float   max_assoc_dist_init;    /**< 新生轨迹关联距离 (m), 更宽松 */

    /* 过程/测量噪声 */
    double  q_pos;              /**< 位置过程噪声方差 */
    double  q_vel;              /**< 速度过程噪声方差 */
    double  r_pos;              /**< 位置测量噪声方差 */
} KalmanTracker;

/* ── API ────────────────────────────────────────────────────── */

/**
 * 初始化跟踪器。
 * @param kt   实例
 * @param dt   预测步长 (s)
 */
void ktracker_init(KalmanTracker* kt, double dt);

/**
 * 预测步: 所有轨迹向前推进一步。
 * 每个传感器帧调用一次（在关联更新之前）。
 */
void ktracker_predict(KalmanTracker* kt);

/**
 * 关联检测并更新轨迹。
 * 执行 Hungarian 匹配 → Kalman 更新 → 轨迹生命周期管理。
 *
 * @param kt     实例
 * @param dets   检测列表
 * @param n_dets 检测数量
 */
void ktracker_associate_and_update(KalmanTracker* kt,
                                    const KTrackDetection* dets, int n_dets);

/**
 * 获取已确认的轨迹数量。
 */
int ktracker_confirmed_count(const KalmanTracker* kt);

/**
 * 获取全部轨迹数量（包括 TENTATIVE/COASTING）。
 */
int ktracker_total_count(const KalmanTracker* kt);

/**
 * 删除所有轨迹（重置）。
 */
void ktracker_reset(KalmanTracker* kt);

#ifdef __cplusplus
}
#endif

#endif /* KALMAN_TRACKER_H */
