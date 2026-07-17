#ifndef ADAS_MSGS_H
#define ADAS_MSGS_H

/**
 * @file adas_msgs.h
 * @brief ADAS 演示用消息类型定义
 *
 * 话题约定：
 *   sensor/lidar            — LidarFrame     (10 Hz)
 *   sensor/gps              — GpsData        (5  Hz)
 *   perception/obstacles    — ObstacleList   (10 Hz)
 *   perception/ego_state    — EgoState       (10 Hz)
 *   control/cmd             — ControlCmd     (on-demand)
 */

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── 传感器原始数据 ─────────────────────────────────────── */

/** 激光雷达帧 — 发布到 sensor/lidar */
typedef struct {
    float    x, y, z;       /**< 点云中心坐标（米，车体坐标系） */
    float    intensity;      /**< 平均反射强度 [0, 1] */
    uint32_t point_count;    /**< 点云数量 */
    uint32_t frame_id;       /**< 帧序号 */
} LidarFrame;

/** GPS 数据 — 发布到 sensor/gps */
typedef struct {
    double latitude;         /**< 纬度（度） */
    double longitude;        /**< 经度（度） */
    float  speed_mps;        /**< 速度（m/s） */
    float  heading_deg;      /**< 航向角（度，北偏东） */
    float  accuracy_m;       /**< 定位精度（米，1-sigma） */
} GpsData;

/* ── 感知输出 ────────────────────────────────────────────── */

/** 障碍物类型 */
typedef enum {
    OBJ_TYPE_UNKNOWN   = 0,
    OBJ_TYPE_VEHICLE   = 1,
    OBJ_TYPE_PEDESTRIAN = 2,
    OBJ_TYPE_CYCLIST   = 3
} ObstacleType;

/** 单个障碍物 */
typedef struct {
    uint32_t      id;           /**< 目标 ID（跨帧唯一） */
    float         x;            /**< 纵向距离，正前方为正（米） */
    float         y;            /**< 横向距离，左正右负（米） */
    float         vx;           /**< 纵向相对速度（m/s，正=同向，负=对向） */
    float         vy;           /**< 横向相对速度（m/s） */
    float         width;        /**< 目标宽度（米） */
    float         length;       /**< 目标长度（米） */
    ObstacleType  type;         /**< 目标类型 */
    float         confidence;   /**< 置信度 [0, 1] */
} Obstacle;

/** 障碍物列表 — 发布到 perception/obstacles */
#define ADAS_MAX_OBSTACLES 8
typedef struct {
    uint32_t frame_id;
    uint64_t timestamp_us;                      /**< CLOCK_MONOTONIC 时间戳（微秒） */
    uint32_t count;                             /**< 有效障碍物数量 */
    Obstacle obstacles[ADAS_MAX_OBSTACLES];
} ObstacleList;

/* ── 场景理解输出 (v0.3 — 6层认知 + 5维环境模型) ──────────── */

/** 跟踪目标 — 发布到 perception/tracked_objects
 *
 *  比 Obstacle 多了3D框、加速度、朝向、车道归属、静态/动态分类 */
typedef struct {
    uint32_t      id;            /**< 跨帧持久 ID（object_tracker 分配） */
    uint32_t      track_age;     /**< 持续跟踪帧数（判断稳定性） */
    ObstacleType  type;          /**< UNKNOWN/VEHICLE/PEDESTRIAN/CYCLIST */
    float         x, y, z;       /**< 3D 位置（车体坐标系，z=地面以上高度） */
    float         vx, vy, vz;    /**< 速度（m/s） */
    float         ax, ay;         /**< 加速度（m/s²） */
    float         heading;       /**< 运动朝向（rad） */
    float         width, length, height; /**< 3D 边框 */
    float         confidence;    /**< 检测置信度 [0,1] */
    int32_t       lane_id;       /**< 所属车道 ID（-1=未知） */
    uint8_t       is_static : 1; /**< 静止目标 */
    uint8_t       is_on_road : 1;/**< 在道路上 */
    uint8_t       _reserved : 6;
} TrackedObject;

/** 跟踪目标列表 — 发布到 perception/tracked_objects */
#define ADAS_MAX_TRACKED 32
typedef struct {
    uint32_t      frame_id;
    uint64_t      timestamp_us;
    uint32_t      count;
    TrackedObject objects[ADAS_MAX_TRACKED];
} TrackedObjectList;

/** 预测轨迹 — 发布到 prediction/tracks
 *
 *  对每个 TrackedObject 生成 1-3 条未来轨迹，带概率 */
#define PRED_MAX_WAYPOINTS 10
typedef struct {
    uint32_t object_id;            /**< 关联 TrackedObject.id */
    float    confidence;           /**< 轨迹置信度 [0,1] */
    float    horizon_s;            /**< 预测时长（如 5.0s） */
    uint8_t  trajectory_count;     /**< 轨迹条数（通常 1-3） */
    struct {
        float prob;                         /**< 该轨迹概率 */
        float waypoints[PRED_MAX_WAYPOINTS][3]; /**< [step][x, y, v] */
        uint8_t waypoint_count;
    } trajectories[3];               /**< 最多3条轨迹 */
} PredictionTrack;

/** 预测列表 — 发布到 prediction/tracks */
#define PRED_MAX_TRACKS 32
typedef struct {
    uint32_t        frame_id;
    uint64_t        timestamp_us;
    uint32_t        count;
    PredictionTrack  tracks[PRED_MAX_TRACKS];
} PredictionList;

/** 车道线 — 发布到 perception/lanes
 *
 *  3次多项式: y = c0 + c1*x + c2*x² + c3*x³ (车体坐标系) */
typedef enum {
    LANE_SOLID   = 0,
    LANE_DASHED  = 1,
    LANE_CURB    = 2,
    LANE_VIRTUAL = 3
} LaneType;

typedef struct {
    int32_t  lane_id;
    LaneType type;
    float    coeffs[4];        /**< 多项式系数 c0..c3 */
    float    visible_range_m;  /**< 可见距离（m） */
    float    confidence;       /**< 检测置信度 [0,1] */
} LaneBoundary;

/** 车道线列表 — 发布到 perception/lanes */
#define LANE_MAX_BOUNDARIES 8
typedef struct {
    uint32_t      frame_id;
    uint64_t      timestamp_us;
    uint32_t      count;
    LaneBoundary  boundaries[LANE_MAX_BOUNDARIES];
} LaneBoundaryList;

/* ── 雷达目标 (预留) ───────────────────────────────────── */

/** 毫米波雷达目标 */
typedef struct {
    uint32_t id;
    float    range;            /**< 径向距离（m） */
    float    azimuth;          /**< 方位角（rad） */
    float    elevation;        /**< 俯仰角（rad，0=水平） */
    float    range_rate;       /**< 径向速度（m/s，正=远离） */
    float    rcs;              /**< 雷达散射截面（dBsm） */
    float    confidence;       /**< 检测置信度 [0,1] */
} RadarTarget;

#define RADAR_MAX_TARGETS 64
typedef struct {
    uint32_t     frame_id;
    uint64_t     timestamp_us;
    uint32_t     count;
    RadarTarget  targets[RADAR_MAX_TARGETS];
} RadarTargetList;

/* ── 自车状态 — 发布到 perception/ego_state ──────────────── */

/** 自车状态 — 发布到 perception/ego_state */
typedef struct {
    double latitude;         /**< 纬度（度） */
    double longitude;        /**< 经度（度） */
    float  heading_deg;      /**< 航向角（度） */
    float  speed_mps;        /**< 车速（m/s） */
    float  yaw_rate_dps;     /**< 偏航角速度（度/秒） */
    float  acceleration_mss; /**< 纵向加速度（m/s²，正=加速，负=制动） */
} EgoState;

/* ── 控制输出 ────────────────────────────────────────────── */

/** 档位 */
typedef enum {
    GEAR_REVERSE = -1,
    GEAR_NEUTRAL =  0,
    GEAR_DRIVE   =  1
} Gear;

/** 控制指令 — 发布到 control/cmd */
typedef struct {
    uint32_t seq;            /**< 指令序号 */
    float    throttle;       /**< 油门 [0.0, 1.0] */
    float    brake;          /**< 制动 [0.0, 1.0] */
    float    steering;       /**< 转向角（rad）：正值右转，负值左转，范围约 ±0.22 rad（约 ±12.6°），actuator 负责归一化到硬件量程 */
    Gear     gear;           /**< 档位 */
    bool     emergency_stop; /**< 紧急制动标志 */
} ControlCmd;

#ifdef __cplusplus
}
#endif

#endif /* ADAS_MSGS_H */
