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
    float    steering;       /**< 转向：-1.0 = 最大左转，+1.0 = 最大右转 */
    Gear     gear;           /**< 档位 */
    bool     emergency_stop; /**< 紧急制动标志 */
} ControlCmd;

#ifdef __cplusplus
}
#endif

#endif /* ADAS_MSGS_H */
