#ifndef SENSOR_DRIVER_H
#define SENSOR_DRIVER_H

/**
 * @file sensor_driver.h
 * @brief 统一传感器驱动接口
 *
 * 所有真实传感器驱动（Camera/LiDAR/GPS/IMU/Radar）实现此接口。
 * 算法层不直接耦合硬件驱动，通过中间件提供的抽象接口访问传感器数据。
 *
 * 设计原则：
 *   1. 解耦 — 传感器硬件更换不影响算法
 *   2. 统一配置 — 所有传感器用 JSON 配置（标定参数、波特率、频率等）
 *   3. 健康上报 — health() 返回传感器状态，中间件汇总到降级标志位
 *   4. 标定接口 — calibrate() 支持在线/离线标定参数注入
 *
 * 用法：
 *   SensorDriver* drv = sensor_driver_create(&lidar_driver);
 *   drv->init("{\"port\":\"/dev/ttyUSB1\",\"baud\":115200}");
 *   drv->start();
 *   while (running) {
 *       if (drv->health() != 0) handle_degradation();
 *   }
 *   drv->stop();
 *   drv->cleanup();
 */

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** 传感器类型 */
typedef enum {
    SENSOR_CAMERA      = 0,
    SENSOR_LIDAR       = 1,
    SENSOR_GPS         = 2,
    SENSOR_IMU         = 3,
    SENSOR_RADAR       = 4,
    SENSOR_ULTRASONIC  = 5,
    SENSOR_CUSTOM      = 99
} SensorType;

/** 传感器健康状态 */
typedef enum {
    SENSOR_HEALTH_OK        = 0,  /**< 正常 */
    SENSOR_HEALTH_DEGRADED  = 1,  /**< 降级（丢帧/高温/低置信） */
    SENSOR_HEALTH_FAULT     = 2,  /**< 故障（无数据/通信中断） */
    SENSOR_HEALTH_OFFLINE   = 3   /**< 离线（未连接/未初始化） */
} SensorHealth;

/** 统一传感器驱动接口 */
typedef struct SensorDriver {
    /* ── 元数据 ── */
    const char* name;           /**< 传感器名称（如 "lidar_front"） */
    SensorType  type;           /**< 传感器类型 */
    const char* topic;          /**< 输出 topic（如 "sensor/lidar"） */
    uint32_t    type_id;        /**< 消息 type_id（用于 transport_advertise） */
    float       frequency_hz;   /**< 目标发布频率 */

    /* ── 生命周期 ── */
    /** 初始化（打开设备、配置参数）
     *  @param config_json  JSON 格式的配置（端口、波特率、频率等）
     *  @return 0=成功，<0=错误码 */
    int  (*init)(const char* config_json);

    /** 开始采集/发布 */
    int  (*start)(void);

    /** 停止采集/发布 */
    void (*stop)(void);

    /** 清理资源（关闭设备、释放内存） */
    void (*cleanup)(void);

    /* ── 运行时 ── */
    /** 健康检查
     *  @return 0=正常, 1=降级, 2=故障, 3=离线 */
    int  (*health)(void);

    /** 获取传感器元数据 JSON（型号、固件版本、出厂信息）
     *  @return malloc 的 JSON 字符串，调用者 free() */
    char* (*get_info_json)(void);

    /** 在线标定
     *  @param calib_json 标定参数 JSON */
    void (*calibrate)(const char* calib_json);

    /* ── 状态 ── */
    uint64_t frames_published;  /**< 累计发布帧数 */
    uint64_t frames_dropped;    /**< 累计丢帧数 */
    uint64_t last_frame_us;     /**< 最后一帧时间戳 */
    float    current_fps;       /**< 当前实际帧率 */
    int      health_status;     /**< 最近一次 health() 返回值 */
} SensorDriver;

/**
 * 创建一个 SensorDriver 实例（分配并初始化为零）
 * 调用者填充字段后使用，cleanup 时 free()。
 */
static inline SensorDriver* sensor_driver_create(void) {
    return (SensorDriver*)calloc(1, sizeof(SensorDriver));
}

/** 释放 SensorDriver */
static inline void sensor_driver_destroy(SensorDriver* drv) {
    if (drv) free(drv);
}

#ifdef __cplusplus
}
#endif

#endif /* SENSOR_DRIVER_H */
