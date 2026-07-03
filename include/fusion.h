#ifndef FUSION_H
#define FUSION_H

/**
 * @file fusion.h
 * @brief 多传感器数据融合框架 (FlowEngine Phase 4)
 *
 * 提供：
 *   - MessageBuffer：环形缓冲 + 时间戳查找
 *   - SyncedFrame：多传感器时间对齐后的同步帧
 *   - FusionPolicy：融合策略（最新值/时间对齐/加权平均/卡尔曼槽位）
 *   - FusionNode：C API 融合节点
 *   - FusionNodeCpp：C++ 协程基类（继承 CoroutineTask）
 *
 * 典型用法（C++）：
 *   class MyFusion : public FusionNodeCpp {
 *       MyFusion(MessageBus* bus) : FusionNodeCpp(bus) {
 *           AddSensorInput("sensor/lidar", LIDARFRAME_TYPE_ID, 32);
 *           AddSensorInput("sensor/gps", GPSDATA_TYPE_ID, 16);
 *           SetOutputTopic("fusion/localization", FUSEDLOC_TYPE_ID);
 *       }
 *       Message Fuse(const SyncedFrame& f) override {
 *           auto* lidar = msg_cast<LidarFrame>(&f.inputs[0]);
 *           auto* gps   = msg_cast<GpsData>(&f.inputs[1]);
 *           // ... fuse ...
 *           return fused_msg;
 *       }
 *   };
 */

#include "message_bus.h"
#include "serializer.h"
#include "task_interface.h"
#include <stdint.h>
#include <stdbool.h>
#include <pthread.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── 常量 ────────────────────────────────────────────────── */

#define FUSION_MAX_INPUTS          8
#define FUSION_DEFAULT_WINDOW_US   5000000ULL  /* 5 seconds */
#define FUSION_DEFAULT_MAX_DELTA_US 50000ULL   /* 50ms */

/* ══════════════════════════════════════════════════════════ */
/* MessageBuffer — 单传感器环形缓冲                           */
/* ══════════════════════════════════════════════════════════ */

typedef struct {
    char      topic[MSG_BUS_MAX_TOPIC_LEN];
    uint32_t  type_id;

    Message*  buffer;          /**< 环形缓冲 */
    uint32_t  capacity;        /**< 最大容量 */
    uint32_t  head;            /**< 写入位置 */
    uint32_t  count;           /**< 有效消息数 */
    uint64_t  window_us;       /**< 保留时间窗口 */

    pthread_mutex_t mutex;
} MessageBuffer;

/** 创建消息缓冲 */
MessageBuffer* message_buffer_create(const char* topic, uint32_t type_id,
                                     uint32_t capacity, uint64_t window_us);

/** 推入一条消息（线程安全） */
int message_buffer_push(MessageBuffer* mb, const Message* msg);

/**
 * 查找最接近 target_us 的消息（时间窗口内）。
 * @param max_delta_us  最大允许的时间偏差
 * @return 找到的消息指针（仅在下次 push 前有效），未找到返回 NULL
 */
const Message* message_buffer_find_nearest(MessageBuffer* mb, uint64_t target_us,
                                           uint64_t max_delta_us);

/** 获取缓冲中最新消息 */
const Message* message_buffer_latest(MessageBuffer* mb);

/** 销毁缓冲 */
void message_buffer_destroy(MessageBuffer* mb);

/* ══════════════════════════════════════════════════════════ */
/* SyncedFrame — 时间对齐后的同步帧                           */
/* ══════════════════════════════════════════════════════════ */

typedef struct {
    uint64_t  reference_ts;                    /**< 参考时间戳 */
    uint32_t  input_count;                     /**< 输入传感器数量 */
    Message   inputs[FUSION_MAX_INPUTS];       /**< 对齐后的消息 */
    bool      input_valid[FUSION_MAX_INPUTS];  /**< 哪些输入有有效数据 */
    uint64_t  input_timestamps[FUSION_MAX_INPUTS]; /**< 各输入的时间戳 */
    double    input_deltas_us[FUSION_MAX_INPUTS];  /**< 与参考时间的偏差 */
} SyncedFrame;

/* ══════════════════════════════════════════════════════════ */
/* FusionPolicy — 融合策略                                    */
/* ══════════════════════════════════════════════════════════ */

typedef enum {
    FUSION_LATEST_WINS,        /**< 使用每个输入的最新值，不管时间对齐 */
    FUSION_TIME_ALIGNED,       /**< 时间对齐到参考帧 */
    FUSION_WEIGHTED_AVERAGE,   /**< 加权平均（需同类型传感器） */
    FUSION_KALMAN_SLOT,        /**< 卡尔曼滤波器槽位 */
} FusionStrategy;

typedef struct {
    FusionStrategy strategy;
    uint64_t max_timestamp_delta_us;  /**< 对齐最大时间偏差 */
    double   sensor_weights[FUSION_MAX_INPUTS]; /**< WAVG 权重 */
    /* Kalman params (reserved) */
    double   kalman_process_q;
    double   kalman_measure_r;
} FusionPolicy;

#define FUSION_POLICY_TIME_ALIGNED \
    { .strategy = FUSION_TIME_ALIGNED, .max_timestamp_delta_us = FUSION_DEFAULT_MAX_DELTA_US }

#define FUSION_POLICY_LATEST \
    { .strategy = FUSION_LATEST_WINS }

/* ══════════════════════════════════════════════════════════ */
/* FusionNode — C API 融合节点                                */
/* ══════════════════════════════════════════════════════════ */

typedef struct FusionNode FusionNode;

/** 创建融合节点 */
FusionNode* fusion_node_create(const char* name, MessageBus* bus,
                               const FusionPolicy* policy);

/** 添加传感器输入 */
int fusion_node_add_input(FusionNode* fn, const char* topic,
                          uint32_t type_id, uint32_t buffer_capacity);

/** 设置输出 topic */
int fusion_node_set_output(FusionNode* fn, const char* topic, uint32_t type_id);

/**
 * 融合回调：收到对齐帧时调用。
 * 在此回调中构建 fused message 并通过 bus publish。
 */
typedef void (*FusionCallback)(const SyncedFrame* frame, MessageBus* bus,
                               const char* output_topic, uint32_t output_type_id,
                               void* user_data);

/** 设置融合回调 */
void fusion_node_set_callback(FusionNode* fn, FusionCallback cb, void* user_data);

/** 启动融合节点（订阅 bus + 启动处理线程） */
int fusion_node_start(FusionNode* fn);

/** 停止 */
void fusion_node_stop(FusionNode* fn);

/** 销毁 */
void fusion_node_destroy(FusionNode* fn);

#ifdef __cplusplus
}
#endif

/* ══════════════════════════════════════════════════════════ */
/* FusionNodeCpp — C++ 协程基类                               */
/* ══════════════════════════════════════════════════════════ */

#ifdef __cplusplus

#include "coroutine_task.h"
#include <memory>
#include <vector>
#include <string>

/**
 * FusionNodeCpp — 基于协程的融合节点基类。
 *
 * 继承 CoroutineTask，可通过 EXPORT_COROUTINE_TASK 导出为 .so 插件。
 * 用户只需继承并实现 Fuse() 方法。
 */
class FusionNodeCpp : public CoroutineTask {
public:
    FusionNodeCpp(MessageBus* bus,
                  const FusionPolicy& policy = FUSION_POLICY_TIME_ALIGNED);
    virtual ~FusionNodeCpp();

    /** 注册传感器输入 */
    void AddSensorInput(const std::string& topic, uint32_t type_id,
                        uint32_t buffer_capacity = 64);

    /** 设置融合结果输出 topic */
    void SetOutputTopic(const std::string& topic, uint32_t type_id);

    /**
     * 融合逻辑 — 子类重写。
     * @param frame 时间对齐后的同步帧
     * @return 融合后的 Message（通过 bus publish 到 output topic）
     */
    virtual Message Fuse(const SyncedFrame& frame) = 0;

    /** 协程入口：等待对齐帧 -> Fuse() -> publish */
    Task run() override;

    /** 启动融合（内部调用 execute） */
    void Start();

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

#endif /* __cplusplus */

#endif /* FUSION_H */
