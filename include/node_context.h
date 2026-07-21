#ifndef NODE_CONTEXT_H
#define NODE_CONTEXT_H

/**
 * @file node_context.h
 * @brief 统一节点上下文 — 捆绑基础设施 + 生命周期管理
 *
 * 替代每个节点 init() 中独立接收 bus/transport/discovery/scheduler
 * 四个指针的散装模式，改为统一注入 NodeContext。节点通过 ctx 访问
 * 所有基础设施，并通过 ctx 的便捷函数完成 subscribe/publish/advertise
 * 等操作，减少样板代码并确保一致性。
 *
 * 用法 (C 节点):
 *   static int my_init(NodeContext* ctx, const char* params_json) {
 *       ctx->subscribe(ctx, TOPIC_SENSOR_LIDAR, callback, NULL);
 *       ctx->advertise(ctx, TOPIC_PERCEPTION_OBSTACLES, TYPE_ID, CAP_PUBLISHER, 10.0);
 *       ctx->start_heartbeat(ctx, "my_node", 5.0);
 *       return 0;
 *   }
 *
 * 用法 (C++ 节点):
 *   在 init() 中保存 ctx 指针，循环中调用 ctx->heartbeat(ctx) 和
 *   ctx->record_error(ctx, "msg")。
 */

#include "node_plugin.h"
#include "topic_registry.h"
#include "health.h"
#include "error_codes.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── 前向声明 ──────────────────────────────────────────────── */

typedef struct NodeContext NodeContext;

/* ── 便捷函数指针类型 ──────────────────────────────────────── */

/** 订阅回调：收到消息时调用 */
typedef void (*NodeMsgCallback)(const char* topic, const uint8_t* data,
                                uint32_t len, void* user);

/** 生命周期回调 */
typedef int  (*NodeInitFn)(NodeContext* ctx, const char* params_json);
typedef int  (*NodeExecFn)(NodeContext* ctx);
typedef void (*NodeCleanupFn)(NodeContext* ctx);

/* ── NodeContext 结构体 ────────────────────────────────────── */

struct NodeContext {
    /* ── 基础设施指针（只读） ──────────────────────────────── */
    MessageBus*       bus;
    Transport*        transport;
    DiscoveryManager* discovery;
    Scheduler*        scheduler;
    TaskBase*         task;
    const NodePlugin* plugin;

    /* ── 配置 ──────────────────────────────────────────────── */
    const char*       node_name;        /**< 节点名称 */
    double            frequency_hz;     /**< 目标频率（用于 RateControl 初始化） */
    double            max_frequency_hz; /**< 最大允许频率（用于 auto_tuner） */

    /* ── 便捷函数 ──────────────────────────────────────────── */

    /** 订阅一个 topic */
    int (*subscribe)(NodeContext* ctx, const char* topic,
                     NodeMsgCallback cb, void* user);

    /** 发布数据到 topic */
    int (*publish)(NodeContext* ctx, const char* topic,
                   const uint8_t* data, uint32_t len);

    /** 向 discovery 注册 producer/consumer */
    int (*advertise)(NodeContext* ctx, const char* topic,
                     uint32_t type_id, uint32_t caps, double max_hz);

    /** 上报心跳（每个周期调用一次） */
    void (*heartbeat)(NodeContext* ctx);

    /** 记录错误 */
    void (*record_error)(NodeContext* ctx, const char* err_msg);

    /** 记录延迟样本 */
    void (*record_latency)(NodeContext* ctx, uint64_t latency_us);

    /** 记录卡顿 */
    void (*record_stall)(NodeContext* ctx);

    /* ── 内部状态（不直接访问） ────────────────────────────── */
    void*              _internal;        /**< 内部状态指针 */
};

/* ── 创建 / 销毁 ───────────────────────────────────────────── */

/**
 * 创建 NodeContext 并绑定到节点的 init() 参数。
 * 调用 node_context_init() 后，ctx 的所有便捷函数指针已就绪。
 *
 * @param ctx        预分配的 NodeContext（可由节点栈分配）
 * @param bus        消息总线
 * @param transport  传输层
 * @param discovery  服务发现
 * @param scheduler  调度器
 * @param task       任务基类
 * @param plugin     节点插件描述符
 * @param node_name  节点名称
 * @param freq_hz    目标频率
 * @param max_hz     最大频率
 * @return 0 成功，负数为错误码
 */
int node_context_init(NodeContext* ctx,
                      MessageBus* bus, Transport* transport,
                      DiscoveryManager* discovery, Scheduler* scheduler,
                      TaskBase* task, const NodePlugin* plugin,
                      const char* node_name,
                      double freq_hz, double max_hz);

/**
 * 销毁 NodeContext，释放内部资源。
 */
void node_context_destroy(NodeContext* ctx);

#ifdef __cplusplus
}
#endif

#endif /* NODE_CONTEXT_H */