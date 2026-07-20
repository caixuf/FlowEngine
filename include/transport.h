#ifndef TRANSPORT_H
#define TRANSPORT_H

/**
 * @file transport.h
 * @brief 统一传输抽象层 — 透明路由进程内/跨进程/跨机通信
 *
 * 设计目标：用户不需要知道数据在哪——同进程用 bus，跨进程用 SHM IPC，
 * 跨机器用 TCP，全部自动选择。
 *
 * 用法：
 *   Transport* t = transport_create(bus, discovery, TRANSPORT_AUTO);
 *   transport_advertise(t, "sensor/lidar", LIDARFRAME_TYPE_ID);
 *   transport_publish(t, "sensor/lidar", &frame, sizeof(frame));
 *   transport_subscribe(t, "sensor/lidar", on_lidar, NULL);
 *   transport_start(t);
 *
 * 底层自动：
 *   - 同进程 → message_bus 直接传递（零开销）
 *   - 同机不同进程 → SHM IPC 通道（零拷贝共享内存）
 *   - 不同机器 → TCP network_transport（自动建连 + 断线重连）
 */

#include "message_bus.h"
#include "discovery.h"
#include "ipc_channel.h"
#include "network_transport.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── 策略 ────────────────────────────────────────────────── */

typedef enum {
    TRANSPORT_AUTO   = 0,  /**< 自动选择最优（默认） */
    TRANSPORT_LOCAL  = 1,  /**< 仅进程内总线 */
    TRANSPORT_IPC    = 2,  /**< 仅同机 IPC */
    TRANSPORT_REMOTE = 3,  /**< 仅跨机 TCP */
} TransportPolicy;

/* ── 路由条目（内部追踪每条 topic 的路由方式）──────────────── */

typedef enum {
    ROUTE_LOCAL,     /**< 同进程总线 */
    ROUTE_IPC,       /**< 共享内存 IPC */
    ROUTE_REMOTE,    /**< TCP 网络 */
} RouteType;

/* ── 不透明句柄 ──────────────────────────────────────────── */

typedef struct Transport Transport;

/* ══════════════════════════════════════════════════════════ */
/* 生命周期                                                    */
/* ══════════════════════════════════════════════════════════ */

/**
 * 创建统一传输层。
 * @param bus        本地消息总线
 * @param discovery  发现管理器（AUTO/REMOTE 模式需要）
 * @param policy     路由策略
 */
Transport* transport_create(MessageBus* bus, DiscoveryManager* discovery,
                            TransportPolicy policy);

/** 销毁传输层，释放所有 IPC/TCP 通道 */
void transport_destroy(Transport* t);

/** 启动传输层（创建后台线程 + 通道管理） */
int transport_start(Transport* t);

/** 停止传输层 */
void transport_stop(Transport* t);

/* ══════════════════════════════════════════════════════════ */
/* 发布/订阅 — 与 message_bus 接口完全一致                    */
/* ══════════════════════════════════════════════════════════ */

/**
 * 广播本节点提供此 topic（供 discovery 使用）。
 * @param type_id  消息类型 ID（0=未知）
 */
int transport_advertise(Transport* t, const char* topic,
                        uint32_t type_id);

/**
 * 发布消息（自动路由到所有订阅者）。
 * AUTO 模式下：本地订阅者走 bus，远端订阅者自动走 IPC/TCP。
 */
int transport_publish(Transport* t, const char* topic,
                      const void* data, uint32_t size);

/**
 * 订阅 topic（自动发现发布者并建立最优通道）。
 * 回调签名与 message_bus 一致。
 */
int transport_subscribe(Transport* t, const char* topic,
                        MessageCallback callback, void* user_data);

/** 取消订阅 */
int transport_unsubscribe(Transport* t, const char* topic,
                          MessageCallback callback);

/* ══════════════════════════════════════════════════════════ */
/* 状态查询                                                    */
/* ══════════════════════════════════════════════════════════ */

/** 查询指定 topic 当前使用的路由方式 */
RouteType transport_route_type(Transport* t, const char* topic);

/** 获取当前管理的 IPC 通道数 */
int transport_ipc_channel_count(Transport* t);

/** 获取当前 TCP 连接数 */
int transport_remote_peer_count(Transport* t);

/** 获取传输统计 */
typedef struct {
    uint64_t local_published;    /**< 本地总线发布次数 */
    uint64_t ipc_published;      /**< IPC 通道发布次数 */
    uint64_t remote_published;    /**< TCP 发布次数 */
    uint64_t local_delivered;
    uint64_t ipc_delivered;
    uint64_t remote_delivered;
    uint64_t ipc_dropped;        /**< 本端作为订阅者时, IPC 广播环形缓冲因落后被丢弃的消息累计数 */
} TransportStats;

void transport_get_stats(Transport* t, TransportStats* stats);

#ifdef __cplusplus
}
#endif

#endif /* TRANSPORT_H */
