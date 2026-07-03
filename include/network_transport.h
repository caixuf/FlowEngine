#ifndef NETWORK_TRANSPORT_H
#define NETWORK_TRANSPORT_H

/**
 * @file network_transport.h
 * @brief TCP 网络传输层 — 跨机器消息通信 (FlowEngine 分布式)
 *
 * 协议:
 *   [length: uint32 BE][Message serialized: N bytes]...
 *
 * 架构:
 *   Node A (local bus) ←→ NetworkTransport ←TCP→ NetworkTransport ←→ (local bus) Node B
 *
 * 与 discovery 集成: 发现新节点 → 自动建立 TCP 连接 → 桥接本地总线
 *
 * 用法:
 *   NetworkTransport* t = net_transport_create("0.0.0.0", 7700, my_bus, my_discovery);
 *   net_transport_bridge_topic(t, "sensor/lidar");     // 将该 topic 转发到远端
 *   net_transport_bridge_topic(t, "fusion/localization");
 *   net_transport_start(t);
 */

#include "message_bus.h"
#include "discovery.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define NET_DEFAULT_PORT       7700
#define NET_MAX_CONNECTIONS    32
#define NET_RECONNECT_BASE_MS  500
#define NET_RECONNECT_MAX_MS   30000
#define NET_FRAME_MAX_SIZE     (MSG_BUS_MAX_DATA_SIZE + 256)

/* ── 不透明句柄 ──────────────────────────────────────────── */

typedef struct NetworkTransport NetworkTransport;

/* ══════════════════════════════════════════════════════════ */
/* 生命周期                                                    */
/* ══════════════════════════════════════════════════════════ */

/**
 * 创建网络传输层。
 * @param bind_addr  监听地址（"0.0.0.0" = 所有网卡, NULL = "0.0.0.0"）
 * @param port       监听端口（0 = 使用 NET_DEFAULT_PORT）
 * @param bus        本地消息总线（用于桥接）
 * @param dm         发现管理器（用于自动获取远端节点地址，可为 NULL）
 */
NetworkTransport* net_transport_create(const char* bind_addr, uint16_t port,
                                       MessageBus* bus, DiscoveryManager* dm);

/** 销毁传输层，断开所有连接 */
void net_transport_destroy(NetworkTransport* t);

/** 启动传输层（创建监听 socket + 连接线程 + 接收线程） */
int net_transport_start(NetworkTransport* t);

/** 停止传输层 */
void net_transport_stop(NetworkTransport* t);

/* ══════════════════════════════════════════════════════════ */
/* 桥接控制                                                    */
/* ══════════════════════════════════════════════════════════ */

/**
 * 将本地 topic 桥接到所有远端节点。
 * 本地 pub 此 topic → 自动 TCP 转发到所有已连接远端 → 远端 bus pub。
 */
int net_transport_bridge_topic(NetworkTransport* t, const char* topic);

/** 取消桥接 */
int net_transport_unbridge_topic(NetworkTransport* t, const char* topic);

/**
 * 手动连接到指定远端节点（discovery 集成时会自动调用）。
 * @param host  远端 IP 或 hostname
 * @param port  远端端口
 * @return 0 成功, -1 失败
 */
int net_transport_connect(NetworkTransport* t, const char* host, uint16_t port);

/** 断开指定远端 */
int net_transport_disconnect(NetworkTransport* t, const char* host, uint16_t port);

/* ══════════════════════════════════════════════════════════ */
/* 状态                                                        */
/* ══════════════════════════════════════════════════════════ */

/** 获取当前连接数 */
int net_transport_connection_count(NetworkTransport* t);

/** 获取传输统计 */
typedef struct {
    uint64_t bytes_sent;
    uint64_t bytes_received;
    uint64_t msgs_sent;
    uint64_t msgs_received;
    uint64_t connects;
    uint64_t disconnects;
    uint64_t send_errors;
} NetTransportStats;

void net_transport_get_stats(NetworkTransport* t, NetTransportStats* stats);

#ifdef __cplusplus
}
#endif

#endif /* NETWORK_TRANSPORT_H */
