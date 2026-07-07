#ifndef MONITOR_SERVER_H
#define MONITOR_SERVER_H

#include "message_bus.h"
#include "discovery.h"
#include "stats_bridge.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct MonitorServer MonitorServer;

/** 创建内嵌 HTTP 监控服务器 */
MonitorServer* monitor_server_create(MessageBus* bus, DiscoveryManager* discovery,
                                     int port, const char* html_path);
/** 启动 (后台线程) */
void monitor_server_start(MonitorServer* ms);
/** 停止 */
void monitor_server_stop(MonitorServer* ms);
/** 销毁 */
void monitor_server_destroy(MonitorServer* ms);

/**
 * 注入来自其他进程的远程统计数据（跨进程 IPC bridge）。
 * 线程安全；由 flowmond IPC 接收线程调用。
 * @param ms   MonitorServer 实例
 * @param pkt  从 IPC 通道收到的 StatsPacket
 */
void monitor_server_inject_remote_stats(MonitorServer* ms, const StatsPacket* pkt);

#ifdef __cplusplus
}
#endif
#endif
