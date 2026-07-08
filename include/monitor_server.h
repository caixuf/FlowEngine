#ifndef MONITOR_SERVER_H
#define MONITOR_SERVER_H

#include "message_bus.h"
#include "discovery.h"
#include "stats_bridge.h"

/** Seconds without IPC data before the reconnect thread tears down and
 *  re-opens the channel (pipeline restart creates new shm/sem). */
#define IPC_RECONNECT_STALE_SEC 5

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

/**
 * 注入来自其他进程的完整 dashboard JSON（跨进程 IPC dashboard bridge）。
 * 线程安全；由 flowmond IPC 接收线程调用。
 * @param ms   MonitorServer 实例
 * @param json Null-terminated JSON string
 * @param len  Length of json
 */
void monitor_server_inject_dashboard_json(MonitorServer* ms,
                                          const char* json, size_t len);

/**
 * 返回自上次 dashboard JSON 注入以来的秒数。
 * 用于 IPC 重连线程判断 publisher 是否已重启。
 * @return 秒数，若从未注入则返回一个大值。
 */
double monitor_server_dashboard_age_sec(MonitorServer* ms);

/**
 * 返回自上次远程统计注入以来的秒数。
 * @return 秒数，若从未注入则返回一个大值。
 */
double monitor_server_stats_age_sec(MonitorServer* ms);

#ifdef __cplusplus
}
#endif
#endif
