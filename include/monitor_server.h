#ifndef MONITOR_SERVER_H
#define MONITOR_SERVER_H

#include "message_bus.h"
#include "discovery.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct MonitorServer MonitorServer;

/** 创建内嵌 HTTP 监控服务器 */
MonitorServer* monitor_server_create(MessageBus* bus, DiscoveryManager* discovery,
                                     int port);
/** 启动 (后台线程) */
void monitor_server_start(MonitorServer* ms);
/** 停止 */
void monitor_server_stop(MonitorServer* ms);
/** 销毁 */
void monitor_server_destroy(MonitorServer* ms);

#ifdef __cplusplus
}
#endif
#endif
