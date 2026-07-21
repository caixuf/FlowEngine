/**
 * @file node_plugin.c
 * @brief NodePlugin 托管模式（managed mode）运行时支持
 *
 * node_start_managed() 需要跨编译单元调用调度器 + TaskBase 运行时，
 * 故单独在此实现并链入 libflowengine_core。
 *
 * node_announce_self() 也从之前的 static inline 迁移至此，
 * 避免头文件内联函数需要 include transport.h 造成污染。
 */

#include "node_plugin.h"
#include "task_interface.h"
#include "scheduler.h"
#include "transport.h"
#include "logger.h"
#include <stdio.h>
#include <string.h>
#include <unistd.h>

void node_announce_self(Transport* transport, const NodePlugin* plugin) {
    if (!transport || !plugin || !plugin->name) return;
    char json[1024];
    int n = snprintf(json, sizeof(json),
        "{\"name\":\"%s\",\"version\":\"%s\",\"description\":\"%s\",\"pid\":%d,\"alive\":true,\"topics\":[",
        plugin->name,
        plugin->version     ? plugin->version     : "1.0.0",
        plugin->description ? plugin->description : "",
        (int)getpid());
    int need_comma = 0;
    if (plugin->input_topics) {
        for (int i = 0; plugin->input_topics[i] && n < (int)sizeof(json)-60; i++) {
            n += snprintf(json+n, sizeof(json)-n,
                         "%s{\"topic\":\"%s\",\"role\":\"sub\",\"caps\":2}",
                         need_comma ? "," : "", plugin->input_topics[i]);
            need_comma = 1;
        }
    }
    if (plugin->output_topics) {
        for (int i = 0; plugin->output_topics[i] && n < (int)sizeof(json)-60; i++) {
            n += snprintf(json+n, sizeof(json)-n,
                         "%s{\"topic\":\"%s\",\"role\":\"pub\",\"caps\":1}",
                         need_comma ? "," : "", plugin->output_topics[i]);
            need_comma = 1;
        }
    }
    n += snprintf(json+n, sizeof(json)-n, "]}");
    transport_publish(transport, "flowengine/node_info", json, (uint32_t)n + 1);
}

int node_start_managed(NodePlugin* plugin, Scheduler* sched) {
    if (!plugin || !sched) {
        return -1;
    }

    /* 仅 v2+ 插件且显式设置了 taskbase 才走托管路径。
     * 旧插件 taskbase 为 NULL，应继续使用自管的 start() 线程路径。 */
    if (plugin->api_version < NODE_PLUGIN_API_VERSION || !plugin->taskbase) {
        LOG_WARN("node_plugin",
                 "%s: managed mode requires api_version>=%u and plugin->taskbase set",
                 plugin->name ? plugin->name : "?", NODE_PLUGIN_API_VERSION);
        return -1;
    }

    /* 1) 注册到调度器：获得 RateControl / LatencyTracker / ResourceQuota 元数据 */
    int tid = scheduler_register_task(sched, plugin->taskbase, plugin->name);
    if (tid < 0) {
        LOG_WARN("node_plugin", "%s: scheduler_register_task failed (%d)",
                 plugin->name ? plugin->name : "?", tid);
        return tid;
    }

    /* 2) 派生工作线程：task_start 创建 joinable pthread，在线程内依次调用
     *    vtable->initialize()（若有）、vtable->execute()（节点主循环，应循环
     *    检查 task->should_stop）、vtable->cleanup()（若有）。
     *    scheduler_start() 启动后台监控线程，定期检查任务延迟和健康状态。 */
    int rc = task_start(plugin->taskbase);
    if (rc != 0) {
        LOG_WARN("node_plugin", "%s: task_start failed (%d)",
                 plugin->name ? plugin->name : "?", rc);
        return rc;
    }

    /* 3) 翻开调度器运行标志（幂等：多个托管节点共享同一调度器，首节点启动） */
    scheduler_start(sched);
    return 0;
}
