#ifndef NODE_PLUGIN_H
#define NODE_PLUGIN_H

/**
 * @file node_plugin.h
 * @brief FlowEngine 节点插件接口 — dlopen 动态加载标准
 *
 * 每个节点编译为独立 .so，通过此接口与 flow_launcher 对接。
 * 节点内部不再使用 g_transport / g_bus 等全局变量，
 * 所有基础设施通过 init() 注入。
 *
 * 节点 .so 必须导出的符号:
 *   NodePlugin* node_get_plugin(void);
 *
 * 典型用法 (launcher):
 *   void* lib = dlopen("libfusion_node.so", RTLD_NOW|RTLD_LOCAL);
 *   NodePlugin* (*get)(void) = dlsym(lib, "node_get_plugin");
 *   NodePlugin* p = get();
 *   p->init(bus, transport, discovery, scheduler, params_json);
 *   p->start();
 *   // ... later:
 *   p->stop();
 *   p->cleanup();
 *   dlclose(lib);
 *
 * 对比旧方式 (e2e_demo.c --role，已移除):
 *   旧: ROLE_MATCH("fusion") → 所有节点代码共享一个进程/文件
 *   新: dlopen("libfusion_node.so") → 节点完全独立，改算法不重编译主程序
 */

#include "message_bus.h"
#include "transport.h"
#include "discovery.h"
#include "scheduler.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * 节点插件 ABI 版本。
 *
 * flow_launcher 加载 .so 后会检查 NodePlugin::api_version：
 *   - 等于本值        → 兼容，正常加载
 *   - 为 0            → 旧插件（未声明版本），告警但允许加载（向后兼容）
 *   - 其它非 0 值     → 不兼容，拒绝加载
 *
 * 每次 NodePlugin 结构体布局或生命周期契约发生不兼容变更时，递增此宏。
 */
#define NODE_PLUGIN_API_VERSION 1u

/* ── 节点插件描述符 ──────────────────────────────────────────── */

typedef struct NodePlugin {
    /* ── ABI 版本（必须首字段，供 launcher 校验） ── */
    uint32_t     api_version;    /**< 置为 NODE_PLUGIN_API_VERSION */

    /* ── 元数据 ─────────────────────────────── */
    const char*  name;           /**< 节点名 (如 "fusion") */
    const char*  version;        /**< 版本号 */
    const char*  description;    /**< 一句话描述 */
    const char** input_topics;   /**< 订阅的 topics, NULL 结尾 */
    const char** output_topics;  /**< 发布的 topics, NULL 结尾 */

    /* ── 生命周期钩子 ───────────────────────── */

    /**
     * 初始化: 注入基础设施，解析 params_json，注册 topic 订阅/发布。
     * 在 start() 前调用。
     * @return 0 成功, 非 0 失败
     */
    int  (*init)(MessageBus* bus, Transport* transport,
                 DiscoveryManager* discovery, Scheduler* scheduler,
                 const char* params_json);

    /**
     * 启动执行: 创建内部任务线程，进入运行循环。
     * @return 0 成功
     */
    int  (*start)(void);

    /**
     * 请求优雅停止: 设置 should_stop 标志，不阻塞。
     */
    void (*stop)(void);

    /**
     * 等待线程退出 + 释放资源。在 stop() 后调用。
     */
    void (*cleanup)(void);

    /**
     * 健康检查 (可选, NULL = 不检查)
     * @return 0 正常, 非 0 异常
     */
    int  (*health)(void);

} NodePlugin;

/* ── 节点 .so 必须导出此函数 ─────────────────────────────────── */
typedef NodePlugin* (*NodeGetPluginFn)(void);
#define NODE_PLUGIN_SYMBOL "node_get_plugin"

/* ── 节点自描述广播 ──────────────────────────────────────────── */

/**
 * 节点在 init() 末尾调用此函数，向 "flowengine/node_info" topic 广播自身元数据。
 *
 * monitor_node 订阅此 topic，收集所有节点信息后写入拓扑 JSON。
 * 这是 dlopen 单进程模式下拓扑感知的标准契约：
 *   - 不依赖共享全局状态（静态库全局变量不跨 dlopen 边界）
 *   - 不依赖 discovery（discovery 只记录远端 peer，不记录本地节点）
 *   - 纯数据驱动：谁启动就广播，monitor 收集即可
 *
 * @param transport  节点 init() 注入的 transport 指针
 * @param plugin     节点自身的 NodePlugin 描述符
 */
#include "transport.h"
#include <stdio.h>
#include <string.h>
#include <unistd.h>

static inline void node_announce_self(Transport* transport, const NodePlugin* plugin) {
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
            n += snprintf(json+n, sizeof(json)-n, "%s{\"topic\":\"%s\",\"role\":\"sub\",\"caps\":2}",
                         need_comma ? "," : "", plugin->input_topics[i]);
            need_comma = 1;
        }
    }
    if (plugin->output_topics) {
        for (int i = 0; plugin->output_topics[i] && n < (int)sizeof(json)-60; i++) {
            n += snprintf(json+n, sizeof(json)-n, "%s{\"topic\":\"%s\",\"role\":\"pub\",\"caps\":1}",
                         need_comma ? "," : "", plugin->output_topics[i]);
            need_comma = 1;
        }
    }
    n += snprintf(json+n, sizeof(json)-n, "]}");
    transport_publish(transport, "flowengine/node_info", json, (uint32_t)n + 1);
}

#ifdef __cplusplus
}
#endif

#endif /* NODE_PLUGIN_H */
