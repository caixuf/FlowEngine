#ifndef FLOW_REGISTRY_H
#define FLOW_REGISTRY_H

/**
 * @file flow_registry.h
 * @brief FlowRegistry — 统一注册中心
 *
 * 把分散的 task/topic/type/plugin 注册信息统一管理。
 * 所有 flowctl 查询和 dashboard 数据都从这里来。
 *
 * 用法:
 *   flow_registry_register_task("perception", "LiDAR+GPS sensor", "libperception.so",
 *       (const char*[]){"sensor/lidar","sensor/gps",NULL},
 *       (const char*[]){"perception/obstacles","perception/ego_state",NULL}, NULL);
 *   flowctl list tasks   → 从这里查
 *   dashboard JSON       → flow_registry_export_json()
 */

#include "serializer.h"
#include "message_bus.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define FLOW_REGISTRY_MAX_TASKS    64
#define FLOW_REGISTRY_MAX_TOPICS   128
#define FLOW_REGISTRY_MAX_PLUGINS  32
#define FLOW_REGISTRY_MAX_TYPES    128
#define FLOW_REGISTRY_NAME_LEN     64
#define FLOW_REGISTRY_PATH_LEN     256
#define FLOW_REGISTRY_MAX_IO        8

/* ── Task Meta ───────────────────────────────────────────── */

typedef struct {
    char   name[FLOW_REGISTRY_NAME_LEN];
    char   description[256];
    char   plugin[FLOW_REGISTRY_PATH_LEN];  /**< 所属插件路径 */
    char   inputs[FLOW_REGISTRY_MAX_IO][FLOW_REGISTRY_NAME_LEN];  /**< 订阅的 topic */
    int    input_count;
    char   outputs[FLOW_REGISTRY_MAX_IO][FLOW_REGISTRY_NAME_LEN]; /**< 发布的 topic */
    int    output_count;
    char   params[FLOW_REGISTRY_MAX_IO][FLOW_REGISTRY_NAME_LEN];  /**< 参数名列表 */
    int    param_count;
    bool   registered;
} TaskMeta;

/* ── Topic Meta ──────────────────────────────────────────── */

typedef struct {
    char     name[FLOW_REGISTRY_NAME_LEN];
    uint32_t type_id;          /**< FNV-1a 类型 ID */
    TopicQos qos;              /**< QoS 配置 */
    bool     registered;
} TopicMeta;

/* ── Plugin Meta ─────────────────────────────────────────── */

typedef struct {
    char   name[FLOW_REGISTRY_NAME_LEN];
    char   path[FLOW_REGISTRY_PATH_LEN];    /**< .so 路径 */
    char   tasks[FLOW_REGISTRY_MAX_IO][FLOW_REGISTRY_NAME_LEN]; /**< 提供的 task */
    int    task_count;
    char   types[FLOW_REGISTRY_MAX_IO][FLOW_REGISTRY_NAME_LEN]; /**< 提供的类型 */
    int    type_count;
    bool   registered;
} PluginMeta;

/* ══════════════════════════════════════════════════════════ */
/* 注册 API                                                    */
/* ══════════════════════════════════════════════════════════ */

/** 注册一个 task */
int flow_registry_register_task(const char* name, const char* description,
                                const char* plugin,
                                const char** inputs,   /**< NULL-terminated array */
                                const char** outputs,
                                const char** params);

/** 注册一个 topic */
int flow_registry_register_topic(const char* name, uint32_t type_id,
                                 const TopicQos* qos);

/** 注册一个类型 (自动调用 serializer_register_type) */
int flow_registry_register_type(const TypeRegistryEntry* entry);

/** 注册一个插件 */
int flow_registry_register_plugin(const char* name, const char* path,
                                  const char** tasks,
                                  const char** types);

/* ══════════════════════════════════════════════════════════ */
/* 查询 API                                                    */
/* ══════════════════════════════════════════════════════════ */

const TaskMeta* flow_registry_get_task(const char* name);
int flow_registry_list_tasks(TaskMeta* buf, int max);

const TopicMeta* flow_registry_get_topic(const char* name);
int flow_registry_list_topics(TopicMeta* buf, int max);

const PluginMeta* flow_registry_get_plugin(const char* name);
int flow_registry_list_plugins(PluginMeta* buf, int max);

int flow_registry_type_count(void);  /**< 已注册类型数 */

/** 注册中心条目总数 */
int flow_registry_total_count(void);

/* ══════════════════════════════════════════════════════════ */
/* 导出                                                        */
/* ══════════════════════════════════════════════════════════ */

/** 导出完整注册中心状态为 JSON (调用者 free) */
char* flow_registry_export_json(void);

#ifdef __cplusplus
}
#endif

#endif /* FLOW_REGISTRY_H */
