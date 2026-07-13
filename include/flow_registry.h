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

/**
 * 类型注册通知（由 serializer_register_type() 调用，避免循环委托）。
 * 仅缓存元数据供 JSON 导出，不回调 serializer。
 */
void flow_registry_on_type_registered(const TypeRegistryEntry* entry);

/* ── MsgSchema Integration ─────────────────────────────── */

/** Schema entry for topic type checking */
typedef struct {
    char    topic[64];       /**< Topic name */
    size_t  struct_size;     /**< Expected struct size */
    char    type_name[64];   /**< C type name */
} FlowSchemaEntry;

/** Register a schema (delegates to msg_schema) */
int flow_registry_register_schema(const char* topic, size_t struct_size,
                                  const char* type_name);
const FlowSchemaEntry* flow_registry_get_schema(const char* topic);
int flow_registry_list_schemas(FlowSchemaEntry* buf, int max);
int flow_registry_schema_count(void);

/* ── Param Integration ──────────────────────────────────── */

/** Re-export param_registry ParamEntry as FlowRegistry's ParamMeta */
typedef struct {
    char        name[64];
    int         type;           /**< 0=int,1=float,2=bool,3=string */
    char        value_str[64];  /**< Current value as string */
    char        description[128];
    bool        hot_reload;
} FlowParamMeta;

int  flow_registry_list_params(FlowParamMeta* buf, int max);
int  flow_registry_param_count(void);

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

/** 类型元信息（来自 serializer 注册通知的缓存） */
typedef struct {
    char     name[64];        /**< C type name */
    uint32_t type_id;         /**< FNV-1a 类型 ID */
    size_t   struct_size;     /**< 结构体大小 */
} FlowTypeMeta;

int flow_registry_list_types(FlowTypeMeta* buf, int max);
int flow_registry_type_count(void);  /**< 已注册类型数 */

/** 注册中心条目总数 */
int flow_registry_total_count(void);

/* ── Unregister ──────────────────────────────────────────── */

int flow_registry_unregister_task(const char* name);
int flow_registry_unregister_topic(const char* name);
int flow_registry_unregister_plugin(const char* name);

/* ══════════════════════════════════════════════════════════ */
/* 导出                                                        */
/* ══════════════════════════════════════════════════════════ */

/** 导出完整注册中心状态为 JSON (调用者 free) */
char* flow_registry_export_json(void);

/* ══════════════════════════════════════════════════════════ */
/* Plugin Self-Declaration Macro                              */
/* ══════════════════════════════════════════════════════════ */

/**
 * 插件自声明宏 — 在 .so 加载时自动向注册中心注册插件元信息。
 *
 * 用法（放在插件 .c 文件顶层）：
 *   FLOW_REGISTRY_DECLARE_PLUGIN(my_plugin, "1.0.0", "My ADAS plugin");
 */
#define FLOW_REGISTRY_DECLARE_PLUGIN(pname, pver, pdesc) \
    __attribute__((constructor)) \
    static void _flow_registry_declare_##pname(void) { \
        flow_registry_register_plugin(#pname, NULL, NULL, NULL); \
    }

#ifdef __cplusplus
}
#endif

#endif /* FLOW_REGISTRY_H */
