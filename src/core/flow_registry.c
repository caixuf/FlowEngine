/**
 * flow_registry.c — 统一注册中心实现
 *
 * 把 msg_schema / serializer / task_manager / process_manager 的
 * 元信息统一管理，提供 JSON 导出供 flowctl / dashboard 使用。
 */

#include "flow_registry.h"
#include "param_registry.h"
#include "error_codes.h"
#include <cjson/cJSON.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <pthread.h>

/* ── 全局注册表 ──────────────────────────────────────────── */

static TaskMeta     g_tasks[FLOW_REGISTRY_MAX_TASKS];
static int          g_task_count = 0;

static TopicMeta    g_topics[FLOW_REGISTRY_MAX_TOPICS];
static int          g_topic_count = 0;

static PluginMeta   g_plugins[FLOW_REGISTRY_MAX_PLUGINS];
static int          g_plugin_count = 0;

static FlowSchemaEntry g_schemas[FLOW_REGISTRY_MAX_TOPICS];
static int          g_schema_count = 0;

static pthread_mutex_t g_mutex = PTHREAD_MUTEX_INITIALIZER;

/* ══════════════════════════════════════════════════════════ */
/* Task Registry                                              */
/* ══════════════════════════════════════════════════════════ */

int flow_registry_register_task(const char* name, const char* description,
                                const char* plugin,
                                const char** inputs,
                                const char** outputs,
                                const char** params) {
    if (!name) return ERR_INVALID_PARAM;

    pthread_mutex_lock(&g_mutex);

    /* Check for duplicates */
    for (int i = 0; i < g_task_count; i++) {
        if (strcmp(g_tasks[i].name, name) == 0) {
            /* Update existing */
            if (description) snprintf(g_tasks[i].description, 256, "%s", description);
            if (plugin) snprintf(g_tasks[i].plugin, FLOW_REGISTRY_PATH_LEN, "%s", plugin);
            pthread_mutex_unlock(&g_mutex);
            return 0;
        }
    }

    if (g_task_count >= FLOW_REGISTRY_MAX_TASKS) {
        pthread_mutex_unlock(&g_mutex);
        return ERR_OVERFLOW;
    }

    TaskMeta* t = &g_tasks[g_task_count++];
    memset(t, 0, sizeof(*t));
    snprintf(t->name, FLOW_REGISTRY_NAME_LEN, "%s", name);
    if (description) snprintf(t->description, 256, "%s", description);
    if (plugin) snprintf(t->plugin, FLOW_REGISTRY_PATH_LEN, "%s", plugin);

    if (inputs) for (int i = 0; inputs[i] && t->input_count < FLOW_REGISTRY_MAX_IO; i++)
        snprintf(t->inputs[t->input_count++], FLOW_REGISTRY_NAME_LEN, "%s", inputs[i]);
    if (outputs) for (int i = 0; outputs[i] && t->output_count < FLOW_REGISTRY_MAX_IO; i++)
        snprintf(t->outputs[t->output_count++], FLOW_REGISTRY_NAME_LEN, "%s", outputs[i]);
    if (params) for (int i = 0; params[i] && t->param_count < FLOW_REGISTRY_MAX_IO; i++)
        snprintf(t->params[t->param_count++], FLOW_REGISTRY_NAME_LEN, "%s", params[i]);

    t->registered = true;
    pthread_mutex_unlock(&g_mutex);
    return 0;
}

const TaskMeta* flow_registry_get_task(const char* name) {
    if (!name) return NULL;
    pthread_mutex_lock(&g_mutex);
    for (int i = 0; i < g_task_count; i++)
        if (strcmp(g_tasks[i].name, name) == 0) {
            pthread_mutex_unlock(&g_mutex);
            return &g_tasks[i];
        }
    pthread_mutex_unlock(&g_mutex);
    return NULL;
}

int flow_registry_list_tasks(TaskMeta* buf, int max) {
    if (!buf || max <= 0) return 0;
    pthread_mutex_lock(&g_mutex);
    int n = (g_task_count < max) ? g_task_count : max;
    memcpy(buf, g_tasks, (size_t)n * sizeof(TaskMeta));
    pthread_mutex_unlock(&g_mutex);
    return n;
}

/* ══════════════════════════════════════════════════════════ */
/* Topic Registry                                             */
/* ══════════════════════════════════════════════════════════ */

int flow_registry_register_topic(const char* name, uint32_t type_id,
                                 const TopicQos* qos) {
    if (!name) return ERR_INVALID_PARAM;

    pthread_mutex_lock(&g_mutex);
    for (int i = 0; i < g_topic_count; i++) {
        if (strcmp(g_topics[i].name, name) == 0) {
            g_topics[i].type_id = type_id;
            if (qos) g_topics[i].qos = *qos;
            pthread_mutex_unlock(&g_mutex);
            return 0;
        }
    }

    if (g_topic_count >= FLOW_REGISTRY_MAX_TOPICS) {
        pthread_mutex_unlock(&g_mutex);
        return ERR_OVERFLOW;
    }

    TopicMeta* t = &g_topics[g_topic_count++];
    memset(t, 0, sizeof(*t));
    snprintf(t->name, FLOW_REGISTRY_NAME_LEN, "%s", name);
    t->type_id = type_id;
    if (qos) t->qos = *qos;
    t->registered = true;
    pthread_mutex_unlock(&g_mutex);
    return 0;
}

const TopicMeta* flow_registry_get_topic(const char* name) {
    if (!name) return NULL;
    pthread_mutex_lock(&g_mutex);
    for (int i = 0; i < g_topic_count; i++)
        if (strcmp(g_topics[i].name, name) == 0) {
            pthread_mutex_unlock(&g_mutex);
            return &g_topics[i];
        }
    pthread_mutex_unlock(&g_mutex);
    return NULL;
}

int flow_registry_list_topics(TopicMeta* buf, int max) {
    if (!buf || max <= 0) return 0;
    pthread_mutex_lock(&g_mutex);
    int n = (g_topic_count < max) ? g_topic_count : max;
    memcpy(buf, g_topics, (size_t)n * sizeof(TopicMeta));
    pthread_mutex_unlock(&g_mutex);
    return n;
}

/* ══════════════════════════════════════════════════════════ */
/* Type Registry (wraps existing serializer)                  */
/* ══════════════════════════════════════════════════════════ */

/* Lightweight type metadata cache — populated by serializer notifications */
#define FLOW_REGISTRY_MAX_TYPES 128
static struct {
    char     type_name[64];
    uint32_t type_id;
    size_t   struct_size;
} g_type_cache[FLOW_REGISTRY_MAX_TYPES];
static int g_type_cache_count = 0;

int flow_registry_register_type(const TypeRegistryEntry* entry) {
    return serializer_register_type(entry);
}

void flow_registry_on_type_registered(const TypeRegistryEntry* entry) {
    if (!entry) return;
    pthread_mutex_lock(&g_mutex);
    /* Avoid duplicates */
    for (int i = 0; i < g_type_cache_count; i++) {
        if (g_type_cache[i].type_id == entry->type_id) {
            snprintf(g_type_cache[i].type_name, sizeof(g_type_cache[i].type_name),
                     "%s", entry->type_name);
            g_type_cache[i].struct_size = entry->struct_size;
            pthread_mutex_unlock(&g_mutex);
            return;
        }
    }
    if (g_type_cache_count < FLOW_REGISTRY_MAX_TYPES) {
        int i = g_type_cache_count++;
        snprintf(g_type_cache[i].type_name, sizeof(g_type_cache[i].type_name),
                 "%s", entry->type_name);
        g_type_cache[i].type_id      = entry->type_id;
        g_type_cache[i].struct_size  = entry->struct_size;
    }
    pthread_mutex_unlock(&g_mutex);
}

int flow_registry_type_count(void) {
    /* Prefer cached count if available, fall back to serializer */
    int cached = g_type_cache_count;
    return cached > 0 ? cached : serializer_type_count();
}

int flow_registry_list_types(FlowTypeMeta* buf, int max) {
    if (!buf || max <= 0) return 0;
    pthread_mutex_lock(&g_mutex);
    int n = (g_type_cache_count < max) ? g_type_cache_count : max;
    for (int i = 0; i < n; i++) {
        snprintf(buf[i].name, sizeof(buf[i].name), "%s", g_type_cache[i].type_name);
        buf[i].type_id     = g_type_cache[i].type_id;
        buf[i].struct_size = g_type_cache[i].struct_size;
    }
    pthread_mutex_unlock(&g_mutex);
    return n;
}

/* ══════════════════════════════════════════════════════════ */
/* Schema Registry (bridges msg_schema)                       */
/* ══════════════════════════════════════════════════════════ */

int flow_registry_register_schema(const char* topic, size_t struct_size,
                                  const char* type_name) {
    if (!topic || !type_name || struct_size == 0) return ERR_INVALID_PARAM;

    pthread_mutex_lock(&g_mutex);
    for (int i = 0; i < g_schema_count; i++) {
        if (strcmp(g_schemas[i].topic, topic) == 0) {
            g_schemas[i].struct_size = struct_size;
            snprintf(g_schemas[i].type_name, sizeof(g_schemas[i].type_name), "%s", type_name);
            pthread_mutex_unlock(&g_mutex);
            return 0;
        }
    }
    if (g_schema_count >= FLOW_REGISTRY_MAX_TOPICS) {
        pthread_mutex_unlock(&g_mutex);
        return ERR_OVERFLOW;
    }
    FlowSchemaEntry* s = &g_schemas[g_schema_count++];
    memset(s, 0, sizeof(*s));
    snprintf(s->topic, sizeof(s->topic), "%s", topic);
    s->struct_size = struct_size;
    snprintf(s->type_name, sizeof(s->type_name), "%s", type_name);
    pthread_mutex_unlock(&g_mutex);
    return 0;
}

const FlowSchemaEntry* flow_registry_get_schema(const char* topic) {
    if (!topic) return NULL;
    pthread_mutex_lock(&g_mutex);
    for (int i = 0; i < g_schema_count; i++)
        if (strcmp(g_schemas[i].topic, topic) == 0) {
            pthread_mutex_unlock(&g_mutex);
            return &g_schemas[i];
        }
    pthread_mutex_unlock(&g_mutex);
    return NULL;
}

int flow_registry_list_schemas(FlowSchemaEntry* buf, int max) {
    if (!buf || max <= 0) return 0;
    pthread_mutex_lock(&g_mutex);
    int n = (g_schema_count < max) ? g_schema_count : max;
    memcpy(buf, g_schemas, (size_t)n * sizeof(FlowSchemaEntry));
    pthread_mutex_unlock(&g_mutex);
    return n;
}

int flow_registry_schema_count(void) {
    return g_schema_count;
}

/* ══════════════════════════════════════════════════════════ */
/* Plugin Registry                                            */
/* ══════════════════════════════════════════════════════════ */

int flow_registry_register_plugin(const char* name, const char* path,
                                  const char** tasks, const char** types) {
    if (!name) return ERR_INVALID_PARAM;

    pthread_mutex_lock(&g_mutex);
    for (int i = 0; i < g_plugin_count; i++) {
        if (strcmp(g_plugins[i].name, name) == 0) {
            if (path) snprintf(g_plugins[i].path, FLOW_REGISTRY_PATH_LEN, "%s", path);
            pthread_mutex_unlock(&g_mutex);
            return 0;
        }
    }

    if (g_plugin_count >= FLOW_REGISTRY_MAX_PLUGINS) {
        pthread_mutex_unlock(&g_mutex);
        return ERR_OVERFLOW;
    }

    PluginMeta* p = &g_plugins[g_plugin_count++];
    memset(p, 0, sizeof(*p));
    snprintf(p->name, FLOW_REGISTRY_NAME_LEN, "%s", name);
    if (path) snprintf(p->path, FLOW_REGISTRY_PATH_LEN, "%s", path);
    if (tasks) for (int i = 0; tasks[i] && p->task_count < FLOW_REGISTRY_MAX_IO; i++)
        snprintf(p->tasks[p->task_count++], FLOW_REGISTRY_NAME_LEN, "%s", tasks[i]);
    if (types) for (int i = 0; types[i] && p->type_count < FLOW_REGISTRY_MAX_IO; i++)
        snprintf(p->types[p->type_count++], FLOW_REGISTRY_NAME_LEN, "%s", types[i]);
    p->registered = true;
    pthread_mutex_unlock(&g_mutex);
    return 0;
}

const PluginMeta* flow_registry_get_plugin(const char* name) {
    if (!name) return NULL;
    pthread_mutex_lock(&g_mutex);
    for (int i = 0; i < g_plugin_count; i++)
        if (strcmp(g_plugins[i].name, name) == 0) {
            pthread_mutex_unlock(&g_mutex);
            return &g_plugins[i];
        }
    pthread_mutex_unlock(&g_mutex);
    return NULL;
}

int flow_registry_list_plugins(PluginMeta* buf, int max) {
    if (!buf || max <= 0) return 0;
    pthread_mutex_lock(&g_mutex);
    int n = (g_plugin_count < max) ? g_plugin_count : max;
    memcpy(buf, g_plugins, (size_t)n * sizeof(PluginMeta));
    pthread_mutex_unlock(&g_mutex);
    return n;
}

/* ══════════════════════════════════════════════════════════ */
/* Unregister                                                  */
/* ══════════════════════════════════════════════════════════ */

int flow_registry_unregister_task(const char* name) {
    if (!name) return ERR_INVALID_PARAM;
    pthread_mutex_lock(&g_mutex);
    for (int i = 0; i < g_task_count; i++) {
        if (strcmp(g_tasks[i].name, name) == 0) {
            if (i < g_task_count - 1)
                memmove(&g_tasks[i], &g_tasks[i+1],
                        (size_t)(g_task_count - i - 1) * sizeof(TaskMeta));
            g_task_count--;
            pthread_mutex_unlock(&g_mutex);
            return 0;
        }
    }
    pthread_mutex_unlock(&g_mutex);
    return ERR_NOT_FOUND;
}

int flow_registry_unregister_topic(const char* name) {
    if (!name) return ERR_INVALID_PARAM;
    pthread_mutex_lock(&g_mutex);
    for (int i = 0; i < g_topic_count; i++) {
        if (strcmp(g_topics[i].name, name) == 0) {
            if (i < g_topic_count - 1)
                memmove(&g_topics[i], &g_topics[i+1],
                        (size_t)(g_topic_count - i - 1) * sizeof(TopicMeta));
            g_topic_count--;
            pthread_mutex_unlock(&g_mutex);
            return 0;
        }
    }
    pthread_mutex_unlock(&g_mutex);
    return ERR_NOT_FOUND;
}

int flow_registry_unregister_plugin(const char* name) {
    if (!name) return ERR_INVALID_PARAM;
    pthread_mutex_lock(&g_mutex);
    for (int i = 0; i < g_plugin_count; i++) {
        if (strcmp(g_plugins[i].name, name) == 0) {
            if (i < g_plugin_count - 1)
                memmove(&g_plugins[i], &g_plugins[i+1],
                        (size_t)(g_plugin_count - i - 1) * sizeof(PluginMeta));
            g_plugin_count--;
            pthread_mutex_unlock(&g_mutex);
            return 0;
        }
    }
    pthread_mutex_unlock(&g_mutex);
    return ERR_NOT_FOUND;
}

/* ══════════════════════════════════════════════════════════ */
/* Summary                                                    */
/* ══════════════════════════════════════════════════════════ */

/* ── Param bridge (delegates to param_registry) ─────────── */

int flow_registry_list_params(FlowParamMeta* buf, int max) {
    if (!buf || max <= 0) return 0;
    ParamEntry pe[64];
    int n = param_list_all(pe, max < 64 ? max : 64);
    for (int i = 0; i < n && i < max; i++) {
        snprintf(buf[i].name, sizeof(buf[i].name), "%s", pe[i].name);
        buf[i].type = (int)pe[i].type;
        buf[i].hot_reload = pe[i].hot_reload;
        snprintf(buf[i].description, sizeof(buf[i].description), "%s", pe[i].description);
        /* Serialize current value to string */
        switch (pe[i].type) {
            case PARAM_INT:
                snprintf(buf[i].value_str, sizeof(buf[i].value_str), "%ld",
                         (long)pe[i].current_value.int_val); break;
            case PARAM_FLOAT:
                snprintf(buf[i].value_str, sizeof(buf[i].value_str), "%.3f",
                         pe[i].current_value.float_val); break;
            case PARAM_BOOL:
                snprintf(buf[i].value_str, sizeof(buf[i].value_str), "%s",
                         pe[i].current_value.bool_val ? "true" : "false"); break;
            case PARAM_STRING:
                snprintf(buf[i].value_str, sizeof(buf[i].value_str), "%s",
                         pe[i].current_value.str_val); break;
            default: buf[i].value_str[0] = '\0'; break;
        }
    }
    return n;
}

int flow_registry_param_count(void) {
    return param_count();
}

/* ══════════════════════════════════════════════════════════ */

int flow_registry_total_count(void) {
    return g_task_count + g_topic_count + g_plugin_count + g_schema_count + serializer_type_count();
}

/* ══════════════════════════════════════════════════════════ */
/* JSON Export                                                */
/* ══════════════════════════════════════════════════════════ */

char* flow_registry_export_json(void) {
    pthread_mutex_lock(&g_mutex);

    cJSON* root = cJSON_CreateObject();
    if (!root) { pthread_mutex_unlock(&g_mutex); return NULL; }

    /* Tasks */
    cJSON* tasks = cJSON_CreateArray();
    for (int i = 0; i < g_task_count; i++) {
        TaskMeta* t = &g_tasks[i];
        cJSON* task = cJSON_CreateObject();
        cJSON_AddStringToObject(task, "name", t->name);
        cJSON_AddStringToObject(task, "desc", t->description);
        cJSON_AddStringToObject(task, "plugin", t->plugin);

        cJSON* inputs = cJSON_CreateArray();
        for (int j = 0; j < t->input_count; j++)
            cJSON_AddItemToArray(inputs, cJSON_CreateString(t->inputs[j]));
        cJSON_AddItemToObject(task, "inputs", inputs);

        cJSON* outputs = cJSON_CreateArray();
        for (int j = 0; j < t->output_count; j++)
            cJSON_AddItemToArray(outputs, cJSON_CreateString(t->outputs[j]));
        cJSON_AddItemToObject(task, "outputs", outputs);

        cJSON_AddItemToArray(tasks, task);
    }
    cJSON_AddItemToObject(root, "tasks", tasks);

    /* Topics */
    cJSON* topics = cJSON_CreateArray();
    for (int i = 0; i < g_topic_count; i++) {
        cJSON* topic = cJSON_CreateObject();
        cJSON_AddStringToObject(topic, "name", g_topics[i].name);
        char type_id_str[16];
        snprintf(type_id_str, sizeof(type_id_str), "0x%08x", g_topics[i].type_id);
        cJSON_AddStringToObject(topic, "type_id", type_id_str);
        cJSON_AddItemToArray(topics, topic);
    }
    cJSON_AddItemToObject(root, "topics", topics);

    /* Plugins */
    cJSON* plugins = cJSON_CreateArray();
    for (int i = 0; i < g_plugin_count; i++) {
        cJSON* plugin = cJSON_CreateObject();
        cJSON_AddStringToObject(plugin, "name", g_plugins[i].name);
        cJSON_AddStringToObject(plugin, "path", g_plugins[i].path);
        cJSON_AddNumberToObject(plugin, "tasks", g_plugins[i].task_count);
        cJSON_AddNumberToObject(plugin, "types", g_plugins[i].type_count);
        cJSON_AddItemToArray(plugins, plugin);
    }
    cJSON_AddItemToObject(root, "plugins", plugins);

    /* Schemas */
    cJSON* schemas = cJSON_CreateArray();
    for (int i = 0; i < g_schema_count; i++) {
        cJSON* schema = cJSON_CreateObject();
        cJSON_AddStringToObject(schema, "topic", g_schemas[i].topic);
        cJSON_AddStringToObject(schema, "type", g_schemas[i].type_name);
        cJSON_AddNumberToObject(schema, "size", (double)g_schemas[i].struct_size);
        cJSON_AddItemToArray(schemas, schema);
    }
    cJSON_AddItemToObject(root, "schemas", schemas);

    /* Params */
    cJSON* params = cJSON_CreateArray();
    {
        FlowParamMeta pm[64];
        int pn = flow_registry_list_params(pm, 64);
        for (int i = 0; i < pn; i++) {
            cJSON* param = cJSON_CreateObject();
            cJSON_AddStringToObject(param, "name", pm[i].name);
            const char* type_str = pm[i].type == 0 ? "int" :
                                   pm[i].type == 1 ? "float" :
                                   pm[i].type == 2 ? "bool" : "string";
            cJSON_AddStringToObject(param, "type", type_str);
            cJSON_AddStringToObject(param, "value", pm[i].value_str);
            cJSON_AddBoolToObject(param, "hot_reload", pm[i].hot_reload);
            cJSON_AddItemToArray(params, param);
        }
    }
    cJSON_AddItemToObject(root, "params", params);

    /* Types */
    cJSON* types = cJSON_CreateArray();
    for (int i = 0; i < g_type_cache_count; i++) {
        cJSON* type = cJSON_CreateObject();
        cJSON_AddStringToObject(type, "name", g_type_cache[i].type_name);
        char type_id_str[16];
        snprintf(type_id_str, sizeof(type_id_str), "0x%08x", g_type_cache[i].type_id);
        cJSON_AddStringToObject(type, "type_id", type_id_str);
        cJSON_AddNumberToObject(type, "size", (double)g_type_cache[i].struct_size);
        cJSON_AddItemToArray(types, type);
    }
    cJSON_AddItemToObject(root, "types", types);

    /* Summary */
    cJSON* summary = cJSON_CreateObject();
    cJSON_AddNumberToObject(summary, "tasks", g_task_count);
    cJSON_AddNumberToObject(summary, "topics", g_topic_count);
    cJSON_AddNumberToObject(summary, "plugins", g_plugin_count);
    cJSON_AddNumberToObject(summary, "schemas", g_schema_count);
    cJSON_AddNumberToObject(summary, "params", flow_registry_param_count());
    cJSON_AddNumberToObject(summary, "types", flow_registry_type_count());
    cJSON_AddItemToObject(root, "summary", summary);

    char* out = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);

    pthread_mutex_unlock(&g_mutex);
    return out;
}
