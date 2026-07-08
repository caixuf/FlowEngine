/**
 * flow_registry.c — 统一注册中心实现
 *
 * 把 msg_schema / serializer / task_manager / process_manager 的
 * 元信息统一管理，提供 JSON 导出供 flowctl / dashboard 使用。
 */

#include "flow_registry.h"
#include "error_codes.h"
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

int flow_registry_register_type(const TypeRegistryEntry* entry) {
    return serializer_register_type(entry);
}

int flow_registry_type_count(void) {
    return serializer_type_count();
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
/* Summary                                                    */
/* ══════════════════════════════════════════════════════════ */

int flow_registry_total_count(void) {
    return g_task_count + g_topic_count + g_plugin_count + g_schema_count + serializer_type_count();
}

/* ══════════════════════════════════════════════════════════ */
/* JSON Export                                                */
/* ══════════════════════════════════════════════════════════ */

char* flow_registry_export_json(void) {
    pthread_mutex_lock(&g_mutex);

    size_t sz = 16384;
    char* buf = (char*)malloc(sz);
    if (!buf) { pthread_mutex_unlock(&g_mutex); return NULL; }

    int off = snprintf(buf, sz,
        "{\"tasks\":[");
    for (int i = 0; i < g_task_count; i++) {
        TaskMeta* t = &g_tasks[i];
        if ((size_t)off + 512 >= sz) { sz *= 2; buf = (char*)realloc(buf, sz); }
        off += snprintf(buf + off, sz - (size_t)off,
            "%s{\"name\":\"%s\",\"desc\":\"%s\",\"plugin\":\"%s\","
            "\"inputs\":[",
            i > 0 ? "," : "", t->name, t->description, t->plugin);
        for (int j = 0; j < t->input_count; j++)
            off += snprintf(buf + off, sz - (size_t)off, "%s\"%s\"", j>0?",":"", t->inputs[j]);
        off += snprintf(buf + off, sz - (size_t)off, "],\"outputs\":[");
        for (int j = 0; j < t->output_count; j++)
            off += snprintf(buf + off, sz - (size_t)off, "%s\"%s\"", j>0?",":"", t->outputs[j]);
        off += snprintf(buf + off, sz - (size_t)off, "]}");
    }

    off += snprintf(buf + off, sz - (size_t)off, "],\"topics\":[");
    for (int i = 0; i < g_topic_count; i++) {
        if ((size_t)off + 256 >= sz) { sz *= 2; buf = (char*)realloc(buf, sz); }
        off += snprintf(buf + off, sz - (size_t)off,
            "%s{\"name\":\"%s\",\"type_id\":\"0x%08x\"}",
            i > 0 ? "," : "", g_topics[i].name, g_topics[i].type_id);
    }

    off += snprintf(buf + off, sz - (size_t)off, "],\"plugins\":[");
    for (int i = 0; i < g_plugin_count; i++) {
        if ((size_t)off + 256 >= sz) { sz *= 2; buf = (char*)realloc(buf, sz); }
        off += snprintf(buf + off, sz - (size_t)off,
            "%s{\"name\":\"%s\",\"path\":\"%s\",\"tasks\":%d,\"types\":%d}",
            i > 0 ? "," : "", g_plugins[i].name, g_plugins[i].path,
            g_plugins[i].task_count, g_plugins[i].type_count);
    }

    off += snprintf(buf + off, sz - (size_t)off, "],\"schemas\":[");
    for (int i = 0; i < g_schema_count; i++) {
        if ((size_t)off + 256 >= sz) { sz *= 2; buf = (char*)realloc(buf, sz); }
        off += snprintf(buf + off, sz - (size_t)off,
            "%s{\"topic\":\"%s\",\"type\":\"%s\",\"size\":%zu}",
            i > 0 ? "," : "", g_schemas[i].topic, g_schemas[i].type_name,
            g_schemas[i].struct_size);
    }

    off += snprintf(buf + off, sz - (size_t)off,
        "],\"summary\":{\"tasks\":%d,\"topics\":%d,\"plugins\":%d,\"schemas\":%d,\"types\":%d}}",
        g_task_count, g_topic_count, g_plugin_count, g_schema_count, serializer_type_count());

    pthread_mutex_unlock(&g_mutex);
    return buf;
}
