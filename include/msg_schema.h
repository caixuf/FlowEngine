#ifndef MSG_SCHEMA_H
#define MSG_SCHEMA_H

/**
 * @file msg_schema.h
 * @brief 轻量级消息类型注册（零开销，纯头文件）
 *
 * 给 topic 绑定 C 结构体类型信息（名称 + 大小），
 * 在 publish/subscribe 时通过 MSG_CHECK_SIZE 宏做大小校验，
 * 类型不符时打印警告，不改动 Message 结构，零运行时开销。
 *
 * 用法：
 *   // 注册（每个 topic 只需注册一次，通常放在初始化代码中）
 *   MSG_REGISTER_TYPE("sensor/lidar", LidarFrame);
 *
 *   // 发布前校验
 *   MSG_CHECK_SIZE("sensor/lidar", sizeof(LidarFrame));
 *   message_bus_publish(bus, "sensor/lidar", "drv", &frame, sizeof(frame));
 *
 *   // 订阅回调中校验
 *   MSG_CHECK_SIZE(msg->topic, msg->data_size);
 */

#include <stddef.h>
#include <string.h>
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Schema 表 ────────────────────────────────────────── */

#define MSG_SCHEMA_MAX_ENTRIES 64
#define MSG_SCHEMA_TOPIC_LEN   64

typedef struct {
    char   topic[MSG_SCHEMA_TOPIC_LEN];
    size_t struct_size;
    char   type_name[64];
} MsgSchemaEntry;

/* 全局 schema 表（定义在头文件，通过 inline / static 避免多重定义）*/
static MsgSchemaEntry g_msg_schema[MSG_SCHEMA_MAX_ENTRIES];
static int            g_msg_schema_count = 0;

/* ── 注册函数 ─────────────────────────────────────────── */

static inline int msg_schema_register(const char* topic,
                                      size_t struct_size,
                                      const char* type_name) {
    if (!topic || g_msg_schema_count >= MSG_SCHEMA_MAX_ENTRIES) return -1;
    /* Update if already registered */
    for (int i = 0; i < g_msg_schema_count; i++) {
        if (strcmp(g_msg_schema[i].topic, topic) == 0) {
            g_msg_schema[i].struct_size = struct_size;
            strncpy(g_msg_schema[i].type_name, type_name ? type_name : "",
                    sizeof(g_msg_schema[i].type_name) - 1);
            return 0;
        }
    }
    MsgSchemaEntry* e = &g_msg_schema[g_msg_schema_count++];
    strncpy(e->topic,     topic,               sizeof(e->topic)     - 1);
    strncpy(e->type_name, type_name ? type_name : "", sizeof(e->type_name) - 1);
    e->struct_size = struct_size;
    return 0;
}

/* ── 校验函数 ─────────────────────────────────────────── */

static inline int msg_schema_check(const char* topic, size_t actual_size,
                                    const char* call_site) {
    for (int i = 0; i < g_msg_schema_count; i++) {
        if (strcmp(g_msg_schema[i].topic, topic) == 0) {
            if (g_msg_schema[i].struct_size != actual_size) {
                fprintf(stderr,
                    "[MSG_SCHEMA WARN] %s: topic='%s' expected size=%zu "
                    "(%s) but got %zu — type mismatch!\n",
                    call_site ? call_site : "?",
                    topic, g_msg_schema[i].struct_size,
                    g_msg_schema[i].type_name, actual_size);
                return -1;
            }
            return 0;   /* OK */
        }
    }
    /* Topic not registered — no complaint, just skip */
    return 0;
}

/* ── 便利宏 ───────────────────────────────────────────── */

/**
 * 注册 topic 对应的结构体类型
 * @param topic  字符串主题名
 * @param type   C 结构体类型名（不加引号）
 */
#define MSG_REGISTER_TYPE(topic, type) \
    msg_schema_register((topic), sizeof(type), #type)

/**
 * 检查 topic 与 size 是否符合注册的类型
 * 类型不符时打印警告（不中断执行）。
 */
#define MSG_CHECK_SIZE(topic, size) \
    msg_schema_check((topic), (size_t)(size), __func__)

#ifdef __cplusplus
}
#endif

#endif /* MSG_SCHEMA_H */
