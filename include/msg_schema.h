#ifndef MSG_SCHEMA_H
#define MSG_SCHEMA_H

/**
 * @file msg_schema.h
 * @brief 轻量级消息类型注册（零开销，声明 + core库实现）
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
 *
 * 注意：注册表存储在 FlowEngine 核心库中（唯一实例），所有插件
 *       共享同一注册表，跨 .so 注册互通。
 */

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Public API (implemented in src/core/msg_schema.c) ── */

/**
 * 注册 topic 对应的结构体类型信息
 * @return 0 成功，-1 失败（表已满或参数非法）
 */
int msg_schema_register(const char* topic, size_t struct_size, const char* type_name);

/**
 * 检查 topic 与 size 是否符合注册的类型
 * @return 0 匹配（或未注册），-1 大小不匹配
 */
int msg_schema_check(const char* topic, size_t actual_size, const char* call_site);

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
