#ifndef PROTO_SUPPORT_H
#define PROTO_SUPPORT_H

/**
 * @file proto_support.h
 * @brief Protobuf 适配层 — 可选集成 (FlowEngine)
 *
 * 编译开关: -DFLOWENGINE_USE_PROTOBUF
 *
 * 用法:
 *   1. 编写 .proto 文件，用 protobuf-c 生成 C 代码
 *   2. #include "proto_support.h"
 *   3. 使用 msg_init_from_pb() / msg_parse_to_pb() 在 Message 和 protobuf 之间转换
 *
 *   发布:
 *     MyMessage pb_msg = MY_MESSAGE__INIT;
 *     // ... fill pb_msg ...
 *     Message msg;
 *     msg_init_from_pb(&msg, "my/topic", "sender", &my_message__descriptor, &pb_msg);
 *     message_bus_publish(bus, msg.topic, msg.sender, msg.data, msg.data_size);
 *
 *   订阅:
 *     const Message* msg = ...;
 *     MyMessage* pb = my_message__unpack(NULL, msg->data_size, msg->data);
 *     // ... use pb ...
 *     my_message__free_unpacked(pb, NULL);
 *
 * Type ID 映射:
 *   每个 .proto message 自动获得 type_id = FNV-1a(descriptor->name)
 *   proto_support_register() 在启动时注册所有使用中的 proto 类型
 */

#include "message_bus.h"
#include "serializer.h"
#include <stdint.h>
#include <stddef.h>

#ifdef FLOWENGINE_USE_PROTOBUF

#include <protobuf-c/protobuf-c.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ══════════════════════════════════════════════════════════ */
/* Protobuf ↔ Message 转换                                    */
/* ══════════════════════════════════════════════════════════ */

/**
 * 从 protobuf message 构建 FlowEngine Message（发布用）。
 *
 * 自动:
 *   - 序列化 protobuf → msg->data（protobuf-c 的 pack 方法）
 *   - 设置 type_id = FNV-1a(descriptor->name)
 *   - 设置 schema_version = 1, endian_marker = LE
 *
 * @param msg        输出的 FlowEngine Message
 * @param topic      主题名
 * @param sender     发送者名
 * @param descriptor protobuf-c 生成的描述符 (如 &my_message__descriptor)
 * @param pb_msg     待序列化的 protobuf message 指针
 * @return 0 成功, -1 失败（数据过大）
 */
int msg_init_from_pb(Message* msg, const char* topic, const char* sender,
                     const ProtobufCMessageDescriptor* descriptor,
                     const ProtobufCMessage* pb_msg);

/**
 * 从 FlowEngine Message 解析为 protobuf message（订阅用）。
 *
 * 调用者负责释放返回的 pb_msg: 使用 descriptor->message_unpack 对应的 free 函数。
 *
 * @param msg        FlowEngine Message
 * @param descriptor protobuf-c 描述符
 * @return 解析后的 protobuf message 指针，失败返回 NULL
 */
ProtobufCMessage* msg_parse_to_pb(const Message* msg,
                                  const ProtobufCMessageDescriptor* descriptor);

/**
 * 对 Message 中的 payload 做 in-place protobuf 解析（零拷贝风格）。
 *
 * msg->data 必须是由 protobuf-c 的 pack 方法生成的连续内存。
 * 不分配新内存，直接返回指向 msg->data 的指针。
 *
 * @return true 如果 descriptor 匹配且可以安全解析
 */
bool msg_validate_pb_type(const Message* msg,
                          const ProtobufCMessageDescriptor* descriptor);

/* ══════════════════════════════════════════════════════════ */
/* 类型注册                                                    */
/* ══════════════════════════════════════════════════════════ */

/**
 * 为 protobuf message 类型注册 FlowEngine type_id。
 *
 * type_id = FNV-1a(descriptor->name)
 *
 * @param descriptor protobuf-c 描述符
 */
void proto_register_type(const ProtobufCMessageDescriptor* descriptor);

/**
 * 批量注册多个 protobuf 类型。
 * @param descriptors 以 NULL 结尾的描述符指针数组
 */
void proto_register_types(const ProtobufCMessageDescriptor** descriptors);

/**
 * 查找 protobuf 描述符对应的 FlowEngine type_id。
 */
uint32_t proto_get_type_id(const ProtobufCMessageDescriptor* descriptor);

#ifdef __cplusplus
}
#endif

#else /* !FLOWENGINE_USE_PROTOBUF */

/* Stub: 未启用 protobuf 时，这些函数不编译 */

#ifdef __cplusplus
extern "C" {
#endif

/* Stub that always fails */
static inline int msg_init_from_pb(void* msg, const char* topic, const char* sender,
                                   const void* descriptor, const void* pb_msg) {
    (void)msg; (void)topic; (void)sender; (void)descriptor; (void)pb_msg;
    return -1;
}

#ifdef __cplusplus
}
#endif

#endif /* FLOWENGINE_USE_PROTOBUF */

#endif /* PROTO_SUPPORT_H */
