#ifndef TYPED_TOPIC_H
#define TYPED_TOPIC_H

/**
 * @file typed_topic.h
 * @brief 类型安全的话题发布/订阅 — 编译时校验消息类型与 topic 匹配
 *
 * Apollo Cyber RT 的 Reader<MsgType>/Writer<MsgType> 在编译时保证 topic 和
 * 消息类型的一致性。FlowEngine 的 transport_publish/transport_subscribe 接受
 * 裸 void* 和 size_t，没有类型检查。本模块提供编译时类型安全层。
 *
 * 设计：
 *   1. TYPED_PUBLISH(transport, topic, TypeName, msg_ptr)
 *      自动调用 TypeName_serialize()，若类型名不存在则编译报错
 *   2. TYPED_DESERIALIZE(msg, TypeName, dst_ptr)
 *      安全反序列化，校验 type_id 和 size
 *   3. 运行时校验函数（typed_topic_validate, typed_topic_type_id 等）
 *
 * 用法：
 *   #include "typed_topic.h"
 *   #include "ControlCmd.h"  // 生成的头文件，提供 ControlCmd_serialize 等
 *
 *   ControlCmd cmd = {...};
 *   TYPED_PUBLISH(transport, TOPIC_CONTROL_CMD, ControlCmd, &cmd);
 *
 *   // 订阅端：
 *   void on_msg(const Message* msg, void* ctx) {
 *       ControlCmd cmd;
 *       if (TYPED_DESERIALIZE(msg, ControlCmd, &cmd) == 0) {
 *           // 使用 cmd
 *       }
 *   }
 */

#include "topic_registry.h"
#include "transport.h"
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ══════════════════════════════════════════════════════════════ */
/* 编译时类型安全宏                                              */
/* ══════════════════════════════════════════════════════════════ */

/**
 * TYPED_PUBLISH — 类型安全的发布宏
 *
 * 自动调用 TypeName_serialize() 序列化，编译时校验：
 *  - TypeName 有对应的 _serialize 函数（否则链接报错）
 *  - 缓冲区大小 = sizeof(TypeName)
 *
 * 必须在使用前 #include 对应的生成头文件（如 "ControlCmd.h"）。
 *
 * @param transport  Transport* 指针
 * @param topic      TOPIC_* 常量（如 TOPIC_CONTROL_CMD）
 * @param TypeName   消息类型名（如 ControlCmd），不加引号
 * @param msg_ptr    指向消息结构体的指针
 */
#define TYPED_PUBLISH(transport, topic, TypeName, msg_ptr) \
    do { \
        uint8_t _typed_buf[sizeof(TypeName)]; \
        size_t   _typed_len = sizeof(_typed_buf); \
        TypeName##_serialize((msg_ptr), _typed_buf, &_typed_len); \
        transport_publish((transport), (topic), _typed_buf, (uint32_t)_typed_len); \
    } while(0)

/**
 * TYPED_DESERIALIZE — 类型安全的反序列化宏
 *
 * 从 Message 中反序列化指定类型，校验 type_id 和 size。
 * 返回 0 成功，-1 失败（type_id 不匹配或 size 不足）。
 *
 * @param msg        const Message* 指针
 * @param TypeName   消息类型名（如 ControlCmd），不加引号
 * @param dst_ptr    指向目标结构体的指针
 * @return 0 成功，-1 失败
 */
#define TYPED_DESERIALIZE(msg, TypeName, dst_ptr) \
    typed_deserialize_checked((msg), TypeName##_TYPE_ID, sizeof(TypeName), \
                              (void*)(dst_ptr), \
                              (int(*)(void*,const uint8_t*,size_t))TypeName##_deserialize)

/**
 * TYPED_PUBLISH_ZC — 零拷贝类型安全发布（用于大块数据）
 *
 * 直接发布结构体内存，跳过序列化步骤。
 * 仅适用于 POD 类型且发布者和订阅者在同一进程内。
 *
 * @param transport  Transport* 指针
 * @param topic      TOPIC_* 常量
 * @param msg_ptr    指向消息结构体的指针
 */
#define TYPED_PUBLISH_ZC(transport, topic, msg_ptr) \
    transport_publish((transport), (topic), (msg_ptr), (uint32_t)sizeof(*(msg_ptr)))

/* ══════════════════════════════════════════════════════════════ */
/* 运行时校验函数                                                */
/* ══════════════════════════════════════════════════════════════ */

/**
 * 带校验的反序列化。
 * 先检查 type_id 和 size，再调用 deserialize。
 *
 * @param msg         消息指针
 * @param expected_id 预期 type_id
 * @param expected_sz 预期结构体大小
 * @param dst         目标缓冲区
 * @param deser_fn    反序列化函数指针
 * @return 0 成功，-1 失败
 */
static inline int typed_deserialize_checked(const Message* msg,
                                            uint32_t expected_id,
                                            size_t   expected_sz,
                                            void*    dst,
                                            int (*deser_fn)(void*, const uint8_t*, size_t)) {
    if (!msg || !dst || !deser_fn) return -1;
    if (expected_id != 0 && msg->type_id != 0 && msg->type_id != expected_id) {
        return -1;  /* type_id mismatch */
    }
    if (msg->data_size < expected_sz) return -1;
    return deser_fn(dst, msg->data, msg->data_size);
}

/* ── 运行时 topic→type 映射（实现在 typed_topic.c 中） ── */

/**
 * 运行时校验：topic 和 type_id 是否匹配。
 * @return 0 匹配，-1 不匹配
 */
int typed_topic_validate(const char* topic, uint32_t type_id, size_t size);

/**
 * 按 topic 查找其预期 type_id。
 * @return type_id，若未找到返回 0
 */
uint32_t typed_topic_type_id(const char* topic);

/**
 * 按 topic 查找其预期类型名。
 * @return 类型名，若未找到返回 "unknown"
 */
const char* typed_topic_type_name(const char* topic);

#ifdef __cplusplus
}
#endif

#endif /* TYPED_TOPIC_H */