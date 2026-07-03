#ifndef IPC_CHANNEL_H
#define IPC_CHANNEL_H

/**
 * @file ipc_channel.h
 * @brief 跨进程共享内存通信通道
 *
 * 基于 POSIX shm_open + mmap + 命名信号量实现，
 * 复用现有 Message 结构，接口对齐 message_bus。
 *
 * 典型用法（发布进程）：
 *   IpcChannel* ch = ipc_channel_open("adas_lidar", IPC_ROLE_PUBLISHER, 32);
 *   ipc_channel_publish(ch, "sensor/lidar", "lidar_drv", &frame, sizeof(frame));
 *
 * 典型用法（订阅进程）：
 *   IpcChannel* ch = ipc_channel_open("adas_lidar", IPC_ROLE_SUBSCRIBER, 32);
 *   ipc_channel_subscribe(ch, on_lidar, NULL);
 *   ipc_channel_spin(ch);   // 阻塞循环，收到消息后调用回调
 */

#include "message_bus.h"   /* 复用 Message, MessageCallback */
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── 角色 ─────────────────────────────────────────────── */
typedef enum {
    IPC_ROLE_PUBLISHER  = 0,   /**< 创建/写入共享内存 */
    IPC_ROLE_SUBSCRIBER = 1,   /**< 打开/读取共享内存 */
} IpcRole;

/* ── 不透明句柄 ───────────────────────────────────────── */
typedef struct IpcChannel IpcChannel;

/**
 * 打开（或创建）一个 IPC 通道
 * @param channel_name  通道名称（用作 shm 名称前缀，无需 '/' 前缀）
 * @param role          IPC_ROLE_PUBLISHER 或 IPC_ROLE_SUBSCRIBER
 * @param queue_depth   环形缓冲区容量（建议 8~64）
 * @return 通道指针，失败返回 NULL
 */
IpcChannel* ipc_channel_open(const char* channel_name, IpcRole role,
                              uint32_t queue_depth);

/**
 * 关闭通道并释放资源
 * Publisher 关闭时会 unlink 共享内存和信号量。
 */
void ipc_channel_close(IpcChannel* ch);

/**
 * 发布消息（仅 Publisher 调用）
 * @return 0 成功，-1 失败（队列满或角色错误）
 */
int ipc_channel_publish(IpcChannel* ch, const char* topic, const char* sender,
                        const void* data, uint32_t size);

/**
 * 注册订阅回调（仅 Subscriber 调用，可注册多个）
 * @return 0 成功，-1 失败
 */
int ipc_channel_subscribe(IpcChannel* ch, MessageCallback callback, void* user_data);

/**
 * 启动后台接收线程（Subscriber 调用，非阻塞）
 * @return 0 成功，非0 失败
 */
int ipc_channel_start(IpcChannel* ch);

/**
 * 停止后台接收线程
 */
void ipc_channel_stop(IpcChannel* ch);

/**
 * 阻塞读取一条消息并投递给所有回调（Subscriber 调用）
 * @param timeout_ms  超时毫秒；0 = 永不超时
 * @return 0 收到消息，-1 超时或错误
 */
int ipc_channel_recv_once(IpcChannel* ch, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* IPC_CHANNEL_H */
