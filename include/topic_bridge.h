#ifndef TOPIC_BRIDGE_H
#define TOPIC_BRIDGE_H

/**
 * @file topic_bridge.h
 * @brief 跨进程 Topic 桥接 — 显式把一组 topic 在两个独立进程间对拷
 *
 * ── 定位 ──────────────────────────────────────────────────
 * 这是一个 **接口契约（scaffold）**，用于 EVOLUTION_ROADMAP 的
 * Phase 10「多进程仿真部署验证」（plan 主线 A3）。
 *
 * 与 `transport.h` 的区别：
 *   - `Transport` 是**自动路由**层：调用方不关心数据在哪，框架自选 local/IPC/TCP。
 *   - `TopicBridge` 是**显式桥接**层：把本进程 bus 上指定的一组 topic，
 *     单向/双向镜像到另一个进程的 bus。用于"多进程仿真部署"时，
 *     把 sim_world / perception / planning 拆到不同进程后仍能互通。
 *
 * 典型用法（进程 A 发布 sensor/lidar，进程 B 消费）：
 *
 *   进程 A:
 *     TopicBridge* b = topic_bridge_create(busA, "flowengine.sim", TOPIC_BRIDGE_PUB);
 *     topic_bridge_add_topic(b, "sensor/lidar", LIDARFRAME_TYPE_ID);
 *     topic_bridge_start(b);
 *
 *   进程 B:
 *     TopicBridge* b = topic_bridge_create(busB, "flowengine.sim", TOPIC_BRIDGE_SUB);
 *     topic_bridge_add_topic(b, "sensor/lidar", LIDARFRAME_TYPE_ID);
 *     topic_bridge_start(b);   // sensor/lidar 现在会出现在 busB 上
 *
 * ── 实现指引（给后续实现者） ────────────────────────────────
 * 建议实现文件：src/core/topic_bridge.c，并加入 CMakeLists 的 flowengine_core。
 * 复用已有能力：
 *   - `ipc_channel.h`   —— 共享内存传输（同机跨进程）。
 *   - `serializer.h`    —— 出站 message_bus_publish 前序列化 / 入站反序列化。
 *   - `message_bus.h`   —— 本进程收发（PUB 侧订阅本地 topic；SUB 侧向本地发布）。
 * 所有函数当前**未实现**，返回 ERR_OK/句柄即视为契约占位；实现前请勿在生产链路依赖。
 * 详见 docs/IMPLEMENTATION_GUIDE.md（A3 小节）与 skills/ 下 IPC 教程。
 */

#include "message_bus.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define TOPIC_BRIDGE_CHANNEL_LEN  64
#define TOPIC_BRIDGE_MAX_TOPICS   64

/* ── 桥接方向 ─────────────────────────────────────────────── */

typedef enum {
    TOPIC_BRIDGE_PUB = 0,  /**< 本地 bus → IPC（导出本进程 topic 给别的进程） */
    TOPIC_BRIDGE_SUB = 1,  /**< IPC → 本地 bus（把别的进程 topic 注入本进程） */
    TOPIC_BRIDGE_BIDIR = 2 /**< 双向镜像（谨慎使用，注意回环去重） */
} TopicBridgeDirection;

/* ── 运行时统计（可用于 flowctl / dashboard 边标注） ───────── */

typedef struct {
    uint64_t forwarded;     /**< 成功跨进程转发的消息数 */
    uint64_t dropped;       /**< 因队列满 / 序列化失败丢弃数 */
    uint64_t bytes;         /**< 累计转发字节数 */
    uint64_t last_ts_us;    /**< 最近一次转发的时间戳（微秒） */
} TopicBridgeStats;

/* ── 不透明句柄 ──────────────────────────────────────────── */

typedef struct TopicBridge TopicBridge;

/* ══════════════════════════════════════════════════════════ */
/* 生命周期                                                    */
/* ══════════════════════════════════════════════════════════ */

/**
 * 创建一个 topic 桥接器（尚未启动）。
 *
 * @param bus          本进程消息总线（不可为 NULL）。
 * @param channel_name IPC 通道名，两个进程必须一致（如 "flowengine.sim"）。
 * @param direction    桥接方向，见 ::TopicBridgeDirection。
 * @return 句柄；失败返回 NULL。调用者负责 topic_bridge_destroy()。
 *
 * @note 契约占位：实现前返回 NULL 亦属合法。
 */
TopicBridge* topic_bridge_create(MessageBus* bus, const char* channel_name,
                                 TopicBridgeDirection direction);

/**
 * 注册一条需要桥接的 topic。可在 start 前多次调用。
 *
 * @param bridge   桥接句柄。
 * @param topic    topic 名（如 "sensor/lidar"）。
 * @param type_id  FNV-1a 类型 ID（用于两端 schema 一致性校验）。
 * @return ERR_OK 成功；ERR_OVERFLOW 超过 TOPIC_BRIDGE_MAX_TOPICS；
 *         ERR_INVALID_PARAM 参数非法。
 */
int topic_bridge_add_topic(TopicBridge* bridge, const char* topic, uint32_t type_id);

/**
 * 启动桥接：PUB 侧开始订阅本地 topic 并向 IPC 转发；
 * SUB 侧开始从 IPC 收包并向本地 bus 发布。
 *
 * @return ERR_OK 成功；ERR_IO IPC 通道建立失败；ERR_BUSY 已启动。
 */
int topic_bridge_start(TopicBridge* bridge);

/**
 * 停止桥接（可再次 start）。幂等。
 */
int topic_bridge_stop(TopicBridge* bridge);

/**
 * 销毁桥接器并释放所有资源（内部会先 stop）。
 */
void topic_bridge_destroy(TopicBridge* bridge);

/* ══════════════════════════════════════════════════════════ */
/* 内省                                                        */
/* ══════════════════════════════════════════════════════════ */

/**
 * 读取某条已桥接 topic 的运行时统计。
 *
 * @param bridge  桥接句柄。
 * @param topic   topic 名。
 * @param out     输出统计（不可为 NULL）。
 * @return ERR_OK 成功；ERR_NOT_FOUND topic 未注册。
 */
int topic_bridge_get_stats(const TopicBridge* bridge, const char* topic,
                           TopicBridgeStats* out);

/**
 * 已注册桥接 topic 的数量。
 */
int topic_bridge_topic_count(const TopicBridge* bridge);

#ifdef __cplusplus
}
#endif

#endif /* TOPIC_BRIDGE_H */
