#ifndef BAG_H
#define BAG_H

/**
 * @file bag.h
 * @brief 消息录制与回放（Bag）
 *
 * 文件格式 v2（带类型安全 + 索引）：
 *   [Header: magic("FLB_") 4B | version 4B | msg_count 8B | duration_us 8B |
 *            index_offset 8B | reserved 32B]
 *   [Records: type_id(4B)|schema_ver(1B)|endian(1B)|ts(8B)|topic_len(1B)|
 *             topic(N)|data_size(4B)|data(N)] × msg_count
 *   [Index: entry_count(8B) | entries[topic(64B)|count(8B)|first_off(8B)|last_off(8B)] |
 *            crc32(4B)]
 *
 * 向后兼容：reader 自动检测旧格式（前 4 字节 ≠ "FLB_"），fallback 到 legacy 解析。
 *
 * 录制：
 *   BagWriter* w = bag_writer_open("out.bag");
 *   bag_writer_attach(w, bus);       // 自动录制所有 topic
 *   bag_writer_close(w);
 *
 * 回放：
 *   BagReader* r = bag_reader_open("out.bag");
 *   bag_reader_play(r, bus, 1.0f);  // 1.0 = 实时速度
 *   bag_reader_close(r);
 */

#include "message_bus.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Writer ───────────────────────────────────────────── */

typedef struct BagWriter BagWriter;

/**
 * 打开 bag 文件准备录制
 * @param path  输出文件路径
 * @return 写入器指针，失败返回 NULL
 */
BagWriter* bag_writer_open(const char* path);

/**
 * 手动写入一条消息（用于自定义录制逻辑）
 */
int bag_writer_write(BagWriter* w, const Message* msg);

/**
 * 订阅总线上的所有 topic，自动录制（非阻塞，启动后台录制线程）
 * @return 0 成功
 */
int bag_writer_attach(BagWriter* w, MessageBus* bus);

/**
 * 停止录制并关闭文件
 */
void bag_writer_close(BagWriter* w);

/* ── Reader ───────────────────────────────────────────── */

typedef struct BagReader BagReader;

/**
 * 打开 bag 文件准备回放
 */
BagReader* bag_reader_open(const char* path);

/**
 * 回放整个 bag 到 message_bus
 * @param bus    目标总线（若为 NULL 则仅打印消息）
 * @param speed  回放速度倍数（1.0 = 实时，0 = 尽快回放）
 * @return 回放的消息条数，-1 失败
 */
int bag_reader_play(BagReader* r, MessageBus* bus, float speed);

/**
 * 带过滤器的回放
 * @param bus          目标总线（若为 NULL 则仅打印消息）
 * @param speed        回放速度倍数（1.0 = 实时，0 = 尽快回放）
 * @param topic_filter 仅回放匹配的 topic（NULL 或 "*" 表示全部）
 * @param start_us     起始时间戳（0 表示从头）
 * @param end_us       结束时间戳（0 表示到末尾）
 * @return 回放的消息条数，-1 失败
 */
int bag_reader_play_filtered(BagReader* r, MessageBus* bus, float speed,
                             const char* topic_filter,
                             uint64_t start_us, uint64_t end_us);

/**
 * 获取 bag 文件元数据（topic 列表、总时长、消息总数）
 * @param r          Reader 指针
 * @param msg_count  输出：消息总数（可为 NULL）
 * @param duration_us 输出：总时长（微秒，可为 NULL）
 * @return 0 成功，-1 失败
 */
int bag_reader_info(BagReader* r, uint64_t* msg_count, uint64_t* duration_us);

/**
 * 获取 bag 文件中的 topic 列表及其消息计数。
 * @param topics    输出缓冲区（每个 topic 64 字节）
 * @param max_count 最多返回的 topic 数
 * @param counts    各 topic 的消息计数（可为 NULL）
 * @return 实际 topic 数，-1 失败
 */
int bag_reader_get_topics(BagReader* r, char topics[][64], int max_count,
                          uint64_t* counts);

/**
 * 获取指定 topic 的类型信息（仅 v2 格式有效）。
 * @param type_id     输出：FNV-1a 类型 ID（可为 NULL）
 * @param schema_ver  输出：schema 版本号（可为 NULL）
 * @return 0 成功，-1 未找到或不适用
 */
int bag_reader_get_type_info(BagReader* r, const char* topic,
                             uint32_t* type_id, uint8_t* schema_ver);

/**
 * 关闭 bag 文件
 */
void bag_reader_close(BagReader* r);

#ifdef __cplusplus
}
#endif

#endif /* BAG_H */
