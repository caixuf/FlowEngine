#ifndef BAG_H
#define BAG_H

/**
 * @file bag.h
 * @brief 消息录制与回放（Bag）
 *
 * 文件格式（纯二进制流，无外部依赖）：
 *   每条记录 = [timestamp_us(8B) | topic_len(1B) | topic | data_size(4B) | data]
 *
 * 录制：
 *   BagWriter* w = bag_writer_open("out.bag");
 *   // 订阅通配符 "*" 自动录制所有 topic
 *   bag_writer_record(w, bus);      // 阻塞，直到调用 bag_writer_close
 *   bag_writer_close(w);
 *
 * 回放：
 *   BagReader* r = bag_reader_open("out.bag");
 *   bag_reader_play(r, bus, 1.0f);  // 1.0 = 实时速度，2.0 = 2倍速
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
 * 关闭 bag 文件
 */
void bag_reader_close(BagReader* r);

#ifdef __cplusplus
}
#endif

#endif /* BAG_H */
