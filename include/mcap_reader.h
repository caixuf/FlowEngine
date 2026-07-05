/**
 * mcap_reader.h — 轻量 MCAP 回放器
 *
 * 读取 FlowEngine 录制的 .mcap 文件（JSON 编码），
 * 按 topic 和 timestamp 顺序回放消息。
 *
 * 用法:
 *   McapReader* r = mcap_reader_open("demo.mcap");
 *   McapMessage msg;
 *   while (mcap_reader_next(r, &msg)) {
 *       // 通过 transport 发布 msg.topic, msg.data, msg.data_len
 *   }
 *   mcap_reader_close(r);
 */

#ifndef MCAP_READER_H
#define MCAP_READER_H

#include <stdint.h>
#include <stddef.h>
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MCAP_READER_MAX_CHANNELS 64
#define MCAP_READER_MAX_MSG_LEN  (64 * 1024)  /* 64KB max single message */

/* ── 单条消息 ────────────────────────────────────────────────── */

typedef struct {
    uint16_t channel_id;
    char     topic[128];
    uint64_t log_time_ns;    /**< 录制时间戳 */
    uint64_t publish_time_ns;
    uint32_t sequence;
    uint8_t  data[MCAP_READER_MAX_MSG_LEN];
    uint32_t data_len;
} McapMessage;

/* ── Channel 信息 ────────────────────────────────────────────── */

typedef struct {
    uint16_t id;
    uint16_t schema_id;
    char     topic[128];
    char     schema_name[64];
    uint64_t message_count;
} McapChannelInfo;

/* ── Reader 实例 ─────────────────────────────────────────────── */

typedef struct {
    FILE*    fp;
    char     profile[32];
    uint64_t data_start;         /**< 第一条 Message 的文件偏移 */
    uint64_t file_size;

    McapChannelInfo channels[MCAP_READER_MAX_CHANNELS];
    uint16_t channel_count;

    /* 回放控制 */
    uint64_t base_time_ns;       /**< 第一条消息的时间戳 */
    double   speed;              /**< 回放速度倍率 (1.0=实时) */
    int      loop;               /**< 循环回放 */
} McapReader;

/* ── API ────────────────────────────────────────────────────── */

/**
 * 打开 MCAP 文件准备回放。
 * @param path  .mcap 文件路径
 * @return reader 实例，失败返回 NULL
 */
McapReader* mcap_reader_open(const char* path);

/**
 * 读取下一条消息。
 * @param r     reader
 * @param msg   输出消息（调用者提供内存）
 * @return      1=成功, 0=文件结束, -1=错误
 */
int mcap_reader_next(McapReader* r, McapMessage* msg);

/**
 * 跳到文件开头重新回放。
 */
int mcap_reader_seek_start(McapReader* r);

/**
 * 获取 channel 数量。
 */
int mcap_reader_channel_count(const McapReader* r);

/**
 * 获取 channel 信息。
 */
const McapChannelInfo* mcap_reader_get_channel(const McapReader* r, int idx);

/**
 * 关闭并释放资源。
 */
void mcap_reader_close(McapReader* r);

#ifdef __cplusplus
}
#endif

#endif /* MCAP_READER_H */
