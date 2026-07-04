#pragma once
#include <stdint.h>
#include <stdarg.h>

typedef struct McapWriter McapWriter;

/** 创建 MCAP 文件并写 Header。profile: "x-json" 或 "ros2" 等 */
McapWriter* mcap_writer_open(const char* path, const char* profile);

/** 注册 channel，返回 channel_id（>=1）。
 *  schema_json: JSON Schema 字符串，Foxglove 用来自动推断消息结构。
 *  encoding 固定为 "json"，schema_name 用于标识消息类型。 */
uint16_t    mcap_writer_register_channel(McapWriter* w, const char* topic,
                                         const char* schema_name,
                                         const char* schema_json);

/** 写一条 JSON 消息。log_time_ns=0 用当前系统时间。 */
int         mcap_writer_write_msg(McapWriter* w, uint16_t channel_id,
                                  uint64_t log_time_ns,
                                  const char* json_data, uint32_t json_len);

/** printf 风格便捷写入 */
int         mcap_writer_write_json(McapWriter* w, uint16_t channel_id,
                                   uint64_t log_time_ns,
                                   const char* fmt, ...);

/** 关闭文件，写入 Footer 和 trailing magic */
int         mcap_writer_close(McapWriter** pw);

/* ── 全局单例 ────────────────────────────────────────────── */
McapWriter* mcap_writer_global(void);
void        mcap_writer_set_global(McapWriter* w);
