/**
 * nmea_parser.h — NMEA 0183 GPS 报文解析器（真实传感器格式接入）
 *
 * 解析真实 GNSS 接收机通过串口/UDP 输出的标准 NMEA 0183 语句，
 * 将其转换为 FlowEngine 内部的 `GpsData` 消息。
 *
 * 支持的语句类型（兼容 GP/GN/GL/GA/BD 等 talker id）：
 *   - $--GGA : 定位质量、经纬度、海拔、卫星数、HDOP
 *   - $--RMC : 推荐最小定位信息（经纬度、地速、航向、日期时间、有效性）
 *
 * 典型用法（逐行喂入串口读到的每一行）：
 *   NmeaParser p;
 *   nmea_parser_init(&p);
 *   GpsData gps;
 *   if (nmea_parse_line(&p, "$GPRMC,...*3F", &gps) == NMEA_OK) {
 *       // gps 已填充最新经纬度/速度/航向
 *   }
 *
 * 说明：
 *   - 校验和（*HH）若存在会被强制校验，不匹配返回 NMEA_ERR_CHECKSUM。
 *   - GGA 与 RMC 字段互补：解析器内部维护累积状态，
 *     每成功解析一条会把已知字段合并进输出，避免单条语句字段缺失。
 *   - 经纬度按 NMEA ddmm.mmmm / dddmm.mmmm 格式转换为十进制度，
 *     并根据 N/S、E/W 象限取正负号。
 *   - RMC 地速单位为节(knots)，转换为 m/s（1 knot = 0.514444 m/s）。
 */

#ifndef NMEA_PARSER_H
#define NMEA_PARSER_H

/* GpsData 可能来自手写头 (adas_msgs.h) 或代码生成头 (adas_msgs_gen.h)。
 * 两者定义相同但不可同时包含，故此处优先复用已包含的生成头，否则回退手写头。 */
#if defined(ADAS_MSGS_GEN_H)
/* GpsData already provided by generated message header */
#else
#include "adas_msgs.h"
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── 返回码 ──────────────────────────────────────────────────── */
typedef enum {
    NMEA_OK            = 0,   /**< 成功解析并更新了 out */
    NMEA_ERR_ARG       = -1,  /**< 参数非法（NULL 等） */
    NMEA_ERR_FORMAT    = -2,  /**< 非 NMEA 语句（缺少 '$'）或字段不足 */
    NMEA_ERR_CHECKSUM  = -3,  /**< 校验和不匹配 */
    NMEA_ERR_UNSUPPORTED = -4,/**< 语句类型不支持（非 GGA/RMC） */
    NMEA_ERR_NO_FIX    = -5,  /**< 语句有效但当前无定位（未更新坐标） */
} NmeaStatus;

/* ── 解析器状态（累积 GGA/RMC 字段）─────────────────────────── */
typedef struct {
    GpsData  last;            /**< 最近一次合并后的 GPS 数据 */
    bool     has_position;    /**< 是否已获得过有效经纬度 */
    bool     has_velocity;    /**< 是否已获得过有效速度/航向 */
    uint32_t sentences_ok;    /**< 成功解析的语句计数 */
    uint32_t sentences_bad;   /**< 解析失败（校验/格式）的语句计数 */
} NmeaParser;

/* ── API ─────────────────────────────────────────────────────── */

/** 初始化解析器（清零状态） */
void nmea_parser_init(NmeaParser* p);

/**
 * 计算 NMEA 语句的校验和（'$' 与 '*' 之间所有字符的 XOR）。
 * @param sentence 完整语句（可含或不含前导 '$' 与尾部 *HH）
 * @return 8-bit 校验和
 */
uint8_t nmea_checksum(const char* sentence);

/**
 * 解析单条 NMEA 语句并把结果合并进解析器状态与 out。
 * @param p    解析器（维护累积状态）
 * @param line 单条语句（允许含尾部 CR/LF）
 * @param out  输出：成功时填充为当前累积的 GpsData（可为 NULL）
 * @return NmeaStatus
 */
int nmea_parse_line(NmeaParser* p, const char* line, GpsData* out);

/** 返回当前累积的 GPS 数据快照（未解析成功过则为全 0）。 */
const GpsData* nmea_parser_last(const NmeaParser* p);

#ifdef __cplusplus
}
#endif

#endif /* NMEA_PARSER_H */
