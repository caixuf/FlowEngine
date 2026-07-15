#ifndef JSON_SCHEMA_H
#define JSON_SCHEMA_H

/**
 * @file json_schema.h
 * @brief JSON Schema 强制校验 — 解决 strstr+sscanf 的"静默容错"问题
 *
 * 背景：项目内 100+ 处 strstr+sscanf JSON 字段解析，字段缺失时静默默认 0。
 * 后果：上游节点发布错误（字段拼写错、类型不匹配、字段丢失）时，下游节点
 * 不会报错，使用 0 值继续运行 — 难定位的"数据哑火"问题。
 *
 * 本模块提供"严格提取"版本：key 不存在、类型不匹配、JSON 无效时返回错误
 * 并 LOG_WARN。同时提供批量 schema 校验 API，一次性检查多个必填字段。
 *
 * 设计原则：
 *   - 复用 json_extract 的扁平化查找逻辑（性能/依赖一致）
 *   - API 命名带 _strict 后缀以避免和宽容版本混淆
 *   - 错误信息带 topic + key 上下文，方便定位
 */

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * 严格提取 double。key 缺失或值不能解析为数字返回 false，*out 保持不变。
 * 调用方应检查返回值。
 */
bool json_get_double_strict(const char* json, const char* key, double* out);

/** 严格提取 int。key 缺失或值不能解析为整数返回 false。 */
bool json_get_int_strict(const char* json, const char* key, int* out);

/**
 * 严格提取字符串。key 缺失或不是带引号字符串返回 false，dst 置空。
 */
bool json_get_string_strict(const char* json, const char* key,
                            char* dst, size_t dst_size);

/* ── Schema 批量校验 ─────────────────────────────────────────── */

/** 字段类型 */
typedef enum {
    JSON_TYPE_DOUBLE = 1,
    JSON_TYPE_INT    = 2,
    JSON_TYPE_STRING = 3,
    JSON_TYPE_BOOL   = 4
} JsonFieldType;

/** 单个字段定义 */
typedef struct {
    const char*    key;     /**< 字段名（不含引号） */
    JsonFieldType  type;    /**< 期望类型 */
    bool           required;/**< true 则缺失时报错；false 仅做格式检查 */
} JsonFieldDef;

/**
 * 批量校验 JSON 是否符合 schema。
 * - json        : 待校验 JSON 文本
 * - topic       : 错误信息中包含的 topic 名（用于日志定位）
 * - fields      : 字段定义数组，以 {NULL, 0, 0} 结尾
 * - err_field   : 可选；失败时填入首个缺失/类型错误字段名（调用方提供缓冲区）
 * - err_size    : err_field 缓冲区大小
 *
 * 返回：所有 required=true 字段都存在且类型正确返回 true；
 *       否则返回 false 并 LOG_WARN(topic, "schema mismatch: missing/invalid '%s'").
 */
bool json_validate(const char* json, const char* topic,
                   const JsonFieldDef* fields,
                   char* err_field, size_t err_size);

/* ── 校验 helper：单字段 ──────────────────────────────────────── */

/**
 * 检查 JSON 中某 key 是否存在且类型符合预期。
 * 返回 true=OK；false=key 缺失或类型不匹配。
 */
bool json_field_ok(const char* json, const char* key, JsonFieldType type);

/* ── DSL 提取 (key=value 文本格式) ──────────────────────────────
 *
 * 项目内 control/planning 节点实际传输的是 key=value DSL（不是 JSON）：
 *   "pos=(x,y) gps=(lat,lon) speed=5.0 dt=XXus"
 * 旧实现用 strstr+sscanf 静默容错。下面 API 提供"严格"版本：
 * 找不到 key 时不修改 *out 并返回 false，调用方可记录/告警。
 *
 * 容忍格式：
 *   "speed=5.0" / "speed = 5.0" / "speed:5.0" 都能正确解析
 *   数值解析规则同 json_get_double_strict（不接受 bool/null/字符串）
 */

bool dsl_get_double_strict(const char* dsl, const char* key, double* out);
bool dsl_get_int_strict(const char* dsl, const char* key, int* out);

/**
 * 找到 dsl 中 "key" 之后 value 的起始指针（已是首个非空白字符），
 * 返回 NULL 表示 key 不存在。返回的指针可直接交给 strtof/strtol。
 *
 * 用于解析非标量值，例如 key=(a,b) 元组：调用方拿到指针后判断
 * 首个字符是否为 '('，然后自行解析。语义与 dsl_get_double_strict 一致。
 */
const char* dsl_find_value(const char* dsl, const char* key);

/* ── 消息总线 Schema 钩子 (transport 边界) ─────────────────────── */

/**
 * 一次性校验多字段 DSL：fields 数组以 {NULL, 0, 0} 结尾。
 * 行为同 json_validate，但作用于 key=value 格式。
 */
typedef struct {
    const char*    key;
    JsonFieldType  type;
    bool           required;
} DslFieldDef;

bool dsl_validate(const char* dsl, const char* topic,
                  const DslFieldDef* fields,
                  char* err_field, size_t err_size);

#ifdef __cplusplus
}
#endif

#endif /* JSON_SCHEMA_H */
