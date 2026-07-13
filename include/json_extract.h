#ifndef JSON_EXTRACT_H
#define JSON_EXTRACT_H

/**
 * @file json_extract.h
 * @brief 轻量级 JSON 标量提取工具 — 统一的 "从 JSON 字符串按 key 取值" 实现
 *
 * 背景：项目中多处需要从一段 JSON 文本里快速取出某个 key 对应的标量值
 * （数字/字符串/布尔/三元数组），但又不想为此引入完整 JSON 解析器的开销
 * （例如高频跨进程消息、离线数据集标注文件）。此前这些逻辑在
 * modules/adas_nodes/monitor_node.c 与 src/algorithms/nuscenes_loader.c 中
 * 各自手写了一份，实现细节（空白符容忍、截断行为、查找方式）并不一致。
 *
 * 本模块把这套"扁平化 JSON 标量提取"逻辑统一到一处，语义如下：
 *   - 按 "\"key\":" 做子串查找（非严格 JSON 解析，不处理嵌套同名 key 歧义）；
 *   - 冒号之后允许任意个空格/制表符/换行符；
 *   - key 不存在或类型不匹配时返回安全的默认值（0 / false / 空字符串），
 *     不会崩溃、不会读越界。
 *
 * 注意：这不是通用 JSON 解析器，仅适用于对性能/依赖敏感、且已知数据形状
 * 简单可控的场景。需要完整解析（嵌套对象、数组遍历等）时请使用 cJSON
 * （参见 src/core/scenario_loader.c / src/core/config_manager.c）。
 */

#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** 提取 double 类型字段，如 "spd":12.5。key 不存在时返回 0.0。 */
double json_extract_double(const char* json, const char* key);

/** 提取 int 类型字段，如 "n_obs":4。key 不存在时返回 0。 */
int json_extract_int(const char* json, const char* key);

/** 提取 bool 类型字段，如 "ok":true。key 不存在时返回默认值 default_val。 */
bool json_extract_bool(const char* json, const char* key, bool default_val);

/**
 * 提取带引号的字符串字段，如 "ot0":"car" → "car"。
 * dst 始终以 NUL 结尾；若 key 不存在，dst 被置为空字符串。
 * 超出 dst_size-1 的部分会被截断。
 */
void json_extract_string(const char* json, const char* key,
                          char* dst, size_t dst_size);

/**
 * 提取三元数值数组字段，如 "loc":[1.0, 2.0, 3.0]。
 * 成功返回 0；key 不存在或格式不符返回 -1（此时 a、b、c 均不被修改）。
 */
int json_extract_vec3(const char* json, const char* key,
                       double* a, double* b, double* c);

#ifdef __cplusplus
}
#endif

#endif /* JSON_EXTRACT_H */
