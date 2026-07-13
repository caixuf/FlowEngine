/**
 * json_extract.c — 轻量级 JSON 标量提取工具实现。
 *
 * 参见 include/json_extract.h 顶部注释了解设计动机与限制。
 */

#include "json_extract.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* 在 "key": 之后跳过空白符，定位到值的起始位置。
 *
 * JSON_EXTRACT_MAX_KEY_LEN 留出足够余量：项目内实际使用的最长 key 是
 * "category_name"/"num_lidar_pts"（13 字符）；64 字节远超此需求。
 * 若 key 超出该长度（不应发生），按未匹配处理，避免缓冲区溢出。 */
#define JSON_EXTRACT_MAX_KEY_LEN 64

static const char* find_value_start(const char* json, const char* key) {
    if (!json || !key) return NULL;

    char search[JSON_EXTRACT_MAX_KEY_LEN + 4]; /* +4: 两个引号 + 冒号 + NUL */
    int n = snprintf(search, sizeof(search), "\"%s\":", key);
    if (n < 0 || (size_t)n >= sizeof(search)) return NULL;

    const char* p = strstr(json, search);
    if (!p) return NULL;
    p += (size_t)n;
    while (*p == ' ' || *p == '\t' || *p == '\n' || *p == '\r') p++;
    return p;
}

double json_extract_double(const char* json, const char* key) {
    const char* p = find_value_start(json, key);
    if (!p) return 0.0;
    return atof(p);
}

int json_extract_int(const char* json, const char* key) {
    const char* p = find_value_start(json, key);
    if (!p) return 0;
    return (int)strtol(p, NULL, 10);
}

bool json_extract_bool(const char* json, const char* key, bool default_val) {
    const char* p = find_value_start(json, key);
    if (!p) return default_val;
    if (strncmp(p, "true", 4) == 0) return true;
    if (strncmp(p, "false", 5) == 0) return false;
    return default_val;
}

void json_extract_string(const char* json, const char* key,
                          char* dst, size_t dst_size) {
    if (!dst || dst_size == 0) return;
    dst[0] = '\0';

    const char* p = find_value_start(json, key);
    if (!p || *p != '"') return;
    p++;

    /* NOTE: does not handle backslash-escaped quotes (\") inside the value —
     * matches the pre-existing behavior this consolidates. Fine for the
     * simple, escape-free string values (names/tokens/categories) this is
     * used for; do not use this for values that may contain '"'. */
    size_t n = 0;
    while (*p && *p != '"' && n < dst_size - 1) dst[n++] = *p++;
    dst[n] = '\0';
}

int json_extract_vec3(const char* json, const char* key,
                       double* a, double* b, double* c) {
    if (!a || !b || !c) return -1;

    const char* p = find_value_start(json, key);
    if (!p || *p != '[') return -1;
    p++;

    /* NOTE: only the first 3 comma-separated numbers are read; a trailing
     * ']' is not required/validated (matches the pre-existing behavior this
     * consolidates). A 4-element array silently ignores the extra value,
     * and a missing ']' is not detected as an error. */
    double va = atof(p);
    p = strchr(p, ',');
    if (!p) return -1;
    p++;
    double vb = atof(p);
    p = strchr(p, ',');
    if (!p) return -1;
    p++;
    double vc = atof(p);

    *a = va;
    *b = vb;
    *c = vc;
    return 0;
}
