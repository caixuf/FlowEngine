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
 * search 缓冲区固定为 96 字节，足以容纳项目内实际使用的 key 长度；
 * 若 key 过长（不应发生），按未匹配处理，避免溢出。 */
static const char* find_value_start(const char* json, const char* key) {
    if (!json || !key) return NULL;

    char search[96];
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
