/**
 * json_schema.c — 严格 JSON 提取与 Schema 校验实现。
 *
 * 复用 json_extract 的 find_value_start 定位逻辑，做类型校验与缺失检测。
 */

#include "json_schema.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

/* NOTE: no log.h dependency — this is a leaf module used by everything
 * from test code to plugins. Logging is the caller's responsibility
 * (use the bool return value + err_field). */

#define JSON_SCHEMA_MAX_KEY_LEN 64

/* Internal helper — same logic as json_extract.c find_value_start.
 * Kept in sync to avoid a public header dependency.
 * Note: tolerant to spaces around the colon: `"x":1` and `"x" : 1` both work. */
static const char* find_value_start_internal(const char* json, const char* key) {
    if (!json || !key) return NULL;
    char search[JSON_SCHEMA_MAX_KEY_LEN + 4];
    int n = snprintf(search, sizeof(search), "\"%s\"", key);
    if (n < 0 || (size_t)n >= sizeof(search)) return NULL;
    const char* p = strstr(json, search);
    if (!p) return NULL;
    p += (size_t)n;
    /* Skip spaces and a single optional colon (with surrounding spaces). */
    while (*p == ' ' || *p == '\t' || *p == '\n' || *p == '\r') p++;
    if (*p == ':') p++;
    while (*p == ' ' || *p == '\t' || *p == '\n' || *p == '\r') p++;
    return p;
}

/* ── 类型探测 ──────────────────────────────────────────── */

static bool value_is_number(const char* p) {
    if (!p) return false;
    if (*p == '-' || *p == '+') p++;
    if (!*p || !(isdigit((unsigned char)*p))) return false;
    return true;
}

static bool value_is_integer(const char* p) {
    if (!p) return false;
    if (*p == '-' || *p == '+') p++;
    if (!*p) return false;
    const char* start = p;
    while (*p && isdigit((unsigned char)*p)) p++;
    if (p == start) return false;
    /* Strict int: no decimal point or exponent (avoids 12.5 being accepted as int). */
    if (*p == '.' || *p == 'e' || *p == 'E') return false;
    return true;
}

static bool value_is_string(const char* p) {
    return p && *p == '"';
}

static bool value_is_bool(const char* p) {
    if (!p) return false;
    return (strncmp(p, "true", 4) == 0) ||
           (strncmp(p, "false", 5) == 0);
}

bool json_field_ok(const char* json, const char* key, JsonFieldType type) {
    const char* p = find_value_start_internal(json, key);
    if (!p) return false;
    switch (type) {
        case JSON_TYPE_DOUBLE: return value_is_number(p);
        case JSON_TYPE_INT:    return value_is_integer(p);
        case JSON_TYPE_STRING: return value_is_string(p);
        case JSON_TYPE_BOOL:   return value_is_bool(p);
    }
    return false;
}

/* ── 严格提取 ──────────────────────────────────────────── */

bool json_get_double_strict(const char* json, const char* key, double* out) {
    if (!out) return false;
    if (!json_field_ok(json, key, JSON_TYPE_DOUBLE)) return false;
    *out = atof(find_value_start_internal(json, key));
    return true;
}

bool json_get_int_strict(const char* json, const char* key, int* out) {
    if (!out) return false;
    if (!json_field_ok(json, key, JSON_TYPE_INT)) return false;
    *out = (int)strtol(find_value_start_internal(json, key), NULL, 10);
    return true;
}

bool json_get_string_strict(const char* json, const char* key,
                            char* dst, size_t dst_size) {
    if (!dst || dst_size == 0) return false;
    dst[0] = '\0';
    if (!json_field_ok(json, key, JSON_TYPE_STRING)) return false;
    const char* p = find_value_start_internal(json, key);
    p++; /* skip opening quote */
    size_t n = 0;
    while (*p && *p != '"' && n < dst_size - 1) dst[n++] = *p++;
    dst[n] = '\0';
    return true;
}

/* ── 批量校验 ──────────────────────────────────────────── */

static const char* type_name(JsonFieldType t) {
    switch (t) {
        case JSON_TYPE_DOUBLE: return "double";
        case JSON_TYPE_INT:    return "int";
        case JSON_TYPE_STRING: return "string";
        case JSON_TYPE_BOOL:   return "bool";
    }
    return "?";
}

bool json_validate(const char* json, const char* topic,
                   const JsonFieldDef* fields,
                   char* err_field, size_t err_size) {
    if (!json || !fields) {
        if (err_field && err_size > 0) snprintf(err_field, err_size, "(null json/schema)");
        if (topic) fprintf(stderr, "[%s] WARN schema: null json or fields\n", topic);
        return false;
    }
    for (const JsonFieldDef* f = fields; f->key != NULL; f++) {
        if (!f->required) continue;
        if (!json_field_ok(json, f->key, f->type)) {
            if (err_field && err_size > 0) snprintf(err_field, err_size, "%s", f->key);
            if (topic) {
                fprintf(stderr,
                        "[%s] WARN schema mismatch: missing/invalid field '%s' (expected %s)\n",
                        topic, f->key, type_name(f->type));
            }
            return false;
        }
    }
    return true;
}

/* ════════════════════════════════════════════════════════════════
 *  DSL 提取 (key=value 文本格式) — 解决 control/planning strstr+sscanf
 * ════════════════════════════════════════════════════════════════ */

/* 在 dsl 中查找 "key" 后面那个 value 的起始位置。
 * 支持 key=value / key = value / key:value 三种分隔符。
 * 注意：按"首次出现"语义；key 作为子串可能误匹配 — 这是当前 DLS 协议的
 * 已知限制（与 json_extract 一致）。 */
static const char* dsl_find_value_start(const char* dsl, const char* key) {
    if (!dsl || !key) return NULL;
    size_t klen = strlen(key);
    if (klen == 0 || klen > JSON_SCHEMA_MAX_KEY_LEN) return NULL;
    const char* p = dsl;
    while ((p = strstr(p, key)) != NULL) {
        /* 左侧必须是边界（开头、空格、tab、左括号等非字母数字） */
        bool left_ok;
        if (p == dsl) {
            left_ok = true;
        } else {
            unsigned char prev = (unsigned char)*(p - 1);
            left_ok = !(isalnum(prev) || prev == '_');
        }
        if (!left_ok) { p += klen; continue; }
        /* 右侧必须是分隔符 = 或 : （含可选空白） */
        const char* q = p + klen;
        while (*q == ' ' || *q == '\t') q++;
        if (*q == '=' || *q == ':') {
            q++;
            while (*q == ' ' || *q == '\t') q++;
            return q;
        }
        p += klen;
    }
    return NULL;
}

static bool dsl_value_is_number(const char* p) {
    if (!p) return false;
    if (*p == '-' || *p == '+') p++;
    if (!*p || !(isdigit((unsigned char)*p))) return false;
    return true;
}

bool dsl_get_double_strict(const char* dsl, const char* key, double* out) {
    if (!out) return false;
    const char* p = dsl_find_value_start(dsl, key);
    if (!p || !dsl_value_is_number(p)) return false;
    *out = strtod(p, NULL);
    return true;
}

const char* dsl_find_value(const char* dsl, const char* key) {
    return dsl_find_value_start(dsl, key);
}

bool dsl_get_int_strict(const char* dsl, const char* key, int* out) {
    if (!out) return false;
    const char* p = dsl_find_value_start(dsl, key);
    if (!p || !dsl_value_is_number(p)) return false;
    /* strict int: 必须整数字面量（无 . e E） */
    const char* q = (*p == '-' || *p == '+') ? p + 1 : p;
    if (!isdigit((unsigned char)*q)) return false;
    while (isdigit((unsigned char)*q)) q++;
    if (*q == '.' || *q == 'e' || *q == 'E') return false;
    *out = (int)strtol(p, NULL, 10);
    return true;
}

static bool dsl_field_ok(const char* dsl, const char* key, JsonFieldType type) {
    const char* p = dsl_find_value_start(dsl, key);
    if (!p) return false;
    switch (type) {
        case JSON_TYPE_DOUBLE:
        case JSON_TYPE_INT:
            return dsl_value_is_number(p);
        case JSON_TYPE_STRING:
            return *p == '"';
        case JSON_TYPE_BOOL:
            return (strncmp(p, "true", 4) == 0) ||
                   (strncmp(p, "false", 5) == 0);
    }
    return false;
}

bool dsl_validate(const char* dsl, const char* topic,
                  const DslFieldDef* fields,
                  char* err_field, size_t err_size) {
    if (!dsl || !fields) {
        if (err_field && err_size > 0) snprintf(err_field, err_size, "(null dsl/schema)");
        if (topic) fprintf(stderr, "[%s] WARN dsl schema: null dsl or fields\n", topic);
        return false;
    }
    for (const DslFieldDef* f = fields; f->key != NULL; f++) {
        if (!f->required) continue;
        if (!dsl_field_ok(dsl, f->key, f->type)) {
            if (err_field && err_size > 0) snprintf(err_field, err_size, "%s", f->key);
            if (topic) {
                fprintf(stderr,
                        "[%s] WARN dsl schema mismatch: missing/invalid field '%s' (expected %s)\n",
                        topic, f->key, type_name(f->type));
            }
            return false;
        }
    }
    return true;
}
