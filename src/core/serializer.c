/**
 * @file serializer.c
 * @brief 类型安全序列化层实现 (FlowEngine Phase 1)
 */

#include "serializer.h"
#include "error_codes.h"
#include <string.h>
#include <stdio.h>
#include <pthread.h>

/* ── FNV-1a Hash ────────────────────────────────────────── */

uint32_t fnv1a_hash(const uint8_t* data, size_t len) {
    uint32_t hash = FNV1A_INIT;
    for (size_t i = 0; i < len; i++) {
        hash = (hash ^ data[i]) * 0x01000193u;
    }
    return hash;
}

/* ── 字节序工具 ──────────────────────────────────────────── */

bool serializer_is_big_endian(void) {
    static const uint16_t probe = 0x1234;
    return *(const uint8_t*)&probe == 0x12;
}

uint8_t serializer_endian_marker(void) {
    return serializer_is_big_endian() ? ENDIAN_MARKER_BE : ENDIAN_MARKER_LE;
}

void serializer_swap16(uint8_t* v) {
    uint8_t t = v[0]; v[0] = v[1]; v[1] = t;
}

void serializer_swap32(uint8_t* v) {
    uint8_t t;
    t = v[0]; v[0] = v[3]; v[3] = t;
    t = v[1]; v[1] = v[2]; v[2] = t;
}

void serializer_swap64(uint8_t* v) {
    uint8_t t;
    t = v[0]; v[0] = v[7]; v[7] = t;
    t = v[1]; v[1] = v[6]; v[6] = t;
    t = v[2]; v[2] = v[5]; v[5] = t;
    t = v[3]; v[3] = v[4]; v[4] = t;
}

void serializer_ensure_endian(void* data, size_t size,
                              uint8_t endian_marker, EndianSwapFunc swap_func) {
    (void)size;
    if (endian_marker == 0) return;           /* 未知字节序，不处理 */
    if (endian_marker == ENDIAN_MARKER_BE && serializer_is_big_endian()) return;  /* 一致 */
    if (endian_marker == ENDIAN_MARKER_LE && !serializer_is_big_endian()) return; /* 一致 */
    if (swap_func) swap_func(data);            /* 需要交换 */
}

int serializer_normalize_endian(Message* msg) {
    if (!msg || msg->type_id == 0) return 0;  /* raw 类型无需处理 */
    const TypeRegistryEntry* entry = serializer_lookup_type(msg->type_id);
    if (!entry) return ERR_INVALID_PARAM;
    serializer_ensure_endian(msg->data, msg->data_size,
                             msg->endian_marker, entry->endian_swap);
    msg->endian_marker = serializer_endian_marker();  /* 标记为已转换 */
    return 0;
}

/* ── 类型注册表 ──────────────────────────────────────────── */

static TypeRegistryEntry  g_type_table[SERIALIZER_MAX_TYPE_ENTRIES];
static int                g_type_count = 0;
static pthread_mutex_t    g_type_mutex = PTHREAD_MUTEX_INITIALIZER;

int serializer_register_type(const TypeRegistryEntry* entry) {
    if (!entry || entry->type_id == 0) return ERR_INVALID_PARAM;

    pthread_mutex_lock(&g_type_mutex);

    /* 检查是否已注册（按 type_id），允许覆盖同 ID 的旧条目 */
    for (int i = 0; i < g_type_count; i++) {
        if (g_type_table[i].type_id == entry->type_id) {
            g_type_table[i] = *entry;
            pthread_mutex_unlock(&g_type_mutex);
            return 0;
        }
    }

    /* 新条目 */
    if (g_type_count >= SERIALIZER_MAX_TYPE_ENTRIES) {
        pthread_mutex_unlock(&g_type_mutex);
        fprintf(stderr, "[serializer] ERROR: type table full (%d entries), cannot register '%s'\n",
                SERIALIZER_MAX_TYPE_ENTRIES, entry->type_name);
        return ERR_INVALID_PARAM;
    }

    g_type_table[g_type_count++] = *entry;
    pthread_mutex_unlock(&g_type_mutex);
    return 0;
}

const TypeRegistryEntry* serializer_lookup_type(uint32_t type_id) {
    if (type_id == 0) return NULL;
    pthread_mutex_lock(&g_type_mutex);
    for (int i = 0; i < g_type_count; i++) {
        if (g_type_table[i].type_id == type_id) {
            pthread_mutex_unlock(&g_type_mutex);
            return &g_type_table[i];
        }
    }
    pthread_mutex_unlock(&g_type_mutex);
    return NULL;
}

const TypeRegistryEntry* serializer_lookup_by_name(const char* type_name) {
    if (!type_name) return NULL;
    pthread_mutex_lock(&g_type_mutex);
    for (int i = 0; i < g_type_count; i++) {
        if (strcmp(g_type_table[i].type_name, type_name) == 0) {
            pthread_mutex_unlock(&g_type_mutex);
            return &g_type_table[i];
        }
    }
    pthread_mutex_unlock(&g_type_mutex);
    return NULL;
}

int serializer_type_count(void) {
    pthread_mutex_lock(&g_type_mutex);
    int c = g_type_count;
    pthread_mutex_unlock(&g_type_mutex);
    return c;
}

/* ── 跨版本 schema 兼容性 ───────────────────────────────── */

SchemaCompat serializer_check_compat(const char* type_name,
                                     uint8_t their_version,
                                     uint32_t their_hash) {
    const TypeRegistryEntry* e = serializer_lookup_by_name(type_name);
    if (!e) return SCHEMA_UNKNOWN;

    /* 若双方都提供了 schema_hash 且一致 → 完全相同 */
    if (their_hash != 0 && e->schema_hash != 0 && their_hash == e->schema_hash) {
        return SCHEMA_IDENTICAL;
    }

    /* hash 缺失时退化为按版本判定 */
    if (their_hash == 0 || e->schema_hash == 0) {
        return (their_version == e->schema_version) ? SCHEMA_IDENTICAL
                                                    : SCHEMA_COMPATIBLE;
    }

    /* hash 不同：版本不同 = 演进（尽力兼容）；版本相同 = 破坏性变更 */
    if (their_version != e->schema_version) return SCHEMA_COMPATIBLE;
    return SCHEMA_INCOMPATIBLE;
}

const char* schema_compat_str(SchemaCompat c) {
    switch (c) {
        case SCHEMA_UNKNOWN:      return "unknown";
        case SCHEMA_IDENTICAL:    return "identical";
        case SCHEMA_COMPATIBLE:   return "compatible";
        case SCHEMA_INCOMPATIBLE: return "incompatible";
        default:                  return "?";
    }
}

const char* field_kind_str(uint8_t kind) {
    switch (kind) {
        case FIELD_KIND_BOOL:   return "bool";
        case FIELD_KIND_INT:    return "int";
        case FIELD_KIND_UINT:   return "uint";
        case FIELD_KIND_FLOAT:  return "float";
        case FIELD_KIND_ENUM:   return "enum";
        case FIELD_KIND_NESTED: return "nested";
        default:                return "unknown";
    }
}

/* ── 类型安全访问 ────────────────────────────────────────── */

const void* _msg_cast_impl(const Message* msg, uint32_t expected_type_id,
                           size_t expected_size, const char* type_name) {
    if (!msg) return NULL;

    /* 新格式：按 type_id 精确匹配 */
    if (msg->type_id != 0 && expected_type_id != 0) {
        if (msg->type_id == expected_type_id) {
            return msg->data;
        }
        /* type_id 不匹配 — 查找名称用于诊断 */
        const TypeRegistryEntry* expected = serializer_lookup_type(expected_type_id);
        const TypeRegistryEntry* actual   = serializer_lookup_type(msg->type_id);
        fprintf(stderr, "[serializer] WARNING: type mismatch on topic '%s': "
                "expected '%s'(id=0x%08x) but got '%s'(id=0x%08x)\n",
                msg->topic,
                expected ? expected->type_name : (type_name ? type_name : "?"),
                expected_type_id,
                actual   ? actual->type_name   : "unknown",
                msg->type_id);
        return NULL;
    }

    /* 旧格式（type_id == 0）或关闭 type_id 检查：回退到 size 检查 */
    if (expected_size > 0 && msg->data_size != expected_size) {
        fprintf(stderr, "[serializer] WARNING: size mismatch on topic '%s': "
                "expected %zu bytes but got %u bytes (type: %s, legacy mode)\n",
                msg->topic, expected_size, msg->data_size,
                type_name ? type_name : "?");
        return NULL;
    }

    return msg->data;
}

/* ── 类型安全消息构造 ────────────────────────────────────── */

void msg_init_typed(Message* msg, const char* topic, const char* sender,
                    uint32_t type_id, uint8_t schema_version,
                    const void* data, size_t data_size) {
    if (!msg) return;
    memset(msg, 0, sizeof(*msg));
    if (topic)  snprintf(msg->topic,  MSG_BUS_MAX_TOPIC_LEN,  "%s", topic);
    if (sender) snprintf(msg->sender, MSG_BUS_MAX_SENDER_LEN, "%s", sender);
    msg->type           = MSG_TYPE_PUBLISH;
    msg->data_size      = (uint32_t)data_size;
    msg->type_id        = type_id;
    msg->schema_version = schema_version;
    msg->endian_marker  = serializer_endian_marker();
    if (data && data_size > 0 && data_size <= MSG_BUS_MAX_DATA_SIZE) {
        memcpy(msg->data, data, data_size);
    }
}
