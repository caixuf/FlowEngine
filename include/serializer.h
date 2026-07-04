#ifndef SERIALIZER_H
#define SERIALIZER_H

/**
 * @file serializer.h
 * @brief 类型安全的序列化层 (FlowEngine Phase 1)
 *
 * 提供：
 *   - FNV-1a 编译期/运行时类型 ID 生成
 *   - 类型注册表：类型名 -> type_id -> 序列化/反序列化函数
 *   - 类型安全的 Message 访问器 msg_cast<T>()
 *   - 字节序检测和交换
 *
 * 典型用法（C）：
 *   MSG_REGISTER_TYPE("sensor/lidar", LIDAR_FRAME_TYPE_ID, sizeof(LidarFrame),
 *                     lidar_frame_serialize, lidar_frame_deserialize);
 *   const LidarFrame* f = msg_cast(msg, LIDAR_FRAME_TYPE_ID);
 *
 * 典型用法（C++）：
 *   const LidarFrame* f = msg_cast<LidarFrame>(msg);
 */

#include "message_bus.h"
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── 常量 ────────────────────────────────────────────────── */

#define SERIALIZER_MAX_TYPE_ENTRIES  128
#define SERIALIZER_TYPE_NAME_LEN     64

/* ── 字节序标记 ──────────────────────────────────────────── */

#define ENDIAN_MARKER_LE  0x12  /**< Little-endian */
#define ENDIAN_MARKER_BE  0x21  /**< Big-endian */

/* ── FNV-1a Hash ────────────────────────────────────────── */

/**
 * 计算数据的 FNV-1a 32-bit hash。
 * hash = 2166136261; for each byte: hash = (hash ^ byte) * 16777619
 */
uint32_t fnv1a_hash(const uint8_t* data, size_t len);

/** FNV-1a 初始值，用于递进式计算 */
#define FNV1A_INIT  0x811c9dc5u

/** FNV-1a 单字节递进 */
static inline uint32_t fnv1a_byte(uint32_t hash, uint8_t byte) {
    return (hash ^ byte) * 0x01000193u;
}

/** FNV-1a 连续字节递进 */
static inline uint32_t fnv1a_update(uint32_t hash, const uint8_t* data, size_t len) {
    for (size_t i = 0; i < len; i++) {
        hash = (hash ^ data[i]) * 0x01000193u;
    }
    return hash;
}

/* ── 类型注册表 ──────────────────────────────────────────── */

/**
 * 序列化函数签名：
 *   将 src 指向的结构体序列化到 buf，写入的字节数存到 *size。
 *   buf 为 NULL 时仅计算所需大小存入 *size（类似 snprintf(NULL, 0)）。
 *   返回 0 成功，-1 失败。
 */
typedef int (*SerializeFunc)(const void* src, uint8_t* buf, size_t* size);

/**
 * 反序列化函数签名：
 *   从 buf 读取 size 字节，反序列化到 dst。
 *   返回 0 成功，-1 失败（数据损坏、类型不匹配等）。
 */
typedef int (*DeserializeFunc)(void* dst, const uint8_t* buf, size_t size);

/**
 * 字节序交换函数：原地交换数据为当前平台字节序。
 */
typedef void (*EndianSwapFunc)(void* data);

/* ── 字段级 schema 描述 ─────────────────────────────────── */

/** 字段基础种类（用于自描述展示，与序列化无关） */
typedef enum {
    FIELD_KIND_UNKNOWN = 0,
    FIELD_KIND_BOOL,
    FIELD_KIND_INT,      /**< 有符号整型 */
    FIELD_KIND_UINT,     /**< 无符号整型 */
    FIELD_KIND_FLOAT,    /**< 浮点（float/double） */
    FIELD_KIND_ENUM,     /**< 枚举（序列化为 int8） */
    FIELD_KIND_NESTED,   /**< 嵌套结构体 */
} FieldKind;

/**
 * 字段描述（由 codegen 生成的静态表），实现字段级 schema。
 * 用于自描述、schema 展示与跨版本兼容诊断。
 */
typedef struct {
    const char* name;       /**< 字段名 */
    uint8_t     kind;       /**< FieldKind */
    uint16_t    offset;     /**< 在 struct 中的字节偏移（offsetof） */
    uint16_t    elem_size;  /**< 单元素序列化字节数 */
    uint16_t    array_len;  /**< 数组长度（标量为 1） */
} FieldDesc;

/** 类型注册条目 */
typedef struct {
    uint32_t        type_id;           /**< FNV-1a hash */
    uint8_t         schema_version;    /**< 当前 schema 版本 */
    size_t          struct_size;       /**< sizeof(struct) */
    char            type_name[SERIALIZER_TYPE_NAME_LEN];
    SerializeFunc   serialize;         /**< NULL = raw memcpy（旧类型） */
    DeserializeFunc deserialize;       /**< NULL = raw memcpy（旧类型） */
    EndianSwapFunc  endian_swap;       /**< NULL = 无需交换 */
    uint32_t        schema_hash;       /**< 字段级布局哈希（0 = 未提供） */
    const FieldDesc* fields;           /**< 字段描述表（NULL = 未提供） */
    uint16_t        field_count;       /**< fields 表元素数 */
} TypeRegistryEntry;

/**
 * 注册一个类型到全局类型表。
 * 通常在模块初始化时调用（或由代码生成的 _register_type() 函数调用）。
 * @return 0 成功，-1 表已满
 */
int serializer_register_type(const TypeRegistryEntry* entry);

/**
 * 按 type_id 查找类型。
 * @return 找到的条目指针，未找到返回 NULL
 */
const TypeRegistryEntry* serializer_lookup_type(uint32_t type_id);

/**
 * 按名称查找类型。
 * @return 找到的条目指针，未找到返回 NULL
 */
const TypeRegistryEntry* serializer_lookup_by_name(const char* type_name);

/**
 * 获取已注册类型数量。
 */
int serializer_type_count(void);

/* ── 跨版本 schema 兼容性 ───────────────────────────────── */

/**
 * schema 兼容性判定结果。
 */
typedef enum {
    SCHEMA_UNKNOWN = 0,     /**< 本地未注册该类型，无法判定 */
    SCHEMA_IDENTICAL,       /**< schema_hash 完全一致 */
    SCHEMA_COMPATIBLE,      /**< 布局不同但版本演进，可尽力兼容（补零/截断） */
    SCHEMA_INCOMPATIBLE,    /**< 同版本布局却不同 = 破坏性变更，不可兼容 */
} SchemaCompat;

/**
 * 判断对端（例如 bag 记录或远端节点）的 schema 是否与本地注册的类型兼容。
 *
 * 兼容性策略：
 *   - 本地未注册该类型名          → SCHEMA_UNKNOWN
 *   - schema_hash 相同            → SCHEMA_IDENTICAL（完全一致）
 *   - hash 不同且版本号不同        → SCHEMA_COMPATIBLE
 *       （视为字段在尾部增删的版本演进，反序列化端补零/截断尽力兼容）
 *   - hash 不同但版本号相同        → SCHEMA_INCOMPATIBLE
 *       （同一版本却有不同布局，属未升版的破坏性变更，拒绝）
 *
 * @param type_name     类型名（稳定标识，跨版本不变）
 * @param their_version 对端 schema 版本号
 * @param their_hash    对端 schema_hash（0 = 未知，退化为按版本判定）
 * @return SchemaCompat 判定结果
 */
SchemaCompat serializer_check_compat(const char* type_name,
                                     uint8_t their_version,
                                     uint32_t their_hash);

/** SchemaCompat 的可读字符串 */
const char* schema_compat_str(SchemaCompat c);

/** FieldKind 的可读字符串 */
const char* field_kind_str(uint8_t kind);

/* ── 类型安全的 Message 访问 ────────────────────────────── */

/**
 * 从 Message 中安全获取类型化数据指针。
 *
 * 检查规则（按顺序）：
 *   1. 如果 msg->type_id != 0（新格式）：匹配 type_id
 *   2. 如果 msg->type_id == 0（旧格式/raw）：回退到 size 检查
 *   3. size 不匹配时打印 warning 并返回 NULL
 *
 * @param msg             消息
 * @param expected_type_id 期望的类型 ID（0 = 关闭检查）
 * @param expected_size   期望的 struct 大小（0 = 关闭 size 检查）
 * @param type_name       类型名（用于诊断日志）
 * @return 类型化数据指针，检查失败返回 NULL
 */
const void* _msg_cast_impl(const Message* msg, uint32_t expected_type_id,
                           size_t expected_size, const char* type_name);

/**
 * C 层 msg_cast 宏：
 *
 *   const MyStruct* p = msg_cast(msg, MY_STRUCT_TYPE_ID, sizeof(MyStruct));
 */
#define msg_cast(msg, type_id, type_size) \
    ((const void*)_msg_cast_impl((msg), (type_id), (type_size), #type_id))

/* ── 字节序工具 ──────────────────────────────────────────── */

/** 检测当前平台是否为 big-endian */
bool serializer_is_big_endian(void);

/** 获取当前平台对应的 endian_marker 值 */
uint8_t serializer_endian_marker(void);

/** 16/32/64 位字节序交换 */
void serializer_swap16(uint8_t* v);
void serializer_swap32(uint8_t* v);
void serializer_swap64(uint8_t* v);

/**
 * 如果需要（平台不匹配）进行结构体的端到端字节序交换。
 * 假定数据按 field_marker 标记的字节序存储。
 * 如果当前平台字节序与 field_marker 一致，则什么也不做。
 *
 * @param data         数据指针
 * @param size         数据大小
 * @param endian_marker 数据存储的字节序标记 (ENDIAN_MARKER_LE/BE)
 * @param swap_func    该类型的字节序交换函数（可为 NULL）
 */
void serializer_ensure_endian(void* data, size_t size,
                              uint8_t endian_marker, EndianSwapFunc swap_func);

/**
 * 将 Message 载荷中的类型化数据转换为当前平台字节序。
 * 自动查询类型注册表获取 swap 函数。
 * @param msg 消息（需有有效的 type_id）
 * @return 0 成功（已原地转换），-1 失败（类型未注册）
 */
int serializer_normalize_endian(Message* msg);

/* ── Message 序列化工具 ─────────────────────────────────── */

/**
 * 构建一个类型安全的消息（C API）。
 * 自动设置 type_id、schema_version、endian_marker。
 *
 * @param msg            输出消息
 * @param topic          主题
 * @param sender         发送者
 * @param type_id        类型 ID
 * @param schema_version schema 版本
 * @param data           类型化数据指针
 * @param data_size      数据大小
 */
void msg_init_typed(Message* msg, const char* topic, const char* sender,
                    uint32_t type_id, uint8_t schema_version,
                    const void* data, size_t data_size);

#ifdef __cplusplus
}
#endif

/* ── C++ 模板辅助 ────────────────────────────────────────── */

#ifdef __cplusplus

#include <type_traits>

/* 主模板声明：generated 头文件会为每个消息类型做特化
 * (template<> struct msg_traits<T> { ... })。这里声明主模板，
 * 使 generated 头文件可安全地被 C++ TU 包含。 */
template<typename T> struct msg_traits;

/* ── Compile-time FNV-1a (C++ only) ─────────────────────── */
namespace ser {
    constexpr uint32_t fnv1a_const(const char* s, size_t len) {
        uint32_t h = FNV1A_INIT;
        for (size_t i = 0; i < len; i++) h = (h ^ (uint8_t)s[i]) * 0x01000193u;
        return h;
    }
    constexpr uint32_t fnv1a_const(const char* s) {
        return fnv1a_const(s, __builtin_strlen(s));
    }
}

/* C++ template overrides the C macro version */
#undef msg_cast

/**
 * C++ 类型特征的 msg_cast<T> 版本。
 *
 * 用法：
 *   const LidarFrame* f = msg_cast<LidarFrame>(msg);
 */
template<typename T>
inline const T* msg_cast(const Message* msg) {
    return static_cast<const T*>(
        _msg_cast_impl(msg, T::TYPE_ID, sizeof(T), T::TYPE_NAME));
}

/**
 * 获取当前平台 endian_marker 的模板版本。
 */
inline uint8_t endian_marker() {
    return serializer_endian_marker();
}

#endif /* __cplusplus */

#endif /* SERIALIZER_H */
