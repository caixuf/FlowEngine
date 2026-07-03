#ifndef ERROR_CODES_H
#define ERROR_CODES_H

/**
 * @file error_codes.h
 * @brief 统一错误码 (FlowEngine)
 *
 * 所有模块返回 int，0=OK，负数=错误。
 * 每个模块有自己的错误码范围，便于快速定位问题。
 *
 * 用法：
 *   int ret = serializer_register_type(&entry);
 *   if (ret != ERR_OK) {
 *       fprintf(stderr, "Failed: %s\n", err_str(ret));
 *   }
 */

/** 通用 OK */
#define ERR_OK  0

/* ── 通用错误 (-1 ~ -9) ──────────────────────────────────── */

#define ERR_INVALID_PARAM  (-1)   /**< 参数非法（NULL/越界） */
#define ERR_NOT_FOUND      (-2)   /**< 未找到 */
#define ERR_TIMEOUT        (-3)   /**< 超时 */
#define ERR_BUSY           (-4)   /**< 资源忙/已运行 */
#define ERR_NOMEM          (-5)   /**< 内存不足 */
#define ERR_PERM           (-6)   /**< 权限不足 */
#define ERR_OVERFLOW       (-7)   /**< 缓冲区满 */
#define ERR_IO             (-8)   /**< I/O错误 */
#define ERR_INTERNAL       (-9)   /**< 内部错误 */

/* ── 序列化 (-10 ~ -19) ──────────────────────────────────── */

#define ERR_TYPE_MISMATCH  (-10)  /**< 类型ID不匹配 */
#define ERR_SIZE_MISMATCH  (-11)  /**< 数据大小不匹配 */
#define ERR_ENDIAN_CONFLICT (-12) /**< 字节序不兼容 */
#define ERR_SERIALIZE_FAIL (-13)  /**< 序列化失败 */
#define ERR_DESERIALIZE_FAIL (-14)/**< 反序列化失败 */
#define ERR_TABLE_FULL     (-15)  /**< 类型表已满 */

/* ── 调度器 (-20 ~ -29) ──────────────────────────────────── */

#define ERR_RATE_LIMITED   (-20)  /**< 频率限制 */
#define ERR_QUOTA_EXCEEDED (-21)  /**< 配额超限 */
#define ERR_CPU_BIND_FAIL  (-22)  /**< CPU绑定失败 */
#define ERR_PRIO_SET_FAIL  (-23)  /**< 优先级设置失败 */

/* ── 状态机 (-30 ~ -39) ──────────────────────────────────── */

#define ERR_ILLEGAL_TRANSITION (-30) /**< 非法状态转移 */
#define ERR_GUARD_REJECTED     (-31) /**< Guard条件拒绝 */
#define ERR_RULE_NOT_FOUND     (-32) /**< 转移规则未找到 */
#define ERR_RULE_EXISTS        (-33) /**< 规则已存在 */

/* ── 服务发现 (-40 ~ -49) ────────────────────────────────── */

#define ERR_NODE_NOT_FOUND  (-40)  /**< 节点未找到 */
#define ERR_TOPIC_NOT_FOUND (-41)  /**< Topic未找到 */
#define ERR_DEP_TIMEOUT     (-42)  /**< 依赖等待超时 */
#define ERR_NETWORK_FAIL    (-43)  /**< 网络错误 */

/* ── 融合 (-50 ~ -59) ────────────────────────────────────── */

#define ERR_NO_DATA         (-50)  /**< 无可对齐数据 */
#define ERR_BUFFER_EMPTY    (-51)  /**< 缓冲为空 */
#define ERR_ALIGNMENT_FAIL  (-52)  /**< 时间对齐失败 */

/* ── 字符串转换 ────────────────────────────────────────────── */

/**
 * 将错误码转换为人类可读字符串。
 * 返回静态常量字符串，不要 free。
 */
static inline const char* err_str(int code) {
    switch (code) {
        case ERR_OK:                 return "OK";
        case ERR_INVALID_PARAM:      return "INVALID_PARAM";
        case ERR_NOT_FOUND:          return "NOT_FOUND";
        case ERR_TIMEOUT:            return "TIMEOUT";
        case ERR_BUSY:               return "BUSY";
        case ERR_NOMEM:              return "NOMEM";
        case ERR_PERM:               return "PERM";
        case ERR_OVERFLOW:           return "OVERFLOW";
        case ERR_IO:                 return "IO";
        case ERR_INTERNAL:           return "INTERNAL";
        case ERR_TYPE_MISMATCH:      return "TYPE_MISMATCH";
        case ERR_SIZE_MISMATCH:      return "SIZE_MISMATCH";
        case ERR_TABLE_FULL:         return "TABLE_FULL";
        case ERR_RATE_LIMITED:       return "RATE_LIMITED";
        case ERR_QUOTA_EXCEEDED:     return "QUOTA_EXCEEDED";
        case ERR_ILLEGAL_TRANSITION: return "ILLEGAL_TRANSITION";
        case ERR_GUARD_REJECTED:     return "GUARD_REJECTED";
        case ERR_NODE_NOT_FOUND:     return "NODE_NOT_FOUND";
        case ERR_DEP_TIMEOUT:        return "DEP_TIMEOUT";
        case ERR_NETWORK_FAIL:       return "NETWORK_FAIL";
        case ERR_NO_DATA:            return "NO_DATA";
        case ERR_BUFFER_EMPTY:       return "BUFFER_EMPTY";
        case ERR_ALIGNMENT_FAIL:     return "ALIGNMENT_FAIL";
        default:                     return "UNKNOWN_ERROR";
    }
}

#endif /* ERROR_CODES_H */
