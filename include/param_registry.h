#ifndef PARAM_REGISTRY_H
#define PARAM_REGISTRY_H

/**
 * @file param_registry.h
 * @brief 参数系统 — 注册、校验、查询、热更新
 *
 * 用法:
 *   param_registry_register_int("control.max_speed", 120, 0, 200, "Max speed km/h");
 *   param_registry_register_float("fusion.max_delta_ms", 50.0, 10.0, 500.0, "Time window");
 *   param_registry_register_bool("control.emergency_brake", true, "Enable AEB");
 *
 *   int speed = param_get_int("control.max_speed");
 *   param_set_int("control.max_speed", 100);  // validated against [0,200]
 *
 *   flowctl param list
 *   flowctl param get control.max_speed
 *   flowctl param set control.max_speed 100
 */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PARAM_MAX_ENTRIES    128
#define PARAM_NAME_LEN       64
#define PARAM_DESC_LEN       128

/* ── Param types ─────────────────────────────────────────── */

typedef enum {
    PARAM_INT    = 0,
    PARAM_FLOAT  = 1,
    PARAM_BOOL   = 2,
    PARAM_STRING = 3,
} ParamType;

typedef union {
    int64_t  int_val;
    double   float_val;
    bool     bool_val;
    char     str_val[64];
} ParamValue;

/* ── Hot-reload callback ─────────────────────────────────── */

typedef void (*ParamChangeCallback)(const char* name, ParamValue old_val,
                                    ParamValue new_val, void* user_data);

/* ── Param entry ─────────────────────────────────────────── */

typedef struct {
    char        name[PARAM_NAME_LEN];
    ParamType   type;
    ParamValue  default_value;
    ParamValue  current_value;
    ParamValue  min_value;
    ParamValue  max_value;
    char        description[PARAM_DESC_LEN];
    bool        hot_reload;          /**< true = supports runtime change */
    ParamChangeCallback on_change;   /**< called when value changes */
    void*       change_user_data;
    bool        registered;
} ParamEntry;

/* ══════════════════════════════════════════════════════════ */
/* Register                                                    */
/* ══════════════════════════════════════════════════════════ */

int param_register_int(const char* name, int64_t default_val,
                       int64_t min_val, int64_t max_val, const char* desc);
int param_register_float(const char* name, double default_val,
                         double min_val, double max_val, const char* desc);
int param_register_bool(const char* name, bool default_val, const char* desc);
int param_register_string(const char* name, const char* default_val, const char* desc);

/**
 * Set hot-reload callback. Called when param value changes at runtime.
 */
int param_set_callback(const char* name, ParamChangeCallback cb, void* user_data);

/* ══════════════════════════════════════════════════════════ */
/* Get/Set                                                     */
/* ══════════════════════════════════════════════════════════ */

int64_t param_get_int(const char* name);
double  param_get_float(const char* name);
bool    param_get_bool(const char* name);
const char* param_get_string(const char* name);

/** Set with validation. Returns 0 on success, error code on failure. */
int param_set_int(const char* name, int64_t val);
int param_set_float(const char* name, double val);
int param_set_bool(const char* name, bool val);
int param_set_string(const char* name, const char* val);

/* ══════════════════════════════════════════════════════════ */
/* Query                                                       */
/* ══════════════════════════════════════════════════════════ */

const ParamEntry* param_get_entry(const char* name);
int param_list_all(ParamEntry* buf, int max);
int param_count(void);

/** Enable hot-reload for a param (allows runtime changes) */
int param_enable_hot_reload(const char* name);

/* ══════════════════════════════════════════════════════════ */
/* Export                                                      */
/* ══════════════════════════════════════════════════════════ */

char* param_export_json(void);

#ifdef __cplusplus
}
#endif

#endif /* PARAM_REGISTRY_H */
