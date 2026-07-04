/**
 * param_registry.c — 参数系统实现
 */

#include "param_registry.h"
#include "error_codes.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <pthread.h>

static ParamEntry    g_params[PARAM_MAX_ENTRIES];
static int           g_param_count = 0;
static pthread_mutex_t g_mutex = PTHREAD_MUTEX_INITIALIZER;

static ParamEntry* find_or_create(const char* name) {
    for (int i = 0; i < g_param_count; i++)
        if (strcmp(g_params[i].name, name) == 0) return &g_params[i];
    if (g_param_count >= PARAM_MAX_ENTRIES) return NULL;
    ParamEntry* e = &g_params[g_param_count++];
    memset(e, 0, sizeof(*e));
    snprintf(e->name, PARAM_NAME_LEN, "%s", name);
    e->registered = true;
    return e;
}

/* ── Register ────────────────────────────────────────────── */

int param_register_int(const char* name, int64_t def, int64_t min, int64_t max, const char* desc) {
    if (!name) return ERR_INVALID_PARAM;
    pthread_mutex_lock(&g_mutex);
    ParamEntry* e = find_or_create(name);
    if (!e) { pthread_mutex_unlock(&g_mutex); return ERR_OVERFLOW; }
    e->type = PARAM_INT;
    e->default_value.int_val = def;
    e->current_value.int_val = def;
    e->min_value.int_val = min;
    e->max_value.int_val = max;
    if (desc) snprintf(e->description, PARAM_DESC_LEN, "%s", desc);
    pthread_mutex_unlock(&g_mutex);
    return 0;
}

int param_register_float(const char* name, double def, double min, double max, const char* desc) {
    if (!name) return ERR_INVALID_PARAM;
    pthread_mutex_lock(&g_mutex);
    ParamEntry* e = find_or_create(name);
    if (!e) { pthread_mutex_unlock(&g_mutex); return ERR_OVERFLOW; }
    e->type = PARAM_FLOAT;
    e->default_value.float_val = def;
    e->current_value.float_val = def;
    e->min_value.float_val = min;
    e->max_value.float_val = max;
    if (desc) snprintf(e->description, PARAM_DESC_LEN, "%s", desc);
    pthread_mutex_unlock(&g_mutex);
    return 0;
}

int param_register_bool(const char* name, bool def, const char* desc) {
    if (!name) return ERR_INVALID_PARAM;
    pthread_mutex_lock(&g_mutex);
    ParamEntry* e = find_or_create(name);
    if (!e) { pthread_mutex_unlock(&g_mutex); return ERR_OVERFLOW; }
    e->type = PARAM_BOOL;
    e->default_value.bool_val = def;
    e->current_value.bool_val = def;
    if (desc) snprintf(e->description, PARAM_DESC_LEN, "%s", desc);
    pthread_mutex_unlock(&g_mutex);
    return 0;
}

int param_register_string(const char* name, const char* def, const char* desc) {
    if (!name) return ERR_INVALID_PARAM;
    pthread_mutex_lock(&g_mutex);
    ParamEntry* e = find_or_create(name);
    if (!e) { pthread_mutex_unlock(&g_mutex); return ERR_OVERFLOW; }
    e->type = PARAM_STRING;
    if (def) snprintf(e->default_value.str_val, 64, "%s", def);
    if (def) snprintf(e->current_value.str_val, 64, "%s", def);
    if (desc) snprintf(e->description, PARAM_DESC_LEN, "%s", desc);
    pthread_mutex_unlock(&g_mutex);
    return 0;
}

int param_set_callback(const char* name, ParamChangeCallback cb, void* user_data) {
    if (!name) return ERR_INVALID_PARAM;
    pthread_mutex_lock(&g_mutex);
    ParamEntry* e = find_or_create(name);
    if (!e) { pthread_mutex_unlock(&g_mutex); return ERR_NOT_FOUND; }
    e->on_change = cb;
    e->change_user_data = user_data;
    pthread_mutex_unlock(&g_mutex);
    return 0;
}

/* ── Get ─────────────────────────────────────────────────── */

int64_t param_get_int(const char* name) {
    if (!name) return 0;
    pthread_mutex_lock(&g_mutex);
    for (int i = 0; i < g_param_count; i++)
        if (strcmp(g_params[i].name, name) == 0 && g_params[i].type == PARAM_INT) {
            int64_t v = g_params[i].current_value.int_val;
            pthread_mutex_unlock(&g_mutex);
            return v;
        }
    pthread_mutex_unlock(&g_mutex);
    return 0;
}

double param_get_float(const char* name) {
    if (!name) return 0.0;
    pthread_mutex_lock(&g_mutex);
    for (int i = 0; i < g_param_count; i++)
        if (strcmp(g_params[i].name, name) == 0 && g_params[i].type == PARAM_FLOAT) {
            double v = g_params[i].current_value.float_val;
            pthread_mutex_unlock(&g_mutex);
            return v;
        }
    pthread_mutex_unlock(&g_mutex);
    return 0.0;
}

bool param_get_bool(const char* name) {
    if (!name) return false;
    pthread_mutex_lock(&g_mutex);
    for (int i = 0; i < g_param_count; i++)
        if (strcmp(g_params[i].name, name) == 0 && g_params[i].type == PARAM_BOOL) {
            bool v = g_params[i].current_value.bool_val;
            pthread_mutex_unlock(&g_mutex);
            return v;
        }
    pthread_mutex_unlock(&g_mutex);
    return false;
}

const char* param_get_string(const char* name) {
    if (!name) return NULL;
    pthread_mutex_lock(&g_mutex);
    for (int i = 0; i < g_param_count; i++)
        if (strcmp(g_params[i].name, name) == 0 && g_params[i].type == PARAM_STRING) {
            pthread_mutex_unlock(&g_mutex);
            return g_params[i].current_value.str_val;
        }
    pthread_mutex_unlock(&g_mutex);
    return NULL;
}

/* ── Set (with validation) ────────────────────────────────── */

static int validate_and_set(ParamEntry* e, ParamValue new_val) {
    ParamValue old = e->current_value;
    e->current_value = new_val;

    if (e->on_change && e->hot_reload) {
        e->on_change(e->name, old, new_val, e->change_user_data);
    }
    return 0;
}

int param_set_int(const char* name, int64_t val) {
    if (!name) return ERR_INVALID_PARAM;
    pthread_mutex_lock(&g_mutex);
    for (int i = 0; i < g_param_count; i++) {
        ParamEntry* e = &g_params[i];
        if (strcmp(e->name, name) != 0 || e->type != PARAM_INT) continue;
        if (val < e->min_value.int_val || val > e->max_value.int_val) {
            pthread_mutex_unlock(&g_mutex);
            return ERR_INVALID_PARAM;
        }
        ParamValue v; v.int_val = val;
        int ret = validate_and_set(e, v);
        pthread_mutex_unlock(&g_mutex);
        return ret;
    }
    pthread_mutex_unlock(&g_mutex);
    return ERR_NOT_FOUND;
}

int param_set_float(const char* name, double val) {
    if (!name) return ERR_INVALID_PARAM;
    pthread_mutex_lock(&g_mutex);
    for (int i = 0; i < g_param_count; i++) {
        ParamEntry* e = &g_params[i];
        if (strcmp(e->name, name) != 0 || e->type != PARAM_FLOAT) continue;
        if (val < e->min_value.float_val || val > e->max_value.float_val) {
            pthread_mutex_unlock(&g_mutex);
            return ERR_INVALID_PARAM;
        }
        ParamValue v; v.float_val = val;
        int ret = validate_and_set(e, v);
        pthread_mutex_unlock(&g_mutex);
        return ret;
    }
    pthread_mutex_unlock(&g_mutex);
    return ERR_NOT_FOUND;
}

int param_set_bool(const char* name, bool val) {
    if (!name) return ERR_INVALID_PARAM;
    pthread_mutex_lock(&g_mutex);
    for (int i = 0; i < g_param_count; i++) {
        ParamEntry* e = &g_params[i];
        if (strcmp(e->name, name) != 0 || e->type != PARAM_BOOL) continue;
        ParamValue v; v.bool_val = val;
        int ret = validate_and_set(e, v);
        pthread_mutex_unlock(&g_mutex);
        return ret;
    }
    pthread_mutex_unlock(&g_mutex);
    return ERR_NOT_FOUND;
}

int param_set_string(const char* name, const char* val) {
    if (!name || !val) return ERR_INVALID_PARAM;
    pthread_mutex_lock(&g_mutex);
    for (int i = 0; i < g_param_count; i++) {
        ParamEntry* e = &g_params[i];
        if (strcmp(e->name, name) != 0 || e->type != PARAM_STRING) continue;
        ParamValue v;
        snprintf(v.str_val, 64, "%s", val);
        int ret = validate_and_set(e, v);
        pthread_mutex_unlock(&g_mutex);
        return ret;
    }
    pthread_mutex_unlock(&g_mutex);
    return ERR_NOT_FOUND;
}

/* ── Query ────────────────────────────────────────────────── */

const ParamEntry* param_get_entry(const char* name) {
    if (!name) return NULL;
    for (int i = 0; i < g_param_count; i++)
        if (strcmp(g_params[i].name, name) == 0) return &g_params[i];
    return NULL;
}

int param_list_all(ParamEntry* buf, int max) {
    if (!buf || max <= 0) return 0;
    int n = (g_param_count < max) ? g_param_count : max;
    memcpy(buf, g_params, (size_t)n * sizeof(ParamEntry));
    return n;
}

int param_count(void) { return g_param_count; }

int param_enable_hot_reload(const char* name) {
    if (!name) return ERR_INVALID_PARAM;
    for (int i = 0; i < g_param_count; i++)
        if (strcmp(g_params[i].name, name) == 0) {
            g_params[i].hot_reload = true;
            return 0;
        }
    return ERR_NOT_FOUND;
}

/* ── JSON Export ──────────────────────────────────────────── */

static const char* type_str(ParamType t) {
    switch (t) {
        case PARAM_INT:    return "int";
        case PARAM_FLOAT:  return "float";
        case PARAM_BOOL:   return "bool";
        case PARAM_STRING: return "string";
        default: return "unknown";
    }
}

char* param_export_json(void) {
    size_t sz = 8192;
    char* buf = (char*)malloc(sz);
    if (!buf) return NULL;
    int off = snprintf(buf, sz, "[");

    pthread_mutex_lock(&g_mutex);
    for (int i = 0; i < g_param_count; i++) {
        ParamEntry* e = &g_params[i];
        if ((size_t)off + 256 >= sz) { sz *= 2; buf = (char*)realloc(buf, sz); }

        const char* cur_str = "";
        char cur_buf[64];
        switch (e->type) {
            case PARAM_INT:    snprintf(cur_buf, 64, "%ld", (long)e->current_value.int_val); cur_str = cur_buf; break;
            case PARAM_FLOAT:  snprintf(cur_buf, 64, "%.1f", e->current_value.float_val); cur_str = cur_buf; break;
            case PARAM_BOOL:   cur_str = e->current_value.bool_val ? "true" : "false"; break;
            case PARAM_STRING: cur_str = e->current_value.str_val; break;
            default: cur_str = "?"; break;
        }

        off += snprintf(buf + off, sz - (size_t)off,
            "%s{\"name\":\"%s\",\"type\":\"%s\",\"value\":\"%s\",\"desc\":\"%s\",\"hot_reload\":%s}",
            i > 0 ? "," : "", e->name, type_str(e->type), cur_str, e->description,
            e->hot_reload ? "true" : "false");
    }
    pthread_mutex_unlock(&g_mutex);

    off += snprintf(buf + off, sz - (size_t)off, "]");
    return buf;
}
