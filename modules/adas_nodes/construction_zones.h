/**
 * construction_zones.h — scene/frame 施工区 AABB 解析（header-only）
 *
 * planning / behavior 共用：可通行域与掉头触发点都依赖同一份权威几何，
 * 禁止两处各抄一份 cJSON 字段读取（schema 变更会静默分叉）。
 *
 * 消息字段（scene/frame.construction_zones[]）：
 *   x, y, length, width  — 中心 + 轴对齐尺寸 (m)
 * 前缘（顺行 +x）：front_x = x - length/2
 */
#ifndef FLOWENGINE_CONSTRUCTION_ZONES_H
#define FLOWENGINE_CONSTRUCTION_ZONES_H

#include <cjson/cJSON.h>

#ifdef __cplusplus
extern "C" {
#endif

/** 从 scene/frame 根对象解析 construction_zones 到调用方缓冲。
 *  @return 写入条数（≤ max_n）；root 无效或无数组时返回 0。 */
static inline int construction_zones_parse_from_scene_root(
    const cJSON* root,
    double* xs, double* ys, double* lens, double* wids,
    int max_n)
{
    if (!root || !xs || !ys || !lens || !wids || max_n <= 0) return 0;
    const cJSON* czs = cJSON_GetObjectItemCaseSensitive((cJSON*)root, "construction_zones");
    if (!cJSON_IsArray(czs)) return 0;
    int n_out = 0;
    const int n = cJSON_GetArraySize((cJSON*)czs);
    for (int i = 0; i < n && n_out < max_n; ++i) {
        const cJSON* z = cJSON_GetArrayItem((cJSON*)czs, i);
        if (!z) continue;
        const cJSON* jx = cJSON_GetObjectItemCaseSensitive((cJSON*)z, "x");
        const cJSON* jy = cJSON_GetObjectItemCaseSensitive((cJSON*)z, "y");
        const cJSON* jl = cJSON_GetObjectItemCaseSensitive((cJSON*)z, "length");
        const cJSON* jw = cJSON_GetObjectItemCaseSensitive((cJSON*)z, "width");
        if (!cJSON_IsNumber(jx) || !cJSON_IsNumber(jl) || !cJSON_IsNumber(jw)) continue;
        if (jl->valuedouble <= 0.0 || jw->valuedouble <= 0.0) continue;
        xs[n_out]   = jx->valuedouble;
        ys[n_out]   = cJSON_IsNumber(jy) ? jy->valuedouble : 0.0;
        lens[n_out] = jl->valuedouble;
        wids[n_out] = jw->valuedouble;
        n_out++;
    }
    return n_out;
}

/** 施工前缘 x（ego 顺行 +x 从低 x 接近）。 */
static inline double construction_zone_front_x(double cx, double length)
{
    return cx - 0.5 * length;
}

#ifdef __cplusplus
}
#endif

#endif /* FLOWENGINE_CONSTRUCTION_ZONES_H */