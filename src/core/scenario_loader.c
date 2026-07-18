/**
 * scenario_loader.c — 场景描述文件加载器
 *
 * 解析 JSON 场景文件，填充 ScenarioConfig 结构体。
 * 依赖 cJSON（已作为 flowengine_core 依赖引入）。
 */

#include "scenario_loader.h"
#include "logger.h"
#include <cjson/cJSON.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ── 内部：读取整个文件到字符串 ───────────────────────────── */

static char* read_file(const char* path) {
    FILE* f = fopen(path, "r");
    if (!f) return NULL;
    fseek(f, 0, SEEK_END);
    long len = ftell(f);
    rewind(f);
    if (len <= 0) { fclose(f); return NULL; }
    char* buf = (char*)malloc((size_t)len + 1);
    if (!buf) { fclose(f); return NULL; }
    size_t n = fread(buf, 1, (size_t)len, f);
    buf[n] = '\0';
    fclose(f);
    return buf;
}

/* ── 公共 API ─────────────────────────────────────────────── */

ScenarioConfig* scenario_load(const char* path) {
    if (!path) return NULL;

    char* content = read_file(path);
    if (!content) {
        LOG_WARN("scenario", "cannot open '%s'", path);
        return NULL;
    }

    cJSON* root = cJSON_Parse(content);
    free(content);
    if (!root) {
        LOG_ERROR("scenario", "JSON parse error in '%s': %s", path,
                  cJSON_GetErrorPtr() ? cJSON_GetErrorPtr() : "unknown");
        return NULL;
    }

    ScenarioConfig* sc = (ScenarioConfig*)calloc(1, sizeof(ScenarioConfig));
    if (!sc) { cJSON_Delete(root); return NULL; }

    /* name */
    cJSON* jname = cJSON_GetObjectItemCaseSensitive(root, "name");
    if (cJSON_IsString(jname) && jname->valuestring)
        strncpy(sc->name, jname->valuestring, sizeof(sc->name) - 1);

    /* description */
    cJSON* jdesc = cJSON_GetObjectItemCaseSensitive(root, "description");
    if (cJSON_IsString(jdesc) && jdesc->valuestring)
        strncpy(sc->description, jdesc->valuestring, sizeof(sc->description) - 1);

    /* random_seed */
    cJSON* jseed = cJSON_GetObjectItemCaseSensitive(root, "random_seed");
    sc->random_seed = cJSON_IsNumber(jseed) ? (uint32_t)jseed->valuedouble : 42u;

    /* duration_s */
    cJSON* jdur = cJSON_GetObjectItemCaseSensitive(root, "duration_s");
    sc->duration_s = cJSON_IsNumber(jdur) ? jdur->valuedouble : 0.0;

    /* ego */
    cJSON* jego = cJSON_GetObjectItemCaseSensitive(root, "ego");
    if (cJSON_IsObject(jego)) {
        cJSON* j;
        j = cJSON_GetObjectItemCaseSensitive(jego, "x");
        if (cJSON_IsNumber(j)) sc->ego.x = j->valuedouble;
        j = cJSON_GetObjectItemCaseSensitive(jego, "y");
        if (cJSON_IsNumber(j)) sc->ego.y = j->valuedouble;
        j = cJSON_GetObjectItemCaseSensitive(jego, "heading");
        if (cJSON_IsNumber(j)) sc->ego.heading = j->valuedouble;
        j = cJSON_GetObjectItemCaseSensitive(jego, "init_speed");
        if (cJSON_IsNumber(j)) sc->ego.init_speed = j->valuedouble;
        j = cJSON_GetObjectItemCaseSensitive(jego, "target_speed");
        if (cJSON_IsNumber(j)) sc->ego.target_speed = j->valuedouble;
    }

    /* actors */
    cJSON* jactors = cJSON_GetObjectItemCaseSensitive(root, "actors");
    if (cJSON_IsArray(jactors)) {
        int n = cJSON_GetArraySize(jactors);
        if (n > SCENARIO_MAX_ACTORS) n = SCENARIO_MAX_ACTORS;
        sc->actor_count = n;
        for (int i = 0; i < n; i++) {
            cJSON* ja = cJSON_GetArrayItem(jactors, i);
            if (!cJSON_IsObject(ja)) continue;
            ScenarioActor* a = &sc->actors[i];
            a->segment_id = -1;  /* 默认旧格式：无 segment，用 x/y */
            cJSON* j;
            j = cJSON_GetObjectItemCaseSensitive(ja, "id");
            if (cJSON_IsNumber(j)) a->id = (int)j->valuedouble;
            j = cJSON_GetObjectItemCaseSensitive(ja, "type");
            if (cJSON_IsString(j) && j->valuestring)
                strncpy(a->type, j->valuestring, sizeof(a->type) - 1);
            j = cJSON_GetObjectItemCaseSensitive(ja, "x");
            if (cJSON_IsNumber(j)) a->x = j->valuedouble;
            j = cJSON_GetObjectItemCaseSensitive(ja, "y");
            if (cJSON_IsNumber(j)) a->y = j->valuedouble;
            j = cJSON_GetObjectItemCaseSensitive(ja, "vx");
            if (cJSON_IsNumber(j)) a->vx = j->valuedouble;
            j = cJSON_GetObjectItemCaseSensitive(ja, "vy");
            if (cJSON_IsNumber(j)) a->vy = j->valuedouble;
            j = cJSON_GetObjectItemCaseSensitive(ja, "len");
            if (cJSON_IsNumber(j)) a->len = j->valuedouble;
            j = cJSON_GetObjectItemCaseSensitive(ja, "wid");
            if (cJSON_IsNumber(j)) a->wid = j->valuedouble;
            /* road_network 新格式：s/l/segment_id → 流速仿真用 Frenet 坐标放置 NPC */
            j = cJSON_GetObjectItemCaseSensitive(ja, "segment_id");
            if (cJSON_IsNumber(j)) a->segment_id = (int)j->valuedouble;
            j = cJSON_GetObjectItemCaseSensitive(ja, "s");
            if (cJSON_IsNumber(j)) a->s = j->valuedouble;
            j = cJSON_GetObjectItemCaseSensitive(ja, "l");
            if (cJSON_IsNumber(j)) a->l = j->valuedouble;
        }
    }

    /* pass_criteria */
    cJSON* jcrit = cJSON_GetObjectItemCaseSensitive(root, "pass_criteria");
    if (cJSON_IsObject(jcrit)) {
        cJSON* j;
        j = cJSON_GetObjectItemCaseSensitive(jcrit, "no_collision");
        /* default: no_collision=true; only override when explicitly set to false.
         * j may be NULL (field absent) or a non-bool type — both treated as default. */
        if (!j || !cJSON_IsBool(j)) sc->criteria.no_collision = true;
        else sc->criteria.no_collision = cJSON_IsTrue(j);

        j = cJSON_GetObjectItemCaseSensitive(jcrit, "max_duration_s");
        if (cJSON_IsNumber(j)) sc->criteria.max_duration_s = j->valuedouble;
        j = cJSON_GetObjectItemCaseSensitive(jcrit, "min_avg_speed_mps");
        if (cJSON_IsNumber(j)) sc->criteria.min_avg_speed_mps = j->valuedouble;
        j = cJSON_GetObjectItemCaseSensitive(jcrit, "min_distance_m");
        if (cJSON_IsNumber(j)) sc->criteria.min_distance_m = j->valuedouble;
    } else {
        sc->criteria.no_collision = true;
    }

    /* route（可选）：导航路线主动变道指令，按 trigger_x 触发，与障碍物无关。
     * NOA Phase 3.1: 新增 type 字段（lane_change/branch_select/merge）+
     * branch_id（branch_select 选路）。无 type 字段 = ROUTE_LANE_CHANGE，
     * 与既有场景文件完全向后兼容。 */
    cJSON* jroute = cJSON_GetObjectItemCaseSensitive(root, "route");
    if (cJSON_IsArray(jroute)) {
        int n = cJSON_GetArraySize(jroute);
        if (n > SCENARIO_MAX_ROUTE_STEPS) n = SCENARIO_MAX_ROUTE_STEPS;
        sc->route_count = n;
        for (int i = 0; i < n; i++) {
            cJSON* jr = cJSON_GetArrayItem(jroute, i);
            if (!cJSON_IsObject(jr)) continue;
            ScenarioRouteStep* r = &sc->route[i];
            /* 默认 lane_change（向后兼容：无 type 字段的旧场景仍走原变道逻辑） */
            r->type = ROUTE_LANE_CHANGE;
            cJSON* j;
            j = cJSON_GetObjectItemCaseSensitive(jr, "trigger_x");
            if (cJSON_IsNumber(j)) r->trigger_x = j->valuedouble;
            j = cJSON_GetObjectItemCaseSensitive(jr, "target_lane");
            if (cJSON_IsNumber(j)) r->target_lane = (int)j->valuedouble;
            j = cJSON_GetObjectItemCaseSensitive(jr, "target_speed");
            if (cJSON_IsNumber(j)) r->target_speed = j->valuedouble;
            j = cJSON_GetObjectItemCaseSensitive(jr, "branch_id");
            if (cJSON_IsNumber(j)) r->branch_id = (int)j->valuedouble;
            /* type 字符串 → 枚举 */
            j = cJSON_GetObjectItemCaseSensitive(jr, "type");
            if (cJSON_IsString(j) && j->valuestring) {
                if (strcmp(j->valuestring, "branch_select") == 0)
                    r->type = ROUTE_BRANCH_SELECT;
                else if (strcmp(j->valuestring, "merge") == 0)
                    r->type = ROUTE_MERGE;
                else
                    r->type = ROUTE_LANE_CHANGE;
            }
            j = cJSON_GetObjectItemCaseSensitive(jr, "label");
            if (cJSON_IsString(j) && j->valuestring)
                strncpy(r->label, j->valuestring, sizeof(r->label) - 1);
        }
    }

    /* road（可选）：道路弯道几何。缺省字段/整个 "road" 对象缺失 = 全零 =
     * 禁用弯道（直道），与既有不含该字段的场景文件完全兼容。 */
    cJSON* jroad = cJSON_GetObjectItemCaseSensitive(root, "road");
    if (cJSON_IsObject(jroad)) {
        cJSON* j;
        j = cJSON_GetObjectItemCaseSensitive(jroad, "curve_start_x");
        if (cJSON_IsNumber(j)) sc->road.curve_start_x = j->valuedouble;
        j = cJSON_GetObjectItemCaseSensitive(jroad, "curve_length_m");
        if (cJSON_IsNumber(j)) sc->road.curve_length_m = j->valuedouble;
        j = cJSON_GetObjectItemCaseSensitive(jroad, "curve_offset_m");
        if (cJSON_IsNumber(j)) sc->road.curve_offset_m = j->valuedouble;
    }

    /* traffic_lights（可选）：红绿灯定义数组。缺省 = 无红绿灯，与既有场景完全兼容。
     * 见 traffic_light.h 的相位状态机。 */
    cJSON* jlights = cJSON_GetObjectItemCaseSensitive(root, "traffic_lights");
    if (cJSON_IsArray(jlights)) {
        int n = cJSON_GetArraySize(jlights);
        if (n > SCENARIO_MAX_TRAFFIC_LIGHTS) n = SCENARIO_MAX_TRAFFIC_LIGHTS;
        sc->traffic_light_count = n;
        for (int i = 0; i < n; i++) {
            cJSON* jl = cJSON_GetArrayItem(jlights, i);
            if (!cJSON_IsObject(jl)) continue;
            ScenarioTrafficLight* tl = &sc->traffic_lights[i];
            cJSON* j;
            j = cJSON_GetObjectItemCaseSensitive(jl, "id");
            tl->id = cJSON_IsNumber(j) ? (int)j->valuedouble : i;
            j = cJSON_GetObjectItemCaseSensitive(jl, "x");
            if (cJSON_IsNumber(j)) tl->x = j->valuedouble;
            j = cJSON_GetObjectItemCaseSensitive(jl, "y_lane");
            if (cJSON_IsNumber(j)) tl->y_lane = j->valuedouble;
            else tl->y_lane = -1.75;  /* 默认单车道横向位置 */
            j = cJSON_GetObjectItemCaseSensitive(jl, "heading");
            if (cJSON_IsNumber(j)) tl->heading = j->valuedouble;
            j = cJSON_GetObjectItemCaseSensitive(jl, "red_s");
            if (cJSON_IsNumber(j)) tl->red_s = j->valuedouble;
            j = cJSON_GetObjectItemCaseSensitive(jl, "yellow_s");
            if (cJSON_IsNumber(j)) tl->yellow_s = j->valuedouble;
            j = cJSON_GetObjectItemCaseSensitive(jl, "green_s");
            if (cJSON_IsNumber(j)) tl->green_s = j->valuedouble;
            j = cJSON_GetObjectItemCaseSensitive(jl, "phase_offset_s");
            if (cJSON_IsNumber(j)) tl->phase_offset_s = j->valuedouble;
        }
    }

    /* etc_gates（可选，FlowSim v2 新增）：高速 ETC 抬杆门架。
     * 缺省 = 无门架，与既有场景完全兼容。 */
    cJSON* jetc = cJSON_GetObjectItemCaseSensitive(root, "etc_gates");
    if (cJSON_IsArray(jetc)) {
        int n = cJSON_GetArraySize(jetc);
        if (n > SCENARIO_MAX_ETC_GATES) n = SCENARIO_MAX_ETC_GATES;
        sc->etc_gate_count = n;
        for (int i = 0; i < n; i++) {
            cJSON* jg = cJSON_GetArrayItem(jetc, i);
            if (!cJSON_IsObject(jg)) continue;
            ScenarioETCGate* g = &sc->etc_gates[i];
            cJSON* j;
            j = cJSON_GetObjectItemCaseSensitive(jg, "id");
            g->id = cJSON_IsNumber(j) ? (int)j->valuedouble : i;
            j = cJSON_GetObjectItemCaseSensitive(jg, "x");
            if (cJSON_IsNumber(j)) g->x = j->valuedouble;
            j = cJSON_GetObjectItemCaseSensitive(jg, "y");
            if (cJSON_IsNumber(j)) g->y = j->valuedouble;
            else g->y = 0.0;  /* 默认跨路面中心 */
            j = cJSON_GetObjectItemCaseSensitive(jg, "heading");
            if (cJSON_IsNumber(j)) g->heading = j->valuedouble;
            j = cJSON_GetObjectItemCaseSensitive(jg, "approach_speed");
            if (cJSON_IsNumber(j)) g->approach_speed = j->valuedouble;
            else g->approach_speed = 5.0;  /* ETC 默认减速到 5 m/s */
            j = cJSON_GetObjectItemCaseSensitive(jg, "open_range_m");
            if (cJSON_IsNumber(j)) g->open_range_m = j->valuedouble;
            else g->open_range_m = 50.0;  /* ego 进入 50m 时抬杆 */
        }
    }

    /* stop_lines（可选，FlowSim v2 新增）：路口/ETC 停车位置标记。
     * 缺省 = 无停止线，与既有场景完全兼容。 */
    cJSON* jsl = cJSON_GetObjectItemCaseSensitive(root, "stop_lines");
    if (cJSON_IsArray(jsl)) {
        int n = cJSON_GetArraySize(jsl);
        if (n > SCENARIO_MAX_STOP_LINES) n = SCENARIO_MAX_STOP_LINES;
        sc->stop_line_count = n;
        for (int i = 0; i < n; i++) {
            cJSON* js = cJSON_GetArrayItem(jsl, i);
            if (!cJSON_IsObject(js)) continue;
            ScenarioStopLine* sl = &sc->stop_lines[i];
            cJSON* j;
            j = cJSON_GetObjectItemCaseSensitive(js, "id");
            sl->id = cJSON_IsNumber(j) ? (int)j->valuedouble : i;
            j = cJSON_GetObjectItemCaseSensitive(js, "x");
            if (cJSON_IsNumber(j)) sl->x = j->valuedouble;
            j = cJSON_GetObjectItemCaseSensitive(js, "y");
            if (cJSON_IsNumber(j)) sl->y = j->valuedouble;
            else sl->y = 0.0;
        }
    }

    cJSON_Delete(root);
    LOG_INFO("scenario", "loaded '%s' (%d actors, %d route steps, %d traffic_lights, %d etc_gates, %d stop_lines, seed=%u)",
             sc->name, sc->actor_count, sc->route_count, sc->traffic_light_count,
             sc->etc_gate_count, sc->stop_line_count, sc->random_seed);
    return sc;
}

void scenario_free(ScenarioConfig* scenario) {
    free(scenario);
}

char* scenario_to_json(const ScenarioConfig* scenario) {
    if (!scenario) return NULL;

    cJSON* root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "name", scenario->name);
    cJSON_AddStringToObject(root, "description", scenario->description);
    cJSON_AddNumberToObject(root, "random_seed", (double)scenario->random_seed);
    cJSON_AddNumberToObject(root, "duration_s", scenario->duration_s);

    /* ego */
    cJSON* jego = cJSON_CreateObject();
    cJSON_AddNumberToObject(jego, "x",            scenario->ego.x);
    cJSON_AddNumberToObject(jego, "y",            scenario->ego.y);
    cJSON_AddNumberToObject(jego, "heading",      scenario->ego.heading);
    cJSON_AddNumberToObject(jego, "init_speed",   scenario->ego.init_speed);
    cJSON_AddNumberToObject(jego, "target_speed", scenario->ego.target_speed);
    cJSON_AddItemToObject(root, "ego", jego);

    /* actors */
    cJSON* jarr = cJSON_CreateArray();
    for (int i = 0; i < scenario->actor_count; i++) {
        const ScenarioActor* a = &scenario->actors[i];
        cJSON* ja = cJSON_CreateObject();
        cJSON_AddNumberToObject(ja, "id",  a->id);
        cJSON_AddStringToObject(ja, "type", a->type);
        cJSON_AddNumberToObject(ja, "x",   a->x);
        cJSON_AddNumberToObject(ja, "y",   a->y);
        cJSON_AddNumberToObject(ja, "vx",  a->vx);
        cJSON_AddNumberToObject(ja, "vy",  a->vy);
        cJSON_AddNumberToObject(ja, "len", a->len);
        cJSON_AddNumberToObject(ja, "wid", a->wid);
        cJSON_AddItemToArray(jarr, ja);
    }
    cJSON_AddItemToObject(root, "actors", jarr);

    /* criteria */
    cJSON* jcrit = cJSON_CreateObject();
    cJSON_AddBoolToObject(jcrit, "no_collision",       scenario->criteria.no_collision);
    cJSON_AddNumberToObject(jcrit, "max_duration_s",   scenario->criteria.max_duration_s);
    cJSON_AddNumberToObject(jcrit, "min_avg_speed_mps",scenario->criteria.min_avg_speed_mps);
    cJSON_AddNumberToObject(jcrit, "min_distance_m",   scenario->criteria.min_distance_m);
    cJSON_AddItemToObject(root, "pass_criteria", jcrit);

    /* route */
    cJSON* jroute = cJSON_CreateArray();
    for (int i = 0; i < scenario->route_count; i++) {
        const ScenarioRouteStep* r = &scenario->route[i];
        cJSON* jr = cJSON_CreateObject();
        cJSON_AddNumberToObject(jr, "trigger_x",   r->trigger_x);
        /* NOA Phase 3.1: 序列化 type + branch_id（branch_select 选路用） */
        const char* type_str = "lane_change";
        switch (r->type) {
            case ROUTE_BRANCH_SELECT: type_str = "branch_select"; break;
            case ROUTE_MERGE:         type_str = "merge";         break;
            default:                  type_str = "lane_change";   break;
        }
        cJSON_AddStringToObject(jr, "type", type_str);
        if (r->type == ROUTE_BRANCH_SELECT)
            cJSON_AddNumberToObject(jr, "branch_id", r->branch_id);
        cJSON_AddNumberToObject(jr, "target_lane", r->target_lane);
        if (r->target_speed > 0.0)
            cJSON_AddNumberToObject(jr, "target_speed", r->target_speed);
        cJSON_AddStringToObject(jr, "label",       r->label);
        cJSON_AddItemToArray(jroute, jr);
    }
    cJSON_AddItemToObject(root, "route", jroute);

    /* road */
    cJSON* jroad = cJSON_CreateObject();
    cJSON_AddNumberToObject(jroad, "curve_start_x",  scenario->road.curve_start_x);
    cJSON_AddNumberToObject(jroad, "curve_length_m", scenario->road.curve_length_m);
    cJSON_AddNumberToObject(jroad, "curve_offset_m", scenario->road.curve_offset_m);
    cJSON_AddItemToObject(root, "road", jroad);

    /* traffic_lights */
    cJSON* jlights = cJSON_CreateArray();
    for (int i = 0; i < scenario->traffic_light_count; i++) {
        const ScenarioTrafficLight* tl = &scenario->traffic_lights[i];
        cJSON* jl = cJSON_CreateObject();
        cJSON_AddNumberToObject(jl, "id",              tl->id);
        cJSON_AddNumberToObject(jl, "x",               tl->x);
        cJSON_AddNumberToObject(jl, "y_lane",          tl->y_lane);
        cJSON_AddNumberToObject(jl, "heading",         tl->heading);
        cJSON_AddNumberToObject(jl, "red_s",           tl->red_s);
        cJSON_AddNumberToObject(jl, "yellow_s",        tl->yellow_s);
        cJSON_AddNumberToObject(jl, "green_s",         tl->green_s);
        cJSON_AddNumberToObject(jl, "phase_offset_s",  tl->phase_offset_s);
        cJSON_AddItemToArray(jlights, jl);
    }
    cJSON_AddItemToObject(root, "traffic_lights", jlights);

    char* out = cJSON_Print(root);
    cJSON_Delete(root);
    return out;
}
