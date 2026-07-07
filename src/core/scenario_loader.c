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

    cJSON_Delete(root);
    LOG_INFO("scenario", "loaded '%s' (%d actors, seed=%u)",
             sc->name, sc->actor_count, sc->random_seed);
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

    char* out = cJSON_Print(root);
    cJSON_Delete(root);
    return out;
}
