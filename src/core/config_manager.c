#include "config_manager.h"
#include "error_codes.h"
#include <cjson/cJSON.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static char* read_file(const char* path) {
    FILE* f = fopen(path, "r");
    if (!f) return NULL;

    fseek(f, 0, SEEK_END);
    long len = ftell(f);
    rewind(f);

    char* buf = (char*)malloc((size_t)len + 1);
    if (!buf) { fclose(f); return NULL; }

    size_t read = fread(buf, 1, (size_t)len, f);
    buf[read] = '\0';
    fclose(f);
    return buf;
}

LauncherConfig* config_load(const char* config_file) {
    if (!config_file) return NULL;

    char* content = read_file(config_file);
    if (!content) return NULL;

    cJSON* root = cJSON_Parse(content);
    free(content);
    if (!root) return NULL;

    LauncherConfig* cfg = (LauncherConfig*)calloc(1, sizeof(LauncherConfig));
    if (!cfg) { cJSON_Delete(root); return NULL; }

    /* log_file */
    cJSON* jlog = cJSON_GetObjectItemCaseSensitive(root, "log_file");
    if (cJSON_IsString(jlog) && jlog->valuestring) {
        strncpy(cfg->log_file, jlog->valuestring, sizeof(cfg->log_file) - 1);
    }

    /* log_level */
    cJSON* jlvl = cJSON_GetObjectItemCaseSensitive(root, "log_level");
    if (cJSON_IsNumber(jlvl)) {
        cfg->log_level = (int)jlvl->valuedouble;
    }

    /* monitor_interval */
    cJSON* jmon = cJSON_GetObjectItemCaseSensitive(root, "monitor_interval");
    if (cJSON_IsNumber(jmon)) {
        cfg->monitor_interval = (int)jmon->valuedouble;
    } else {
        cfg->monitor_interval = 5;
    }

    /* enable_monitor */
    cJSON* jenm = cJSON_GetObjectItemCaseSensitive(root, "enable_monitor");
    if (cJSON_IsBool(jenm)) {
        cfg->enable_monitor = cJSON_IsTrue(jenm);
    }

    /* processes */
    cJSON* jprocs = cJSON_GetObjectItemCaseSensitive(root, "processes");
    if (cJSON_IsArray(jprocs)) {
        int count = cJSON_GetArraySize(jprocs);
        cfg->processes = (ProcessConfig*)calloc((size_t)count, sizeof(ProcessConfig));
        if (!cfg->processes) {
            cJSON_Delete(root);
            free(cfg);
            return NULL;
        }
        cfg->process_count = 0;

        cJSON* item = NULL;
        cJSON_ArrayForEach(item, jprocs) {
            ProcessConfig* pc = &cfg->processes[cfg->process_count];

            cJSON* jname = cJSON_GetObjectItemCaseSensitive(item, "name");
            if (cJSON_IsString(jname) && jname->valuestring)
                strncpy(pc->name, jname->valuestring, sizeof(pc->name) - 1);

            cJSON* jlib = cJSON_GetObjectItemCaseSensitive(item, "library_path");
            if (cJSON_IsString(jlib) && jlib->valuestring)
                strncpy(pc->library_path, jlib->valuestring, sizeof(pc->library_path) - 1);

            cJSON* jcfg = cJSON_GetObjectItemCaseSensitive(item, "config_data");
            if (cJSON_IsString(jcfg) && jcfg->valuestring)
                strncpy(pc->config_data, jcfg->valuestring, sizeof(pc->config_data) - 1);

            cJSON* jpri = cJSON_GetObjectItemCaseSensitive(item, "priority");
            if (cJSON_IsNumber(jpri)) pc->priority = (int)jpri->valuedouble;

            cJSON* jauto = cJSON_GetObjectItemCaseSensitive(item, "auto_start");
            if (cJSON_IsBool(jauto)) pc->auto_start = cJSON_IsTrue(jauto);

            cfg->process_count++;
        }
    }

    cJSON_Delete(root);
    return cfg;
}

void config_free(LauncherConfig* config) {
    if (!config) return;
    if (config->processes) free(config->processes);
    free(config);
}

int config_save(const LauncherConfig* config, const char* config_file) {
    if (!config || !config_file) return ERR_INVALID_PARAM;

    cJSON* root = cJSON_CreateObject();
    if (!root) return ERR_INVALID_PARAM;

    cJSON_AddStringToObject(root, "log_file", config->log_file);
    cJSON_AddNumberToObject(root, "log_level", config->log_level);
    cJSON_AddNumberToObject(root, "monitor_interval", config->monitor_interval);
    cJSON_AddBoolToObject(root, "enable_monitor", config->enable_monitor);

    cJSON* jprocs = cJSON_CreateArray();
    for (int i = 0; i < config->process_count; i++) {
        const ProcessConfig* pc = &config->processes[i];
        cJSON* item = cJSON_CreateObject();
        cJSON_AddStringToObject(item, "name", pc->name);
        cJSON_AddStringToObject(item, "library_path", pc->library_path);
        cJSON_AddStringToObject(item, "config_data", pc->config_data);
        cJSON_AddNumberToObject(item, "priority", pc->priority);
        cJSON_AddBoolToObject(item, "auto_start", pc->auto_start);
        cJSON_AddItemToArray(jprocs, item);
    }
    cJSON_AddItemToObject(root, "processes", jprocs);

    char* str = cJSON_Print(root);
    cJSON_Delete(root);
    if (!str) return ERR_INVALID_PARAM;

    FILE* f = fopen(config_file, "w");
    if (!f) { free(str); return ERR_INVALID_PARAM; }
    fputs(str, f);
    fclose(f);
    free(str);
    return 0;
}
