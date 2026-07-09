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

    /* ── scheduler (global) ───────────────────────────── */
    cJSON* jsch = cJSON_GetObjectItemCaseSensitive(root, "scheduler");
    if (cJSON_IsObject(jsch)) {
        cJSON* jmode = cJSON_GetObjectItemCaseSensitive(jsch, "mode");
        if (cJSON_IsString(jmode)) {
            cfg->scheduler.mode = (strcmp(jmode->valuestring, "choreo") == 0) ? 1 : 0;
        }
        cJSON* jwrk = cJSON_GetObjectItemCaseSensitive(jsch, "worker_threads");
        if (cJSON_IsNumber(jwrk)) cfg->scheduler.worker_threads = (int)jwrk->valuedouble;
        cJSON* jtick = cJSON_GetObjectItemCaseSensitive(jsch, "tick_us");
        if (cJSON_IsNumber(jtick)) cfg->scheduler.tick_us = (int)jtick->valuedouble;
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

            /* ── publish ── */
            cJSON* jpub = cJSON_GetObjectItemCaseSensitive(item, "publish");
            if (cJSON_IsArray(jpub)) {
                int n = cJSON_GetArraySize(jpub);
                for (int k = 0; k < n && k < PROC_MAX_TOPICS; k++) {
                    cJSON* jt = cJSON_GetArrayItem(jpub, k);
                    if (!jt) continue;
                    cJSON* jtn = cJSON_GetObjectItemCaseSensitive(jt, "topic");
                    cJSON* jtp = cJSON_GetObjectItemCaseSensitive(jt, "type");
                    cJSON* jqos = cJSON_GetObjectItemCaseSensitive(jt, "qos");
                    if (jtn) snprintf(pc->publish[k].topic, 64, "%s", jtn->valuestring);
                    if (jtp) snprintf(pc->publish[k].type,  64, "%s", jtp->valuestring);
                    if (jqos) {
                        cJSON* jd = cJSON_GetObjectItemCaseSensitive(jqos, "depth");
                        cJSON* jp = cJSON_GetObjectItemCaseSensitive(jqos, "policy");
                        cJSON* jr = cJSON_GetObjectItemCaseSensitive(jqos, "reliability");
                        cJSON* jdl = cJSON_GetObjectItemCaseSensitive(jqos, "deadline_ms");
                        cJSON* jls = cJSON_GetObjectItemCaseSensitive(jqos, "lifespan_ms");
                        cJSON* jtr = cJSON_GetObjectItemCaseSensitive(jqos, "transport");

                        pc->publish[k].qos_depth = jd ? (int)jd->valuedouble : 0;
                        if (jp) snprintf(pc->publish[k].qos_policy, 16, "%s", jp->valuestring);
                        if (jr) snprintf(pc->publish[k].qos_reliability, 16, "%s", jr->valuestring);
                        if (jdl) pc->publish[k].qos_deadline_ms = (int)jdl->valuedouble;
                        if (jls) pc->publish[k].qos_lifespan_ms = (int)jls->valuedouble;
                    }
                    pc->publish_count++;
                }
            }

            /* ── subscribe ── */
            cJSON* jsub = cJSON_GetObjectItemCaseSensitive(item, "subscribe");
            if (cJSON_IsArray(jsub)) {
                int n = cJSON_GetArraySize(jsub);
                for (int k = 0; k < n && k < PROC_MAX_TOPICS; k++) {
                    cJSON* jt = cJSON_GetArrayItem(jsub, k);
                    if (!jt) continue;
                    if (cJSON_IsString(jt)) {
                        /* Simple string: just topic name */
                        snprintf(pc->subscribe[k].topic, 64, "%s", jt->valuestring);
                    } else if (cJSON_IsObject(jt)) {
                        cJSON* jtn = cJSON_GetObjectItemCaseSensitive(jt, "topic");
                        cJSON* jrm = cJSON_GetObjectItemCaseSensitive(jt, "remap");
                        if (jtn) snprintf(pc->subscribe[k].topic, 64, "%s", jtn->valuestring);
                        if (jrm) snprintf(pc->subscribe[k].remap, 64, "%s", jrm->valuestring);
                    }
                    pc->subscribe_count++;
                }
            }

            /* ── depends ── */
            cJSON* jdep = cJSON_GetObjectItemCaseSensitive(item, "depends");
            if (cJSON_IsArray(jdep)) {
                int n = cJSON_GetArraySize(jdep);
                for (int k = 0; k < n && k < PROC_MAX_DEPS; k++) {
                    cJSON* jd = cJSON_GetArrayItem(jdep, k);
                    if (cJSON_IsString(jd))
                        snprintf(pc->depends[k], 64, "%s", jd->valuestring);
                    pc->depends_count++;
                }
            }

            /* ── resources ── */
            cJSON* jres = cJSON_GetObjectItemCaseSensitive(item, "resources");
            if (cJSON_IsObject(jres)) {
                cJSON* jmm = cJSON_GetObjectItemCaseSensitive(jres, "max_memory_mb");
                cJSON* jcp = cJSON_GetObjectItemCaseSensitive(jres, "max_cpu_percent");
                if (jmm) pc->resources.max_memory_mb = (int)jmm->valuedouble;
                if (jcp) pc->resources.max_cpu_percent = (int)jcp->valuedouble;
            }

            /* ── params (key=value) ── */
            cJSON* jprm = cJSON_GetObjectItemCaseSensitive(item, "params");
            if (cJSON_IsObject(jprm)) {
                char* ps = cJSON_PrintUnformatted(jprm);
                if (ps) {
                    snprintf(pc->params, sizeof(pc->params), "%s", ps);
                    free(ps);
                }
            }

            /* ── scheduling (per-process) ───────────────── */
            cJSON* jsched = cJSON_GetObjectItemCaseSensitive(item, "scheduling");
            if (cJSON_IsObject(jsched)) {
                /* priority: "low"|"normal"|"high"|"critical" or number */
                cJSON* jp = cJSON_GetObjectItemCaseSensitive(jsched, "priority");
                if (cJSON_IsString(jp)) {
                    if (strcmp(jp->valuestring, "critical") == 0) pc->scheduling.priority = 3;
                    else if (strcmp(jp->valuestring, "high") == 0) pc->scheduling.priority = 2;
                    else if (strcmp(jp->valuestring, "normal") == 0) pc->scheduling.priority = 1;
                    else pc->scheduling.priority = 0;
                } else if (cJSON_IsNumber(jp)) {
                    pc->scheduling.priority = (int)jp->valuedouble;
                }

                /* max_frequency_hz */
                cJSON* jfreq = cJSON_GetObjectItemCaseSensitive(jsched, "max_frequency_hz");
                if (cJSON_IsNumber(jfreq)) pc->scheduling.max_frequency_hz = jfreq->valuedouble;

                /* cpu_affinity: [0, 1, 2] or bitmask number */
                cJSON* jaff = cJSON_GetObjectItemCaseSensitive(jsched, "cpu_affinity");
                if (cJSON_IsArray(jaff)) {
                    int n = cJSON_GetArraySize(jaff);
                    for (int k = 0; k < n && k < 64; k++) {
                        cJSON* ji = cJSON_GetArrayItem(jaff, k);
                        if (cJSON_IsNumber(ji))
                            pc->scheduling.cpu_affinity_mask |= (1ULL << (int)ji->valuedouble);
                    }
                } else if (cJSON_IsNumber(jaff)) {
                    pc->scheduling.cpu_affinity_mask = (uint64_t)jaff->valuedouble;
                }

                /* cpuset: "0-3" or "0,2,4" */
                cJSON* jset = cJSON_GetObjectItemCaseSensitive(jsched, "cpuset");
                if (cJSON_IsString(jset) && jset->valuestring) {
                    /* Parse "0-3" format */
                    int start = -1, end = -1;
                    if (sscanf(jset->valuestring, "%d-%d", &start, &end) == 2) {
                        pc->scheduling.cpuset_start = start;
                        pc->scheduling.cpuset_end   = end;
                        for (int c = start; c <= end && c < 64; c++)
                            pc->scheduling.cpu_affinity_mask |= (1ULL << c);
                    }
                }
            }

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

    /* scheduler global */
    cJSON* jsch = cJSON_CreateObject();
    cJSON_AddStringToObject(jsch, "mode", config->scheduler.mode == 1 ? "choreo" : "classic");
    cJSON_AddNumberToObject(jsch, "worker_threads", config->scheduler.worker_threads);
    cJSON_AddNumberToObject(jsch, "tick_us", config->scheduler.tick_us);
    cJSON_AddItemToObject(root, "scheduler", jsch);

    cJSON* jprocs = cJSON_CreateArray();
    for (int i = 0; i < config->process_count; i++) {
        const ProcessConfig* pc = &config->processes[i];
        cJSON* item = cJSON_CreateObject();
        cJSON_AddStringToObject(item, "name", pc->name);
        cJSON_AddStringToObject(item, "library_path", pc->library_path);
        cJSON_AddStringToObject(item, "config_data", pc->config_data);
        cJSON_AddNumberToObject(item, "priority", pc->priority);
        cJSON_AddBoolToObject(item, "auto_start", pc->auto_start);

        /* scheduling */
        cJSON* js = cJSON_CreateObject();
        const char* pname = (pc->scheduling.priority == 3) ? "critical" :
                            (pc->scheduling.priority == 2) ? "high" :
                            (pc->scheduling.priority == 1) ? "normal" : "low";
        cJSON_AddStringToObject(js, "priority", pname);
        if (pc->scheduling.max_frequency_hz > 0)
            cJSON_AddNumberToObject(js, "max_frequency_hz", pc->scheduling.max_frequency_hz);
        if (pc->scheduling.cpu_affinity_mask != 0)
            cJSON_AddNumberToObject(js, "cpu_affinity", (double)pc->scheduling.cpu_affinity_mask);
        cJSON_AddItemToObject(item, "scheduling", js);

        /* resources */
        if (pc->resources.max_memory_mb > 0 || pc->resources.max_cpu_percent > 0) {
            cJSON* jr = cJSON_CreateObject();
            if (pc->resources.max_memory_mb > 0)
                cJSON_AddNumberToObject(jr, "max_memory_mb", pc->resources.max_memory_mb);
            if (pc->resources.max_cpu_percent > 0)
                cJSON_AddNumberToObject(jr, "max_cpu_percent", pc->resources.max_cpu_percent);
            cJSON_AddItemToObject(item, "resources", jr);
        }

        /* publish topics */
        if (pc->publish_count > 0) {
            cJSON* jp = cJSON_CreateArray();
            for (int k = 0; k < pc->publish_count; k++) {
                cJSON* jt = cJSON_CreateObject();
                cJSON_AddStringToObject(jt, "topic", pc->publish[k].topic);
                if (pc->publish[k].type[0])
                    cJSON_AddStringToObject(jt, "type", pc->publish[k].type);
                if (pc->publish[k].qos_depth > 0 || pc->publish[k].qos_policy[0]
                    || pc->publish[k].qos_deadline_ms > 0 || pc->publish[k].qos_reliability[0]) {
                    cJSON* jq = cJSON_CreateObject();
                    if (pc->publish[k].qos_depth > 0)
                        cJSON_AddNumberToObject(jq, "depth", pc->publish[k].qos_depth);
                    if (pc->publish[k].qos_policy[0])
                        cJSON_AddStringToObject(jq, "policy", pc->publish[k].qos_policy);
                    if (pc->publish[k].qos_reliability[0])
                        cJSON_AddStringToObject(jq, "reliability", pc->publish[k].qos_reliability);
                    if (pc->publish[k].qos_deadline_ms > 0)
                        cJSON_AddNumberToObject(jq, "deadline_ms", pc->publish[k].qos_deadline_ms);
                    if (pc->publish[k].qos_lifespan_ms > 0)
                        cJSON_AddNumberToObject(jq, "lifespan_ms", pc->publish[k].qos_lifespan_ms);
                    cJSON_AddItemToObject(jt, "qos", jq);
                }
                cJSON_AddItemToArray(jp, jt);
            }
            cJSON_AddItemToObject(item, "publish", jp);
        }

        /* subscribe topics */
        if (pc->subscribe_count > 0) {
            cJSON* jsb = cJSON_CreateArray();
            for (int k = 0; k < pc->subscribe_count; k++) {
                if (pc->subscribe[k].remap[0]) {
                    cJSON* jt = cJSON_CreateObject();
                    cJSON_AddStringToObject(jt, "topic", pc->subscribe[k].topic);
                    cJSON_AddStringToObject(jt, "remap", pc->subscribe[k].remap);
                    cJSON_AddItemToArray(jsb, jt);
                } else {
                    cJSON_AddItemToArray(jsb,
                        cJSON_CreateString(pc->subscribe[k].topic));
                }
            }
            cJSON_AddItemToObject(item, "subscribe", jsb);
        }

        /* depends */
        if (pc->depends_count > 0) {
            cJSON* jd = cJSON_CreateArray();
            for (int k = 0; k < pc->depends_count; k++)
                cJSON_AddItemToArray(jd, cJSON_CreateString(pc->depends[k]));
            cJSON_AddItemToObject(item, "depends", jd);
        }

        /* params */
        if (pc->params[0]) {
            cJSON* jpm = cJSON_Parse(pc->params);
            if (jpm) {
                cJSON_AddItemToObject(item, "params", jpm);
            } else {
                cJSON_AddStringToObject(item, "params", pc->params);
            }
        }

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
