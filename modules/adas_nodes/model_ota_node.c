/**
 * model_ota_node.c — 模型 OTA + 版本管理节点 (车端学习闭环 · Stage 4)
 *
 * 复用 Transport 总线做模型灰度发布、版本管理和 A-B 对比推理：
 *
 *   - 订阅 model_ota/cmd 接受 JSON 命令
 *   - 轮询 /tmp/flow_ota_cmd.json 接受 CLI 命令（供 modelctl.py ota 子命令使用）
 *   - 维护版本注册表 models/registry.json，记录所有已知模型版本及激活状态
 *   - 支持热加载 (load)、回滚 (rollback)、A-B 对比测试 (ab_test)
 *   - 热加载时将新权重原子复制到 runtime_model_path，再向 model_ota/active 发布
 *     重载信号，inference_node 订阅后自动热加载
 *   - A-B 测试时同时加载两个模型，对同一输入分别推理，将结果对比发布至
 *     model_ota/ab_result（旁路影子，不影响控制链路）
 *   - 定期发布 model_ota/status，同时写 /tmp/flow_ota_status.json
 *
 * 命令 JSON 格式 (topic model_ota/cmd 或文件 /tmp/flow_ota_cmd.json):
 *   {"cmd":"load",    "id":"v002","path":"models/e2e_tiny_v002/model.txt"}
 *   {"cmd":"rollback"}
 *   {"cmd":"ab_test", "enable":true, "model_b_path":"models/e2e_tiny_v001/model.txt", "ratio":0.5}
 *   {"cmd":"status"}
 *
 * NodePlugin 接口，编译为 libmodel_ota_node.so。
 */

#include "node_plugin.h"
#include "state_machine.h"
/* adas_msgs_gen.h previously included for Localization_deserialize — removed in cJSON-only cleanup */
#include "logger.h"
#include "tiny_mlp.h"
#include "clock_service.h"
#include <cjson/cJSON.h>

#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <pthread.h>
#include <unistd.h>
#include <time.h>
#include <sys/stat.h>

/* ── 最大版本历史数 ─────────────────────────────────────────── */
#define OTA_MAX_VERSIONS  32
#define OTA_ID_LEN        64
#define OTA_PATH_LEN     256
#define OTA_CMD_FILE     "/tmp/flow_ota_cmd.json"
#define OTA_STATUS_FILE  "/tmp/flow_ota_status.json"

/* ── 版本条目 ──────────────────────────────────────────────── */

typedef struct {
    char    id[OTA_ID_LEN];
    char    path[OTA_PATH_LEN];
    long    loaded_at;   /* Unix timestamp */
    int     active;
} OtaVersion;

/* ── 节点本地状态 ───────────────────────────────────────────── */

static struct {
    Transport*        transport;
    DiscoveryManager* discovery;
    Scheduler*        scheduler;

    pthread_t         thread;
    volatile int      running;
    volatile int      should_stop;

    ReflectiveStateMachine sm;

    /* 配置 */
    char   runtime_model_path[OTA_PATH_LEN];  /* 覆盖写入目标：tools/train/model.txt */
    char   registry_path[OTA_PATH_LEN];       /* 版本注册表：models/registry.json */
    char   watch_dir[OTA_PATH_LEN];           /* 扫描目录 */
    int    poll_interval_ms;
    double cfg_status_hz;
    double cfg_ab_ratio;                       /* A/B 采用比例（0~1，1=全用 A） */

    /* 版本注册表（内存） */
    OtaVersion versions[OTA_MAX_VERSIONS];
    int        version_count;
    int        current_idx;   /* -1 = 无 */
    int        previous_idx;  /* -1 = 无 */

    pthread_mutex_t reg_mutex;

    /* A-B 测试 */
    int     ab_enabled;
    float   ab_ratio;         /* 0~1，ab_ratio 概率用模型 A */
    TinyMLP model_a;          /* 当前激活模型副本（A） */
    TinyMLP model_b;          /* 候选模型（B） */
    int     model_b_loaded;
    char    model_b_path[OTA_PATH_LEN];
    long    ab_step;          /* A-B 步计数 */
    long    ab_count_a;
    long    ab_count_b;

    /* 最新 ego 特征（用于 A-B 影子推理） */
    double  ego_v, ego_y, ego_heading, ego_yaw_rate;
    volatile int has_fusion;

    /* 命令文件 mtime 用于增量检测 */
    time_t  cmd_file_mtime;

    /* 统计 */
    int  reload_count;
    int  rollback_count;
} g;

/* ─────────────────────────────────────────────────────────── */
/*  版本注册表 I/O                                              */
/* ─────────────────────────────────────────────────────────── */

static void registry_save(void) {
    FILE* f = fopen(g.registry_path, "w");
    if (!f) return;

    cJSON* root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "schema", "flowengine-ota-registry v1");
    cJSON_AddStringToObject(root, "current_id",
            g.current_idx >= 0 ? g.versions[g.current_idx].id : "");
    cJSON_AddStringToObject(root, "previous_id",
            g.previous_idx >= 0 ? g.versions[g.previous_idx].id : "");

    cJSON* ab = cJSON_CreateObject();
    cJSON_AddBoolToObject(ab, "enabled", g.ab_enabled);
    cJSON_AddNumberToObject(ab, "ratio", g.ab_ratio);
    cJSON_AddStringToObject(ab, "model_b_path", g.model_b_path);
    cJSON_AddItemToObject(root, "ab_test", ab);

    cJSON* versions = cJSON_CreateArray();
    for (int i = 0; i < g.version_count; i++) {
        cJSON* v = cJSON_CreateObject();
        cJSON_AddStringToObject(v, "id", g.versions[i].id);
        cJSON_AddStringToObject(v, "path", g.versions[i].path);
        cJSON_AddNumberToObject(v, "loaded_at", g.versions[i].loaded_at);
        cJSON_AddBoolToObject(v, "active", g.versions[i].active);
        cJSON_AddItemToArray(versions, v);
    }
    cJSON_AddItemToObject(root, "versions", versions);

    char* s = cJSON_Print(root);
    if (s) { fprintf(f, "%s\n", s); free(s); }
    cJSON_Delete(root);
    fclose(f);
}

static void registry_load(void) {
    FILE* f = fopen(g.registry_path, "r");
    if (!f) return;

    /* 读取整个文件到缓冲 */
    fseek(f, 0, SEEK_END);
    long fsize = ftell(f);
    if (fsize <= 0) { fclose(f); return; }
    rewind(f);
    char* buf = (char*)malloc((size_t)fsize + 1);
    if (!buf) { fclose(f); return; }
    size_t nread = fread(buf, 1, (size_t)fsize, f);
    fclose(f);
    buf[nread] = '\0';

    g.version_count = 0;
    g.current_idx   = -1;
    g.previous_idx  = -1;

    cJSON* root = cJSON_Parse(buf);
    free(buf);
    if (!root) return;

    char cur_id[OTA_ID_LEN] = {0};
    char prev_id[OTA_ID_LEN] = {0};

    cJSON* j;
    j = cJSON_GetObjectItemCaseSensitive(root, "current_id");
    if (cJSON_IsString(j) && j->valuestring)
        strncpy(cur_id, j->valuestring, OTA_ID_LEN - 1);
    j = cJSON_GetObjectItemCaseSensitive(root, "previous_id");
    if (cJSON_IsString(j) && j->valuestring)
        strncpy(prev_id, j->valuestring, OTA_ID_LEN - 1);

    cJSON* ab = cJSON_GetObjectItemCaseSensitive(root, "ab_test");
    if (cJSON_IsObject(ab)) {
        j = cJSON_GetObjectItemCaseSensitive(ab, "enabled");
        if (cJSON_IsBool(j)) g.ab_enabled = cJSON_IsTrue(j) ? 1 : 0;
        j = cJSON_GetObjectItemCaseSensitive(ab, "ratio");
        if (cJSON_IsNumber(j)) g.ab_ratio = (float)j->valuedouble;
        j = cJSON_GetObjectItemCaseSensitive(ab, "model_b_path");
        if (cJSON_IsString(j) && j->valuestring)
            strncpy(g.model_b_path, j->valuestring, OTA_PATH_LEN - 1);
    }

    cJSON* versions = cJSON_GetObjectItemCaseSensitive(root, "versions");
    if (cJSON_IsArray(versions)) {
        int nv = cJSON_GetArraySize(versions);
        if (nv > OTA_MAX_VERSIONS) nv = OTA_MAX_VERSIONS;
        for (int i = 0; i < nv; i++) {
            cJSON* v = cJSON_GetArrayItem(versions, i);
            if (!cJSON_IsObject(v)) continue;
            OtaVersion* ov = &g.versions[g.version_count];
            memset(ov, 0, sizeof(*ov));
            j = cJSON_GetObjectItemCaseSensitive(v, "id");
            if (cJSON_IsString(j) && j->valuestring)
                strncpy(ov->id, j->valuestring, OTA_ID_LEN - 1);
            j = cJSON_GetObjectItemCaseSensitive(v, "path");
            if (cJSON_IsString(j) && j->valuestring)
                strncpy(ov->path, j->valuestring, OTA_PATH_LEN - 1);
            j = cJSON_GetObjectItemCaseSensitive(v, "loaded_at");
            if (cJSON_IsNumber(j)) ov->loaded_at = (long)j->valuedouble;
            j = cJSON_GetObjectItemCaseSensitive(v, "active");
            if (cJSON_IsBool(j)) ov->active = cJSON_IsTrue(j) ? 1 : 0;
            if (ov->id[0] != '\0') g.version_count++;
        }
    }

    cJSON_Delete(root);

    /* 重建 current/previous 索引 */
    for (int i = 0; i < g.version_count; i++) {
        if (cur_id[0]  && strcmp(g.versions[i].id, cur_id)  == 0) g.current_idx  = i;
        if (prev_id[0] && strcmp(g.versions[i].id, prev_id) == 0) g.previous_idx = i;
    }
}

/* ─────────────────────────────────────────────────────────── */
/*  文件复制工具                                                */
/* ─────────────────────────────────────────────────────────── */

static int copy_file(const char* src, const char* dst) {
    FILE* fi = fopen(src, "rb");
    if (!fi) return -1;
    /* 写到临时文件，再原子重命名 */
    char tmp[OTA_PATH_LEN];
    snprintf(tmp, sizeof(tmp), "%s.tmp", dst);
    FILE* fo = fopen(tmp, "wb");
    if (!fo) { fclose(fi); return -1; }
    char buf[4096];
    size_t n;
    while ((n = fread(buf, 1, sizeof(buf), fi)) > 0) {
        if (fwrite(buf, 1, n, fo) != n) { fclose(fi); fclose(fo); remove(tmp); return -1; }
    }
    fclose(fi);
    fclose(fo);
    return rename(tmp, dst);
}

/* ─────────────────────────────────────────────────────────── */
/*  状态发布                                                    */
/* ─────────────────────────────────────────────────────────── */

static void publish_status(void) {
    cJSON* root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "current_id",
            g.current_idx >= 0 ? g.versions[g.current_idx].id : "");
    cJSON_AddStringToObject(root, "previous_id",
            g.previous_idx >= 0 ? g.versions[g.previous_idx].id : "");
    cJSON_AddNumberToObject(root, "version_count", g.version_count);
    cJSON_AddNumberToObject(root, "reload_count", g.reload_count);
    cJSON_AddNumberToObject(root, "rollback_count", g.rollback_count);
    cJSON_AddBoolToObject(root, "ab_enabled", g.ab_enabled);
    cJSON_AddNumberToObject(root, "ab_ratio", g.ab_ratio);
    cJSON_AddNumberToObject(root, "ab_count_a", g.ab_count_a);
    cJSON_AddNumberToObject(root, "ab_count_b", g.ab_count_b);

    char* s = cJSON_PrintUnformatted(root);
    transport_publish(g.transport, "model_ota/status",
                      (const uint8_t*)s, (uint32_t)strlen(s) + 1);

    /* 同时写入状态文件供 modelctl.py ota status 读取 */
    FILE* f = fopen(OTA_STATUS_FILE, "w");
    if (f) { fprintf(f, "%s\n", s); fclose(f); }
    free(s);
    cJSON_Delete(root);
}

/* ─────────────────────────────────────────────────────────── */
/*  命令处理                                                    */
/* ─────────────────────────────────────────────────────────── */

/*
 * 激活新版本：复制模型文件到 runtime 路径，发布 model_ota/active 信号，更新注册表。
 */
static int activate_version(const char* id, const char* path) {
    /* 校验文件存在 */
    FILE* test = fopen(path, "r");
    if (!test) {
        LOG_WARN("model_ota", "load rejected: file not found: %s", path);
        return -1;
    }
    fclose(test);

    pthread_mutex_lock(&g.reg_mutex);

    /* 将当前 active 降为非 active，记录 previous */
    if (g.current_idx >= 0) {
        g.previous_idx = g.current_idx;
        g.versions[g.current_idx].active = 0;
    }

    /* 查找已有版本或新建 */
    int found = -1;
    for (int i = 0; i < g.version_count; i++)
        if (strcmp(g.versions[i].id, id) == 0) { found = i; break; }

    if (found < 0) {
        if (g.version_count >= OTA_MAX_VERSIONS) {
            /* 环形淘汰最旧非 previous 版本 */
            found = 0;
            for (int i = 1; i < g.version_count; i++)
                if (i != g.previous_idx && g.versions[i].loaded_at < g.versions[found].loaded_at)
                    found = i;
        } else {
            found = g.version_count++;
        }
    }

    OtaVersion* v = &g.versions[found];
    strncpy(v->id, id, OTA_ID_LEN - 1);
    strncpy(v->path, path, OTA_PATH_LEN - 1);
    v->loaded_at = (long)time(NULL);
    v->active    = 1;
    g.current_idx = found;

    /* 更新模型 A（A-B 测试用）*/
    tiny_mlp_load(&g.model_a, path);

    pthread_mutex_unlock(&g.reg_mutex);

    /* 复制到 runtime 路径 */
    if (copy_file(path, g.runtime_model_path) != 0) {
        LOG_WARN("model_ota", "copy to runtime path failed: %s → %s",
                 path, g.runtime_model_path);
        /* 继续，runtime 路径不能写时仍发信号 */
    }

    /* 发布 model_ota/active 信号（通知 inference_node 热重载） */
    cJSON* sig_root = cJSON_CreateObject();
    cJSON_AddStringToObject(sig_root, "cmd", "reload");
    cJSON_AddStringToObject(sig_root, "id", id);
    cJSON_AddStringToObject(sig_root, "path", g.runtime_model_path);
    char* sig_s = cJSON_PrintUnformatted(sig_root);
    transport_publish(g.transport, "model_ota/active",
                      (const uint8_t*)sig_s, (uint32_t)strlen(sig_s) + 1);
    free(sig_s);
    cJSON_Delete(sig_root);

    /* 持久化注册表 */
    registry_save();

    g.reload_count++;
    LOG_INFO("model_ota", "activated version '%s' from %s (reload #%d)",
             id, path, g.reload_count);
    return 0;
}

static void handle_cmd(const char* json) {
    if (!json || json[0] == '\0') return;

    cJSON* root = cJSON_Parse(json);
    if (!root) {
        LOG_WARN("model_ota", "invalid JSON command: %.80s", json);
        return;
    }

    cJSON* j_cmd = cJSON_GetObjectItemCaseSensitive(root, "cmd");
    if (!cJSON_IsString(j_cmd) || !j_cmd->valuestring) {
        cJSON_Delete(root);
        return;
    }
    const char* cmd = j_cmd->valuestring;

    /* "cmd":"load" */
    if (strcmp(cmd, "load") == 0) {
        char id[OTA_ID_LEN] = "unknown";
        char path[OTA_PATH_LEN] = {0};
        cJSON* j;
        j = cJSON_GetObjectItemCaseSensitive(root, "id");
        if (cJSON_IsString(j) && j->valuestring)
            strncpy(id, j->valuestring, OTA_ID_LEN - 1);
        j = cJSON_GetObjectItemCaseSensitive(root, "path");
        if (cJSON_IsString(j) && j->valuestring)
            strncpy(path, j->valuestring, OTA_PATH_LEN - 1);
        if (path[0] != '\0')
            activate_version(id, path);
        else
            LOG_WARN("model_ota", "load command missing path");
        cJSON_Delete(root);
        return;
    }

    /* "cmd":"rollback" */
    if (strcmp(cmd, "rollback") == 0) {
        cJSON_Delete(root);
        pthread_mutex_lock(&g.reg_mutex);
        int prev = g.previous_idx;
        pthread_mutex_unlock(&g.reg_mutex);

        if (prev < 0) {
            LOG_WARN("model_ota", "rollback: no previous version");
            return;
        }
        char prev_id[OTA_ID_LEN];
        char prev_path[OTA_PATH_LEN];
        strncpy(prev_id,   g.versions[prev].id,   OTA_ID_LEN   - 1);
        strncpy(prev_path, g.versions[prev].path, OTA_PATH_LEN - 1);
        activate_version(prev_id, prev_path);
        g.rollback_count++;
        LOG_INFO("model_ota", "rollback to version '%s' (rollback #%d)",
                 prev_id, g.rollback_count);
        return;
    }

    /* "cmd":"ab_test" */
    if (strcmp(cmd, "ab_test") == 0) {
        float ratio = g.ab_ratio;
        char mb_path[OTA_PATH_LEN] = {0};
        cJSON* j;
        j = cJSON_GetObjectItemCaseSensitive(root, "enable");
        int enable = cJSON_IsTrue(j) ? 1 : 0;
        j = cJSON_GetObjectItemCaseSensitive(root, "ratio");
        if (cJSON_IsNumber(j)) ratio = (float)j->valuedouble;
        j = cJSON_GetObjectItemCaseSensitive(root, "model_b_path");
        if (cJSON_IsString(j) && j->valuestring)
            strncpy(mb_path, j->valuestring, OTA_PATH_LEN - 1);

        pthread_mutex_lock(&g.reg_mutex);
        g.ab_enabled = enable;
        g.ab_ratio   = ratio;
        if (mb_path[0] != '\0') {
            strncpy(g.model_b_path, mb_path, OTA_PATH_LEN - 1);
            g.model_b_loaded = (tiny_mlp_load(&g.model_b, mb_path) == 0) ? 1 : 0;
            if (!g.model_b_loaded)
                LOG_WARN("model_ota", "ab_test: failed to load model_b: %s", mb_path);
        }
        pthread_mutex_unlock(&g.reg_mutex);

        LOG_INFO("model_ota", "ab_test %s ratio=%.2f model_b=%s",
                 enable ? "on" : "off", (double)ratio,
                 mb_path[0] ? mb_path : "(unchanged)");
        cJSON_Delete(root);
        return;
    }

    /* "cmd":"status" */
    if (strcmp(cmd, "status") == 0) {
        cJSON_Delete(root);
        publish_status();
        return;
    }

    LOG_WARN("model_ota", "unknown command: %.80s", json);
    cJSON_Delete(root);
}

/* ─────────────────────────────────────────────────────────── */
/*  订阅回调                                                    */
/* ─────────────────────────────────────────────────────────── */

static void on_ota_cmd(const Message* msg, void* user_data) {
    (void)user_data;
    if (!msg || !msg->data) return;
    handle_cmd((const char*)msg->data);
}

static void on_fusion(const Message* msg, void* user_data) {
    (void)user_data;
    if (!msg || !msg->data) return;
    /* fusion/localization now publishes cJSON */
    cJSON* root = cJSON_Parse((const char*)msg->data);
    if (root) {
        cJSON* j;
        j = cJSON_GetObjectItemCaseSensitive(root, "v");
        if (cJSON_IsNumber(j)) g.ego_v = j->valuedouble;
        j = cJSON_GetObjectItemCaseSensitive(root, "y");
        if (cJSON_IsNumber(j)) g.ego_y = j->valuedouble;
        j = cJSON_GetObjectItemCaseSensitive(root, "heading");
        if (cJSON_IsNumber(j)) g.ego_heading = j->valuedouble;
        cJSON_Delete(root);
    }
    g.has_fusion = 1;
}

/* ─────────────────────────────────────────────────────────── */
/*  命令文件轮询                                                */
/* ─────────────────────────────────────────────────────────── */

static void poll_cmd_file(void) {
    struct stat st;
    if (stat(OTA_CMD_FILE, &st) != 0) return;
    if (st.st_mtime <= g.cmd_file_mtime) return;  /* 未修改 */
    g.cmd_file_mtime = st.st_mtime;

    FILE* f = fopen(OTA_CMD_FILE, "r");
    if (!f) return;
    char cmd_buf[1024] = {0};
    size_t n = fread(cmd_buf, 1, sizeof(cmd_buf) - 1, f);
    fclose(f);
    if (n > 0) {
        cmd_buf[n] = '\0';
        handle_cmd(cmd_buf);
        /* 处理后清空文件，防止重复执行 */
        FILE* fw = fopen(OTA_CMD_FILE, "w");
        if (fw) fclose(fw);
    }
}

/* ─────────────────────────────────────────────────────────── */
/*  A-B 影子推理                                                */
/* ─────────────────────────────────────────────────────────── */

static void run_ab_inference(void) {
    if (!g.ab_enabled || !g.has_fusion) return;
    if (!g.model_a.loaded && !g.model_b_loaded) return;

    float x4[4] = {
        (float)g.ego_v,
        (float)g.ego_y,
        (float)g.ego_heading,
        (float)g.ego_yaw_rate
    };

    float ya[TINY_MLP_MAX_OUT] = {0};
    float yb[TINY_MLP_MAX_OUT] = {0};
    int na = 0, nb = 0;

    pthread_mutex_lock(&g.reg_mutex);
    if (g.model_a.loaded) {
        float xa[TINY_MLP_MAX_IN] = {0};
        int in_a = g.model_a.in_dim <= 4 ? g.model_a.in_dim : 4;
        for (int i = 0; i < in_a; i++) xa[i] = x4[i];
        na = tiny_mlp_forward(&g.model_a, xa, ya);
    }
    if (g.model_b_loaded) {
        float xb[TINY_MLP_MAX_IN] = {0};
        int in_b = g.model_b.in_dim <= 4 ? g.model_b.in_dim : 4;
        for (int i = 0; i < in_b; i++) xb[i] = x4[i];
        nb = tiny_mlp_forward(&g.model_b, xb, yb);
    }

    /* A/B 选择（基于 ratio）: ab_step % 100 < ratio*100 则用 A */
    g.ab_step++;
    int use_a = ((g.ab_step % 100) < (long)(g.ab_ratio * 100.0f));
    if (use_a) g.ab_count_a++; else g.ab_count_b++;

    pthread_mutex_unlock(&g.reg_mutex);

    /* 发布对比结果 */
    const char* cur_id = (g.current_idx >= 0) ? g.versions[g.current_idx].id : "a";
    cJSON* ar_root = cJSON_CreateObject();
    cJSON_AddNumberToObject(ar_root, "step", g.ab_step);
    cJSON_AddStringToObject(ar_root, "use", use_a ? "A" : "B");
    cJSON* a_obj = cJSON_CreateObject();
    cJSON_AddStringToObject(a_obj, "id", cur_id);
    cJSON_AddNumberToObject(a_obj, "speed", na >= 1 ? (double)ya[0] : 0.0);
    cJSON_AddItemToObject(ar_root, "a", a_obj);
    cJSON* b_obj = cJSON_CreateObject();
    cJSON_AddStringToObject(b_obj, "path", g.model_b_path);
    cJSON_AddNumberToObject(b_obj, "speed", nb >= 1 ? (double)yb[0] : 0.0);
    cJSON_AddItemToObject(ar_root, "b", b_obj);
    cJSON_AddNumberToObject(ar_root, "delta",
            (na >= 1 && nb >= 1) ? (double)(ya[0] - yb[0]) : 0.0);
    char* ar_s = cJSON_PrintUnformatted(ar_root);
    transport_publish(g.transport, "model_ota/ab_result",
                      (const uint8_t*)ar_s, (uint32_t)strlen(ar_s) + 1);
    free(ar_s);
    cJSON_Delete(ar_root);
}

/* ─────────────────────────────────────────────────────────── */
/*  主线程                                                      */
/* ─────────────────────────────────────────────────────────── */

static void* ota_thread(void* arg) {
    (void)arg;
    pthread_setname_np(pthread_self(), "model_ota");

    const useconds_t poll_us = (useconds_t)(g.poll_interval_ms * 1000);
    const double status_period = g.cfg_status_hz > 0.0 ? 1.0 / g.cfg_status_hz : 1.0;

    double accum_status = 0.0;
    double accum_ab     = 0.0;

    uint64_t t_last_us = clock_now_us();

    while (!g.should_stop) {
        usleep(poll_us);
        if (g.should_stop) break;

        uint64_t t_now_us = clock_now_us();
        double dt = (double)(t_now_us - t_last_us) * 1e-6;
        t_last_us = t_now_us;
        accum_status += dt;
        accum_ab     += dt;

        /* 1. 轮询命令文件 */
        poll_cmd_file();

        /* 2. A-B 影子推理（与 status 同频） */
        if (g.ab_enabled && accum_ab >= status_period) {
            accum_ab = 0.0;
            run_ab_inference();
        }

        /* 3. 发布 status */
        if (accum_status >= status_period) {
            accum_status = 0.0;
            publish_status();
        }
    }

    LOG_INFO("model_ota", "stopped (reload=%d rollback=%d)",
             g.reload_count, g.rollback_count);
    statem_send_event(&g.sm, SM_EVENT_STOP, NULL);
    statem_send_event(&g.sm, SM_EVENT_DONE, NULL);
    return NULL;
}

/* ─────────────────────────────────────────────────────────── */
/*  NodePlugin 实现                                             */
/* ─────────────────────────────────────────────────────────── */

static const char* s_inputs[]  = {
    "model_ota/cmd",
    "fusion/localization",
    NULL
};
static const char* s_outputs[] = {
    "model_ota/active",
    "model_ota/status",
    "model_ota/ab_result",
    NULL
};

static NodePlugin s_plugin;

static int ota_init(MessageBus* bus, Transport* transport,
                    DiscoveryManager* discovery, Scheduler* scheduler,
                    const char* params_json) {
    (void)bus;
    memset(&g, 0, sizeof(g));
    g.transport   = transport;
    g.discovery   = discovery;
    g.scheduler   = scheduler;
    g.should_stop = 0;

    /* 默认配置 */
    g.poll_interval_ms = 500;
    g.cfg_status_hz    = 1.0;
    g.cfg_ab_ratio     = 0.5;
    g.ab_ratio         = 0.5f;
    g.current_idx      = -1;
    g.previous_idx     = -1;
    strncpy(g.runtime_model_path, "tools/train/model.txt", OTA_PATH_LEN - 1);
    strncpy(g.registry_path,      "models/registry.json",  OTA_PATH_LEN - 1);
    strncpy(g.watch_dir,          "models",                 OTA_PATH_LEN - 1);

    if (params_json) {
        cJSON* p = cJSON_Parse(params_json);
        if (p) {
            cJSON* j;
            j = cJSON_GetObjectItemCaseSensitive(p, "poll_interval_ms");
            if (cJSON_IsNumber(j)) g.poll_interval_ms = j->valueint;
            j = cJSON_GetObjectItemCaseSensitive(p, "status_hz");
            if (cJSON_IsNumber(j)) g.cfg_status_hz = j->valuedouble;
            j = cJSON_GetObjectItemCaseSensitive(p, "ab_ratio");
            if (cJSON_IsNumber(j)) g.ab_ratio = (float)j->valuedouble;
            j = cJSON_GetObjectItemCaseSensitive(p, "runtime_model_path");
            if (cJSON_IsString(j) && j->valuestring)
                strncpy(g.runtime_model_path, j->valuestring, OTA_PATH_LEN - 1);
            j = cJSON_GetObjectItemCaseSensitive(p, "registry_path");
            if (cJSON_IsString(j) && j->valuestring)
                strncpy(g.registry_path, j->valuestring, OTA_PATH_LEN - 1);
            cJSON_Delete(p);
        }
    }

    pthread_mutex_init(&g.reg_mutex, NULL);

    /* 加载持久化注册表（若存在） */
    registry_load();

    /* 初始化 model_a 为当前 runtime 模型 */
    if (tiny_mlp_load(&g.model_a, g.runtime_model_path) == 0) {
        LOG_INFO("model_ota", "model_a loaded from runtime %s", g.runtime_model_path);
    }

    /* 若注册表为空，添加初始版本 */
    if (g.version_count == 0) {
        OtaVersion* v = &g.versions[0];
        strncpy(v->id, "initial", OTA_ID_LEN - 1);
        strncpy(v->path, g.runtime_model_path, OTA_PATH_LEN - 1);
        v->loaded_at = (long)time(NULL);
        v->active    = 1;
        g.version_count = 1;
        g.current_idx   = 0;
        registry_save();
    }

    /* 恢复 A-B 测试模型 B */
    if (g.model_b_path[0] != '\0')
        g.model_b_loaded = (tiny_mlp_load(&g.model_b, g.model_b_path) == 0) ? 1 : 0;

    transport_subscribe(transport, "model_ota/cmd",        on_ota_cmd, NULL);
    transport_subscribe(transport, "fusion/localization",  on_fusion,  NULL);
    transport_advertise(transport, "model_ota/active",     0x4F544101u);
    transport_advertise(transport, "model_ota/status",     0x4F544102u);
    transport_advertise(transport, "model_ota/ab_result",  0x4F544103u);

    discovery_advertise(discovery, "model_ota/active",    0x4F544101u,
                        CAP_PUBLISHER, 1.0);
    discovery_advertise(discovery, "model_ota/status",    0x4F544102u,
                        CAP_PUBLISHER, g.cfg_status_hz);
    discovery_advertise(discovery, "model_ota/cmd",       0x4F544100u,
                        CAP_SUBSCRIBER, 0);

    statem_init(&g.sm, NULL, SM_STATE_INITIALIZED, "model_ota");
    statem_send_event(&g.sm, SM_EVENT_START, NULL);

    LOG_INFO("model_ota",
             "initialized (versions=%d current='%s' poll=%dms ab=%s)",
             g.version_count,
             g.current_idx >= 0 ? g.versions[g.current_idx].id : "none",
             g.poll_interval_ms,
             g.ab_enabled ? "on" : "off");
    return 0;
}

static int ota_start(void) {
    g.running = 1;
    g.should_stop = 0;
    if (pthread_create(&g.thread, NULL, ota_thread, NULL) != 0) {
        LOG_WARN("model_ota", "pthread_create failed: %s", strerror(errno));
        g.running = 0;
        return -1;
    }
    LOG_INFO("model_ota", "started [state=%s]",
             statem_state_name(&g.sm, g.sm.current));
    node_announce_self(g.transport, &s_plugin);
    return 0;
}

static void ota_stop(void) { g.should_stop = 1; }

static void ota_cleanup(void) {
    if (g.running) {
        g.should_stop = 1;
        pthread_join(g.thread, NULL);
        g.running = 0;
    }
    registry_save();
    pthread_mutex_destroy(&g.reg_mutex);
    LOG_INFO("model_ota", "cleanup done");
}

static int ota_health(void) { return 0; }

static NodePlugin s_plugin = {
    .api_version   = NODE_PLUGIN_API_VERSION,
    .name          = "model_ota",
    .version       = "1.0.0",
    .description   = "Model OTA + version management with A-B testing (Stage 4)",
    .input_topics  = s_inputs,
    .output_topics = s_outputs,
    .init          = ota_init,
    .start         = ota_start,
    .stop          = ota_stop,
    .cleanup       = ota_cleanup,
    .health        = ota_health,
};

NodePlugin* node_get_plugin(void) { return &s_plugin; }
