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
#include "adas_msgs_gen.h"
#include "logger.h"
#include "tiny_mlp.h"

#include <stdlib.h>
#include <string.h>
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

    fprintf(f, "{\n  \"schema\":\"flowengine-ota-registry v1\",\n");
    fprintf(f, "  \"current_id\":\"%s\",\n",
            g.current_idx >= 0 ? g.versions[g.current_idx].id : "");
    fprintf(f, "  \"previous_id\":\"%s\",\n",
            g.previous_idx >= 0 ? g.versions[g.previous_idx].id : "");
    fprintf(f, "  \"ab_test\":{\"enabled\":%s,\"ratio\":%.2f,\"model_b_path\":\"%s\"},\n",
            g.ab_enabled ? "true" : "false",
            (double)g.ab_ratio, g.model_b_path);
    fprintf(f, "  \"versions\":[\n");
    for (int i = 0; i < g.version_count; i++) {
        fprintf(f, "    {\"id\":\"%s\",\"path\":\"%s\",\"loaded_at\":%ld,\"active\":%s}%s\n",
                g.versions[i].id,
                g.versions[i].path,
                g.versions[i].loaded_at,
                g.versions[i].active ? "true" : "false",
                (i + 1 < g.version_count) ? "," : "");
    }
    fprintf(f, "  ]\n}\n");
    fclose(f);
}

static void registry_load(void) {
    FILE* f = fopen(g.registry_path, "r");
    if (!f) return;

    /* 简单文本扫描，提取 versions 数组 */
    char line[512];
    g.version_count = 0;
    g.current_idx   = -1;
    g.previous_idx  = -1;

    char cur_id[OTA_ID_LEN] = {0};
    char prev_id[OTA_ID_LEN] = {0};

    while (fgets(line, sizeof(line), f) && g.version_count < OTA_MAX_VERSIONS) {
        const char* p;

        /* 顶层字段 */
        if ((p = strstr(line, "\"current_id\":\""))) {
            const char* s = p + 14; const char* e = strchr(s, '"');
            if (e) { size_t l = (size_t)(e-s) < OTA_ID_LEN-1 ? (size_t)(e-s) : OTA_ID_LEN-1;
                     memcpy(cur_id, s, l); cur_id[l] = '\0'; }
        }
        if ((p = strstr(line, "\"previous_id\":\""))) {
            const char* s = p + 15; const char* e = strchr(s, '"');
            if (e) { size_t l = (size_t)(e-s) < OTA_ID_LEN-1 ? (size_t)(e-s) : OTA_ID_LEN-1;
                     memcpy(prev_id, s, l); prev_id[l] = '\0'; }
        }
        if ((p = strstr(line, "\"enabled\":")))
            g.ab_enabled = (strstr(p + 10, "true") != NULL) ? 1 : 0;
        if ((p = strstr(line, "\"ratio\":")))
            sscanf(p + 8, "%f", &g.ab_ratio);
        if ((p = strstr(line, "\"model_b_path\":\""))) {
            const char* s = p + 16; const char* e = strchr(s, '"');
            if (e) { size_t l = (size_t)(e-s) < OTA_PATH_LEN-1 ? (size_t)(e-s) : OTA_PATH_LEN-1;
                     memcpy(g.model_b_path, s, l); g.model_b_path[l] = '\0'; }
        }

        /* 版本条目: {"id":"...", "path":"...", "loaded_at":..., "active":...} */
        if (strstr(line, "\"id\":") && strstr(line, "\"path\":") &&
            strstr(line, "\"loaded_at\":") && g.version_count < OTA_MAX_VERSIONS) {
            OtaVersion* v = &g.versions[g.version_count];
            memset(v, 0, sizeof(*v));
            if ((p = strstr(line, "\"id\":\""))) {
                const char* s = p + 6; const char* e = strchr(s, '"');
                if (e) { size_t l = (size_t)(e-s) < OTA_ID_LEN-1 ? (size_t)(e-s) : OTA_ID_LEN-1;
                         memcpy(v->id, s, l); v->id[l] = '\0'; }
            }
            if ((p = strstr(line, "\"path\":\""))) {
                const char* s = p + 8; const char* e = strchr(s, '"');
                if (e) { size_t l = (size_t)(e-s) < OTA_PATH_LEN-1 ? (size_t)(e-s) : OTA_PATH_LEN-1;
                         memcpy(v->path, s, l); v->path[l] = '\0'; }
            }
            if ((p = strstr(line, "\"loaded_at\":")))
                sscanf(p + 12, "%ld", &v->loaded_at);
            if ((p = strstr(line, "\"active\":")))
                v->active = (strstr(p + 9, "true") != NULL) ? 1 : 0;
            if (v->id[0] != '\0') g.version_count++;
        }
    }
    fclose(f);

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
    char status[1024];
    const char* cur_id   = (g.current_idx  >= 0) ? g.versions[g.current_idx].id  : "";
    const char* prev_id  = (g.previous_idx >= 0) ? g.versions[g.previous_idx].id : "";
    int n = snprintf(status, sizeof(status),
        "{\"current_id\":\"%s\",\"previous_id\":\"%s\","
        "\"version_count\":%d,\"reload_count\":%d,\"rollback_count\":%d,"
        "\"ab_enabled\":%s,\"ab_ratio\":%.2f,"
        "\"ab_count_a\":%ld,\"ab_count_b\":%ld}",
        cur_id, prev_id,
        g.version_count, g.reload_count, g.rollback_count,
        g.ab_enabled ? "true" : "false", (double)g.ab_ratio,
        g.ab_count_a, g.ab_count_b);

    transport_publish(g.transport, "model_ota/status",
                      (const uint8_t*)status, (uint32_t)(n + 1));

    /* 同时写入状态文件供 modelctl.py ota status 读取 */
    FILE* f = fopen(OTA_STATUS_FILE, "w");
    if (f) { fprintf(f, "%s\n", status); fclose(f); }
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
    char sig[512];
    int n = snprintf(sig, sizeof(sig),
        "{\"cmd\":\"reload\",\"id\":\"%s\",\"path\":\"%s\"}",
        id, g.runtime_model_path);
    transport_publish(g.transport, "model_ota/active",
                      (const uint8_t*)sig, (uint32_t)(n + 1));

    /* 持久化注册表 */
    registry_save();

    g.reload_count++;
    LOG_INFO("model_ota", "activated version '%s' from %s (reload #%d)",
             id, path, g.reload_count);
    return 0;
}

static void handle_cmd(const char* json) {
    if (!json || json[0] == '\0') return;

    const char* p;

    /* "cmd":"load" */
    if (strstr(json, "\"load\"")) {
        char id[OTA_ID_LEN]    = "unknown";
        char path[OTA_PATH_LEN] = {0};
        if ((p = strstr(json, "\"id\":\""))) {
            const char* s = p + 6; const char* e = strchr(s, '"');
            if (e) { size_t l = (size_t)(e-s) < OTA_ID_LEN-1 ? (size_t)(e-s) : OTA_ID_LEN-1;
                     memcpy(id, s, l); id[l] = '\0'; }
        }
        if ((p = strstr(json, "\"path\":\""))) {
            const char* s = p + 8; const char* e = strchr(s, '"');
            if (e) { size_t l = (size_t)(e-s) < OTA_PATH_LEN-1 ? (size_t)(e-s) : OTA_PATH_LEN-1;
                     memcpy(path, s, l); path[l] = '\0'; }
        }
        if (path[0] != '\0')
            activate_version(id, path);
        else
            LOG_WARN("model_ota", "load command missing path");
        return;
    }

    /* "cmd":"rollback" */
    if (strstr(json, "\"rollback\"")) {
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
    if (strstr(json, "\"ab_test\"")) {
        int enable = strstr(json, "\"enable\":true") ? 1 : 0;
        float ratio = g.ab_ratio;
        if ((p = strstr(json, "\"ratio\":")))
            sscanf(p + 8, "%f", &ratio);
        char mb_path[OTA_PATH_LEN] = {0};
        if ((p = strstr(json, "\"model_b_path\":\""))) {
            const char* s = p + 16; const char* e = strchr(s, '"');
            if (e) { size_t l = (size_t)(e-s) < OTA_PATH_LEN-1 ? (size_t)(e-s) : OTA_PATH_LEN-1;
                     memcpy(mb_path, s, l); mb_path[l] = '\0'; }
        }

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
        return;
    }

    /* "cmd":"status" */
    if (strstr(json, "\"status\"")) {
        publish_status();
        return;
    }

    LOG_WARN("model_ota", "unknown command: %.80s", json);
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
    Localization loc;
    if (Localization_deserialize(&loc, (const uint8_t*)msg->data, msg->data_size) == 0) {
        g.ego_v = loc.v; g.ego_y = loc.y;
        g.ego_heading = loc.heading; g.ego_yaw_rate = loc.yaw_rate;
        g.has_fusion = 1;
        return;
    }
    const char* d = (const char*)msg->data;
    const char* p;
    if ((p = strstr(d, "\"v\":")))       sscanf(p + 4,  "%lf", &g.ego_v);
    if ((p = strstr(d, "\"y\":")))       sscanf(p + 4,  "%lf", &g.ego_y);
    if ((p = strstr(d, "\"heading\":"))) sscanf(p + 10, "%lf", &g.ego_heading);
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
    char result[512];
    int n = snprintf(result, sizeof(result),
        "{\"step\":%ld,\"use\":\"%s\","
        "\"a\":{\"id\":\"%s\",\"speed\":%.2f},"
        "\"b\":{\"path\":\"%s\",\"speed\":%.2f},"
        "\"delta\":%.3f}",
        g.ab_step, use_a ? "A" : "B",
        cur_id,      na >= 1 ? (double)ya[0] : 0.0,
        g.model_b_path, nb >= 1 ? (double)yb[0] : 0.0,
        (na >= 1 && nb >= 1) ? (double)(ya[0] - yb[0]) : 0.0);
    transport_publish(g.transport, "model_ota/ab_result",
                      (const uint8_t*)result, (uint32_t)(n + 1));
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

    struct timespec t_last, t_now;
    clock_gettime(CLOCK_MONOTONIC, &t_last);

    while (!g.should_stop) {
        usleep(poll_us);
        if (g.should_stop) break;

        clock_gettime(CLOCK_MONOTONIC, &t_now);
        double dt = (double)(t_now.tv_sec  - t_last.tv_sec)
                  + (double)(t_now.tv_nsec - t_last.tv_nsec) * 1e-9;
        t_last = t_now;
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
        const char* p;
        if ((p = strstr(params_json, "\"poll_interval_ms\":")))
            sscanf(p + 19, "%d", &g.poll_interval_ms);
        if ((p = strstr(params_json, "\"status_hz\":")))
            sscanf(p + 12, "%lf", &g.cfg_status_hz);
        if ((p = strstr(params_json, "\"ab_ratio\":"))) {
            float r; if (sscanf(p + 11, "%f", &r) == 1) g.ab_ratio = r;
        }
        if ((p = strstr(params_json, "\"runtime_model_path\":\""))) {
            const char* s = p + 22; const char* e = strchr(s, '"');
            if (e) { size_t l = (size_t)(e-s) < OTA_PATH_LEN-1 ? (size_t)(e-s) : OTA_PATH_LEN-1;
                     memcpy(g.runtime_model_path, s, l); g.runtime_model_path[l] = '\0'; }
        }
        if ((p = strstr(params_json, "\"registry_path\":\""))) {
            const char* s = p + 17; const char* e = strchr(s, '"');
            if (e) { size_t l = (size_t)(e-s) < OTA_PATH_LEN-1 ? (size_t)(e-s) : OTA_PATH_LEN-1;
                     memcpy(g.registry_path, s, l); g.registry_path[l] = '\0'; }
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
    if (pthread_create(&g.thread, NULL, ota_thread, NULL) != 0) return -1;
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
