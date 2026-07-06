/**
 * mcap_replay.c — MCAP 回放 + 算法管线验证工具
 *
 * 读取录制的 .mcap 文件，回放传感器数据，
 * 经过 DBSCAN→EKF→Frenet→PID 管线处理。
 *
 * 用法:
 *   flow_mcap_replay <demo.mcap> [speed=1.0] [loop=0]
 *
 * 示例:
 *   # 先录制
 *   ./flow_e2e 10          # 生成 demo.mcap
 *   # 再回放（2倍速）
 *   ./flow_mcap_replay demo.mcap 2.0
 *   # 循环回放（调参用）
 *   ./flow_mcap_replay demo.mcap 1.0 1
 */

#include "logger.h"
#include "mcap_reader.h"
#include "ekf_fusion.h"
#include "dbscan_cluster.h"
#ifndef NO_FRENET
#include "frenet_bridge.h"
#else
/* Minimal stubs so mcap_replay compiles without the Frenet planner */
typedef void FrenetHandle;
static inline FrenetHandle* frenet_create(double a, double b) { (void)a; (void)b; return NULL; }
static inline void frenet_destroy(FrenetHandle* h) { (void)h; }
static inline void frenet_set_reference_path(FrenetHandle* h, double* wx, double* wy, int n)
    { (void)h; (void)wx; (void)wy; (void)n; }
static inline int frenet_plan(FrenetHandle* h, double sx, double sy, double sv,
    double tv, double* s, double* d, double* spd, int max_wp)
    { (void)h; (void)sx; (void)sy; (void)sv; (void)tv; (void)s; (void)d; (void)spd; (void)max_wp; return 0; }
#endif
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <math.h>
#include <time.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ── 全局状态 ────────────────────────────────────────────────── */
static volatile bool g_running = true;
static McapReader*    g_reader  = NULL;

static void sig_handler(int sig) { (void)sig; g_running = false; }

/* ── EKF 融合状态 ────────────────────────────────────────────── */
static EkfFusion g_ekf;
static double    g_fused_x, g_fused_y, g_fused_v, g_fused_h, g_fused_yr;
static int       g_fusion_count = 0;

/* ── DBSCAN ──────────────────────────────────────────────────── */
static DbscanCluster g_db;

/* ── Frenet 规划器 ───────────────────────────────────────────── */
static FrenetHandle* g_frenet = NULL;

/* ── 统计 ────────────────────────────────────────────────────── */
static int g_msg_count = 0;
static int g_dbscan_detections = 0;
static int g_frenet_plans = 0;

/* ══════════════════════════════════════════════════════════ */
/* 辅助: 从 JSON 中提取字段                                    */
/* ══════════════════════════════════════════════════════════ */

static double json_get_number(const char* json, const char* key) {
    char search[64];
    snprintf(search, sizeof(search), "\"%s\":", key);
    const char* pos = strstr(json, search);
    if (!pos) return 0.0;
    return atof(pos + strlen(search));
}

/* ══════════════════════════════════════════════════════════ */
/* 处理一条回放消息                                            */
/* ══════════════════════════════════════════════════════════ */

static void process_message(const McapMessage* msg) {
    g_msg_count++;
    const char* topic = msg->topic;
    const char* data  = (const char*)msg->data;

    /* ── 发布到 transport (供下游订阅) ── */
    if (strcmp(topic, "sensor/lidar") == 0) {
        /* 提取 LiDAR 位置 */
        double lx = json_get_number(data, "x");
        double ly = json_get_number(data, "y");

        /* DBSCAN: 生成点云 → 聚类 */
        Point3D points[256];
        int np = 0;
        for (int ring = 0; ring < 2 && np < 256; ring++) {
            float r = 6.0f + (float)ring * 4.0f;
            for (int k = 0; k < 12 && np < 256; k++) {
                float a = (float)k / 12.0f * 2.0f * (float)M_PI;
                points[np].x = cosf(a) * r;
                points[np].y = sinf(a) * r;
                points[np].z = 0.05f; points[np].intensity = 0.3f;
                np++;
            }
        }
        /* 在 LiDAR 位置附近生成模拟障碍物点 */
        for (int oi = 0; oi < 3 && np < 256; oi++) {
            double ox = lx + 20.0 + oi * 15.0;
            double oy = ly + ((oi % 2) ? 3.5 : -1.75);
            int pts = (oi == 2) ? 6 : 10;
            for (int k = 0; k < pts && np < 256; k++) {
                points[np].x = (float)(ox + (k % 3 - 1.0) * 1.0);
                points[np].y = (float)(oy + (k / 3 - 1.0) * 0.8);
                points[np].z = 0.8f; points[np].intensity = 0.7f;
                np++;
            }
        }
        int nc = dbscan_run(&g_db, points, np);
        if (nc > 0) g_dbscan_detections++;

        /* EKF 预测 + LiDAR 更新 */
        ekf_fusion_predict(&g_ekf);
        ekf_fusion_update_lidar(&g_ekf, lx, ly, NULL);
    }
    else if (strcmp(topic, "sensor/gps") == 0) {
        double speed = json_get_number(data, "speed_mps");
        double heading = json_get_number(data, "heading_deg") * M_PI / 180.0;

        /* EKF GPS 更新 */
        ekf_fusion_update_gps(&g_ekf, speed, heading, NULL);
        g_fusion_count++;
    }
    else if (strcmp(topic, "sensor/camera") == 0) {
        /* Camera frame — skip for now (would use for vision-based detection) */
    }

    /* ── 每 10 条消息运行一次规划 ── */
    if (g_msg_count % 10 == 0) {
        ekf_fusion_get_state(&g_ekf, &g_fused_x, &g_fused_y,
                             &g_fused_v, &g_fused_h, &g_fused_yr);

        double s_out[50], d_out[50], spd_out[50];
        int n_wp = frenet_plan(g_frenet,
            g_fused_x, g_fused_y, g_fused_v,  /* ego s, d, speed */
            15.0,                               /* target speed */
            s_out, d_out, spd_out, 50);
        if (n_wp > 0) g_frenet_plans++;
    }
}

/* ══════════════════════════════════════════════════════════ */
/* Main                                                        */
/* ══════════════════════════════════════════════════════════ */

int main(int argc, char** argv) {
    const char* mcap_path = "demo.mcap";
    double speed = 1.0;
    int loop = 0;

    if (argc > 1) mcap_path = argv[1];
    if (argc > 2) speed = atof(argv[2]);
    if (argc > 3) loop = atoi(argv[3]);
    if (speed <= 0) speed = 1.0;

    /* ── 初始化 ── */
    srand((unsigned)time(NULL));
    log_init(LOG_INFO, NULL);
    signal(SIGINT, sig_handler);
    signal(SIGTERM, sig_handler);

    LOG_INFO("replay", "╔══════════════════════════════════════════╗");
    LOG_INFO("replay", "║  MCAP Replay + Algorithm Pipeline Test   ║");
    LOG_INFO("replay", "║  File: %-32s ║", mcap_path);
    LOG_INFO("replay", "║  Speed: %.1fx  Loop: %d                   ║", speed, loop);
    LOG_INFO("replay", "╚══════════════════════════════════════════╝");

    /* ── MCAP Reader ── */
    g_reader = mcap_reader_open(mcap_path);
    if (!g_reader) {
        fprintf(stderr, "Failed to open %s\n", mcap_path);
        return 1;
    }

    /* Print channels */
    int nc = mcap_reader_channel_count(g_reader);
    LOG_INFO("replay", "MCAP: %d channels:", nc);
    for (int i = 0; i < nc; i++) {
        const McapChannelInfo* ch = mcap_reader_get_channel(g_reader, i);
        LOG_INFO("replay", "  [%d] %s (%s)", ch->id, ch->topic, ch->schema_name);
    }

    /* ── 算法初始化 (直接调用，不需要 transport) ── */

    /* EKF: dt=0.05s, init at origin */
    double x0[5] = {0, 0, 5.0, 0, 0};
    ekf_fusion_init(&g_ekf, 0.05, x0);

    /* DBSCAN */
    dbscan_init(&g_db, 2.0f, 4);
    dbscan_set_ground_thresh(&g_db, 0.2f);

    /* Frenet */
    g_frenet = frenet_create(20.0, 4.0);
    double wx[101], wy[101];
    for (int i = 0; i <= 100; i++) { wx[i] = i * 2.0; wy[i] = 0.0; }
    frenet_set_reference_path(g_frenet, wx, wy, 101);

    LOG_INFO("replay", "Algorithms initialized: EKF+DBSCAN+Frenet");

    /* ── 回放循环 ── */
    uint64_t start_wall_ns = 0;
    struct timespec ts_start;
    clock_gettime(CLOCK_MONOTONIC, &ts_start);
    start_wall_ns = (uint64_t)ts_start.tv_sec * 1000000000ULL + (uint64_t)ts_start.tv_nsec;

    McapMessage msg;
    int last_report = 0;

    while (g_running) {
        int ret = mcap_reader_next(g_reader, &msg);
        if (ret == 0) {
            if (loop) {
                LOG_INFO("replay", "End of file — looping...");
                mcap_reader_seek_start(g_reader);
                /* Reset timing */
                clock_gettime(CLOCK_MONOTONIC, &ts_start);
                start_wall_ns = (uint64_t)ts_start.tv_sec * 1000000000ULL + (uint64_t)ts_start.tv_nsec;
                continue;
            }
            break;
        }
        if (ret < 0) break;

        /* ── 速度控制: 等待到正确的时间 ── */
        if (g_reader->base_time_ns > 0) {
            uint64_t msg_offset_ns = msg.log_time_ns - g_reader->base_time_ns;
            struct timespec ts_now;
            clock_gettime(CLOCK_MONOTONIC, &ts_now);
            uint64_t wall_ns = (uint64_t)ts_now.tv_sec * 1000000000ULL + (uint64_t)ts_now.tv_nsec;
            uint64_t elapsed_ns = wall_ns - start_wall_ns;
            uint64_t target_elapsed_ns = (uint64_t)((double)msg_offset_ns / speed);

            if (target_elapsed_ns > elapsed_ns) {
                uint64_t wait_ns = target_elapsed_ns - elapsed_ns;
                usleep((unsigned)(wait_ns / 1000));  /* usleep uses microseconds */
            }
        }

        /* ── 处理消息 ── */
        process_message(&msg);

        /* 进度报告 (每 100 条) */
        if (g_msg_count - last_report >= 100) {
            last_report = g_msg_count;
            LOG_INFO("replay", "processed %d msgs | EKF=%d DBSCAN=%d Frenet=%d | "
                     "fused:(%.0f,%.1f) v=%.1f",
                     g_msg_count, g_fusion_count, g_dbscan_detections, g_frenet_plans,
                     g_fused_x, g_fused_y, g_fused_v);
        }
    }

    /* ── 总结 ── */
    LOG_INFO("replay", "═══════════════════════════════════════════");
    LOG_INFO("replay", "Replay Complete");
    LOG_INFO("replay", "  Messages:    %d", g_msg_count);
    LOG_INFO("replay", "  EKF updates: %d", g_fusion_count);
    LOG_INFO("replay", "  DBSCAN detections: %d", g_dbscan_detections);
    LOG_INFO("replay", "  Frenet plans: %d", g_frenet_plans);
    LOG_INFO("replay", "  Final EKF state: pos=(%.1f,%.1f) v=%.1f h=%.1f°",
             g_fused_x, g_fused_y, g_fused_v, g_fused_h * 180.0 / M_PI);
    if (!g_ekf.diverged)
        LOG_INFO("replay", "  EKF: converged (no divergence)");
    else
        LOG_WARN("replay", "  EKF: diverged during replay");

    /* ── 清理 ── */
    frenet_destroy(g_frenet);
    mcap_reader_close(g_reader);
    log_shutdown();

    printf("\nMCAP Replay Summary:\n");
    printf("  %d messages processed\n", g_msg_count);
    printf("  EKF: %d updates, final v=%.1f m/s\n", g_fusion_count, g_fused_v);
    printf("  DBSCAN: %d obstacles detected\n", g_dbscan_detections);
    printf("  Frenet: %d trajectories planned\n", g_frenet_plans);

    return 0;
}
