/**
 * traversability_node.c — 可通行区域检测节点 (StereoFrame 深度图 → 地面分割 → 可通行栅格)
 *
 * 订阅 stereo_camera_node 发布的 sensor/stereo (StereoFrame)，把 80×60 深度图反投影成
 * 3D 点云（带 Z 高度，与 stereo_vision_node 的 2D 简化版不同），用"相机高度+高度阈值"
 * 做简易地面分割，构建本车前方局部占用栅格 (Occupancy Grid)，输出可通行性摘要到
 * perception/traversability：
 *
 *   [OAK-D] → stereo_camera → sensor/stereo → [本节点] → perception/traversability
 *
 * ── 为什么需要这个节点 ──
 *   perception/obstacles 只告诉下游"哪里有障碍物"，不告诉"哪里是路"。RC 小车在
 *   非结构化环境（校园/公园/草地边缘）跑 waypoint_follower 时，航点附近可能有
 *   低矮植被、路沿、台阶——这些既不在 LiDAR 检测范围（太近/太矮），也不算障碍物
 *   （DBSCAN 聚不出簇），但实际不可通行。本节点用立体视觉直接回答"前方 X 米内
 *   哪些区域地面是平的、能走"，是 L2 → L3 的桥梁。
 *
 * ── 算法（沙箱简化版，无 PCL 依赖） ──
 *   1. 3D 反投影（针孔模型）:
 *      对每个深度像素 (i, j)（i=0..dw-1, j=0..dh-1）:
 *        depth = depth_data[j*dw + i]
 *        theta = ((i+0.5)/dw - 0.5) * h_fov_rad      (水平角，左 +)
 *        phi   = -((j+0.5)/dh - 0.5) * v_fov_rad      (俯仰角，上 +；图顶 j=0 → +，图底 j=dh-1 → -)
 *        X_body = depth * cos(theta) * cos(phi)       (前向)
 *        Y_body = depth * sin(theta) * cos(phi)       (左向)
 *        Z_body = depth * sin(phi)                    (向上，+Z 朝天)
 *   2. 地面分割（高度阈值法，比 RANSAC 轻、对 RC 场景足够）:
 *      相机离地高 h_cam，世界 Z = h_cam + Z_body (相对地面，地面 = 0)。
 *      但相机一般有下倾角 tilt，需先旋转补偿：
 *        Z_body' = Z_body * cos(tilt) + X_body * sin(tilt)
 *        X_body' = X_body * cos(tilt) - Z_body * sin(tilt)
 *      ground world_z = h_cam + Z_body'
 *      |ground world_z| < ground_tol  →  地面点 (FREE)
 *      ground world_z > obstacle_height →  障碍点 (OCCUPIED)
 *   3. 占用栅格:
 *      x ∈ [0, x_range_m], y ∈ [-y_range_m, +y_range_m], cell_size_m 网格化
 *      地面点 → FREE；障碍点 → OCCUPIED（OCCUPIED 优先级高于 FREE）；无点 → UNKNOWN
 *   4. 走廊提取:
 *      在每个 y 列上扫描，若整列无 OCCUPIED 则记为可通行列，找最宽的连续可通行带
 *      → corridor_left_y / corridor_right_y / corridor_width_m
 *
 *   完整 RANSAC 地面拟合、3D 占用栅格 (Octomap)、可通行性代价图等更复杂的算法
 *   可作为后续替换点，把 segment_ground() / build_grid() 替换即可。
 *
 * 话题契约:
 *   输入: sensor/stereo (StereoFrame 二进制, type_id=0x669200d2)
 *   输出: perception/traversability (text JSON)
 *
 * 典型 pipeline_car.json 配置:
 *   {
 *     "name": "traversability",
 *     "library_path": "build/lib/libtraversability_node.so",
 *     "subscribe": ["sensor/stereo"],
 *     "publish": [{"topic": "perception/traversability", "type": "text"}],
 *     "params": "{\"enable\":1,\"camera_height_m\":0.30,\"camera_tilt_deg\":0.0,
 *                 \"cell_size_m\":0.20,\"x_range_m\":6.0,\"y_range_m\":3.0,
 *                 \"publish_hz\":10}"
 *   }
 *
 * 编译依赖: adas_msgs_gen.h (StereoFrame 反序列化)，随构建生成。
 */

#include "node_plugin.h"
#include "adas_msgs_gen.h"
#include "dbscan_cluster.h"   /* Point3D 类型 {x,y,z,intensity} */
#include "transport.h"
#include "discovery.h"
#include "logger.h"

#include <math.h>
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define TV_DEPTH_W       80     /* StereoFrame.depth_data 降采样宽度 */
#define TV_DEPTH_H       60     /* StereoFrame.depth_data 降采样高度 */
#define TV_MAX_POINTS    4000   /* 反投影点云上限（stride 降采样后通常 <1000） */
#define TV_GRID_MAX_CELLS 4096  /* 占用栅格最大 cell 数 (64*64=4096) */

/* cell 状态 */
#define TV_CELL_UNKNOWN  0
#define TV_CELL_FREE     1
#define TV_CELL_OCCUPIED 2

/* ── 节点状态 ─────────────────────────────────────────────── */

static struct {
    Transport*        transport;
    DiscoveryManager* discovery;

    /* 配置 */
    int    enabled;
    double min_range;          /* 最小有效距离 (m)，默认 0.5 */
    double max_range;          /* 最大有效距离 (m)，默认 6.0 (RC 场景近距够用) */
    int    stride;             /* 深度图降采样步长，默认 2 */
    double camera_height_m;    /* 相机离地高度 (m)，默认 0.30 */
    double camera_tilt_deg;    /* 相机下倾角 (度，+ = 向下看)，默认 0 */
    double ground_tol_m;       /* 地面高度容差 (m)，默认 0.08 */
    double obstacle_height_m;  /* 障碍物高度阈值 (m)，默认 0.10 (高于地面此值视为障碍) */
    double cell_size_m;        /* 栅格 cell 边长 (m)，默认 0.20 */
    double x_range_m;          /* 栅格前向范围 (m)，默认 6.0 */
    double y_range_m;          /* 栅格横向半范围 (m)，默认 3.0 (即 [-3, +3]) */
    double v_fov_deg;          /* 垂直视场角 (度)，默认 50.0 */
    double min_corridor_width_m;/* 最小可通行走廊宽度 (m)，默认 0.6 */
    int    publish_hz;         /* 发布频率，默认 10 */
    char   output_topic[64];   /* 输出 topic，默认 perception/traversability */

    /* 缓冲（init 分配，cleanup 释放） */
    Point3D* point_buf;        /* 复用 stereo_vision 的 Point3D 类型 {x,y,z,intensity} */
    int      point_buf_cap;
    uint8_t  grid[TV_GRID_MAX_CELLS];  /* 占用栅格 */

    /* 最新 StereoFrame（订阅回调写，工作线程读，mutex 保护） */
    StereoFrame last_frame;
    volatile int has_frame;
    pthread_mutex_t lock;

    /* 统计 */
    uint64_t frames_received;
    uint64_t grids_published;
    uint32_t frame_id;
    time_t   last_frame_time;

    /* 工作线程 */
    pthread_t thread;
    volatile int thread_running;
    volatile int should_stop;
} g;

/* ── 时间工具 ─────────────────────────────────────────────── */

static uint64_t now_us(void) {
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    return (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;
}

/* ── 参数解析（与 stereo_vision/perception_fusion 一致的极简 JSON 提取） ── */

static double parse_double(const char* json, const char* key, double default_val) {
    if (!json || !key) return default_val;
    char pat[64];
    snprintf(pat, sizeof(pat), "\"%s\"", key);
    const char* p = strstr(json, pat);
    if (!p) return default_val;
    p = strchr(p + strlen(pat), ':');
    if (!p) return default_val;
    p++;
    while (*p == ' ' || *p == '\t') p++;
    return strtod(p, NULL);
}

static int parse_int(const char* json, const char* key, int default_val) {
    return (int)parse_double(json, key, (double)default_val);
}

static void parse_string(const char* json, const char* key, char* out, size_t out_sz, const char* default_val) {
    if (!json || !key || !out || out_sz == 0) { if (default_val) snprintf(out, out_sz, "%s", default_val); return; }
    char pat[64];
    snprintf(pat, sizeof(pat), "\"%s\"", key);
    const char* p = strstr(json, pat);
    if (!p) { if (default_val) snprintf(out, out_sz, "%s", default_val); return; }
    p = strchr(p + strlen(pat), ':');
    if (!p) { if (default_val) snprintf(out, out_sz, "%s", default_val); return; }
    p++;
    while (*p == ' ' || *p == '\t') p++;
    if (*p != '"') { if (default_val) snprintf(out, out_sz, "%s", default_val); return; }
    p++;
    size_t n = 0;
    while (*p && *p != '"' && n < out_sz - 1) out[n++] = *p++;
    out[n] = '\0';
}

/* ── 订阅回调：收到 sensor/stereo ─────────────────────────── */
static void on_stereo(const Message* msg, void* user_data) {
    (void)user_data;
    if (!msg || !msg->data || !g.enabled) return;

    StereoFrame frame;
    if (StereoFrame_deserialize(&frame, (const uint8_t*)msg->data, msg->data_size) != 0) {
        LOG_WARN("traversability", "StereoFrame deserialize failed (size=%u)", msg->data_size);
        return;
    }
    if (frame.depth_count == 0) return;  /* 空帧 */

    pthread_mutex_lock(&g.lock);
    g.last_frame = frame;
    g.has_frame = 1;
    g.frames_received++;
    g.last_frame_time = time(NULL);
    pthread_mutex_unlock(&g.lock);
}

/* ── 3D 反投影：StereoFrame → Point3D[] (带 Z 高度) ─────────
 * 与 stereo_vision_node.depth_to_points 不同：本函数保留 Z 高度信息，
 * 用于后续地面分割。
 */
static int depth_to_points_3d(const StereoFrame* frame, Point3D* points, int max_n) {
    if (!frame || !points || max_n <= 0) return 0;
    if (frame->depth_count == 0) return 0;

    int dw = TV_DEPTH_W;
    int dh = frame->depth_count / dw;
    if (dh <= 0) dh = TV_DEPTH_H;
    if (frame->depth_count < dw * dh) dh = frame->depth_count / dw;

    double h_fov_rad = (frame->fov_deg > 0.1 ? (double)frame->fov_deg : 65.0) * M_PI / 180.0;
    double v_fov_rad = (g.v_fov_deg > 0.1 ? g.v_fov_deg : 50.0) * M_PI / 180.0;

    /* 相机下倾角 (弧度) */
    double tilt_rad = g.camera_tilt_deg * M_PI / 180.0;
    double cos_t = cos(tilt_rad);
    double sin_t = sin(tilt_rad);

    int n = 0;
    int stride = g.stride > 0 ? g.stride : 1;
    if (stride < 1) stride = 1;

    for (int j = 0; j < dh && n < max_n; j += stride) {
        for (int i = 0; i < dw && n < max_n; i += stride) {
            int idx = j * dw + i;
            if (idx >= (int)frame->depth_count) break;

            float depth = frame->depth_data[idx];
            if (depth < (float)g.min_range) continue;
            if (depth > (float)g.max_range) continue;
            if (depth != depth) continue;  /* NaN */

            /* 像素 → 光线方向 (相机坐标系) */
            double theta = ((double)(i + 0.5) / (double)dw - 0.5) * h_fov_rad;  /* 水平角，左 + */
            double phi   = -((double)(j + 0.5) / (double)dh - 0.5) * v_fov_rad;  /* 俯仰角，上 +；j=0 上，j=dh-1 下 */

            double cp = cos(phi);
            double Xc = (double)depth * cos(theta) * cp;  /* 前 */
            double Yc = (double)depth * sin(theta) * cp;  /* 左 */
            double Zc = (double)depth * sin(phi);         /* 上 (+Z 朝天) */

            /* 相机下倾角补偿：把相机坐标系旋转回水平车体坐标系
             * (相机下倾 tilt 时，相机 +X 方向实际指向车体前下方，
             *  绕 Y 轴反向旋转 tilt 即可还原) */
            double Xb =  Xc * cos_t - Zc * sin_t;
            double Zb =  Xc * sin_t + Zc * cos_t;
            /* Yb = Yc (绕 Y 轴旋转不影响 Y) */

            points[n].x = (float)Xb;
            points[n].y = (float)Yc;
            points[n].z = (float)Zb;
            points[n].intensity = 1.0f;
            n++;
        }
    }
    return n;
}

/* ── 占用栅格构建 ───────────────────────────────────────────
 * 把 3D 点云投影到 (X, Y) 平面栅格。
 *   地面点 (|world_z| < ground_tol) → 标 FREE
 *   障碍点 (world_z > obstacle_height) → 标 OCCUPIED（覆盖 FREE）
 * 其中 world_z = camera_height_m + Z_body (相机离地高 + 车体 Z)
 */
static void build_grid(const Point3D* pts, int n,
                       int grid_w, int grid_h,
                       double cell_size, double x_range, double y_range,
                       int* free_cnt, int* occ_cnt, int* unknown_cnt) {
    int total = grid_w * grid_h;
    if (total > TV_GRID_MAX_CELLS) total = TV_GRID_MAX_CELLS;
    memset(g.grid, TV_CELL_UNKNOWN, (size_t)total);

    int fcnt = 0, ocnt = 0;

    for (int k = 0; k < n; k++) {
        double Xb = pts[k].x;
        double Yb = pts[k].y;
        double Zb = pts[k].z;

        /* 范围裁剪 */
        if (Xb < 0.0 || Xb > x_range) continue;
        if (Yb < -y_range || Yb > y_range) continue;

        int gx = (int)(Xb / cell_size);
        int gy = (int)((Yb + y_range) / cell_size);
        if (gx < 0 || gx >= grid_w) continue;
        if (gy < 0 || gy >= grid_h) continue;
        int idx = gx * grid_h + gy;
        if (idx < 0 || idx >= total) continue;

        /* 世界 Z (相对地面)：相机离地高 + 车体 Z */
        double world_z = g.camera_height_m + Zb;

        if (world_z > g.obstacle_height_m) {
            if (g.grid[idx] != TV_CELL_OCCUPIED) {
                g.grid[idx] = TV_CELL_OCCUPIED;
                ocnt++;
            }
        } else if (fabs(world_z) < g.ground_tol_m) {
            if (g.grid[idx] == TV_CELL_UNKNOWN) {
                g.grid[idx] = TV_CELL_FREE;
                fcnt++;
            }
            /* 已是 OCCUPIED 则不降级为 FREE */
        }
    }

    *free_cnt = fcnt;
    *occ_cnt = ocnt;
    *unknown_cnt = total - fcnt - ocnt;
}

/* ── 走廊提取：找最宽的连续可通行 y 列带 ────────────────────
 * 在所有 x 行上扫描每个 y 列：若该列所有 cell 都非 OCCUPIED → 可通行
 * 然后找最长的连续可通行 y 段，输出左右边界与宽度。
 */
static void find_corridor(int grid_w, int grid_h,
                          double cell_size, double y_range,
                          double* left_y, double* right_y, double* width,
                          int* blocked) {
    /* col_passable[gy] = 1 表示该 y 列所有 x 都不是 OCCUPIED */
    int col_passable[256];
    if (grid_h > 256) grid_h = 256;  /* 安全上限 */
    for (int gy = 0; gy < grid_h; gy++) {
        col_passable[gy] = 1;
        for (int gx = 0; gx < grid_w && col_passable[gy]; gx++) {
            int idx = gx * grid_h + gy;
            if (idx >= 0 && idx < TV_GRID_MAX_CELLS &&
                g.grid[idx] == TV_CELL_OCCUPIED) {
                col_passable[gy] = 0;
            }
        }
    }

    /* 找最长连续可通行段 */
    int best_start = -1, best_len = 0;
    int cur_start = -1, cur_len = 0;
    for (int gy = 0; gy < grid_h; gy++) {
        if (col_passable[gy]) {
            if (cur_start < 0) cur_start = gy;
            cur_len++;
            if (cur_len > best_len) {
                best_len = cur_len;
                best_start = cur_start;
            }
        } else {
            cur_start = -1;
            cur_len = 0;
        }
    }

    double best_w = (double)best_len * cell_size;
    if (best_len <= 0 || best_w < g.min_corridor_width_m) {
        *left_y = 0.0;
        *right_y = 0.0;
        *width = 0.0;
        *blocked = 1;
        return;
    }

    /* cell 中心对应的车体 Y 坐标 */
    double y_left  = ((double)best_start + 0.5) * cell_size - y_range;
    double y_right = ((double)(best_start + best_len) - 0.5) * cell_size - y_range;
    *left_y  = y_left;
    *right_y = y_right;
    *width   = best_w;
    *blocked = 0;
}

/* ── 最近障碍距离 ─────────────────────────────────────────── */
static double nearest_obstacle_x(int grid_w, int grid_h, double cell_size) {
    double nearest = 1e9;
    for (int gx = 0; gx < grid_w; gx++) {
        for (int gy = 0; gy < grid_h; gy++) {
            int idx = gx * grid_h + gy;
            if (idx >= 0 && idx < TV_GRID_MAX_CELLS &&
                g.grid[idx] == TV_CELL_OCCUPIED) {
                double cx = ((double)gx + 0.5) * cell_size;
                if (cx < nearest) nearest = cx;
            }
        }
    }
    return (nearest > g.max_range) ? -1.0 : nearest;
}

/* ── 工作线程：收到帧 → 反投影 → 分割 → 栅格 → 发布 ────────── */
static void* traversability_thread(void* arg) {
    (void)arg;
    pthread_setname_np(pthread_self(), "traversable");

    long period_us = 1000000L / (g.publish_hz > 0 ? g.publish_hz : 10);

    /* 栅格维度（不变，循环外算） */
    int grid_w = (int)(g.x_range_m / g.cell_size_m);
    int grid_h = (int)((2.0 * g.y_range_m) / g.cell_size_m);
    if (grid_w < 1) grid_w = 1;
    if (grid_h < 1) grid_h = 1;
    if (grid_w * grid_h > TV_GRID_MAX_CELLS) {
        /* 网格太大，按比例缩到上限 */
        double scale = sqrt((double)TV_GRID_MAX_CELLS / (double)(grid_w * grid_h));
        grid_w = (int)(grid_w * scale);
        grid_h = (int)(grid_h * scale);
        if (grid_w < 1) grid_w = 1;
        if (grid_h < 1) grid_h = 1;
    }

    while (g.thread_running && !g.should_stop) {
        usleep((useconds_t)period_us);
        if (g.should_stop) break;

        StereoFrame frame;
        pthread_mutex_lock(&g.lock);
        if (!g.has_frame) {
            pthread_mutex_unlock(&g.lock);
            continue;
        }
        frame = g.last_frame;
        g.has_frame = 0;  /* 消费掉，避免重复处理 */
        pthread_mutex_unlock(&g.lock);

        /* 1. 3D 反投影 */
        int n = depth_to_points_3d(&frame, g.point_buf, g.point_buf_cap);
        if (n < 4) continue;  /* 点太少，栅格无意义 */

        /* 2. 构建占用栅格 */
        int free_cnt = 0, occ_cnt = 0, unknown_cnt = 0;
        build_grid(g.point_buf, n, grid_w, grid_h,
                   g.cell_size_m, g.x_range_m, g.y_range_m,
                   &free_cnt, &occ_cnt, &unknown_cnt);

        /* 3. 走廊提取 */
        double cor_left_y = 0, cor_right_y = 0, cor_width = 0;
        int blocked = 1;
        find_corridor(grid_w, grid_h, g.cell_size_m, g.y_range_m,
                      &cor_left_y, &cor_right_y, &cor_width, &blocked);

        /* 4. 最近障碍距离 */
        double nearest_obs = nearest_obstacle_x(grid_w, grid_h, g.cell_size_m);

        /* 5. 发布 JSON 摘要到 perception/traversability */
        char text[512];
        int len = snprintf(text, sizeof(text),
            "{\"frame_id\":%u,\"timestamp_us\":%llu,"
            "\"grid_w\":%d,\"grid_h\":%d,\"cell_size_m\":%.3f,"
            "\"x_range_m\":%.2f,\"y_range_m\":%.2f,"
            "\"free_cells\":%d,\"occupied_cells\":%d,\"unknown_cells\":%d,"
            "\"nearest_obstacle_x\":%.2f,"
            "\"corridor_left_y\":%.2f,\"corridor_right_y\":%.2f,"
            "\"corridor_width_m\":%.2f,\"blocked\":%d}",
            g.frame_id,
            (unsigned long long)now_us(),
            grid_w, grid_h, g.cell_size_m,
            g.x_range_m, g.y_range_m,
            free_cnt, occ_cnt, unknown_cnt,
            nearest_obs,
            cor_left_y, cor_right_y, cor_width, blocked);
        if (len > 0 && (size_t)len < sizeof(text)) {
            transport_publish(g.transport, g.output_topic,
                              (const uint8_t*)text, (uint32_t)len + 1);
            g.grids_published++;
            g.frame_id++;
        }

        /* 周期性日志 */
        if (g.grids_published % 30 == 1) {
            LOG_INFO("traversability", "frame #%u: pts=%d grid=%dx%d "
                     "free=%d occ=%d nearest=%.2fm corridor=%.2fm blocked=%d",
                     g.frame_id, n, grid_w, grid_h,
                     free_cnt, occ_cnt, nearest_obs, cor_width, blocked);
        }
    }
    return NULL;
}

/* ── NodePlugin 实现 ─────────────────────────────────────── */

static const char* s_inputs[]  = { "sensor/stereo", NULL };
static const char* s_outputs[] = { "perception/traversability", NULL };

static NodePlugin s_plugin;

static int traversability_init(MessageBus* bus, Transport* transport,
                                DiscoveryManager* discovery, Scheduler* scheduler,
                                const char* params_json) {
    (void)bus; (void)scheduler;
    memset(&g, 0, sizeof(g));
    g.transport = transport;
    g.discovery = discovery;

    pthread_mutex_init(&g.lock, NULL);

    /* 默认参数 */
    g.enabled              = 1;
    g.min_range            = 0.5;
    g.max_range            = 6.0;
    g.stride               = 2;
    g.camera_height_m      = 0.30;
    g.camera_tilt_deg      = 0.0;
    g.ground_tol_m         = 0.08;
    g.obstacle_height_m    = 0.10;
    g.cell_size_m          = 0.20;
    g.x_range_m            = 6.0;
    g.y_range_m            = 3.0;
    g.v_fov_deg            = 50.0;
    g.min_corridor_width_m = 0.6;
    g.publish_hz           = 10;
    snprintf(g.output_topic, sizeof(g.output_topic), "perception/traversability");

    if (params_json) {
        g.enabled              = parse_int(params_json, "enable", 1);
        g.min_range            = parse_double(params_json, "min_range", 0.5);
        g.max_range            = parse_double(params_json, "max_range", 6.0);
        g.stride               = parse_int(params_json, "stride", 2);
        g.camera_height_m      = parse_double(params_json, "camera_height_m", 0.30);
        g.camera_tilt_deg      = parse_double(params_json, "camera_tilt_deg", 0.0);
        g.ground_tol_m         = parse_double(params_json, "ground_tol_m", 0.08);
        g.obstacle_height_m    = parse_double(params_json, "obstacle_height_m", 0.10);
        g.cell_size_m          = parse_double(params_json, "cell_size_m", 0.20);
        g.x_range_m            = parse_double(params_json, "x_range_m", 6.0);
        g.y_range_m            = parse_double(params_json, "y_range_m", 3.0);
        g.v_fov_deg            = parse_double(params_json, "v_fov_deg", 50.0);
        g.min_corridor_width_m = parse_double(params_json, "min_corridor_width_m", 0.6);
        g.publish_hz           = parse_int(params_json, "publish_hz", 10);
        parse_string(params_json, "output_topic", g.output_topic,
                     sizeof(g.output_topic), "perception/traversability");
    }

    if (!g.enabled) {
        LOG_INFO("traversability", "disabled by config (enable=0)");
        return 0;
    }

    /* 分配点云缓冲 */
    g.point_buf_cap = TV_MAX_POINTS;
    g.point_buf = (Point3D*)malloc(sizeof(Point3D) * (size_t)g.point_buf_cap);
    if (!g.point_buf) {
        LOG_ERROR("traversability", "point buffer alloc failed");
        return -1;
    }

    /* 订阅 sensor/stereo */
    transport_subscribe(transport, "sensor/stereo", on_stereo, NULL);
    discovery_advertise(discovery, "sensor/stereo", STEREOFRAME_TYPE_ID, CAP_SUBSCRIBER, 0);

    /* 发布 output_topic（text 类型，无 type_id） */
    discovery_advertise(discovery, g.output_topic, 0, CAP_PUBLISHER, (double)g.publish_hz);

    g.last_frame_time = time(NULL);

    LOG_INFO("traversability", "initialized: range=[%.1f,%.1f]m stride=%d "
             "cam_h=%.2fm tilt=%.1fdeg cell=%.2fm grid=[%.1fx%.1f]m hz=%d",
             g.min_range, g.max_range, g.stride,
             g.camera_height_m, g.camera_tilt_deg, g.cell_size_m,
             g.x_range_m, 2.0 * g.y_range_m, g.publish_hz);
    return 0;
}

static int traversability_start(void) {
    if (!g.enabled) return 0;
    g.thread_running = 1;
    g.should_stop = 0;
    if (pthread_create(&g.thread, NULL, traversability_thread, NULL) != 0) {
        LOG_WARN("traversability", "thread create failed");
        g.thread_running = 0;
        return -1;
    }
    LOG_INFO("traversability", "started");
    node_announce_self(g.transport, &s_plugin);
    return 0;
}

static void traversability_stop(void) {
    g.should_stop = 1;
    g.thread_running = 0;
}

static void traversability_cleanup(void) {
    g.should_stop = 1;
    g.thread_running = 0;
    if (g.thread) { pthread_join(g.thread, NULL); g.thread = 0; }
    if (g.point_buf) { free(g.point_buf); g.point_buf = NULL; }
    pthread_mutex_destroy(&g.lock);
    LOG_INFO("traversability", "cleanup: frames=%lu grids=%lu",
             (unsigned long)g.frames_received,
             (unsigned long)g.grids_published);
}

static int traversability_health(void) {
    if (!g.enabled) return 0;
    time_t now = time(NULL);
    /* 60 秒未收到任何帧视为异常（双目相机掉线） */
    if (g.frames_received == 0 && (now - g.last_frame_time) > 60) return -1;
    return 0;
}

static NodePlugin s_plugin = {
    .api_version   = NODE_PLUGIN_API_VERSION,
    .name          = "traversability",
    .version       = "1.0.0",
    .description   = "Traversability analysis (StereoFrame depth → 3D → ground segmentation → occupancy grid)",
    .input_topics  = s_inputs,
    .output_topics = s_outputs,
    .init          = traversability_init,
    .start         = traversability_start,
    .stop          = traversability_stop,
    .cleanup       = traversability_cleanup,
    .health        = traversability_health,
};
