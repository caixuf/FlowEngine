/**
 * stereo_vision_node.c — 双目视觉感知节点 (StereoFrame → 障碍物)
 *
 * 订阅 OAK-D 双目发布的 sensor/stereo (StereoFrame)，把深度图反投影成 3D 点云，
 * DBSCAN 聚类后输出 ObstacleList 到 perception/obstacles。这是 RC 小车感知链路的
 * 视觉入口，与 lidar_driver_node 互补（LiDAR 测远距，双目测近距稠密）：
 *
 *   [OAK-D] → stereo_camera_node → sensor/stereo → [本节点] → perception/obstacles
 *
 * ── 为什么需要这个节点 ──
 *   RPLIDAR A1 在 RC 小车上点云稀疏（~2000 点/帧），DBSCAN 对远处小目标聚类不稳。
 *   双目深度图是稠密网格（80×60=4800 像素），近距离目标（<8m）反投影后点密度高，
 *   DBSCAN 聚类更稳。两者融合：LiDAR 负责远距（8-60m），双目负责近距（0.5-8m）。
 *
 *   注意：两个节点都发布到 perception/obstacles，topic 后写者覆盖先写者（drop_oldest
 *   QoS）。如果需要真正融合，下游 fusion/perception 应分别订阅，或加一个独立的
 *   perception_fusion_node 合并两个 ObstacleList。当前 RC 小车场景，单源（LiDAR 或
 *   双目）已够用，按场景在 pipeline 里二选一即可。
 *
 * ── 深度反投影算法（针孔模型简化版） ──
 *   StereoFrame 含 80×60 降采样深度图 + fov_deg + baseline_m。
 *   对每个深度像素 (i, j)（i=0..79, j=0..59）:
 *     depth = depth_data[j*80 + i]  (米)
 *     if depth < min_range or depth > max_range: skip
 *     # 像素列 i 对应的水平角: 中心为 0，左为 +，右为 -
 *     theta = ((i + 0.5)/80 - 0.5) * fov_rad
 *     # 反投影到车体坐标系（X=前, Y=左）
 *     X = depth * cos(theta)
 *     Y = depth * sin(theta)
 *     # 像素行 j 对应俯仰角（向下为正，地面在画面下半部）
 *     # 简化：忽略俯仰，所有点投到 z=0 平面（2D 障碍物用）
 *     # 如需 z 高度，可从 j 反推: pitch = ((j+0.5)/60 - 0.5) * v_fov_rad
 *     #                    Z = depth * sin(pitch)
 *   为减少点数（DBSCAN 是 O(n²)），按 stride 降采样（如每隔 2 像素取一个）。
 *
 * ── 地面去除 ──
 *   简化：只取深度在 [min_range, max_range] 内的点。地面点（深度很大或为 0）自动被滤除。
 *   更精确的地面去除可加 RANSAC 平面拟合，复用 dbscan_cluster.h 的 RANSAC 模式。
 *
 * 话题契约:
 *   输入: sensor/stereo (StereoFrame 二进制, type_id=0x669200d2)
 *   输出: perception/obstacles (ObstacleList 二进制, type_id=0x308f5f71)
 *
 * 典型 pipeline_car.json 配置:
 *   {
 *     "name": "stereo_vision",
 *     "library": "libstereo_vision_node.so",
 *     "subscribe": ["sensor/stereo"],
 *     "publish": [{"topic": "perception/obstacles", "type": "ObstacleList"}],
 *     "params": "{\"min_range\":0.5,\"max_range\":8.0,\"stride\":2,\"dbscan_eps\":0.4,\"dbscan_min_pts\":6}"
 *   }
 */

#include "node_plugin.h"
#include "adas_msgs_gen.h"
#include "dbscan_cluster.h"
#include "tiny_mlp.h"
#include "transport.h"
#include "discovery.h"
#include "logger.h"
#include "clock_service.h"
#include <cjson/cJSON.h>

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

#define SV_DEPTH_W   80    /* StereoFrame.depth_data 降采样宽度 */
#define SV_DEPTH_H   60    /* StereoFrame.depth_data 降采样高度 */
#define SV_MAX_POINTS 4000 /* DBSCAN 输入点数上限（stride 降采样后通常 <1000） */

/* Phase 4: 分类头 — MLP 输入特征维度 */
#define SV_CLS_FEAT_DIM 10   /* width,length,height,pts,depth_mean,depth_std,density,cx,cy,cz */
#define SV_CLS_NUM_CLASSES 4  /* vehicle, pedestrian, cyclist, unknown */

/* ── 节点状态 ─────────────────────────────────────────────── */

static struct {
    Transport*        transport;
    DiscoveryManager* discovery;
    Scheduler*        scheduler;

    DbscanCluster dbscan;

    /* 配置 */
    int    enabled;
    double min_range;        /* 最小有效距离 (m)，过滤近处噪声，默认 0.5 */
    double max_range;        /* 最大有效距离 (m)，过滤远处稀疏点，默认 8.0 */
    int    stride;           /* 深度图降采样步长（每隔 stride 像素取一个），默认 2 */
    double dbscan_eps;       /* DBSCAN 邻域半径 (m)，双目近距点密，默认 0.4 */
    int    dbscan_min_pts;   /* DBSCAN 最小邻域点数，默认 6 */
    double ground_z_thresh;  /* 地面 z 阈值（简化模式 z=0 不生效，保留接口），默认 0.1 */
    double min_cluster_size; /* 聚类最小点数（小于此视为噪声），默认 8 */
    char   output_topic[64]; /* 发布 topic，默认 perception/obstacles。
                                用 perception_fusion 融合时改 perception/obstacles_stereo */

    /* Phase 4: MLP 分类头 */
    int    cls_enabled;         /* 是否启用 MLP 分类 (默认 0,需要模型文件) */
    char   cls_model_path[256]; /* MLP 模型文件路径 */
    TinyMLP cls_model;          /* MLP 推理模型 */

    /* 缓冲（init 分配，cleanup 释放） */
    Point3D* point_buf;
    int      point_buf_cap;

    /* 最新 StereoFrame（订阅回调写入，DBSCAN 读取，mutex 保护） */
    StereoFrame last_frame;
    volatile int has_frame;
    pthread_mutex_t lock;

    /* 统计 */
    uint64_t frames_received;
    uint64_t obstacles_published;
    uint64_t clusters_total;
    uint32_t frame_id;
    time_t   last_frame_time;

    /* 托管模式：嵌入 TaskBase，由 node_start_managed 派生线程跑 stereo_execute。
     * 取代原先自管的 pthread thread / running / should_stop 三件套。 */
    TaskBase   taskbase;
} g;

/* ── 参数解析（cJSON，CLAUDE.md 唯一合法入口） ─────────────── */

/* ── ClusterClass → ObstacleType 映射 ─────────────────────── */
static int cluster_class_to_obs_type(ClusterClass cls) {
    switch (cls) {
        case CLS_VEHICLE:    return OBJ_TYPE_VEHICLE;
        case CLS_PEDESTRIAN: return OBJ_TYPE_PEDESTRIAN;
        case CLS_CYCLIST:    return OBJ_TYPE_CYCLIST;
        default:             return OBJ_TYPE_UNKNOWN;
    }
}

/* ── Phase 4: MLP 深度聚类分类器 ─────────────────────────────
 *
 * 从聚类包围盒 + 深度数据中提取 10 维特征向量，用 tiny_mlp 推理分类。
 * 特征 (SV_CLS_FEAT_DIM=10):
 *   [0] width       — 聚类 X 跨度 (m)
 *   [1] length      — 聚类 Y 跨度 (m)
 *   [2] height      — 聚类 Z 跨度 (m)
 *   [3] point_count — 聚类点数
 *   [4] depth_mean  — 平均深度 (m)
 *   [5] depth_std   — 深度标准差 (m)
 *   [6] density     — 点密度代理 (confidence)
 *   [7] cx          — 聚类中心 X (m, 前向)
 *   [8] cy          — 聚类中心 Y (m, 横向)
 *   [9] cz          — 聚类中心 Z (m, 高度)
 *
 * 输出: 4 类概率 [vehicle, pedestrian, cyclist, unknown]，取 argmax。
 * 模型未加载/未启用时返回 -1（调用方 fallback 到启发式分类）。
 */
static int classify_depth_cluster(const ClusterBounds* cb,
                                   const StereoFrame* frame) {
    if (!g.cls_enabled || !g.cls_model.loaded || !cb || !frame) return -1;

    /* 提取深度统计: 在深度图中采样与聚类中心深度接近的像素 */
    double depth_sum = 0.0, depth_sq = 0.0;
    int depth_cnt = 0;
    int dw = SV_DEPTH_W, dh = frame->depth_count / dw;
    if (dh <= 0 || dh > SV_DEPTH_H) dh = SV_DEPTH_H;

    for (int j = 0; j < dh; j++) {
        for (int i = 0; i < dw; i++) {
            int idx = j * dw + i;
            if (idx >= (int)frame->depth_count) continue;
            float d = frame->depth_data[idx];
            if (d < 0.3f || d > 8.0f) continue;
            if (fabs((double)d - (double)cb->cx) < (double)cb->cx * 0.3) {
                depth_sum += (double)d;
                depth_sq += (double)d * (double)d;
                depth_cnt++;
            }
        }
    }
    double depth_mean = (depth_cnt > 0) ? depth_sum / (double)depth_cnt : (double)cb->cx;
    double depth_var  = (depth_cnt > 1)
        ? (depth_sq - depth_sum * depth_sum / (double)depth_cnt) / (double)(depth_cnt - 1)
        : 0.0;
    if (depth_var < 0.0) depth_var = 0.0;

    float feat[SV_CLS_FEAT_DIM];
    feat[0] = cb->width;
    feat[1] = cb->length;
    feat[2] = cb->max_z - cb->min_z;
    feat[3] = (float)cb->point_count;
    feat[4] = (float)depth_mean;
    feat[5] = (float)sqrt(depth_var);
    feat[6] = cb->confidence;
    feat[7] = cb->cx;
    feat[8] = cb->cy;
    feat[9] = (cb->max_z + cb->min_z) * 0.5f;

    float prob[SV_CLS_NUM_CLASSES];
    if (tiny_mlp_forward(&g.cls_model, feat, prob) != SV_CLS_NUM_CLASSES) {
        return -1;
    }

    int best = 0;
    for (int k = 1; k < SV_CLS_NUM_CLASSES; k++) {
        if (prob[k] > prob[best]) best = k;
    }
    static const int8_t cls_map[SV_CLS_NUM_CLASSES] = {
        OBJ_TYPE_VEHICLE, OBJ_TYPE_PEDESTRIAN, OBJ_TYPE_CYCLIST, OBJ_TYPE_UNKNOWN
    };
    return (int)cls_map[best];
}

/* ── 订阅回调：收到 sensor/stereo ─────────────────────────── */
static void on_stereo(const Message* msg, void* user_data) {
    (void)user_data;
    if (!msg || !msg->data || !g.enabled) return;

    StereoFrame frame;
    if (StereoFrame_deserialize(&frame, (const uint8_t*)msg->data, msg->data_size) != 0) {
        LOG_WARN("stereo_vision", "StereoFrame deserialize failed (size=%u)", msg->data_size);
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

/* ── 深度反投影：StereoFrame → Point3D[] ────────────────────
 * 针孔模型简化版，把 80×60 深度图反投影到车体坐标系 2D 点云（z=0）。
 * @param frame   输入立体帧
 * @param points  输出点云缓冲
 * @param max_n   缓冲容量
 * @return 点数
 */
static int depth_to_points(const StereoFrame* frame, Point3D* points, int max_n) {
    if (!frame || !points || max_n <= 0) return 0;
    if (frame->depth_count == 0) return 0;

    /* 深度图实际尺寸：优先用 depth_count 推算，否则用默认 80×60 */
    int dw = SV_DEPTH_W;
    int dh = frame->depth_count / dw;
    if (dh <= 0) dh = SV_DEPTH_H;
    if (frame->depth_count < dw * dh) dh = frame->depth_count / dw;

    /* 水平视场角（弧度） */
    double fov_rad = (frame->fov_deg > 0.1 ? (double)frame->fov_deg : 65.0) * M_PI / 180.0;

    int n = 0;
    int stride = g.stride > 0 ? g.stride : 1;
    if (stride < 1) stride = 1;

    for (int j = 0; j < dh && n < max_n; j += stride) {
        for (int i = 0; i < dw && n < max_n; i += stride) {
            int idx = j * dw + i;
            if (idx >= (int)frame->depth_count) break;

            float depth = frame->depth_data[idx];
            /* 过滤无效深度（0 或 NaN 或超出范围） */
            if (depth < (float)g.min_range) continue;
            if (depth > (float)g.max_range) continue;
            if (depth != depth) continue;  /* NaN */

            /* 像素列 i 对应的水平角（中心 0，左 +，右 -） */
            double theta = ((double)(i + 0.5) / (double)dw - 0.5) * fov_rad;

            /* 反投影到车体坐标系：X=前, Y=左 */
            points[n].x = (float)((double)depth * cos(theta));
            points[n].y = (float)((double)depth * sin(theta));
            points[n].z = 0.0f;  /* 2D 简化，z=0 */
            points[n].intensity = 1.0f;  /* 双目无强度，统一填 1 */
            n++;
        }
    }
    return n;
}

/* ── 托管模式主循环：收到帧 → 反投影 → DBSCAN → 发布 ──────────────
 *
 * task_thread_fn 调用本函数一次（完整主循环），循环中检查 task->should_stop
 * 退出；task_stop() 置 should_stop=true 并 join 本线程。这与原先自管 pthread
 * 的 stereo_thread 行为等价，只是 should_stop 改由 TaskBase 提供。 */
static int stereo_execute(TaskBase* task) {
    pthread_setname_np(pthread_self(), "stereo_vis");

    /* 等 30ms 一帧（~30fps 上限），实际由 sensor/stereo 到达驱动 */
    while (!task->should_stop) {
        usleep(30000);
        if (task->should_stop) break;

        StereoFrame frame;
        pthread_mutex_lock(&g.lock);
        if (!g.has_frame) {
            pthread_mutex_unlock(&g.lock);
            continue;
        }
        frame = g.last_frame;
        g.has_frame = 0;  /* 消费掉，避免重复处理同一帧 */
        pthread_mutex_unlock(&g.lock);

        /* 1. 深度反投影 → 点云 */
        int n = depth_to_points(&frame, g.point_buf, g.point_buf_cap);
        if (n < g.dbscan_min_pts) continue;  /* 点太少，跳过 */

        /* 2. DBSCAN 聚类 */
        int n_clusters = dbscan_run(&g.dbscan, g.point_buf, n);
        g.clusters_total += (uint64_t)(n_clusters > 0 ? n_clusters : 0);

        /* 3. ClusterBounds → ObstacleList（最多 8 个，按点数排序取前 8） */
        ObstacleList obs_list;
        memset(&obs_list, 0, sizeof(obs_list));
        obs_list.frame_id     = g.frame_id++;
        obs_list.timestamp_us = clock_now_us();

        int cnt = 0;
        for (int i = 0; i < n_clusters && cnt < 8; i++) {
            const ClusterBounds* cb = dbscan_get_cluster(&g.dbscan, i);
            if (!cb) continue;
            /* 小聚类视为噪声 */
            if ((double)cb->point_count < g.min_cluster_size) continue;

            Obstacle* o = &obs_list.obstacles[cnt];
            o->id         = (uint32_t)i;
            o->x          = cb->cx;        /* 聚类中心 X（前向距离） */
            o->y          = cb->cy;        /* 聚类中心 Y（横向距离） */
            o->vx         = 0.0f;          /* 单帧无速度 */
            o->vy         = 0.0f;
            o->width      = cb->width;
            o->length     = cb->length;
            /* Phase 4: MLP 分类器优先,未加载则 fallback 到启发式 */
            int mlp_type = classify_depth_cluster(cb, &frame);
            o->type       = (mlp_type >= 0) ? (int8_t)mlp_type
                            : (int8_t)cluster_class_to_obs_type(cb->cls);
            o->confidence = cb->confidence;
            cnt++;
        }
        obs_list.count = (uint32_t)cnt;
        g.obstacles_published += (uint64_t)cnt;

        /* 4. 序列化 + 发布到 output_topic（默认 perception/obstacles） */
        if (cnt > 0) {
            uint8_t buf[280];
            size_t len = 0;
            if (ObstacleList_serialize(&obs_list, buf, &len) == 0 && len > 0) {
                transport_publish(g.transport, g.output_topic,
                                  buf, (uint32_t)len);
            }
        }

        /* 周期性日志 */
        if (g.frames_received % 30 == 1) {
            LOG_INFO("stereo_vision", "frame #%u: points=%d clusters=%d obstacles=%d",
                     obs_list.frame_id, n, n_clusters, cnt);
        }
    }
    return 0;
}

/* 托管模式虚函数表：仅实现 execute()（完整主循环）。initialize/cleanup 由
 * task_thread_fn 在 execute 前后按需调用，这里不需要——节点初始化在
 * NodePlugin.init，资源释放在 NodePlugin.cleanup。 */
static const TaskInterface stereo_vtable = {
    .execute = stereo_execute,
};

/* ── NodePlugin 实现 ─────────────────────────────────────── */

static const char* s_inputs[]  = { "sensor/stereo", NULL };
static const char* s_outputs[] = { "perception/obstacles", NULL };

static NodePlugin s_plugin;

static int stereo_vision_init(MessageBus* bus, Transport* transport,
                               DiscoveryManager* discovery, Scheduler* scheduler,
                               const char* params_json) {
    (void)bus;
    memset(&g, 0, sizeof(g));
    g.transport = transport;
    g.discovery = discovery;
    g.scheduler = scheduler;

    pthread_mutex_init(&g.lock, NULL);

    /* 默认参数 */
    g.enabled         = 1;
    g.min_range       = 0.5;
    g.max_range       = 8.0;
    g.stride          = 2;
    g.dbscan_eps      = 0.4;
    g.dbscan_min_pts  = 6;
    g.ground_z_thresh = 0.1;
    g.min_cluster_size = 8.0;
    snprintf(g.output_topic, sizeof(g.output_topic), "perception/obstacles");

    /* Phase 4: 分类器默认参数 */
    g.cls_enabled = 0;
    snprintf(g.cls_model_path, sizeof(g.cls_model_path), "models/stereo_classifier.txt");

    if (params_json) {
        cJSON* root = cJSON_Parse(params_json);
        if (root) {
            cJSON* j;
            g.enabled         = 1;
            if ((j = cJSON_GetObjectItem(root, "enable")) && cJSON_IsNumber(j)) g.enabled = j->valueint;
            g.min_range       = 0.5;
            if ((j = cJSON_GetObjectItem(root, "min_range")) && cJSON_IsNumber(j)) g.min_range = j->valuedouble;
            g.max_range       = 8.0;
            if ((j = cJSON_GetObjectItem(root, "max_range")) && cJSON_IsNumber(j)) g.max_range = j->valuedouble;
            g.stride          = 2;
            if ((j = cJSON_GetObjectItem(root, "stride")) && cJSON_IsNumber(j)) g.stride = j->valueint;
            g.dbscan_eps      = 0.4;
            if ((j = cJSON_GetObjectItem(root, "dbscan_eps")) && cJSON_IsNumber(j)) g.dbscan_eps = j->valuedouble;
            g.dbscan_min_pts  = 6;
            if ((j = cJSON_GetObjectItem(root, "dbscan_min_pts")) && cJSON_IsNumber(j)) g.dbscan_min_pts = j->valueint;
            g.ground_z_thresh = 0.1;
            if ((j = cJSON_GetObjectItem(root, "ground_z_thresh")) && cJSON_IsNumber(j)) g.ground_z_thresh = j->valuedouble;
            g.min_cluster_size = 8.0;
            if ((j = cJSON_GetObjectItem(root, "min_cluster_size")) && cJSON_IsNumber(j)) g.min_cluster_size = j->valuedouble;
            snprintf(g.output_topic, sizeof(g.output_topic), "%s", "perception/obstacles");
            if ((j = cJSON_GetObjectItem(root, "output_topic")) && cJSON_IsString(j))
                snprintf(g.output_topic, sizeof(g.output_topic), "%s", j->valuestring);

            /* Phase 4 分类器参数 */
            g.cls_enabled = 0;
            if ((j = cJSON_GetObjectItem(root, "cls_enabled")) && cJSON_IsNumber(j)) g.cls_enabled = j->valueint;
            snprintf(g.cls_model_path, sizeof(g.cls_model_path), "%s", "models/stereo_classifier.txt");
            if ((j = cJSON_GetObjectItem(root, "cls_model_path")) && cJSON_IsString(j))
                snprintf(g.cls_model_path, sizeof(g.cls_model_path), "%s", j->valuestring);
            cJSON_Delete(root);
        }
    }

    if (!g.enabled) {
        LOG_INFO("stereo_vision", "disabled by config (enable=0)");
        return 0;
    }

    /* Phase 4: 加载 MLP 分类器模型 */
    if (g.cls_enabled) {
        memset(&g.cls_model, 0, sizeof(g.cls_model));
        if (tiny_mlp_load(&g.cls_model, g.cls_model_path) == 0) {
            LOG_INFO("stereo_vision", "MLP classifier loaded: %s (in=%d hid=%d out=%d)",
                     g.cls_model_path, g.cls_model.in_dim,
                     g.cls_model.hid_dim, g.cls_model.out_dim);
        } else {
            LOG_WARN("stereo_vision", "MLP classifier load failed: %s, "
                     "fallback to heuristic", g.cls_model_path);
            g.cls_enabled = 0;
        }
    }

    /* 初始化 DBSCAN */
    dbscan_init(&g.dbscan, (float)g.dbscan_eps, g.dbscan_min_pts);
    g.dbscan.ground_z_thresh = (float)g.ground_z_thresh;

    /* 分配点云缓冲 */
    g.point_buf_cap = SV_MAX_POINTS;
    g.point_buf = (Point3D*)malloc(sizeof(Point3D) * (size_t)g.point_buf_cap);
    if (!g.point_buf) {
        LOG_ERROR("stereo_vision", "point buffer alloc failed");
        return -1;
    }

    /* 订阅 sensor/stereo */
    transport_subscribe(transport, "sensor/stereo", on_stereo, NULL);
    discovery_advertise(discovery, "sensor/stereo", STEREOFRAME_TYPE_ID, CAP_SUBSCRIBER, 0);

    /* 发布 output_topic（默认 perception/obstacles） */
    discovery_advertise(discovery, g.output_topic,
                        OBSTACLELIST_TYPE_ID, CAP_PUBLISHER, 30.0);

    /* 托管模式：初始化嵌入的 TaskBase 并挂上 vtable。s_plugin.taskbase 在
     * 静态初始化里已指向 &g.taskbase，故此处只需填好其内容。max_frequency_hz
     * 喂给调度器 RateControl，与 execute() 内 usleep 周期一致（30ms → ~33Hz）。 */
    TaskConfig cfg;
    memset(&cfg, 0, sizeof(cfg));
    snprintf(cfg.name, sizeof(cfg.name), "stereo_vision");
    cfg.priority         = TASK_PRIORITY_NORMAL;
    cfg.max_frequency_hz = 1000000.0 / 30000.0;
    cfg.enable_stats     = true;
    if (task_base_init(&g.taskbase, &stereo_vtable, &cfg) != 0) {
        LOG_WARN("stereo_vision", "task_base_init failed");
        return -1;
    }

    g.last_frame_time = time(NULL);

    LOG_INFO("stereo_vision", "initialized: range=[%.1f,%.1f]m stride=%d "
             "eps=%.2f min_pts=%d min_cluster=%.0f",
             g.min_range, g.max_range, g.stride,
             g.dbscan_eps, g.dbscan_min_pts, g.min_cluster_size);
    return 0;
}

static int stereo_vision_start(void) {
    if (!g.enabled) return 0;
    /* 托管模式：node_start_managed 注册 taskbase 到调度器并派生工作线程跑
     * stereo_execute()。节点不再 pthread_create 自建线程。 */
    int rc = node_start_managed(&s_plugin, g.scheduler);
    if (rc != 0) {
        LOG_WARN("stereo_vision", "node_start_managed failed: %d", rc);
        return rc;
    }
    LOG_INFO("stereo_vision", "started (managed)");
    node_announce_self(g.transport, &s_plugin);
    return 0;
}

static void stereo_vision_stop(void) {
    /* task_stop 置 should_stop=true 并 join 工作线程（stereo_execute 随即退出）。
     * launcher 保证 stop() 在 cleanup() 前调用，故此处阻塞 join 是安全的。 */
    task_stop(&g.taskbase);
}

static void stereo_vision_cleanup(void) {
    /* stop() 已 join 线程；此处再 task_stop 一次作幂等保险（STOPPED 态直接
     * 返回 0），随后释放 TaskBase 资源（互斥锁等）。 */
    task_stop(&g.taskbase);
    task_base_destroy(&g.taskbase);
    if (g.point_buf) { free(g.point_buf); g.point_buf = NULL; }
    pthread_mutex_destroy(&g.lock);
    LOG_INFO("stereo_vision", "cleanup: frames=%lu clusters=%lu obstacles=%lu",
             (unsigned long)g.frames_received,
             (unsigned long)g.clusters_total,
             (unsigned long)g.obstacles_published);
}

static int stereo_vision_health(void) {
    if (!g.enabled) return 0;
    time_t now = time(NULL);
    /* 60 秒未收到任何帧视为异常（双目相机掉线） */
    if (g.frames_received == 0 && (now - g.last_frame_time) > 60) return -1;
    return 0;
}

static NodePlugin s_plugin = {
    .api_version   = NODE_PLUGIN_API_VERSION,
    .name          = "stereo_vision",
    .version       = "1.0.0",
    .description   = "Stereo vision perception (StereoFrame depth → DBSCAN → ObstacleList)",
    .input_topics  = s_inputs,
    .output_topics = s_outputs,
    .init          = stereo_vision_init,
    .start         = stereo_vision_start,
    .stop          = stereo_vision_stop,
    .cleanup       = stereo_vision_cleanup,
    .health        = stereo_vision_health,
    .taskbase      = &g.taskbase,   /* v2: 托管模式钩子，指向嵌入的 TaskBase */
};

NodePlugin* node_get_plugin(void) { return &s_plugin; }
