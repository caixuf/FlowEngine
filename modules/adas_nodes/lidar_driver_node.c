/**
 * lidar_driver_node.c — 真实 LiDAR 驱动节点插件 (点云 → DBSCAN → ObstacleList)
 *
 * 从串口/UDP 直接读取真实 LiDAR 点云，做 DBSCAN 聚类后发布障碍物列表到
 * perception/obstacles。这是真车感知入口，替代仿真用的 perception_node
 * （后者依赖 vehicle/state 真值生成"理想"障碍物，无法上真车）：
 *
 *   真车链路: [lidar_driver_node] → perception/obstacles → fusion_node → planning_node
 *   仿真链路: sim_world_node → vehicle/state → perception_node → perception/obstacles
 *
 * 本节点绕过 perception_node 的真值依赖：直接读点云 → DBSCAN → 发 ObstacleList，
 * 让 fusion/planning 在真车上也能拿到感知结果。常见 LiDAR 硬件：
 *   - RPLIDAR A1/A2 (2D, 串口二进制协议, /dev/ttyUSB0, 115200/256000 baud)
 *   - 速腾聚创 RoboSense M1 (3D, UDP, 固态)
 *   - Velodyne VLP-16 (16线, UDP PCAP, 360°)
 *
 * 设计要点：
 *   - read_lidar_frame() 是**硬件适配点**。默认实现一个简化协议：假设串口输出
 *     ASCII 格式点云（每行 "x,y,z,intensity\n"），用 serial_read_line 逐行解析
 *     直到凑够一帧（max_points 个点或超时）。社区用户按自己 LiDAR 协议改这个
 *     函数即可，例如接 RPLIDAR SDK 或速腾浦 SDK，把点云填到 Point3D 数组返回点数。
 *   - 串口打开失败（沙箱/开发机/无硬件）自动降级为 dry_run：生成模拟点云
 *     （几个车辆/行人形状的点簇 + 随机噪声），走完整 DBSCAN + publish 链路，
 *     让 fusion/planning 算法链在无硬件时也能测通。
 *   - 单帧无速度信息：vx=vy=0（速度估计由 fusion_node 跨帧关联给出）。
 *
 * 话题契约：
 *   无输入 topic（直接从硬件读点云）
 *   输出: perception/obstacles (ObstacleList 二进制, type_id=0x308f5f71)
 *
 * 典型 pipeline.json 配置:
 *   {
 *     "name": "lidar_driver",
 *     "library": "liblidar_driver_node.so",
 *     "params": {
 *       "serial_port": "/dev/ttyUSB1",
 *       "baud_rate": 115200,
 *       "enable": true,
 *       "scan_hz": 10,
 *       "dbscan_eps": 2.0,
 *       "dbscan_min_pts": 4,
 *       "ground_z_thresh": 0.15,
 *       "max_points": 2000,
 *       "max_range_m": 60.0,
 *       "dry_run": false
 *     }
 *   }
 *
 * 编译依赖: serial_port.h (POSIX termios, Linux), dbscan_cluster.h, adas_msgs_gen.h
 */

#include "node_plugin.h"
#include "adas_msgs_gen.h"
#include "serial_port.h"
#include "dbscan_cluster.h"
#include "transport.h"
#include "discovery.h"
#include "logger.h"
#include "clock_service.h"
#include <cjson/cJSON.h>

#include <math.h>
#include <pthread.h>
#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

/* ── 节点状态 ─────────────────────────────────────────────── */

static struct {
    Transport*        transport;
    DiscoveryManager* discovery;
    Scheduler*        scheduler;

    /* 硬件句柄，NULL 表示未连接（dry-run 模式） */
    SerialPort*   serial;
    DbscanCluster dbscan;

    /* 配置参数（pipeline.json 注入） */
    char   serial_port[64];     /* 串口设备路径，默认 "/dev/ttyUSB1" */
    int    baud_rate;           /* 串口波特率，默认 115200 */
    int    enabled;             /* 总开关，false=不读 LiDAR 也不发布 */
    int    scan_hz;             /* 扫描/发布频率，默认 10Hz */
    float  dbscan_eps;          /* DBSCAN 邻域半径 (m)，默认 2.0 */
    int    dbscan_min_pts;      /* DBSCAN 最小邻域点数，默认 4 */
    float  ground_z_thresh;     /* 地面 z 阈值 (m)，低于此值视为地面，默认 0.15 */
    int    max_points;          /* 单帧最大点数，默认 2000 */
    float  max_range_m;         /* 最大有效距离 (m)，超出丢弃，默认 60.0 */
    int    dry_run;             /* true=生成模拟点云不走真实串口（开发调试） */
    char   output_topic[64];    /* 发布 topic，默认 perception/obstacles。
                                  用 perception_fusion 融合时改 perception/obstacles_lidar */

    /* 运行时 */
    Point3D* point_buf;         /* 点云缓冲（init 时分配，cleanup 时释放） */
    int      point_buf_cap;     /* 缓冲容量 */
    uint32_t frame_id;          /* 发布帧计数 */

    /* 统计 */
    uint64_t frames_read;          /* 成功读到的帧数 */
    uint64_t clusters_total;       /* 累计聚类数 */
    uint64_t obstacles_published;  /* 累计发布的障碍物数 */
    time_t   last_frame_time;      /* 最近一次成功读帧的时间（health 用） */

    /* 托管模式：嵌入 TaskBase，由 node_start_managed 派生线程跑 lidar_execute。
     * 取代原先自管的 pthread thread / running / should_stop 三件套。 */
    TaskBase taskbase;
} g;

/* [lo, hi] 均匀随机浮点。 */
static float frand(float lo, float hi) {
    return lo + (hi - lo) * ((float)rand() / (float)RAND_MAX);
}

/* ── ClusterClass → ObstacleType 映射 ─────────────────────── */

static int cluster_class_to_obs_type(ClusterClass cls) {
    switch (cls) {
        case CLS_VEHICLE:    return OBJ_TYPE_VEHICLE;
        case CLS_PEDESTRIAN: return OBJ_TYPE_PEDESTRIAN;
        case CLS_CYCLIST:    return OBJ_TYPE_CYCLIST;
        default:             return OBJ_TYPE_UNKNOWN;
    }
}

/* ── dry-run 模拟点云生成 ───────────────────────────────────
 * 生成 ~200 个点，分布在 4 个簇（2 辆车 + 1 行人 + 1 骑行者），
 * 加 ~20 个散点噪声。簇中心间隔 > 2*eps 避免被 DBSCAN 合并。
 * 加小幅正弦漂移让障碍物位置逐帧变化，模拟运动目标。
 */
static int generate_dry_run_points(Point3D* points, int max_points) {
    struct ClusterTemplate {
        float cx, cy, cz0;   /* 中心 + 底部高度 */
        float w, l, h;       /* 包围盒宽、长、高 (m) */
        int   n;              /* 该簇生成点数 */
    };
    static const struct ClusterTemplate tpls[] = {
        /* 车辆1: 前方偏右 */
        { 12.0f, -3.0f, 0.0f, 1.8f, 4.2f, 1.5f, 50 },
        /* 车辆2: 左前方 */
        {  8.0f,  3.5f, 0.0f, 1.8f, 3.8f, 1.5f, 50 },
        /* 行人: 右前方远处 */
        {  6.0f, -5.5f, 0.0f, 0.6f, 0.6f, 1.7f, 40 },
        /* 骑行者: 左前方远处 */
        { 16.0f,  2.5f, 0.0f, 0.7f, 1.8f, 1.7f, 40 },
    };
    int n = 0;
    /* 整体小幅前后漂移，模拟目标运动（±1m） */
    float drift = (float)sin((double)g.frame_id * 0.1) * 1.0f;
    for (size_t i = 0; i < sizeof(tpls) / sizeof(tpls[0]) && n < max_points; i++) {
        const struct ClusterTemplate* t = &tpls[i];
        for (int k = 0; k < t->n && n < max_points; k++) {
            points[n].x = t->cx + drift + frand(-t->w * 0.5f, t->w * 0.5f);
            points[n].y = t->cy          + frand(-t->l * 0.5f, t->l * 0.5f);
            points[n].z = t->cz0         + frand(0.2f, t->h);
            points[n].intensity = frand(0.3f, 1.0f);
            n++;
        }
    }
    /* 散点噪声：稀疏分布，DBSCAN 视为噪声（min_pts=4 不够成簇） */
    int noise = 20;
    for (int k = 0; k < noise && n < max_points; k++) {
        points[n].x = frand(-5.0f, 30.0f);
        points[n].y = frand(-10.0f, 10.0f);
        points[n].z = frand(0.2f, 2.0f);
        points[n].intensity = frand(0.0f, 0.3f);
        n++;
    }
    return n;
}

/* ── 读取一帧 LiDAR 点云 ───────────────────────────────────
 *
 * 硬件适配点：真实 LiDAR 每家协议不同。
 *   - RPLIDAR A1/A2: 串口二进制协议（需 RPLIDAR SDK 解析扫描帧）
 *   - 速腾浦 M1:    UDP 数据包（需 RS-SDK 解析点云）
 *   - Velodyne VLP-16: UDP PCAP（需 velodyne_decoder 解析）
 *
 * 默认实现一个**简化协议**：假设串口输出 ASCII 格式点云，每行
 *   "x,y,z,intensity\n"  (intensity 可选)
 * 用 serial_read_line 逐行解析，凑够一帧（max_points 个点）或
 * 超过 frame_budget_ms 即返回。
 *
 * 社区用户按自己 LiDAR 协议改这个函数：把点云填到 points[] 数组，
 * 返回点数即可，下游 DBSCAN + publish 逻辑无需改动。
 *
 * @param points           输出点云缓冲
 * @param max_points       缓冲容量
 * @param frame_budget_ms  本帧读取预算（毫秒）
 * @return 读到的点数；0 表示本帧无数据
 */
static int read_lidar_frame(Point3D* points, int max_points, int frame_budget_ms) {
    if (g.dry_run) {
        return generate_dry_run_points(points, max_points);
    }
    if (!g.serial || max_points <= 0) {
        return 0;
    }

    int n = 0;
    int per_line_to = 50;
    if (per_line_to > frame_budget_ms && frame_budget_ms > 0) {
        per_line_to = frame_budget_ms;
    }
    long deadline_ms = (long)(clock_now_realtime_us()/1000) + frame_budget_ms;

    while (n < max_points) {
        long now = (long)(clock_now_realtime_us()/1000);
        if (n > 0 && now >= deadline_ms) break;  /* 已有部分点 + 超预算，返回 */

        int to = per_line_to;
        long left = deadline_ms - now;
        if (left < to) to = (int)(left > 0 ? left : 1);
        if (to < 1) to = 1;

        char line[160];
        int r = serial_read_line(g.serial, line, sizeof(line), to);
        if (r < 0) {
            /* 设备错误（断开/硬件故障） */
            LOG_WARN("lidar_driver", "serial_read_line error: %d (device disconnected?)", r);
            break;
        }
        if (r == 0) {
            /* 本行超时：已有部分点则返回，否则继续等直到 deadline */
            if (n > 0) break;
            if (now >= deadline_ms) break;
            continue;
        }

        /* 解析 "x,y,z,intensity" (intensity 可选) */
        float x = 0.0f, y = 0.0f, z = 0.0f, intensity = 0.0f;
        int matched = sscanf(line, "%f,%f,%f,%f", &x, &y, &z, &intensity);
        if (matched < 3) continue;  /* 不可解析的行，跳过 */

        /* 距离过滤 */
        float d2 = x * x + y * y;
        if (d2 > g.max_range_m * g.max_range_m) continue;
        /* 地面过滤 */
        if (z < g.ground_z_thresh) continue;

        points[n].x = x;
        points[n].y = y;
        points[n].z = z;
        points[n].intensity = intensity;
        n++;
    }
    return n;
}

/* ── 托管模式主循环：读帧 → DBSCAN → 发布 ObstacleList ────────────
 *
 * task_thread_fn 调用本函数一次（完整主循环），循环中检查 task->should_stop
 * 退出；task_stop() 置 should_stop=true 并 join 本线程。这与原先自管 pthread
 * 的 lidar_thread 行为等价，只是 should_stop 改由 TaskBase 提供。 */
static int lidar_execute(TaskBase* task) {
    pthread_setname_np(pthread_self(), "lidar_drv");

    long period_ms = 1000L / (g.scan_hz > 0 ? g.scan_hz : 10);

    while (!task->should_stop) {
        if (!g.enabled) {
            usleep((unsigned long)period_ms * 1000UL);
            continue;
        }

        long t0 = (long)(clock_now_realtime_us()/1000);

        /* a. 读取一帧点云 */
        int n = read_lidar_frame(g.point_buf, g.point_buf_cap, (int)period_ms);
        if (n <= 0) {
            /* 本帧无数据（串口超时且未凑到点），按周期重试 */
            long elapsed = (long)(clock_now_realtime_us()/1000) - t0;
            long remain = period_ms - elapsed;
            if (remain > 0) usleep((unsigned long)remain * 1000UL);
            continue;
        }
        g.frames_read++;
        g.last_frame_time = time(NULL);

        /* b. DBSCAN 聚类 */
        int n_clusters = dbscan_run(&g.dbscan, g.point_buf, n);
        g.clusters_total += (uint64_t)(n_clusters > 0 ? n_clusters : 0);

        /* c. ClusterBounds → ObstacleList（最多 8 个） */
        ObstacleList obs_list;
        memset(&obs_list, 0, sizeof(obs_list));
        obs_list.frame_id     = g.frame_id++;
        obs_list.timestamp_us = clock_now_us();

        int cnt = 0;
        for (int i = 0; i < n_clusters && cnt < 8; i++) {
            const ClusterBounds* cb = dbscan_get_cluster(&g.dbscan, i);
            if (!cb) continue;
            Obstacle* o = &obs_list.obstacles[cnt];
            o->id         = (uint32_t)i;          /* 簇索引作为 id */
            o->x          = cb->cx;
            o->y          = cb->cy;
            o->vx         = 0.0f;                  /* 单帧无速度 */
            o->vy         = 0.0f;
            o->width      = cb->width;
            o->length     = cb->length;
            o->type       = (int8_t)cluster_class_to_obs_type(cb->cls);
            o->confidence = cb->confidence;
            cnt++;
        }
        obs_list.count = (uint32_t)cnt;
        g.obstacles_published += (uint64_t)cnt;

        /* d. 序列化 + 发布到 output_topic（默认 perception/obstacles） */
        uint8_t buf[280];
        size_t len = 0;
        if (ObstacleList_serialize(&obs_list, buf, &len) == 0 && len > 0) {
            transport_publish(g.transport, g.output_topic,
                              buf, (uint32_t)len);
        }

        /* 周期性日志（每 50 帧一次，避免刷屏） */
        if (g.frames_read % 50 == 1) {
            LOG_INFO("lidar_driver", "frame #%u: points=%d clusters=%d obstacles=%d %s",
                     obs_list.frame_id, n, n_clusters, cnt,
                     g.dry_run ? "[DRY-RUN]" : "[LIVE]");
        }

        /* 按周期定频：睡剩余时间 */
        long elapsed = (long)(clock_now_realtime_us()/1000) - t0;
        long remain = period_ms - elapsed;
        if (remain > 0) usleep((unsigned long)remain * 1000UL);
    }
    return 0;
}

/* 托管模式虚函数表：仅实现 execute()（完整主循环）。initialize/cleanup 由
 * task_thread_fn 在 execute 前后按需调用，这里不需要——节点初始化在
 * NodePlugin.init，资源释放在 NodePlugin.cleanup。 */
static const TaskInterface lidar_vtable = {
    .execute = lidar_execute,
};

/* ── NodePlugin 实现 ─────────────────────────────────────── */

static const char* s_inputs[]  = { NULL };
static const char* s_outputs[] = { "perception/obstacles", NULL };

static NodePlugin s_plugin;

static int lidar_init(MessageBus* bus, Transport* transport,
                      DiscoveryManager* discovery, Scheduler* scheduler,
                      const char* params_json) {
    (void)bus;
    g.scheduler = scheduler;
    memset(&g, 0, sizeof(g));
    g.transport = transport;
    g.discovery = discovery;
    g.serial    = NULL;

    /* 默认参数 */
    snprintf(g.serial_port, sizeof(g.serial_port), "/dev/ttyUSB1");
    g.baud_rate        = 115200;
    g.enabled          = 1;
    g.scan_hz          = 10;
    g.dbscan_eps       = 2.0f;
    g.dbscan_min_pts   = 4;
    g.ground_z_thresh  = 0.15f;
    g.max_points       = 2000;
    g.max_range_m      = 60.0f;
    g.dry_run          = 0;
    snprintf(g.output_topic, sizeof(g.output_topic), "perception/obstacles");

    /* cJSON 参数解析（替代手写 parse_*，CLAUDE.md 规范唯一合法入口） */
    if (params_json) {
        cJSON* root = cJSON_Parse(params_json);
        if (root) {
            cJSON* j;
            snprintf(g.serial_port, sizeof(g.serial_port), "%s", "/dev/ttyUSB1");
            if ((j = cJSON_GetObjectItem(root, "serial_port")) && cJSON_IsString(j))
                snprintf(g.serial_port, sizeof(g.serial_port), "%s", j->valuestring);
            g.baud_rate = 115200;
            if ((j = cJSON_GetObjectItem(root, "baud_rate")) && cJSON_IsNumber(j))
                g.baud_rate = j->valueint;
            g.enabled = 1;
            if ((j = cJSON_GetObjectItem(root, "enable")) && cJSON_IsNumber(j))
                g.enabled = j->valueint;
            g.scan_hz = 10;
            if ((j = cJSON_GetObjectItem(root, "scan_hz")) && cJSON_IsNumber(j))
                g.scan_hz = j->valueint;
            g.dbscan_eps = (float)2.0;
            if ((j = cJSON_GetObjectItem(root, "dbscan_eps")) && cJSON_IsNumber(j))
                g.dbscan_eps = (float)j->valuedouble;
            g.dbscan_min_pts = 4;
            if ((j = cJSON_GetObjectItem(root, "dbscan_min_pts")) && cJSON_IsNumber(j))
                g.dbscan_min_pts = j->valueint;
            g.ground_z_thresh = (float)0.15;
            if ((j = cJSON_GetObjectItem(root, "ground_z_thresh")) && cJSON_IsNumber(j))
                g.ground_z_thresh = (float)j->valuedouble;
            g.max_points = 2000;
            if ((j = cJSON_GetObjectItem(root, "max_points")) && cJSON_IsNumber(j))
                g.max_points = j->valueint;
            g.max_range_m = (float)60.0;
            if ((j = cJSON_GetObjectItem(root, "max_range_m")) && cJSON_IsNumber(j))
                g.max_range_m = (float)j->valuedouble;
            g.dry_run = 0;
            if ((j = cJSON_GetObjectItem(root, "dry_run")) && cJSON_IsNumber(j))
                g.dry_run = j->valueint;
            snprintf(g.output_topic, sizeof(g.output_topic), "%s", "perception/obstacles");
            if ((j = cJSON_GetObjectItem(root, "output_topic")) && cJSON_IsString(j))
                snprintf(g.output_topic, sizeof(g.output_topic), "%s", j->valuestring);
            cJSON_Delete(root);
        }
    }

    if (!g.enabled) {
        LOG_INFO("lidar_driver", "disabled by config (enable=0), will not publish");
        return 0;
    }

    /* 初始化 DBSCAN 聚类器 + 同步地面 z 阈值 */
    dbscan_init(&g.dbscan, g.dbscan_eps, g.dbscan_min_pts);
    g.dbscan.ground_z_thresh = g.ground_z_thresh;

    /* 分配点云缓冲（容量做上下限钳位，避免极端配置爆栈/超限） */
    int cap = g.max_points;
    if (cap > 20000) cap = 20000;
    if (cap < 64)     cap = 64;
    g.point_buf_cap = cap;
    g.point_buf = (Point3D*)malloc(sizeof(Point3D) * (size_t)cap);
    if (!g.point_buf) {
        LOG_ERROR("lidar_driver", "point buffer alloc failed (cap=%d), abort init", cap);
        return -1;
    }

    /* 初始化随机数（dry-run 模拟点云用） */
    srand((unsigned)time(NULL) ^ (unsigned)getpid());
    g.last_frame_time = time(NULL);

    /* 尝试打开串口（失败则自动降级为 dry_run） */
    if (!g.dry_run) {
        g.serial = serial_open(g.serial_port, g.baud_rate);
        if (!g.serial) {
            LOG_WARN("lidar_driver", "serial_open('%s', %d) failed, falling back to dry-run "
                     "(真实硬件部署时检查设备权限/路径，社区用户改 read_lidar_frame 适配 UDP LiDAR)",
                     g.serial_port, g.baud_rate);
            g.dry_run = 1;
        } else {
            LOG_INFO("lidar_driver", "serial opened: %s @ %d baud", g.serial_port, g.baud_rate);
        }
    }

    /* 广播 output_topic 为发布者（供 fusion_node 等订阅方发现） */
    discovery_advertise(discovery, g.output_topic,
                        OBSTACLELIST_TYPE_ID, CAP_PUBLISHER, (double)g.scan_hz);

    /* 托管模式：初始化嵌入的 TaskBase 并挂上 vtable。s_plugin.taskbase 在
     * 静态初始化里已指向 &g.taskbase，故此处只需填好其内容。max_frequency_hz
     * 喂给调度器 RateControl，与 execute() 内 usleep 周期一致。 */
    TaskConfig cfg;
    memset(&cfg, 0, sizeof(cfg));
    snprintf(cfg.name, sizeof(cfg.name), "lidar_driver");
    cfg.priority         = TASK_PRIORITY_NORMAL;
    cfg.max_frequency_hz = (double)g.scan_hz;
    cfg.enable_stats     = true;
    if (task_base_init(&g.taskbase, &lidar_vtable, &cfg) != 0) {
        LOG_WARN("lidar_driver", "task_base_init failed");
        return -1;
    }

    LOG_INFO("lidar_driver", "initialized: serial=%s baud=%d scan=%dHz "
             "eps=%.2f min_pts=%d ground_z=%.2f max_pts=%d max_range=%.1fm %s",
             g.serial_port, g.baud_rate, g.scan_hz,
             g.dbscan_eps, g.dbscan_min_pts, g.ground_z_thresh,
             g.max_points, g.max_range_m,
             g.dry_run ? "[DRY-RUN]" : "[LIVE]");

    if (g.dry_run) {
        LOG_INFO("lidar_driver", "DRY-RUN 模式：生成模拟点云（2 车 + 1 行人 + 1 骑行者）走完整 "
                 "DBSCAN + publish 链路，供 fusion/planning 算法链测通。"
                 "真车部署时连接 LiDAR 并把 dry_run=false。"
                 "社区用户按自己 LiDAR 协议（RPLIDAR/速腾浦/Velodyne）改 read_lidar_frame()。");
    }
    return 0;
}

static int lidar_start(void) {
    if (!g.enabled) return 0;
    int rc = node_start_managed(&s_plugin, g.scheduler);
    if (rc != 0) {
        LOG_WARN("lidar_driver", "node_start_managed failed: %d", rc);
        return rc;
    }
    LOG_INFO("lidar_driver", "started (managed) (scan %dHz, %s)",
             g.scan_hz, g.dry_run ? "dry-run" : "live serial");
    node_announce_self(g.transport, &s_plugin);
    return 0;
}

static void lidar_stop(void) {
    task_stop(&g.taskbase);
}

static void lidar_cleanup(void) {
    task_stop(&g.taskbase);
    task_base_destroy(&g.taskbase);
    if (g.point_buf) { free(g.point_buf); g.point_buf = NULL; g.point_buf_cap = 0; }
    if (g.serial) { serial_close(g.serial); g.serial = NULL; }
    LOG_INFO("lidar_driver", "cleanup: frames=%lu clusters=%lu obstacles=%lu",
             (unsigned long)g.frames_read,
             (unsigned long)g.clusters_total,
             (unsigned long)g.obstacles_published);
}

static int lidar_health(void) {
    if (!g.enabled) return 0;
    /* 健康判定：若至今未读到任何帧且已超过 60 秒，视为异常 */
    time_t now = time(NULL);
    if (g.frames_read == 0 && (now - g.last_frame_time) > 60) return -1;
    return 0;
}

static NodePlugin s_plugin = {
    .api_version   = NODE_PLUGIN_API_VERSION,
    .name          = "lidar_driver",
    .version       = "1.0.0",
    .description   = "Real LiDAR driver (point cloud → DBSCAN → ObstacleList on perception/obstacles)",
    .input_topics  = s_inputs,
    .output_topics = s_outputs,
    .init          = lidar_init,
    .start         = lidar_start,
    .stop          = lidar_stop,
    .cleanup       = lidar_cleanup,
    .health        = lidar_health,
    .taskbase      = &g.taskbase,
};

NodePlugin* node_get_plugin(void) { return &s_plugin; }
