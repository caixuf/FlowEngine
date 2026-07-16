/**
 * waypoint_follower_node.c — Pure Pursuit 航点跟随节点 (L2 自主跟随)
 *
 * 为 RC 小车提供 L2 级"航点跟随 + 局部避障"能力。读取预录制的 GPS 航点列表，
 * 用 Pure Pursuit 算法跟踪航点，输出 planning/trajectory（覆盖 planning_node）：
 *
 *   [航点文件] + fusion/localization → Pure Pursuit → planning/trajectory → control
 *
 * ── 与 planning_node 的关系 ──
 *   planning_node 是为高速 NOA 场景设计的（Frenet + NOA 状态机 + 高速变道），
 *   对 RC 小车（<5m/s，校园/公园）来说过重且参数不匹配。本节点是 RC 小车的
 *   轻量规划器，pipeline_car.json 里二选一：
 *     - 高速真车: 用 planning_node (libplanning_node.so)
 *     - RC 小车 : 用 waypoint_follower (libwaypoint_follower_node.so)
 *
 * ── Pure Pursuit 算法 ──
 *   经典几何路径跟踪算法：
 *     1. 在航点列表里找距离自车最近的"前瞻点"（lookahead distance L_d）
 *     2. 计算自车到前瞻点的横向偏移 e_y 与航向偏差
 *     3. 曲率 κ = 2*e_y / L_d²
 *     4. 期望转向角 δ = atan(κ * L)（L=wheelbase，由 control_node 处理）
 *   这里只输出"路径点 + 目标速度"，转向计算由 control_node 完成（control 已支持
 *   JSON trajectory 的 path 字段解析）。
 *
 * ── 航点文件格式 (JSON) ──
 *   {
 *     "wheelbase": 0.3,
 *     "lookahead_m": 1.5,
 *     "cruise_speed": 2.0,
 *     "waypoints": [
 *       {"x": 0.0, "y": 0.0},
 *       {"x": 5.0, "y": 0.0},
 *       {"x": 10.0, "y": 5.0}
 *     ]
 *   }
 *   x/y 为局部笛卡尔坐标（米），由 GPS 航点录制工具 tools/waypoint_record.py
 *   把经纬度转换后写入。循环模式（loop=true）下到达终点后回到第 0 个航点。
 *
 * ── 避障 ──
 *   本节点订阅 perception/obstacles，遇前方近距离障碍物自动减速（不做绕行，
 *   绕行交给 control/safety 的限速逻辑）。RC 小车场景"减速→停"已足够。
 *
 * 话题契约:
 *   输入: fusion/localization (Localization 二进制 or JSON)
 *         perception/obstacles (ObstacleList, 可选 — 用于前方障碍物减速)
 *   输出: planning/trajectory (JSON 文本, 与 planning_node 兼容格式)
 *
 * 典型 pipeline_car.json 配置:
 *   {
 *     "name": "waypoint_follower",
 *     "library": "libwaypoint_follower_node.so",
 *     "subscribe": ["fusion/localization", "perception/obstacles"],
 *     "publish": [{"topic": "planning/trajectory", "type": "Trajectory"}],
 *     "params": "{\"waypoints_file\":\"/tmp/waypoints.json\",\"loop\":true,\"cruise_speed\":2.0,\"lookahead_m\":1.5}"
 *   }
 */

#include "node_plugin.h"
#include "adas_msgs_gen.h"
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

#define WF_MAX_WAYPOINTS   512
#define WF_TRAJ_JSON_LEN   1200
#define WF_PATH_SAMPLES    16       /* trajectory JSON 里输出多少个前瞻路径点 */

/* ── 节点状态 ─────────────────────────────────────────────── */

static struct {
    Transport*        transport;
    DiscoveryManager* discovery;
    Scheduler*        scheduler;

    /* 配置 */
    char   waypoints_file[256];     /* 航点 JSON 文件路径 */
    int    loop;                    /* true=到达终点循环回起点 */
    double cruise_speed;            /* 巡航速度 (m/s) */
    double lookahead_m;             /* Pure Pursuit 前瞻距离 (m) */
    int    plan_hz;                 /* 规划频率，默认 10Hz */
    double obstacle_slow_dist;      /* 前方障碍物减速触发距离 (m) */
    double obstacle_stop_dist;      /* 前方障碍物停车距离 (m) */

    /* 航点列表（从文件加载） */
    double wpx[WF_MAX_WAYPOINTS];
    double wpy[WF_MAX_WAYPOINTS];
    int    wp_count;
    int    wp_current;              /* 下一个待跟踪的航点索引 */

    /* 自车状态（订阅 fusion/localization 写入，mutex 保护） */
    double ego_x, ego_y, ego_v, ego_heading;
    volatile int has_fusion;
    pthread_mutex_t lock;

    /* 障碍物（订阅 perception/obstacles 写入） */
    double nearest_obs_x;           /* 前方最近障碍物 x（自车坐标系） */
    double nearest_obs_y;
    volatile int has_obstacle;
    time_t last_obstacle_time;

    /* 可通行区域（订阅 perception/traversability 写入, Phase 1） */
    double traversability_nearest_obs;  /* 双目检测到的前方最近障碍 (m), -1 无效 */
    double traversability_corridor_w;   /* 最宽可通行走廊宽度 (m) */
    volatile int traversability_blocked; /* 前方无可行走走廊? 1=堵死 */
    volatile int has_traversability;
    time_t last_traversability_time;

    /* 统计 */
    uint64_t plans_published;
    uint64_t wp_advanced;
    int    lap_count;               /* 循环模式下完成的圈数 */

    /* 工作线程 */
    pthread_t thread;
    volatile int thread_running;
    volatile int should_stop;
} g;

/* ── 加载航点文件 ───────────────────────────────────────────
 * 简易 JSON 解析：扫描 {"x":...,"y":...} 对。
 * 文件格式见文件头注释。
 */
static int load_waypoints(const char* path) {
    FILE* f = fopen(path, "r");
    if (!f) {
        LOG_WARN("waypoint_follower", "无法打开航点文件: %s", path);
        return -1;
    }

    /* 读全文到内存 */
    fseek(f, 0, SEEK_END);
    long sz = ftell(f);
    fseek(f, 0, SEEK_SET);
    if (sz <= 0 || sz > 1024 * 1024) {
        LOG_WARN("waypoint_follower", "航点文件大小异常: %ld", sz);
        fclose(f);
        return -1;
    }
    char* buf = (char*)malloc((size_t)sz + 1);
    if (!buf) { fclose(f); return -1; }
    size_t n = fread(buf, 1, (size_t)sz, f);
    buf[n] = '\0';
    fclose(f);

    cJSON* root = cJSON_Parse(buf);
    free(buf);
    if (!root) {
        LOG_WARN("waypoint_follower", "航点文件 JSON 解析失败: %s", path);
        return -1;
    }

    /* 扫描顶层参数 */
    {
        cJSON* j;
        j = cJSON_GetObjectItemCaseSensitive(root, "lookahead_m");
        if (cJSON_IsNumber(j) && j->valuedouble > 0.1) g.lookahead_m = j->valuedouble;
        j = cJSON_GetObjectItemCaseSensitive(root, "cruise_speed");
        if (cJSON_IsNumber(j) && j->valuedouble > 0.1) g.cruise_speed = j->valuedouble;
    }

    /* 扫描 waypoints 数组 */
    g.wp_count = 0;
    cJSON* wps = cJSON_GetObjectItemCaseSensitive(root, "waypoints");
    if (!cJSON_IsArray(wps)) {
        LOG_WARN("waypoint_follower", "航点文件缺 waypoints 字段");
        cJSON_Delete(root);
        return -1;
    }

    int nw = cJSON_GetArraySize(wps);
    if (nw > WF_MAX_WAYPOINTS) nw = WF_MAX_WAYPOINTS;
    for (int i = 0; i < nw; i++) {
        cJSON* wp = cJSON_GetArrayItem(wps, i);
        if (!cJSON_IsObject(wp)) continue;
        cJSON* jx = cJSON_GetObjectItemCaseSensitive(wp, "x");
        cJSON* jy = cJSON_GetObjectItemCaseSensitive(wp, "y");
        if (cJSON_IsNumber(jx) && cJSON_IsNumber(jy)) {
            g.wpx[g.wp_count] = jx->valuedouble;
            g.wpy[g.wp_count] = jy->valuedouble;
            g.wp_count++;
        }
    }

    cJSON_Delete(root);

    if (g.wp_count < 2) {
        LOG_WARN("waypoint_follower", "航点文件有效航点不足 2 个 (解析到 %d)", g.wp_count);
        return -1;
    }

    LOG_INFO("waypoint_follower", "加载 %d 个航点 from %s (lookahead=%.2fm cruise=%.1fm/s)",
             g.wp_count, path, g.lookahead_m, g.cruise_speed);
    return 0;
}

/* ── fusion/localization 订阅回调 ─────────────────────────── */
static void on_fusion(const Message* msg, void* user_data) {
    (void)user_data;
    if (!msg || !msg->data) return;

    /* 优先尝试二进制反序列化 */
    Localization loc;
    if (msg->data_size >= sizeof(Localization) &&
        Localization_deserialize(&loc, (const uint8_t*)msg->data, msg->data_size) == 0) {
        pthread_mutex_lock(&g.lock);
        g.ego_x = loc.x;
        g.ego_y = loc.y;
        g.ego_v = loc.v;
        g.ego_heading = loc.heading;
        g.has_fusion = 1;
        pthread_mutex_unlock(&g.lock);
        return;
    }

    /* 回退到 JSON 文本解析 */
    cJSON* root = cJSON_Parse((const char*)msg->data);
    double x = 0, y = 0, v = 0, h = 0;
    if (root) {
        cJSON* j;
        j = cJSON_GetObjectItemCaseSensitive(root, "x");
        if (cJSON_IsNumber(j)) x = j->valuedouble;
        j = cJSON_GetObjectItemCaseSensitive(root, "y");
        if (cJSON_IsNumber(j)) y = j->valuedouble;
        j = cJSON_GetObjectItemCaseSensitive(root, "v");
        if (cJSON_IsNumber(j)) v = j->valuedouble;
        j = cJSON_GetObjectItemCaseSensitive(root, "heading");
        if (cJSON_IsNumber(j)) h = j->valuedouble;
        cJSON_Delete(root);
    }

    pthread_mutex_lock(&g.lock);
    g.ego_x = x; g.ego_y = y; g.ego_v = v; g.ego_heading = h;
    g.has_fusion = 1;
    pthread_mutex_unlock(&g.lock);
}

/* ── perception/obstacles 订阅回调 ──────────────────────────
 * 取前方最近障碍物（自车坐标系 x>0, |y|<1m）。
 */
static void on_obstacles(const Message* msg, void* user_data) {
    (void)user_data;
    if (!msg || !msg->data) return;

    ObstacleList list;
    if (ObstacleList_deserialize(&list, (const uint8_t*)msg->data, msg->data_size) != 0) {
        return;
    }

    pthread_mutex_lock(&g.lock);
    g.has_obstacle = 0;
    g.nearest_obs_x = 1e9;
    g.nearest_obs_y = 0;
    for (uint32_t i = 0; i < list.count; i++) {
        const Obstacle* o = &list.obstacles[i];
        /* 障碍物坐标是车体坐标系: x=前方距离, y=横向距离 */
        if (o->x > 0 && o->x < g.nearest_obs_x && fabs(o->y) < 1.5) {
            g.nearest_obs_x = o->x;
            g.nearest_obs_y = o->y;
            g.has_obstacle = 1;
        }
    }
    g.last_obstacle_time = time(NULL);
    pthread_mutex_unlock(&g.lock);
}

/* ── perception/traversability 订阅回调 (Phase 1) ──────────
 * 解析 traversability JSON,取 nearest_obstacle_x / blocked / corridor_width_m。
 * 用于补充 LiDAR 盲区(近距低矮障碍),以及窄路/堵死场景的减速决策。
 */
static void on_traversability(const Message* msg, void* user_data) {
    (void)user_data;
    if (!msg || !msg->data) return;

    cJSON* root = cJSON_Parse((const char*)msg->data);
    double near_obs = -1.0, cor_w = 0.0;
    int blocked = 0;
    if (root) {
        cJSON* j;
        j = cJSON_GetObjectItemCaseSensitive(root, "nearest_obstacle_x");
        if (cJSON_IsNumber(j)) near_obs = j->valuedouble;
        j = cJSON_GetObjectItemCaseSensitive(root, "corridor_width_m");
        if (cJSON_IsNumber(j)) cor_w = j->valuedouble;
        j = cJSON_GetObjectItemCaseSensitive(root, "blocked");
        if (cJSON_IsBool(j)) blocked = cJSON_IsTrue(j) ? 1 : 0;
        cJSON_Delete(root);
    }

    pthread_mutex_lock(&g.lock);
    g.traversability_nearest_obs = near_obs;
    g.traversability_corridor_w  = cor_w;
    g.traversability_blocked     = blocked;
    g.has_traversability         = 1;
    g.last_traversability_time   = time(NULL);
    pthread_mutex_unlock(&g.lock);
}

/* ── Pure Pursuit 核心 ──────────────────────────────────────
 * 找前瞻点 → 计算曲率 → 输出轨迹点列表。
 *
 * @param ego_x, ego_y, ego_heading  自车位姿（局部笛卡尔，弧度）
 * @param path_x[], path_y[]         输出: 前瞻路径点（车体坐标系，供 control 解析）
 * @param path_n                     输出路径点数（最多 WF_PATH_SAMPLES）
 * @param target_speed_out           输出: 目标速度（已考虑障碍物减速）
 * @return 0 成功, -1 无航点/无定位
 */
static int pure_pursuit_step(double ego_x, double ego_y, double ego_heading,
                              double path_x[], double path_y[], int* path_n,
                              double* target_speed_out) {
    if (g.wp_count < 2) return -1;

    /* 1. 找最近的航点索引（自车当前位置在航点列表上的投影） */
    double min_d2 = 1e18;
    int nearest_idx = g.wp_current;
    for (int i = 0; i < g.wp_count; i++) {
        double dx = g.wpx[i] - ego_x;
        double dy = g.wpy[i] - ego_y;
        double d2 = dx * dx + dy * dy;
        if (d2 < min_d2) {
            min_d2 = d2;
            nearest_idx = i;
        }
    }
    /* 推进 wp_current：只允许向前，避免回跳（除非 loop 模式绕回） */
    if (nearest_idx >= g.wp_current) {
        if (g.wp_current != nearest_idx) {
            g.wp_advanced += (uint64_t)(nearest_idx - g.wp_current);
            g.wp_current = nearest_idx;
        }
    } else if (g.loop && g.wp_current >= g.wp_count - 1 && nearest_idx < 3) {
        /* loop 模式绕回起点 */
        g.wp_current = 0;
        g.lap_count++;
        LOG_INFO("waypoint_follower", "完成第 %d 圈，绕回起点", g.lap_count);
    }

    /* 2. 找前瞻点：从当前航点向前找距离 ≥ lookahead 的航点 */
    double target_x = g.wpx[g.wp_current];
    double target_y = g.wpy[g.wp_current];
    double accum = 0.0;
    int lookahead_idx = g.wp_current;
    for (int k = g.wp_current; k < g.wp_count; k++) {
        double dx = g.wpx[k] - ego_x;
        double dy = g.wpy[k] - ego_y;
        double d = sqrt(dx * dx + dy * dy);
        if (d >= g.lookahead_m) {
            target_x = g.wpx[k];
            target_y = g.wpy[k];
            lookahead_idx = k;
            break;
        }
        accum = d;
    }
    /* 到达终点（非 loop） */
    if (lookahead_idx == g.wp_current && !g.loop) {
        double dx = g.wpx[g.wp_count - 1] - ego_x;
        double dy = g.wpy[g.wp_count - 1] - ego_y;
        if (sqrt(dx * dx + dy * dy) < 0.5) {
            *target_speed_out = 0.0;  /* 到达终点，停车 */
            *path_n = 0;
            return 0;
        }
    }

    /* 3. 把前瞻路径点（世界坐标）转换到车体坐标系输出，供 control 直接消费 */
    double cos_h = cos(-ego_heading);
    double sin_h = sin(-ego_heading);
    int n = 0;
    int step = (g.wp_count - g.wp_current) / WF_PATH_SAMPLES + 1;
    if (step < 1) step = 1;
    for (int k = g.wp_current; k < g.wp_count && n < WF_PATH_SAMPLES; k += step) {
        double dx = g.wpx[k] - ego_x;
        double dy = g.wpy[k] - ego_y;
        /* 旋转到车体坐标系: x'=forward, y'=left */
        path_x[n] = dx * cos_h - dy * sin_h;
        path_y[n] = -dx * sin_h - dy * cos_h;
        n++;
    }
    *path_n = n;

    /* 4. 目标速度：基础巡航速度，遇障碍物减速。融合 LiDAR(obstacles) + 双目(traversability)。 */
    double speed = g.cruise_speed;

    /* 4a. 双目可通行区域检查 (Phase 1: traversability 接线)
     *     数据过期(>3s 未更新)视为无效,降级到纯 LiDAR 模式。 */
    int trav_fresh = g.has_traversability &&
                     (time(NULL) - g.last_traversability_time) < 3;
    if (trav_fresh) {
        /* 前方堵死 → 立即停车 */
        if (g.traversability_blocked) {
            speed = 0.0;
        }
        /* 双目检测到前方障碍物 → 按距离减速 */
        else if (g.traversability_nearest_obs > 0.0 &&
                 g.traversability_nearest_obs < g.obstacle_slow_dist) {
            double obs = g.traversability_nearest_obs;
            if (obs < g.obstacle_stop_dist) {
                speed = 0.0;
            } else {
                double r = (obs - g.obstacle_stop_dist) /
                           (g.obstacle_slow_dist - g.obstacle_stop_dist);
                double s = g.cruise_speed * r;
                if (s < speed) speed = s;
            }
        }
        /* 走廊太窄(<0.6m) → 降速到巡航的 40% */
        if (g.traversability_corridor_w > 0.0 &&
            g.traversability_corridor_w < 0.6) {
            double narrow_speed = g.cruise_speed * 0.4;
            if (narrow_speed < speed) speed = narrow_speed;
        }
    }

    /* 4b. LiDAR 障碍物检查（与双目取更保守者） */
    if (g.has_obstacle && g.nearest_obs_x < g.obstacle_slow_dist) {
        if (g.nearest_obs_x < g.obstacle_stop_dist) {
            speed = 0.0;  /* 停车 */
        } else {
            /* 线性减速: slow_dist→cruise, stop_dist→0 */
            double r = (g.nearest_obs_x - g.obstacle_stop_dist) /
                       (g.obstacle_slow_dist - g.obstacle_stop_dist);
            double s = g.cruise_speed * r;
            if (s < speed) speed = s;  /* 取更保守的速度 */
        }
    }

    /* 接近终点减速（非 loop 模式） */
    if (!g.loop) {
        double dx_end = g.wpx[g.wp_count - 1] - ego_x;
        double dy_end = g.wpy[g.wp_count - 1] - ego_y;
        double d_end = sqrt(dx_end * dx_end + dy_end * dy_end);
        if (d_end < 3.0) {
            double end_speed = g.cruise_speed * (d_end / 3.0);
            if (end_speed < speed) speed = end_speed;
        }
    }

    *target_speed_out = speed;
    return 0;
}

/* ── 工作线程 ─────────────────────────────────────────────── */
static void* wp_thread(void* arg) {
    (void)arg;
    pthread_setname_np(pthread_self(), "wp_follow");
    long period_ms = 1000L / (g.plan_hz > 0 ? g.plan_hz : 10);

    while (g.thread_running && !g.should_stop) {
        long t0 = (long)(clock_now_us() / 1000);

        if (!g.has_fusion) {
            usleep((unsigned long)period_ms * 1000UL);
            continue;
        }

        /* 拷贝自车状态（避免长时间持锁） */
        double ego_x, ego_y, ego_v, ego_heading;
        pthread_mutex_lock(&g.lock);
        ego_x = g.ego_x; ego_y = g.ego_y;
        ego_v = g.ego_v; ego_heading = g.ego_heading;
        pthread_mutex_unlock(&g.lock);

        /* Pure Pursuit 计算 */
        double path_x[WF_PATH_SAMPLES], path_y[WF_PATH_SAMPLES];
        int path_n = 0;
        double target_speed = 0.0;
        if (pure_pursuit_step(ego_x, ego_y, ego_heading,
                               path_x, path_y, &path_n, &target_speed) != 0) {
            usleep((unsigned long)period_ms * 1000UL);
            continue;
        }

        /* 序列化 trajectory JSON（与 planning_node 兼容格式） */
        cJSON* tj_root = cJSON_CreateObject();
        cJSON_AddStringToObject(tj_root, "type", "pure_pursuit");
        cJSON_AddNumberToObject(tj_root, "plan", (double)g.plans_published);
        cJSON_AddNumberToObject(tj_root, "wp_idx", g.wp_current);
        cJSON_AddNumberToObject(tj_root, "lap", g.lap_count);
        cJSON_AddNumberToObject(tj_root, "target_speed", target_speed);
        cJSON* path_arr = cJSON_CreateArray();
        for (int i = 0; i < path_n; i++) {
            cJSON* pt = cJSON_CreateArray();
            cJSON_AddItemToArray(pt, cJSON_CreateNumber(path_x[i]));
            cJSON_AddItemToArray(pt, cJSON_CreateNumber(path_y[i]));
            cJSON_AddItemToArray(pt, cJSON_CreateNumber(target_speed));
            cJSON_AddItemToArray(path_arr, pt);
        }
        cJSON_AddItemToObject(tj_root, "path", path_arr);

        char* traj = cJSON_PrintUnformatted(tj_root);
        /* 追加 speed= 字段（control_node 用 strstr 解析这个） */
        char traj_final[WF_TRAJ_JSON_LEN + 64];
        snprintf(traj_final, sizeof(traj_final), "%s speed=%.2f mode=WAYPOINT_FOLLOW",
                 traj, target_speed);
        free(traj);
        cJSON_Delete(tj_root);

        transport_publish(g.transport, "planning/trajectory",
                          (const uint8_t*)traj_final, (uint32_t)(strlen(traj_final) + 1));
        g.plans_published++;

        /* 周期性日志 */
        if (g.plans_published % 50 == 1) {
            LOG_INFO("waypoint_follower", "#%lu wp=%d/%d tgt_spd=%.2f "
                     "obs=%s(%.1fm) st_blk=%d st_obs=%.1fm st_cor=%.2fm lap=%d",
                     (unsigned long)g.plans_published, g.wp_current, g.wp_count,
                     target_speed,
                     g.has_obstacle ? "Y" : "N",
                     g.has_obstacle ? g.nearest_obs_x : 0.0,
                     g.has_traversability ? g.traversability_blocked : -1,
                     g.has_traversability ? g.traversability_nearest_obs : -1.0,
                     g.has_traversability ? g.traversability_corridor_w : -1.0,
                     g.lap_count);
        }

        long elapsed = (long)(clock_now_us() / 1000) - t0;
        long remain = period_ms - elapsed;
        if (remain > 0) usleep((unsigned long)remain * 1000UL);
    }
    return NULL;
}

/* ── NodePlugin 实现 ─────────────────────────────────────── */

static const char* s_inputs[]  = { "fusion/localization", "perception/obstacles",
                                    "perception/traversability", NULL };
static const char* s_outputs[] = { "planning/trajectory", NULL };

static NodePlugin s_plugin;

static int wp_init(MessageBus* bus, Transport* transport,
                   DiscoveryManager* discovery, Scheduler* scheduler,
                   const char* params_json) {
    (void)bus;
    memset(&g, 0, sizeof(g));
    g.transport = transport;
    g.discovery = discovery;
    g.scheduler = scheduler;

    pthread_mutex_init(&g.lock, NULL);

    /* 默认参数 */
    snprintf(g.waypoints_file, sizeof(g.waypoints_file), "/tmp/waypoints.json");
    g.loop = 1;
    g.cruise_speed = 2.0;
    g.lookahead_m = 1.5;
    g.plan_hz = 10;
    g.obstacle_slow_dist = 3.0;
    g.obstacle_stop_dist = 0.8;

    if (params_json) {
        cJSON* p = cJSON_Parse(params_json);
        if (p) {
            cJSON* j;
            j = cJSON_GetObjectItemCaseSensitive(p, "waypoints_file");
            if (cJSON_IsString(j) && j->valuestring)
                strncpy(g.waypoints_file, j->valuestring, sizeof(g.waypoints_file) - 1);
            j = cJSON_GetObjectItemCaseSensitive(p, "loop");
            if (cJSON_IsBool(j)) g.loop = cJSON_IsTrue(j) ? 1 : 0;
            else if (cJSON_IsNumber(j)) g.loop = j->valueint;
            j = cJSON_GetObjectItemCaseSensitive(p, "cruise_speed");
            if (cJSON_IsNumber(j)) g.cruise_speed = j->valuedouble;
            j = cJSON_GetObjectItemCaseSensitive(p, "lookahead_m");
            if (cJSON_IsNumber(j)) g.lookahead_m = j->valuedouble;
            j = cJSON_GetObjectItemCaseSensitive(p, "plan_hz");
            if (cJSON_IsNumber(j)) g.plan_hz = j->valueint;
            j = cJSON_GetObjectItemCaseSensitive(p, "obstacle_slow_dist");
            if (cJSON_IsNumber(j)) g.obstacle_slow_dist = j->valuedouble;
            j = cJSON_GetObjectItemCaseSensitive(p, "obstacle_stop_dist");
            if (cJSON_IsNumber(j)) g.obstacle_stop_dist = j->valuedouble;
            cJSON_Delete(p);
        }
    }

    /* 加载航点文件 */
    if (load_waypoints(g.waypoints_file) != 0) {
        LOG_WARN("waypoint_follower", "航点加载失败，节点启动后不会发布 trajectory。"
                 "用 tools/waypoint_record.py 录制航点后写入 %s", g.waypoints_file);
    }

    /* 订阅输入 */
    transport_subscribe(transport, "fusion/localization", on_fusion, NULL);
    transport_subscribe(transport, "perception/obstacles", on_obstacles, NULL);
    transport_subscribe(transport, "perception/traversability", on_traversability, NULL);
    discovery_advertise(discovery, "fusion/localization", 0xF0ED10C0u, CAP_SUBSCRIBER, 0);
    discovery_advertise(discovery, "perception/obstacles", OBSTACLELIST_TYPE_ID, CAP_SUBSCRIBER, 0);
    discovery_advertise(discovery, "perception/traversability", 0, CAP_SUBSCRIBER, 0);

    /* 发布 trajectory */
    discovery_advertise(discovery, "planning/trajectory", 0x0, CAP_PUBLISHER, (double)g.plan_hz);
    transport_advertise(transport, "planning/trajectory", 0x0);

    LOG_INFO("waypoint_follower", "initialized: file=%s wps=%d loop=%d "
             "cruise=%.1fm/s lookahead=%.2fm obs_slow=%.1fm obs_stop=%.1fm hz=%d",
             g.waypoints_file, g.wp_count, g.loop, g.cruise_speed, g.lookahead_m,
             g.obstacle_slow_dist, g.obstacle_stop_dist, g.plan_hz);
    return 0;
}

static int wp_start(void) {
    g.thread_running = 1;
    g.should_stop = 0;
    if (pthread_create(&g.thread, NULL, wp_thread, NULL) != 0) {
        LOG_WARN("waypoint_follower", "thread create failed");
        g.thread_running = 0;
        return -1;
    }
    LOG_INFO("waypoint_follower", "started (Pure Pursuit, %dHz)", g.plan_hz);
    node_announce_self(g.transport, &s_plugin);
    return 0;
}

static void wp_stop(void) {
    g.should_stop = 1;
    g.thread_running = 0;
}

static void wp_cleanup(void) {
    g.should_stop = 1;
    g.thread_running = 0;
    if (g.thread) { pthread_join(g.thread, NULL); g.thread = 0; }
    pthread_mutex_destroy(&g.lock);
    LOG_INFO("waypoint_follower", "cleanup: plans=%lu wp_adv=%lu laps=%d",
             (unsigned long)g.plans_published,
             (unsigned long)g.wp_advanced, g.lap_count);
}

static int wp_health(void) {
    if (g.wp_count < 2) return -1;
    return 0;
}

static NodePlugin s_plugin = {
    .api_version   = NODE_PLUGIN_API_VERSION,
    .name          = "waypoint_follower",
    .version       = "1.0.0",
    .description   = "Pure Pursuit waypoint follower (L2 for RC car, replaces planning_node)",
    .input_topics  = s_inputs,
    .output_topics = s_outputs,
    .init          = wp_init,
    .start         = wp_start,
    .stop          = wp_stop,
    .cleanup       = wp_cleanup,
    .health        = wp_health,
};

NodePlugin* node_get_plugin(void) { return &s_plugin; }
