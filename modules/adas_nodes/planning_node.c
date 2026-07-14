/**
 * planning_node.c — Frenet 轨迹规划节点插件
 *
 * 订阅 fusion/localization → 发布 planning/trajectory
 * 使用 Frenet 最优轨迹规划器生成平滑行驶轨迹。
 *
 * NodePlugin 接口，编译为 libplanning_node.so。
 */

#include "node_plugin.h"
#include "state_machine.h"
#include "scenario_loader.h"
#include "road_geometry.h"
#include "adas_msgs_gen.h"
#include "logger.h"

#ifdef HAVE_FRENET
#include "frenet_bridge.h"
#endif

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <pthread.h>
#include <time.h>
#include <unistd.h>

/* ── 节点本地状态 ───────────────────────────────────────────── */

static struct {
    Transport*        transport;
    DiscoveryManager* discovery;
    Scheduler*        scheduler;

    pthread_t   thread;
    volatile int running;
    volatile int should_stop;

    /* 反射式状态机：跟踪生命周期 */
    ReflectiveStateMachine sm;

    /* 驾驶模式状态机（NA/ACC/CP/NP/NOA）：系统级功能仲裁, 由感知/定位/
     * 路况条件驱动升降级, 是 NOA 相对 LCC 的核心能力入口。 */
    ReflectiveStateMachine mode_sm;
    uint64_t mode_last_check_us;
    uint64_t last_fusion_us;      /* fusion 消息到达的单调时间戳（模式降级用） */
    int      highway_ready;       /* ego_v 持续高于阈值一段时间 -> 视为高速工况 */
    double   highway_speed_timer;

    /* 从场景文件加载的导航路线（可选, 仅当 params 提供 scenario_file 时有效） */
    char              scenario_file[256];
    ScenarioRouteStep route[SCENARIO_MAX_ROUTE_STEPS];
    int               route_count;
    int               route_next_idx;   /* 下一条待触发的路线指令 */
    int               route_target_lane; /* 当前路线要求的目标车道: 0=无, -1=y<0一侧, +1=y>0一侧 */

    /* Frenet 规划器 */
#ifdef HAVE_FRENET
    FrenetHandle* frenet;
#else
    void*         frenet;  /* unused stub */
#endif
    int           plan_count;
    double        target_speed;

    /* 从 fusion/localization 解析的最新状态 */
    double ego_x, ego_y, ego_v, ego_heading;
    volatile int has_fusion;

    /* 从 vehicle/state 解析的障碍物位置（世界坐标） */
    double obs_x[4], obs_y[4], obs_vx[4];
    volatile int has_vstate;

    /* 配置参数 */
    double cfg_target_speed;
    double cfg_max_speed;
    double cfg_max_accel;
    double cfg_ref_path_length;
    double ref_path_start_x;
    double cfg_highway_speed_mps; /* CP->NP 升级所需的持续速度阈值 (m/s) */

    /* 道路几何（可选弯道，来自场景文件 "road"；全零 = 直道，行为不变） */
    double curve_start_x;
    double curve_length_m;
    double curve_offset_m;

    int tid;  /* scheduler task id */
} g;

/* 障碍物过滤与传递给 Frenet 规划器的空间范围 */
#define OBS_MIN_DX_M      -10.0   /* 忽略已经落后 ego 超过此距离的障碍物 */
#define OBS_MAX_DX_M      120.0   /* 忽略前方超过此距离的障碍物 */
#define OBS_MAX_ABS_Y_M     6.0   /* 忽略横向距离超出道路范围的障碍物 */
#define OBSTACLE_WIDTH_M    2.0   /* 默认障碍物宽度 (m) */
#define OBSTACLE_LENGTH_M   4.6   /* 默认障碍物长度 (m) */

/* 驾驶模式仲裁常量 */
#define MODE_CHECK_INTERVAL_US   1000000ULL  /* 每 1s 检查一次模式升降级条件 */
#define FUSION_STALE_TIMEOUT_US  1500000ULL  /* 定位超过此时长未更新 -> 判定条件丢失 */
#define HIGHWAY_SPEED_HOLD_S     3.0          /* 速度需持续高于阈值这么久才算"高速工况" */

static uint64_t monotonic_us(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;
}

/**
 * 驾驶模式转移守卫：把真实感知/定位/路况条件接到状态机上，
 * 而不是无条件放行——这是把"演示用状态机"变成"真正的模式仲裁器"的关键。
 *
 *   NA  -> ACC : 定位与车辆状态均已上线
 *   ACC -> CP  : 定位持续有效（车道居中所需的姿态/位置可信）
 *   CP  -> NP  : 达到并保持高速工况（导航加速辅助的 ODD 之一）
 *   NP  -> NOA : 已加载导航路线（等效 HD 地图/路线规划可用）
 * 其余转移（降级/故障/退出）默认放行。
 */
static bool mode_transition_guard(void* task, StateId from, EventId event, StateId to) {
    (void)task; (void)from;
    StateId to_mode = SM_MODE_OF(to);
    if (event == SM_EVT_CONDITIONS_MET) {
        return g.has_fusion && g.has_vstate;
    }
    if (event == SM_EVT_MODE_UPGRADE) {
        switch (to_mode) {
            case SM_MODE_CP:  return g.has_fusion != 0;
            case SM_MODE_NP:  return g.highway_ready != 0;
            case SM_MODE_NOA: return g.route_count > 0;
            default: return true;
        }
    }
    return true;
}

/** 每个控制周期尝试驱动模式向上一级演进（受 guard 约束，条件不满足时静默保持）。 */
static void try_mode_progress(void) {
    StateId cur  = statem_current(&g.mode_sm);
    StateId mode = SM_MODE_OF(cur);
    EventId ev   = (mode == SM_MODE_NA) ? SM_EVT_CONDITIONS_MET : SM_EVT_MODE_UPGRADE;

    if (statem_send_event(&g.mode_sm, ev, NULL)) {
        char buf[32];
        statem_format_hierarchical(statem_current(&g.mode_sm), buf, sizeof(buf));
        LOG_INFO("planning", "driving mode -> %s", buf);
    }
}

/** 定位/车辆状态长时间未更新 -> 条件丢失，模式退回 NA（安全兜底）。 */
static void check_mode_downgrade(uint64_t now_us) {
    if (SM_MODE_OF(statem_current(&g.mode_sm)) == SM_MODE_NA) return;
    if (g.last_fusion_us != 0 && now_us - g.last_fusion_us > FUSION_STALE_TIMEOUT_US) {
        if (statem_send_event(&g.mode_sm, SM_EVT_CONDITIONS_LOST, NULL)) {
            LOG_WARN("planning", "fusion stale (%.1fs) -> driving mode downgraded to NA",
                     (double)(now_us - g.last_fusion_us) / 1e6);
        }
    }
}

static void update_reference_path(double start_x) {
#ifdef HAVE_FRENET
    double wx[101], wy[101];
    const int ref_n = 101;
    for (int i = 0; i < ref_n; i++) {
        wx[i] = start_x + (double)i * (g.cfg_ref_path_length / (double)(ref_n - 1));
        /* 弯道时参考路径跟随道路中心线；curve_length_m<=0 时 road_center_y()
         * 恒为 0，与既有直线参考路径完全一致。 */
        wy[i] = -1.75 + road_center_y(wx[i], g.curve_start_x, g.curve_length_m, g.curve_offset_m);
    }
    frenet_set_reference_path(g.frenet, wx, wy, ref_n);
    g.ref_path_start_x = start_x;
#else
    (void)start_x;
#endif
}

static void on_fusion(const Message* msg, void* user_data) {
    (void)user_data;
    if (!msg || !msg->data) return;

    /* Try binary deserialization (serializer path) */
    {
        Localization loc;
        if (Localization_deserialize(&loc, (const uint8_t*)msg->data, msg->data_size) == 0) {
            g.ego_x       = loc.x;
            g.ego_y       = loc.y;
            g.ego_v       = loc.v;
            g.ego_heading = loc.heading;
            g.has_fusion  = 1;
            g.last_fusion_us = monotonic_us();
            return;
        }
    }

    /* Fallback: text JSON parsing */
    const char* d = (const char*)msg->data;
    const char* p;
    if ((p = strstr(d, "\"x\":")))       sscanf(p + 4, "%lf", &g.ego_x);
    if ((p = strstr(d, "\"y\":")))       sscanf(p + 4, "%lf", &g.ego_y);
    if ((p = strstr(d, "\"v\":")))       sscanf(p + 4, "%lf", &g.ego_v);
    if ((p = strstr(d, "\"heading\":"))) sscanf(p + 10, "%lf", &g.ego_heading);
    g.has_fusion = 1;
    g.last_fusion_us = monotonic_us();
}

/* ── vehicle/state 订阅 — 解析障碍物位置（世界坐标） ─────────── */

static void on_vehicle_state(const Message* msg, void* user_data) {
    (void)user_data;
    if (!msg || !msg->data) return;
    const char* d = (const char*)msg->data;
    for (int i = 0; i < 4; i++) {
        char key[16];
        int klen;
        klen = snprintf(key, sizeof(key), "\"ox%d\":", i);
        const char* p = strstr(d, key);
        if (p) sscanf(p + klen, "%lf", &g.obs_x[i]);
        klen = snprintf(key, sizeof(key), "\"oy%d\":", i);
        if ((p = strstr(d, key))) sscanf(p + klen, "%lf", &g.obs_y[i]);
        klen = snprintf(key, sizeof(key), "\"ov%d\":", i);
        if ((p = strstr(d, key))) sscanf(p + klen, "%lf", &g.obs_vx[i]);
    }
    g.has_vstate = 1;
}

/* ── 任务线程 ────────────────────────────────────────────────── */

static void* planning_thread(void* arg) {
    (void)arg;
    pthread_setname_np(pthread_self(), "planning");

    /* 参考路径: 长直线（左车道中心 y=-1.75）。运行时间较长时，ego 会开出
     * 初始路径范围；接近末端时向前滑动 reference path，避免 Frenet 插值越界
     * 导致 planning 线程挂掉。 */
    update_reference_path(0.0);

    while (!g.should_stop) {
        usleep(50000);  /* 20Hz 检查 */
        if (g.should_stop || !g.has_fusion) continue;

        /* ── 驾驶模式仲裁：周期性检查条件，尝试升级；定位丢失时立即降级 ── */
        uint64_t now_us = monotonic_us();
        g.highway_ready = (g.ego_v >= g.cfg_highway_speed_mps) &&
                           (g.highway_speed_timer >= HIGHWAY_SPEED_HOLD_S);
        if (g.ego_v >= g.cfg_highway_speed_mps) g.highway_speed_timer += 0.05;
        else                                    g.highway_speed_timer = 0.0;

        check_mode_downgrade(now_us);
        if (now_us - g.mode_last_check_us >= MODE_CHECK_INTERVAL_US) {
            try_mode_progress();
            g.mode_last_check_us = now_us;
        }

        /* ── 路线驱动的主动变道（仅 NOA 生效）：不依赖障碍物，由导航路线
         * 提前决定车道，是 NOA 区别于被动跟车/超车（LCC/NP）的核心行为。
         * route_target_lane 一经触发即持续下发，直到下一条路线指令覆盖它，
         * 供 control 节点在其既有安全变道状态机上执行（rear/front gap 检查）。 ── */
        if (SM_MODE_OF(statem_current(&g.mode_sm)) == SM_MODE_NOA &&
            g.route_next_idx < g.route_count &&
            g.ego_x >= g.route[g.route_next_idx].trigger_x) {
            g.route_target_lane = g.route[g.route_next_idx].target_lane;
            const char* label = g.route[g.route_next_idx].label;
            LOG_INFO("planning", "NOA route step #%d triggered @x=%.0f -> lane=%d (%s)",
                     g.route_next_idx, g.ego_x, g.route_target_lane,
                     label && label[0] ? label : "-");
            g.route_next_idx++;
        }

        if (g.ego_x > g.ref_path_start_x + g.cfg_ref_path_length * 0.8) {
            double new_start = g.ego_x - 50.0;
            if (new_start < 0.0) new_start = 0.0;
            update_reference_path(new_start);
            LOG_INFO("planning", "reference path shifted to x=%.0f..%.0f",
                     g.ref_path_start_x, g.ref_path_start_x + g.cfg_ref_path_length);
        }

        double command_speed = g.target_speed;

        /* 向 Frenet 规划器注入障碍物（世界坐标），触发自动避障/变道 */
#ifdef HAVE_FRENET
        if (g.has_vstate) {
            double ox[4], oy[4], ow[4], ol[4];
            int n_obs = 0;
            for (int i = 0; i < 4; i++) {
                /* 只传入前方和侧方的有效障碍物（排除行人 y>4） */
                double dx = g.obs_x[i] - g.ego_x;
                if (dx < OBS_MIN_DX_M || dx > OBS_MAX_DX_M) continue;
                if (fabs(g.obs_y[i]) > OBS_MAX_ABS_Y_M) continue;
                ox[n_obs] = g.obs_x[i];
                oy[n_obs] = g.obs_y[i];
                ow[n_obs] = OBSTACLE_WIDTH_M;
                ol[n_obs] = OBSTACLE_LENGTH_M;
                n_obs++;
            }
            frenet_set_obstacles(g.frenet, ox, oy, ow, ol, n_obs);
        }
#endif

        /* 规划轨迹 */
        double s_out[50], d_out[50], spd_out[50];
        int n_wp = 0;

#ifdef HAVE_FRENET
        {
            double ego_d = g.ego_y + 1.75;  /* 相对 y=-1.75 的偏移 */
            n_wp = frenet_plan(g.frenet,
                g.ego_x, ego_d, g.ego_v,
                command_speed,
                s_out, d_out, spd_out, 50);
        }
#else
        /* Fallback: 生成简单的车道保持 + 恒速轨迹 */
        {
            double horizon = 50.0;   /* 前方 50m */
            int n = 10;              /* 10 个路径点 */
            for (int i = 0; i < n; i++) {
                s_out[i] = g.ego_x + horizon * (double)i / (double)(n - 1);
                d_out[i] = 0.0;     /* 保持参考线（y=-1.75） */
                spd_out[i] = command_speed;
            }
            n_wp = n;
        }
#endif

        char traj[1024];
        int off;
        if (n_wp > 0) {
            command_speed = spd_out[0];
            /* Debuggability: report the ACTUAL planner mode instead of always
             * claiming "frenet". When built without Eigen (HAVE_FRENET undefined),
             * this path is the simple lane-keep fallback (d=0.0 always, no lane
             * changes) — that must be visible in the trajectory JSON / dashboard,
             * not just a one-line startup log that scrolls away. */
#ifdef HAVE_FRENET
            const char* traj_type = "frenet";
#else
            const char* traj_type = "lane_keep_fallback";
#endif
            off = snprintf(traj, sizeof(traj),
                "{\"type\":\"%s\",\"plan\":%d,\"wp\":%d,",
                traj_type, g.plan_count, n_wp);
            off += snprintf(traj + off, sizeof(traj) - (size_t)off,
                "\"target_speed\":%.1f,", command_speed);
            off += snprintf(traj + off, sizeof(traj) - (size_t)off, "\"path\":[");
            for (int i = 0; i < n_wp && off < (int)sizeof(traj) - 50; i++) {
                if (i % 3 != 0 && i > 0 && i < n_wp - 1) continue;
                off += snprintf(traj + off, sizeof(traj) - (size_t)off,
                    "%s[%.1f,%.1f,%.1f]",
                    i > 0 ? "," : "", s_out[i], d_out[i], spd_out[i]);
            }
            off += snprintf(traj + off, sizeof(traj) - (size_t)off, "]}");
        } else {
            double failsafe = command_speed;
            if (failsafe > g.ego_v + 1.0) failsafe = g.ego_v + 1.0;
            if (failsafe > g.cfg_max_speed) failsafe = g.cfg_max_speed;
            off = snprintf(traj, sizeof(traj),
                "{\"type\":\"failsafe\",\"target_speed\":%.1f,\"plan\":%d,"
                "\"lane_keep_d\":%.2f}",
                failsafe, g.plan_count, 0.0);  /* Frenet d=0 → 保持在参考线上 */
        }

        /* 后向兼容: PID 也读取 speed= 字段。附加 mode/route_lane 供 control 消费
         * (NOA 主动变道) 及仪表盘展示驾驶模式，采用与 speed= 相同的宽松追加格式,
         * 不影响既有基于 strstr 的解析。 */
        char mode_buf[32];
        statem_format_hierarchical(statem_current(&g.mode_sm), mode_buf, sizeof(mode_buf));
        char traj_final[1200];
        snprintf(traj_final, sizeof(traj_final), "%s speed=%.1f mode=%s route_lane=%d",
                 traj, command_speed, mode_buf, g.route_target_lane);

        transport_publish(g.transport, "planning/trajectory",
                          (const uint8_t*)traj_final,
                          (uint32_t)strlen(traj_final) + 1);
        g.plan_count++;

        if (g.plan_count % 25 == 1) {
            LOG_INFO("planning", "#%d ego@(%.0f,%.1f) v=%.1f → target=%.1f wp=%d",
                     g.plan_count, g.ego_x, g.ego_y, g.ego_v,
                     command_speed, n_wp);
        }
#ifndef HAVE_FRENET
        /* Repeat this loudly and periodically (not just once at init) so it
         * doesn't get lost in scrollback during a long-running demo — this is
         * exactly the kind of "why won't it overtake" question that should be
         * answerable from logs alone. */
        if (g.plan_count % 200 == 1) {
            LOG_WARN("planning", "#%d running WITHOUT Frenet planner — "
                     "lane-keep-only fallback, ego will NEVER change lanes. "
                     "Install libeigen3-dev and rebuild modules/adas_nodes.",
                     g.plan_count);
        }
#endif
    }

    LOG_INFO("planning", "stopped (%d trajectories, state=%s)",
             g.plan_count, statem_state_name(&g.sm, g.sm.current));
    statem_send_event(&g.sm, SM_EVENT_STOP, NULL);
    statem_send_event(&g.sm, SM_EVENT_DONE, NULL);
    return NULL;
}

/* ── NodePlugin 实现 ─────────────────────────────────────────── */

static const char* s_inputs[]  = { "fusion/localization", "vehicle/state", NULL };
static const char* s_outputs[] = { "planning/trajectory", NULL };

static NodePlugin s_plugin;  /* forward decl */
static int planning_init(MessageBus* bus, Transport* transport,
                         DiscoveryManager* discovery, Scheduler* scheduler,
                         const char* params_json) {
    (void)bus;

    memset(&g, 0, sizeof(g));
    g.transport    = transport;
    g.discovery    = discovery;
    g.scheduler    = scheduler;
    g.should_stop  = 0;
    g.has_fusion   = 0;

    /* 默认参数 */
    g.cfg_target_speed      = 15.0;
    g.cfg_max_speed         = 20.0;
    g.cfg_max_accel         = 4.0;
    g.cfg_ref_path_length   = 5000.0;
    g.cfg_highway_speed_mps = 13.0;  /* 未提供 highway_speed_mps 参数时的兜底默认值，
                                        需低于当前场景实际巡航速度才能触发 NP 升级；
                                        pipeline.json 会显式覆盖为更贴近实际场景的值。 */

    if (params_json) {
        const char* p;
        if ((p = strstr(params_json, "\"target_speed\":")))
            sscanf(p + 15, "%lf", &g.cfg_target_speed);
        if ((p = strstr(params_json, "\"max_speed\":")))
            sscanf(p + 12, "%lf", &g.cfg_max_speed);
        if ((p = strstr(params_json, "\"max_accel\":")))
            sscanf(p + 12, "%lf", &g.cfg_max_accel);
        if ((p = strstr(params_json, "\"ref_path_length_m\":")))
            sscanf(p + 20, "%lf", &g.cfg_ref_path_length);
        if ((p = strstr(params_json, "\"highway_speed_mps\":")))
            sscanf(p + 20, "%lf", &g.cfg_highway_speed_mps);
        if ((p = strstr(params_json, "\"scenario_file\":"))) {
            const char* start = strchr(p + 16, '"');
            if (start) {
                start++;
                const char* end = strchr(start, '"');
                if (end) {
                    size_t len = (size_t)(end - start);
                    if (len >= sizeof(g.scenario_file)) len = sizeof(g.scenario_file) - 1;
                    memcpy(g.scenario_file, start, len);
                    g.scenario_file[len] = '\0';
                }
            }
        }
    }

    g.target_speed = g.cfg_target_speed;

    /* 从场景文件加载导航路线（可选）：NOA 主动变道所需的"路线/地图"数据来源。
     * 找不到文件或未配置 scenario_file 时静默保留 route_count=0，此时模式
     * 状态机最高只能升到 NP（guard 会持续拒绝 NP->NOA 的升级）。 */
    if (g.scenario_file[0] != '\0') {
        ScenarioConfig* sc = scenario_load(g.scenario_file);
        if (sc) {
            g.route_count = sc->route_count;
            /* sc 由 calloc 分配，未用槽位已清零；这里按实际 route_count 精确
             * 拷贝，避免依赖 calloc 的清零语义。 */
            memcpy(g.route, sc->route, sizeof(ScenarioRouteStep) * (size_t)g.route_count);
            /* 道路弯道几何（可选）：缺省全为 0 = 直道，行为与之前完全一致 */
            g.curve_start_x  = sc->road.curve_start_x;
            g.curve_length_m = sc->road.curve_length_m;
            g.curve_offset_m = sc->road.curve_offset_m;
            scenario_free(sc);
            LOG_INFO("planning", "loaded %d NOA route step(s) from '%s'",
                     g.route_count, g.scenario_file);
        } else {
            LOG_WARN("planning", "scenario_file '%s' not loadable — NOA route disabled",
                     g.scenario_file);
        }
    }

    /* Frenet 规划器 */
#ifdef HAVE_FRENET
    g.frenet = frenet_create(g.cfg_max_speed, g.cfg_max_accel);
    if (!g.frenet) {
        LOG_ERROR("planning", "frenet_create failed");
        return -1;
    }
#else
    g.frenet = NULL;
    LOG_INFO("planning", "built without Frenet — using lane-keep fallback");
#endif

    transport_subscribe(transport, "fusion/localization", on_fusion, NULL);
    transport_subscribe(transport, "vehicle/state", on_vehicle_state, NULL);
    transport_advertise(transport, "planning/trajectory", 0x3A7B1C2Du);

    discovery_advertise(discovery, "fusion/localization", 0xF0ED10C0u,
                        CAP_SUBSCRIBER, 0);
    discovery_advertise(discovery, "vehicle/state", 0x1C0E5A7Eu,
                        CAP_SUBSCRIBER, 0);
    discovery_advertise(discovery, "planning/trajectory", 0x3A7B1C2Du,
                        CAP_PUBLISHER, 10.0);

    /* 初始化反射式状态机 */
    statem_init(&g.sm, NULL, SM_STATE_INITIALIZED, "planning");
    statem_send_event(&g.sm, SM_EVENT_START, NULL);

    /* 初始化驾驶模式状态机（NA/ACC/CP/NP/NOA）：真实条件驱动升降级 */
    statem_init(&g.mode_sm, SM_TABLE_MODE_SWITCHING, SM_MODE_NA, "driving_mode");
    statem_set_guard(&g.mode_sm, mode_transition_guard);

    LOG_INFO("planning", "initialized (Frenet, target=%.0f m/s, max=%.0f m/s)",
             g.cfg_target_speed, g.cfg_max_speed);
    return 0;
}

static int planning_start(void) {
    g.running = 1; g.should_stop = 0;
    if (pthread_create(&g.thread, NULL, planning_thread, NULL) != 0) return -1;
    LOG_INFO("planning", "started [state=%s]", statem_state_name(&g.sm, g.sm.current));
    node_announce_self(g.transport, &s_plugin);  /* start() 时广播: monitor 已订阅 */
    return 0;
}

static void planning_stop(void)       { g.should_stop = 1; }
static void planning_cleanup(void) {
    if (g.running) { g.should_stop = 1; pthread_join(g.thread, NULL); g.running = 0; }
#ifdef HAVE_FRENET
    if (g.frenet) { frenet_destroy(g.frenet); g.frenet = NULL; }
#else
    g.frenet = NULL;
#endif
    statem_cleanup(&g.sm);
    statem_cleanup(&g.mode_sm);
    LOG_INFO("planning", "cleanup done");
}
static int  planning_health(void)     { return 0; }

static NodePlugin s_plugin = {
    .api_version   = NODE_PLUGIN_API_VERSION,
    .name          = "planning",
    .version       = "1.0.0",
    .description   = "Frenet Optimal Trajectory Planner",
    .input_topics  = s_inputs,
    .output_topics = s_outputs,
    .init          = planning_init,
    .start         = planning_start,
    .stop          = planning_stop,
    .cleanup       = planning_cleanup,
    .health        = planning_health,
};

NodePlugin* node_get_plugin(void) { return &s_plugin; }
