/**
 * sim_world_node.c — 仿真世界节点插件
 *
 * 模拟车辆动力学 + 障碍物运动学，作为传感器数据源。
 * 可订阅 control/cmd 接收 PID 控制指令，无指令时按默认参数运行。
 * 输出 vehicle/state JSON 供 perception_node 消费。
 *
 * NodePlugin 接口，编译为 libsim_world.so。
 *
 * P0 改进：
 *   1. 确定性时钟：使用 clock_service sim 模式步进逻辑时间（脱离 wall-clock）。
 *      每个 tick 调用 clock_advance_us(DT_US)，并发布 sim/tick 主题。
 *      随机种子通过 params "random_seed" 配置，默认 42（固定）。
 *   2. 场景驱动：从 JSON 文件加载 actors 初始状态（scenario_loader）。
 *      params "scenario_file" 指向场景文件；缺省退回内置 pedestrian_crossing。
 */

#include "node_plugin.h"
#include "clock_service.h"
#include "scenario_loader.h"
#include "road_geometry.h"
#include "state_machine.h"
#include "adas_msgs_gen.h"
#include "logger.h"

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <inttypes.h>
#include <math.h>
#include <time.h>
#include <pthread.h>
#include <unistd.h>

/* ── 仿真常量 ────────────────────────────────────────────────── */

#define SAME_LANE_TOL_M    2.0
#define EGO_LEN_M          4.6
#define EGO_WID_M          2.0    /* 车宽（用于 AABB 碰撞检测） */
#define AEB_GAP_RATIO      0.5    /* AEB 触发阈值：实际间距 < 安全间距 × 此比例 */
#define AEB_MAX_GAP_M      40.0   /* AEB 仅在前车 < 此距离时激活 */
#define PEDESTRIAN_LATERAL_TOL_M 5.0  /* 行人横向检测容差（可能随时横穿车道） */
#define SIM_OBSTACLE_COUNT 16     /* 最多支持 16 个 actor（场景文件上限） */
#define MAX_SPEED          20.0
#define FREQUENCY_HZ       20.0
#define DT_SEC             (1.0 / FREQUENCY_HZ)
#define DT_US              ((uint64_t)(DT_SEC * 1e6))   /* 逻辑时钟步长（微秒） */
#define ROAD_CENTER_LIMIT_M 2.4  /* 路面半宽(3.5) - 车体半宽(1.0) - 安全余量(0.1) */

/* Bytes reserved at the end of the vstate JSON buffer for the closing "}"
 * and any trailing characters.  Each obstacle entry is at most ~100 B;
 * 2 B for the closing brace is well within this margin. */
#define VSTATE_JSON_FOOTER_RESERVE 128

/* ── 仿真障碍物 ────────────────────────────────────────────────── */

typedef struct {
    int    id;
    char   type[16];
    double vx, vy;
    double x, y;
    double len, wid;
    int    ped_crossed_center;
    int    ped_parked;
    double lane_offset;  /**< y 相对道路中心线的横向偏移（弯道时用于跟随中心线） */
    double cross_start_y; /**< 切入车横向运动起点 Y（用于判断变道是否完成） */
    double ped_wait_timer; /**< 行人到达对侧后的等待计时器（秒） */
} SimObstacle;

/* ── 车辆状态 ────────────────────────────────────────────────── */

typedef struct {
    double x, y;
    double speed;
    double heading;
    double steer;
    double throttle, brake;
    double target_speed;
    double lane_target;
    double wheelbase;
    double mass;
    double drag_coeff;
} VehicleSim;

/* ── 节点本地状态 ───────────────────────────────────────────── */

static struct {
    Transport*        transport;
    DiscoveryManager* discovery;

    pthread_t   thread;
    volatile int running;
    volatile int should_stop;

    /* 反射式状态机：跟踪生命周期 */
    ReflectiveStateMachine sm;

    /* 仿真状态 */
    VehicleSim    vehicle;
    SimObstacle   obstacles[SIM_OBSTACLE_COUNT];
    int           obstacle_count;   /* 本次场景中实际的 actor 数量 */
    uint32_t      cycle;
    int           has_control_input;  /* 是否收到过 control/cmd */
    uint64_t      last_control_cmd_us;  /* 最后一次 control/cmd 的 wall-clock 时间戳 (μs) */
    int           collision_recovery_ticks; /* 碰撞冻结剩余 tick 数，到期后允许重新起步 */

    /* 变道状态机 */
    double lc_start_y;
    double lc_target_y;
    double lc_elapsed;
    double lc_duration;
    int    lc_active;
    double lc_timer;
    int    lc_state;     /* 0=正常 1=左变道中 2=左车道巡航 3=右回正 */
    double lc_wait;

    /* 配置参数 */
    double init_speed;
    double target_speed;
    double lane_width;

    /* 道路几何（可选弯道，来自场景文件 "road"；全零 = 直道，行为不变） */
    double curve_start_x;
    double curve_length_m;
    double curve_offset_m;

    /* P0.1 确定性时钟：固定随机种子 */
    uint32_t random_seed;

    /* P0.2 场景文件路径（空字符串表示使用内置默认值） */
    char scenario_file[256];

    /* 碰撞冷却（防止每帧重复报错） */
    int    collision_cooldown[SIM_OBSTACLE_COUNT];
} g;

/* ── 内置默认场景（等价于 pedestrian_crossing.json）─────────── */

static void init_obstacles_default(void) {
    g.obstacle_count = 4;
    /* 障碍物 0: 在 ego 同车道前方 35m, 慢车 7 m/s — 用于触发变道 */
    g.obstacles[0] = (SimObstacle){ 0, "car",          7.0,  0.0, 35.0, -1.75, 4.6, 2.0 };
    /* 障碍物 1: 在 ego 同车道前方 90m, 也是慢车 — 第二次变道触发 */
    g.obstacles[1] = (SimObstacle){ 1, "car",          7.0,  0.0, 90.0, -1.75, 4.6, 2.0 };
    /* 障碍物 2: 行人, 在更远处路边往复行走 */
    g.obstacles[2] = (SimObstacle){ 2, "pedestrian",   0.0,  0.6, 140.0,  8.0,  0.6, 0.6 };
    /* 障碍物 3: 在邻道(右车道 y=1.75), 120m 前, 稍快 */
    g.obstacles[3] = (SimObstacle){ 3, "car",         12.0,  0.0, 130.0, 1.75, 4.6, 2.0 };
}

/* ── 从场景文件初始化障碍物 ───────────────────────────────────── */

static void init_obstacles_from_scenario(const ScenarioConfig* sc) {
    int n = sc->actor_count;
    if (n > SIM_OBSTACLE_COUNT) n = SIM_OBSTACLE_COUNT;
    g.obstacle_count = n;
    for (int i = 0; i < n; i++) {
        const ScenarioActor* a = &sc->actors[i];
        g.obstacles[i].id  = a->id;
        strncpy(g.obstacles[i].type, a->type, sizeof(g.obstacles[i].type) - 1);
        g.obstacles[i].vx  = a->vx;
        g.obstacles[i].vy  = a->vy;
        g.obstacles[i].x   = a->x;
        g.obstacles[i].y   = a->y;
        g.obstacles[i].len = a->len;
        g.obstacles[i].wid = a->wid;
        g.obstacles[i].ped_crossed_center = 0;
        g.obstacles[i].ped_parked = 0;
        g.obstacles[i].ped_wait_timer = 0.0;
        /* cross_start_y: 横向运动起点世界 Y（切入车变道完成判定用）。
         * 沿车道行驶车(vy==0)的 y 在下方 if 分支被弯道中心线修正，此处先记
         * a->y；横穿/切入车(vy!=0)的 y 不修正，a->y 即世界 Y，记录正确。 */
        g.obstacles[i].cross_start_y = a->y;
        /* 场景文件的 y 是"相对道路中心线的车道偏移"（-1.75=左车道, +1.75=右车道），
         * 与 ego.y 语义一致。沿车道行驶的车辆(vy==0)需加上当前 x 处的道路中心线
         * 偏移得到绝对世界坐标；行人(vy!=0)的 y 是绝对坐标（横穿位置），不偏移。
         * 无弯道时 road_center_y 恒为 0，lane_offset 即等于 a->y，完全向后兼容。 */
        g.obstacles[i].lane_offset = a->y;
        if (a->vy == 0.0) {
            double road_c = road_center_y(a->x, g.curve_start_x,
                                          g.curve_length_m, g.curve_offset_m);
            g.obstacles[i].y = road_c + a->y;
        }
    }
}

/* ── 障碍物运动学 ─────────────────────────────────────────────── */

static void obstacles_tick(void) {
    for (int i = 0; i < g.obstacle_count; i++) {
        SimObstacle* o = &g.obstacles[i];

        /* Defect 6: 行人周期性往返 — 到达对侧路沿后等待 3s 再反向横穿，
         * 而非永久停放。vy 不归零（保留幅值供反向使用），故此处只累计
         * 计时器，到点后翻转 vy 方向并清除停放标记。 */
        if (strcmp(o->type, "pedestrian") == 0 && o->ped_parked) {
            o->ped_wait_timer += DT_SEC;
            if (o->ped_wait_timer >= 3.0) {
                o->ped_wait_timer = 0.0;
                o->ped_parked = 0;
                o->ped_crossed_center = 0;
                o->vy = -o->vy;
            }
            continue;
        }

        /* Defect 1: 完全静止的障碍物（vx==0 && vy==0，如停放故障车、锥桶）
         * 跳过 X 循环瞬移与弯道跟随，保持原地——否则 ego 越过 100m 后会被
         * 瞬移到前方 150m，3D 场景里出现"障碍物突然跳到车前"的异常。 */
        if (o->vx == 0.0 && o->vy == 0.0) continue;

        o->x += o->vx * DT_SEC;
        o->y += o->vy * DT_SEC;
        if (strcmp(o->type, "pedestrian") == 0) {
            /* One-shot crossing behavior:
             * - enter road once
             * - after crossing centerline, stop at opposite curb
             * - 到达路沿后由上方 ped_parked 分支计时 3s 再反向（Defect 6） */
            if (o->y >= 7.8 && o->vy > 0.0) o->vy = -fabs(o->vy);
            if (o->y <= -7.8 && o->vy < 0.0) o->vy = fabs(o->vy);

            if (fabs(o->y) < 1.0) {
                o->ped_crossed_center = 1;
            }

            if (o->ped_crossed_center) {
                if (o->vy < 0.0 && o->y <= -7.2) {
                    o->y = -7.2;
                    o->ped_parked = 1;
                } else if (o->vy > 0.0 && o->y >= 7.2) {
                    o->y = 7.2;
                    o->ped_parked = 1;
                }
            }

            /* Pedestrian should not respawn/teleport with traffic flow. */
            continue;
        }

        /* Defect 2: 横向运动 Y 边界。带 vy 的非行人车辆若不设边界会沿 Y 无限
         * 漂移，3D 里飞出场景。切入车（|vy| 较小）变道一个车道宽度后停 vy；
         * 横穿车（|vy| 较大）驶离路面（|y|>10m）后停 vy。 */
        if (o->vy != 0.0) {
            if (fabs(o->vy) >= 4.0) {
                if (fabs(o->y) > 10.0) o->vy = 0.0;
            } else {
                if (fabs(o->y - o->cross_start_y) >= g.lane_width) o->vy = 0.0;
            }
        }

        double rel = o->x - g.vehicle.x;
        if (o->vx >= 0) {
            /* 循环障碍车只在远离视野 (自车后方 >100m) 时才重置位置,
             * 复位到前方 150m (>2.5×LiDAR 60m 量程), 确保不在传感器可见范围内。 */
            if (rel < -100.0) o->x = g.vehicle.x + 150.0 + (double)i * 5.0;
            if (rel >  220.0) o->x = g.vehicle.x + 150.0;
        } else {
            if (rel < -100.0) o->x = g.vehicle.x + 500.0;
        }

        /* 弯道跟随：仅对沿车道纵向行驶的车辆(vy==0)生效，横穿/路口来车
         * (vy!=0) 不受影响。curve_length_m<=0 时 road_center_y() 恒为 0，
         * 该分支对既有直道场景是无操作（o->y 与之前完全一致）。 */
        if (g.curve_length_m > 0.0 && o->vy == 0.0) {
            o->y = road_center_y(o->x, g.curve_start_x, g.curve_length_m, g.curve_offset_m)
                 + o->lane_offset;
        }
    }
}

/* ── 同车道前车间距 ───────────────────────────────────────────── */

static double lead_gap(void) {
    double best = 1e9;
    for (int i = 0; i < g.obstacle_count; i++) {
        SimObstacle* o = &g.obstacles[i];
        if (o->vx < 0) continue;
        if (fabs(o->y - g.vehicle.y) > SAME_LANE_TOL_M) continue;
        double center = o->x - g.vehicle.x;
        double gap = center - (o->len * 0.5 + EGO_LEN_M * 0.5);
        if (center > 0 && gap < best) best = gap;
    }
    return best;
}

/* ── 车辆运动学（bicycle model） ──────────────────────────────── */

static void vehicle_tick(void) {
    /* 驱动力 */
    double drive_force  = g.vehicle.throttle * 5000.0;
    double brake_force  = g.vehicle.brake    * 8000.0;
    double drag_force   = g.vehicle.drag_coeff * g.vehicle.speed * g.vehicle.speed;
    double net_force    = drive_force - brake_force - drag_force;
    double accel        = net_force / g.vehicle.mass;

    g.vehicle.speed += accel * DT_SEC;
    if (g.vehicle.speed < 0) g.vehicle.speed = 0;

    /* ── 横向: 收到外部 control/cmd 时完全交由外部 PID 控制 ── */
    if (g.has_control_input) {
        /* 外部 steer 已由 on_control_cmd 设置到 g.vehicle.steer,
         * 直接走自行车模型积分，不覆盖 steer。 */
    } else {
        /* ── 无外部控制时使用内置车道保持 + 平滑变道轨迹 ── */
        /* lane_target 是相对道路中心线的偏移（如 -1.75=左车道），
         * 弯道时需加上 road_center_y(ego.x) 得到世界横向目标。 */
        double road_c = road_center_y(g.vehicle.x, g.curve_start_x,
                                      g.curve_length_m, g.curve_offset_m);
        double lane_target_world = road_c + g.vehicle.lane_target;

        if (fabs(lane_target_world - g.lc_target_y) > 0.1 && !g.lc_active) {
            g.lc_start_y  = g.vehicle.y;
            g.lc_target_y = lane_target_world;
            g.lc_elapsed  = 0;
            g.lc_active   = 1;
        }
        if (g.lc_active && g.lc_elapsed >= g.lc_duration) {
            g.vehicle.y = g.lc_target_y;
            g.lc_active = 0;
        }

        double y_desired;
        if (g.lc_active) {
            g.lc_elapsed += DT_SEC;
            double t = g.lc_elapsed / g.lc_duration;
            if (t > 1.0) t = 1.0;
            double s = sin(t * M_PI / 2.0);
            y_desired = g.lc_start_y + (g.lc_target_y - g.lc_start_y) * s * s;
        } else {
            y_desired = lane_target_world;
        }

        double y_err   = y_desired - g.vehicle.y;
        double psi_des = 0.5 * y_err;
        psi_des = fmax(-0.3, fmin(0.3, psi_des));
        double steer   = 2.0 * (psi_des - g.vehicle.heading);
        steer = fmax(-0.25, fmin(0.25, steer));
        g.vehicle.steer = steer;
    }

    g.vehicle.heading += (g.vehicle.speed / g.vehicle.wheelbase)
                         * tan(g.vehicle.steer) * DT_SEC;
    g.vehicle.x += g.vehicle.speed * DT_SEC * cos(g.vehicle.heading);
    g.vehicle.y += g.vehicle.speed * DT_SEC * sin(g.vehicle.heading);
    /* 道路边界钳制：相对当前道路中心线（弯道禁用时中心线恒为 0，
     * 与之前的绝对钳制完全等价）。 */
    double road_c = road_center_y(g.vehicle.x, g.curve_start_x, g.curve_length_m, g.curve_offset_m);
    if (g.vehicle.y - road_c > ROAD_CENTER_LIMIT_M) {
        g.vehicle.y = road_c + ROAD_CENTER_LIMIT_M;
        if (g.vehicle.heading > 0.0) g.vehicle.heading = 0.0;
        if (g.vehicle.speed > 6.0) g.vehicle.speed = 6.0;
    } else if (g.vehicle.y - road_c < -ROAD_CENTER_LIMIT_M) {
        g.vehicle.y = road_c - ROAD_CENTER_LIMIT_M;
        if (g.vehicle.heading < 0.0) g.vehicle.heading = 0.0;
        if (g.vehicle.speed > 6.0) g.vehicle.speed = 6.0;
    }

    obstacles_tick();
}

/* ── 发布道路几何（road/geometry topic） ────────────────────── */
/* Phase 2 统一道路几何：sim_world 作为道路几何的唯一权威发布者，
 * control/planning/monitor 通过订阅本 topic 获取弯道参数，
 * 消除三节点各自 scenario_load 的冗余。
 * init() 时发布一次 + sim_thread 中每 ~1s 重发，确保后启动的订阅者也能收到。 */
#define ROAD_GEOMETRY_TYPE_ID          0x80AD5C12u
#define ROAD_GEOMETRY_REPUBLISH_CYCLES 50  /* 50 cycle ≈ 1s @ 50Hz */

static void publish_road_geometry(void) {
    char geo[256];
    int off = snprintf(geo, sizeof(geo),
             "{\"curve_start_x\":%.4f,\"curve_length_m\":%.4f,"
             "\"curve_offset_m\":%.4f,\"lane_width\":%.2f,\"lane_count\":2}",
             g.curve_start_x, g.curve_length_m, g.curve_offset_m, g.lane_width);
    transport_publish(g.transport, "road/geometry",
                      (const uint8_t*)geo, (uint32_t)off + 1);
}

/* ── control/cmd 订阅回调 ──────────────────────────────────── */

static void on_control_cmd(const Message* msg, void* user_data) {
    (void)user_data;
    if (!msg || !msg->data) return;

    /* Try binary deserialization first (serializer path) */
    {
        ControlCmd bin;
        if (ControlCmd_deserialize(&bin, (const uint8_t*)msg->data, msg->data_size) == 0) {
            g.vehicle.throttle     = bin.throttle;
            g.vehicle.brake        = bin.brake;
            g.vehicle.steer        = bin.steering;
            g.vehicle.target_speed = g.target_speed;  /* use config value, not hardcoded */
            g.has_control_input    = 1;
            struct timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts);
            g.last_control_cmd_us = (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;
            return;
        }
    }

    /* Fallback: text format parsing */
    const char* d = (const char*)msg->data;
    const char* p;
    if ((p = strstr(d, "throttle="))) sscanf(p + 9, "%lf", &g.vehicle.throttle);
    if ((p = strstr(d, "brake=")))    sscanf(p + 6,  "%lf", &g.vehicle.brake);
    if ((p = strstr(d, "steer=")))    sscanf(p + 6,  "%lf", &g.vehicle.steer);
    if ((p = strstr(d, "target=")))   sscanf(p + 7,  "%lf", &g.vehicle.target_speed);
    g.has_control_input = 1;
    struct timespec ts_fb; clock_gettime(CLOCK_MONOTONIC, &ts_fb);
    g.last_control_cmd_us = (uint64_t)ts_fb.tv_sec * 1000000ULL + (uint64_t)ts_fb.tv_nsec / 1000ULL;
}

/* ── 任务线程 ────────────────────────────────────────────────── */

static void* sim_thread(void* arg) {
    (void)arg;
    pthread_setname_np(pthread_self(), "sim_world");

    /* P0.1: 仿真时钟从 0 开始步进（逻辑时间，不依赖 wall-clock） */
    clock_set_sim_mode(true);
    clock_set_sim_time(0);
    clock_set_step_us(DT_US);

    while (!g.should_stop) {
        /* 用 wall-clock sleep 来限制实时输出速率，但逻辑时间由 clock 独立管理 */
        usleep((unsigned long)DT_US);
        if (g.should_stop) break;

        /* 控制指令陈旧超时: 500ms 未收到则回退到内置巡航 */
        if (g.has_control_input) {
            struct timespec now_ts; clock_gettime(CLOCK_MONOTONIC, &now_ts);
            uint64_t now_us = (uint64_t)now_ts.tv_sec * 1000000ULL + (uint64_t)now_ts.tv_nsec / 1000ULL;
            if (now_us - g.last_control_cmd_us > 500000ULL) {
                g.has_control_input = 0;
                LOG_WARN("sim_world", "control/cmd stale >500ms — fallback to internal cruise");
            }
        }

        /* 未收到 control/cmd 时：使用内置简单巡航 + 车道保持 */
        if (!g.has_control_input) {
            double gap = lead_gap();
            double time_hw  = 2.0;
            double min_gap  = 6.0;
            double safe_gap = min_gap + g.vehicle.speed * time_hw;
            double target   = g.target_speed;

            if (gap < safe_gap && gap < 80.0) {
                double ratio = gap / safe_gap;
                if (ratio < 0.2) ratio = 0.2;
                target = g.target_speed * ratio;
            }

            double error = target - g.vehicle.speed;
            double output = 800.0 * error;
            if (output > 0) {
                g.vehicle.throttle = output / 5000.0;
                if (g.vehicle.throttle > 1.0) g.vehicle.throttle = 1.0;
                g.vehicle.brake = 0;
            } else {
                g.vehicle.throttle = 0;
                g.vehicle.brake = (-output) / 8000.0;
                if (g.vehicle.brake > 1.0) g.vehicle.brake = 1.0;
            }
        }

        /* ── AEB 兜底（物理运算前执行，确保本帧生效） ── */
        if (g.collision_recovery_ticks <= 0) {
            /* 通用 AEB — 同车道前车 */
            double gap = lead_gap();
            double safe_gap = 6.0 + g.vehicle.speed * 2.0;
            if (gap < AEB_GAP_RATIO * safe_gap && gap < AEB_MAX_GAP_M) {
                g.vehicle.throttle = 0;
                g.vehicle.brake    = 1.0;
            }
            /* 行人/障碍物 AEB — 行人不限车道检测（随时可能横穿）；
             * 车辆类障碍物必须收紧到同车道容差，否则相邻车道内的正常
             * 车辆 (dy 可达半个车道宽 ~1.75m, 之前误用 5.0m 容差) 会被
             * 当成"即将横穿"的行人持续触发满刹车，导致本车速度被死锁
             * 钉死在旁道车速上，永远无法超车（表现为"跟在车屁股后面"）。 */
            for (int i = 0; i < g.obstacle_count; i++) {
                SimObstacle* o = &g.obstacles[i];
                double dx = o->x - g.vehicle.x;
                double dy = fabs(o->y - g.vehicle.y);
                if (dx < 0.0 || dx > AEB_MAX_GAP_M) continue;
                int is_pedestrian = (strcmp(o->type, "pedestrian") == 0);
                double dy_tol = is_pedestrian ? PEDESTRIAN_LATERAL_TOL_M : SAME_LANE_TOL_M;
                if (dy > dy_tol) continue;
                double frontal_gap = dx - (o->len * 0.5 + EGO_LEN_M * 0.5);
                /* 阈值分离（缺陷4修复）：行人保留 max(10, speed*2) 保守距离；
                 * 车辆类障碍物改用 AEB_GAP_RATIO*(6+speed*2)，避免拥堵跟车
                 * 时 10m 平台阈值导致"急刹—停—急刹"剧烈抖动。 */
                double safe_gap = is_pedestrian
                    ? fmax(10.0, g.vehicle.speed * 2.0)
                    : AEB_GAP_RATIO * (6.0 + g.vehicle.speed * 2.0);
                if (frontal_gap < safe_gap) {
                    g.vehicle.throttle = 0;
                    g.vehicle.brake    = 1.0;
                    break;
                }
            }
        }

        vehicle_tick();

        /* ── AABB 碰撞检测 ── */
        const double ego_half_len = EGO_LEN_M * 0.5;
        const double ego_half_wid = EGO_WID_M * 0.5;
        for (int i = 0; i < g.obstacle_count; i++) {
            SimObstacle* o = &g.obstacles[i];
            double overlap_x = (ego_half_len + o->len * 0.5) - fabs(g.vehicle.x - o->x);
            double overlap_y = (ego_half_wid + o->wid * 0.5) - fabs(g.vehicle.y - o->y);
            if (overlap_x > 0.10 && overlap_y > 0.30) {
                if (g.collision_cooldown[i] <= 0) {
                    LOG_ERROR("sim_world", "COLLISION ego(%.1f,%.1f) ↔ obs%d(%.1f,%.1f) ovlp(%.2f,%.2f)",
                              g.vehicle.x, g.vehicle.y, i, o->x, o->y, overlap_x, overlap_y);
                    char col[128];
                    snprintf(col, sizeof(col),
                             "{\"ego_x\":%.1f,\"ego_y\":%.1f,\"obs_id\":%d,\"overlap_x\":%.2f,\"overlap_y\":%.2f}",
                             g.vehicle.x, g.vehicle.y, i, overlap_x, overlap_y);
                    transport_publish(g.transport, "sim/collision",
                                      (const uint8_t*)col, (uint32_t)strlen(col) + 1);
                    g.collision_cooldown[i] = (int)FREQUENCY_HZ * 3;  /* 3 秒冷却，防止恢复后立即重判 */
                    g.collision_recovery_ticks = (int)(FREQUENCY_HZ * 3);  /* 3 秒碰撞冻结 */
                }
                /* 碰撞冻结期内强制停车；冻结期外允许物理恢复分离 */
                if (g.collision_recovery_ticks > 0) {
                    g.vehicle.speed    = 0.0;
                    g.vehicle.throttle = 0.0;
                    g.vehicle.brake    = 1.0;
                }
            } else {
                if (g.collision_cooldown[i] > 0) g.collision_cooldown[i]--;
            }
        }

        /* 碰撞冻结倒数：冻结期结束后允许重新起步 */
        if (g.collision_recovery_ticks > 0) {
            g.collision_recovery_ticks--;
            if (g.collision_recovery_ticks == 0) {
                LOG_INFO("sim_world", "collision freeze expired — allowing movement");
            }
        }

        /* ── P0.1: 推进逻辑时钟并发布 sim/tick ── */
        clock_advance_us(DT_US);
        uint64_t sim_time_us = clock_now_us();
        char tick_buf[64];
        snprintf(tick_buf, sizeof(tick_buf),
                 "{\"t_us\":%" PRIu64 ",\"cycle\":%u}", sim_time_us, g.cycle);
        transport_publish(g.transport, "sim/tick",
                          (const uint8_t*)tick_buf, (uint32_t)strlen(tick_buf) + 1);

        /* Phase 2: 定期重发 road/geometry（~1Hz），确保后启动的订阅者收到 */
        if (g.cycle % ROAD_GEOMETRY_REPUBLISH_CYCLES == 0) {
            publish_road_geometry();
        }

        /* ── 发布 vehicle/state（动态生成，覆盖实际 actor 数量）── */
        /* Buffer sizing: fixed header ~150 B + per-obstacle ~100 B (worst: "pedestrian")
         * × SIM_OBSTACLE_COUNT(16) ≈ 1750 B → 2048 is sufficient.
         * The loop guard (voff < sizeof - 128) ensures the closing "}" always fits;
         * if the estimate is wrong the tail of the JSON is truncated — receivers must
         * tolerate missing keys (they already do via json_extract_double returning 0).
         * Phase 2: 弯道几何不再附在 vehicle/state 中，改由 road/geometry topic 独立发布。 */
        char vstate[2048];
        int voff = snprintf(vstate, sizeof(vstate),
                 "{\"x\":%.4f,\"y\":%.4f,\"spd\":%.3f,\"hdg\":%.4f,"
                 "\"thr\":%.3f,\"brk\":%.3f,\"tgt\":%.2f,\"st\":%.4f,"
                 "\"t_us\":%" PRIu64 ",\"n_obs\":%d",
                 g.vehicle.x, g.vehicle.y, g.vehicle.speed, g.vehicle.heading,
                 g.vehicle.throttle, g.vehicle.brake, g.vehicle.target_speed, g.vehicle.steer,
                 sim_time_us, g.obstacle_count);
        for (int i = 0; i < g.obstacle_count && voff < (int)sizeof(vstate) - VSTATE_JSON_FOOTER_RESERVE; i++) {
            voff += snprintf(vstate + voff, sizeof(vstate) - (size_t)voff,
                             ",\"ox%d\":%.2f,\"oy%d\":%.2f,\"ov%d\":%.3f"
                             ",\"ovy%d\":%.3f,\"ot%d\":\"%s\",\"ol%d\":%.2f,\"ow%d\":%.2f",
                             i, g.obstacles[i].x,
                             i, g.obstacles[i].y,
                             i, g.obstacles[i].vx,
                             i, g.obstacles[i].vy,
                             i, g.obstacles[i].type,
                             i, g.obstacles[i].len,
                             i, g.obstacles[i].wid);
        }
        voff += snprintf(vstate + voff, sizeof(vstate) - (size_t)voff, "}");
        transport_publish(g.transport, "vehicle/state", (const uint8_t*)vstate,
                          (uint32_t)voff + 1);

        g.cycle++;
    }

    LOG_INFO("sim_world", "stopped (%u cycles, sim_time=%.3fs, final speed=%.1f m/s, state=%s)",
             g.cycle, (double)clock_now_us() / 1e6, g.vehicle.speed,
             statem_state_name(&g.sm, g.sm.current));
    statem_send_event(&g.sm, SM_EVENT_STOP, NULL);
    statem_send_event(&g.sm, SM_EVENT_DONE, NULL);
    return NULL;
}

/* ── NodePlugin 实现 ─────────────────────────────────────────── */

static const char* s_inputs[]  = { "control/cmd", NULL };
static const char* s_outputs[] = { "vehicle/state", "sim/world_state", "sim/collision", "sim/tick", "road/geometry", NULL };

static NodePlugin s_plugin;  /* forward decl */
static int sim_init(MessageBus* bus, Transport* transport,
                    DiscoveryManager* discovery, Scheduler* scheduler,
                    const char* params_json) {
    (void)bus; (void)scheduler;

    memset(&g, 0, sizeof(g));
    g.transport    = transport;
    g.discovery    = discovery;
    g.should_stop  = 0;

    /* 默认参数 */
    g.init_speed    = 5.0;
    g.target_speed  = 10.0;
    g.lane_width    = 3.5;
    g.random_seed   = 42;   /* P0.1: 固定默认种子 */

    if (params_json) {
        const char* p;
        if ((p = strstr(params_json, "\"init_speed\":")))
            sscanf(p + 13, "%lf", &g.init_speed);
        if ((p = strstr(params_json, "\"target_speed\":")))
            sscanf(p + 14, "%lf", &g.target_speed);
        if ((p = strstr(params_json, "\"lane_width\":")))
            sscanf(p + 12, "%lf", &g.lane_width);
        if ((p = strstr(params_json, "\"lane_target\":")))
            sscanf(p + 13, "%lf", &g.vehicle.lane_target);
        /* P0.1: 固定随机种子 */
        if ((p = strstr(params_json, "\"random_seed\":")))
            sscanf(p + 14, "%u", &g.random_seed);
        /* P0.2: 场景文件 */
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

    /* P0.2: 优先从场景文件加载 actors；否则使用内置默认 */
    ScenarioConfig* scenario = NULL;
    if (g.scenario_file[0] != '\0') {
        scenario = scenario_load(g.scenario_file);
    }

    /* Seed precedence (highest to lowest): scenario file → params JSON → built-in default (42).
     * 场景文件的种子优先于 params 里的种子（两者都有时以场景为准） */
    if (scenario) {
        g.random_seed = scenario->random_seed;
        /* 道路弯道几何（可选）：缺省字段全为 0 = 直道，行为与之前完全一致 */
        g.curve_start_x  = scenario->road.curve_start_x;
        g.curve_length_m = scenario->road.curve_length_m;
        g.curve_offset_m = scenario->road.curve_offset_m;
    }

    /* P0.1: 使用固定种子（保证确定性），在所有种子来源确定后只调用一次 */
    srand(g.random_seed);

    /* 车辆初始状态：场景文件 > params > 内置默认 */
    double ego_x = 0.0, ego_y = -1.75, ego_heading = 0.0;
    if (scenario) {
        ego_x       = scenario->ego.x;
        ego_y       = scenario->ego.y;
        ego_heading = scenario->ego.heading;
        if (scenario->ego.init_speed > 0)   g.init_speed   = scenario->ego.init_speed;
        if (scenario->ego.target_speed > 0) g.target_speed = scenario->ego.target_speed;
    }

    g.vehicle.x           = ego_x;
    g.vehicle.y           = ego_y;
    g.vehicle.speed       = g.init_speed;
    g.vehicle.heading     = ego_heading;
    g.vehicle.steer       = 0.0;
    g.vehicle.throttle    = 0.0;
    g.vehicle.brake       = 0.0;
    g.vehicle.target_speed = g.target_speed;
    g.vehicle.lane_target  = ego_y;
    g.vehicle.wheelbase   = 2.7;
    g.vehicle.mass        = 1500.0;
    g.vehicle.drag_coeff  = 0.3;

    /* 变道状态 */
    g.lc_duration = 3.5;

    if (scenario) {
        init_obstacles_from_scenario(scenario);
        scenario_free(scenario);
    } else {
        init_obstacles_default();
    }

    /* 订阅 control/cmd */
    transport_subscribe(transport, "control/cmd", on_control_cmd, NULL);
    discovery_advertise(discovery, "control/cmd", 0x2D95C6D2u, CAP_SUBSCRIBER, 0);

    /* 发布 vehicle/state */
    transport_advertise(transport, "vehicle/state", 0x1C0E5A7Eu);
    discovery_advertise(discovery, "vehicle/state", 0x1C0E5A7Eu, CAP_PUBLISHER, 20.0);

    /* 发布 sim/collision（碰撞事件） */
    transport_advertise(transport, "sim/collision", 0xC0115101u);
    discovery_advertise(discovery, "sim/collision", 0xC0115101u, CAP_PUBLISHER, 0);

    /* P0.1: 发布 sim/tick（每步逻辑时钟心跳） */
    transport_advertise(transport, "sim/tick", 0x51C7710Cu);
    discovery_advertise(discovery, "sim/tick", 0x51C7710Cu, CAP_PUBLISHER, FREQUENCY_HZ);

    /* Phase 2: 发布 road/geometry — 道路几何的唯一权威来源。
     * control/planning/monitor 订阅此 topic 获取弯道参数，不再各自 scenario_load。 */
    transport_advertise(transport, "road/geometry", ROAD_GEOMETRY_TYPE_ID);
    discovery_advertise(discovery, "road/geometry", ROAD_GEOMETRY_TYPE_ID, CAP_PUBLISHER, 1.0);
    publish_road_geometry();  /* init 时立即发布一次，确保订阅者启动时即有数据 */

    /* 初始化反射式状态机 */
    statem_init(&g.sm, NULL, SM_STATE_INITIALIZED, "sim_world");
    statem_send_event(&g.sm, SM_EVENT_START, NULL);

    LOG_INFO("sim_world", "initialized (init=%.1f m/s, target=%.1f m/s, %.0f Hz, "
             "seed=%u, actors=%d, scenario='%s')",
             g.init_speed, g.target_speed, FREQUENCY_HZ,
             g.random_seed, g.obstacle_count,
             g.scenario_file[0] ? g.scenario_file : "(built-in)");
    return 0;
}

static int sim_start(void) {
    g.running = 1; g.should_stop = 0;
    if (pthread_create(&g.thread, NULL, sim_thread, NULL) != 0) return -1;
    LOG_INFO("sim_world", "started [state=%s]", statem_state_name(&g.sm, g.sm.current));
    node_announce_self(g.transport, &s_plugin);  /* start() 时广播: monitor 已订阅 */
    return 0;
}

static void sim_stop(void)          { g.should_stop = 1; }
static void sim_cleanup(void) {
    if (g.running) { g.should_stop = 1; pthread_join(g.thread, NULL); g.running = 0; }
    LOG_INFO("sim_world", "cleanup done");
}
static int  sim_health(void)        { return 0; }

static NodePlugin s_plugin = {
    .api_version   = NODE_PLUGIN_API_VERSION,
    .name          = "sim_world",
    .version       = "1.0.0",
    .description   = "Vehicle dynamics + obstacle simulation",
    .input_topics  = s_inputs,
    .output_topics = s_outputs,
    .init          = sim_init,
    .start         = sim_start,
    .stop          = sim_stop,
    .cleanup       = sim_cleanup,
    .health        = sim_health,
};

NodePlugin* node_get_plugin(void) { return &s_plugin; }
