/**
 * flowsim_node.cpp — FlowSim v2 仿真世界节点 (C++20 + flowcoro + esmini)
 *
 * 替换 sim_world_node.c。集成 Phase 1 组件（road_network / entity / physics /
 * npc_ai / collision / scene_events）为一个完整的 20Hz 仿真主循环节点。
 *
 * 架构（见 docs/FLOWSIM_ARCHITECTURE.md）：
 *   - esmini RoadManager 处理道路网络（Frenet↔World / 限速 / 车道）
 *   - Entity System 固定池管理 ego + NPC + 事件触发器
 *   - 自行车模型积分 ego 动力学
 *   - IDM 跟车 + 状态机驱动 NPC
 *   - OBB SAT 碰撞检测
 *   - 红绿灯/ETC 场景事件调度
 *   - flowcoro 协程主循环（FlowCoroTask，线程池 resume）
 *
 * 向后兼容（与 sim_world_node.c 的 topic 契约完全一致）：
 *   订阅: control/cmd（二进制 ControlCmd + JSON fallback）
 *   发布: vehicle/state, road/geometry, road/traffic_lights, sim/tick, sim/collision
 *
 * 采用 FlowCoroTask（线程池 resume）：物理积分 + NPC AI + 碰撞 + 事件 + JSON
 * 序列化单次 ~50-200μs，同步 resume 会阻塞消息总线分发线程，故改用线程池 resume。
 */

#include "node_plugin.h"
#include "scenario_loader.h"
#include "road_geometry.h"
#include "clock_service.h"
#include "coroutine_task.h"
#include "logger.h"
#include <cjson/cJSON.h>

/* adas_msgs_gen.h 提供 ControlCmd 二进制反序列化（control/cmd） */
#include "adas_msgs_gen.h"

#include "flowsim/road_network.h"
#include "flowsim/entity.h"
#include "flowsim/physics.h"
#include "flowsim/npc_ai.h"
#include "flowsim/collision.h"
#include "flowsim/scene_events.h"
#include "flowsim/scene_pub.h"

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <time.h>
#include <pthread.h>
#include <unistd.h>
#include <sys/stat.h>

#include <memory>
#include <string>
#include <vector>
#include <atomic>

namespace {

/* ── 仿真常量（与 sim_world_node.c 一致，保证下游节点兼容） ───── */

#define FLOWSIM_FREQUENCY_HZ   20.0
#define FLOWSIM_DT_SEC         (1.0 / FLOWSIM_FREQUENCY_HZ)   /* 0.05s */
#define FLOWSIM_DT_US          ((uint64_t)(FLOWSIM_DT_SEC * 1e6))  /* 50000 */

/* control/cmd 陈旧超时：500ms 未收到则回退内置巡航（同 sim_world_node） */
#define CONTROL_STALE_TIMEOUT_US  500000ULL

/* road/geometry 重发周期：50 cycle ≈ 1s @ 20Hz（同 sim_world_node） */
#define ROAD_GEOMETRY_REPUBLISH_CYCLES 50

/* topic type IDs（与 sim_world_node.c 一致） */
#define VEHICLE_STATE_TYPE_ID       0x1C0E5A7Eu
#define SIM_COLLISION_TYPE_ID       0xC0115101u
#define SIM_TICK_TYPE_ID            0x51C7710Cu
#define ROAD_GEOMETRY_TYPE_ID       0x80AD5C12u
#define ROAD_TRAFFIC_LIGHTS_TYPE_ID 0x7E5C0FFEu
#define CONTROL_CMD_TYPE_ID         0x2D95C6D2u
/* scene/frame 是 FlowSim v2 新增 topic，无历史兼容负担，独立 type ID */
#define SCENE_FRAME_TYPE_ID         0x5CE4E011u

/* ego 车辆几何（与 sim_world_node EGO_LEN_M/EGO_WID_M 一致） */
#define EGO_LEN_M   4.6
#define EGO_WID_M   2.0

/* ── 节点状态 ───────────────────────────────────────────────── */

struct FlowSimContext {
    /* 注入的基础设施 */
    Transport*        transport{nullptr};
    DiscoveryManager* discovery{nullptr};
    Scheduler*        scheduler{nullptr};

    /* 协程宿主线程 */
    pthread_t         thread{};
    bool              running{false};
    std::atomic<bool> should_stop{false};

    /* Phase 1 组件 */
    flowsim::FlowRoadNetwork  roads;
    flowsim::EntityPool       pool;
    flowsim::NpcAiConfig      ai_cfg;

    /* Phase 2.2: scene/frame 发布配置（init 阶段填充，主循环每帧传入） */
    flowsim::ScenePubConfig   scene_pub_cfg;

    /* 场景配置 */
    ScenarioConfig*   scenario{nullptr};
    char              scenario_file[256]{};
    double            init_speed{5.0};
    double            target_speed{12.0};
    double            lane_width{3.5};
    uint32_t          random_seed{42};

    /* 道路几何（可选弯道，兼容旧场景；esmini 加载失败时仍用此做车道保持） */
    double            curve_start_x{0};
    double            curve_length_m{0};
    double            curve_offset_m{0};

    /* control/cmd 状态 */
    std::atomic<int>  has_control_input{0};
    std::atomic<uint64_t> last_control_cmd_us{0};
    /* ego 当前控制量（on_control_cmd 写，tick 读；单写单读，无需锁） */
    std::atomic<double> ego_throttle{0};
    std::atomic<double> ego_brake{0};
    std::atomic<double> ego_steer{0};

    /* 统计 */
    uint32_t          cycle{0};
    uint64_t          sim_start_us{0};
    bool              roads_loaded{false};

    /* 协程任务 */
    std::unique_ptr<class FlowSimTask> task;
};

FlowSimContext g;

/* ── control/cmd 订阅回调 ─────────────────────────────────────── */
/* 与 sim_world_node.c on_control_cmd 行为一致：先试二进制 ControlCmd，
 * 失败则 JSON fallback。解析结果写入 g.ego_throttle/brake/steer。 */

static void on_control_cmd(const Message* msg, void* user_data) {
    (void)user_data;
    if (!msg || msg->data_size == 0) return;

    /* 二进制 ControlCmd 路径 */
    {
        ControlCmd bin;
        if (ControlCmd_deserialize(&bin, (const uint8_t*)msg->data, msg->data_size) == 0) {
            g.ego_throttle.store(bin.throttle, std::memory_order_relaxed);
            g.ego_brake.store(bin.brake, std::memory_order_relaxed);
            g.ego_steer.store(bin.steering, std::memory_order_relaxed);
            g.has_control_input.store(1, std::memory_order_relaxed);
            g.last_control_cmd_us.store(clock_now_us(), std::memory_order_relaxed);
            return;
        }
    }

    /* JSON fallback */
    cJSON* root = cJSON_Parse((const char*)msg->data);
    if (root) {
        cJSON* j;
        if ((j = cJSON_GetObjectItemCaseSensitive(root, "throttle")) && cJSON_IsNumber(j))
            g.ego_throttle.store(j->valuedouble, std::memory_order_relaxed);
        if ((j = cJSON_GetObjectItemCaseSensitive(root, "brake")) && cJSON_IsNumber(j))
            g.ego_brake.store(j->valuedouble, std::memory_order_relaxed);
        if ((j = cJSON_GetObjectItemCaseSensitive(root, "steer")) && cJSON_IsNumber(j))
            g.ego_steer.store(j->valuedouble, std::memory_order_relaxed);
        cJSON_Delete(root);
    }
    g.has_control_input.store(1, std::memory_order_relaxed);
    g.last_control_cmd_us.store(clock_now_us(), std::memory_order_relaxed);
}

/* ── JSON → xodr 转换（调 json_to_xodr.py 子进程） ─────────────── */
/* 在 init() 中调用一次，把场景 JSON 转成 esmini 可加载的 xodr。
 * 失败返回空字符串，调用方按 roads=nullptr 降级（NPC AI 用横向距离判车道）。 */

static std::string convert_scenario_to_xodr(const std::string& scenario_path) {
    /* 输出到 /tmp，文件名用场景 basename 避免并发场景冲突 */
    const char* slash = strrchr(scenario_path.c_str(), '/');
    std::string base = slash ? slash + 1 : scenario_path;
    /* 去掉 .json 后缀 */
    if (base.size() > 5 && base.compare(base.size() - 5, 5, ".json") == 0) {
        base.resize(base.size() - 5);
    }
    std::string xodr_path = "/tmp/flowsim_" + base + ".xodr";

    /* json_to_xodr.py 候选路径：场景目录上级的 tools/ + /workspace/tools/ */
    std::string scenario_dir;
    if (slash) scenario_dir.assign(scenario_path.c_str(), slash);
    std::vector<std::string> candidates = {
        scenario_dir + "/../tools/json_to_xodr.py",
        "/workspace/tools/json_to_xodr.py",
        "tools/json_to_xodr.py",
    };

    for (const auto& tool : candidates) {
        struct stat st;
        if (stat(tool.c_str(), &st) != 0) continue;

        /* 构造命令：python3 <tool> <scenario> -o <xodr> */
        std::string cmd = "python3 \"";
        cmd += tool;
        cmd += "\" \"";
        cmd += scenario_path;
        cmd += "\" -o \"";
        cmd += xodr_path;
        cmd += "\" >/dev/null 2>&1";
        int rc = system(cmd.c_str());
        if (rc == 0 && stat(xodr_path.c_str(), &st) == 0 && st.st_size > 0) {
            LOG_INFO("flowsim", "json_to_xodr: %s → %s (via %s)",
                     scenario_path.c_str(), xodr_path.c_str(), tool.c_str());
            return xodr_path;
        }
        LOG_WARN("flowsim", "json_to_xodr failed (rc=%d) via %s", rc, tool.c_str());
    }

    LOG_WARN("flowsim", "json_to_xodr: no working tool found, esmini road network disabled");
    return "";
}

/* ── 从场景配置填充 EntityPool ─────────────────────────────────── */

static flowsim::EntityType actor_type_to_entity(const char* type) {
    if (!type) return flowsim::EntityType::Car;
    if (strcmp(type, "pedestrian") == 0) return flowsim::EntityType::Pedestrian;
    if (strcmp(type, "truck") == 0)      return flowsim::EntityType::Truck;
    if (strcmp(type, "suv") == 0)        return flowsim::EntityType::SUV;
    return flowsim::EntityType::Car;
}

static void populate_entities_from_scenario(const ScenarioConfig* sc) {
    g.pool.clear();

    /* Ego 固定 index 0 */
    flowsim::EntityId ego_id = g.pool.alloc(flowsim::EntityType::Ego);
    flowsim::Entity& ego = g.pool[ego_id];
    ego.x = sc->ego.x;
    ego.y = sc->ego.y;
    ego.heading = sc->ego.heading;
    ego.speed = (sc->ego.init_speed > 0) ? sc->ego.init_speed : g.init_speed;
    ego.target_vx = (sc->ego.target_speed > 0) ? sc->ego.target_speed : g.target_speed;
    ego.vx = ego.speed;  /* 初始沿 x 方向 */
    ego.length = EGO_LEN_M;
    ego.width = EGO_WID_M;
    ego.wheelbase = 2.7;
    ego.mass = 1500.0;
    ego.drag_coeff = 0.3;
    flowsim::apply_vehicle_defaults(ego);

    /* Actors → NPC 车辆 / 行人 */
    for (int i = 0; i < sc->actor_count && i < flowsim::MAX_ENTITIES - 1; i++) {
        const ScenarioActor* a = &sc->actors[i];
        flowsim::EntityType et = actor_type_to_entity(a->type);
        flowsim::EntityId id = g.pool.alloc(et);
        if (id == flowsim::INVALID_ENTITY) break;
        flowsim::Entity& e = g.pool[id];
        e.id = a->id;
        e.x = a->x;
        /* 沿车道行驶的车 (vy==0) 需加道路中心线偏移；行人/横穿车用绝对 y */
        if (a->vy == 0.0) {
            e.y = road_center_y(a->x, g.curve_start_x, g.curve_length_m, g.curve_offset_m) + a->y;
        } else {
            e.y = a->y;
        }
        e.vx = a->vx;
        e.vy = a->vy;
        e.speed = sqrt(a->vx * a->vx + a->vy * a->vy);
        e.target_vx = a->vx;  /* NPC 初始目标速度 = 场景速度 */
        e.length = (a->len > 0) ? a->len : 4.6;
        e.width = (a->wid > 0) ? a->wid : 2.0;
        if (e.is_vehicle()) {
            flowsim::apply_vehicle_defaults(e);
            e.length = (a->len > 0) ? a->len : e.length;
            e.width = (a->wid > 0) ? a->wid : e.width;
        }
    }

    /* 红绿灯 → TrafficLight 实体 */
    for (int i = 0; i < sc->traffic_light_count && i < SCENARIO_MAX_TRAFFIC_LIGHTS; i++) {
        const ScenarioTrafficLight* tl = &sc->traffic_lights[i];
        flowsim::EntityId id = g.pool.alloc(flowsim::EntityType::TrafficLight);
        if (id == flowsim::INVALID_ENTITY) break;
        flowsim::Entity& e = g.pool[id];
        e.id = tl->id;
        e.x = tl->x;
        e.y = tl->y_lane;
        /* 相位时长复用字段（见 scene_events.h 约定） */
        e.throttle = tl->green_s;     /* green 时长 */
        e.brake    = tl->yellow_s;    /* yellow 时长 */
        e.steer    = tl->red_s;       /* red 时长 */
        e.target_vx = tl->phase_offset_s;  /* 相位偏移 */
    }

    /* Phase 4: ETC 门架 → ETCGate 实体（高速收费站抬杆）。
     * scene_events.cpp 的 tick_etc_gates() 根据 ego 距离门架的距离驱动
     * 抬杆动画：远距 closed → 进入 open_range_m 时 opening → 通过后 open。
     * approach_speed 复用 target_vx 字段，open_range_m 复用 phase_timer 字段。 */
    for (int i = 0; i < sc->etc_gate_count && i < SCENARIO_MAX_ETC_GATES; i++) {
        const ScenarioETCGate* eg = &sc->etc_gates[i];
        flowsim::EntityId id = g.pool.alloc(flowsim::EntityType::ETCGate);
        if (id == flowsim::INVALID_ENTITY) break;
        flowsim::Entity& e = g.pool[id];
        e.id = eg->id;
        e.x = eg->x;
        e.y = eg->y;
        e.target_vx = eg->approach_speed;   /* ETC 通过目标速度 */
        e.phase_timer = 0.0;                 /* 抬杆进度 [0,1]，初始 closed */
        /* open_range_m 存到 width 字段（ETCGate 无碰撞包围盒，width 空闲） */
        e.width = eg->open_range_m;
        e.ai_state = flowsim::AIState::Stop; /* 初始 closed 状态 */
    }

    /* Phase 4: 停止线 → StopLine 实体（路口/ETC 停车位置标记）。
     * 纯可视化标记，无动力学无状态机，scene_pub 直接序列化位置。 */
    for (int i = 0; i < sc->stop_line_count && i < SCENARIO_MAX_STOP_LINES; i++) {
        const ScenarioStopLine* sl = &sc->stop_lines[i];
        flowsim::EntityId id = g.pool.alloc(flowsim::EntityType::StopLine);
        if (id == flowsim::INVALID_ENTITY) break;
        flowsim::Entity& e = g.pool[id];
        e.id = sl->id;
        e.x = sl->x;
        e.y = sl->y;
    }
}

/* ── 发布函数 ─────────────────────────────────────────────────── */

static void publish_vehicle_state(uint64_t sim_time_us) {
    flowsim::Entity& ego = g.pool[0];

    cJSON* vstate = cJSON_CreateObject();
    cJSON_AddNumberToObject(vstate, "x", ego.x);
    cJSON_AddNumberToObject(vstate, "y", ego.y);
    cJSON_AddNumberToObject(vstate, "spd", ego.speed);
    cJSON_AddNumberToObject(vstate, "hdg", ego.heading);
    cJSON_AddNumberToObject(vstate, "thr", ego.throttle);
    cJSON_AddNumberToObject(vstate, "brk", ego.brake);
    cJSON_AddNumberToObject(vstate, "tgt", ego.target_vx);
    cJSON_AddNumberToObject(vstate, "st", ego.steer);
    cJSON_AddNumberToObject(vstate, "t_us", (double)sim_time_us);

    /* 收集非 ego 的活跃实体作为障碍物（车辆 + 行人） */
    int n_obs = 0;
    for (int i = 1; i < g.pool.size(); i++) {
        const flowsim::Entity& e = g.pool[i];
        if (!e.active) continue;
        if (e.type == flowsim::EntityType::TrafficLight ||
            e.type == flowsim::EntityType::ETCGate ||
            e.type == flowsim::EntityType::StopLine) continue;
        if (n_obs >= 16) break;

        char key[20];
        snprintf(key, sizeof(key), "ox%d", n_obs);
        cJSON_AddNumberToObject(vstate, key, e.x);
        snprintf(key, sizeof(key), "oy%d", n_obs);
        cJSON_AddNumberToObject(vstate, key, e.y);
        snprintf(key, sizeof(key), "ov%d", n_obs);
        cJSON_AddNumberToObject(vstate, key, e.vx);
        snprintf(key, sizeof(key), "ovy%d", n_obs);
        cJSON_AddNumberToObject(vstate, key, e.vy);
        snprintf(key, sizeof(key), "ot%d", n_obs);
        const char* tname = "car";
        switch (e.type) {
            case flowsim::EntityType::Pedestrian: tname = "pedestrian"; break;
            case flowsim::EntityType::Truck:      tname = "truck"; break;
            case flowsim::EntityType::SUV:        tname = "suv"; break;
            default: break;
        }
        cJSON_AddStringToObject(vstate, key, tname);
        snprintf(key, sizeof(key), "ol%d", n_obs);
        cJSON_AddNumberToObject(vstate, key, e.length);
        snprintf(key, sizeof(key), "ow%d", n_obs);
        cJSON_AddNumberToObject(vstate, key, e.width);
        n_obs++;
    }
    cJSON_AddNumberToObject(vstate, "n_obs", n_obs);

    char* s = cJSON_PrintUnformatted(vstate);
    transport_publish(g.transport, "vehicle/state",
                      (const uint8_t*)s, (uint32_t)strlen(s) + 1);
    free(s);
    cJSON_Delete(vstate);
}

static void publish_road_geometry(void) {
    cJSON* root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "curve_start_x", g.curve_start_x);
    cJSON_AddNumberToObject(root, "curve_length_m", g.curve_length_m);
    cJSON_AddNumberToObject(root, "curve_offset_m", g.curve_offset_m);
    cJSON_AddNumberToObject(root, "lane_width", g.lane_width);
    cJSON_AddNumberToObject(root, "lane_count", 2);
    char* s = cJSON_PrintUnformatted(root);
    transport_publish(g.transport, "road/geometry",
                      (const uint8_t*)s, (uint32_t)strlen(s) + 1);
    free(s);
    cJSON_Delete(root);
}

static const char* tl_phase_str(flowsim::TLPhase ph) {
    switch (ph) {
        case flowsim::TLPhase::Green:  return "green";
        case flowsim::TLPhase::Yellow: return "yellow";
        case flowsim::TLPhase::Red:    return "red";
    }
    return "green";
}

static void publish_traffic_lights() {
    /* 无红绿灯实体则不发布（同 sim_world_node） */
    bool has_tl = false;
    for (int i = 0; i < g.pool.size(); i++) {
        if (g.pool[i].active && g.pool[i].type == flowsim::EntityType::TrafficLight) {
            has_tl = true; break;
        }
    }
    if (!has_tl) return;

    cJSON* root = cJSON_CreateObject();
    cJSON* lights = cJSON_CreateArray();
    for (int i = 0; i < g.pool.size(); i++) {
        const flowsim::Entity& e = g.pool[i];
        if (!e.active || e.type != flowsim::EntityType::TrafficLight) continue;
        cJSON* light = cJSON_CreateObject();
        cJSON_AddNumberToObject(light, "id", e.id);
        cJSON_AddNumberToObject(light, "x", e.x);
        cJSON_AddNumberToObject(light, "y_lane", e.y);
        flowsim::TLPhase ph = static_cast<flowsim::TLPhase>(e.phase_state);
        cJSON_AddStringToObject(light, "state", tl_phase_str(ph));
        cJSON_AddNumberToObject(light, "remain_s", e.phase_timer);
        cJSON_AddItemToArray(lights, light);
    }
    cJSON_AddItemToObject(root, "lights", lights);
    char* s = cJSON_PrintUnformatted(root);
    transport_publish(g.transport, "road/traffic_lights",
                      (const uint8_t*)s, (uint32_t)strlen(s) + 1);
    free(s);
    cJSON_Delete(root);
}

static void publish_sim_tick(uint64_t sim_time_us) {
    cJSON* tick = cJSON_CreateObject();
    cJSON_AddNumberToObject(tick, "t_us", (double)sim_time_us);
    cJSON_AddNumberToObject(tick, "cycle", g.cycle);
    char* s = cJSON_PrintUnformatted(tick);
    transport_publish(g.transport, "sim/tick",
                      (const uint8_t*)s, (uint32_t)strlen(s) + 1);
    free(s);
    cJSON_Delete(tick);
}

static void publish_sim_collision(const flowsim::Entity& a, const flowsim::Entity& b) {
    cJSON* col = cJSON_CreateObject();
    cJSON_AddNumberToObject(col, "ego_x", a.x);
    cJSON_AddNumberToObject(col, "ego_y", a.y);
    cJSON_AddNumberToObject(col, "obs_id", b.id);
    cJSON_AddNumberToObject(col, "overlap_x",
        (a.length + b.length) * 0.5 - fabs(a.x - b.x));
    cJSON_AddNumberToObject(col, "overlap_y",
        (a.width + b.width) * 0.5 - fabs(a.y - b.y));
    char* s = cJSON_PrintUnformatted(col);
    transport_publish(g.transport, "sim/collision",
                      (const uint8_t*)s, (uint32_t)strlen(s) + 1);
    free(s);
    cJSON_Delete(col);
}

/* ── 内置巡航（无 control/cmd 时的 ego 闭环） ─────────────────── */
/* 与 sim_world_node.c 的内置巡航一致：简单 P 控制追 target_speed + 车道保持。 */

static void internal_cruise_control(flowsim::Entity& ego) {
    /* 纵向：P 控制追 target_vx */
    double err = ego.target_vx - ego.speed;
    if (err > 0.1) {
        ego.throttle = 0.5;
        ego.brake = 0.0;
    } else if (err < -0.1) {
        ego.throttle = 0.0;
        ego.brake = 0.3;
    } else {
        ego.throttle = 0.1;
        ego.brake = 0.0;
    }
    /* 横向：车道保持 — 朝道路中心线 + 目标车道偏移 */
    double target_y = road_center_y(ego.x, g.curve_start_x, g.curve_length_m, g.curve_offset_m)
                      + (-g.lane_width * 0.5);  /* 默认左车道中心 */
    double y_err = target_y - ego.y;
    /* 用道路切线航向做前馈 + 横向偏差 P 反馈 */
    double road_h = road_center_heading(ego.x, g.curve_start_x, g.curve_length_m, g.curve_offset_m);
    ego.steer = road_h - ego.heading + y_err * 0.05;
    if (ego.steer > 0.25) ego.steer = 0.25;
    if (ego.steer < -0.25) ego.steer = -0.25;
}

/* ── 协程主循环 ───────────────────────────────────────────────── */

class FlowSimTask : public FlowCoroTask {
public:
    FlowSimTask(MessageBus* bus, Transport* transport)
        : FlowCoroTask(bus), transport_(transport) {}

protected:
    Task run() override {
        LOG_INFO("flowsim", "FlowCoro flowsim started (20Hz, roads=%s)",
                 g.roads_loaded ? "esmini" : "fallback");

        flowsim::Entity& ego = g.pool[0];

        while (!should_stop()) {
            /* 50ms tick：select_for 等待 control/cmd 或超时（20Hz 节拍）。
             * stop() 触发 cancel_token 立即唤醒，无需外发消息。 */
            auto res = co_await select_for({"control/cmd"}, FLOWSIM_DT_US);
            if (should_stop()) break;

            /* 直接从 select_for 返回的消息中解析控制指令，不再依赖
             * transport 层回调更新 atomics（两者订阅同一个 bus，但
             * 回调触发时序不可靠，这里直接用消息体更鲁棒）。 */
            bool use_internal_cruise = true;
            if (res.ok() && res->data_size > 0) {
                /* 二进制 ControlCmd 路径 */
                ControlCmd bin;
                if (ControlCmd_deserialize(&bin,
                        (const uint8_t*)res->data, res->data_size) == 0) {
                    g.ego_throttle.store(bin.throttle, std::memory_order_relaxed);
                    g.ego_brake.store(bin.brake, std::memory_order_relaxed);
                    g.ego_steer.store(bin.steering, std::memory_order_relaxed);
                    g.last_control_cmd_us.store(clock_now_us(),
                        std::memory_order_relaxed);
                    use_internal_cruise = false;
                } else {
                    /* JSON fallback */
                    cJSON* root = cJSON_Parse((const char*)res->data);
                    if (root) {
                        cJSON* j;
                        if ((j = cJSON_GetObjectItemCaseSensitive(root, "throttle"))
                            && cJSON_IsNumber(j))
                            g.ego_throttle.store(j->valuedouble,
                                std::memory_order_relaxed);
                        if ((j = cJSON_GetObjectItemCaseSensitive(root, "brake"))
                            && cJSON_IsNumber(j))
                            g.ego_brake.store(j->valuedouble,
                                std::memory_order_relaxed);
                        if ((j = cJSON_GetObjectItemCaseSensitive(root, "steer"))
                            && cJSON_IsNumber(j))
                            g.ego_steer.store(j->valuedouble,
                                std::memory_order_relaxed);
                        cJSON_Delete(root);
                        g.last_control_cmd_us.store(clock_now_us(),
                            std::memory_order_relaxed);
                        use_internal_cruise = false;
                    }
                }
            }

            /* control/cmd 陈旧检查：超时且无近期控制指令 → 内置巡航 */
            if (use_internal_cruise) {
                uint64_t now = clock_now_us();
                uint64_t last = g.last_control_cmd_us.load(
                    std::memory_order_relaxed);
                if (last > 0 && now > last
                    && now - last < CONTROL_STALE_TIMEOUT_US) {
                    /* 消息在 select_for 订阅间隙到达，但 transport 回调
                     * 已更新 atomics — 仍可信且可用。 */
                    use_internal_cruise = false;
                }
            }
            if (use_internal_cruise && g.last_control_cmd_us.load(
                    std::memory_order_relaxed) == 0) {
                ; /* 冷启动：尚无任何控制指令，用内置巡航起步 */
            }

            /* ── Step 1: ego 动力学 ── */
            if (use_internal_cruise) {
                internal_cruise_control(ego);
            } else {
                ego.throttle = g.ego_throttle.load(std::memory_order_relaxed);
                ego.brake    = g.ego_brake.load(std::memory_order_relaxed);
                ego.steer    = g.ego_steer.load(std::memory_order_relaxed);
            }
            flowsim::step_bicycle(ego, FLOWSIM_DT_SEC,
                                  ego.throttle, ego.brake, ego.steer);

            /* ── Step 2: 场景事件预检查（让 NPC 知道前方红绿灯/ETC） ── */
            flowsim::check_npc_scene_events(g.pool, g.ai_cfg.look_ahead);

            /* ── Step 3: NPC AI ── */
            flowsim::FlowRoadNetwork* roads_ptr = g.roads_loaded ? &g.roads : nullptr;
            for (int i = 1; i < g.pool.size(); i++) {
                flowsim::Entity& e = g.pool[i];
                if (!e.active) continue;
                if (e.is_npc_vehicle()) {
                    flowsim::step_npc_vehicle(e, g.pool, FLOWSIM_DT_SEC,
                                              g.ai_cfg, roads_ptr);
                } else if (e.type == flowsim::EntityType::Pedestrian) {
                    flowsim::step_npc_pedestrian(e, FLOWSIM_DT_SEC, g.ai_cfg);
                }
            }

            /* ── Step 4: 碰撞检测 ── */
            std::vector<flowsim::CollisionPair> pairs;
            int n_col = flowsim::detect_collisions(g.pool, pairs);
            if (n_col > 0) {
                flowsim::apply_collision_response(g.pool, pairs);
                for (const auto& p : pairs) {
                    int is_ego = (p.a == 0 || p.b == 0);
                    if (is_ego) {
                        LOG_ERROR("flowsim", "COLLISION ego↔entity%d", p.a == 0 ? p.b : p.a);
                    }
                    /* 仅发布涉及 ego 的碰撞事件，与旧 sim_world 行为一致；
                     * NPC 间 AABB 重叠不产生 topic 消息，避免 evaluator 误判。 */
                    if (is_ego) publish_sim_collision(g.pool[p.a], g.pool[p.b]);
                }
            }

            /* ── Step 5: 场景事件推进 ── */
            double sim_time_s = (double)(clock_now_us() - g.sim_start_us) / 1e6;
            flowsim::tick_traffic_lights(g.pool, sim_time_s);
            flowsim::tick_etc_gates(g.pool, ego, FLOWSIM_DT_SEC);

            /* ── Step 6: 推进逻辑时钟 ── */
            clock_advance_us(FLOWSIM_DT_US);
            uint64_t sim_time_us = clock_now_us();

            /* ── Step 7: 发布 ── */
            publish_sim_tick(sim_time_us);
            if (g.cycle % ROAD_GEOMETRY_REPUBLISH_CYCLES == 0) {
                publish_road_geometry();
            }
            publish_traffic_lights();
            publish_vehicle_state(sim_time_us);
            /* scene/frame：完整场景帧 20Hz 给 3D 前端（Phase 2.2） */
            flowsim::publish_scene_frame(g.transport, g.pool, g.scene_pub_cfg,
                                         sim_time_us, g.cycle);

            g.cycle++;
            if (g.cycle % 100 == 0) {
                LOG_INFO("flowsim", "#%u ego(%.1f,%.1f) spd=%.1f npc=%d",
                         g.cycle, ego.x, ego.y, ego.speed,
                         g.pool.active_count() - 1);
            }
        }

        LOG_INFO("flowsim", "stopped (%u cycles, sim_time=%.3fs, final speed=%.1f)",
                 g.cycle, (double)clock_now_us() / 1e6, ego.speed);
    }

private:
    Transport* transport_;
};

/* ── 协程宿主线程 ─────────────────────────────────────────────── */

void* flowsim_thread(void*) {
    pthread_setname_np(pthread_self(), "flowsim");
    try {
        g.task->execute();
    } catch (...) {
        LOG_ERROR("flowsim", "FlowCoro task failed");
    }
    return nullptr;
}

/* ── NodePlugin 实现 ─────────────────────────────────────────── */

static const char* s_inputs[]  = { "control/cmd", nullptr };
static const char* s_outputs[] = {
    "vehicle/state", "road/geometry", "road/traffic_lights",
    "sim/tick", "sim/collision", "scene/frame", nullptr
};

extern NodePlugin s_plugin;

static int flowsim_init(MessageBus* bus, Transport* transport,
                        DiscoveryManager* discovery, Scheduler* scheduler,
                        const char* params_json) {
    g.transport  = transport;
    g.discovery  = discovery;
    g.scheduler  = scheduler;
    g.should_stop = false;
    g.running    = false;
    g.cycle      = 0;

    /* 默认 AI 配置（与 Phase 1 测试一致） */
    g.ai_cfg.lane_width = 3.5;
    g.ai_cfg.same_lane_tol = 2.0;
    g.ai_cfg.look_ahead = 80.0;

    /* 解析 params_json */
    if (params_json) {
        cJSON* p = cJSON_Parse(params_json);
        if (p) {
            cJSON* j;
            if ((j = cJSON_GetObjectItemCaseSensitive(p, "init_speed")) && cJSON_IsNumber(j))
                g.init_speed = j->valuedouble;
            if ((j = cJSON_GetObjectItemCaseSensitive(p, "target_speed")) && cJSON_IsNumber(j))
                g.target_speed = j->valuedouble;
            if ((j = cJSON_GetObjectItemCaseSensitive(p, "lane_width")) && cJSON_IsNumber(j)) {
                g.lane_width = j->valuedouble;
                g.ai_cfg.lane_width = g.lane_width;
            }
            if ((j = cJSON_GetObjectItemCaseSensitive(p, "random_seed")) && cJSON_IsNumber(j))
                g.random_seed = (uint32_t)j->valuedouble;
            if ((j = cJSON_GetObjectItemCaseSensitive(p, "scenario_file")) && cJSON_IsString(j))
                strncpy(g.scenario_file, j->valuestring, sizeof(g.scenario_file) - 1);
            cJSON_Delete(p);
        }
    }

    /* 加载场景文件 */
    if (g.scenario_file[0] != '\0') {
        g.scenario = scenario_load(g.scenario_file);
    }
    if (!g.scenario) {
        LOG_WARN("flowsim", "scenario_load failed for '%s' — using defaults",
                 g.scenario_file[0] ? g.scenario_file : "(none)");
        /* 分配一个空场景，ego 用默认值 */
        g.scenario = (ScenarioConfig*)calloc(1, sizeof(ScenarioConfig));
        g.scenario->ego.x = 0.0;
        g.scenario->ego.y = -1.75;
        g.scenario->ego.init_speed = g.init_speed;
        g.scenario->ego.target_speed = g.target_speed;
    }

    /* 场景参数覆盖 */
    g.random_seed = g.scenario->random_seed ? g.scenario->random_seed : g.random_seed;
    g.curve_start_x  = g.scenario->road.curve_start_x;
    g.curve_length_m = g.scenario->road.curve_length_m;
    g.curve_offset_m = g.scenario->road.curve_offset_m;
    if (g.scenario->ego.init_speed > 0)   g.init_speed   = g.scenario->ego.init_speed;
    if (g.scenario->ego.target_speed > 0) g.target_speed = g.scenario->ego.target_speed;
    srand(g.random_seed);

    /* JSON → xodr → esmini RoadManager */
    if (g.scenario_file[0] != '\0') {
        std::string xodr = convert_scenario_to_xodr(g.scenario_file);
        if (!xodr.empty()) {
            if (g.roads.load(xodr)) {
                g.roads_loaded = true;
                LOG_INFO("flowsim", "esmini road network loaded: %d roads",
                         g.roads.road_count());
            } else {
                LOG_WARN("flowsim", "esmini load failed for %s — NPC AI falls back to lateral distance",
                         xodr.c_str());
            }
        }
    }

    /* 填充 EntityPool */
    populate_entities_from_scenario(g.scenario);

    /* 仿真时钟：逻辑时间从 0 开始步进 */
    clock_set_sim_mode(true);
    clock_set_sim_time(0);
    clock_set_step_us(FLOWSIM_DT_US);
    g.sim_start_us = 0;  /* sim 时间从 0 起，sim_start_us=0 使 sim_time_s = clock_now_us/1e6 */

    /* 订阅 control/cmd */
    transport_subscribe(transport, "control/cmd", on_control_cmd, nullptr);
    discovery_advertise(discovery, "control/cmd", CONTROL_CMD_TYPE_ID, CAP_SUBSCRIBER, 0);

    /* 广告输出 topics */
    transport_advertise(transport, "vehicle/state",       VEHICLE_STATE_TYPE_ID);
    discovery_advertise(discovery, "vehicle/state",       VEHICLE_STATE_TYPE_ID, CAP_PUBLISHER, 20.0);
    transport_advertise(transport, "road/geometry",       ROAD_GEOMETRY_TYPE_ID);
    discovery_advertise(discovery, "road/geometry",       ROAD_GEOMETRY_TYPE_ID, CAP_PUBLISHER, 1.0);
    transport_advertise(transport, "road/traffic_lights", ROAD_TRAFFIC_LIGHTS_TYPE_ID);
    transport_advertise(transport, "sim/tick",            SIM_TICK_TYPE_ID);
    transport_advertise(transport, "sim/collision",       SIM_COLLISION_TYPE_ID);
    /* scene/frame：20Hz 完整场景帧，给 3D 前端用（Phase 2.2 新增） */
    transport_advertise(transport, "scene/frame",         SCENE_FRAME_TYPE_ID);
    discovery_advertise(discovery, "scene/frame",         SCENE_FRAME_TYPE_ID, CAP_PUBLISHER, 20.0);

    /* 填充 scene_pub_cfg：roads_loaded 之后才有 esmini 网络指针 */
    g.scene_pub_cfg.curve_start_x  = g.curve_start_x;
    g.scene_pub_cfg.curve_length_m = g.curve_length_m;
    g.scene_pub_cfg.curve_offset_m = g.curve_offset_m;
    g.scene_pub_cfg.lane_width     = g.lane_width;
    g.scene_pub_cfg.lane_count     = 2;
    g.scene_pub_cfg.roads          = g.roads_loaded ? &g.roads : nullptr;
    g.scene_pub_cfg.type_id        = SCENE_FRAME_TYPE_ID;

    /* 构造协程任务 */
    g.task = std::make_unique<FlowSimTask>(bus, transport);

    LOG_INFO("flowsim", "initialized (scenario=%s, actors=%d, traffic_lights=%d, esmini=%s)",
             g.scenario_file[0] ? g.scenario_file : "(default)",
             g.scenario->actor_count,
             g.scenario->traffic_light_count,
             g.roads_loaded ? "on" : "off");
    return 0;
}

static int flowsim_start(void) {
    if (!g.task) return -1;
    g.should_stop = false;
    if (pthread_create(&g.thread, nullptr, flowsim_thread, nullptr) != 0) {
        LOG_WARN("flowsim", "failed to create thread");
        return -1;
    }
    g.running = true;
    LOG_INFO("flowsim", "started");
    node_announce_self(g.transport, &s_plugin);
    return 0;
}

static void flowsim_stop(void) {
    g.should_stop = true;
    if (g.task) g.task->stop();  /* 触发 CancelToken 唤醒挂起的 select_for */
}

static void flowsim_cleanup(void) {
    flowsim_stop();
    if (g.running) {
        pthread_join(g.thread, nullptr);
        g.running = false;
    }
    g.task.reset();
    g.roads.close();
    g.roads_loaded = false;
    if (g.scenario) {
        scenario_free(g.scenario);
        g.scenario = nullptr;
    }
    g.pool.clear();
    LOG_INFO("flowsim", "cleanup done");
}

static int flowsim_health(void) {
    return (g.cycle > 0) ? 0 : 1;
}

/* ── 导出入口 ────────────────────────────────────────────────── */

NodePlugin s_plugin = {
    NODE_PLUGIN_API_VERSION,
    "flowsim",
    "2.0.0",
    "FlowSim v2 simulation world (C++ flowcoro + esmini RoadManager)",
    s_inputs,
    s_outputs,
    flowsim_init,
    flowsim_start,
    flowsim_stop,
    flowsim_cleanup,
    flowsim_health,
};

} // namespace

extern "C" NodePlugin* node_get_plugin(void) { return &s_plugin; }
