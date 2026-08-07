/**
 * flowsim_node.cpp — FlowSim v2 仿真世界节点 (C++20 + flowcoro + esmini)
 *
 * 替换 sim_world_node.c。集成 Phase 1 组件（road_network / entity / physics /
 * npc_ai / collision / scene_events）为一个完整的 60Hz 仿真主循环节点。
 *
 * 架构（见 docs/FLOWSIM_ARCHITECTURE.md）：
 *   - esmini RoadManager 处理道路网络（Frenet↔World / 限速 / 车道）
 *   - Entity System 固定池管理 ego + NPC + 事件触发器
 *   - 自行车模型积分 ego 动力学
 *   - IDM 跟车 + 状态机驱动 NPC
 *   - OBB SAT 碰撞检测
 *   - 红绿灯/ETC 场景事件调度
 *   - flowcoro 协程主循环（CoroutineTask，线程池 resume）
 *
 * 向后兼容（与 sim_world_node.c 的 topic 契约完全一致）：
 *   订阅: control/cmd（二进制 ControlCmd + JSON fallback）
 *   发布: vehicle/state, road/geometry, road/traffic_lights, sim/tick, sim/collision
 *
 * 采用 CoroutineTask（线程池 resume）：物理积分 + NPC AI + 碰撞 + 事件 + JSON
 * 序列化单次 ~50-200μs，同步 resume 会阻塞消息总线分发线程，故改用线程池 resume。
 */

#include "node_plugin.h"
#include "topic_registry.h"
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
#include "flowsim/actor/VehicleActor.h"   /* 车灯信号派生（Phase 1 重构）*/
#include "flowsim/route.h"
#include "flowsim/sim_digest.h"
#include "flowsim/lane_frenet.h"          /* C-2: 共享车道中心横向偏移公式 */

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <time.h>
#include <unistd.h>
#include <sys/stat.h>

#include <string>
#include <vector>
#include <atomic>

namespace {

/* ── 仿真常量（与 sim_world_node.c 一致，保证下游节点兼容） ───── */

#define FLOWSIM_FREQUENCY_HZ   60.0
#define FLOWSIM_DT_SEC         (1.0 / FLOWSIM_FREQUENCY_HZ)   /* ~0.0167s */
#define FLOWSIM_DT_US          ((uint64_t)(FLOWSIM_DT_SEC * 1e6))  /* 16666 */

/* control/cmd 陈旧超时：2000ms 未收到则回退 FSAFE 停车。
 * 原 500ms 在高负载（15 tasks, ~34% 丢包率）下过于激进：
 * 消息总线偶尔拥塞导致 2-3 帧连续丢失 → 500ms 内无新消息 → FSAFE 误触发
 * → brake=1.0 与间歇到达的 throttle 形成"走/停"振荡 → 车速恒为 0
 * （2026-08-03 事故链）。改为 2000ms（120 帧 @60Hz），给传输层恢复窗口。 */
#define CONTROL_STALE_TIMEOUT_US  2000000ULL

/* road/geometry 周期性重发：150 cycle = 2.5s @ 60Hz。
 * 另：路段/车道数变化时会立即补发，避免 planning/behavior 在新 road 上继续用旧 lane_count。 */
#define ROAD_GEOMETRY_REPUBLISH_CYCLES 150

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

    /* TaskBase 包装器（由 EXPORT_COROUTINE_TASK 宏创建） */
    struct flowsim_Wrapper* task_wrapper{nullptr};

    /* Phase 1 组件 */
    flowsim::FlowRoadNetwork  roads;
    flowsim::EntityPool       pool;
    flowsim::NpcAiConfig      ai_cfg;
    flowsim::Route            route;         // 中央有序 route（NPC 车道跟随，见 route.h）

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

    /* 物理模型选择："kinematic"（默认）| "dynamic"（预留，未实现） */
    char              physics_model[32]{"kinematic"};

    /* control/cmd 状态 */
    std::atomic<int>  has_control_input{0};
    std::atomic<uint64_t> last_control_cmd_us{0};
    /* ego 当前控制量（on_control_cmd 写，tick 读；单写单读，无需锁） */
    std::atomic<double> ego_throttle{0};
    std::atomic<double> ego_brake{0};
    std::atomic<double> ego_steer{0};
    /* 灯光指令（control_node 决策下发，意图先行） */
    std::atomic<uint8_t> ego_turn_signal{0};
    std::atomic<bool>    ego_hazard{false};
    /* 档位（control_node→safety_control→flowsim 透传，DRIVE=1/REVERSE=-1） */
    std::atomic<int8_t>  ego_gear{GEAR_DRIVE};

    /* 统计 */
    uint32_t          cycle{0};
    uint64_t          sim_start_us{0};
    bool              roads_loaded{false};
    int               last_road_geom_road_id{-9999};
    int               last_road_geom_lane_count{-1};

    /* 仿真基础层：静态 digest（几何变更时建一次）+ 上一帧动态 digest（时序 invariant） */
    flowsim::StaticDigest  static_digest;
    flowsim::DynamicDigest prev_dynamic_digest;
    bool                   digest_initialized{false};

    /* P2-7: invariant 失败计数。cleanup 时打印汇总 marker，
     * demo_evaluator.py 扫描此 marker 把 invariant 失败升级为 FAIL。
     * 旧行为只 LOG_WARN，不影响退出码也不被 evaluator 捕获 → 同类回归漏检。 */
    std::atomic<uint32_t>  invariant_fail_count{0};
    uint32_t               ego_maneuver_grace_until{0};

    /* P2-8: 物理掉头标志。当车辆在对向车道（lane_id > 0）时为 true，
     * 此时 esmini advance 方向与车辆行驶方向相反，需要用 world_to_frenet
     * 同步 esmini position。由 lane_id 符号自动推导，不再由硬编码状态机设置。 */
    bool                   u_turn_active{false};

    /* 机动模式（off-rails）：掉头/倒车期间自行车模型是位姿唯一权威。
     * 进入：control 命令 |steer|>0.28（超巡航钳位域，只能是掉头轨迹）或倒挡。
     * 退出：转向需求消失 且 车头与车道方向夹角 < 20°（对齐后才重新上轨）。
     * 期间跳过 Frenet 轨道位置覆盖 —— 轨道覆盖是纯"沿车头平移"，丢掉了
     * step_bicycle 的 half_wb·yaw_rate 旋转项，大角速度时车绕自身中心
     * 原地旋转（"屁股横扫"视觉伪影）。 */
    bool                   off_rails{false};

    /* 进程级缓存，重复初始化时必须清零。 */
    double                prev_steer{0.0};
    uint64_t              bridge_last_cb_check{0};
    uint64_t              bridge_prev_cb_count{0};
    uint64_t              bridge_last_reconnect_us{0};  /* 桥重连防抖时间戳（10s 窗口） */
    };

FlowSimContext g;

static void reset_runtime_state() {
    if (g.scenario) {
        scenario_free(g.scenario);
        g.scenario = nullptr;
    }

    g.roads.release();
    g.pool.clear();
    g.route = flowsim::Route();
    g.scene_pub_cfg = flowsim::ScenePubConfig();
    g.scene_pub_cfg.lane_count = 2;
    g.scene_pub_cfg.lane_width = 3.5;
    g.scene_pub_cfg.cached_road_network_json.clear();
    g.scene_pub_cfg.roads = nullptr;
    g.scene_pub_cfg.construction_zones.clear();

    g.has_control_input.store(0, std::memory_order_relaxed);
    g.last_control_cmd_us.store(0, std::memory_order_relaxed);
    g.ego_throttle.store(0.0, std::memory_order_relaxed);
    g.ego_brake.store(0.0, std::memory_order_relaxed);
    g.ego_steer.store(0.0, std::memory_order_relaxed);
    g.ego_turn_signal.store(0, std::memory_order_relaxed);
    g.ego_hazard.store(false, std::memory_order_relaxed);
    g.ego_gear.store(GEAR_DRIVE, std::memory_order_relaxed);

    g.cycle = 0;
    g.sim_start_us = 0;
    g.roads_loaded = false;
    g.last_road_geom_road_id = -9999;
    g.last_road_geom_lane_count = -1;
    g.static_digest = flowsim::StaticDigest();
    g.prev_dynamic_digest = flowsim::DynamicDigest();
    g.digest_initialized = false;
    g.invariant_fail_count.store(0, std::memory_order_relaxed);
    g.u_turn_active = false;
    g.off_rails = false;
    g.ego_maneuver_grace_until = 0;
    g.prev_steer = 0.0;
    g.bridge_last_cb_check = 0;
    g.bridge_prev_cb_count = 0;
    g.bridge_last_reconnect_us = 0;

    g.scenario_file[0] = '\0';
    std::snprintf(g.physics_model, sizeof(g.physics_model), "%s", "kinematic");
    g.init_speed = 5.0;
    g.target_speed = 12.0;
    g.lane_width = 3.5;
    g.random_seed = 42;
    g.curve_start_x = 0.0;
    g.curve_length_m = 0.0;
    g.curve_offset_m = 0.0;

    flowsim::reset_choreography_state();
}

/* ── control/cmd 订阅回调 ─────────────────────────────────────── */
/* 与 sim_world_node.c on_control_cmd 行为一致：先试二进制 ControlCmd，
 * 失败则 JSON fallback。解析结果写入 g.ego_throttle/brake/steer。 */

static void on_control_cmd(const Message* msg, void* user_data) {
    (void)user_data;
    if (!msg || msg->data_size == 0) return;
    /* 游戏模式（demo.sh --game）：玩家键盘操控，忽略自动驾驶指令
     * （control_node 会覆盖游戏输入 → 车"自己走"）。 */
    if (access("/tmp/game_mode", F_OK) == 0) return;

    /* 二进制 ControlCmd 路径 */
    {
        ControlCmd bin;
        if (ControlCmd_deserialize(&bin, (const uint8_t*)msg->data, msg->data_size) == 0) {
            g.ego_throttle.store(bin.throttle, std::memory_order_relaxed);
            g.ego_brake.store(bin.brake, std::memory_order_relaxed);
            g.ego_steer.store(bin.steering, std::memory_order_relaxed);
            g.ego_turn_signal.store(bin.turn_signal, std::memory_order_relaxed);
            g.ego_hazard.store(bin.hazard, std::memory_order_relaxed);
            g.ego_gear.store(bin.gear, std::memory_order_relaxed);
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

        /* 构造命令：python3 <tool> <scenario> -o <xodr>
         * stderr 落到 /tmp/flowsim_xodr_err.log，失败时读回并打印根因
         * （直接 >/dev/null 会把 python traceback 吞掉，无从排查）。 */
        const char* err_log = "/tmp/flowsim_xodr_err.log";
        std::string cmd = "python3 \"";
        cmd += tool;
        cmd += "\" \"";
        cmd += scenario_path;
        cmd += "\" -o \"";
        cmd += xodr_path;
        cmd += "\" >/dev/null 2>\"";
        cmd += err_log;
        cmd += "\"";
        int rc = system(cmd.c_str());
        if (rc == 0 && stat(xodr_path.c_str(), &st) == 0 && st.st_size > 0) {
            LOG_INFO("flowsim", "json_to_xodr: %s → %s (via %s)",
                     scenario_path.c_str(), xodr_path.c_str(), tool.c_str());
            return xodr_path;
        }
        LOG_WARN("flowsim", "json_to_xodr failed (rc=%d) via %s", rc, tool.c_str());
        if (rc != 0) {
            FILE* ef = std::fopen(err_log, "r");
            if (ef) {
                char line[256];
                while (std::fgets(line, sizeof line, ef)) {
                    LOG_WARN("flowsim", "json_to_xodr: %s", line); /* 含换行，仅作诊断 */
                }
                std::fclose(ef);
            }
        }
    }

    LOG_WARN("flowsim", "json_to_xodr: no working tool found, esmini road network disabled");
    return "";
}

/* ── 从场景配置填充 EntityPool ─────────────────────────────────── */

/**
 * 计算 (x, y) 处的道路切线航向角。
 * - esmini 模式：world_to_frenet → frenet_to_world 获取准确航向
 * - legacy 模式：road_center_heading() 解析计算
 */
static double compute_road_heading_at(double x, double y) {
    if (g.roads_loaded) {
        flowsim::FrenetPos fp;
        if (g.roads.world_to_frenet(x, y, fp)) {
            flowsim::WorldPos wp;
            if (g.roads.frenet_to_world(fp.road_id, 0, fp.s, 0.0, wp)) {
                return wp.h;
            }
        }
    }
    return road_center_heading(x, g.curve_start_x, g.curve_length_m, g.curve_offset_m);
}

static flowsim::EntityType actor_type_to_entity(const char* type) {
    if (!type) return flowsim::EntityType::Car;
    if (strcmp(type, "pedestrian") == 0) return flowsim::EntityType::Pedestrian;
    if (strcmp(type, "truck") == 0)      return flowsim::EntityType::Truck;
    if (strcmp(type, "suv") == 0)        return flowsim::EntityType::SUV;
    return flowsim::EntityType::Car;
}

/* ── 工况脚本（Task 3）：触发器评估 + actor_overrides 应用 ──
 *
 * 每个仿真 tick 在 NPC AI 之前调用。对每个未触发的脚本评估 trigger：
 *   ego_x_gte   → ego.x ≥ value
 *   ego_x_lte   → ego.x ≤ value
 *   time_gte    → sim_time_s ≥ value
 *   route_s_gte → ego route 累计 s ≥ value
 * 触发后把 actor_overrides 应用到 pool 中 actor_id 匹配的实体（一次性，fired=true）。
 *
 * ai_state 字符串 → AIState 枚举映射（与 scene_pub.cpp::ai_state_str 反向）。
 */
static flowsim::NpcState ai_state_from_str(const char* s) {
    if (!s || !s[0]) return flowsim::NpcState::Cruise;
    if (strcmp(s, "follow") == 0)       return flowsim::NpcState::Follow;
    if (strcmp(s, "stop") == 0)         return flowsim::NpcState::Stopped;
    if (strcmp(s, "stop_for_tl") == 0)  return flowsim::NpcState::StopForTL;
    if (strcmp(s, "etc_approach") == 0) return flowsim::NpcState::Yield;      /* ETC 接近 → 让行减速 */
    if (strcmp(s, "branch_sel") == 0)   return flowsim::NpcState::Cruise;     /* 路口选路由 road_pos 处理 */
    if (strcmp(s, "merge") == 0)        return flowsim::NpcState::Cruise;     /* 汇入由 road_pos 处理 */
    if (strcmp(s, "yield") == 0)        return flowsim::NpcState::Yield;
    if (strcmp(s, "cutin") == 0)        return flowsim::NpcState::CutIn;
    return flowsim::NpcState::Cruise;
}

static flowsim::Entity* find_entity_by_actor_id(int actor_id) {
    for (int i = 0; i < g.pool.size(); ++i) {
        flowsim::Entity& e = g.pool[i];
        if (e.active && e.scenario_id == actor_id) return &e;
    }
    return nullptr;
}

static void apply_actor_override(flowsim::Entity& e,
                                 const ScenarioActorOverride* o) {
    /* P1.3 修复：状态转移改走统一入口 npc_request_state。
     *
     * 原实现直接 `e.state = ai_state_from_str(o->ai_state)` 绕过状态机，
     * 并手动初始化 cutin_pid_integral / cutin_pid_prev / cutin_active，
     * 与 npc_request_state 内部清理逻辑重复——任何后续在 npc_request_state
     * 新增的状态相关字段重置（LaneChange 冷却、Stopped 立即刹车等）都不会
     * 被脚本 override 路径覆盖到，导致脚本触发的 CutIn/Stop 与状态机自动
     * 触发的同类状态行为不一致。
     *
     * 修复：用 NpcEvent::ScriptSet + req.target_state 把脚本请求的目标状态
     * 喂回统一入口；target_offset / target_vx / vx / vy 也通过 req 字段下发，
     * 由 npc_request_state 末尾"应用请求覆盖字段"段统一写入。
     * ScenarioActorOverride 用 NaN 标记未设置，NpcTransitionRequest 用
     * NPC_REQ_UNSET，这里做转换。无 ai_state 时用当前 state 作占位，
     * target==old → 状态不变，仅应用覆盖字段。 */
    flowsim::NpcTransitionRequest req;
    req.event        = flowsim::NpcEvent::ScriptSet;
    req.target_state = o->ai_state[0] ? ai_state_from_str(o->ai_state) : e.state;
    if (!isnan(o->target_offset)) req.target_offset = o->target_offset;
    if (!isnan(o->target_vx))      req.target_vx     = o->target_vx;
    if (!isnan(o->vx))             req.vx            = o->vx;
    if (!isnan(o->vy))             req.vy            = o->vy;
    flowsim::npc_request_state(e, req, g.ai_cfg);
}

/* Compute ego's cumulative s along the central route.
 *
 * 性能优化：ego.road_pos 是本地 esmini position handle，frenet() 查询是 O(1)
 * 本地操作；而 world_to_frenet 是 O(roads) 全网扫描（遍历所有 road 找最近）。
 * 主循环每帧原本调用 world_to_frenet 3 次（apply_scenario_scripts /
 * publish_ref_path / Step 3 NPC AI），60Hz 下冗余全网扫描。
 *
 * 此 helper 优先用 ego.road_pos.frenet()（road_pos.ok() 时），仅在 road_pos
 * 未初始化或查询失败时回退到 world_to_frenet。返回 -1.0 表示 ego 不在 route 上。 */
static double compute_ego_route_s() {
    if (!g.route.ok() || !g.roads_loaded) return -1.0;
    const flowsim::Entity& ego = g.pool[0];
    flowsim::FrenetPos ef;
    bool ok = ego.road_pos.ok() ? ego.road_pos.frenet(ef)
                                 : g.roads.world_to_frenet(ego.x, ego.y, ef);
    if (!ok) return -1.0;
    int ei = g.route.index_of(ef.road_id);
    return (ei >= 0) ? g.route.to_route_s(ei, ef.s) : -1.0;
}

static void apply_scenario_scripts(double sim_time_s) {
    if (!g.scenario || g.scenario->script_count <= 0) return;
    const flowsim::Entity& ego = g.pool[0];
    /* 预算 ego route_s（route_s_gte 触发器用） */
    double ego_route_s = compute_ego_route_s();
    /* Phase 2: ego.road_pos.ok() 时也用 road_pos.s() 作为触发条件的 OR 来源。
     * route_s 是沿中央 route 的累计 s（跨 road 段），road_pos.s() 是当前 road
     * 内的局部 s；二者度量的不是同一个量，但 route_s_gte 触发器一般是粗粒度
     * 「ego 已驶过 X 米」判断，取 max(route_s, road_pos.s()) 让任一来源满足即触发，
     * 避免旧 route 在 fork 路段失真时触发器永远不 fire。 */
    double ego_pos_s = -1.0;
    if (ego.road_pos.ok()) {
        ego_pos_s = ego.road_pos.s();
    }
    for (int i = 0; i < g.scenario->script_count; ++i) {
        ScenarioScript* s = &g.scenario->scripts[i];
        if (s->fired) continue;
        bool fire = false;
        switch (s->trigger.type) {
            case SCRIPT_TRIGGER_EGO_X_GTE:
                fire = (ego.x >= s->trigger.value); break;
            case SCRIPT_TRIGGER_EGO_X_LTE:
                fire = (ego.x <= s->trigger.value); break;
            case SCRIPT_TRIGGER_TIME_GTE:
                fire = (sim_time_s >= s->trigger.value); break;
            case SCRIPT_TRIGGER_ROUTE_S_GTE:
                /* route_s 或 road_pos.s() 任一 ≥ value 即触发（OR 语义） */
                if (ego_route_s >= 0.0 && ego_route_s >= s->trigger.value) {
                    fire = true;
                } else if (ego_pos_s >= 0.0 && ego_pos_s >= s->trigger.value) {
                    fire = true;
                }
                break;
        }
        if (!fire) continue;
        s->fired = true;
        LOG_INFO("flowsim", "scenario script '%s' fired (trigger type=%d val=%.2f)",
                 s->name, (int)s->trigger.type, s->trigger.value);
        for (int k = 0; k < s->override_count; ++k) {
            const ScenarioActorOverride* o = &s->overrides[k];
            flowsim::Entity* e = find_entity_by_actor_id(o->actor_id);
            if (!e) {
                LOG_WARN("flowsim", "scenario '%s' override actor id=%d not found in pool",
                         s->name, o->actor_id);
                continue;
            }
            apply_actor_override(*e, o);
            LOG_INFO("flowsim", "  override actor id=%d ai_state='%s' target_offset=%.2f target_vx=%.2f",
                     o->actor_id, o->ai_state, o->target_offset, o->target_vx);
        }
    }
}

static void populate_entities_from_scenario(const ScenarioConfig* sc) {
    g.pool.clear();

    /* Ego 固定 index 0。
     * ScenarioEgo 结构体无 id 字段（ego 无场景业务 id），scenario_id 保留
     * 默认 0 —— 与 pool 索引一致，且 actors(id=1..N)/tls/etc_gates 不会
     * 用 0 与 ego 冲突（前端 (type, id) 复合键已隔离，type='ego' 唯一）。 */
    flowsim::EntityId ego_id = g.pool.alloc(flowsim::EntityType::Ego);
    flowsim::Entity& ego = g.pool[ego_id];
    ego.x = sc->ego.x;
    ego.y = sc->ego.y;
    ego.heading = sc->ego.heading;
    ego.speed = 0.0;  /* 冷启动：初始速度恒为 0，等控制指令到达后再加速。
                       * 原 init_speed=10.0 在控制未就绪时让 ego 以 10m/s 无控行驶，
                       * 可能与 NPC 碰撞。冷启动分支（use_internal_cruise=true,
                       * last==0）已设 brake=1.0/speed=0，但 init 阶段设非零速度
                       * 会在第一帧 step_bicycle 前残留。 */
    ego.target_vx = (sc->ego.target_speed > 0) ? sc->ego.target_speed : g.target_speed;
    ego.vx = ego.speed;  /* 初始沿 x 方向 */
    /* 车参单一事实源 = 场景 ego 块（scenario_loader 已解析）。
     * 场景未配置（0）时用默认值，再交给 apply_vehicle_defaults 兜底。
     * 旧实现三处硬编码 2.7（entity.h/flowsim/planning 2.8 甚至不一致），
     * 2026-08 收口：场景配了 → Entity 用之并广播给 planning/control。 */
    ego.length = (sc->ego.length > 0.0) ? sc->ego.length : EGO_LEN_M;
    ego.width = (sc->ego.width > 0.0) ? sc->ego.width : EGO_WID_M;
    ego.wheelbase = (sc->ego.wheelbase > 0.0) ? sc->ego.wheelbase : 2.7;
    ego.mass = 1500.0;
    ego.drag_coeff = 0.3;
    flowsim::apply_vehicle_defaults(ego);

    /* Phase 2: ego 持久 road_pos 初始化。
     * 用 world_to_frenet 把场景 ego.x/y 转成 Frenet 后 init esmini position handle。
     * 失败时 road_pos.ok()==false，主循环走旧 route 逻辑兜底。
     * 成功时用 esmini 算出的实际 WorldPos 反向覆盖 ego.x/y/heading，保证 ego
     * 起点严格在 road 0 参考线/车道中心上（场景 x/y 可能是手填的相对值，
     * 例如中凯路 ego.x=5 含义是沿 road 0 s=5m 而非世界坐标 (5, -1.75)，
     * 不覆盖会出现在路网外的"鬼影 ego"）。 */
    if (g.roads_loaded) {
        flowsim::FrenetPos fp;
        if (g.roads.world_to_frenet(sc->ego.x, sc->ego.y, fp)) {
            if (ego.road_pos.init(g.roads, fp.road_id, fp.lane_id, fp.s, fp.offset)) {
                /* 用 esmini 算出的 WorldPos 覆盖 ego 初位置（消除"鬼影 ego"） */
                flowsim::WorldPos wp;
                if (ego.road_pos.world(wp)) {
                    ego.x = wp.x;
                    ego.y = wp.y;
                    ego.heading = wp.h;
                }
            } else {
                LOG_WARN("flowsim", "ego road_pos.init failed (road=%d lane=%d s=%.1f)",
                         fp.road_id, fp.lane_id, fp.s);
            }
        } else {
            LOG_WARN("flowsim", "ego world_to_frenet failed at (%.1f,%.1f) — road_pos off",
                     sc->ego.x, sc->ego.y);
        }
    }

    /* Actors → NPC 车辆 / 行人 */
    for (int i = 0; i < sc->actor_count && i < flowsim::MAX_ENTITIES - 1; i++) {
        const ScenarioActor* a = &sc->actors[i];
        flowsim::EntityType et = actor_type_to_entity(a->type);
        flowsim::EntityId id = g.pool.alloc(et);
        if (id == flowsim::INVALID_ENTITY) break;
        flowsim::Entity& e = g.pool[id];
        e.scenario_id = a->id;  /* 业务 id 存独立字段；e.id 保持 pool 索引，全局唯一 */

        /* 新格式：segment_id ≥ 0 → 用 esmini Frenet→World 转换
         *
         * NOA Phase 6 P2 兜底：esmini 加载失败（roads_loaded=false）时，新格式场景
         * 的 actors 仍走此分支做线性放置（e.x=s, e.y=l），而非 fallthrough 到旧
         * 格式 x/y（新场景 x/y 可能为 0，导致 NPC 全堆原点）。线性放置虽不精确
         * （忽略道路弯曲），但至少沿 s 方向分散，NPC AI 仍能正常巡航/跟车。 */
        if (a->segment_id >= 0) {
            if (g.roads_loaded) {
                flowsim::WorldPos wp;
                if (g.roads.frenet_to_world(a->segment_id, 0, a->s, a->l, wp)) {
                    e.x = wp.x;
                    e.y = wp.y;
                    e.heading = wp.h;
                } else {
                    /* esmini 查不到此 road id → 退化为沿 x 轴线性放置 */
                    e.x = a->s;
                    e.y = a->l;
                    e.heading = 0.0;
                    LOG_WARN("flowsim", "NPC %d: road %d not in esmini, fallback to (%.1f, %.1f)",
                             a->id, a->segment_id, e.x, e.y);
                }
            } else {
                /* esmini 未加载 → 线性放置：s 当 x，l 当 y，heading=0（直道假设）。
                 * 叠加弯道中心线偏移（若场景配了弯道），让 NPC 不全在 y=l 上。 */
                e.x = a->s;
                e.y = a->l + road_center_y(a->s, g.curve_start_x, g.curve_length_m, g.curve_offset_m);
                e.heading = 0.0;
                if (a->id <= 2) {  /* 只对前几个 NPC 日志，避免刷屏 */
                    LOG_WARN("flowsim", "NPC %d: esmini offline, linear spawn at (%.1f, %.1f)",
                             a->id, e.x, e.y);
                }
            }
        } else {
            /* 旧格式：全局 x/y 坐标。
             * road_network/esmini 已加载时 x/y 视为世界坐标（弯道场景把 NPC
             * 投到路中心线附近，再 world_to_frenet snap）；未加载时回退
             * pipeline 参数弯道：y = road_center_y(x) + 横向偏移（vy==0 约定）。 */
            e.x = a->x;
            if (g.roads_loaded) {
                e.y = a->y;
            } else if (a->vy == 0.0) {
                e.y = road_center_y(a->x, g.curve_start_x, g.curve_length_m, g.curve_offset_m) + a->y;
            } else {
                e.y = a->y;
            }
        }
        e.vx = a->vx;
        e.vy = a->vy;
        e.speed = sqrt(a->vx * a->vx + a->vy * a->vy);
        /* 对向来车 (vx<0)：叠加 π 到 esmini 报告的 heading，
         * 而非覆盖为固定 M_PI。弯道路段 esmini heading 可能非零。 */
        if (a->vx < 0.0 && a->vy == 0.0) {
            e.heading = fmod(e.heading + M_PI, 2.0 * M_PI);
            e.target_vx = -a->vx;
        } else {
            e.target_vx = a->vx;  /* NPC 初始目标速度 = 场景速度 */
        }
        e.length = (a->len > 0) ? a->len : 4.6;
        e.width = (a->wid > 0) ? a->wid : 2.0;
        if (e.is_vehicle()) {
            flowsim::apply_vehicle_defaults(e);
            e.length = (a->len > 0) ? a->len : e.length;
            e.width = (a->wid > 0) ? a->wid : e.width;
        } else if (e.type == flowsim::EntityType::Pedestrian) {
            /* 行人默认尺寸：0.5m × 0.5m（场景配置了 len/wid 时优先使用） */
            if (a->len <= 0) e.length = 0.5;
            if (a->wid <= 0) e.width = 0.5;
        }
        /* 中央 route 初始化：所有 NPC 车辆（新旧格式）都通过 world_to_frenet
         * 定位到路网后初始化 route_dir/route_s/offset，保证严格贴道路几何行驶。
         *
         * 旧格式（segment_id<0）NPC 以前走世界系直线积分兜底，在弯道/匝道上
         * 会沿切线飞出路面；对向车（vx<0）更是 route_dir=0 导致 heading 翻转
         * 缺失，在路网中朝错误方向行驶（逆行到对向车道）。
         *
         * 现在统一用 world_to_frenet 定位 → 填 road_id/s/offset → npc_init_route
         * 设置 route_dir（顺行 +1 / 对向 -1），step_npc_vehicle 自动走正确的
         * Frenet 车道跟随分支。新格式（segment_id≥0）优先用场景指定值，
         * world_to_frenet 失败时回退到旧格式兜底。 */
        if (g.roads_loaded && g.route.ok() && e.is_npc_vehicle()) {
            if (a->segment_id >= 0) {
                /* 新格式：用场景指定的 segment_id/s/l 直接初始化 */
                e.road_id = a->segment_id;
                e.s       = a->s;
                e.offset  = a->l;
                e.target_offset = a->l;
            } else {
                /* 旧格式：用 world_to_frenet 反算路网位置 */
                flowsim::FrenetPos fp;
                if (g.roads.world_to_frenet(e.x, e.y, fp)) {
                    e.road_id = fp.road_id;
                    e.lane_id = fp.lane_id;   /* P1 修复：保存 lane_id，
                                              * 供 road_pos.init 用真实车道而非 lane_id=0
                                              * 中心线。原代码丢掉 fp.lane_id 让下游
                                              * road_pos.init 硬编码 lane_id=0，
                                              * RM_SetLanePosition 对 type="none" 的中心
                                              * 线车道 align=true 时返回 h=PI（对向），
                                              * 导致同向 NPC vx = speed*cos(PI) = -speed
                                              * 倒着开（npc1 从 x=120 倒退到 x=99）。 */
                    e.s       = fp.s;
                    /* fp.offset 是车道内偏移（车在车道中心时≈0），需换算成相对
                     * 参考线的横向偏移：ref_offset = lane_center + fp.offset。
                     * C-2: 用 lane_frenet.h 共享 helper 替换手写公式，避免各处
                     * 复制 sign(lane_id)·(|lane_id|−0.5)·lane_width 同一公式。
                     * 这与 ego 路径一致（ego 走 fp.lane_id，road_pos 内部算车道中心）。 */
                    e.offset        = flowsim::offset_from_lane_internal(
                                          g.roads, fp.road_id, fp.lane_id, fp.s, fp.offset);
                    e.target_offset = e.offset;
                }
                /* world_to_frenet 失败 → route_dir 保持 0，走旧世界系兜底 */
            }
            /* ── Bug 修复：原条件 `e.road_id > 0` 漏掉了 road_id == 0 的合法道路。
             *
             * straight_road.json 等场景的 road_network.edges[0].id = 0，
             * world_to_frenet 正确返回 fp.road_id = 0，但原 `> 0` 判断让
             * npc_init_route 永远不被调用 → route_dir 保持 0 → step_npc_vehicle
             * 走 world_to_frenet 兜底分支（line ~762）。
             *
             * 兜底分支用 `npc.offset = f.offset` 覆写 offset（raw lane offset，
             * 不是 lane_center + offset），与 init 时 `e.offset = lane_center_t + fp.offset`
             * 不一致 → 第一帧 npc.offset 从 -1.75 跳到 0（车道边线/中心线），
             * 之后 world_to_frenet 在车道边界抖动把 NPC 反复 snap 到不同 lane_id，
             * npc.y 在 -1.75 / 0.0 / +1.75 之间"变来变去"。
             *
             * 改为 `>= 0`：road_id=0 是合法道路，npc_init_route 内部会用
             * route.index_of(road_id) 判断是否在主 route 上，不在则置 route_dir=0
             * 自动走兜底，所以这里无条件调用是安全的。 */
            if (e.road_id >= 0 || a->segment_id >= 0) {
                flowsim::npc_init_route(e, g.route, (a->vx < 0.0) ? -1 : 1);
            }
        }

        /* Phase 2: NPC 持久 road_pos 初始化（与 route 共存）。
         * 新格式 (segment_id≥0) NPC：直接用场景指定的 segment_id/s/l init。
         * 旧格式 (segment_id<0) NPC：用上一步 world_to_frenet 反算的
         * (road_id, lane_id, s, offset) init，让 NPC 也能沿真实 OpenDRIVE 拓扑推进。
         *
         * 对向车 (route_dir<0) 虽然 road_pos.advance 不支持 -s 方向（step_npc_vehicle
         * 会走旧 route 分支），但 road_pos.init 后 set_offset 可正确更新横向位置，
         * 且 road_pos.frenet() 供 same_lane/find_lead 等用。 */
        if (g.roads_loaded && e.is_npc_vehicle()) {
            int rid = e.road_id;
            int lid = 0;          /* 默认中心线（新格式旧分支） */
            double sl = e.s;
            double ol = e.offset;
            if (a->segment_id >= 0) {
                rid = a->segment_id;
                sl = a->s;
                ol = a->l;
            } else {
                /* P1 修复：旧格式 NPC 用 world_to_frenet 反算的真实 lane_id。
                 * 原代码硬编码 lane_id=0（中心线 type="none"），RM_SetLanePosition
                 * 对该车道 align=true 返回 h=PI（对向），导致同向 NPC
                 * vx = speed*cos(PI) = -speed 倒着开（npc1 从 x=120 倒退到 x=99）。 */
                lid = e.lane_id;
                /* road_pos.init 的 offset 是 lane 内偏移（车道中心时≈0）。
                 * e.offset 是道路参考线偏移（lane_center + fp.offset），不能直接传；
                 * 车在车道中心时 fp.offset≈0，多数场景满足，用 0 即可。 */
                ol = 0.0;
            }
            /* ── Bug 修复：原条件 `rid > 0` 漏掉了 road_id == 0 的合法道路。
             *
             * 这与 line 561 的 `e.road_id >= 0` 修复是同一个 bug 模式：
             * straight_road.json 的 road_network.edges[0].id = 0，NPC 通过
             * world_to_frenet 反算得到 road_id=0，但此处 `> 0` 让 road_pos
             * 永远不被 init → step_npc_vehicle 中 `npc.road_pos.ok()` 为 false
             * → 跳过 line 637 的 RoadPosition 推进分支，落到 line 701 的
             * route 兜底分支。
             *
             * route 分支虽然也能工作（用 npc.offset 直接调 frenet_to_world），
             * 但失去了 road_pos 的精确车道对齐 + junction 选支路能力，且
             * recycle_npc 内部也依赖 road_pos.init 把 NPC 重新定位到新位置。
             * 改为 `>= 0` 与 line 561 一致，让 road_id=0 的 NPC 也走 road_pos
             * 主路径。esmini 内部 RM_CreatePosition 对 road_id=0 是合法的。 */
            if (rid >= 0) {
                if (!e.road_pos.init(g.roads, rid, lid, sl, ol) && a->id <= 2) {
                    LOG_WARN("flowsim", "NPC %d road_pos.init failed — fallback to route/world logic",
                             a->id);
                }
            }
        }
    }

    /* 红绿灯 → TrafficLight 实体 */
    /* 杆位置修正（两层语义）：
     *   1) 场景 y_lane 是「停止线所在车道中心」的横向偏移（仅用于规划/停止线判定）；
     *   2) 3D 渲染里红绿灯杆应放在路缘外侧 +1.5m 退让，避免「杆立在路中间」的诡异画面。
     * 之前 world_half_width 硬编码为 3.5m（2 车道 × 3.5），但中凯路 road 3 是 3 车道
     * （半宽 5.25m）→ 杆位 y=5.0 实际在中央车道分隔线上。修复：esmini 加载时
     * 用 RM_GetRoadNumberOfDrivableLanes + RM_GetLaneIdByIndex 反查车道数+宽度
     * 算 road_half_width；非 esmini 路径从 sc->road.lanes/lane_width
     * （scenario_loader 从 road_network.edges[0] 提取）算 road_half_width，
     * 避免 4 车道场景灯杆落在路面内与车辆位置重叠。
     * 此外场景 traffic_lights[*].x 可能是手填的「近似世界坐标」（中凯路 x=770
     * 实际应在路口中心 x=560）——esmini 加载时用 world_to_frenet 校正 x/y，
     * 找到最近的 (road_id, s, lane_id, offset) 再 frenet_to_world 反算真值。
     * 这样无论场景 JSON 用相对值还是手填世界坐标，灯杆永远落在正确车道。
     *
     * heading 修正：灯杆臂应垂直于道路切线方向（指向道路中心）。
     *   - 北侧杆 (sign=+1)：heading = road_tangent - π/2（臂朝南）
     *   - 南侧杆 (sign=-1)：heading = road_tangent + π/2（臂朝北）
     * 之前 `if (e.heading == 0.0)` 检查会在垂直计算恰好得 0 时误覆盖
     * （如道路朝北 + 北侧杆 → π/2 - π/2 = 0），且非 esmini 路径从未设垂直朝向。 */
    {
        /* 非 esmini 路径的 road_half_width：从场景配置算一次，循环外缓存。
         * sc->road.lanes/lane_width 由 scenario_loader 从 road_network.edges[0]
         * 提取；未配置（旧场景）时 fallback 2 车道 × 3.5m = 7m 宽，半宽 3.5m。 */
        int    fallback_lanes = (sc->road.lanes > 0) ? sc->road.lanes : 2;
        double fallback_lw    = (sc->road.lane_width > 0.0) ? sc->road.lane_width : 3.5;
        double fallback_half_width = (fallback_lanes * fallback_lw) / 2.0;

        /* 缓存：每个 (road_id, s) 位置处的 road_half_width，避免重复 esmini 调用 */
        double cached_half_width = -1.0;
        for (int i = 0; i < sc->traffic_light_count && i < SCENARIO_MAX_TRAFFIC_LIGHTS; i++) {
            const ScenarioTrafficLight* tl = &sc->traffic_lights[i];
            flowsim::EntityId id = g.pool.alloc(flowsim::EntityType::TrafficLight);
            if (id == flowsim::INVALID_ENTITY) break;
            flowsim::Entity& e = g.pool[id];
            e.scenario_id = tl->id;  /* 业务 id 存独立字段；e.id 保持 pool 索引，全局唯一 */
            e.throttle = tl->green_s;
            e.brake    = tl->yellow_s;
            e.steer    = tl->red_s;
            e.target_vx = tl->phase_offset_s;
            e.heading = tl->heading;
            double sign = (tl->y_lane >= 0.0) ? 1.0 : -1.0;
            /* esmini 接管世界坐标：world_to_frenet → 反算真值。
             * 两层语义：
             *   1) e.x/e.y = 灯杆 3D 位置（路缘外侧 +1.5m 退让，避免杆立在路中间）
             *   2) e.target_vx 存相位偏移，用 e.width 存 lane_y（车道中心 y），
             *      供 NPC 红绿灯响应（check_npc_scene_events）判断同侧车流。 */
            if (g.roads_loaded) {
                flowsim::FrenetPos fp;
                /* 用 y_lane 作横向偏移（与 x 一起试探路网最近点） */
                double probe_y = tl->y_lane;
                if (g.roads.world_to_frenet(tl->x, probe_y, fp)) {
                    /* 先拿道路中心线世界坐标（offset=0 的 reference line） */
                    flowsim::WorldPos rc_wp;
                    bool have_rc = g.roads.frenet_to_world(fp.road_id, 0, fp.s, 0.0, rc_wp);
                    if (cached_half_width < 0.0 || fp.road_id != e.road_id) {
                        int n_drivable = g.roads.drivable_lane_count(fp.road_id, fp.s);
                        cached_half_width = (n_drivable * g.lane_width) / 2.0;
                        e.road_id = fp.road_id;
                    }
                    /* 车道中心 y：从道路中心 + sign * |y_lane|（沿路宽方向） */
                    double lane_center_y = have_rc ? (rc_wp.y + tl->y_lane) : tl->y_lane;
                    /* 灯杆 3D 位置：车道侧路缘外 +1.5m 退让 */
                    double pole_y = have_rc ? (rc_wp.y + sign * (cached_half_width + 1.5)) : (sign * (cached_half_width + 1.5));
                    e.x = have_rc ? rc_wp.x : tl->x;
                    e.y = pole_y;
                    /* e.width 存车道中心 y，供 NPC 红绿灯响应判断同侧车流 */
                    e.width = lane_center_y;
                    /* heading：灯杆臂垂直于道路切线方向（指向道路中心）。
                     * 场景显式配置 tl->heading 时优先用配置值；否则从道路切线推算。
                     * 用 have_rc 标志判断是否成功取到道路切线，不再用
                     * `e.heading == 0.0` 检查（会在垂直计算恰好得 0 时误覆盖）。 */
                    if (tl->heading != 0.0) {
                        e.heading = tl->heading;
                    } else if (have_rc) {
                        e.heading = rc_wp.h + (sign > 0.0 ? -M_PI_2 : M_PI_2);
                    } else {
                        double tan_h = compute_road_heading_at(tl->x, tl->y_lane);
                        e.heading = tan_h + (sign > 0.0 ? -M_PI_2 : M_PI_2);
                    }
                } else {
                    /* world_to_frenet 失败：场景坐标不在路网附近，fallback */
                    e.x = tl->x;
                    e.y = sign * (fallback_half_width + 1.5);
                    e.width = tl->y_lane;  /* 存原始 lane_y */
                    double tan_h = compute_road_heading_at(tl->x, tl->y_lane);
                    e.heading = (tl->heading != 0.0) ? tl->heading
                                : (tan_h + (sign > 0.0 ? -M_PI_2 : M_PI_2));
                }
            } else {
                /* esmini 不可用：从场景配置算 road_half_width + 垂直朝向 */
                e.x = tl->x;
                e.y = sign * (fallback_half_width + 1.5);
                e.width = tl->y_lane;  /* 存原始 lane_y */
                double tan_h = compute_road_heading_at(tl->x, tl->y_lane);
                e.heading = (tl->heading != 0.0) ? tl->heading
                            : (tan_h + (sign > 0.0 ? -M_PI_2 : M_PI_2));
            }
        }
    }

    /* Phase 4: ETC 门架 → ETCGate 实体（高速收费站抬杆）。
     * scene_events.cpp 的 tick_etc_gates() 根据 ego 距离门架的距离驱动
     * 抬杆动画：远距 closed → 进入 open_range_m 时 opening → 通过后 open。
     * approach_speed 复用 target_vx 字段，open_range_m 复用 phase_timer 字段。
     * esmini 接管世界坐标：etc_gates[*].x/y 可能是手填的「近似世界坐标」（中凯路
     * 实际 road 1 起点 (250,0) 但 etc_gates x=290 对应 road 1 内 s=40，y 通道
     * 中心 ±1.75/±5.25 来自场景车道布局）——esmini 加载时用 world_to_frenet 找
     * 最近 (road_id, s, offset)，再 frenet_to_world 反算真值。这样 etc_gates
     * 自动跟随路网几何（弯曲/匝道等），不依赖场景手填精度。 */
    for (int i = 0; i < sc->etc_gate_count && i < SCENARIO_MAX_ETC_GATES; i++) {
        const ScenarioETCGate* eg = &sc->etc_gates[i];
        flowsim::EntityId id = g.pool.alloc(flowsim::EntityType::ETCGate);
        if (id == flowsim::INVALID_ENTITY) break;
        flowsim::Entity& e = g.pool[id];
        e.scenario_id = eg->id;  /* 业务 id 存独立字段；e.id 保持 pool 索引，全局唯一 */
        e.target_vx = eg->approach_speed;   /* ETC 通过目标速度 */
        e.phase_timer = 0.0;                 /* 抬杆进度 [0,1]，初始 closed */
        e.width = eg->open_range_m;          /* open_range_m 存到 width 字段 */
        e.state = flowsim::NpcState::Stopped; /* 初始 closed 状态 */
        e.heading = eg->heading;
        /* esmini 校正坐标 */
        if (g.roads_loaded) {
            flowsim::FrenetPos fp;
            if (g.roads.world_to_frenet(eg->x, eg->y, fp)) {
                flowsim::WorldPos wp;
                if (g.roads.frenet_to_world(fp.road_id, fp.lane_id, fp.s, fp.offset, wp)) {
                    e.x = wp.x;
                    e.y = wp.y;
                    e.heading = wp.h;
                } else {
                    e.x = eg->x; e.y = eg->y;
                }
            } else {
                e.x = eg->x; e.y = eg->y;
            }
        } else {
            e.x = eg->x; e.y = eg->y;
        }
        if (e.heading == 0.0) {
            e.heading = compute_road_heading_at(e.x, e.y);
        }
    }

}

/* ── 发布函数 ─────────────────────────────────────────────────── */

/* 施工区域障碍物注入：把场景 construction_zones 前缘按车道宽铺成一排静态
 * 障碍物写进 vehicle/state，供 perception 识别为 type="construction"（前方封路）。
 * 位置 = 施工段前缘 front_x = center_x - length/2（ego 顺行 +x 从低 x 侧接近），
 * 横向铺满施工宽度。这些障碍物不入实体池（不参与碰撞/NPC AI），仅作感知目标。
 *
 * 2026-08-07：围栏厚度 2→4m，中心放在 front_x + 0.5*ol，使障碍物前表面贴齐
 * 施工前缘（与 ConstructionView 围栏 / planning 可通行域一致）。旧 ol=2 中心
 * 落在 front → 前表面在 front-1，且薄墙在 FOV 边缘易被遮挡/跟踪丢掉，
 * 表现为"感知没看到那面墙"。
 * 返回追加后的障碍物总数 n_obs。 */
static int append_construction_obstacles(cJSON* vstate, int n_obs) {
    if (!g.scenario) return n_obs;
    const double lw = (g.lane_width > 0.5) ? g.lane_width : 3.5;
    const double wall_ol = 4.0;  /* 围栏纵向厚度：覆盖薄墙漏检 */
    for (int z = 0; z < g.scenario->construction_zone_count; ++z) {
        const ScenarioConstructionZone* cz = &g.scenario->construction_zones[z];
        const double front_x = cz->x - cz->length * 0.5;
        const double ox = front_x + 0.5 * wall_ol;  /* 中心：前表面贴齐 front_x */
        const double width = (cz->width > 0.0) ? cz->width : (lw * 4.0);
        int lanes = (int)std::ceil(width / lw);
        if (lanes < 1) lanes = 1;
        const double y0 = cz->y - width * 0.5 + lw * 0.5;
        for (int k = 0; k < lanes; ++k) {
            if (n_obs >= 120) break;  /* 留余量给 128 上限 */
            const double oy = y0 + (double)k * lw;
            char key[24];
            snprintf(key, sizeof(key), "oid%d", n_obs);
            cJSON_AddNumberToObject(vstate, key, (double)(9000 + cz->id * 10 + k));
            snprintf(key, sizeof(key), "ox%d", n_obs);
            cJSON_AddNumberToObject(vstate, key, ox);
            snprintf(key, sizeof(key), "oy%d", n_obs);
            cJSON_AddNumberToObject(vstate, key, oy);
            snprintf(key, sizeof(key), "ov%d", n_obs);
            cJSON_AddNumberToObject(vstate, key, 0.0);
            snprintf(key, sizeof(key), "ovy%d", n_obs);
            cJSON_AddNumberToObject(vstate, key, 0.0);
            snprintf(key, sizeof(key), "ot%d", n_obs);
            cJSON_AddStringToObject(vstate, key, "construction");
            snprintf(key, sizeof(key), "ol%d", n_obs);
            cJSON_AddNumberToObject(vstate, key, wall_ol);
            snprintf(key, sizeof(key), "ow%d", n_obs);
            cJSON_AddNumberToObject(vstate, key, lw);     /* 单段宽 ≈ 车道宽 */
            n_obs++;
        }
    }
    return n_obs;
}

static void publish_vehicle_state(uint64_t sim_time_us) {
    flowsim::Entity& ego = g.pool[0];

    cJSON* vstate = cJSON_CreateObject();
    /* vehicle/state 字段约定（缩写 → 全称映射）：
     *   x, y    → ENU 世界坐标
     *   spd     → speed
     *   hdg     → heading
     *   thr     → throttle
     *   brk     → brake
     *   tgt     → target_vx
     *   st      → steer
     *   t_us    → time (micros)
     *   road_id, lane_id → Frenet 路网定位
     *   oxN, oyN, ovN, ovyN, otN, olN, owN → 障碍物（N=索引）
     *   n_obs   → 障碍物计数
     * scene/frame 用全称，本 topic 用缩写是为了保持向后兼容。 */
    cJSON_AddNumberToObject(vstate, "x", ego.x);
    cJSON_AddNumberToObject(vstate, "y", ego.y);
    cJSON_AddNumberToObject(vstate, "spd", ego.speed);
    cJSON_AddNumberToObject(vstate, "hdg", ego.heading);
    cJSON_AddNumberToObject(vstate, "thr", ego.throttle);
    cJSON_AddNumberToObject(vstate, "brk", ego.brake);
    cJSON_AddNumberToObject(vstate, "tgt", ego.target_vx);
    cJSON_AddNumberToObject(vstate, "st", ego.steer);
    cJSON_AddNumberToObject(vstate, "yr", ego.yaw_rate);
    cJSON_AddNumberToObject(vstate, "t_us", (double)sim_time_us);
    cJSON_AddNumberToObject(vstate, "road_id", (double)ego.road_id);
    cJSON_AddNumberToObject(vstate, "lane_id", (double)ego.lane_id);
    /* 车参广播（车参单一事实源 = 场景 ego 块）：仅场景显式配置时才发，
     * 下游（planning/control）收到字段即同步，未收到保持各自默认/配置。
     * 这样 RC 小车（pipeline_car.json 场景配 wheelbase=0.25）也能同步。 */
    if (g.scenario && g.scenario->ego.wheelbase > 0.0) {
        cJSON_AddNumberToObject(vstate, "wheelbase", ego.wheelbase);
        cJSON_AddNumberToObject(vstate, "length", ego.length);
        cJSON_AddNumberToObject(vstate, "width", ego.width);
        cJSON_AddNumberToObject(vstate, "max_steer",
                                (g.scenario->ego.max_steer > 0.0)
                                    ? g.scenario->ego.max_steer : 0.60);
    }

    /* 收集非 ego 的活跃实体作为障碍物（车辆 + 行人） */
    int n_obs = 0;
    for (int i = 1; i < g.pool.size(); i++) {
        const flowsim::Entity& e = g.pool[i];
        if (!e.active) continue;
        if (e.type == flowsim::EntityType::TrafficLight ||
            e.type == flowsim::EntityType::ETCGate ||
            e.type == flowsim::EntityType::StopLine) continue;
        if (n_obs >= 128) break;

        char key[20];
        snprintf(key, sizeof(key), "oid%d", n_obs);
        cJSON_AddNumberToObject(vstate, key, (double)e.id);
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
    /* 施工区域障碍物（前方封路）：追加到障碍物列表供感知识别 */
    n_obs = append_construction_obstacles(vstate, n_obs);
    /* Mock NPC 注入已移除（2026-08）：空场景时伪造两辆前车会把"无车场景"
     * 变成"有前车场景"，behavior/planning 无法区分真伪（测试脚手架不应在
     * 产线路径上）。空场景 = 无前车 = 正常巡航。 */
    cJSON_AddNumberToObject(vstate, "n_obs", n_obs);

    char* s = cJSON_PrintUnformatted(vstate);
    transport_publish(g.transport, TOPIC_VEHICLE_STATE,
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
    /* 按当前 ego 所在 road 实时查询可行驶车道数。
     * 旧实现硬编码 lane_count=2，导致 4 车道 toll_plaza / 3 车道 urban 段
     * control_node 的 cruise_lane_y 算式只能算出 ±half_lane（中间两车道），
     * ego 永远到不了 +5.25 / -5.25 外侧车道，看起来像"逆行"。
     * 这里查询失败时回退 2（向后兼容 2 车道场景）。 */
    int lane_count = 2;
    int road_id = -1;
    if (g.roads_loaded) {
        const flowsim::Entity& ego = g.pool[0];
        flowsim::FrenetPos ef;
        if (g.roads.world_to_frenet(ego.x, ego.y, ef)) {
            road_id = ef.road_id;
            int n = g.roads.drivable_lane_count(ef.road_id, ef.s);
            if (n > 0) lane_count = n;
        }
    }
    cJSON_AddNumberToObject(root, "lane_count", lane_count);
    /* 单双向：behavior 变道候选限制用（双向路禁止越线变道到对向） */
    cJSON_AddNumberToObject(root, "oneway",
                            (g.scenario && g.scenario->road.oneway) ? 1 : 0);
    /* 同步到 scene_pub_cfg，供 ego fallback 横向控制（lane_keep_ego_fallback）
     * 和 scene/frame 发布使用，否则它们仍用 init 默认值 2，导致 4 车道场景
     * ego target_y 算错。 */
    g.scene_pub_cfg.lane_count = lane_count;
    char* s = cJSON_PrintUnformatted(root);
    transport_publish(g.transport, TOPIC_ROAD_GEOMETRY,
                      (const uint8_t*)s, (uint32_t)strlen(s) + 1);
    free(s);
    cJSON_Delete(root);
    g.last_road_geom_road_id = road_id;
    g.last_road_geom_lane_count = lane_count;
}

static bool should_publish_road_geometry_now(void) {
    if (g.cycle % ROAD_GEOMETRY_REPUBLISH_CYCLES == 0) return true;
    if (!g.roads_loaded) return false;
    const flowsim::Entity& ego = g.pool[0];
    flowsim::FrenetPos ef;
    if (!g.roads.world_to_frenet(ego.x, ego.y, ef)) return false;
    int lane_count = g.last_road_geom_lane_count;
    int n = g.roads.drivable_lane_count(ef.road_id, ef.s);
    if (n > 0) lane_count = n;
    return ef.road_id != g.last_road_geom_road_id ||
           lane_count != g.last_road_geom_lane_count;
}

/**
 * publish_ref_path — ego route-following 参考路径发布。
 *
 * 背景：control_node Stanley 横向控制原本依赖全局单段 curve_*（curve_start_x/
 * curve_length_m/curve_offset_m）算 cte/heading/kappa，road_network 多 edge 场景
 * 下 curve_* 全零 → 参考线退化为 y=0 直线 → ego 过 fork 后沿直线开进空地。
 * 本函数发布 **route centerline** 前方 N 个参考点（含曲率前馈），让 planning/
 * control 对 d=0 的理解统一为"道路中心线"，而不是当前 ego 车道中心。
 * 旧实现优先用 ego.road_pos.sample_ahead()，它采到的是当前 lane handle 的
 * 中心线；planning 再叠 target_lane_offset 时会出现双重偏移，右转/支路场景下
 * control 只会忠实跟踪一条已经偏到路肩上的坏轨迹。
 *
 * 每 cycle 发布（60Hz），control_node 按需消费。
 * 无 route / esmini 加载失败时发布空数组（control_node 检测到空数组回退 curve_*）。
 */
static void publish_ref_path(void) {
    cJSON* root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "t_us", (double)clock_now_us());

    cJSON* pts = cJSON_CreateArray();
    if (g.roads_loaded && g.route.ok()) {
        const flowsim::Entity& ego = g.pool[0];
        /* ego 在 route 上的累计 s：用 compute_ego_route_s 复用 road_pos handle */
        double ego_route_s = compute_ego_route_s();
        if (ego_route_s < 0.0) ego_route_s = 0.0;  /* 不在 route 上时从起点采样 */
        /* 采样 ego 前方 100m、间距 5m（21 个点）；lookahead 远一些保证
         * 控制器在高速段（27m/s）每步 1.35m 也有充足前视距离。 */
        std::vector<flowsim::RefPathPoint> samples;
        int n = 0;
        /* u_turn_active（ego 在对向车道，行进方向 = route_s 递减）时反向采样：
         * ref_path 始终代表「ego 真实行进方向前方的道路」，heading 已翻转 π。
         * 这样 planning/control 无需知道 +x/-x，把 ref_path 当作前方路径自然跟随。 */
        n = g.route.sample_ahead(g.roads, ego_route_s, 100.0, 5.0, samples,
                                 /*reverse=*/g.u_turn_active);
        /* fallback：route 采样失败时再退回当前车道中心线，避免完全断 ref_path。
         * 对向行驶（u_turn_active）时跳过：road_pos.sample_ahead 只能沿 +s 前进采样，
         * 在返程（实际行进方向 = -s）会产出指向车后的坏 ref_path，比空 ref_path 更糟
         * （control 会主动朝错误方向跟）。返程 route 采样失败宁可发空、走 control 的
         * curve_* 兜底。 */
        if (n == 0 && !g.u_turn_active && ego.road_pos.ok()) {
            flowsim::FrenetPos saved_fp;
            bool has_saved = ego.road_pos.frenet(saved_fp);
            flowsim::Entity& ego_mut = g.pool[0];
            n = ego_mut.road_pos.sample_ahead(g.roads, 100.0, 5.0, M_PI, samples);
            if (has_saved) {
                ego_mut.road_pos.relocate(g.roads, saved_fp.road_id,
                                          saved_fp.lane_id, saved_fp.s,
                                          saved_fp.offset);
            }
        }
        for (int i = 0; i < n; ++i) {
            const auto& p = samples[i];
            cJSON* pt = cJSON_CreateObject();
            cJSON_AddNumberToObject(pt, "x", p.x);
            cJSON_AddNumberToObject(pt, "y", p.y);
            cJSON_AddNumberToObject(pt, "h", p.h);
            cJSON_AddNumberToObject(pt, "kappa", p.kappa);
            cJSON_AddNumberToObject(pt, "rs", p.route_s);
            cJSON_AddItemToArray(pts, pt);
        }
    }
    cJSON_AddItemToObject(root, "points", pts);
    /* 权威行进方向信号：u_turn_active 由 flowsim 掉头状态机在 finalize 时置位、
     * 整个返程保持 true（第二次掉头才复位），永不抖动。navigation 消费此标志作为
     * 唯一权威 travel_dir，取代基于 dx/heading 的猜测（掉头/绕圈时会来回翻）。 */
    cJSON_AddBoolToObject(root, "reverse", g.u_turn_active ? 1 : 0);
    /* 路端位置（单一事实源）：route 总长。behavior 的掉头触发用它做固定
     * 参考（替代"采样末点封顶检测"——低速起步时末点位移小会误判封顶，
     * 导致启动即触发掉头逆行，2026-08-03 实测）。 */
    cJSON_AddNumberToObject(root, "road_end_x", g.route.total_length());
    char* s = cJSON_PrintUnformatted(root);
    transport_publish(g.transport, TOPIC_ROAD_REF_PATH,
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
        /* C-6: 加 scenario_id 字段（场景业务 id，前端用它匹配场景 JSON 的
         * traffic_lights[*].id 做 phase override / choreography beat 定位）。
         * e.id 是 pool 索引（全局唯一但与场景无关），scenario_id 是场景里
         * 写死的业务 id，二者不可混用——前端旧代码用 e.id 反查场景配置会失败。 */
        cJSON_AddNumberToObject(light, "scenario_id", e.scenario_id);
        cJSON_AddNumberToObject(light, "x", e.x);
        /* y_lane = 车道中心 y（停止线判定用），非灯杆 3D 位置。
         * 灯杆的 3D y 在 scene/frame entities 中以 "y" 字段发布。
         * 这里复用 e.width 存储初始化时写入的 lane_center_y（见 line 701）。 */
        cJSON_AddNumberToObject(light, "y_lane", e.width);
        flowsim::TLPhase ph = static_cast<flowsim::TLPhase>(e.phase_state);
        cJSON_AddStringToObject(light, "state", tl_phase_str(ph));
        cJSON_AddNumberToObject(light, "remain_s", e.phase_timer);
        cJSON_AddItemToArray(lights, light);
    }
    cJSON_AddItemToObject(root, "lights", lights);
    char* s = cJSON_PrintUnformatted(root);
    transport_publish(g.transport, TOPIC_ROAD_TRAFFIC_LIGHTS,
                      (const uint8_t*)s, (uint32_t)strlen(s) + 1);
    free(s);
    cJSON_Delete(root);
}

static void publish_sim_tick(uint64_t sim_time_us) {
    cJSON* tick = cJSON_CreateObject();
    cJSON_AddNumberToObject(tick, "t_us", (double)sim_time_us);
    cJSON_AddNumberToObject(tick, "cycle", g.cycle);
    char* s = cJSON_PrintUnformatted(tick);
    transport_publish(g.transport, TOPIC_SIM_TICK,
                      (const uint8_t*)s, (uint32_t)strlen(s) + 1);
    free(s);
    cJSON_Delete(tick);
}

static void publish_sim_collision(const flowsim::Entity& a, const flowsim::Entity& b) {
    cJSON* col = cJSON_CreateObject();
    cJSON_AddNumberToObject(col, "ego_x", a.x);
    cJSON_AddNumberToObject(col, "ego_y", a.y);
    cJSON_AddNumberToObject(col, "ego_heading", a.heading);
    cJSON_AddNumberToObject(col, "ego_speed", a.speed);
    cJSON_AddNumberToObject(col, "obs_id", b.id);
    /* NPC 现场数据：位置/速度/航向，复盘时无需推演 */
    cJSON_AddNumberToObject(col, "obs_x", b.x);
    cJSON_AddNumberToObject(col, "obs_y", b.y);
    cJSON_AddNumberToObject(col, "obs_heading", b.heading);
    cJSON_AddNumberToObject(col, "obs_vx", b.vx);
    cJSON_AddNumberToObject(col, "obs_vy", b.vy);
    cJSON_AddNumberToObject(col, "obs_speed", b.speed);
    cJSON_AddNumberToObject(col, "overlap_x",
        (a.length + b.length) * 0.5 - fabs(a.x - b.x));
    cJSON_AddNumberToObject(col, "overlap_y",
        (a.width + b.width) * 0.5 - fabs(a.y - b.y));
    char* s = cJSON_PrintUnformatted(col);
    transport_publish(g.transport, TOPIC_SIM_COLLISION,
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

    /* 横向：车道保持 — 朝 ego 当前所在车道中心。
     * 旧实现硬编码 -0.5*lane_width（2 车道左车道中心），是 2 车道假设。
     * N 车道模型：用 lane_idx_from_y 量化 ego 当前车道 idx，再算该车道中心 y。
     * 单车道路段（如 ramp_curve）回退到道路中心。
     *
     * 2026-07-26 fix：替换旧公式 road_h - heading + y_err*0.05。
     * 运动学模型下heading 在变道中累积偏转角后不自动回正，
     * road_h - heading 在变道结束 heading 未收敛时产生不必要的大 steer。
     * 改用 heading_err*0.3 + lat_err*0.03 降低对 heading 残留的敏感度。 */
    double rc_y = road_center_y(ego.x, g.curve_start_x, g.curve_length_m, g.curve_offset_m);
    int ego_lane_idx = lane_idx_from_y(ego.y, g.scene_pub_cfg.lane_count, g.lane_width, rc_y, 0.0);
    double target_y = lane_center_y(ego_lane_idx, g.scene_pub_cfg.lane_count, g.lane_width, rc_y, 0.0);
    double y_err = target_y - ego.y;
    double road_h = road_center_heading(ego.x, g.curve_start_x, g.curve_length_m, g.curve_offset_m);
    double heading_err = road_h - ego.heading;
    while (heading_err >  M_PI) heading_err -= 2.0 * M_PI;
    while (heading_err < -M_PI) heading_err += 2.0 * M_PI;
    const double IH_KD_HEADING = 0.3;    /* heading 阻尼系数 */
    const double IH_KP_LAT = 0.03;        /* 横向偏差 P 增益 */
    ego.steer = heading_err * IH_KD_HEADING
              + y_err * IH_KP_LAT;
    if (ego.steer > 0.15) ego.steer = 0.15;
    if (ego.steer < -0.15) ego.steer = -0.15;
}

/* 返回 ego 当前 road 前方最近施工区前缘的 Frenet s（仅前进方向 lane_id<0 调用）；
 * 无施工区/不在前方返回 -1。掉头点据此落在施工区之前（front - clearance）。 */
static double forward_construction_front_s(const flowsim::Entity& ego) {
    if (!g.scenario || g.scenario->construction_zone_count <= 0) return -1.0;
    double best = -1.0;
    for (int z = 0; z < g.scenario->construction_zone_count; ++z) {
        const ScenarioConstructionZone* cz = &g.scenario->construction_zones[z];
        const double fx = cz->x - cz->length * 0.5;  /* 前缘世界 x（顺行接近侧） */
        double fs;
        if (g.roads_loaded) {
            flowsim::FrenetPos fp;
            if (!g.roads.world_to_frenet(fx, cz->y, fp) || fp.road_id != ego.road_id)
                continue;
            fs = fp.s;
        } else {
            fs = fx;  /* 无路网：直道近似 front_x ≈ s */
        }
        if (fs > ego.s && (best < 0.0 || fs < best)) best = fs;
    }
    return best;
}

/* ── 协程主循环 ───────────────────────────────────────────────── */

class FlowSimTask : public CoroutineTask {
public:
    FlowSimTask(MessageBus* bus) : CoroutineTask(bus) {}

    void set_params(Transport* transport) {
        transport_ = transport;
    }

protected:
    Task run() override {
        LOG_INFO("flowsim", "FlowCoro flowsim started (60Hz, roads=%s)",
                 g.roads_loaded ? "esmini" : "fallback");

        flowsim::Entity& ego = g.pool[0];

        /* 常驻订阅桥：订阅生命周期=节点，替代 WhenAnyBusAwaitableT
         * （select_for）的反复订阅/退订——该适配器多次循环后消息与超时
         * fire 双失效（2026-07-31 safety/flowsim 均复现永久挂起，
         * control/cmd 断流 → 内置巡航追尾）。桥的回调只覆盖单槽
         * （depth=1 drop_oldest 语义），协程每 tick try_take 取走。 */
        BusQueueBridge cmd_bridge(bus(), {TOPIC_CONTROL_CMD});

        /* use_internal_cruise 是持久状态（不再每 tick 重置）。
         * 之前是每 tick 局部变量 → 无新消息的 tick 上 FSAFE 误触发
         * brake=1.0 → 与控制指令的 throttle 形成"走/停"振荡 → 车速
         * 恒为 0（2026-08-03 事故链）。改为持久状态后：
         *   - 收到指令 → false（用控制值）
         *   - 指令陈旧 >2000ms → true（FSAFE 停车）
         *   - 再次收到指令 → false（恢复正常） */
        bool use_internal_cruise = true;

        while (!should_stop()) {
            /* 用墙钟单调时间测量帧工作时间——clock_now_us() 在仿真模式下
             * 返回按 50ms/tick 推进的逻辑时间，同一 tick 内 clock_advance_us
             * 后 t_frame_us 恒为 50000 → sleep 恒为 0 → 节点以 3000+Hz
             * 空转，消息总线被淹没（2026-08-03 根因）。
             * clock_now_monotonic_wall_us() 始终返回真实 CLOCK_MONOTONIC，
             * 不受仿真时钟污染。 */
            uint64_t t_start = clock_now_monotonic_wall_us();

            /* 每 tick：桥取最新控制指令（若有）并解析到 atomics。
             * 固定 50ms 周期（sleep_us），不再依赖消息唤醒——即使
             * 消息迟到/丢失，主循环仍以 60Hz 稳定推进。
             * 解析成功即置 use_internal_cruise=false（直接路径，不依赖
             * 陈旧检查——仿真时钟同 tick 内 now==last）。 */
            {
                Message take_msg;
                if (cmd_bridge.try_take(TOPIC_CONTROL_CMD, &take_msg) &&
                    take_msg.data_size > 0) {
                    /* 二进制 ControlCmd 路径 */
                    ControlCmd bin;
                    if (ControlCmd_deserialize(&bin,
                            (const uint8_t*)take_msg.data, take_msg.data_size) == 0) {
                        use_internal_cruise = false;
                        g.ego_throttle.store(bin.throttle, std::memory_order_relaxed);
                        g.ego_brake.store(bin.brake, std::memory_order_relaxed);
                        g.ego_steer.store(bin.steering, std::memory_order_relaxed);
                        g.ego_turn_signal.store(bin.turn_signal, std::memory_order_relaxed);
                        g.ego_hazard.store(bin.hazard, std::memory_order_relaxed);
                        g.ego_gear.store(bin.gear, std::memory_order_relaxed);
                        g.last_control_cmd_us.store(clock_now_us(),
                            std::memory_order_relaxed);
                    } else {
                        /* JSON fallback */
                        cJSON* root = cJSON_Parse((const char*)take_msg.data);
                        if (root) {
                            use_internal_cruise = false;
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
                        } else {
                            /* 两种解析路径都失败 → 诊断日志 */
                            if (g.cycle % 300 == 0) {
                                LOG_WARN("flowsim",
                                         "[DESER_FAIL] control/cmd msg %uB "
                                         "neither binary nor JSON — "
                                         "first 16B: %02x%02x%02x%02x %02x%02x%02x%02x "
                                         "%02x%02x%02x%02x %02x%02x%02x%02x",
                                         (unsigned)take_msg.data_size,
                                         take_msg.data_size > 0 ? (unsigned)take_msg.data[0] : 0,
                                         take_msg.data_size > 1 ? (unsigned)take_msg.data[1] : 0,
                                         take_msg.data_size > 2 ? (unsigned)take_msg.data[2] : 0,
                                         take_msg.data_size > 3 ? (unsigned)take_msg.data[3] : 0,
                                         take_msg.data_size > 4 ? (unsigned)take_msg.data[4] : 0,
                                         take_msg.data_size > 5 ? (unsigned)take_msg.data[5] : 0,
                                         take_msg.data_size > 6 ? (unsigned)take_msg.data[6] : 0,
                                         take_msg.data_size > 7 ? (unsigned)take_msg.data[7] : 0,
                                         take_msg.data_size > 8 ? (unsigned)take_msg.data[8] : 0,
                                         take_msg.data_size > 9 ? (unsigned)take_msg.data[9] : 0,
                                         take_msg.data_size > 10 ? (unsigned)take_msg.data[10] : 0,
                                         take_msg.data_size > 11 ? (unsigned)take_msg.data[11] : 0,
                                         take_msg.data_size > 12 ? (unsigned)take_msg.data[12] : 0,
                                         take_msg.data_size > 13 ? (unsigned)take_msg.data[13] : 0,
                                         take_msg.data_size > 14 ? (unsigned)take_msg.data[14] : 0,
                                         take_msg.data_size > 15 ? (unsigned)take_msg.data[15] : 0);
                            }
                        }
                    }
                }
            }

            /* 持久状态切换：无近期控制指令 → 切回内置巡航/FSAFE */
            if (!use_internal_cruise) {
                uint64_t now = clock_now_us();
                uint64_t last = g.last_control_cmd_us.load(
                    std::memory_order_relaxed);
                if (last == 0 || (now > last &&
                    now - last >= CONTROL_STALE_TIMEOUT_US)) {
                    use_internal_cruise = true;
                }
            }

            /* 桥自愈：仅当回调真正停滞（200 cycle 内 cb_count 零增长）
             * 才重建订阅。原逻辑无条件重连 → 重置 cb/take 计数器 →
             * 重连窗口内消息丢失 → FSAFE 立即触发 → 车速恒为 0
             * （2026-08-03 事故链：cb=4→reconnect→cb=0→FSAFE→cb=5
             * →reconnect→cb=0... 无限循环）。
             *
             * 修复：跟踪上一检查点的 cb_count，仅当它未增长（桥回调
             * 确实停滞）时才重连。重连后重置 last_control_cmd_us 给
             * 新订阅 2s 窗口期，避免立即 FSAFE。 */
            {
                /* 桥自愈检查：状态用 g 字段（reset_runtime_state 场景切换清零），
                 * 旧实现是函数内 static —— 跨场景残留 + 与 g 字段双份状态
                 * （2026-08 修复）。 */
                if (g.cycle % 600 == 0) {
                    uint64_t last = g.last_control_cmd_us.load(std::memory_order_relaxed);
                    uint64_t now  = clock_now_us();
                    uint64_t cur_cb = cmd_bridge.cb_count;
                    bool cb_stalled = (cur_cb == g.bridge_prev_cb_count &&
                                       g.bridge_last_cb_check > 0);
                    g.bridge_prev_cb_count = cur_cb;
                    g.bridge_last_cb_check = now;

                    if (last > 0 && now > last &&
                        now - last >= CONTROL_STALE_TIMEOUT_US &&
                        use_internal_cruise && cb_stalled) {
                        /* 重连防抖：距上次重连 <10s 不再重连。
                         * 旧实现无防抖：短暂卡顿（2s 断流后恢复）在 10s 检查点
                         * 若 cb 恰好未增长 → 误判停滞 → reconnect → 重连本身
                         * 造成短暂断流 → 下一轮又满足 → 反复重连放大控制断流
                         * （2026-08 事故链 cb=4→reconnect→cb=0→FSAFE 循环）。 */
                        if (g.bridge_last_reconnect_us == 0 ||
                            now - g.bridge_last_reconnect_us >= 10000000ULL) {
                            cmd_bridge.reconnect();
                            g.bridge_last_reconnect_us = now;
                            /* 重连后重置时间戳：给新订阅 CONTROL_STALE_TIMEOUT_US
                             * 窗口期建立连接，避免重连后立即 FSAFE。 */
                            g.last_control_cmd_us.store(now, std::memory_order_relaxed);
                            LOG_WARN("flowsim",
                                     "[BRIDGE_RECONNECT] cb stalled %llu→%llu over %.0fms — "
                                     "resubscribed, reset timer (debounced 10s)",
                                     (unsigned long long)g.bridge_prev_cb_count,
                                     (unsigned long long)cur_cb,
                                     (double)(now - last) / 1000.0);
                        }
                    }
                }
            }

            /* ── 游戏模式（demo.sh --game）：玩家键盘操控 ──
             * /tmp/game_mode 存在 = 游戏模式：主循环从 /tmp/game_input.json
             * 读 {throttle, brake, steer}（flowmond POST /api/game/control
             * 写入，浏览器键盘 → HTTP）当控制指令，绕过 control_node。
             * 同时刷新 last_control_cmd_us 避免 FSAFE 误触发。 */
            if (access("/tmp/game_mode", F_OK) == 0) {
                FILE* gf = fopen("/tmp/game_input.json", "r");
                if (gf) {
                    char gbuf[512] = {0};
                    size_t gn = fread(gbuf, 1, sizeof(gbuf) - 1, gf);
                    fclose(gf);
                    if (gn > 0) {
                        cJSON* gj = cJSON_Parse(gbuf);
                        if (gj) {
                            cJSON* gjt = cJSON_GetObjectItemCaseSensitive(gj, "throttle");
                            cJSON* gjb = cJSON_GetObjectItemCaseSensitive(gj, "brake");
                            cJSON* gjs = cJSON_GetObjectItemCaseSensitive(gj, "steer");
                            if (cJSON_IsNumber(gjs)) {
                                double st = gjs->valuedouble;
                                if (st >  0.6) st =  0.6;
                                if (st < -0.6) st = -0.6;
                                g.ego_steer.store(st, std::memory_order_relaxed);
                            }
                            if (cJSON_IsNumber(gjt)) {
                                double th = gjt->valuedouble;
                                if (th >  1.0) th = 1.0;
                                if (th <  0.0) th = 0.0;
                                g.ego_throttle.store(th, std::memory_order_relaxed);
                            }
                            if (cJSON_IsNumber(gjb)) {
                                double br = gjb->valuedouble;
                                if (br >  1.0) br = 1.0;
                                if (br <  0.0) br = 0.0;
                                g.ego_brake.store(br, std::memory_order_relaxed);
                            }
                            cJSON_Delete(gj);
                        }
                    }
                }
                g.last_control_cmd_us.store(clock_now_us(), std::memory_order_relaxed);
            }

            /* ── Step 1: ego 动力学 ──
             * 失效安全：控制指令陈旧（断流 >2000ms）时停车而非内置巡航——
             * 内置巡航用油门维持速度，会把"失去控制"伪装成"正常巡航"，
             * 15.1 直冲前车追尾（2026-07-31 事故链：select_for/transport
             * 回调在 control/cmd 高频下间歇断流，三通道全断 30s+）。
             * Apollo 原则：控制丢失 → 减速停车，不允许继续前进。
             * 超时从 500ms 提升到 2000ms：高负载下消息总线 ~34% 丢包率，
             * 500ms 过于激进导致 FSAFE 误触发（2026-08-03 事故链）。 */
            if (use_internal_cruise) {
                uint64_t last = g.last_control_cmd_us.load(std::memory_order_relaxed);
                uint64_t now  = clock_now_us();
                if (last == 0) {
                    /* 冷启动：从未收到控制指令 → 保持静止 */
                    ego.throttle = 0.0;
                    ego.brake    = 1.0;
                    ego.steer    = 0.0;
                    ego.speed = 0.0;
                    ego.vx = 0.0;
                    ego.vy = 0.0;
                } else if (now > last &&
                    now - last >= CONTROL_STALE_TIMEOUT_US) {
                    /* 曾收到过控制指令但已陈旧 → 失效安全停车 */
                    ego.throttle = 0.0;
                    ego.brake    = 1.0;
                    ego.steer    = 0.0;
                    if (g.cycle % 300 == 0) {
                        LOG_WARN("flowsim",
                                 "[FSAFE] control cmd stale %.0fms — fail-safe stop "
                                 "(was cruise fallback: throttle ramp)",
                                 (double)(now - last) / 1000.0);
                    }
                } else {
                    /* 短暂间隙（<500ms）→ 不应到达（持久状态保证） */
                    internal_cruise_control(ego);
                }
            } else {
                ego.throttle = g.ego_throttle.load(std::memory_order_relaxed);
                ego.brake    = g.ego_brake.load(std::memory_order_relaxed);
                ego.steer    = g.ego_steer.load(std::memory_order_relaxed);
            }
            /* 灯光指令：从 ControlCmd 决策下发（意图先行），非 steer 反推。
             * 在 VehicleActor::update_all_lights 之前设置，VehicleActor 只补
             * 夜间近光灯等非决策灯光。 */
            {
                uint8_t ts = g.ego_turn_signal.load(std::memory_order_relaxed);
                bool    hz = g.ego_hazard.load(std::memory_order_relaxed);
                ego.lights.clear();
                if (ts == 1)      ego.lights.set_turn_left(true);
                else if (ts == 2) ego.lights.set_turn_right(true);
                if (hz)           ego.lights.set_hazard(true);
            }
            /* ── off-rails 机动模式进入判定 ──
             * control 巡航钳位上限 0.16rad，只有掉头/倒车轨迹会命令 |steer|>0.28。
             * 进入后 ego.steer_override=true 放开物理层满舵限幅（0.25→0.60），
             * 且下方 Frenet 轨道位置覆盖整体跳过（见 off_rails 声明处注释）。 */
            {
                int8_t gear = g.ego_gear.load(std::memory_order_relaxed);
                if (!g.off_rails &&
                    (std::fabs(ego.steer) > 0.28 || gear == GEAR_REVERSE)) {
                    g.off_rails = true;
                    LOG_WARN("flowsim", "[OFFRAILS] enter: steer=%.3f gear=%d x=%.1f y=%.2f h=%.2f v=%.1f",
                             ego.steer, (int)gear, ego.x, ego.y, ego.heading, ego.speed);
                }
                ego.steer_override = g.off_rails;
            }
            /* EPS 转向执行器低通滤波：模拟电动助力转向惯性。
             * 倒车/机动（off-rails）时绕过此滤波器：掉头需要满舵迅速到位，
             * physics.cpp 的 update_steer 提供执行器层滤波（τ=0.15s）已是足够约束。 */
            {
                int8_t gear = g.ego_gear.load(std::memory_order_relaxed);
                if (gear != GEAR_REVERSE && !g.off_rails) {
                    /* prev_steer 用 g 结构（reset_runtime_state 场景切换清零），
                     * 不再用函数内 static —— 旧实现跨场景残留上一场景的 steer
                     * 滤波状态，重进场景首帧转向被旧值污染（2026-08 修复）。 */
                    const double eps_alpha = 0.4;  /* 滤波系数：0-1，越小滤波越强 */
                    double raw = ego.steer;
                    ego.steer = eps_alpha * raw + (1.0 - eps_alpha) * g.prev_steer;
                    g.prev_steer = ego.steer;
                }
            }
            if (strcmp(g.physics_model, "dynamic") == 0) {
                flowsim::step_bicycle_dynamic(ego, FLOWSIM_DT_SEC,
                                              ego.throttle, ego.brake, ego.steer);
            } else {
                flowsim::step_bicycle(ego, FLOWSIM_DT_SEC,
                                      ego.throttle, ego.brake, ego.steer);
            }
            /* 掉头对向车道标志（2026-08 修复）：由「车头方向」判定，不再用
             * lane_id 符号。旧判定 (lane_id > 0) 在"同向道路跨中心线"（4 车道
             * 同向 y 跨 ±5.25，OpenDRIVE lane_id 正=左）时把 y>0 的车道误判为
             * 对向 → ego 变道到超车道（y=+1.75）即触发返程逻辑 → ref_path 反向
             * + esmini 反向同步 → 启动即逆行（2026-08-03 实测 lane=1 逆行）。
             * 语义正确的判据：返程 = 车头朝 -x（heading≈±π，掉头完成后的
             * 行驶方向），与车道编号无关。 */
            if (!g.off_rails && ego.road_pos.ok()) {
                double hn = ego.heading;
                while (hn >  M_PI) hn -= 2.0 * M_PI;
                while (hn < -M_PI) hn += 2.0 * M_PI;
                g.u_turn_active = (std::fabs(hn) > M_PI * 0.5);
            }

            /* Phase 2: ego 用 road_pos 推进纵向 + set_offset 做横向变道。
             *
             * 两个模式都自由积分 heading（不做道路切线重置）：
             *   运动学模式由 step_bicycle 的自行车模型积分 heading，稳定性来自
             *     sin(dh) 负反馈闭环 + control 的 cte_term/heading_term/低通/死区
             *     （旧版曾每帧重置到道路切线，已改为不重置，见下方 is_dynamic
             *     分支只做归一化）；
             *   动力学模式由 step_bicycle_dynamic 的轮胎侧偏力积分自主演化 heading。
             *
             * 横向位移由 delta_lat 独立控制（与 heading 解耦）：
             *   delta_lat = speed * dt * tan(steer) * gain
             *   gain=1.0：tan(0.05) * 12 * 0.05 = 0.030 m/帧
             *   直路巡航时 steer≈0.02 → delta_lat≈0.012 m/帧 → 过冲可控
             *   变道 steer≈0.10 → delta_lat≈0.060 m/帧 → ~3.5s 完成车道变换 */
            bool is_dynamic = (strcmp(g.physics_model, "dynamic") == 0);
            if (g.off_rails) {
                /* ── off-rails：机动期（掉头/倒车）自行车模型是位姿唯一权威 ──
                 * 不做任何轨道位置覆盖（轨道覆盖 = 纯沿车头平移，丢掉
                 * step_bicycle 的 half_wb·yaw_rate 旋转项 → 车绕自身中心
                 * 原地旋转、"屁股横扫"）。每帧只把物理真值投影回 Frenet：
                 *   1. 遥测字段回写（road_id/lane_id/s/offset + u_turn_active）
                 *   2. relocate 让 handle 跟随（不回写 ego.x/y/heading）
                 *   3. 退出判定：转向需求消失 且 车头与车道方向对折角 <20° */
                bool exit_ok = false;
                /* 退出残差观测（迭代返程对齐用）：退出瞬间车头与车道方向的对折角
                 * 和到车道中心的横向偏移。残差大 = 掉头结束位姿差 → 之后 on-rails
                 * 段要靠 slew 原地转车身对齐（"屁股横扫"），或位置被 relocate 钉
                 * 回车道时产生横向跳变。 */
                double exit_fold_deg = 1e9;
                double exit_lat_m    = 1e9;
                if (g.roads_loaded) {
                    flowsim::FrenetPos fp;
                    if (g.roads.world_to_frenet(ego.x, ego.y, fp)) {
                        ego.road_id = fp.road_id;
                        ego.lane_id = fp.lane_id;
                        ego.s = fp.s;
                        ego.offset = fp.offset;
                        /* 同上方修复：车头方向判定（lane_id 符号在跨中心线
                         * 同向道路误判对向 → 启动即逆行） */
                        {
                            double hn = ego.heading;
                            while (hn >  M_PI) hn -= 2.0 * M_PI;
                            while (hn < -M_PI) hn += 2.0 * M_PI;
                            g.u_turn_active = (std::fabs(hn) > M_PI * 0.5);
                        }
                        exit_lat_m = fp.offset;
                        double ref_off = flowsim::offset_from_lane_internal(
                            g.roads, fp.road_id, fp.lane_id, fp.s, fp.offset);
                        if (ego.road_pos.relocate(g.roads, fp.road_id, 0, fp.s, ref_off)) {
                            flowsim::WorldPos wp;
                            if (ego.road_pos.world(wp)) {
                                double dh = ego.heading - wp.h;
                                while (dh >  M_PI) dh -= 2.0 * M_PI;
                                while (dh < -M_PI) dh += 2.0 * M_PI;
                                /* 对折角：与车道方向或其反向的最小夹角
                                 * （掉头终点在对向车道，heading ≈ 路切线 +π）。 */
                                double fold = std::min(std::fabs(dh),
                                                       M_PI - std::fabs(dh));
                                exit_fold_deg = fold * 180.0 / M_PI;
                                int8_t gear = g.ego_gear.load(std::memory_order_relaxed);
                                exit_ok = (gear != GEAR_REVERSE) &&
                                          (std::fabs(ego.steer) <= 0.28) &&
                                          (fold < 20.0 * M_PI / 180.0);
                            }
                        }
                    }
                }
                if (exit_ok) {
                    g.off_rails = false;
                    LOG_WARN("flowsim", "[OFFRAILS] exit: x=%.1f y=%.2f h=%.2f v=%.1f fold=%.1fdeg lat=%.2fm",
                             ego.x, ego.y, ego.heading, ego.speed, exit_fold_deg, exit_lat_m);
                } else if (g.cycle % 300 == 0) {
                    LOG_WARN("flowsim", "[OFFRAILS] active: x=%.1f y=%.2f h=%.2f v=%.1f steer=%.3f",
                             ego.x, ego.y, ego.heading, ego.speed, ego.steer);
                }
            } else if (ego.road_pos.ok()) {
                /* ⚠ 横向位移由 heading 与道路切线的夹角 dh 驱动，不是由 steer 直接驱动。
                 *
                 * 原 bug（3d092ad）：delta_lat = v * dt * tan(steer) 把转角当成了
                 * 横向速度，真车不是这样。两个后果：
                 *   1. 变道慢：v_lat = v*tan(0.047) = 0.56 m/s，3.5m 要 6 秒
                 *   2. MPC 模型失配（P1 级）：AutoMPC 内部用含侧偏角模型
                 *      x_dot = v*cos(θ+β), y_dot = v*sin(θ+β), β=atan(0.5*tan(δ))
                 *      θ_dot = v/L*cos(β)*tan(δ)，而 flowsim step_bicycle 用纯运动学
                 *      θ_dot = v/L*tan(δ), x_dot=v*cos(θ)。AutoMPC 是代码生成器产物，
                 *      QP 求解器矩阵与模型绑定，单独改前向积分公式会破坏内部一致性。
                 *      MPC 依赖反馈校正补偿此失配，影响预测精度但不影响稳定性。
                 *
                 * 模型：转角 → heading（step_bicycle 自由积分）→ 横向速度，
                 * delta_lat = v * dt * sin(ego.heading - wp.h)
                 * sin(dh) 本身构成负反馈：heading 偏 → 横向漂 → 控制器看到
                 * lat_error → 回打 → heading 收回来，闭环自洽。 */
                double dist = ego.speed * FLOWSIM_DT_SEC;
                bool advanced = false;
                /* 前进帧开始的「车头 − 道路切线」夹角 dh0（一次算好，advance 与
                 * 横向共用）：
                 *   沿道路推进 = dist·cos(dh0) —— 车沿车头方向的分量投影到路上。
                 *                旧版 advance(满 dist) 让车沿路"斜滑"：车头偏角
                 *                dh 大时运动方向 atan(sin dh) 滞后车头、掉头转不过
                 *                去（py-sim 量化：dh=45° 斜走 9.7°，60° 斜走 19°）。
                 *   横向偏移   = dist·sin(dh0) —— 车头偏角产生的横向移动。
                 * 净位移 = dist·(cos·路切 + sin·法向) = 沿车头方向 dist（贴路约束），
                 * 与 step_bicycle 的世界系积分一致，消除"斜着直行"。 */
                double dh0 = 0.0;
                {
                    flowsim::WorldPos wp0;
                    if (ego.road_pos.world(wp0)) {
                        dh0 = ego.heading - wp0.h;
                        while (dh0 >  M_PI) dh0 -= 2.0 * M_PI;
                        while (dh0 < -M_PI) dh0 += 2.0 * M_PI;
                    }
                }
                if (dist > 0.0) {
                    /* 物理掉头后：对向车道行驶时 esmini advance 方向（+s）
                     * 与车辆实际行驶方向相反。改用 world_to_frenet 从物理
                     * 模型的世界坐标同步 esmini position。 */
                    if (g.u_turn_active && g.roads_loaded) {
                        flowsim::FrenetPos fp;
                        if (g.roads.world_to_frenet(ego.x, ego.y, fp)) {
                            /* 掉头返程段（2026-08 架构收口）：
                             * 位置 = step_bicycle 积分（物理真值，与 HTML 车辆
                             * 实验室一致）。旧实现 relocate 读回 ego.x/y 把位置
                             * 钉到参考线 + heading slew 对齐 —— 位置被轨道拽、
                             * heading 原地转 → "屁股横扫/大移动"（用户实测
                             * 掉头屁股大移动）。现在 relocate 仅同步 esmini
                             * handle（碰撞/路网查询），位置/heading 保持
                             * step_bicycle 自由积分，车道对齐由 control 横向
                             * 控制完成（与正常模式一致；Phase 4 收严后掉头
                             * 结束残差 <6°，无"斜着直行"）。 */
                            double ref_off = flowsim::offset_from_lane_internal(
                                g.roads, fp.road_id, fp.lane_id, fp.s, fp.offset);
                            if (ego.road_pos.relocate(g.roads, fp.road_id, 0, fp.s, ref_off)) {
                                advanced = true;
                                /* 字段从 world_to_frenet 的 fp 直接读（权威）；
                                 * 不从 relocate 后的 handle 读（参考线上 lane_id 恒 0）。 */
                                ego.road_id = fp.road_id;
                                ego.lane_id = fp.lane_id;
                                ego.s = fp.s;
                                ego.offset = ref_off;
                            }
                        }
                        /* world_to_frenet 失败时回退到 esmini advance */
                        if (!advanced) {
                            advanced = ego.road_pos.advance(dist, M_PI);
                        }
                    } else {
                        /* 正常模式：位置 = step_bicycle 积分（物理真值，架构
                         * 收口 2026-08）。旧实现 advance+set_offset 后把
                         * ego.x/y 覆盖为 esmini 轨道位置 —— 车的位置由"参考线
                         * 推进 + 手填横向偏移"决定而非车模型 → 变道 = 轨道
                         * 拖动（"车屁股平移"的根因）。现在位置姿态单一来源
                         * = 自行车模型（GTA 式：一个积分器算位置+朝向）；
                         * 车道保持由 control 的 Stanley 横向控制完成。
                         * esmini 仅做投影查询：车道归属/限速/碰撞。 */
                        advanced = false;
                    }
                    if (true) {
                        /* esmini handle 每帧 relocate 跟随物理位置（碰撞/路网
                         * 查询用；物理位置本身不依赖 handle）。车道归属用物理
                         * 位置的 world_to_frenet 投影（比 handle 的轨道位置准，
                         * 车在物理位置而非轨道上）。 */
                        flowsim::FrenetPos fp;
                        if (g.roads_loaded && g.roads.world_to_frenet(ego.x, ego.y, fp)) {
                            ego.road_id = fp.road_id;
                            ego.lane_id = fp.lane_id;
                            ego.s = fp.s;
                            ego.offset = fp.offset;
                            double ref_off = flowsim::offset_from_lane_internal(
                                g.roads, fp.road_id, fp.lane_id, fp.s, fp.offset);
                            ego.road_pos.relocate(g.roads, fp.road_id, fp.lane_id, fp.s, ref_off);
                        }
                    }
                }
                /* ── 物理掉头循环 ──────────────────────────────────────────
                 * 判据根据当前车道方向区分：
                 *   前进车道（lane_id < 0）：前方施工区前缘之前（无施工则路末 s≈road_len）
                 *                            掉头到对向车道
                 *   对向车道（lane_id > 0）：起点（s ≈ 0）掉头回前进车道
                 * 对向车道行驶时 esmini advance 方向与车辆行驶方向相反，
                 * 由上方 world_to_frenet 同步块处理。 */
                /* U-turn 触发已迁移到 behavior_planner → planning → control 链路。
             * flowsim 只执行控制指令（含 gear），不再做掉头决策。 */
            }

            /* ── Step 2: 场景事件预检查（让 NPC 知道前方红绿灯/ETC） ── */
            flowsim::check_npc_scene_events(g.pool, g.ai_cfg.look_ahead, g.ai_cfg);

            /* ── Step 2.5: 工况脚本（Task 3）—— 触发器评估 + actor_overrides 应用 ──
             * 必须在 NPC AI 之前：CutIn 触发后 set ai_state+target_offset，
             * 同 tick step_npc_vehicle 看到 ai_state==CutIn 即进入 PID 横向控制，
             * 实现一次触发即生效（无 1-tick 延迟）。 */
            double pre_time_s = (double)(clock_now_us() - g.sim_start_us) / 1e6;
            apply_scenario_scripts(pre_time_s);

            /* ── Step 3: NPC AI ── */
            flowsim::FlowRoadNetwork* roads_ptr = g.roads_loaded ? &g.roads : nullptr;
            const flowsim::Route* route_ptr =
                (g.roads_loaded && g.route.ok()) ? &g.route : nullptr;
            /* ego 在 route 上的累计 s：回收 NPC 时用来放到 ego 附近，形成持续车流。
             * 用 compute_ego_route_s 复用 road_pos handle，避免每帧 world_to_frenet 全网扫描 */
            double ego_route_s = compute_ego_route_s();
            if (ego_route_s < 0.0) ego_route_s = 0.0;  /* 不在 route 上时回退到起点 */
            for (int i = 1; i < g.pool.size(); i++) {
                flowsim::Entity& e = g.pool[i];
                if (!e.active) continue;
                if (e.is_npc_vehicle()) {
                    flowsim::step_npc_vehicle(e, g.pool, FLOWSIM_DT_SEC,
                                              g.ai_cfg, roads_ptr, route_ptr, ego_route_s,
                                              g.cycle, g.off_rails);
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
                    /* 碰撞分离是位置跳变，标记 teleport 供 invariant 跳过 Δpos 检查 */
                    if (g.pool[p.a].is_vehicle()) g.pool[p.a].last_teleport_cycle = g.cycle;
                    if (g.pool[p.b].is_vehicle()) g.pool[p.b].last_teleport_cycle = g.cycle;
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
            /* 编舞循环：每 loop_period_s 重置 actor 到 ego 附近，实现反复演示 */
            if (g.scenario) {
                flowsim::tick_choreography(g.pool, ego, sim_time_s, FLOWSIM_DT_SEC,
                                           &g.scenario->choreography,
                                           g.roads_loaded ? &g.roads : nullptr,
                                           g.route.ok() ? &g.route : nullptr,
                                           g.cycle);
            }

            /* ── Step 5.5: 车灯信号派生 ──
             * 从 ego 的 steer/brake/speed 和 NPC 的 ai_state 派生车灯位掩码，
             * 写入 Entity.lights。scene_pub 序列化为 JSON "lights" 字段传给前端。
             * 在 scene_pub 之前调用，确保当前帧 lights 已更新。 */
            flowsim::VehicleActor::update_all_lights(g.pool, sim_time_s);

            /* ── Step 6: 推进逻辑时钟 ── */
            clock_advance_us(FLOWSIM_DT_US);
            uint64_t sim_time_us = clock_now_us();

            /* ── Step 7: 发布 ── */
            publish_sim_tick(sim_time_us);
            if (should_publish_road_geometry_now()) {
                publish_road_geometry();
            }
            /* road/ref_path：ego 前方参考路径，每 cycle 发布给 control_node Stanley
             * 横向控制消费；无 route 时发布空数组（control_node 回退到 curve_*）。 */
            publish_ref_path();
            publish_traffic_lights();
            publish_vehicle_state(sim_time_us);
            /* scene/frame：完整场景帧 60Hz 给 3D 前端（Phase 2.2） */
            flowsim::publish_scene_frame(g.transport, g.pool, g.scene_pub_cfg,
                                         sim_time_us, g.cycle);

            /* 固定 60Hz 节拍：原 select_for 的消息/超时唤醒由 BusQueueBridge
             * + 固定周期取代——主循环不依赖消息到达即可稳定推进。
             *
             * 自适应 sleep：减去本帧工作时间，维持稳定 60Hz。
             * 旧代码 sleep_us(FLOWSIM_DT_US) 固定 50ms，不扣除工作时间，
             * 实际帧率 = 1/(T_work + 50ms)，T_work 越大帧率越低——
             * 场景跑到一半 T_work 从 5ms 涨到 55ms 时 FPS 从 18 跌到 9.5。 */
            uint64_t t_frame_us = clock_now_monotonic_wall_us() - t_start;
            uint64_t sleep_us_val = FLOWSIM_DT_US;
            if (t_frame_us < sleep_us_val) {
                sleep_us_val = FLOWSIM_DT_US - t_frame_us;
            } else {
                sleep_us_val = 0;  /* 帧超时：不休眠，下一帧立即开始追 */
            }
            if (g.cycle % 600 == 0) {
                LOG_INFO("flowsim", "[PERF] cycle=%u frame_time=%llu us (%.1f ms) sleep=%llu us",
                         g.cycle, (unsigned long long)t_frame_us,
                         (double)t_frame_us / 1000.0,
                         (unsigned long long)sleep_us_val);
            }
            co_await sleep_us(sleep_us_val);

            g.cycle++;
            if (g.cycle % 600 == 0) {
                uint64_t last_cmd = g.last_control_cmd_us.load(std::memory_order_relaxed);
                double cmd_age_ms = last_cmd > 0
                    ? (double)(clock_now_us() - last_cmd) / 1000.0 : -1.0;
                TopicStats cmd_stats;
                int sc = message_bus_get_topic_stats(bus(), TOPIC_CONTROL_CMD, &cmd_stats);
                LOG_INFO("flowsim", "#%u ego(%.1f,%.1f) spd=%.1f thr=%.2f brk=%.2f st=%.3f npc=%d cruise=%d cmd_age=%.0fms cb=%llu take=%llu subs=%d",
                         g.cycle, ego.x, ego.y, ego.speed,
                         ego.throttle, ego.brake, ego.steer,
                         g.pool.active_count() - 1, use_internal_cruise ? 1 : 0,
                         cmd_age_ms,
                         (unsigned long long)cmd_bridge.cb_count,
                         (unsigned long long)cmd_bridge.take_count,
                         sc == 0 ? (int)cmd_stats.subscriber_count : -1);
            }
            /* 低速急刹诊断：speed>2 且 brake>0.5 且速度未降 → 每 50 帧打一次，
             * 抓"指令全刹但物理速度不掉"的执行断点（2026-07 追尾事故）。
             * 旧频率每 5 帧（250ms）过于密集，配合 fflush 阻塞主循环。 */
            if (ego.speed > 2.0 && ego.brake > 0.5 && g.cycle % 150 == 0) {
                LOG_WARN("flowsim", "[BRK] cyc=%u spd=%.2f thr=%.2f brk=%.2f last_cmd_us_age=%.0fms",
                         g.cycle, ego.speed, ego.throttle, ego.brake,
                         g.last_control_cmd_us.load(std::memory_order_relaxed) > 0
                             ? (double)(clock_now_us() - g.last_control_cmd_us.load(
                                   std::memory_order_relaxed)) / 1000.0
                             : -1.0);
            }

            /* ── 仿真基础层：每帧 digest + invariant 检查 ──
             * 每 20 帧（~1s）跑一次完整 invariant，避免每帧序列化开销过大。
             * 时序 invariant 需要连续两帧，从第 2 帧开始。 */
            if (g.cycle % 60 == 0 && g.digest_initialized) {
                /* ego_maneuver：掉头机动期豁免 lane-keeping invariant
                 * （倒车/横穿/heading 扫过 ±π 是机动路径本身，非"横着/倒着开"故障）。
                 * 2026-08-04 多把方向掉头后豁免窗口扩宽：
                 *   旧 = u_turn_active（heading 派生，|hn|>π/2）—— Phase 0 腾挪
                 *   倒车（heading≈0）与返程倒车起始段（|hn|<π/2）漏豁免 → 误报
                 *   dot=-1.000/-0.946（实测 540s 长跑 motion_direction 2 次 FAIL）。
                 *   掉头倒车是仿真里唯一的倒车场景（GEAR_REVERSE 只由掉头轨迹
                 *   触发），故 gear==REVERSE 并入豁免 = 覆盖整个掉头执行窗口。 */
                const int8_t gear = g.ego_gear.load(std::memory_order_relaxed);
                if (g.u_turn_active || g.off_rails || gear == GEAR_REVERSE) {
                    g.ego_maneuver_grace_until = g.cycle + 120;  // ~2s at 60Hz
                }
                bool ego_maneuver = (g.cycle <= g.ego_maneuver_grace_until);
                auto dd = flowsim::build_dynamic_digest(g.pool, sim_time_s, (int)g.cycle, true,
                                                        ego_maneuver);
                // 空间 invariant
                auto spatial = flowsim::check_spatial_invariants(dd, g.static_digest,
                    g.roads_loaded ? &g.roads : nullptr);
                /* 仅失败时输出到 stderr；每帧计数用 DEBUG 级别避免刷屏。 */
                if (spatial.failed > 0) {
                    g.invariant_fail_count.fetch_add(spatial.failed, std::memory_order_relaxed);
                    if (!spatial.details.empty()) {
                        fprintf(stderr, "[INV] spatial_invariant cycle=%u\n%s",
                                g.cycle, spatial.details.c_str());
                    }
                }
                // 运动方向 invariant
                auto motion = flowsim::check_motion_direction(dd, g.static_digest,
                    g.roads_loaded ? &g.roads : nullptr);
                if (motion.failed > 0) {
                    g.invariant_fail_count.fetch_add(motion.failed, std::memory_order_relaxed);
                    if (!motion.details.empty()) {
                        fprintf(stderr, "[INV] motion_direction cycle=%u\n%s",
                                g.cycle, motion.details.c_str());
                    }
                }
                // 时序 invariant（需要上一帧）
                if (g.prev_dynamic_digest.actors.size() > 0) {
                    /* 用实际 sim_time 差值作为 dt，避免帧率波动导致
                     * Δpos/accel 检查误报。旧代码用固定 FLOWSIM_DT_SEC*20=1.0s，
                     * 但实际帧间隔可能因系统负载偏移。 */
                    double inv_dt = dd.sim_time - g.prev_dynamic_digest.sim_time;
                    if (inv_dt < 0.01) inv_dt = FLOWSIM_DT_SEC * 60;  // fallback（digest 周期 = 60 cycle ≈ 1s）
                    auto temporal = flowsim::check_temporal_invariants(
                        g.prev_dynamic_digest, dd, inv_dt);
                    if (temporal.failed > 0) {
                        g.invariant_fail_count.fetch_add(temporal.failed, std::memory_order_relaxed);
                        if (!temporal.details.empty()) {
                            fprintf(stderr, "[INV] temporal_invariant cycle=%u\n%s",
                                    g.cycle, temporal.details.c_str());
                        }
                    }
                }
                // ASCII 俯视图：3D 运行时自动生成，写到 /tmp/flow_ascii_overhead.txt
                // 供 dashboard / 终端 cat 查看。每 100 帧（5s）更新一次，
                // 避免高频文件 I/O 阻塞主循环。
                if (g.cycle % 300 == 0) {
                    std::string ascii = flowsim::render_ascii_overhead(g.static_digest, dd, 80, 40);
                    FILE* fp = fopen("/tmp/flow_ascii_overhead.txt", "w");
                    if (fp) {
                        fputs(ascii.c_str(), fp);
                        fclose(fp);
                    }
                }
                g.prev_dynamic_digest = std::move(dd);
            } else if (!g.digest_initialized && g.cycle == 1) {
                // 第一帧：初始化 prev_digest 供后续时序检查
                g.prev_dynamic_digest = flowsim::build_dynamic_digest(g.pool, sim_time_s, 0, true);
                g.digest_initialized = true;
            }
        }

        LOG_INFO("flowsim", "stopped (%u cycles, sim_time=%.3fs, final speed=%.1f)",
                 g.cycle, (double)clock_now_us() / 1e6, ego.speed);
    }

private:
    Transport* transport_;
};

/* ── TaskBase 包装器（宏生成）— 必须在 flowsim_init 前展开 ──────── */
EXPORT_COROUTINE_TASK(FlowSimTask, flowsim)

/* ── NodePlugin 实现 ─────────────────────────────────────────── */

static const char* s_inputs[]  = { TOPIC_CONTROL_CMD, nullptr };
static const char* s_outputs[] = {
    TOPIC_VEHICLE_STATE, TOPIC_ROAD_GEOMETRY, TOPIC_ROAD_TRAFFIC_LIGHTS,
    TOPIC_ROAD_REF_PATH,
    TOPIC_SIM_TICK, TOPIC_SIM_COLLISION, TOPIC_SCENE_FRAME, nullptr
};

extern NodePlugin s_plugin;

static int flowsim_init(MessageBus* bus, Transport* transport,
                        DiscoveryManager* discovery, Scheduler* scheduler,
                        const char* params_json) {
    g.transport  = transport;
    g.discovery  = discovery;
    g.scheduler  = scheduler;
    g.cycle      = 0;

    /* 运行时状态重置必须在配置解析之前：reset 会清 scenario_file/
     * physics_model/init_speed 等配置字段（它们是"输入"，由 params_json
     * 重新注入）。旧接线放在场景加载前（配置解析后）→ scenario_file 被
     * 清空 → scenario_load 跳过 → 默认空场景（actors=0，NPC 全灭，
     * 2026-08-03 实测回归）。重复 init 时这里同样先清旧状态再解析。 */
    reset_runtime_state();

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
            if ((j = cJSON_GetObjectItemCaseSensitive(p, "physics_model")) && cJSON_IsString(j)) {
                strncpy(g.physics_model, j->valuestring, sizeof(g.physics_model) - 1);
                if (strcmp(g.physics_model, "dynamic") == 0) {
                    LOG_INFO("flowsim", "physics_model=dynamic selected "
                             "(线性轮胎二自由度；<5m/s 退化运动学，见 CALIBRATION_GUIDE.md)");
                }
            }
            cJSON_Delete(p);
        }
    }

    /* 加载场景文件（reset_runtime_state 已在 flowsim_init 开头调用，
     * 此时 scenario_file 由 params_json 解析注入完毕） */
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

    /* NPC 行为开关：启用 MOBIL 自主变道 */
    g.ai_cfg.enable_mobil = g.scenario->npc_lane_change;

    srand(g.random_seed);

    /* JSON → xodr → esmini RoadManager */
    if (g.scenario_file[0] != '\0') {
        std::string xodr = convert_scenario_to_xodr(g.scenario_file);
        if (!xodr.empty()) {
            if (g.roads.load(xodr)) {
                g.roads_loaded = true;
                LOG_INFO("flowsim", "esmini road network loaded: %d roads",
                         g.roads.road_count());
                if (g.route.build(g.roads)) {
                    LOG_INFO("flowsim", "central route built: %d segments, %.0fm total",
                             g.route.count(), g.route.total_length());
                } else {
                    LOG_WARN("flowsim", "route build failed — NPC lane-follow off (straight fallback)");
                }
            } else {
                LOG_WARN("flowsim", "esmini load failed for %s — NPC AI falls back to lateral distance",
                         xodr.c_str());
                /* 离线排查指引：逐段验证链路（对齐 pipeline_check.py 的离线哲学）。
                 * 1) 手动生成 xodr 看报错：python3 tools/json_to_xodr.py <scenario> -o /tmp/t.xodr
                 * 2) 离线自检：cd build/modules/adas_nodes && ctest -R road_network
                 * 3) 预构建库格式：file lib/libesminiRMLib.so 必须是 ELF（Mach-O 交叉提交会晦涩失败） */
                LOG_WARN("flowsim",
                         "esmini 排查: 1) 手动转 xodr: python3 tools/json_to_xodr.py %s -o /tmp/t.xodr"
                         "  2) 离线自检: ctest -R road_network (build/modules/adas_nodes)"
                         "  3) 库格式: file lib/libesminiRMLib.so 应为 ELF",
                         g.scenario_file);
            }
        }
    }

    /* 填充 EntityPool（ego + actors + 红绿灯 + ETC 门架）。
     * 必须在 build_static_digest 之前——digest 的 traffic_lights 字段从 pool
     * 提取（位置/朝向/受控车道），早于 populate 会导致 sd.traffic_lights 为空，
     * ASCII 渲染看不到灯杆，闭环调试失效。 */
    populate_entities_from_scenario(g.scenario);

    /* 仿真基础层：几何变更时建一次静态 digest。
     * 放在 populate_entities_from_scenario 之后，使 traffic_light 实体的
     * 位置/朝向能被 build_static_digest 提取到 TrafficLightDigest.x/y/heading，
     * 供 ASCII 俯视渲染（render_ascii_overhead）画 G/Y/R 字符 + 静态 invariant
     * 检查（check_static_invariants）校验灯杆朝向。 */
    if (g.roads_loaded) {
        g.static_digest = flowsim::build_static_digest(g.roads, g.route, g.pool);
        LOG_INFO("flowsim", "static digest: %zu lanes, %zu markings, %zu traffic_lights",
                 g.static_digest.lanes.size(),
                 g.static_digest.markings.size(),
                 g.static_digest.traffic_lights.size());
        /* 静态 invariant：车道宽/边界自洽/标线/红绿灯朝向等 */
        auto static_inv = flowsim::check_static_invariants(g.static_digest);
        /* A-4b: 始终打印 passed/failed/warned 计数——旧行为只在 failed>0 时打印，
         * 让"所有检查都 passed"的正常路径看不到任何输出，无法判断 invariant
         * 是否真的在跑。改为无条件 LOG_INFO，失败时再补 LOG_WARN + stderr。 */
        LOG_INFO("flowsim", "static_invariant: %d passed, %d failed, %d warned",
                static_inv.passed, static_inv.failed, static_inv.warned);
        if (static_inv.failed > 0) {
            g.invariant_fail_count.fetch_add(static_inv.failed, std::memory_order_relaxed);
            if (!static_inv.details.empty()) {
                fprintf(stderr, "[flowsim::static_invariant]\n%s", static_inv.details.c_str());
            }
        }
    }

    /* 仿真时钟：逻辑时间从 0 开始步进 */
    clock_set_sim_mode(true);
    clock_set_sim_time(0);
    clock_set_step_us(FLOWSIM_DT_US);
    g.sim_start_us = 0;  /* sim 时间从 0 起，sim_start_us=0 使 sim_time_s = clock_now_us/1e6 */

    /* 订阅 control/cmd */
    transport_subscribe(transport, TOPIC_CONTROL_CMD, on_control_cmd, nullptr);
    discovery_advertise(discovery, TOPIC_CONTROL_CMD, CONTROL_CMD_TYPE_ID, CAP_SUBSCRIBER, 0);

    /* 广告输出 topics */
    transport_advertise(transport, TOPIC_VEHICLE_STATE,       VEHICLE_STATE_TYPE_ID);
    discovery_advertise(discovery, TOPIC_VEHICLE_STATE,       VEHICLE_STATE_TYPE_ID, CAP_PUBLISHER, 20.0);
    transport_advertise(transport, TOPIC_ROAD_GEOMETRY,       ROAD_GEOMETRY_TYPE_ID);
    discovery_advertise(discovery, TOPIC_ROAD_GEOMETRY,       ROAD_GEOMETRY_TYPE_ID, CAP_PUBLISHER, 1.0);
    transport_advertise(transport, TOPIC_ROAD_TRAFFIC_LIGHTS, ROAD_TRAFFIC_LIGHTS_TYPE_ID);
    transport_advertise(transport, TOPIC_SIM_TICK,            SIM_TICK_TYPE_ID);
    transport_advertise(transport, TOPIC_SIM_COLLISION,       SIM_COLLISION_TYPE_ID);
    /* scene/frame：60Hz 完整场景帧，给 3D 前端用（Phase 2.2 新增） */
    transport_advertise(transport, TOPIC_SCENE_FRAME,         SCENE_FRAME_TYPE_ID);
    discovery_advertise(discovery, TOPIC_SCENE_FRAME,         SCENE_FRAME_TYPE_ID, CAP_PUBLISHER, 20.0);

    /* 填充 scene_pub_cfg：roads_loaded 之后才有 esmini 网络指针 */
    g.scene_pub_cfg.curve_start_x  = g.curve_start_x;
    g.scene_pub_cfg.curve_length_m = g.curve_length_m;
    g.scene_pub_cfg.curve_offset_m = g.curve_offset_m;
    g.scene_pub_cfg.lane_width     = g.lane_width;
    g.scene_pub_cfg.lane_count     = 2;
    g.scene_pub_cfg.roads          = g.roads_loaded ? &g.roads : nullptr;
    g.scene_pub_cfg.type_id        = SCENE_FRAME_TYPE_ID;
    /* Task 4：把场景 JSON 的 lighting 字段透传到 scene/frame topic，
     * 前端 scene3d.js 据此调整 AmbientLight/DirectionalLight/Bloom 阈值。 */
    g.scene_pub_cfg.lighting       = (int)g.scenario->lighting;
    /* 道路类型：从 road_network.edges[0].type 提取（如 viaduct_highway），
     * 用于前端识别场景类型并选择对应的渲染模式。 */
    if (g.scenario->road.type[0]) {
        g.scene_pub_cfg.road_type = g.scenario->road.type;
    }

    /* 施工区：把 scenario 定义拷入 scene_pub_cfg，每帧透传给前端渲染
     * （后端单一事实源，取代前端"道路末端 30m"自算逻辑）。 */
    g.scene_pub_cfg.construction_zones.clear();
    for (int z = 0; z < g.scenario->construction_zone_count; ++z) {
        const ScenarioConstructionZone* cz = &g.scenario->construction_zones[z];
        flowsim::ScenePubConstructionZone pz;
        pz.id     = cz->id;
        pz.x      = cz->x;
        pz.y      = cz->y;
        pz.length = cz->length;
        pz.width  = cz->width;
        g.scene_pub_cfg.construction_zones.push_back(pz);
    }

    /* 构造协程任务（托管模式） */
    TaskConfig tcfg = {};
    snprintf(tcfg.name, sizeof(tcfg.name), "flowsim");
    tcfg.priority = TASK_PRIORITY_NORMAL;
    g.task_wrapper = flowsim_create(&tcfg, bus);
    if (!g.task_wrapper) return -1;
    g.task_wrapper->impl->set_params(transport);
    s_plugin.taskbase = flowsim_get_base(g.task_wrapper);

    LOG_INFO("flowsim", "initialized (scenario=%s, actors=%d, traffic_lights=%d, esmini=%s)",
             g.scenario_file[0] ? g.scenario_file : "(default)",
             g.scenario->actor_count,
             g.scenario->traffic_light_count,
             g.roads_loaded ? "on" : "off");
    return 0;
}

static int flowsim_start(void) {
    if (!g.task_wrapper) return -1;
    int rc = node_start_managed(&s_plugin, g.scheduler);
    if (rc != 0) {
        LOG_WARN("flowsim", "node_start_managed failed: %d", rc);
    }
    node_announce_self(g.transport, &s_plugin);
    LOG_INFO("flowsim", "started (managed mode)");
    return 0;
}

static void flowsim_stop(void) {
    if (g.task_wrapper) {
        flowsim_stop(&g.task_wrapper->base);  /* 宏生成：设 impl->set_stop() */
    }
}

static void flowsim_cleanup(void) {
    /* 先销毁包装器（task_start 创建的线程在 execute 返回后自动退出，
     * flowsim_destroy 会 delete impl + free 包装器内存） */
    if (g.task_wrapper) {
        flowsim_destroy(g.task_wrapper);
        g.task_wrapper = nullptr;
    }
    s_plugin.taskbase = nullptr;
    /* P2-7: invariant 失败汇总 marker。demo_evaluator.py 扫描此 marker
     * 把 invariant 失败升级为 FAIL。demo.sh 实时监控也通过 [INV] 标签
     * 展示失败详情。此处打印汇总（含 total=0 的正常路径，便于确认 invariant
     * 确实跑了）。 */
    uint32_t inv_fails = g.invariant_fail_count.load(std::memory_order_relaxed);
    fprintf(stderr, "[INV] summary total=%u (spatial+motion+temporal)\n", inv_fails);
    fflush(stderr);
    /* 在 RM_Close 之前释放所有 entity 的 road_pos handle。
     * g.pool/Entity 是静态对象，dlclose 后它们的 dtor 还会跑一次，
     * 若 RoadPosition::handle_ 仍有效，dtor 调 RM_DeletePosition 时
     * RM 已经 close → crash (SIGABRT)。先把所有 handle 清零，dtor
     * 就变成 no-op。 */
    for (int i = 0; i < flowsim::MAX_ENTITIES; ++i) {
        g.pool[i].road_pos = flowsim::RoadPosition{};
    }
    g.roads.release();
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
    nullptr,  /* taskbase: 在 init() 中通过 flowsim_create 设置 */
};

} // namespace

extern "C" NodePlugin* node_get_plugin(void) { return &s_plugin; }
