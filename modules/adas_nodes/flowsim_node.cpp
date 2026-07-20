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
#include "flowsim/route.h"

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
static flowsim::AIState ai_state_from_str(const char* s) {
    if (!s || !s[0]) return flowsim::AIState::Cruise;
    if (strcmp(s, "follow") == 0)       return flowsim::AIState::Follow;
    if (strcmp(s, "stop") == 0)         return flowsim::AIState::Stop;
    if (strcmp(s, "stop_for_tl") == 0)  return flowsim::AIState::StopForTL;
    if (strcmp(s, "etc_approach") == 0) return flowsim::AIState::ETCApproach;
    if (strcmp(s, "branch_sel") == 0)   return flowsim::AIState::BranchSel;
    if (strcmp(s, "merge") == 0)        return flowsim::AIState::Merge;
    if (strcmp(s, "yield") == 0)        return flowsim::AIState::Yield;
    if (strcmp(s, "cutin") == 0)        return flowsim::AIState::CutIn;
    return flowsim::AIState::Cruise;
}

static flowsim::Entity* find_entity_by_actor_id(int actor_id) {
    for (int i = 0; i < g.pool.size(); ++i) {
        flowsim::Entity& e = g.pool[i];
        if (e.active && e.id == actor_id) return &e;
    }
    return nullptr;
}

static void apply_actor_override(flowsim::Entity& e,
                                 const ScenarioActorOverride* o) {
    if (o->ai_state[0]) {
        e.ai_state = ai_state_from_str(o->ai_state);
        /* 进入 CutIn 时初始化 PID 状态，避免残留旧值影响新变道 */
        if (e.ai_state == flowsim::AIState::CutIn) {
            e.cutin_pid_integral = 0.0;
            e.cutin_pid_prev = 0.0;
            e.cutin_active = true;
        }
    }
    if (!isnan(o->target_offset)) {
        e.target_offset = o->target_offset;
    }
    if (!isnan(o->target_vx)) {
        e.target_vx = o->target_vx;
    }
    if (!isnan(o->vx)) e.vx = o->vx;
    if (!isnan(o->vy)) e.vy = o->vy;
}

static void apply_scenario_scripts(double sim_time_s) {
    if (!g.scenario || g.scenario->script_count <= 0) return;
    const flowsim::Entity& ego = g.pool[0];
    /* 预算 ego route_s（route_s_gte 触发器用） */
    double ego_route_s = -1.0;
    if (g.route.ok() && g.roads_loaded) {
        flowsim::FrenetPos ef;
        if (g.roads.world_to_frenet(ego.x, ego.y, ef)) {
            int ei = g.route.index_of(ef.road_id);
            if (ei >= 0) ego_route_s = g.route.to_route_s(ei, ef.s);
        }
    }
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
        e.id = a->id;

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
            /* 旧格式：全局 x/y 坐标 + 弯道中心线偏移 */
            e.x = a->x;
            if (a->vy == 0.0) {
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
        }
        /* 中央 route 初始化：新格式 actor + esmini + route 就绪时，让 NPC 车辆
         * 沿车道 Frenet 行驶（过弯/爬匝道自动跟随）。否则 route_dir 保持 0 走旧逻辑。 */
        if (a->segment_id >= 0 && g.roads_loaded && g.route.ok() && e.is_npc_vehicle()) {
            e.road_id = a->segment_id;
            e.s       = a->s;
            e.offset  = a->l;
            e.target_offset = a->l;  /* E2: 换道目标 offset 初始 = 当前 offset */
            flowsim::npc_init_route(e, g.route, (a->vx < 0.0) ? -1 : 1);
        }

        /* Phase 2: NPC 持久 road_pos 初始化（与 route 共存）。
         * 仅对新格式 (segment_id≥0) NPC init：直接用 segment_id/lane_id=0/s/l，
         * 与现有 NPC frenet_to_world(rid, 0, s, npc.offset) 调用约定一致
         * （NPC 用 lane_id=0 + offset_from_reference_line 模型，offset 是相对
         * 参考线的横向位置，非 lane 内 offset）。
         * 旧格式 (segment_id<0) NPC 不 init road_pos：world_to_frenet 返回的
         * (lane_id, offset_within_lane) 是 Model B 语义，与 NPC 的 Model A
         * (lane_id=0 + offset_from_ref) 不一致；且旧格式对向车 route_dir=0
         * 会导致 step5 航向翻转缺失。让旧格式 NPC 走 world_to_frenet 兜底更安全。
         * 失败时 road_pos.ok()==false，npc_ai 走旧 route/世界系兜底。 */
        if (g.roads_loaded && a->segment_id >= 0 && e.is_npc_vehicle()) {
            if (!e.road_pos.init(g.roads, a->segment_id, 0, a->s, a->l) && a->id <= 2) {
                LOG_WARN("flowsim", "NPC %d road_pos.init failed — fallback to route/world logic",
                         a->id);
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
     * 算 road_half_width；非 finite/esmini 不可用时再 fallback 硬编码。
     * 此外场景 traffic_lights[*].x 可能是手填的「近似世界坐标」（中凯路 x=770
     * 实际应在路口中心 x=560）——esmini 加载时用 world_to_frenet 校正 x/y，
     * 找到最近的 (road_id, s, lane_id, offset) 再 frenet_to_world 反算真值。
     * 这样无论场景 JSON 用相对值还是手填世界坐标，灯杆永远落在正确车道。 */
    {
        /* 缓存：每个 (road_id, s) 位置处的 road_half_width，避免重复 esmini 调用 */
        double cached_half_width = -1.0;
        for (int i = 0; i < sc->traffic_light_count && i < SCENARIO_MAX_TRAFFIC_LIGHTS; i++) {
            const ScenarioTrafficLight* tl = &sc->traffic_lights[i];
            flowsim::EntityId id = g.pool.alloc(flowsim::EntityType::TrafficLight);
            if (id == flowsim::INVALID_ENTITY) break;
            flowsim::Entity& e = g.pool[id];
            e.id = tl->id;
            e.throttle = tl->green_s;
            e.brake    = tl->yellow_s;
            e.steer    = tl->red_s;
            e.target_vx = tl->phase_offset_s;
            e.heading = tl->heading;
            if (e.heading == 0.0) {
                e.heading = compute_road_heading_at(tl->x, tl->y_lane);
            }
            /* esmini 接管世界坐标：world_to_frenet → 反算真值 */
            if (g.roads_loaded) {
                flowsim::FrenetPos fp;
                /* 用 y_lane 作横向偏移（与 x 一起试探路网最近点） */
                double probe_y = tl->y_lane;
                if (g.roads.world_to_frenet(tl->x, probe_y, fp)) {
                    flowsim::WorldPos wp;
                    if (g.roads.frenet_to_world(fp.road_id, fp.lane_id, fp.s, fp.offset, wp)) {
                        e.x = wp.x;
                        /* y 取自 Frenet 反算的车道外侧 +1.5m 路缘退让 */
                        if (cached_half_width < 0.0 || fp.road_id != e.road_id) {
                            /* 现算：esmini 给定 road 在 s 处的可行驶车道总宽 */
                            int n_drivable = g.roads.drivable_lane_count(fp.road_id, fp.s);
                            /* 半宽 = (n_drivable * lane_width) / 2，lane_width 取场景默认 */
                            cached_half_width = (n_drivable * g.lane_width) / 2.0;
                            e.road_id = fp.road_id;
                        }
                        double sign = (tl->y_lane >= 0.0) ? 1.0 : -1.0;
                        e.y = sign * (cached_half_width + 1.5);
                        /* heading 重新用 Frenet 反算的切线 + π/2（灯杆垂直于道路） */
                        e.heading = wp.h + (sign > 0.0 ? -M_PI_2 : M_PI_2);
                    } else {
                        /* 反算失败 fallback 到旧逻辑 */
                        e.x = tl->x;
                        double road_half_width = 3.5;
                        double sign = (tl->y_lane >= 0.0) ? 1.0 : -1.0;
                        e.y = sign * (road_half_width + 1.5);
                    }
                } else {
                    /* world_to_frenet 失败：场景坐标不在路网附近，fallback */
                    e.x = tl->x;
                    double road_half_width = 3.5;
                    double sign = (tl->y_lane >= 0.0) ? 1.0 : -1.0;
                    e.y = sign * (road_half_width + 1.5);
                }
            } else {
                /* esmini 不可用：旧逻辑 */
                e.x = tl->x;
                double road_half_width = 3.5;
                double sign = (tl->y_lane >= 0.0) ? 1.0 : -1.0;
                e.y = sign * (road_half_width + 1.5);
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
        e.id = eg->id;
        e.target_vx = eg->approach_speed;   /* ETC 通过目标速度 */
        e.phase_timer = 0.0;                 /* 抬杆进度 [0,1]，初始 closed */
        e.width = eg->open_range_m;          /* open_range_m 存到 width 字段 */
        e.ai_state = flowsim::AIState::Stop; /* 初始 closed 状态 */
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

    /* Phase 4: 停止线 → StopLine 实体（路口/ETC 停车位置标记）。
     * 纯可视化标记，无动力学无状态机，scene_pub 直接序列化位置。
     * esmini 校正坐标：stop_lines[*].x/y 手填值经常和实际路网偏移（与 traffic_lights
     * 同根因），走 world_to_frenet → frenet_to_world 反算真值。 */
    for (int i = 0; i < sc->stop_line_count && i < SCENARIO_MAX_STOP_LINES; i++) {
        const ScenarioStopLine* sl = &sc->stop_lines[i];
        flowsim::EntityId id = g.pool.alloc(flowsim::EntityType::StopLine);
        if (id == flowsim::INVALID_ENTITY) break;
        flowsim::Entity& e = g.pool[id];
        e.id = sl->id;
        e.heading = sl->heading;
        /* esmini 校正坐标 */
        if (g.roads_loaded) {
            flowsim::FrenetPos fp;
            if (g.roads.world_to_frenet(sl->x, sl->y, fp)) {
                flowsim::WorldPos wp;
                if (g.roads.frenet_to_world(fp.road_id, fp.lane_id, fp.s, fp.offset, wp)) {
                    e.x = wp.x;
                    e.y = wp.y;
                    e.heading = wp.h;
                } else {
                    e.x = sl->x; e.y = sl->y;
                }
            } else {
                e.x = sl->x; e.y = sl->y;
            }
        } else {
            e.x = sl->x; e.y = sl->y;
        }
        if (e.heading == 0.0) {
            e.heading = compute_road_heading_at(e.x, e.y);
        }
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
    cJSON_AddNumberToObject(vstate, "road_id", (double)ego.road_id);
    cJSON_AddNumberToObject(vstate, "lane_id", (double)ego.lane_id);

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
    /* 按当前 ego 所在 road 实时查询可行驶车道数。
     * 旧实现硬编码 lane_count=2，导致 4 车道 toll_plaza / 3 车道 urban 段
     * control_node 的 cruise_lane_y 算式只能算出 ±half_lane（中间两车道），
     * ego 永远到不了 +5.25 / -5.25 外侧车道，看起来像"逆行"。
     * 这里查询失败时回退 2（向后兼容 2 车道场景）。 */
    int lane_count = 2;
    if (g.roads_loaded) {
        const flowsim::Entity& ego = g.pool[0];
        flowsim::FrenetPos ef;
        if (g.roads.world_to_frenet(ego.x, ego.y, ef)) {
            int n = g.roads.drivable_lane_count(ef.road_id, ef.s);
            if (n > 0) lane_count = n;
        }
    }
    cJSON_AddNumberToObject(root, "lane_count", lane_count);
    /* 同步到 scene_pub_cfg，供 ego fallback 横向控制（lane_keep_ego_fallback）
     * 和 scene/frame 发布使用，否则它们仍用 init 默认值 2，导致 4 车道场景
     * ego target_y 算错。 */
    g.scene_pub_cfg.lane_count = lane_count;
    char* s = cJSON_PrintUnformatted(root);
    transport_publish(g.transport, "road/geometry",
                      (const uint8_t*)s, (uint32_t)strlen(s) + 1);
    free(s);
    cJSON_Delete(root);
}

/**
 * publish_ref_path — ego route-following 参考路径发布。
 *
 * 背景：control_node Stanley 横向控制原本依赖全局单段 curve_*（curve_start_x/
 * curve_length_m/curve_offset_m）算 cte/heading/kappa，road_network 多 edge 场景
 * 下 curve_* 全零 → 参考线退化为 y=0 直线 → ego 过 fork 后沿直线开进空地。
 * 本函数从 esmini 路网采样 ego 前方 N 个参考点（含曲率前馈），发布 JSON 给
 * control_node 替代 curve_* 直线参考。
 *
 * 每 cycle 发布（20Hz），与 control_node 控制周期对齐。
 * 无 route / esmini 加载失败时发布空数组（control_node 检测到空数组回退 curve_*）。
 */
static void publish_ref_path(void) {
    cJSON* root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "t_us", (double)clock_now_us());

    cJSON* pts = cJSON_CreateArray();
    if (g.roads_loaded && g.route.ok()) {
        /* ego 在 route 上的累计 s：与主循环同样的算法 */
        const flowsim::Entity& ego = g.pool[0];
        flowsim::FrenetPos ef;
        double ego_route_s = 0.0;
        if (g.roads.world_to_frenet(ego.x, ego.y, ef)) {
            int ei = g.route.index_of(ef.road_id);
            if (ei >= 0) ego_route_s = g.route.to_route_s(ei, ef.s);
        }
        /* 采样 ego 前方 100m、间距 5m（21 个点）；lookahead 远一些保证
         * 控制器在高速段（27m/s）每步 1.35m 也有充足前视距离。 */
        std::vector<flowsim::RefPathPoint> samples;
        int n = 0;
        /* Phase 2: ego 有 road_pos 时优先用 RoadPosition 采样。
         * 用 clone 避免污染 ego 主 position（sample_ahead 会推进 handle 状态）。
         * junction_angle=M_PI 默认直行（后续可从 route step type 映射左/右转）。 */
        if (ego.road_pos.ok()) {
            flowsim::RoadPosition tmp = ego.road_pos.clone();
            if (tmp.ok()) {
                n = tmp.sample_ahead(g.roads, 100.0, 5.0, M_PI, samples);
            }
        }
        /* fallback：road_pos 未初始化或采样失败 → 旧 route.sample_ahead */
        if (n == 0) {
            n = g.route.sample_ahead(g.roads, ego_route_s, 100.0, 5.0, samples);
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
    char* s = cJSON_PrintUnformatted(root);
    transport_publish(g.transport, "road/ref_path",
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
    /* 横向：车道保持 — 朝 ego 当前所在车道中心。
     * 旧实现硬编码 -0.5*lane_width（2 车道左车道中心），是 2 车道假设。
     * N 车道模型：用 lane_idx_from_y 量化 ego 当前车道 idx，再算该车道中心 y。
     * 单车道路段（如 ramp_curve）回退到道路中心。 */
    double rc_y = road_center_y(ego.x, g.curve_start_x, g.curve_length_m, g.curve_offset_m);
    int ego_lane_idx = lane_idx_from_y(ego.y, g.scene_pub_cfg.lane_count, g.lane_width, rc_y);
    double target_y = lane_center_y(ego_lane_idx, g.scene_pub_cfg.lane_count, g.lane_width, rc_y);
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

            /* Phase 2: ego 有 road_pos 时用 RoadPosition 推进 + 覆盖世界坐标。
             * step_bicycle 已更新 ego.speed（throttle/brake 驱动）和 ego.x/y/heading
             * （世界系自行车模型积分）。若 road_pos.ok()，用 road_pos.advance 沿真实
             * OpenDRIVE 拓扑推进 ego.speed*dt，再用 road_pos.world() 把 ego.x/y/heading
             * 覆盖为路网对齐坐标——保证 ego 严格贴路面、过 fork/路口不飞出路网。
             * junction_angle 暂用 M_PI（直行），后续可从 route step type 映射。
             * road_pos 不 ok 时保留 step_bicycle 结果（旧逻辑）。 */
            if (ego.road_pos.ok()) {
                double dist = ego.speed * FLOWSIM_DT_SEC;
                if (dist > 0.0) {
                    if (!ego.road_pos.advance(dist, M_PI)) {
                        /* 推进失败（路网边界）— 不立即销毁 handle，下一 tick 仍可重试。
                         * 此处不强制 recycle ego（ego 不回收），只 log 限速。 */
                        if (g.cycle % 100 == 0) {
                            LOG_WARN("flowsim", "ego road_pos.advance failed (dist=%.2f)", dist);
                        }
                    } else {
                        flowsim::WorldPos wp;
                        if (ego.road_pos.world(wp)) {
                            ego.x = wp.x;
                            ego.y = wp.y;
                            ego.heading = wp.h;
                            ego.vx = ego.speed * std::cos(wp.h);
                            ego.vy = ego.speed * std::sin(wp.h);
                        }
                        /* 同步 Frenet 字段（部分下游逻辑 / 调试日志读 ego.road_id 等） */
                        flowsim::FrenetPos fp;
                        if (ego.road_pos.frenet(fp)) {
                            ego.road_id = fp.road_id;
                            ego.lane_id = fp.lane_id;
                            ego.s = fp.s;
                            ego.offset = fp.offset;
                        }
                    }
                }
            }

            /* ── Step 2: 场景事件预检查（让 NPC 知道前方红绿灯/ETC） ── */
            flowsim::check_npc_scene_events(g.pool, g.ai_cfg.look_ahead);

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
            /* ego 在 route 上的累计 s：回收 NPC 时用来放到 ego 附近，形成持续车流 */
            double ego_route_s = 0.0;
            if (route_ptr) {
                flowsim::FrenetPos ef;
                if (g.roads.world_to_frenet(g.pool[0].x, g.pool[0].y, ef)) {
                    int ei = g.route.index_of(ef.road_id);
                    if (ei >= 0) ego_route_s = g.route.to_route_s(ei, ef.s);
                }
            }
            for (int i = 1; i < g.pool.size(); i++) {
                flowsim::Entity& e = g.pool[i];
                if (!e.active) continue;
                if (e.is_npc_vehicle()) {
                    flowsim::step_npc_vehicle(e, g.pool, FLOWSIM_DT_SEC,
                                              g.ai_cfg, roads_ptr, route_ptr, ego_route_s);
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
            /* road/ref_path：ego 前方参考路径，每 cycle 发布给 control_node Stanley
             * 横向控制消费；无 route 时发布空数组（control_node 回退到 curve_*）。 */
            publish_ref_path();
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
    "road/ref_path",
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
                if (g.route.build(g.roads)) {
                    LOG_INFO("flowsim", "central route built: %d segments, %.0fm total",
                             g.route.count(), g.route.total_length());
                } else {
                    LOG_WARN("flowsim", "route build failed — NPC lane-follow off (straight fallback)");
                }
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
    /* Task 4：把场景 JSON 的 lighting 字段透传到 scene/frame topic，
     * 前端 scene3d.js 据此调整 AmbientLight/DirectionalLight/Bloom 阈值。 */
    g.scene_pub_cfg.lighting       = (int)g.scenario->lighting;

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
