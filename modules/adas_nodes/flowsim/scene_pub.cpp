/**
 * scene_pub.cpp — scene/frame topic 发布实现
 *
 * 把 EntityPool 当前状态 + 道路网络快照序列化成 JSON，发布到 scene/frame topic。
 * 设计文档 §6.3 约定的 JSON 帧格式，20Hz 与仿真 tick 一致。
 *
 * 关键设计：
 *   - 道路网络采样：若 esmini RoadManager 已加载，按 road 索引遍历，每条 road 在
 *     参考线（lane_id=0, offset=0）上等距采样 N 点，输出为 nodes [[x,y],...]
 *     给前端用 CatmullRomCurve3 渲染。esmini 未加载时退化为单条直/弯道 edge，
 *     端点用 road_geometry.h 的 road_center_y() 计算。
 *   - 实体序列化：按 EntityType 分发到不同字段集合，事件触发器（TL/ETCGate）
 *     只发可视化必需字段，避免每帧冗余传输。
 *   - cJSON 内存管理：所有 cJSON 节点最终由 cJSON_Delete(root) 递归释放，
 *     打印结果 free(s) 显式释放 cJSON_PrintUnformatted 的返回字符串。
 */

#include "scene_pub.h"
#include "road_geometry.h"   /* road_center_y / road_center_heading */
#include "transport.h"
#include "logger.h"

#include <cjson/cJSON.h>

#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>

namespace flowsim {

namespace {

/* ── 类型/状态 → JSON 字符串 ─────────────────────────────────── */

const char* entity_type_str(EntityType t) {
    switch (t) {
        case EntityType::Ego:         return "ego";
        case EntityType::Car:         return "car";
        case EntityType::SUV:         return "suv";
        case EntityType::Truck:       return "truck";
        case EntityType::Pedestrian:  return "pedestrian";
        case EntityType::TrafficLight:return "tl";
        case EntityType::ETCGate:     return "etc_gate";
        case EntityType::StopLine:    return "stop_line";
        default:                      return "unknown";
    }
}

const char* ai_state_str(AIState s) {
    switch (s) {
        case AIState::Cruise:      return "cruise";
        case AIState::Follow:      return "follow";
        case AIState::Stop:        return "stop";
        case AIState::StopForTL:   return "stop_for_tl";
        case AIState::ETCApproach: return "etc_approach";
        case AIState::BranchSel:   return "branch_sel";
        case AIState::Merge:       return "merge";
        case AIState::Yield:       return "yield";
        case AIState::CutIn:       return "cutin";
        default:                   return "cruise";
    }
}

const char* tl_phase_str(int phase_state) {
    /* phase_state 与 TLPhase 枚举一致（scene_events.h）：0=Green 1=Yellow 2=Red */
    switch (phase_state) {
        case 0:  return "green";
        case 1:  return "yellow";
        case 2:  return "red";
        default: return "green";
    }
}

const char* etc_gate_state_str(AIState s) {
    /* ETC 抬杆状态（scene_events.cpp 约定）：
     *   Stop    = closed   (远距/已通过)
     *   Yield   = opening  (10-50m，正在抬杆)
     *   Cruise  = open     (<10m，全开) */
    switch (s) {
        case AIState::Stop:    return "closed";
        case AIState::Yield:   return "opening";
        case AIState::Cruise:  return "open";
        default:               return "closed";
    }
}

/* ── 道路网络采样 ─────────────────────────────────────────────── */

/* 每条 road 沿参考线采样的节点数。前端用 CatmullRomCurve3 平滑插值，
 * 8 个点对直道/弯道都足够（200m 直道每 25m 一点；圆弧曲率变化清晰可见）。 */
constexpr int ROAD_NODES_PER_EDGE = 8;

/* 无 esmini 时单条 legacy 道路的端点采样数（弯道用更多点保证平滑）。 */
constexpr int LEGACY_ROAD_NODES = 8;

/**
 * 从 esmini RoadManager 采样一条道路的参考线节点。
 * @param roads   esmini 网络封装（非 const：frenet_to_world 会更新内部 position 状态）
 * @param index   道路索引 [0, roads.road_count())
 * @param nodes   输出 [x,y,z] 三元组数组（追加到末尾），z 为道路 elevation
 * @return true 成功采样；false 表示道路不存在或采样失败
 */
bool sample_road_nodes(FlowRoadNetwork& roads, int index,
                       cJSON* nodes_array) {
    RoadInfo info;
    if (!roads.road_info(index, info) || info.length <= 0) return false;

    /* 沿 s 等距采样：lane_id=0（参考线）, offset=0 */
    for (int i = 0; i < ROAD_NODES_PER_EDGE; ++i) {
        double s = info.length * (double)i / (ROAD_NODES_PER_EDGE - 1);
        WorldPos w;
        if (!roads.frenet_to_world(info.id, 0, s, 0.0, w)) {
            /* 中途采样失败：用最后成功点占位，保证 nodes 数组等长 */
            w.x = 0; w.y = 0; w.z = 0;
        }
        cJSON* pt = cJSON_CreateArray();
        cJSON_AddItemToArray(pt, cJSON_CreateNumber(w.x));
        cJSON_AddItemToArray(pt, cJSON_CreateNumber(w.y));
        /* 第三元素 z 是道路 elevation（高架高度）。平地场景恒为 0，
         * 与旧版 [x,y] 二元组前端读取 nodes[ni][2] || 0 完全兼容。 */
        cJSON_AddItemToArray(pt, cJSON_CreateNumber(w.z));
        cJSON_AddItemToArray(nodes_array, pt);
    }
    return true;
}

/**
 * 旧场景几何（curve_start_x/length/offset）→ 单条 edge 的节点采样。
 * 弯道段用 smoothstep 参数化，在 [curve_start_x, curve_start_x+length] 上等距采样，
 * 每点 y = road_center_y(x)。直道（curve_length_m<=0）只输出起点+终点。
 *
 * 道路总长 = max(curve_start_x + curve_length_m, 200m)，保证 ego 行进过程中
 * 总有道路可渲（旧场景无显式 length 字段，按场景 actor 的 x 范围外推 200m 即可）。
 */
void sample_legacy_road_nodes(const ScenePubConfig& cfg, cJSON* nodes_array) {
    double total_len = cfg.curve_start_x + cfg.curve_length_m;
    if (total_len < 200.0) total_len = 200.0;

    bool is_curve = (cfg.curve_length_m > 0.0 && cfg.curve_offset_m != 0.0);
    int n = is_curve ? LEGACY_ROAD_NODES : 2;

    for (int i = 0; i < n; ++i) {
        double x = total_len * (double)i / (n - 1);
        double y = road_center_y(x, cfg.curve_start_x,
                                  cfg.curve_length_m, cfg.curve_offset_m);
        cJSON* pt = cJSON_CreateArray();
        cJSON_AddItemToArray(pt, cJSON_CreateNumber(x));
        cJSON_AddItemToArray(pt, cJSON_CreateNumber(y));
        cJSON_AddItemToArray(nodes_array, pt);
    }
}

cJSON* build_road_network_json(ScenePubConfig& cfg) {
    cJSON* rn = cJSON_CreateObject();
    cJSON* edges = cJSON_CreateArray();

    if (cfg.roads && cfg.roads->loaded() && cfg.roads->road_count() > 0) {
        /* esmini 模式：每条 road 一个 edge。
         * 注意 cfg.roads 是非 const 指针 — frenet_to_world 会更新内部 position
         * 状态（esmini C API 限制），所以 ScenePubConfig 在 publish_scene_frame
         * 里需要去 const。 */
        for (int i = 0; i < cfg.roads->road_count(); ++i) {
            RoadInfo info;
            if (!cfg.roads->road_info(i, info)) continue;

            cJSON* edge = cJSON_CreateObject();
            cJSON_AddNumberToObject(edge, "id", (double)info.id);
            cJSON_AddStringToObject(edge, "name", info.str_id.c_str());

            cJSON* nodes = cJSON_CreateArray();
            if (!sample_road_nodes(*cfg.roads, i, nodes)) {
                cJSON_Delete(nodes);
                cJSON_Delete(edge);
                continue;
            }
            cJSON_AddItemToObject(edge, "nodes", nodes);

            cJSON_AddNumberToObject(edge, "lanes", (double)(info.drivable_lanes / 2));
            /* 从 esmini 查询该 road 第一条行驶车道的实际宽度；
             * 查询失败或为 0 时退回 cfg.lane_width（默认 3.5m）。 */
            {
                double road_lw = cfg.roads->lane_width(info.id, -1, 0.0);
                cJSON_AddNumberToObject(edge, "lane_width",
                                        (road_lw > 0.0) ? road_lw : cfg.lane_width);
            }
            /* oneway: 匝道(ramp)为单向道路，其余默认双向对称 */
            bool is_oneway = (info.str_id.find("ramp") != std::string::npos);
            cJSON_AddBoolToObject(edge, "oneway", is_oneway);
            cJSON_AddNumberToObject(edge, "length", info.length);
            cJSON_AddItemToArray(edges, edge);
        }
    } else {
        /* 旧场景模式：单条 edge 表示整条道路 */
        cJSON* edge = cJSON_CreateObject();
        cJSON_AddNumberToObject(edge, "id", 0);
        cJSON_AddStringToObject(edge, "name", "legacy_road");

        cJSON* nodes = cJSON_CreateArray();
        sample_legacy_road_nodes(cfg, nodes);
        cJSON_AddItemToObject(edge, "nodes", nodes);

        cJSON_AddNumberToObject(edge, "lanes", (double)cfg.lane_count);
        cJSON_AddNumberToObject(edge, "lane_width", cfg.lane_width);
        cJSON_AddBoolToObject(edge, "oneway", false);  // 旧版直道默认双向
        double total_len = cfg.curve_start_x + cfg.curve_length_m;
        if (total_len < 200.0) total_len = 200.0;
        cJSON_AddNumberToObject(edge, "length", total_len);
        cJSON_AddItemToArray(edges, edge);
    }

    cJSON_AddItemToObject(rn, "edges", edges);
    return rn;
}

/* ── 实体序列化 ─────────────────────────────────────────────── */

cJSON* build_ego_json(const Entity& e) {
    cJSON* j = cJSON_CreateObject();
    cJSON_AddStringToObject(j, "type", "ego");
    cJSON_AddNumberToObject(j, "id", (double)e.id);
    cJSON_AddNumberToObject(j, "x", e.x);
    cJSON_AddNumberToObject(j, "y", e.y);
    cJSON_AddNumberToObject(j, "h", e.heading);
    cJSON_AddNumberToObject(j, "spd", e.speed);
    cJSON_AddNumberToObject(j, "steer", e.steer);
    cJSON_AddNumberToObject(j, "throttle", e.throttle);
    cJSON_AddNumberToObject(j, "brake", e.brake);
    cJSON_AddNumberToObject(j, "len", e.length);
    cJSON_AddNumberToObject(j, "wid", e.width);
    cJSON_AddNumberToObject(j, "tgt", e.target_vx);
    /* vx/vy 用于前端速度向量可视化（弯道时 vy≠0） */
    cJSON_AddNumberToObject(j, "vx", e.vx);
    cJSON_AddNumberToObject(j, "vy", e.vy);
    /* 车灯位掩码（vehicle_lights.h）：bit0=左转 bit1=右转 bit2=双闪
     * bit3=远光 bit4=近光 bit6=倒车 bit7=雾灯。刹车灯由 brake 字段驱动。 */
    cJSON_AddNumberToObject(j, "lights", (double)e.lights.mask);
    return j;
}

cJSON* build_npc_vehicle_json(const Entity& e) {
    cJSON* j = cJSON_CreateObject();
    cJSON_AddStringToObject(j, "type", entity_type_str(e.type));
    cJSON_AddNumberToObject(j, "id", (double)e.id);
    cJSON_AddNumberToObject(j, "x", e.x);
    cJSON_AddNumberToObject(j, "y", e.y);
    cJSON_AddNumberToObject(j, "h", e.heading);
    cJSON_AddNumberToObject(j, "spd", e.speed);
    cJSON_AddNumberToObject(j, "len", e.length);
    cJSON_AddNumberToObject(j, "wid", e.width);
    cJSON_AddStringToObject(j, "ai", ai_state_str(e.ai_state));
    cJSON_AddNumberToObject(j, "vx", e.vx);
    cJSON_AddNumberToObject(j, "vy", e.vy);
    /* 车灯位掩码（同 ego）：CutIn→转向灯，Yield/Stop→双闪，Cruise→随 steer */
    cJSON_AddNumberToObject(j, "lights", (double)e.lights.mask);
    return j;
}

cJSON* build_pedestrian_json(const Entity& e) {
    cJSON* j = cJSON_CreateObject();
    cJSON_AddStringToObject(j, "type", "pedestrian");
    cJSON_AddNumberToObject(j, "id", (double)e.id);
    cJSON_AddNumberToObject(j, "x", e.x);
    cJSON_AddNumberToObject(j, "y", e.y);
    cJSON_AddNumberToObject(j, "spd", e.speed);
    cJSON_AddNumberToObject(j, "vx", e.vx);
    cJSON_AddNumberToObject(j, "vy", e.vy);
    /* ped_parked 状态供前端区分"站立"vs"行走"动画 */
    cJSON_AddBoolToObject(j, "parked", e.ped_parked != 0);
    return j;
}

cJSON* build_traffic_light_json(const Entity& e) {
    cJSON* j = cJSON_CreateObject();
    cJSON_AddStringToObject(j, "type", "tl");
    cJSON_AddNumberToObject(j, "id", (double)e.id);
    cJSON_AddNumberToObject(j, "x", e.x);
    cJSON_AddNumberToObject(j, "y", e.y);
    cJSON_AddNumberToObject(j, "h", e.heading);
    cJSON_AddStringToObject(j, "state", tl_phase_str(e.phase_state));
    cJSON_AddNumberToObject(j, "remain_s", e.phase_timer);
    return j;
}

cJSON* build_etc_gate_json(const Entity& e) {
    cJSON* j = cJSON_CreateObject();
    cJSON_AddStringToObject(j, "type", "etc_gate");
    cJSON_AddNumberToObject(j, "id", (double)e.id);
    cJSON_AddNumberToObject(j, "x", e.x);
    cJSON_AddNumberToObject(j, "y", e.y);
    cJSON_AddNumberToObject(j, "h", e.heading);
    cJSON_AddStringToObject(j, "state", etc_gate_state_str(e.ai_state));
    /* phase_timer ∈ [0,1] 表示抬杆进度（scene_events.cpp 约定） */
    cJSON_AddNumberToObject(j, "progress", e.phase_timer);
    return j;
}

cJSON* build_stop_line_json(const Entity& e) {
    cJSON* j = cJSON_CreateObject();
    cJSON_AddStringToObject(j, "type", "stop_line");
    cJSON_AddNumberToObject(j, "id", (double)e.id);
    cJSON_AddNumberToObject(j, "x", e.x);
    cJSON_AddNumberToObject(j, "y", e.y);
    cJSON_AddNumberToObject(j, "h", e.heading);
    return j;
}

cJSON* build_entities_json(const EntityPool& pool) {
    cJSON* arr = cJSON_CreateArray();
    for (int i = 0; i < pool.size(); ++i) {
        const Entity& e = pool[i];
        if (!e.active) continue;

        cJSON* j = nullptr;
        switch (e.type) {
            case EntityType::Ego:          j = build_ego_json(e); break;
            case EntityType::Car:
            case EntityType::SUV:
            case EntityType::Truck:        j = build_npc_vehicle_json(e); break;
            case EntityType::Pedestrian:   j = build_pedestrian_json(e); break;
            case EntityType::TrafficLight: j = build_traffic_light_json(e); break;
            case EntityType::ETCGate:      j = build_etc_gate_json(e); break;
            case EntityType::StopLine:     j = build_stop_line_json(e); break;
            default: continue;
        }
        if (j) cJSON_AddItemToArray(arr, j);
    }
    return arr;
}

}  // anonymous namespace

char* build_scene_frame_json(const EntityPool& pool,
                             ScenePubConfig& cfg,
                             uint64_t sim_time_us,
                             uint32_t cycle) {
    cJSON* root = cJSON_CreateObject();
    if (!root) return nullptr;

    cJSON_AddNumberToObject(root, "t_us", (double)sim_time_us);
    cJSON_AddNumberToObject(root, "cycle", (double)cycle);

    /* lighting（Task 4）：day/night/dusk。前端 scene3d.js 据此切换光照参数。
     * 整数编码与 ScenarioLighting 枚举一致：0=day, 1=night, 2=dusk。
     * 前端首次收到后缓存，避免每帧切换灯光导致抖动。 */
    const char* light_str = "day";
    switch (cfg.lighting) {
        case 1:  light_str = "night"; break;
        case 2:  light_str = "dusk";  break;
        default: light_str = "day";   break;
    }
    cJSON_AddStringToObject(root, "lighting", light_str);

    /* road_network JSON 缓存：道路网络在仿真过程中不变，首次构建后缓存为
     * 字符串，后续帧用 cJSON_AddRawToObject 复用（strdup 开销远小于全网采样）。
     * 性能：build_road_network_json 调用 sample_road_nodes → frenet_to_world
     * 做 esmini position 查询，20Hz × road_count × sample_count 是显著开销。 */
    if (cfg.cached_road_network_json.empty()) {
        cJSON* rn = build_road_network_json(cfg);
        if (rn) {
            char* rn_str = cJSON_PrintUnformatted(rn);
            cJSON_Delete(rn);
            if (rn_str) {
                cfg.cached_road_network_json = rn_str;
                free(rn_str);
            }
        }
    }
    if (!cfg.cached_road_network_json.empty()) {
        /* cJSON_AddRawToObject 会 strdup 字符串并作为 raw 子项添加，
         * cJSON_Delete(root) 时随 root 一起释放，无内存泄漏。 */
        cJSON_AddRawToObject(root, "road_network", cfg.cached_road_network_json.c_str());
    } else {
        /* 缓存失败（极少见，如首次 build 返回 nullptr）→ 降级为直接构建 */
        cJSON* rn = build_road_network_json(cfg);
        if (rn) cJSON_AddItemToObject(root, "road_network", rn);
    }

    cJSON* entities = build_entities_json(pool);
    cJSON_AddItemToObject(root, "entities", entities);

    char* s = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    return s;
}

void publish_scene_frame(Transport* transport,
                         const EntityPool& pool,
                         ScenePubConfig& cfg,
                         uint64_t sim_time_us,
                         uint32_t cycle) {
    if (!transport) return;

    char* s = build_scene_frame_json(pool, cfg, sim_time_us, cycle);
    if (!s) return;

    transport_publish(transport, "scene/frame",
                      (const uint8_t*)s, (uint32_t)strlen(s) + 1);
    free(s);
}

}  // namespace flowsim
