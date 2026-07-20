/**
 * scene_pub.h — scene/frame topic 发布
 *
 * FlowSim v2 设计文档 §6.3：新增 scene/frame topic（20Hz），给 3D 前端用的
 * 完整场景帧。本模块负责把 EntityPool 当前状态 + 道路网络快照序列化成 JSON。
 *
 * 帧格式（设计文档约定）：
 * {
 *   "t_us": 12345678,             // 仿真逻辑时间（μs）
 *   "cycle": 246,
 *   "road_network": {              // 道路网络快照（每帧发布，前端无需缓存状态）
 *     "edges": [
 *       { "id": 0, "nodes": [[0,0],[200,0]], "lanes": 2, "lane_width": 3.5 }
 *     ]
 *   },
 *   "entities": [
 *     { "id": 0, "type": "ego", "x": 102.5, "y": -1.75, "h": 0.05,
 *       "spd": 8.0, "steer": 0.02, "len": 4.6, "wid": 2.0,
 *       "throttle": 0.3, "brake": 0.0 },
 *     { "id": 1, "type": "car", "x": 120, "y": -1.75, "h": 0,
 *       "spd": 3.0, "len": 4.6, "wid": 2.0, "ai": "follow" },
 *     { "id": 6, "type": "tl", "x": 200, "y": -1.75,
 *       "state": "red", "remain_s": 12.3 },
 *     { "id": 12, "type": "etc_gate", "x": 450, "y": 0,
 *       "state": "closed", "progress": 0.0 }
 *   ]
 * }
 *
 * 设计原则：
 *   - 自包含：每帧完整快照，前端订阅即渲染，无需握手/状态同步
 *   - 与 vehicle/state 解耦：vehicle/state 给感知/规划链路用（扁平 ox/oy/ov 数组），
 *     scene/frame 给可视化用（结构化 entities 数组，含事件触发器）
 *   - 道路网络快照：每帧都带 road_network（小数据量 <2KB），避免前端"首帧丢包
 *     就无路可渲"的问题。可改为低频发布，但简单起见先每帧带。
 */

#ifndef FLOWSIM_SCENE_PUB_H
#define FLOWSIM_SCENE_PUB_H

#include "entity.h"
#include "road_network.h"

#include <cstdint>

/* 前向声明：transport.h 里的 Transport 结构 */
struct Transport;

namespace flowsim {

/**
 * scene/frame 发布配置。
 * 由 flowsim_node 在 init 阶段填充，每帧传入 publish_scene_frame()。
 */
struct ScenePubConfig {
    /* 旧场景道路几何（curve_offset_m==0 → 直道；esmini 加载失败时用） */
    double curve_start_x{0};
    double curve_length_m{0};
    double curve_offset_m{0};
    double lane_width{3.5};
    int    lane_count{2};

    /* esmini 道路网络（加载成功时非空，优先用其采样节点） */
    FlowRoadNetwork* roads{nullptr};

    /* scene/frame topic type ID（与 flowsim_node 一致） */
    uint32_t type_id{0};

    /* ── Task 4：全局光照模式 ──
     * 0=day（默认），1=night（夜间：环境光/平行光下调，emissive 提升），
     * 2=dusk（黄昏）。每帧发布给前端，scene3d.js 据此调整 AmbientLight 等。
     * 来源：scenario_loader 解析 JSON 顶层 "lighting" 字段。 */
    int lighting{0};
};

/**
 * 构造 scene/frame JSON 字符串（调用方负责 free）。
 *
 * 与 publish_scene_frame 共享同一套序列化逻辑，但返回字符串而非直接发布，
 * 便于单元测试和复用（如落盘、调试输出、转发到其它传输层）。
 *
 * @return 动态分配的 JSON 字符串，调用方 free()；失败返回 nullptr。
 */
char* build_scene_frame_json(const EntityPool& pool,
                             ScenePubConfig& cfg,
                             uint64_t sim_time_us,
                             uint32_t cycle);

/**
 * 构造 scene/frame JSON 并通过 transport 发布。
 *
 * @param transport   传输层句柄
 * @param pool        实体池（ego 在 index 0）
 * @param cfg         发布配置（道路几何 + esmini 网络指针）。非 const：esmini
 *                    的 frenet_to_world 会更新内部 position 状态，故 cfg 内
 *                    的 roads 指针需可变。
 * @param sim_time_us 仿真逻辑时间（μs）
 * @param cycle       仿真周期计数
 *
 * 内部行为：
 *   1. 调用 build_scene_frame_json 构造 JSON
 *   2. transport_publish("scene/frame", ...)
 *   3. free 字符串
 *
 * 失败时静默返回（不抛异常，仿真主循环不应被可视化抖动中断）。
 */
void publish_scene_frame(Transport* transport,
                         const EntityPool& pool,
                         ScenePubConfig& cfg,
                         uint64_t sim_time_us,
                         uint32_t cycle);

}  // namespace flowsim

#endif  // FLOWSIM_SCENE_PUB_H
