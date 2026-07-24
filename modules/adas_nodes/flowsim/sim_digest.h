/**
 * sim_digest.h — 仿真基础层：静态场景 + 动态演员 digest + 数值 invariant
 *
 * 帧契约头：
 *   frame: THREE  | up: +Y | 单位: m | ENU→THREE: [x, z, -y] | ego_centered: true/false
 *
 * 用法：
 *   - 几何变更时调用 dump_static_digest() 输出 JSON
 *   - 每帧调用 dump_dynamic_digest() + check_invariants() 输出 JSON + 断言
 *   - ASCII 俯视：render_ascii_overhead() 输出文本
 *
 * 设计要点：
 *   - invariant 断言编码"眼睛会检查的位置关系"为数值，错一个符号 = FAIL
 *   - 单帧 invariant 抓浮空/埋地/尺度错/朝向错/车道归属错乱
 *   - 时序 invariant 抓瞬移/朝向瞬变/运动学不可行/压实线变道
 *   - 运动方向 invariant 抓"车头与运动不一致"（车横着开）
 */

#ifndef FLOWSIM_SIM_DIGEST_H
#define FLOWSIM_SIM_DIGEST_H

#include "entity.h"
#include "road_network.h"
#include "route.h"

#include <string>
#include <vector>
#include <cstdio>

namespace flowsim {

// ═══════════════════════════════════════════════════════════
// 静态场景 digest
// ═══════════════════════════════════════════════════════════

struct LaneDigest {
    int    id;
    int    road_id;
    int    lane_id;
    std::vector<double> centerline_x;  // 中线采样点
    std::vector<double> centerline_y;
    double width;
    int    left_boundary_type;   // 0=虚线 1=实线 2=双黄
    int    right_boundary_type;
    int    direction;            // +1=正向 -1=反向
    double speed_limit;
    double s_start, s_end;
};

struct RoadMarkingDigest {
    int    road_id;
    double s_start, s_end;
    int    type;                 // 0=虚线 1=实线 2=双黄
    double dash_length;         // 虚线段长 (m)
    double gap_length;          // 虚线间距 (m)
};

struct TrafficLightDigest {
    int    id;
    double x, y, z;
    double heading;
    int    controlled_road_id;
    int    controlled_lane_id;
    int    phase;               // 0=绿 1=黄 2=红
};

struct StaticDigest {
    std::vector<LaneDigest>        lanes;
    std::vector<RoadMarkingDigest> markings;
    std::vector<TrafficLightDigest> traffic_lights;
    // 可行驶区多边形
    std::vector<double> drivable_poly_x;
    std::vector<double> drivable_poly_y;
    // 路面高程采样
    std::vector<double> height_samples_x;
    std::vector<double> height_samples_y;
    std::vector<double> height_samples_z;
    // 全局
    double road_half_width{0};
    int    total_lanes{0};
};

// ═══════════════════════════════════════════════════════════
// 动态演员 digest（每帧）
// ═══════════════════════════════════════════════════════════

struct ActorDigest {
    int    id;
    int    type;                 // 0=ego 1=car 2=suv 3=truck 4=pedestrian
    double pos[3];              // 世界坐标 [x,y,z]
    double bbox[3];             // 尺寸 [L,W,H]
    double heading;             // 朝向 (rad)
    double vel[2];              // 速度矢量 [vx,vy]
    double speed;
    double yaw_rate;
    double accel;
    int    road_id;
    int    lane_id;
    double lateral_offset;
    double s;                   // 沿路里程
    double rotation_y;          // 实际给 THREE 的 rotation.y
    int    route_dir;           // 行驶方向
    int    ai_state;            // AI 状态
    uint32_t last_teleport_cycle;  // 最近一次被显式传送的 cycle（choreography/recycle）
};

struct DynamicDigest {
    double sim_time;
    int    frame;
    bool   ego_centered;
    double origin[2];
    std::vector<ActorDigest> actors;
};

// ═══════════════════════════════════════════════════════════
// 静态 invariant 检查结果
// ═══════════════════════════════════════════════════════════

struct InvariantResult {
    int  passed{0};
    int  failed{0};
    int  warned{0};
    std::string details;  // 失败详情（换行分隔）
};

/**
 * 检查静态场景 invariant（用户 spec 表全字段）：
 *   1. 车道宽 ∈ [2.5, 4.0]m
 *   2. 边界类型自洽：同向分隔=虚线、对向=双黄/实线、外沿=实线
 *   3. 虚线段长 ~3m、间距 ~6–9m；实线连续无断
 *   4. 可行驶区闭合、不自交
 *   5. 路面高程连续、无阶跃跳变
 *   6. 红绿灯朝向 · 车道行驶方向 < 0（面向来车，不背对）
 *   7. 没有物体堆在 (0,0,0)（除非本该在）
 *   8. 每条 lane 的中线都落在可行驶多边形内
 */
InvariantResult check_static_invariants(const StaticDigest& sd);

// ═══════════════════════════════════════════════════════════
// Digest 生成
// ═══════════════════════════════════════════════════════════

/** 从路网生成静态场景 digest（几何变更时调用一次） */
StaticDigest build_static_digest(FlowRoadNetwork& roads, const Route& route,
                                  const EntityPool& pool);

/** 从当前帧生成动态演员 digest */
DynamicDigest build_dynamic_digest(const EntityPool& pool, double sim_time,
                                    int frame, bool ego_centered);

/** 将 digest 序列化为 JSON 字符串 */
std::string digest_to_json(const StaticDigest& d);
std::string digest_to_json(const DynamicDigest& d);

// ═══════════════════════════════════════════════════════════
// 单帧 invariant 检查
// ═══════════════════════════════════════════════════════════

/**
 * 检查单帧空间 invariant：
 *   1. |z − roadHeight(x,y)| < ε          — 浮空/埋地
 *   2. |lateral_offset| ≤ 半路宽(+裕量)    — 飞出路面
 *   3. rotationY == headingToRotationY(heading) — ENU→THREE 符号翻错
 *   4. 0 ≤ speed ≤ 1.5×限速               — 超速/呆滞
 *   5. bbox ≈ 标准尺寸                    — 尺度错
 *   6. 两 actor bbox 不重叠               — 穿模/重叠
 */
InvariantResult check_spatial_invariants(const DynamicDigest& d,
                                          const StaticDigest& sd,
                                          FlowRoadNetwork* roads);

/**
 * 检查运动方向 invariant（单帧）：
 *   1. dot(forward(heading), vel/|vel|) > cos(30°)  — 车头≈前进方向
 *   2. dot(forward(heading), lane_dir) > cos(45°)   — 与车道方向一致
 *   3. sign(lane 允许方向) == sign(沿 s 前进)        — 不逆行
 */
InvariantResult check_motion_direction(const DynamicDigest& d,
                                        const StaticDigest& sd,
                                        FlowRoadNetwork* roads);

// ═══════════════════════════════════════════════════════════
// 时序 invariant 检查（需要连续 N 帧）
// ═══════════════════════════════════════════════════════════

/**
 * 检查时序 invariant（需要上一帧的 digest）：
 *   1. Δpos ≈ vel × dt          — 不瞬移
 *   2. |Δpos| ≤ v_max × dt      — 不超速瞬移
 *   3. |Δheading| ≤ yaw_max × dt — 朝向不瞬变
 *   4. accel ∈ [−8, +4] m/s²    — 运动学可行
 *
 * @param prev  上一帧 digest
 * @param curr  当前帧 digest
 * @param dt    帧间隔 (s)
 */
InvariantResult check_temporal_invariants(const DynamicDigest& prev,
                                           const DynamicDigest& curr,
                                           double dt);

// ═══════════════════════════════════════════════════════════
// ASCII 俯视渲染
// ═══════════════════════════════════════════════════════════

/**
 * 渲染 ASCII 俯视图（盲 agent 可读）。
 * ═ 实线  : 虚线  ‖ 双黄  C 车(→↑←↓↗ 表朝向)  ● 行人  🚦 灯
 *
 * @param sd   静态 digest（车道线/红绿灯位置）
 * @param dd   动态 digest（车辆/行人位置+朝向）
 * @param width_chars  输出宽度（字符数），默认 80
 * @param height_chars 输出高度（字符数），默认 40
 * @return ASCII 俯视图字符串
 */
std::string render_ascii_overhead(const StaticDigest& sd, const DynamicDigest& dd,
                                   int width_chars = 80, int height_chars = 40);

// ═══════════════════════════════════════════════════════════
// Golden 快照（transform 记账 + diff）
// ═══════════════════════════════════════════════════════════

/**
 * 生成 golden 快照 JSON：排序后的 (name, pos, rotY, scale) 列表。
 * 可 commit 到仓库作为 golden 参考，任何位置漂移 = diff FAIL。
 * 只有几何合法变更时才需更新 golden（PR 附截图）。
 *
 * @param dd  当前帧动态 digest
 * @return     排序后的 JSON 字符串
 */
std::string golden_snapshot(const DynamicDigest& dd);

/**
 * 比较两帧 golden 快照，检测位置漂移。
 * 任何 actor 的 pos/rotY/scale 偏差超过 tolerance 即 FAIL。
 *
 * @param golden  基准 golden JSON
 * @param current 当前帧 golden JSON
 * @param tolerance 位置容差 (m)，默认 0.01
 * @return 差异报告（空字符串 = 通过）
 */
std::string golden_diff(const std::string& golden, const std::string& current,
                         double tolerance = 0.01);

}  // namespace flowsim

#endif  // FLOWSIM_SIM_DIGEST_H