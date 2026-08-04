/**
 * sim_digest.cpp — 仿真基础层实现：digest 生成 + invariant 检查 + ASCII 俯视
 */

#include "sim_digest.h"
#include "logger.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <algorithm>
#include <sstream>
#include <istream>

namespace flowsim {

// ═══════════════════════════════════════════════════════════
// 静态场景 digest
// ═══════════════════════════════════════════════════════════

StaticDigest build_static_digest(FlowRoadNetwork& roads, const Route& route,
                                  const EntityPool& pool) {
    (void)route;
    StaticDigest sd;
    if (!roads.loaded()) return sd;

    int rc = roads.road_count();
    double max_half_width = 0;

    for (int i = 0; i < rc; ++i) {
        RoadInfo ri;
        if (!roads.road_info(i, ri)) continue;

        // 采样道路中线
        int n_lanes = roads.drivable_lane_count((int)ri.id, 0);

        /* B-2 修复：原代码硬编码 lane_id=0，把参考线采样 n_lanes 次，每条
         * "车道"的中线都是参考线，车道宽恒为 3.5。这导致：
         *   - check_static_invariants 的"车道宽 ∈ [2.5,4.0]"检查恒为 pass（硬编码 3.5）
         *   - "lane 中线落在可行驶多边形内"检查的是参考线而非真实车道中心
         *   - "红绿灯朝向 · 车道方向"用参考线方向，对单向多车道场景无意义
         *
         * 修复：用 lane_width 探测真实 drivable lane_ids（OpenDRIVE 约定
         * 0=参考线非 drivable，正=左，负=右），为每条车道采样各自中心线 +
         * 查询真实车道宽度。无 lane_id 枚举 API，用 lane_width>0 作存在性探测。 */
        std::vector<int> lane_ids;
        for (int lid = -n_lanes; lid <= n_lanes; ++lid) {
            if (lid == 0) continue;  // 参考线不是 drivable lane
            if (roads.lane_width((int)ri.id, lid, 0.0) > 0.0) {
                lane_ids.push_back(lid);
            }
        }

        // 用真实车道宽度算 half_w（替代硬编码 3.5）
        double road_lane_w = 3.5;  // fallback
        if (!lane_ids.empty()) {
            road_lane_w = roads.lane_width((int)ri.id, lane_ids[0], 0.0);
            if (road_lane_w <= 0.0) road_lane_w = 3.5;
        }
        double half_w = n_lanes * road_lane_w * 0.5;
        if (half_w > max_half_width) max_half_width = half_w;

        // 为每个车道生成 digest（用探测到的真实 lane_id 采样各自中心线）
        for (size_t li = 0; li < lane_ids.size(); ++li) {
            int lane_id = lane_ids[li];
            double lw = roads.lane_width((int)ri.id, lane_id, 0.0);
            if (lw <= 0.0) lw = road_lane_w;  // 探测已成功，兜底防 s 变化

            LaneDigest ld;
            ld.id = (int)sd.lanes.size();
            ld.road_id = (int)ri.id;
            ld.lane_id = lane_id;       // 真实车道 id（替代硬编码 0）
            ld.width = lw;              // 真实车道宽度（替代硬编码 3.5）

            // 采样该车道中心线（沿 s 每 10m 采样，offset=0 即车道中心）
            for (double s = 0; s < ri.length; s += 10.0) {
                WorldPos wp;
                if (roads.frenet_to_world((int)ri.id, lane_id, s, 0, wp)) {
                    ld.centerline_x.push_back(wp.x);
                    ld.centerline_y.push_back(wp.y);
                }
            }
            // 末点
            {
                WorldPos wp;
                if (roads.frenet_to_world((int)ri.id, lane_id, ri.length, 0, wp)) {
                    ld.centerline_x.push_back(wp.x);
                    ld.centerline_y.push_back(wp.y);
                }
            }

            ld.direction = 1;

            // 边界类型推断：同向分隔=虚线，外沿=实线，对向=双黄
            ld.left_boundary_type = (li > 0) ? 0 : 1;   // 内边界虚线，外边界实线
            sd.lanes.push_back(ld);
        }

        // Road marking digest
        for (int li = 0; li < (int)lane_ids.size() - 1; ++li) {
            RoadMarkingDigest rm;
            rm.type = 0;  // 虚线（同向分隔）
            rm.dash_length = 3.0;
            rm.gap_length = 6.0;
            sd.markings.push_back(rm);
        }
    }

    // 可行驶区多边形（简化：从车道中线外扩）
    if (sd.lanes.size() > 0) {
        const auto& l0 = sd.lanes[0];
        double half = max_half_width;
        if (l0.centerline_x.size() >= 2) {
            for (size_t i = 0; i < l0.centerline_x.size(); ++i) {
                sd.drivable_poly_x.push_back(l0.centerline_x[i]);
                sd.drivable_poly_y.push_back(l0.centerline_y[i] + half);
            }
            for (int i = (int)l0.centerline_x.size() - 1; i >= 0; --i) {
                sd.drivable_poly_x.push_back(l0.centerline_x[i]);
                sd.drivable_poly_y.push_back(l0.centerline_y[i] - half);
            }
        }
    }

    // 红绿灯 digest
    for (int i = 0; i < pool.size(); ++i) {
        const Entity& e = pool[i];
        if (!e.active || e.type != EntityType::TrafficLight) continue;
        TrafficLightDigest tl;
        tl.id = e.id;
        tl.x = e.x;                  /* 灯杆世界坐标（ENU）— ASCII 渲染定位 */
        tl.y = e.y;
        tl.heading = e.heading;
        tl.controlled_road_id = e.road_id;
        tl.controlled_lane_id = e.lane_id;
        sd.traffic_lights.push_back(tl);
    }

    sd.road_half_width = max_half_width;
    return sd;
}

// ═══════════════════════════════════════════════════════════
// 动态演员 digest
// ═══════════════════════════════════════════════════════════

static int entity_type_to_digest(EntityType t) {
    switch (t) {
        case EntityType::Ego: return 0;
        case EntityType::Car: return 1;
        case EntityType::SUV: return 2;
        case EntityType::Truck: return 3;
        case EntityType::Pedestrian: return 4;
        default: return -1;
    }
}

DynamicDigest build_dynamic_digest(const EntityPool& pool, double sim_time,
                                    int frame, bool ego_centered,
                                    bool ego_maneuver) {
    DynamicDigest dd;
    dd.sim_time = sim_time;
    dd.frame = frame;
    dd.ego_centered = ego_centered;
    dd.maneuver = ego_maneuver;

    const Entity& ego = pool[0];
    double ox = ego_centered ? ego.x : 0.0;
    double oy = ego_centered ? ego.y : 0.0;
    dd.origin[0] = ox;
    dd.origin[1] = oy;

    for (int i = 0; i < pool.size(); ++i) {
        const Entity& e = pool[i];
        if (!e.active) continue;
        int dt = entity_type_to_digest(e.type);
        if (dt < 0) continue;

        ActorDigest ad;
        ad.id = e.id;
        ad.type = dt;
        ad.pos[0] = e.x - ox;
        ad.pos[1] = e.y - oy;
        ad.pos[2] = 0;  // z 从 road_height 查
        ad.bbox[0] = e.length;
        ad.bbox[1] = e.width;
        ad.bbox[2] = (e.type == EntityType::Pedestrian) ? 1.7 : 1.5;
        ad.heading = e.heading;
        ad.vel[0] = e.vx;
        ad.vel[1] = e.vy;
        /* speed 取模长：e.speed 在掉头倒车段是负的（显式负油门倒车），
         * 但 digest 的 speed 语义是标量速率（√(vx²+vy²)，见 entity.h）。
         * 时序 Δpos 检查用 speed×dt 估算期望位移，负 speed 会把 expected
         * 算成负值 → 合法倒车位移被误报"瞬移/teleport"（2026-08-03 掉头
         * 实测 Δpos=0.96 >> expected=-0.09）。方向性由 vel[] 和 anti-reverse
         * 检查承担，speed 只当模长用。 */
        ad.speed = std::fabs(e.speed);
        ad.road_id = e.road_id;
        ad.lane_id = e.lane_id;
        ad.lateral_offset = e.offset;
        ad.s = e.s;
        ad.rotation_y = e.heading;  // headingToRotationY(h) = h
        ad.route_dir = e.route_dir;
        ad.last_teleport_cycle = e.last_teleport_cycle;
        ad.yaw_rate = e.yaw_rate;
        dd.actors.push_back(ad);
    }

    /* 红绿灯相位（每帧）：与 StaticDigest::traffic_lights 按 id 关联。
     * render_ascii_overhead 用此画 G/Y/R 字符。entity_type_to_digest 把
     * TrafficLight 映射为 -1（不进 actors 列表），故相位走单独通道。 */
    for (int i = 0; i < pool.size(); ++i) {
        const Entity& e = pool[i];
        if (!e.active || e.type != EntityType::TrafficLight) continue;
        TrafficLightStateDigest s;
        s.id = e.id;
        s.phase_state = e.phase_state;
        s.phase_timer = e.phase_timer;
        dd.traffic_light_states.push_back(s);
    }
    return dd;
}

// ═══════════════════════════════════════════════════════════
// 静态 invariant 检查（用户 spec 表全字段）
// ═══════════════════════════════════════════════════════════

InvariantResult check_static_invariants(const StaticDigest& sd) {
    InvariantResult r;

    // 1. 车道宽 ∈ [2.5, 4.0]m
    for (size_t i = 0; i < sd.lanes.size(); ++i) {
        const auto& l = sd.lanes[i];
        if (l.width < 2.5 || l.width > 4.0) {
            r.failed++;
            char buf[256];
            snprintf(buf, sizeof(buf),
                "  FAIL lane[%d]: width=%.2f ∉ [2.5, 4.0]m\n", l.id, l.width);
            r.details += buf;
        } else {
            r.passed++;
        }
    }

    // 2. 边界类型自洽
    //    - 同向分隔（相邻同向车道之间）= 虚线 (type=0)
    //    - 外沿（最边缘）= 实线 (type=1) 或双黄 (type=2)
    //    - 对向（方向相反）= 双黄 (type=2) 或实线 (type=1)
    for (size_t i = 0; i < sd.lanes.size(); ++i) {
        const auto& l = sd.lanes[i];
        // 左边界：如果是同向车道分隔 → 虚线；否则 → 实线或双黄
        bool left_is_same_dir = false;
        for (size_t j = 0; j < sd.lanes.size(); ++j) {
            if (i == j) continue;
            const auto& lj = sd.lanes[j];
            if (lj.road_id == l.road_id && lj.direction == l.direction) {
                // 检查是否是相邻车道（通过 lane_id 差值判断）
                if (std::abs(lj.lane_id - l.lane_id) == 1) {
                    left_is_same_dir = true;
                    break;
                }
            }
        }
        if (l.left_boundary_type == 0 && !left_is_same_dir) {
            // 虚线但并非同向分隔 → 可能是配置问题，warn
            r.warned++;
            char buf[256];
            snprintf(buf, sizeof(buf),
                "  WARN lane[%d]: left_boundary=dashed but not same-direction adjacent\n", l.id);
            r.details += buf;
        }
    }

    // 3. 虚线段长 ~3m、间距 ~6–9m
    /* B-2 删除：此检查依赖 build_static_digest 中硬编码的 dash_length=3.0 /
     * gap_length=6.0（esmini 不暴露标线几何），检查恒为 pass，无法检测真实
     * 标线问题。RoadMarkingDigest 仍保留（未来接入真实标线数据后可恢复检查）。 */

    // 4. 可行驶区闭合检查
    if (sd.drivable_poly_x.size() >= 3) {
        size_t n = sd.drivable_poly_x.size();
        double dx = sd.drivable_poly_x[0] - sd.drivable_poly_x[n - 1];
        double dy = sd.drivable_poly_y[0] - sd.drivable_poly_y[n - 1];
        if (std::sqrt(dx * dx + dy * dy) > 0.1) {
            r.warned++;
            char buf[128];
            snprintf(buf, sizeof(buf),
                "  WARN drivable_poly: not closed (gap=%.2fm)\n",
                std::sqrt(dx * dx + dy * dy));
            r.details += buf;
        } else {
            r.passed++;
        }
    }

    // 5. 红绿灯朝向检查：朝向 · 车道行驶方向 < 0（面向来车）
    /* P2 清理：原检查项 5（路面高程连续检查）已删除——其依赖的
     * StaticDigest::height_samples_z 从未被 build_static_digest 填充，
     * size 永远 0，if 块恒不执行，属死分支。原检查项 6 重编号为 5。 */
    for (size_t i = 0; i < sd.traffic_lights.size(); ++i) {
        const auto& tl = sd.traffic_lights[i];
        // 找管辖车道方向
        for (size_t j = 0; j < sd.lanes.size(); ++j) {
            const auto& l = sd.lanes[j];
            if (l.road_id == tl.controlled_road_id && l.lane_id == tl.controlled_lane_id) {
                // 车道方向 = (centerline 末点 - 起点) 归一化
                if (l.centerline_x.size() >= 2) {
                    double ldx = l.centerline_x.back() - l.centerline_x[0];
                    double ldy = l.centerline_y.back() - l.centerline_y[0];
                    double ln = std::sqrt(ldx * ldx + ldy * ldy);
                    if (ln > 0.01) {
                        double tlx = std::cos(tl.heading);
                        double tly = std::sin(tl.heading);
                        double dot = (tlx * ldx + tly * ldy) / ln;
                        // 红绿灯朝向应面向来车，即 dot < 0
                        if (dot > 0.1) {
                            r.warned++;
                            char buf[256];
                            snprintf(buf, sizeof(buf),
                                "  WARN traffic_light[%d]: dot=%.3f (背对来车?)\n",
                                tl.id, dot);
                            r.details += buf;
                        } else {
                            r.passed++;
                        }
                    }
                }
                break;
            }
        }
    }

    // 6. 没有物体堆在 (0,0,0)
    {
        int at_origin = 0;
        for (const auto& l : sd.lanes) {
            if (l.centerline_x.size() > 0) {
                if (std::fabs(l.centerline_x[0]) < 0.01 &&
                    std::fabs(l.centerline_y[0]) < 0.01) {
                    at_origin++;
                }
            }
        }
        if (at_origin > 3) {
            r.warned++;
            r.details += "  WARN: multiple lanes at (0,0,0) — possible uninitialized road network\n";
        }
    }

    // 7. 每条 lane 中线落在可行驶多边形内
    if (sd.drivable_poly_x.size() >= 3) {
        for (const auto& l : sd.lanes) {
            for (size_t k = 0; k < l.centerline_x.size(); ++k) {
                double cx = l.centerline_x[k];
                double cy = l.centerline_y[k];
                // 射线法点-in-多边形
                bool inside = false;
                size_t np = sd.drivable_poly_x.size();
                for (size_t pi = 0, pj = np - 1; pi < np; pj = pi++) {
                    double xi = sd.drivable_poly_x[pi], yi = sd.drivable_poly_y[pi];
                    double xj = sd.drivable_poly_x[pj], yj = sd.drivable_poly_y[pj];
                    if (((yi > cy) != (yj > cy)) &&
                        (cx < (xj - xi) * (cy - yi) / (yj - yi) + xi)) {
                        inside = !inside;
                    }
                }
                if (!inside) {
                    r.warned++;
                    char buf[256];
                    snprintf(buf, sizeof(buf),
                        "  WARN lane[%d]: centerline[%zu]=(%.2f,%.2f) outside drivable polygon\n",
                        l.id, k, cx, cy);
                    r.details += buf;
                    break;  // 每个 lane 只报一次
                }
            }
        }
    }

    return r;
}

// ═══════════════════════════════════════════════════════════
// 空间 invariant 检查
// ═══════════════════════════════════════════════════════════

static const double EPS_Z = 0.5;        // 高度容差 (m)
static const double EPS_LATERAL = 1.0;  // 横向容差 (m)
static const double EPS_LATERAL_PARKED = 3.0;  // 路侧静止占位车允许停在路缘外一个车位宽
static const double MAX_SPEED_FACTOR = 1.5;

InvariantResult check_spatial_invariants(const DynamicDigest& d,
                                          const StaticDigest& sd,
                                          FlowRoadNetwork* roads) {
    InvariantResult r;
    double half_w = sd.road_half_width;
    if (half_w < 1.0) half_w = 10.0;  // 无路网时的默认值

    for (size_t i = 0; i < d.actors.size(); ++i) {
        const auto& a = d.actors[i];
        char tag[32];
        snprintf(tag, sizeof(tag), "actor[%d]", a.id);

        // 1. 高度检查：|z − roadHeight(x,y)| < ε
        if (roads && roads->loaded()) {
            WorldPos wp;
            if (roads->frenet_to_world(a.road_id, 0, a.s, a.lateral_offset, wp)) {
                double road_z = wp.z;
                if (std::fabs(a.pos[2] - road_z) > EPS_Z) {
                    r.failed++;
                    char buf[256];
                    snprintf(buf, sizeof(buf),
                        "  FAIL %s: z=%.2f road_z=%.2f diff=%.2f > %.2f (浮空/埋地)\n",
                        tag, a.pos[2], road_z, std::fabs(a.pos[2] - road_z), EPS_Z);
                    r.details += buf;
                } else {
                    r.passed++;
                }
            }
        }

        // 2. 横向范围：运动中的 actor 必须留在路面内；静止路侧泊车可位于路缘外一个车位宽
        /* ego 掉头机动豁免：三把方向要把车从本侧车道横穿到对向车道，
         * lateral_offset（Frenet 相对参考线）必然越过半路宽——这是机动
         * 本身的路径，不是"飞出路面"。飞行方向正确性由 speed/Δpos 检查
         * 兜底，这里只放行机动期 ego。 */
        double lateral_eps = EPS_LATERAL;
        if (a.type != 0 && std::fabs(a.speed) < 0.1) {
            lateral_eps = EPS_LATERAL_PARKED;
        }
        if (std::fabs(a.lateral_offset) > half_w + lateral_eps &&
            !(a.type == 0 && d.maneuver)) {
            r.failed++;
            char buf[256];
            snprintf(buf, sizeof(buf),
                "  FAIL %s: lateral_offset=%.2f > half_w=%.2f + eps=%.2f (飞出路面)\n",
                tag, std::fabs(a.lateral_offset), half_w, lateral_eps);
            r.details += buf;
        } else {
            r.passed++;
        }

        // 3. rotationY == headingToRotationY(heading)
        /* B-2 删除：此检查恒为 pass —— build_dynamic_digest 中
         * `ad.rotation_y = e.heading` 直接从 heading 赋值，二者必然相等，
         * 检查 |rotation_y - heading| < 0.01 永远成立，无法检测 ENU→THREE
         * 符号翻转 bug。rotation_y 字段保留，仅供调试观察；要恢复真实检查需
         * 让 rotation_y 由独立路径计算而非复制 heading。 */

        // 4. 速度范围：0 ≤ speed ≤ 1.5×限速
        double speed_limit = 33.3;  // 默认 120km/h
        if (roads && roads->loaded()) {
            speed_limit = roads->speed_limit(a.road_id, a.lane_id, a.s, 33.3);
        }
        if (a.speed < 0 || a.speed > MAX_SPEED_FACTOR * speed_limit) {
            if (a.speed > MAX_SPEED_FACTOR * speed_limit) {
                r.failed++;
                char buf[256];
                snprintf(buf, sizeof(buf),
                    "  FAIL %s: speed=%.2f > %.2f * %.2f (超速)\n",
                    tag, a.speed, MAX_SPEED_FACTOR, speed_limit);
                r.details += buf;
            }
        } else {
            r.passed++;
        }

        // 5. bbox 合理性检查（用范围而非硬编码参考值）
        /* 旧代码用硬编码 CAR_BBOX=[4.5,1.8,1.5] 做精确匹配，但场景中 NPC
         * 可能有不同尺寸（卡车 length=8m、行人 length=0.5m 等），导致误报。
         * 改为范围检查：length ∈ [0.3, 30]、width ∈ [0.3, 3.5]、height ∈ [0.5, 5.0]。 */
        if (a.bbox[0] < 0.3 || a.bbox[0] > 30.0 ||
            a.bbox[1] < 0.3 || a.bbox[1] > 3.5 ||
            a.bbox[2] < 0.5 || a.bbox[2] > 5.0) {
            r.failed++;
            char buf[256];
            snprintf(buf, sizeof(buf),
                "  FAIL %s: bbox=[%.2f,%.2f,%.2f] 超出合理范围 (尺度错)\n",
                tag, a.bbox[0], a.bbox[1], a.bbox[2]);
            r.details += buf;
        } else {
            r.passed++;
        }
    }

    // 6. 两 actor bbox 不重叠检查
    /* B-2 升级：从 WARN 升级为 FAIL。bbox 重叠意味着两车穿模/碰撞，是必须
     * 修复的硬错误，WARN 会被 evaluator 忽略。collision.cpp 已做 OBB SAT
     * 碰撞响应，此处 digest 检查是事后校验（采样间隔 20 帧），重叠即说明
     * 碰撞响应未生效或 NPC 被传送到重叠位置。 */
    for (size_t i = 0; i < d.actors.size(); ++i) {
        for (size_t j = i + 1; j < d.actors.size(); ++j) {
            const auto& a = d.actors[i];
            const auto& b = d.actors[j];
            double dx = std::fabs(a.pos[0] - b.pos[0]);
            double dy = std::fabs(a.pos[1] - b.pos[1]);
            double ox = (a.bbox[0] + b.bbox[0]) * 0.5;
            double oy = (a.bbox[1] + b.bbox[1]) * 0.5;
            if (dx < ox && dy < oy) {
                r.failed++;
                char buf[256];
                snprintf(buf, sizeof(buf),
                    "  FAIL actor[%d]↔actor[%d]: bbox 重叠 dx=%.2f<%.2f dy=%.2f<%.2f (碰撞/穿模)\n",
                    a.id, b.id, dx, ox, dy, oy);
                r.details += buf;
            }
        }
    }

    return r;
}

// ═══════════════════════════════════════════════════════════
// 运动方向 invariant 检查
// ═══════════════════════════════════════════════════════════

InvariantResult check_motion_direction(const DynamicDigest& d,
                                        const StaticDigest& sd,
                                        FlowRoadNetwork* roads) {
    (void)sd;
    InvariantResult r;
    const double SPEED_THRESH = 0.5;  // m/s，低于此值不检查方向

    for (size_t i = 0; i < d.actors.size(); ++i) {
        const auto& a = d.actors[i];
        if (a.speed < SPEED_THRESH) continue;
        if (a.type == 4) continue;  // 行人不检查

        char tag[32];
        snprintf(tag, sizeof(tag), "actor[%d]", a.id);

        /* ego 掉头机动豁免：三把方向掉头时车头要横穿路面（heading 扫过
         * ±π）并在 Phase 3 倒车，dot(forward,vel) 与 dot(forward,lane)
         * 在机动期必然越界——这是机动路径本身，不是"横着/倒着开"故障。
         * 倒车段的正确性由时序 Δs/Δpos 检查 + 控制层机动控制律兜底。 */
        const bool ego_maneuver_skip = (a.type == 0 && d.maneuver);

        // 1. dot(forward(heading), vel/|vel|) > cos(30°)
        double fwd_x = std::cos(a.heading);
        double fwd_y = std::sin(a.heading);
        double vel_norm = std::sqrt(a.vel[0] * a.vel[0] + a.vel[1] * a.vel[1]);
        if (vel_norm > 0.01) {
            double vx = a.vel[0] / vel_norm;
            double vy = a.vel[1] / vel_norm;
            double dot = fwd_x * vx + fwd_y * vy;
            if (dot < std::cos(M_PI / 6.0) && !ego_maneuver_skip) {  // cos(30°)
                r.failed++;
                char buf[256];
                snprintf(buf, sizeof(buf),
                    "  FAIL %s: dot(forward,vel)=%.3f < cos(30)=%.3f (车头≠前进方向, 横着/倒着开)\n",
                    tag, dot, std::cos(M_PI / 6.0));
                r.details += buf;
            } else {
                r.passed++;
            }
        }

        // 2. dot(forward(heading), lane_dir) > cos(45°)
        // 从 road network 取车道方向
        /* route_dir==0 = actor 尚未被指定到任何车道方向（掉头横穿路面期间
         * ego 的 route_dir 被清 0，车头合法扫过所有车道方向）。对没有车道
         * 方向的 actor 断言"车头要与车道方向一致"是无意义的——它当前不在
         * 跟车道上。掉头横穿的正确性由 wrong-way 门禁 + 时序 Δs 检查兜底。 */
        if (a.route_dir == 0) {
            r.passed++;
        } else if (roads && roads->loaded()) {
            WorldPos wp_s, wp_s1;
            if (roads->frenet_to_world(a.road_id, a.lane_id, a.s, 0, wp_s) &&
                roads->frenet_to_world(a.road_id, a.lane_id, a.s + 1.0, 0, wp_s1)) {
                double ldx = wp_s1.x - wp_s.x;
                double ldy = wp_s1.y - wp_s.y;
                double lnorm = std::sqrt(ldx * ldx + ldy * ldy);
                if (lnorm > 0.01) {
                    double ldx_n = ldx / lnorm;
                    double ldy_n = ldy / lnorm;
                    double dot_lane = fwd_x * ldx_n + fwd_y * ldy_n;
                    /* route_dir > 0: 顺行车，dot_lane 应 > cos(45)
                     * route_dir < 0: 对向来车，dot_lane 应 < -cos(45)（朝向相反=正确）
                     * 旧代码不区分 route_dir，对向来车恒判 FAIL。 */
                    bool dir_ok;
                    if (a.route_dir < 0) {
                        dir_ok = (dot_lane < -std::cos(M_PI / 4.0));
                    } else {
                        dir_ok = (dot_lane > std::cos(M_PI / 4.0));
                    }
                    if (!dir_ok && !ego_maneuver_skip) {
                        r.failed++;
                        char buf[256];
                        snprintf(buf, sizeof(buf),
                            "  FAIL %s: dot(forward,lane)=%.3f route_dir=%d (与车道方向不一致)\n",
                            tag, dot_lane, a.route_dir);
                        r.details += buf;
                    } else {
                        r.passed++;
                    }
                }
            }
        }

        // 3. sign(lane 允许方向) == sign(沿 s 前进)
        // 单行道：dir=固定；双行道：按 heading 判断
        if (a.route_dir != 0) {
            // 有 route_dir 表示已确定方向
            if (a.route_dir > 0 && a.speed > 0) {
                // 检查是否在倒退（s 减小）
                // 单帧无法检测 s 变化，此检查在时序 invariant 中完成
            }
        }
    }

    return r;
}

// ═══════════════════════════════════════════════════════════
// 时序 invariant 检查
// ═══════════════════════════════════════════════════════════

InvariantResult check_temporal_invariants(const DynamicDigest& prev,
                                           const DynamicDigest& curr,
                                           double dt) {
    InvariantResult r;
    if (dt <= 0) return r;

    const double V_MAX = 55.0;     // 200 km/h
    const double YAW_MAX = 1.5;    // rad/s
    /* 加速度物理上限（2026-08-04 修正）：-8 是舒适性阈值不是物理极限，
     * 真车急刹（ABS）可达 -10~-12 m/s² —— 红灯/跟车急刹被误判"运动学
     * 不可行"（实测 NPC 急刹 -11 误报 FAIL）。invariant 只抓物理不可行，
     * 急刹允许（-12），舒适性由控制层调。 */
    const double ACCEL_MIN = -12.0; // m/s²
    const double ACCEL_MAX = 4.0;  // m/s²

    for (size_t i = 0; i < curr.actors.size(); ++i) {
        const auto& ca = curr.actors[i];
        char tag[32];
        snprintf(tag, sizeof(tag), "actor[%d]", ca.id);

        // 找上一帧对应 actor
        const ActorDigest* pa = nullptr;
        for (size_t j = 0; j < prev.actors.size(); ++j) {
            if (prev.actors[j].id == ca.id) { pa = &prev.actors[j]; break; }
        }
        if (!pa) continue;

        // 1. Δpos ≈ vel × dt
        //    用绝对位置算 Δpos：digest 可能是 ego_centered（pos 相对 ego），
        //    此时若用相对位置算 Δpos，ego 移动会让静止 actor 也"瞬移"，
        //    而 expected 用的是绝对 speed，坐标系不一致必误报。
        //    绝对 pos = ad.pos + dd.origin。
        double abs_prev_x = pa->pos[0] + prev.origin[0];
        double abs_prev_y = pa->pos[1] + prev.origin[1];
        double abs_curr_x = ca.pos[0] + curr.origin[0];
        double abs_curr_y = ca.pos[1] + curr.origin[1];
        double dx = abs_curr_x - abs_prev_x;
        double dy = abs_curr_y - abs_prev_y;
        double dist = std::sqrt(dx * dx + dy * dy);
        /* 用平均速度估算期望位移，避免起步时 prev.speed=0 导致 expected≈0、
         * 实际位移因加速远大于 expected 而误报瞬移。
         * 例：红灯起步 2 m/s²，1s 位移 1m，但 prev.speed=0 → expected=0，
         * 阈值仅 0.5m → 误报。用平均速度 (prev+curr)/2 × dt 更合理。 */
        double avg_speed = (pa->speed + ca.speed) * 0.5;
        double expected = avg_speed * dt;

        // 设计内瞬移：choreography beat / recycle_npc 把 actor 显式传送到新位置，
        // 这是场景循环机制，非 bug。若上次采样帧到本次采样帧之间（含 prev 帧
        // 本身，因 beat 在 Step5 触发、digest 在 Step6 采样，prev 帧的 pos 已含
        // 传送）发生过传送，跳过 Δpos 检查 1 和 2（仍检查 heading/accel）。
        bool teleported = (ca.last_teleport_cycle >= (uint32_t)prev.frame &&
                           ca.last_teleport_cycle <= (uint32_t)curr.frame &&
                           ca.last_teleport_cycle > 0);
        if (teleported) {
            r.passed++;  // 跳过，计为通过
        } else if (dist > expected * 2.0 + 0.5) {
            r.failed++;
            char buf[256];
            snprintf(buf, sizeof(buf),
                "  FAIL %s: Δpos=%.3f >> expected=%.3f (瞬移/teleport tp_cycle=%u prev=%d curr=%d)\n",
                tag, dist, expected, ca.last_teleport_cycle, prev.frame, curr.frame);
            r.details += buf;
        } else {
            r.passed++;
        }

        // 2. |Δpos| ≤ v_max × dt
        if (!teleported && dist > V_MAX * dt + 0.5) {
            r.failed++;
            char buf[256];
            snprintf(buf, sizeof(buf),
                "  FAIL %s: Δpos=%.3f > v_max*dt=%.3f (超速瞬移)\n",
                tag, dist, V_MAX * dt);
            r.details += buf;
        }

        // 3. |Δheading| ≤ yaw_max × dt
        double dh = ca.heading - pa->heading;
        while (dh > M_PI) dh -= 2.0 * M_PI;
        while (dh < -M_PI) dh += 2.0 * M_PI;
        if (std::fabs(dh) > YAW_MAX * dt + 0.1) {
            r.failed++;
            char buf[256];
            snprintf(buf, sizeof(buf),
                "  FAIL %s: Δheading=%.4f > yaw_max*dt=%.4f (朝向瞬变)\n",
                tag, std::fabs(dh), YAW_MAX * dt);
            r.details += buf;
        } else {
            r.passed++;
        }

        // 4. accel ∈ [−8, +4] m/s²
        /* B-2 升级：从 WARN 升级为 FAIL。超出运动学可行加速度范围意味着
         * 物理积分出错或外部传送未标记，是必须修复的硬错误。 */
        double dv = ca.speed - pa->speed;
        double accel = dv / dt;
        if (accel < ACCEL_MIN || accel > ACCEL_MAX) {
            r.failed++;
            char buf[256];
            snprintf(buf, sizeof(buf),
                "  FAIL %s: accel=%.2f ∉ [%.0f,%.0f] m/s² (运动学不可行)\n",
                tag, accel, ACCEL_MIN, ACCEL_MAX);
            r.details += buf;
        } else {
            r.passed++;
        }

        // 5. anti-reverse: Δs 与 route_dir 方向一致（不倒车）
        /* B-2 新增：用 Δs 和 route_dir 检测倒车。route_dir>0 的顺行车 s 应递增，
         * route_dir<0 的对向车 s 应递减。同一 road 上 Δs 方向与 route_dir 相反
         * 即倒车（如顺行车 s 减小 = 倒退）。teleported 时 s 被重置，跳过检查。
         * 跨 road 时 s 范围重置，也跳过（road_id 不一致）。容差 0.5m 防数值噪声。
         * ego 掉头机动豁免：三把方向的 Phase 3 就是倒车段（s 合法减小），
         * 这不是逆行故障。掉头是否完成/走向由 behavior 状态机 + wrong-way
         * 门禁兜底。 */
        if (!teleported && !(ca.type == 0 && curr.maneuver)
            && ca.route_dir != 0 && pa->route_dir != 0
            && ca.route_dir == pa->route_dir && ca.road_id == pa->road_id) {
            double ds = ca.s - pa->s;
            if (ca.route_dir > 0 && ds < -0.5) {
                r.failed++;
                char buf[256];
                snprintf(buf, sizeof(buf),
                    "  FAIL %s: Δs=%.3f < 0 but route_dir=+1 (倒车/逆行)\n",
                    tag, ds);
                r.details += buf;
            } else if (ca.route_dir < 0 && ds > 0.5) {
                r.failed++;
                char buf[256];
                snprintf(buf, sizeof(buf),
                    "  FAIL %s: Δs=%.3f > 0 but route_dir=-1 (倒车/逆行)\n",
                    tag, ds);
                r.details += buf;
            } else {
                r.passed++;
            }
        }
    }

    return r;
}

// ═══════════════════════════════════════════════════════════
// ASCII 俯视渲染（调试可视化，终端无 GUI 时使用）
// ═══════════════════════════════════════════════════════════

std::string render_ascii_overhead(const StaticDigest& sd, const DynamicDigest& dd,
                                   int width_chars, int height_chars) {
    if (sd.lanes.empty()) return "(no lanes)";

    // 确定渲染范围（沿车道中心线扫描极值）
    double min_x = 1e9, max_x = -1e9, min_y = 1e9, max_y = -1e9;
    for (const auto& l : sd.lanes) {
        for (size_t i = 0; i < l.centerline_x.size(); ++i) {
            double cx = l.centerline_x[i];
            double cy = l.centerline_y[i];
            if (cx < min_x) min_x = cx;
            if (cx > max_x) max_x = cx;
            if (cy < min_y) min_y = cy;
            if (cy > max_y) max_y = cy;
        }
    }
    /* 红绿灯位置也纳入极值扫描——否则 4 车道场景灯杆落在 road_half_width 外侧
     * （7m 半宽 × 2 = 14m，再加臂 4m ≈ 18m 外），靠车道中线极值会被裁出画面，
     * 闭环调试看不到灯杆和车辆的相对位置关系。 */
    for (const auto& tl : sd.traffic_lights) {
        if (tl.x < min_x) min_x = tl.x;
        if (tl.x > max_x) max_x = tl.x;
        if (tl.y < min_y) min_y = tl.y;
        if (tl.y > max_y) max_y = tl.y;
    }
    // 扩展边界留白
    double margin = 20;
    min_x -= margin; max_x += margin;
    min_y -= margin; max_y += margin;
    double range_x = max_x - min_x;
    double range_y = max_y - min_y;
    if (range_x < 1) range_x = 1;
    if (range_y < 1) range_y = 1;

    // 等比缩放，保持纵横比
    double scale_x = (double)(width_chars - 2) / range_x;
    double scale_y = (double)(height_chars - 2) / range_y;
    double scale = std::min(scale_x, scale_y);

    auto to_grid = [&](double wx, double wy, int& gx, int& gy) {
        gx = 1 + (int)((wx - min_x) * scale);
        gy = 1 + (int)((wy - min_y) * scale);
        if (gx < 0) gx = 0;
        if (gx >= width_chars) gx = width_chars - 1;
        if (gy < 0) gy = 0;
        if (gy >= height_chars) gy = height_chars - 1;
    };

    std::vector<std::vector<char>> grid(height_chars, std::vector<char>(width_chars, ' '));

    // 绘制车道中心线（按左边界类型区分：双黄 #，其余 -）
    for (const auto& l : sd.lanes) {
        for (size_t i = 0; i < l.centerline_x.size(); ++i) {
            int gx, gy;
            to_grid(l.centerline_x[i], l.centerline_y[i], gx, gy);
            if (gx >= 0 && gx < width_chars && gy >= 0 && gy < height_chars) {
                grid[gy][gx] = (l.left_boundary_type == 2) ? '#' : '-';
            }
        }
    }

    // 绘制红绿灯（位置取自 sd.traffic_lights，相位取自 dd.traffic_light_states 按 id 关联）
    /* 顺序：先画灯（再画车辆），车辆若与灯同格会被车辆字符覆盖——这正是闭环
     * 调试要抓的 bug：灯杆落在路面内 = 与车重叠。看到车辆字符覆盖灯字符即报警。 */
    for (const auto& tl : sd.traffic_lights) {
        int gx, gy;
        to_grid(tl.x, tl.y, gx, gy);
        if (gx >= 0 && gx < width_chars && gy >= 0 && gy < height_chars) {
            // 查 id 对应的当前相位
            int phase = 0;  // 默认绿（找不到状态记录时）
            for (const auto& s : dd.traffic_light_states) {
                if (s.id == tl.id) { phase = s.phase_state; break; }
            }
            char c = (phase == 2) ? 'R' : (phase == 1) ? 'Y' : 'G';
            grid[gy][gx] = c;
        }
    }

    // 绘制演员（ego=位置 0，行人=*, 车辆=朝向箭头）
    for (const auto& a : dd.actors) {
        int gx, gy;
        double wx = a.pos[0] + dd.origin[0];
        double wy = a.pos[1] + dd.origin[1];
        to_grid(wx, wy, gx, gy);
        if (gx >= 0 && gx < width_chars && gy >= 0 && gy < height_chars) {
            double h = a.heading;
            char dir = 'C';
            if (std::fabs(std::cos(h)) > 0.7)      dir = (std::cos(h) > 0) ? '>' : '<';
            else if (std::fabs(std::sin(h)) > 0.7) dir = (std::sin(h) > 0) ? '^' : 'v';
            else if (std::cos(h) > 0 && std::sin(h) > 0)  dir = '7';
            else if (std::cos(h) > 0 && std::sin(h) < 0)  dir = 'L';
            else if (std::cos(h) < 0 && std::sin(h) > 0)  dir = 'J';
            else                                          dir = '\\';
            grid[gy][gx] = (a.type == 0) ? 'E' :      // ego
                           (a.type == 4) ? '*' : dir;  // pedestrian or vehicle
        }
    }

    // 组装输出（Unicode 边框 + 图例）
    std::string out;
    out += "┌";
    for (int i = 0; i < width_chars - 2; ++i) out += "─";
    out += "┐\n";
    for (int y = 0; y < height_chars; ++y) {
        out += "│";
        for (int x = 0; x < width_chars; ++x) out += grid[y][x];
        out += "│\n";
    }
    out += "└";
    for (int i = 0; i < width_chars - 2; ++i) out += "─";
    out += "┘\n";
    out += "E=ego C=car *=pedestrian ><^v=朝向 -=车道线 #=双黄 G/Y/R=灯(绿黄红)\n";
    out += "frame:" + std::to_string(dd.frame) + " time:" + std::to_string(dd.sim_time) + "\n";
    return out;
}

}  // namespace flowsim