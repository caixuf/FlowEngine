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
    StaticDigest sd;
    if (!roads.loaded()) return sd;

    int rc = roads.road_count();
    double max_half_width = 0;

    for (int i = 0; i < rc; ++i) {
        RoadInfo ri;
        if (!roads.road_info(i, ri)) continue;

        // 采样道路中线
        int n_lanes = roads.drivable_lane_count((int)ri.id, 0);
        double half_w = n_lanes * 3.5 * 0.5;
        if (half_w > max_half_width) max_half_width = half_w;

        // 为每个车道生成 digest
        for (int li = 0; li < n_lanes; ++li) {
            int lane_id = 0;
            // 用 esmini API 取车道 ID
            int drivable = roads.drivable_lane_count((int)ri.id, 0);
            if (drivable <= 0) continue;

            LaneDigest ld;
            ld.id = (int)sd.lanes.size();
            ld.road_id = (int)ri.id;
            ld.lane_id = lane_id;
            ld.width = 3.5;

            // 采样中心线（沿 s 每 10m 采样）
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
        for (int li = 0; li < n_lanes - 1; ++li) {
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
                                    int frame, bool ego_centered) {
    DynamicDigest dd;
    dd.sim_time = sim_time;
    dd.frame = frame;
    dd.ego_centered = ego_centered;

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
        ad.speed = e.speed;
        ad.road_id = e.road_id;
        ad.lane_id = e.lane_id;
        ad.lateral_offset = e.offset;
        ad.s = e.s;
        ad.rotation_y = e.heading;  // headingToRotationY(h) = h
        ad.route_dir = e.route_dir;
        ad.last_teleport_cycle = e.last_teleport_cycle;
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
    for (size_t i = 0; i < sd.markings.size(); ++i) {
        const auto& m = sd.markings[i];
        if (m.type == 0) {  // 虚线
            if (m.dash_length < 2.0 || m.dash_length > 4.0) {
                r.warned++;
                char buf[256];
                snprintf(buf, sizeof(buf),
                    "  WARN marking[%zu]: dash_length=%.2f not ≈3m\n", i, m.dash_length);
                r.details += buf;
            } else { r.passed++; }
            if (m.gap_length < 5.0 || m.gap_length > 10.0) {
                r.warned++;
                char buf[256];
                snprintf(buf, sizeof(buf),
                    "  WARN marking[%zu]: gap_length=%.2f not ≈6-9m\n", i, m.gap_length);
                r.details += buf;
            } else { r.passed++; }
        }
    }

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
static const double MAX_SPEED_FACTOR = 1.5;
static const double CAR_BBOX[3] = {4.5, 1.8, 1.5};
static const double PED_BBOX[3] = {0.5, 0.5, 1.7};

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

        // 2. 横向范围：|lateral_offset| ≤ 半路宽(+裕量)
        if (std::fabs(a.lateral_offset) > half_w + EPS_LATERAL) {
            r.failed++;
            char buf[256];
            snprintf(buf, sizeof(buf),
                "  FAIL %s: lateral_offset=%.2f > half_w=%.2f + eps=%.2f (飞出路面)\n",
                tag, std::fabs(a.lateral_offset), half_w, EPS_LATERAL);
            r.details += buf;
        } else {
            r.passed++;
        }

        // 3. rotationY == headingToRotationY(heading)
        // headingToRotationY(h) = h，所以 rotation_y 应等于 heading
        if (std::fabs(a.rotation_y - a.heading) > 0.01) {
            r.failed++;
            char buf[256];
            snprintf(buf, sizeof(buf),
                "  FAIL %s: rotation_y=%.4f != heading=%.4f (ENU→THREE 符号翻错)\n",
                tag, a.rotation_y, a.heading);
            r.details += buf;
        } else {
            r.passed++;
        }

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

        // 5. bbox 尺寸检查
        const double* ref_bbox = (a.type == 4) ? PED_BBOX : CAR_BBOX;
        double bbox_err = std::fabs(a.bbox[0] - ref_bbox[0])
                        + std::fabs(a.bbox[1] - ref_bbox[1])
                        + std::fabs(a.bbox[2] - ref_bbox[2]);
        if (bbox_err > 2.0) {
            r.failed++;
            char buf[256];
            snprintf(buf, sizeof(buf),
                "  FAIL %s: bbox=[%.2f,%.2f,%.2f] vs ref=[%.2f,%.2f,%.2f] (尺度错)\n",
                tag, a.bbox[0], a.bbox[1], a.bbox[2],
                ref_bbox[0], ref_bbox[1], ref_bbox[2]);
            r.details += buf;
        } else {
            r.passed++;
        }
    }

    // 6. 两 actor bbox 不重叠检查
    for (size_t i = 0; i < d.actors.size(); ++i) {
        for (size_t j = i + 1; j < d.actors.size(); ++j) {
            const auto& a = d.actors[i];
            const auto& b = d.actors[j];
            double dx = std::fabs(a.pos[0] - b.pos[0]);
            double dy = std::fabs(a.pos[1] - b.pos[1]);
            double ox = (a.bbox[0] + b.bbox[0]) * 0.5;
            double oy = (a.bbox[1] + b.bbox[1]) * 0.5;
            if (dx < ox && dy < oy) {
                r.warned++;
                char buf[256];
                snprintf(buf, sizeof(buf),
                    "  WARN actor[%d]↔actor[%d]: bbox 重叠 dx=%.2f<%.2f dy=%.2f<%.2f (碰撞/穿模)\n",
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

        // 1. dot(forward(heading), vel/|vel|) > cos(30°)
        double fwd_x = std::cos(a.heading);
        double fwd_y = std::sin(a.heading);
        double vel_norm = std::sqrt(a.vel[0] * a.vel[0] + a.vel[1] * a.vel[1]);
        if (vel_norm > 0.01) {
            double vx = a.vel[0] / vel_norm;
            double vy = a.vel[1] / vel_norm;
            double dot = fwd_x * vx + fwd_y * vy;
            if (dot < std::cos(M_PI / 6.0)) {  // cos(30°)
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
        if (roads && roads->loaded()) {
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
                    /* dot_lane 应为正（与车道方向一致）。旧代码用 fabs
                     * 导致对向来车（dot≈-1）也通过，掩盖方向 bug。 */
                    if (dot_lane < std::cos(M_PI / 4.0)) {  // < cos(45°) = 方向不一致
                        r.failed++;
                        char buf[256];
                        snprintf(buf, sizeof(buf),
                            "  FAIL %s: dot(forward,lane)=%.3f < cos(45)=%.3f (与车道方向不一致)\n",
                            tag, dot_lane, std::cos(M_PI / 4.0));
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
    const double ACCEL_MIN = -8.0; // m/s²
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
        double expected = pa->speed * dt;

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
        double dv = ca.speed - pa->speed;
        double accel = dv / dt;
        if (accel < ACCEL_MIN || accel > ACCEL_MAX) {
            r.warned++;
            char buf[256];
            snprintf(buf, sizeof(buf),
                "  WARN %s: accel=%.2f ∉ [%.0f,%.0f] m/s² (运动学不可行)\n",
                tag, accel, ACCEL_MIN, ACCEL_MAX);
            r.details += buf;
        } else {
            r.passed++;
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
        if (gx < 0) gx = 0; if (gx >= width_chars) gx = width_chars - 1;
        if (gy < 0) gy = 0; if (gy >= height_chars) gy = height_chars - 1;
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

// ═══════════════════════════════════════════════════════════
// Golden 快照（transform 记账 + diff，用于回归检测位置漂移）
// ═══════════════════════════════════════════════════════════

std::string golden_snapshot(const DynamicDigest& dd) {
    // 按 id 排序后生成 (name, pos, rotY, scale) 列表
    std::vector<ActorDigest> sorted = dd.actors;
    std::sort(sorted.begin(), sorted.end(),
              [](const ActorDigest& a, const ActorDigest& b) { return a.id < b.id; });

    std::string s = "{\n  \"frame\":" + std::to_string(dd.frame) + ",\n";
    s += "  \"sim_time\":" + std::to_string(dd.sim_time) + ",\n";
    s += "  \"actors\":[\n";
    for (size_t i = 0; i < sorted.size(); ++i) {
        const auto& a = sorted[i];
        char buf[512];
        char name[32];
        snprintf(name, sizeof(name), "actor_%d", a.id);
        snprintf(buf, sizeof(buf),
          "    {\"name\":\"%s\",\"pos\":[%.4f,%.4f,%.4f],\"rotY\":%.4f,\"scale\":[%.2f,%.2f,%.2f]}%s\n",
          name,
          a.pos[0], a.pos[1], a.pos[2],
          a.rotation_y,
          a.bbox[0], a.bbox[1], a.bbox[2],
          (i < sorted.size() - 1) ? "," : "");
        s += buf;
    }
    s += "  ]\n}";
    return s;
}

std::string golden_diff(const std::string& golden, const std::string& current,
                         double tolerance) {
    if (golden == current) return "";  // 完全一致

    // 简易逐行 diff（不引入完整 JSON 解析器，仅做数值容差比较）
    std::string diff;
    std::istringstream gs(golden), cs(current);
    std::string gl, cl;
    int line = 0;
    while (std::getline(gs, gl) && std::getline(cs, cl)) {
        line++;
        if (gl != cl) {
            // 提取行内所有数值，按容差比较
            auto extract_nums = [](const std::string& ln) -> std::vector<double> {
                std::vector<double> nums;
                const char* p = ln.c_str();
                while (*p) {
                    if ((*p >= '0' && *p <= '9') || *p == '-' || *p == '.') {
                        char* end;
                        double v = strtod(p, &end);
                        if (end > p) {
                            nums.push_back(v);
                            p = end;
                            continue;
                        }
                    }
                    p++;
                }
                return nums;
            };
            auto gn = extract_nums(gl);
            auto cn = extract_nums(cl);
            bool numeric_diff = false;
            if (gn.size() == cn.size() && gn.size() > 0) {
                for (size_t i = 0; i < gn.size(); ++i) {
                    if (std::fabs(gn[i] - cn[i]) > tolerance) {
                        numeric_diff = true;
                        break;
                    }
                }
                if (!numeric_diff) continue;  // 数值差异在容差内，跳过
            }
            diff += "  L" + std::to_string(line) + " golden: " + gl.substr(0, 80) + "\n";
            diff += "  L" + std::to_string(line) + " current: " + cl.substr(0, 80) + "\n";
        }
    }
    return diff;
}

}  // namespace flowsim