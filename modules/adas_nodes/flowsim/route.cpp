/**
 * route.cpp — Route 构建与查询实现
 */

#include "route.h"

#include "road_network.h"

#include <algorithm>
#include <cmath>

namespace flowsim {

namespace {
/** 归一化到 [0,π] 的航向差绝对值。 */
double heading_gap(double a, double b) {
    double d = a - b;
    while (d > M_PI)  d -= 2.0 * M_PI;
    while (d < -M_PI) d += 2.0 * M_PI;
    return std::fabs(d);
}
}  // namespace

bool Route::build(FlowRoadNetwork& roads, double tol) {
    segs_.clear();
    all_lengths_.clear();
    total_ = 0.0;

    const int n = roads.road_count();
    if (n <= 0) return false;

    struct RoadEnds {
        int    id{0};
        double len{0.0};
        double sx{0}, sy{0}, sh{0};   // start 端点 + 航向
        double ex{0}, ey{0}, eh{0};   // end 端点 + 航向
        bool   used{false};
    };
    std::vector<RoadEnds> rs;
    rs.reserve(n);

    for (int i = 0; i < n; ++i) {
        RoadInfo info;
        if (!roads.road_info(i, info) || info.length <= 0.0) continue;
        WorldPos a, b;
        if (!roads.frenet_to_world((int)info.id, 0, 0.0, 0.0, a)) continue;
        if (!roads.frenet_to_world((int)info.id, 0, info.length, 0.0, b)) continue;
        rs.push_back({(int)info.id, info.length, a.x, a.y, a.h, b.x, b.y, b.h, false});
        all_lengths_.push_back({(int)info.id, info.length});
    }
    if (rs.empty()) return false;

    // 选起点：start 端点不与任何其它 road 的 end 端点重合的 road（无前驱）。
    auto is_pred = [&](const RoadEnds& p, const RoadEnds& q) {
        return std::hypot(p.ex - q.sx, p.ey - q.sy) < tol;
    };
    int seed = -1;
    for (size_t i = 0; i < rs.size(); ++i) {
        bool has_pred = false;
        for (size_t j = 0; j < rs.size(); ++j) {
            if (i == j) continue;
            if (is_pred(rs[j], rs[i])) { has_pred = true; break; }
        }
        if (!has_pred) { seed = (int)i; break; }
    }
    if (seed < 0) seed = 0;  // 全环路：随便从 0 起

    // 沿后继链构建
    int    cur = seed;
    double acc = 0.0;
    while (cur >= 0 && !rs[cur].used) {
        rs[cur].used = true;
        segs_.push_back({rs[cur].id, rs[cur].len, acc});
        acc += rs[cur].len;

        int    best = -1;
        double best_score = 1e18;
        for (size_t j = 0; j < rs.size(); ++j) {
            if (rs[j].used) continue;
            double d = std::hypot(rs[cur].ex - rs[j].sx, rs[cur].ey - rs[j].sy);
            if (d > tol) continue;
            // 航向连续优先；长度更长的减分（tiebreak：主路 > 短存根/急弯匝道）
            double score = heading_gap(rs[cur].eh, rs[j].sh) - 0.001 * rs[j].len;
            if (score < best_score) { best_score = score; best = (int)j; }
        }
        cur = best;
    }

    total_ = acc;
    return !segs_.empty();
}

int Route::index_of(int road_id) const {
    for (size_t i = 0; i < segs_.size(); ++i) {
        if (segs_[i].road_id == road_id) return (int)i;
    }
    return -1;
}

double Route::road_length(int road_id) const {
    for (const auto& p : all_lengths_) {
        if (p.first == road_id) return p.second;
    }
    return 0.0;
}

void Route::locate(double route_s, int& road_id, double& s_local, int& route_idx) const {
    if (segs_.empty()) { road_id = 0; s_local = 0; route_idx = -1; return; }
    if (route_s < 0.0)      route_s = 0.0;
    if (route_s > total_)   route_s = total_;

    for (size_t i = 0; i < segs_.size(); ++i) {
        double seg_end = segs_[i].s_start + segs_[i].length;
        if (route_s <= seg_end || i + 1 == segs_.size()) {
            route_idx = (int)i;
            road_id   = segs_[i].road_id;
            s_local   = route_s - segs_[i].s_start;
            if (s_local < 0.0)              s_local = 0.0;
            if (s_local > segs_[i].length)  s_local = segs_[i].length;
            return;
        }
    }
    // 理论不可达（上面循环末段兜底）
    route_idx = (int)segs_.size() - 1;
    road_id   = segs_.back().road_id;
    s_local   = segs_.back().length;
}

double Route::to_route_s(int route_idx, double s_local) const {
    if (route_idx < 0 || route_idx >= (int)segs_.size()) return 0.0;
    return segs_[route_idx].s_start + s_local;
}

int Route::sample_ahead(FlowRoadNetwork& roads, double route_s_start,
                        double lookahead, double step_m,
                        std::vector<RefPathPoint>& out) const {
    out.clear();
    if (segs_.empty() || lookahead <= 0.0 || step_m <= 0.0) return 0;

    // 起点 route_s 夹到 [0, total]
    if (route_s_start < 0.0)      route_s_start = 0.0;
    if (route_s_start > total_)   route_s_start = total_;

    // 第一遍：采样原始点 (route_s, x, y, h)，跳过 frenet_to_world 失败的点
    struct Raw { double rs; double x; double y; double h; };
    std::vector<Raw> raw;
    raw.reserve(static_cast<size_t>(lookahead / step_m) + 2);

    double rs_end = route_s_start + lookahead;
    if (rs_end > total_) rs_end = total_;

    for (double rs = route_s_start; rs <= rs_end + 1e-6; rs += step_m) {
        int rid = 0, ridx = -1;
        double sl = 0.0;
        locate(rs, rid, sl, ridx);
        WorldPos wp;
        if (!roads.frenet_to_world(rid, 0, sl, 0.0, wp)) continue;
        raw.push_back({rs, wp.x, wp.y, wp.h});
        if (rs == rs_end) break;  // 避免浮点循环多采一个
    }
    if (raw.size() < 2) {
        // 至少把所有原始点填进去（即使无法估计 kappa）
        for (const auto& r : raw) {
            out.push_back({r.x, r.y, r.h, 0.0, r.rs});
        }
        return static_cast<int>(out.size());
    }

    // 第二遍：中心差分估计曲率 kappa
    //   相邻两点的转角 dtheta = normalize(h[i+1] - h[i-1])
    //   弦长 chord = hypot(dx, dy)
    //   kappa = dtheta / chord
    // 端点用前向/后向差分
    auto norm_angle = [](double a) {
        while (a >  M_PI) a -= 2.0 * M_PI;
        while (a < -M_PI) a += 2.0 * M_PI;
        return a;
    };

    for (size_t i = 0; i < raw.size(); ++i) {
        double dtheta = 0.0, chord = 0.0;
        if (i == 0) {
            dtheta = norm_angle(raw[1].h - raw[0].h);
            chord  = std::hypot(raw[1].x - raw[0].x, raw[1].y - raw[0].y);
        } else if (i + 1 == raw.size()) {
            dtheta = norm_angle(raw[i].h - raw[i - 1].h);
            chord  = std::hypot(raw[i].x - raw[i - 1].x, raw[i].y - raw[i - 1].y);
        } else {
            dtheta = norm_angle(raw[i + 1].h - raw[i - 1].h);
            chord  = std::hypot(raw[i + 1].x - raw[i - 1].x, raw[i + 1].y - raw[i - 1].y);
        }
        double kappa = (chord > 1e-6) ? (dtheta / chord) : 0.0;
        out.push_back({raw[i].x, raw[i].y, raw[i].h, kappa, raw[i].rs});
    }

    return static_cast<int>(out.size());
}

}  // namespace flowsim
