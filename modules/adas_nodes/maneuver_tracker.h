// maneuver_tracker.h — 通用机动跟踪器（C++ 移植）
//
// 规格：tools/maneuver_tracker.py（Python 原型，掉头 + 倒车入库双 PASS）。
// 目标（回应"遇新操作就改好久、没通用能力"）：
//   机动 = 参考轨迹数据（点列 + 带符号 v），执行 = 这一个跟踪器。
//   新操作（倒库/侧方/坡道）= 加一份轨迹数据，不改执行逻辑。
//
// 核心设计（vs 掉头 5 层特例 gate）：
//   1. 弧长推进 s 单调，不猜段 —— 段归属由 s 在轨迹上的位置唯一决定，
//      掉头 debug 的 D/R 交界几何歧义（prog_i 锁段、heading gate）根源
//      是"几何最近点在两条弧间跳"。用弧长参数天生无歧义。
//   2. 挡位 = 执行点 v 符号（v<0→R），换挡滞回 + 刹停由 s 处 v 决定。
//   3. 横向 = kappa 前馈 + 车体系 e_lat/dh 反馈（倒挡反馈反号）。
//   4. 速度 = s 处 |v|（执行点），6m 前视 min|v| 同挡段，下限 1.5。
//
// 纵向 PID 留在 control_node（tracker 只算 steer/target_speed/gear/
// gear_pending），复用已验证的 C++ PID（reverse mirror + anti-windup +
// SHIFT_STOP）。
//
// 纯算法：只依赖 <cmath>/<cstring>/<algorithm> + adas_msgs_gen.h
// （TrajectoryPoint）。无 transport/param/framework 依赖，可独立单测。
// header-only，编译进 control_node 插件和 test_maneuver_tracker 测试。
#pragma once
#include <cmath>
#include <cstdint>
#include <cstring>
#include <algorithm>

#include "adas_msgs_gen.h"          // TrajectoryPoint

namespace maneuver {

struct ManeuverTrackerParams {
    double segment_v_threshold = 0.1;   // v < -this => reverse（段边界测试）
    double heading_gate_rad    = 0.4;   // 跨 D/R 边界需车头接近下一段起点
    double diverge_guard_rad   = 1.2;   // |dh| vs exec heading > 此值 => 暂停 s
    double lookahead_m         = 2.0;   // 横向前视弧长（同挡段）
    double speed_scan_m        = 6.0;   // 目标速度 min|v| 扫描窗口
    double speed_floor_mps     = 1.5;   // 目标速度下限
    double gear_v_threshold    = 0.3;   // 向前扫第一个 |v|>此值 => gear 意图
    double gear_pending_speed  = 0.8;   // |speed|>此值 => 换挡前刹停
    double lat_gain_dh         = 0.8;   // heading 误差反馈增益
    double lat_gain_elat       = 0.25;  // 车体系横向误差增益
    double max_steer           = 0.60;  // rad 满舵
};

struct TrackPoint {
    double x = 0, y = 0, heading = 0, kappa = 0, v = 0, l = 0;
};

struct ManeuverResult {
    double steer        = 0.0;   // rad，已 clip
    double target_speed = 0.0;   // 带符号（负=倒挡）
    int    gear         = 1;     // +1 DRIVE / -1 REVERSE
    bool   gear_pending = false;
};

class ManeuverTracker {
public:
    static constexpr int kMaxPts = 64;   // 与 Trajectory::points[64] 一致

    ManeuverTracker() { std::memset(pts_, 0, sizeof(pts_)); }

    /// 机动轨迹新到达（maneuver_mode 0→1）：重置 s/gear，建 cum_s。
    void init(const TrajectoryPoint* pts, int n, double wheelbase) {
        s_ = 0.0;
        gear_ = 1;
        setTrajectory(pts, n, wheelbase);
    }

    /// 每机动帧：刷新点 + cum_s（planning 可能 >40m cache miss 重生成），保留 s/gear。
    void setTrajectory(const TrajectoryPoint* pts, int n, double wheelbase) {
        n_ = std::min(n, (int)kMaxPts);
        if (n_ > 0) std::memcpy(pts_, pts, sizeof(TrajectoryPoint) * (size_t)n_);
        wheelbase_ = wheelbase;
        rebuildCumS();
        int i = 0;
        exec(&i);           // 刷新缓存参考几何（on_trajectory init 后可读）
        lookahead(i);
    }

    void setParams(const ManeuverTrackerParams& p) { p_ = p; }

    bool hasTrajectory() const { return n_ > 0; }
    double arcLengthS() const { return s_; }
    bool done() const { return n_ > 0 && s_ >= cum_s_[n_ - 1]; }

    /// 参考几何（control 的 target_path_*/lane_d 簿记用）。
    const TrackPoint& execPoint() const { return last_exec_; }
    const TrackPoint& lookaheadPoint() const { return last_lookahead_; }

    /// 镜像 Python ManeuverTracker.tick()。只返回跟随器输出；纵向 PID 在 control。
    ManeuverResult tick(double ego_x, double ego_y, double ego_heading,
                        double current_speed, double dt);

private:
    TrajectoryPoint pts_[kMaxPts];
    double cum_s_[kMaxPts];
    int    n_ = 0;
    double wheelbase_ = 2.7;
    double s_ = 0.0;
    int    gear_ = 1;
    ManeuverTrackerParams p_;
    TrackPoint last_exec_, last_lookahead_;

    void rebuildCumS() {
        cum_s_[0] = 0.0;
        for (int i = 1; i < n_; i++) {
            cum_s_[i] = cum_s_[i - 1] +
                std::hypot((double)pts_[i].x - (double)pts_[i - 1].x,
                           (double)pts_[i].y - (double)pts_[i - 1].y);
        }
    }

    static double normAngle(double a) {
        while (a >  M_PI) a -= 2.0 * M_PI;
        while (a < -M_PI) a += 2.0 * M_PI;
        return a;
    }

    static TrackPoint toTrack(const TrajectoryPoint& p) {
        TrackPoint t;
        t.x = p.x; t.y = p.y; t.heading = p.heading;
        t.kappa = p.kappa; t.v = p.v; t.l = p.l;
        return t;
    }

    /// 弧长推进：s 按车实际行驶距离单调推进（Python _advance_s）。
    void advanceS(double ego_heading, double current_speed, double dt) {
        TrackPoint ex = exec(nullptr);
        double dh = normAngle(ex.heading - ego_heading);
        if (std::fabs(dh) > p_.diverge_guard_rad) return;   // 车脱离执行方向，暂停推进
        double new_s = s_ + std::fabs(current_speed) * dt;
        // 段边界 guard：跨 v 符号变化点，若车头还没转到接近下一段起点，停在边界
        for (int i = 0; i < n_ - 1; i++) {
            bool a = (double)pts_[i + 1].v < -p_.segment_v_threshold;
            bool b = (double)pts_[i].v     < -p_.segment_v_threshold;
            if (a != b && s_ < cum_s_[i] && cum_s_[i] < new_s) {
                double next_h = pts_[i + 1].heading;
                if (std::fabs(normAngle(next_h - ego_heading)) > p_.heading_gate_rad)
                    new_s = cum_s_[i];
            }
        }
        s_ = std::min(new_s, cum_s_[n_ - 1]);
    }

    /// 执行点：当前弧长处的轨迹点，线性插值（Python _exec）。
    TrackPoint exec(int* out_i) {
        int i = 0;
        while (i + 1 < n_ && cum_s_[i + 1] < s_) i++;
        TrackPoint pt;
        if (i + 1 >= n_) {
            pt = toTrack(pts_[n_ - 1]);
            if (out_i) *out_i = n_ - 1;
            last_exec_ = pt;
            return pt;
        }
        double span = cum_s_[i + 1] - cum_s_[i];
        double frac = (span > 1e-9) ? (s_ - cum_s_[i]) / span : 0.0;
        if (frac < 0.0) frac = 0.0;
        if (frac > 1.0) frac = 1.0;   // 防 s==末点 1-ulp 外推
        const TrajectoryPoint& a = pts_[i];
        const TrajectoryPoint& b = pts_[i + 1];
        pt.x = a.x + (b.x - a.x) * frac;
        pt.y = a.y + (b.y - a.y) * frac;
        pt.heading = normAngle(a.heading + normAngle(b.heading - a.heading) * frac);
        pt.kappa = a.kappa + (b.kappa - a.kappa) * frac;
        pt.v = a.v + (b.v - a.v) * frac;
        pt.l = a.l + (b.l - a.l) * frac;   // 仅 control lane_d 簿记用
        if (out_i) *out_i = i;
        last_exec_ = pt;
        return pt;
    }

    /// 前视点：执行点前视 ~2m 弧长，同一挡段内（跨 D/R 边界就停）（Python _lookahead）。
    TrackPoint lookahead(int base_i) {
        bool rev = (double)pts_[base_i].v < -p_.segment_v_threshold;
        TrackPoint target = toTrack(pts_[base_i]);
        int j = base_i;
        double arc = 0.0;
        while (j + 1 < n_ && arc < p_.lookahead_m) {
            bool next_rev = (double)pts_[j + 1].v < -p_.segment_v_threshold;
            if (next_rev != rev) break;
            arc += std::hypot((double)pts_[j + 1].x - (double)pts_[j].x,
                              (double)pts_[j + 1].y - (double)pts_[j].y);
            j++;
            target = toTrack(pts_[j]);
        }
        last_lookahead_ = target;
        return target;
    }

    /// 挡位：默认跟随执行点当前段；若前视 speed_scan_m 内出现相反挡段，
    /// 则提前请求刹停换挡。这样短机动（倒库/侧方）不会因为 s 逼近太慢而错过换挡。
    bool updateGear(int base_i, double current_speed) {
        int want = gear_;
        const double base_v = (double)pts_[base_i].v;
        const bool has_base_motion = (std::fabs(base_v) > p_.gear_v_threshold);
        if (has_base_motion) want = (base_v < 0) ? -1 : 1;

        double arc = 0.0;
        for (int i = base_i; i < n_ - 1 && arc <= p_.speed_scan_m; i++) {
            arc += std::hypot((double)pts_[i + 1].x - (double)pts_[i].x,
                              (double)pts_[i + 1].y - (double)pts_[i].y);
            const double v = (double)pts_[i + 1].v;
            if (std::fabs(v) <= p_.gear_v_threshold) continue;
            const int candidate = (v < 0) ? -1 : 1;
            if (!has_base_motion) { want = candidate; break; }
            if (candidate != want) { want = candidate; break; }
        }
        if (want != gear_ && std::fabs(current_speed) > p_.gear_pending_speed)
            return true;   // gear_pending：带速想换挡，本帧刹停
        gear_ = want;
        return false;
    }
};

// ── 方法体（header-only inline）──

inline ManeuverResult ManeuverTracker::tick(double ego_x, double ego_y,
                                            double ego_heading,
                                            double current_speed, double dt) {
    advanceS(ego_heading, current_speed, dt);
    int exec_i = 0;
    TrackPoint exec_pt = exec(&exec_i);
    bool gear_pending = updateGear(exec_i, current_speed);

    // 目标速度方向跟随「已落定的 gear」，而非插值 exec v：
    // D/R 边界处 exec 在 v=-3 与 v=+3.5 间插值，v 符号在边界模糊（可能跨 0），
    // 若用 exec_rev 会与 gear 不一致 → 倒挡期拿到正向 target → REV_BRAKE 卡死
    // （2026-08-04 实测掉头倒车后 spd 卡 0 直到 40s timeout）。
    bool exec_rev = (gear_ == -1);
    // 目标速度幅值 = |执行点 v|，前视 speed_scan_m 内同挡段最小 |v|，下限 floor
    double mag = std::fabs(exec_pt.v);
    double arc = 0.0;
    for (int i = exec_i; i < n_ - 1; i++) {
        arc += std::hypot((double)pts_[i + 1].x - (double)pts_[i].x,
                          (double)pts_[i + 1].y - (double)pts_[i].y);
        bool next_rev = (double)pts_[i + 1].v < -p_.segment_v_threshold;
        if (arc > p_.speed_scan_m || next_rev != exec_rev) break;
        // 与 updateGear 一致：跳过 v≈0 的换挡刹停/驻停点，否则 min|v| 被
        // 压成 floor，车在短前进段里 0.5m/s 爬行，200 tick 到不了 D/R 边界
        // 进不了倒挡（parking/parallel CI 失败，2026-08-05 实测）。
        if (std::fabs((double)pts_[i + 1].v) > p_.gear_v_threshold)
            mag = std::min(mag, std::fabs((double)pts_[i + 1].v));
    }
    mag = std::max(mag, p_.speed_floor_mps);
    double target_speed = exec_rev ? -mag : mag;

    // 轨迹终点 = 机动完成 → 目标 0，车刹停（不继续反向冲）
    if (s_ >= cum_s_[n_ - 1]) { target_speed = 0.0; gear_ = 1; }
    if (gear_pending) target_speed = 0.0;

    // 横向：kappa 前馈 + 车体系 e_lat/dh 反馈，倒挡反馈反号
    TrackPoint tgt = lookahead(exec_i);
    bool rev = (gear_ == -1);
    double dh = normAngle(tgt.heading - ego_heading);
    double e_lat = -std::sin(tgt.heading) * (tgt.x - ego_x)
                   + std::cos(tgt.heading) * (tgt.y - ego_y);
    double ff = std::atan(wheelbase_ * tgt.kappa);
    double fb = p_.lat_gain_dh * dh + p_.lat_gain_elat * e_lat;
    if (rev) fb = -fb;
    double steer = std::max(-p_.max_steer, std::min(p_.max_steer, ff + fb));

    ManeuverResult r{ steer, target_speed, gear_, gear_pending };
    return r;
}

} // namespace maneuver
