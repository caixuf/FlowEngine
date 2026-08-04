// test_maneuver_tracker.cpp — 通用机动跟踪器单元测试
//
// 镜像 Python tools/maneuver_tracker.py 的 run()/run_parking() 不变量：
//   1. 掉头（三把方向）：gear 序列 D→R→D、翻转 ≤10、heading→π、全程 y∈[-7,7]
//   2. 倒车入库：gear D→R、倒车执行
//   微测：
//   3. advanceS 在车头偏离 >heading_gate 时停在 D/R 边界，对齐后跨过
//   4. target_speed = 6m 同挡段 min|v|，floor 1.5，带符号
//   5. s>=end → target 0 / gear DRIVE
//
// 纯算法单测：只 include maneuver_tracker.h，无 transport/param 依赖。
// 注意：tracker 不包含纵向 PID（留在 control_node），故闭环测试用简单的
// P 控制器驱动速度（聚焦验证横向/挡位/弧长推进），不验证纵向。
#include <cassert>
#include <cmath>
#include <cstdio>
#include <vector>

#include "maneuver_tracker.h"

using maneuver::ManeuverTracker;
using maneuver::ManeuverTrackerParams;
using maneuver::TrackPoint;
using maneuver::ManeuverResult;

static constexpr double WHEELBASE = 2.7;
static constexpr double UTURN_SPEED = 3.5;
static constexpr double REVERSE_SPEED = -3.0;
static constexpr double DT = 0.05;

static double norm_angle(double a) {
    while (a >  M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

// ── 下采样到 64 点（镜像 planning downsample_uturn）──
// Trajectory::points 上限 64；真实掉头轨迹经 downsample_uturn 压缩到 64 点，
// 保留 v 符号翻转边界点（D/R 换挡点）以免丢失挡段。测试必须复刻这一点，
// 否则 >64 点的轨迹会被 ManeuverTracker 截断（kMaxPts=64）→ 倒车段被砍 →
// s>=end 提前触发 → 车半途停住（2026-08-04 实测）。
static std::vector<TrajectoryPoint> downsample64(const std::vector<TrajectoryPoint>& src) {
    const int cap = 64;
    if ((int)src.size() <= cap) return src;
    std::vector<bool> keep(src.size(), false);
    keep[0] = true; keep[src.size() - 1] = true;
    int forced = 2;
    for (size_t i = 1; i < src.size(); i++) {
        bool a = src[i - 1].v < -0.05f, b = src[i].v < -0.05f;
        if (a != b) {
            if (!keep[i - 1]) { keep[i - 1] = true; forced++; }
            if (!keep[i])     { keep[i] = true;     forced++; }
        }
    }
    int remain = cap - forced;
    double stride = (double)src.size() / (double)(remain > 0 ? remain + 1 : 1);
    for (int k = 1; k <= remain; k++) {
        int idx = (int)(k * stride);
        if (idx < 1) idx = 1;
        if (idx >= (int)src.size()) idx = (int)src.size() - 1;
        while (idx < (int)src.size() && keep[idx]) idx++;
        if (idx < (int)src.size()) keep[idx] = true;
    }
    std::vector<TrajectoryPoint> out;
    for (size_t i = 0; i < src.size() && (int)out.size() < cap; i++)
        if (keep[i]) out.push_back(src[i]);
    return out;
}

// ── 车辆积分：复刻 flowsim/physics.cpp step_bicycle（车辆中心参考点，支持倒车）──
// 注意：v 由外部纵向模型驱动（本测试用简单 P 控制器），此函数只积分运动学。
static void step_bicycle(double& x, double& y, double& h, double v,
                         double steer, double dt) {
    double yaw = v / WHEELBASE * std::tan(steer);
    double half_wb = WHEELBASE * 0.5;
    h += yaw * dt;
    x += (v * std::cos(h) - half_wb * std::sin(h) * yaw) * dt;
    y += (v * std::sin(h) + half_wb * std::cos(h) * yaw) * dt;
    h = norm_angle(h);
}

// 简单纵向 P 控制器：把 v 拉到 target（限加速率，模拟物理惯性）
static void drive_speed(double& v, double target, double dt) {
    double dv = target - v;
    double max_dv = 3.0 * dt;   // ~3 m/s² 加减速上限
    if (dv >  max_dv) dv =  max_dv;
    if (dv < -max_dv) dv = -max_dv;
    v += dv;
    if (v > 22.0) v = 22.0;
    if (v < -4.0) v = -4.0;
}

// ── 轨迹生成器：复刻 Python gen_uturn_traj（自适应相位，几何条件退出）──
static std::vector<TrajectoryPoint> gen_uturn_traj() {
    std::vector<TrajectoryPoint> pts;
    double x = 2900.0, y = -1.75, h = 0.0;
    double t = 0.0;
    auto push = [&](double steer, double v, double dur) {
        int n = (int)(dur / DT);
        for (int _ = 0; _ < n; _++) {
            double yaw = v / WHEELBASE * std::tan(steer);
            double half_wb = WHEELBASE * 0.5;
            h += yaw * DT;
            x += (v * std::cos(h) - half_wb * std::sin(h) * yaw) * DT;
            y += (v * std::sin(h) + half_wb * std::cos(h) * yaw) * DT;
            TrajectoryPoint p{};
            p.x = (float)x; p.y = (float)y; p.heading = (float)h;
            p.kappa = (float)(std::tan(steer) / WHEELBASE); p.v = (float)v;
            pts.push_back(p);
            t += DT;
        }
    };
    auto push_adaptive = [&](double steer, double v, double target_h,
                             double phase_max_t, double tolerance_deg) {
        double p_t = 0.0;
        while (p_t < phase_max_t &&
               std::fabs(norm_angle(target_h - h)) > tolerance_deg * M_PI / 180.0) {
            double yaw = v / WHEELBASE * std::tan(steer);
            double half_wb = WHEELBASE * 0.5;
            h += yaw * DT;
            x += (v * std::cos(h) - half_wb * std::sin(h) * yaw) * DT;
            y += (v * std::sin(h) + half_wb * std::cos(h) * yaw) * DT;
            TrajectoryPoint p{};
            p.x = (float)x; p.y = (float)y; p.heading = (float)h;
            p.kappa = (float)(std::tan(steer) / WHEELBASE); p.v = (float)v;
            pts.push_back(p);
            p_t += DT;
        }
    };
    // Phase 2: 前进满舵弧，heading 从 start 转到 105°（镜像 Python gen_uturn_traj）
    {
        double start_h = h;
        double p2_t = 0.0;
        while (p2_t < 3.0 && std::fabs(norm_angle(h - start_h)) < 105.0 * M_PI / 180.0) {
            double yaw = UTURN_SPEED / WHEELBASE * std::tan(0.60);
            double half_wb = WHEELBASE * 0.5;
            h += yaw * DT;
            x += (UTURN_SPEED * std::cos(h) - half_wb * std::sin(h) * yaw) * DT;
            y += (UTURN_SPEED * std::sin(h) + half_wb * std::cos(h) * yaw) * DT;
            TrajectoryPoint p{}; p.x = (float)x; p.y = (float)y; p.heading = (float)h;
            p.kappa = (float)(std::tan(0.60) / WHEELBASE); p.v = (float)UTURN_SPEED;
            pts.push_back(p);
            p2_t += DT;
        }
    }
    { TrajectoryPoint b{}; b.x = (float)x; b.y = (float)y; b.heading = (float)h; b.v = 0; pts.push_back(b); }
    // Phase 3: 倒车右打，heading →π
    push_adaptive(-0.60, REVERSE_SPEED, M_PI, 3.5, 5.0);
    { TrajectoryPoint b{}; b.x = (float)x; b.y = (float)y; b.heading = (float)h; b.v = 0; pts.push_back(b); }
    // Phase 4: 前进对齐 heading→π（修正残差）
    push_adaptive(0.30, UTURN_SPEED, M_PI, 2.0, 1.0);
    // Phase 5: 巡航填充
    push(0.0, 20.0, 2.0);
    return pts;
}

// ── 倒车入库轨迹（spec gen_parking_traj）──
static std::vector<TrajectoryPoint> gen_parking_traj() {
    std::vector<TrajectoryPoint> pts;
    double x = 0.0, y = 1.75, h = 0.0;
    auto push = [&](double steer, double v, double dur) {
        int n = (int)(dur / DT);
        for (int _ = 0; _ < n; _++) {
            double yaw = v / WHEELBASE * std::tan(steer);
            double half_wb = WHEELBASE * 0.5;
            h += yaw * DT;
            x += (v * std::cos(h) - half_wb * std::sin(h) * yaw) * DT;
            y += (v * std::sin(h) + half_wb * std::cos(h) * yaw) * DT;
            TrajectoryPoint p{};
            p.x = (float)x; p.y = (float)y; p.heading = (float)h;
            p.kappa = (float)(std::tan(steer) / WHEELBASE); p.v = (float)v;
            pts.push_back(p);
        }
    };
    push(0.0, 5.0, 1.5);
    { TrajectoryPoint b{}; b.x = (float)x; b.y = (float)y; b.heading = (float)h; b.v = 0; pts.push_back(b); }
    push(-0.60, REVERSE_SPEED, 0.7);
    push(0.0, REVERSE_SPEED, 1.2);
    { TrajectoryPoint b{}; b.x = (float)x; b.y = (float)y; b.heading = (float)h; b.v = 0; pts.push_back(b); }
    return pts;
}

// ── 侧方停车（平行泊车）轨迹：第三个操作，验证同一 tracker 通用性 ──
// 库位在右侧：倒车右打满舵摆尾入位 → 回正直倒 → 停正。含两段方向相反的
// 倒车弧（S 形），比倒库更考验换挡/横向反号的鲁棒性。
static std::vector<TrajectoryPoint> gen_parallel_parking_traj() {
    std::vector<TrajectoryPoint> pts;
    double x = 0.0, y = 1.75, h = 0.0;
    auto push = [&](double steer, double v, double dur) {
        int n = (int)(dur / DT);
        for (int _ = 0; _ < n; _++) {
            double yaw = v / WHEELBASE * std::tan(steer);
            double half_wb = WHEELBASE * 0.5;
            h += yaw * DT;
            x += (v * std::cos(h) - half_wb * std::sin(h) * yaw) * DT;
            y += (v * std::sin(h) + half_wb * std::cos(h) * yaw) * DT;
            TrajectoryPoint p{};
            p.x = (float)x; p.y = (float)y; p.heading = (float)h;
            p.kappa = (float)(std::tan(steer) / WHEELBASE); p.v = (float)v;
            pts.push_back(p);
        }
    };
    // 前进过库位
    push(0.0, 5.0, 1.2);
    { TrajectoryPoint b{}; b.x = (float)x; b.y = (float)y; b.heading = (float)h; b.v = 0; pts.push_back(b); }
    // 倒车右打摆尾入位
    push(0.50, REVERSE_SPEED, 0.8);
    // 倒车回正/左打修直
    push(-0.30, REVERSE_SPEED, 0.7);
    // 停正
    { TrajectoryPoint b{}; b.x = (float)x; b.y = (float)y; b.heading = (float)h; b.v = 0; pts.push_back(b); }
    return pts;
}

static int failures = 0;
#define CHECK(cond, msg) do { if (!(cond)) { std::printf("FAIL: %s\n", msg); failures++; } } while (0)

// ── 测试 1: 掉头闭环 ──
static void test_uturn_closed_loop() {
    auto traj = downsample64(gen_uturn_traj());   // 镜像 planning: 压缩到 64 点
    ManeuverTracker mt;
    mt.init(traj.data(), (int)traj.size(), WHEELBASE);
    double x = 2900.0, y = -1.75, h = 0.0, v = 0.0;
    double y_min = 1e9, y_max = -1e9;
    std::vector<int> gears;
    for (int t = 0; t < 400; t++) {
        ManeuverResult r = mt.tick(x, y, h, v, DT);
        drive_speed(v, r.target_speed, DT);
        step_bicycle(x, y, h, v, r.steer, DT);
        y_min = std::min(y_min, y); y_max = std::max(y_max, y);
        gears.push_back(r.gear);
    }
    int flips = 0;
    for (size_t i = 1; i < gears.size(); i++) if (gears[i] != gears[i-1]) flips++;
    bool saw_r = false, saw_d_after_r = false;
    for (size_t i = 0; i < gears.size(); i++) {
        if (gears[i] == -1) saw_r = true;
        if (i > 0 && gears[i-1] == -1 && gears[i] == 1) saw_d_after_r = true;
    }
    std::printf("[uturn] final x=%.1f y=%.2f h=%.2f v=%.1f y[%.1f,%.1f] flips=%d\n",
                x, y, h, v, y_min, y_max, flips);
    CHECK(flips >= 2, "uturn: gear flips < 2 (need D->R->D)");
    CHECK(flips <= 10, "uturn: gear oscillation (flips > 10)");
    CHECK(saw_r, "uturn: never entered REVERSE");
    CHECK(saw_d_after_r, "uturn: never returned to DRIVE after reverse");
    CHECK(std::fabs(norm_angle(h - M_PI)) < 0.3, "uturn: heading not ~pi");
    CHECK(y_min >= -7.0 && y_max <= 7.0, "uturn: off-road");
}

// ── 测试 2: 倒车入库闭环 ──
static void test_parking_closed_loop() {
    auto traj = downsample64(gen_parking_traj());
    ManeuverTracker mt;
    mt.init(traj.data(), (int)traj.size(), WHEELBASE);
    double x = 0.0, y = 1.75, h = 0.0, v = 0.0;
    std::vector<int> gears;
    for (int t = 0; t < 200; t++) {
        ManeuverResult r = mt.tick(x, y, h, v, DT);
        drive_speed(v, r.target_speed, DT);
        step_bicycle(x, y, h, v, r.steer, DT);
        gears.push_back(r.gear);
    }
    bool saw_r = false;
    for (int g : gears) if (g == -1) saw_r = true;
    int flips = 0;
    for (size_t i = 1; i < gears.size(); i++) if (gears[i] != gears[i-1]) flips++;
    std::printf("[parking] final x=%.1f y=%.2f h=%.2f v=%.1f flips=%d\n", x, y, h, v, flips);
    CHECK(saw_r, "parking: never entered REVERSE");
    CHECK(flips >= 1, "parking: no gear change");
}

// ── 测试 2b: 侧方停车闭环（第三个操作，验证同一 tracker 通用性）──
static void test_parallel_parking_closed_loop() {
    auto traj = downsample64(gen_parallel_parking_traj());
    ManeuverTracker mt;
    mt.init(traj.data(), (int)traj.size(), WHEELBASE);
    double x = 0.0, y = 1.75, h = 0.0, v = 0.0;
    std::vector<int> gears;
    for (int t = 0; t < 200; t++) {
        ManeuverResult r = mt.tick(x, y, h, v, DT);
        drive_speed(v, r.target_speed, DT);
        step_bicycle(x, y, h, v, r.steer, DT);
        gears.push_back(r.gear);
    }
    bool saw_r = false;
    for (int g : gears) if (g == -1) saw_r = true;
    int flips = 0;
    for (size_t i = 1; i < gears.size(); i++) if (gears[i] != gears[i-1]) flips++;
    std::printf("[parallel] final x=%.1f y=%.2f h=%.2f v=%.1f flips=%d\n", x, y, h, v, flips);
    CHECK(saw_r, "parallel: never entered REVERSE");
    CHECK(flips >= 1, "parallel: no gear change");
    CHECK(std::fabs(norm_angle(h)) < 0.5, "parallel: heading drifted (should stay ~0)");
}

// ── 测试 3: advanceS 段边界 heading gate ──
// 连续轨迹：D 段前进（heading 从 0 缓慢转），末尾 v0 刹停，R 段倒车（heading 继续）。
// 车头偏离下一段起点 >0.4 rad 时，s 应停在边界；车头转齐后 s 跨过。
// 用连续几何（D 末 heading = R 起 heading）避免 diverge guard 把非物理跳变当发散。
static void test_segment_boundary_gate() {
    std::vector<TrajectoryPoint> traj;
    // D 段：直线前进，heading 0，长 5m（s=0..5）
    for (int i = 0; i <= 25; i++) {
        TrajectoryPoint p{}; p.x = (float)(i * 0.2); p.y = 0; p.heading = 0; p.v = 3.5;
        traj.push_back(p);
    }
    { TrajectoryPoint b{}; b.x = 5.0f; b.y = 0; b.heading = 0; b.v = 0; traj.push_back(b); }
    // R 段：倒车，heading 恒定 0.5（D 末 0 → R 起 0.5，有 0.5 rad 台阶）
    for (int i = 1; i <= 25; i++) {
        TrajectoryPoint p{}; p.x = (float)(5.0 + i * 0.2); p.y = 0;
        p.heading = 0.5f; p.v = -3.0f;
        traj.push_back(p);
    }
    ManeuverTracker mt;
    mt.init(traj.data(), (int)traj.size(), WHEELBASE);
    // 车头 0（对齐 D 段）、速度 3.0：s 推进到边界 5.0。
    // 边界后是 R 段，起点 heading 0.5，车头 0 偏离 0.5>0.4 → s 停在边界。
    for (int t = 0; t < 200; t++) {
        mt.tick(4.0, 0, 0, 3.0, DT);
        if (mt.arcLengthS() >= 4.99) break;
    }
    double s_boundary = mt.arcLengthS();
    std::printf("[boundary] s while misaligned = %.2f (expect 5.0)\n", s_boundary);
    CHECK(std::fabs(s_boundary - 5.0) < 0.02, "boundary: s crossed D/R while heading misaligned");
    // 车头转到 0.5（对齐 R 段起点），s 应跨过边界
    for (int t = 0; t < 100; t++) {
        mt.tick(5.0, 0, 0.5, 3.0, DT);
        if (mt.arcLengthS() > 5.2) break;
    }
    std::printf("[boundary] s after heading aligned = %.2f (expect >5)\n", mt.arcLengthS());
    CHECK(mt.arcLengthS() > 5.1, "boundary: s stuck at boundary after heading aligned");
}

// ── 测试 4: target_speed 6m min + floor + sign ──
// 前进段：直线，heading 0。前 4m 点 v=3.5，之后 v=1.5（6m 扫描窗内取到 min）。
// 推进 s 越过 R 段边界后 target 应为负。
static void test_target_speed() {
    std::vector<TrajectoryPoint> traj;
    for (int i = 0; i < 30; i++) {
        TrajectoryPoint p{}; p.x = (float)(i * 0.5); p.y = 0; p.heading = 0;
        p.v = (i < 8) ? 3.5f : 1.5f;    // 前 4m 是 3.5，之后 1.5
        traj.push_back(p);
    }
    { TrajectoryPoint b{}; b.x = 15.0f; b.y = 0; b.heading = 0; b.v = 0; traj.push_back(b); }
    for (int i = 1; i <= 20; i++) {
        TrajectoryPoint p{}; p.x = (float)(15.0 + i * 0.5); p.y = 0;
        p.heading = 0; p.v = -3.0f;   // R 段（连续 heading 0，纯直线倒车）
        traj.push_back(p);
    }
    ManeuverTracker mt;
    mt.init(traj.data(), (int)traj.size(), WHEELBASE);
    // exec 在开头，前视 6m 内 min|v| = 1.5（第 8 点 4m 处）
    ManeuverResult r = mt.tick(0, 0, 0, 0, DT);
    std::printf("[target] forward target_speed=%.2f (expect 1.5)\n", r.target_speed);
    CHECK(std::fabs(r.target_speed - 1.5) < 0.05, "target: min|v| over 6m not 1.5");
    // 推进 s 进入 R 段但不到终点（终点 s=25，R 段 s=15..25；推进到 ~16 停）
    for (int t = 0; t < 130; t++) {
        ManeuverResult rt = mt.tick(5.0, 0, 0, 3.0, DT);   // 每 tick 推进 0.15m
        if (mt.arcLengthS() >= 16.0) break;
    }
    double s_rev = mt.arcLengthS();
    // 首 tick 进入 R 段时 gear_pending 触发（换挡前刹停）→ target 0，这是正确行为。
    // 用 speed=0 让 gear 落定到 REVERSE，再用 -3.0 运动 → target 应为负。
    ManeuverResult r_settle = mt.tick(16.0, 0, 0, 0.0, DT);   // gear 落定
    ManeuverResult r2 = mt.tick(16.0, 0, 0, -3.0, DT);
    std::printf("[target] s=%.1f settle_gear=%d reverse target_speed=%.2f (expect <0)\n",
                s_rev, r_settle.gear, r2.target_speed);
    CHECK(s_rev > 15.0 && s_rev < 25.0, "target: s did not enter reverse segment");
    CHECK(r2.gear == -1, "target: gear not REVERSE in reverse segment");
    CHECK(r2.target_speed < 0, "target: reverse segment target not negative");
}

// ── 测试 5: s>=end → target 0 / gear DRIVE ──
// 用一个短直线轨迹，推进 s 到终点后验证 done()/target 0/gear DRIVE。
static void test_end_stop() {
    std::vector<TrajectoryPoint> traj;
    for (int i = 0; i <= 20; i++) {
        TrajectoryPoint p{}; p.x = (float)(i * 0.5); p.y = 0; p.heading = 0; p.v = 3.5;
        traj.push_back(p);
    }
    { TrajectoryPoint b{}; b.x = 10.0f; b.y = 0; b.heading = 0; b.v = 0; traj.push_back(b); }
    ManeuverTracker mt;
    mt.init(traj.data(), (int)traj.size(), WHEELBASE);
    for (int t = 0; t < 200; t++) {
        ManeuverResult r = mt.tick(9.0, 0, 0, 3.0, DT);
        if (mt.done()) break;
    }
    CHECK(mt.done(), "end: tracker not done at end");
    ManeuverResult r = mt.tick(9.0, 0, 0, 3.0, DT);
    std::printf("[end] done=%d target_speed=%.2f gear=%d\n", (int)mt.done(), r.target_speed, r.gear);
    CHECK(r.target_speed == 0.0, "end: target_speed not 0 at end");
    CHECK(r.gear == 1, "end: gear not DRIVE at end");
}

int main() {
    test_uturn_closed_loop();
    test_parking_closed_loop();
    test_parallel_parking_closed_loop();
    test_segment_boundary_gate();
    test_target_speed();
    test_end_stop();
    if (failures == 0) {
        std::printf("ALL PASS\n");
        return 0;
    }
    std::printf("%d FAILURES\n", failures);
    return 1;
}
