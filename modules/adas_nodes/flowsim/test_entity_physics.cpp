// Unit test for EntityPool + physics (Phase 1.3 验收).

#include "flowsim/entity.h"
#include "flowsim/physics.h"

#include <cassert>
#include <cmath>
#include <cstdio>

using flowsim::Entity;
using flowsim::EntityPool;
using flowsim::EntityType;
using flowsim::EntityId;
using flowsim::INVALID_ENTITY;
using flowsim::MAX_ENTITIES;
using flowsim::NpcState;
using flowsim::apply_vehicle_defaults;
using flowsim::step_bicycle;
using flowsim::step_bicycle_dynamic;
using flowsim::step_pedestrian;

static int failures = 0;
#define CHECK(cond, msg) do { \
    if (!(cond)) { std::fprintf(stderr, "FAIL: %s (line %d)\n", msg, __LINE__); ++failures; } \
    else { std::printf("ok: %s\n", msg); } \
} while (0)
static bool approx(double a, double b, double eps = 0.05) { return std::fabs(a - b) < eps; }

static void test_pool() {
    std::printf("--- EntityPool ---\n");
    EntityPool pool;
    CHECK(pool.size() == 0, "empty pool size == 0");
    CHECK(pool.active_count() == 0, "empty pool active_count == 0");

    EntityId a = pool.alloc(EntityType::Car);
    CHECK(a == 0, "first alloc == 0");
    CHECK(pool.size() == 1, "size == 1 after alloc");
    CHECK(pool[0].active, "entity 0 active");
    CHECK(pool[0].type == EntityType::Car, "entity 0 type == Car");

    EntityId b = pool.alloc(EntityType::Pedestrian);
    CHECK(b == 1, "second alloc == 1");

    pool.free(a);
    CHECK(!pool[0].active, "entity 0 freed");
    CHECK(pool[1].active, "entity 1 still active");

    // 复用释放的槽位
    EntityId c = pool.alloc(EntityType::Truck);
    CHECK(c == 0, "reuse freed slot 0");
    CHECK(pool[0].type == EntityType::Truck, "slot 0 reused as Truck");

    pool.clear();
    CHECK(pool.size() == 0, "clear resets size");
}

static void test_vehicle_defaults() {
    std::printf("--- vehicle defaults ---\n");
    Entity car;  car.type = EntityType::Car;     apply_vehicle_defaults(car);
    Entity suv;  suv.type = EntityType::SUV;     apply_vehicle_defaults(suv);
    Entity trk;  trk.type = EntityType::Truck;   apply_vehicle_defaults(trk);

    CHECK(approx(car.length, 4.6), "car length 4.6");
    CHECK(approx(car.wheelbase, 2.7), "car wheelbase 2.7");
    CHECK(approx(suv.length, 4.8), "suv length 4.8");
    CHECK(approx(trk.length, 8.0), "truck length 8.0");
    CHECK(approx(trk.mass, 8000.0), "truck mass 8000");
    CHECK(trk.mass > car.mass, "truck heavier than car");

    // 动力学参数（physics_model=dynamic 才用；默认 0 会让动力学积分崩，必须被填上）
    CHECK(car.tire_stiffness_f > 0.0, "car tire_stiffness_f set");
    CHECK(car.tire_stiffness_r > 0.0, "car tire_stiffness_r set");
    CHECK(car.yaw_inertia > 0.0, "car yaw_inertia set");
    CHECK(trk.yaw_inertia > car.yaw_inertia, "truck higher yaw inertia than car");
}

static void test_bicycle_straight() {
    std::printf("--- bicycle straight line ---\n");
    // 油门 0.3，无转向，从静止起步
    Entity e;
    e.type = EntityType::Car;
    apply_vehicle_defaults(e);
    e.x = 0; e.y = 0; e.heading = 0; e.speed = 0;

    double dt = 0.05;
    for (int i = 0; i < 100; ++i) {  // 5 秒
        step_bicycle(e, dt, 0.3, 0.0, 0.0);
    }
    std::printf("  after 5s: x=%.2f y=%.2f speed=%.2f heading=%.4f\n",
                e.x, e.y, e.speed, e.heading);
    CHECK(e.speed > 4.5, "accelerated to >4.5 m/s");
    CHECK(approx(e.y, 0.0, 0.01), "no lateral drift (y≈0)");
    CHECK(approx(e.heading, 0.0, 0.001), "heading unchanged (steer=0)");
    CHECK(e.x > 10.0, "moved forward >10m");
}

static void test_bicycle_braking() {
    std::printf("--- bicycle braking ---\n");
    Entity e;
    e.type = EntityType::Car;
    apply_vehicle_defaults(e);
    e.speed = 20.0;  // 初始 20 m/s
    e.heading = 0;

    // 全刹车 4 秒
    double dt = 0.05;
    for (int i = 0; i < 80; ++i) {
        step_bicycle(e, dt, 0.0, 1.0, 0.0);
    }
    std::printf("  after 4s brake: speed=%.3f\n", e.speed);
    CHECK(e.speed < 1.0, "braked to near stop (<1 m/s)");
}

static void test_bicycle_turn() {
    std::printf("--- bicycle turning ---\n");
    Entity e;
    e.type = EntityType::Car;
    apply_vehicle_defaults(e);
    e.speed = 10.0;
    e.heading = 0;
    e.x = 0; e.y = 0;

    // 恒定右转 0.1 rad，2 秒
    double dt = 0.05;
    for (int i = 0; i < 40; ++i) {
        step_bicycle(e, dt, 0.0, 0.0, 0.1);
    }
    std::printf("  after 2s turn: x=%.2f y=%.2f heading=%.4f\n",
                e.x, e.y, e.heading);
    CHECK(e.heading > 0.05, "turned right (heading > 0)");
    // 右转 → y 应该减小（heading 正 → cos(heading) ≈ 1, sin(heading) > 0）
    // 但自行模型中正 steer 让 heading 增加 → y 增加
    CHECK(e.y > 0.5, "lateral displacement > 0.5m");
}

static void test_dynamic_straight() {
    std::printf("--- dynamic straight line (no drift) ---\n");
    Entity e;
    e.type = EntityType::Car;
    apply_vehicle_defaults(e);
    e.x = 0; e.y = 0; e.heading = 0; e.speed = 15.0;  // 高速直行

    double dt = 0.05;
    for (int i = 0; i < 100; ++i) {  // 5 秒，无转向
        step_bicycle_dynamic(e, dt, 0.0, 0.0, 0.0);
    }
    std::printf("  after 5s: x=%.2f y=%.4f v_y=%.5f yaw_rate=%.5f heading=%.5f\n",
                e.x, e.y, e.v_y_body, e.yaw_rate, e.heading);
    CHECK(approx(e.v_y_body, 0.0, 1e-3), "no lateral velocity (v_y≈0)");
    CHECK(approx(e.yaw_rate, 0.0, 1e-3), "no yaw rate (steer=0)");
    CHECK(approx(e.y, 0.0, 0.05), "no lateral drift (y≈0)");
    CHECK(e.x > 60.0, "moved forward (drag only, ~13.x m/s avg)");
}

static void test_dynamic_turn() {
    std::printf("--- dynamic steady-state turn ---\n");
    Entity e;
    e.type = EntityType::Car;
    apply_vehicle_defaults(e);
    e.x = 0; e.y = 0; e.heading = 0; e.speed = 15.0;

    double dt = 0.05;
    double yaw_prev = 0.0;
    // 恒定转向 0.05 rad，跑 4 秒到稳态（throttle 维持车速抵消阻力）
    for (int i = 0; i < 80; ++i) {
        step_bicycle_dynamic(e, dt, 0.15, 0.0, 0.05);
        if (i == 60) yaw_prev = e.yaw_rate;  // 3.0s 快照
    }
    double yaw_ss = e.yaw_rate;  // 4.0s
    std::printf("  steady: v=%.2f yaw_rate=%.4f (3s=%.4f) heading=%.3f y=%.2f\n",
                e.speed, yaw_ss, yaw_prev, e.heading, e.y);
    // 正 steer → 正 yaw_rate → heading/y 增大（与运动学符号一致）
    CHECK(yaw_ss > 0.05, "positive steer -> positive yaw_rate");
    CHECK(e.heading > 0.1 && e.y > 0.5, "vehicle turned toward +y");
    // 收敛：末秒 yaw_rate 变化很小（未发散、未极限环）
    CHECK(approx(yaw_ss, yaw_prev, 0.02), "yaw_rate converged to steady state");
    // 不足转向：动力学稳态 yaw 低于运动学预测 v/L·tan(steer)
    double yaw_kin = (e.speed / e.wheelbase) * std::tan(0.05);
    CHECK(yaw_ss < yaw_kin, "understeer: dynamic yaw < kinematic prediction");
}

static void test_dynamic_highspeed_bounded() {
    std::printf("--- dynamic high-speed large steer (bounded, no NaN) ---\n");
    Entity e;
    e.type = EntityType::Car;
    apply_vehicle_defaults(e);
    e.x = 0; e.y = 0; e.heading = 0; e.speed = 30.0;

    double dt = 0.05;
    // 120 秒持续大转向：修复前 ≥25s 即发散（v_y_body→5×10⁴、speed→5×10³，
    // 2026-07 PR #72 实测 30 m/s steer=0.2 约 25s、steer=0.05 约 42s 发散）。
    // 拉长到 120s 让回归必现；稳定性护栏把状态钳在物理界内。
    for (int i = 0; i < 2400; ++i) {
        step_bicycle_dynamic(e, dt, 0.2, 0.0, 0.2);
    }
    std::printf("  after 120s: speed=%.2f yaw_rate=%.4f v_y=%.2f x=%.1f y=%.1f\n",
                e.speed, e.yaw_rate, e.v_y_body, e.x, e.y);
    CHECK(!std::isnan(e.x) && !std::isnan(e.y), "position finite (no NaN)");
    CHECK(!std::isnan(e.yaw_rate) && !std::isnan(e.v_y_body), "state finite (no NaN)");
    CHECK(std::fabs(e.yaw_rate) < 2.0, "yaw_rate bounded (<2 rad/s)");
    // 摩擦圆护栏：|v_y| ≤ vx·tan(slip_max)·1.5 → 30 m/s 时 ≈ 5.4 m/s
    CHECK(std::fabs(e.v_y_body) < 60.0, "v_y_body bounded (slip guard)");
    // 阻力限速终端 √(0.2·5000/0.4)=50 m/s，<60 即证明未发散（非发散到数千 m/s）
    CHECK(e.speed < 60.0, "speed bounded (near drag-terminal ~50 m/s)");
}

static void test_dynamic_lowspeed_degrade() {
    std::printf("--- dynamic low-speed degrades to kinematic ---\n");
    // 低速下动力学应退化运动学：两条轨迹逐帧一致
    Entity ed, ek;
    ed.type = ek.type = EntityType::Car;
    apply_vehicle_defaults(ed);
    apply_vehicle_defaults(ek);
    ed.speed = ek.speed = 2.0;  // < LOW_SPEED_MS(5.0)

    double dt = 0.05;
    for (int i = 0; i < 40; ++i) {
        step_bicycle_dynamic(ed, dt, 0.2, 0.0, 0.1);
        step_bicycle(ek, dt, 0.2, 0.0, 0.1);
    }
    std::printf("  dyn:(x=%.3f y=%.3f h=%.4f)  kin:(x=%.3f y=%.3f h=%.4f)\n",
                ed.x, ed.y, ed.heading, ek.x, ek.y, ek.heading);
    CHECK(approx(ed.x, ek.x, 1e-6), "low-speed x matches kinematic");
    CHECK(approx(ed.y, ek.y, 1e-6), "low-speed y matches kinematic");
    CHECK(approx(ed.heading, ek.heading, 1e-6), "low-speed heading matches kinematic");
}

static void test_pedestrian() {
    std::printf("--- pedestrian ---\n");
    Entity p;
    p.type = EntityType::Pedestrian;
    p.x = 0; p.y = 0; p.vx = 0.6; p.vy = 0.0;

    step_pedestrian(p, 1.0);
    CHECK(approx(p.x, 0.6), "ped moved 0.6m in x");
    CHECK(approx(p.speed, 0.6), "ped speed = 0.6");
}

int main() {
    test_pool();
    test_vehicle_defaults();
    test_bicycle_straight();
    test_bicycle_braking();
    test_bicycle_turn();
    test_dynamic_straight();
    test_dynamic_turn();
    test_dynamic_highspeed_bounded();
    test_dynamic_lowspeed_degrade();
    test_pedestrian();
    std::printf("=== %d failures ===\n", failures);
    return failures == 0 ? 0 : 1;
}
