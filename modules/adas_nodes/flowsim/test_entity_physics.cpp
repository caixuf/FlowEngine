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
using flowsim::AIState;
using flowsim::apply_vehicle_defaults;
using flowsim::step_bicycle;
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
    test_pedestrian();
    std::printf("=== %d failures ===\n", failures);
    return failures == 0 ? 0 : 1;
}
