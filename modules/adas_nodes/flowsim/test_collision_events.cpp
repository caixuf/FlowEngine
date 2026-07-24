// Unit test for collision + scene_events (Phase 1.5 + 1.6 验收).

#include "flowsim/entity.h"
#include "flowsim/physics.h"
#include "flowsim/collision.h"
#include "flowsim/scene_events.h"

#include <cstdio>
#include <cmath>
#include <vector>

using namespace flowsim;

static int failures = 0;
#define CHECK(cond, msg) do { \
    if (!(cond)) { std::fprintf(stderr, "FAIL: %s (line %d)\n", msg, __LINE__); ++failures; } \
    else { std::printf("ok: %s\n", msg); } \
} while (0)

/* ── 碰撞检测 ── */
static void test_collision_separated() {
    std::printf("--- collision: separated ---\n");
    EntityPool pool;
    EntityId a = pool.alloc(EntityType::Car);
    pool[a].x = 0; pool[a].y = 0; pool[a].heading = 0;
    apply_vehicle_defaults(pool[a]);
    EntityId b = pool.alloc(EntityType::Car);
    pool[b].x = 20; pool[b].y = 0; pool[b].heading = 0;
    apply_vehicle_defaults(pool[b]);

    std::vector<CollisionPair> pairs;
    int n = detect_collisions(pool, pairs);
    CHECK(n == 0, "no collision when 20m apart");
}

static void test_collision_overlap() {
    std::printf("--- collision: overlap ---\n");
    EntityPool pool;
    EntityId a = pool.alloc(EntityType::Car);
    pool[a].x = 0; pool[a].y = 0; pool[a].heading = 0;
    apply_vehicle_defaults(pool[a]);
    EntityId b = pool.alloc(EntityType::Car);
    pool[b].x = 3; pool[b].y = 0; pool[b].heading = 0;  // 3m 间距，车长 4.6，会重叠
    apply_vehicle_defaults(pool[b]);

    std::vector<CollisionPair> pairs;
    int n = detect_collisions(pool, pairs);
    CHECK(n == 1, "1 collision pair");
    CHECK(pairs[0].a == a && pairs[0].b == b, "correct pair");
}

static void test_collision_obb_rotated() {
    std::printf("--- collision: OBB rotated ---\n");
    // 两车垂直交叉，AABB 会误判为碰撞，OBB 应判为不碰撞
    EntityPool pool;
    EntityId a = pool.alloc(EntityType::Car);
    pool[a].x = 0; pool[a].y = 0; pool[a].heading = 0;       // 沿 x 轴
    apply_vehicle_defaults(pool[a]);
    EntityId b = pool.alloc(EntityType::Car);
    pool[b].x = 0; pool[b].y = 0; pool[b].heading = M_PI/2;  // 沿 y 轴，中心重合
    apply_vehicle_defaults(pool[b]);

    std::vector<CollisionPair> pairs;
    int n = detect_collisions(pool, pairs);
    // 中心重合必然碰撞
    CHECK(n == 1, "rotated cars at same center collide");

    // 把 b 移开 4m（沿 y），AABB 会判碰（wid 2+2=4），但 OBB 应判不碰
    // 因为 a 沿 x（len 4.6），b 沿 y（len 4.6），b 中心在 (0,4)
    // a 的 y 范围 [-1,1]，b 的 y 范围 [4-2.3, 4+2.3] = [1.7, 6.3]，不重叠
    pool[b].y = 4.0;
    n = detect_collisions(pool, pairs);
    CHECK(n == 0, "rotated cars 4m apart in y don't collide");
}

static void test_collision_response() {
    std::printf("--- collision response ---\n");
    EntityPool pool;
    EntityId a = pool.alloc(EntityType::Car);
    pool[a].speed = 10.0; pool[a].vx = 10.0;
    EntityId b = pool.alloc(EntityType::Car);
    pool[b].speed = 5.0; pool[b].vx = 5.0;

    std::vector<CollisionPair> pairs = {{a, b}};
    apply_collision_response(pool, pairs);
    CHECK(pool[a].speed == 0.0, "car A stopped");
    CHECK(pool[b].speed == 0.0, "car B stopped");
    CHECK(pool[a].brake == 1.0, "car A braking");
}

static void test_collision_excludes_pedestrian() {
    std::printf("--- collision excludes pedestrian ---\n");
    EntityPool pool;
    EntityId car = pool.alloc(EntityType::Car);
    pool[car].x = 0; pool[car].y = 0; pool[car].heading = 0;
    apply_vehicle_defaults(pool[car]);
    EntityId ped = pool.alloc(EntityType::Pedestrian);
    pool[ped].x = 0; pool[ped].y = 0;  // 重合

    std::vector<CollisionPair> pairs;
    int n = detect_collisions(pool, pairs);
    CHECK(n == 0, "pedestrian not in collision pairs");
}

/* ── 红绿灯 ── */
static void test_traffic_light_cycle() {
    std::printf("--- traffic light cycle ---\n");
    EntityPool pool;
    EntityId tl = pool.alloc(EntityType::TrafficLight);
    Entity& light = pool[tl];
    light.throttle = 5.0;  // green 5s
    light.brake    = 2.0;  // yellow 2s
    light.steer    = 8.0;  // red 8s

    // t=0: 应该是绿灯
    tick_traffic_lights(pool, 0.0);
    CHECK(light.phase_state == (int)TLPhase::Green, "t=0 green");

    // t=5: 绿灯结束，黄灯开始
    tick_traffic_lights(pool, 5.0);
    CHECK(light.phase_state == (int)TLPhase::Yellow, "t=5 yellow");

    // t=7: 黄灯结束，红灯开始
    tick_traffic_lights(pool, 7.0);
    CHECK(light.phase_state == (int)TLPhase::Red, "t=7 red");

    // t=15: 一个周期结束（5+2+8=15），回到绿灯
    tick_traffic_lights(pool, 15.0);
    CHECK(light.phase_state == (int)TLPhase::Green, "t=15 green (new cycle)");

    // t=20: 第二周期黄灯（20-15=5，green 边界，5 < green(5) false → 5 < 7 yellow）
    tick_traffic_lights(pool, 20.0);
    CHECK(light.phase_state == (int)TLPhase::Yellow, "t=20 yellow");
}

/* ── ETC 闸门 ── */
static void test_etc_gate() {
    std::printf("--- ETC gate ---\n");
    EntityPool pool;
    EntityId ego_id = pool.alloc(EntityType::Ego);
    Entity& ego = pool[ego_id];
    ego.x = 0;

    EntityId gate_id = pool.alloc(EntityType::ETCGate);
    Entity& gate = pool[gate_id];
    gate.x = 100; gate.y = 0;

    // ego 远距：闸门关
    ego.x = 0;
    tick_etc_gates(pool, ego, 0.05);
    CHECK(gate.phase_state == 0, "far: gate closed");
    CHECK(gate.phase_timer == 0.0, "far: bar down");

    // ego 中距（40m）：抬杆中
    ego.x = 60;
    tick_etc_gates(pool, ego, 0.05);
    CHECK(gate.phase_state == 1, "mid: gate opening");
    CHECK(gate.phase_timer > 0.0, "mid: bar rising");

    // 多 tick 后抬满
    for (int i = 0; i < 100; ++i) tick_etc_gates(pool, ego, 0.05);
    CHECK(gate.phase_timer == 1.0, "mid: bar fully raised");

    // ego 近距（5m）：全开
    ego.x = 95;
    tick_etc_gates(pool, ego, 0.05);
    CHECK(gate.phase_state == 2, "near: gate open");

    // ego 已通过：开始放下
    ego.x = 105;
    tick_etc_gates(pool, ego, 0.05);
    CHECK(gate.phase_state == 0, "passed: gate closing");
}

int main() {
    test_collision_separated();
    test_collision_overlap();
    test_collision_obb_rotated();
    test_collision_response();
    test_collision_excludes_pedestrian();
    test_traffic_light_cycle();
    test_etc_gate();
    std::printf("=== %d failures ===\n", failures);
    return failures == 0 ? 0 : 1;
}
