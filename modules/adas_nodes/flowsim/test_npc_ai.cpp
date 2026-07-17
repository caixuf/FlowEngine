// Unit test for NPC AI (Phase 1.4 验收).

#include "flowsim/entity.h"
#include "flowsim/physics.h"
#include "flowsim/npc_ai.h"

#include <cstdio>
#include <cmath>

using namespace flowsim;

static int failures = 0;
#define CHECK(cond, msg) do { \
    if (!(cond)) { std::fprintf(stderr, "FAIL: %s (line %d)\n", msg, __LINE__); ++failures; } \
    else { std::printf("ok: %s\n", msg); } \
} while (0)
static bool approx(double a, double b, double eps = 0.1) { return std::fabs(a - b) < eps; }

static void test_cruise() {
    std::printf("--- cruise (no lead) ---\n");
    EntityPool pool;
    NpcAiConfig cfg;

    EntityId npc_id = pool.alloc(EntityType::Car);
    Entity& npc = pool[npc_id];
    apply_vehicle_defaults(npc);
    npc.x = 0; npc.y = -1.75; npc.heading = 0; npc.speed = 5.0;
    npc.target_vx = 15.0;  // 目标 15 m/s

    // 跑 5 秒，应加速到接近 15
    double dt = 0.05;
    for (int i = 0; i < 100; ++i) {
        step_npc_vehicle(npc, pool, dt, cfg);
    }
    std::printf("  after 5s: speed=%.2f state=%d\n", npc.speed, (int)npc.ai_state);
    CHECK(npc.speed > 10.0, "cruise accelerated to >10 m/s");
    CHECK(npc.ai_state == AIState::Cruise, "state == Cruise");
    CHECK(npc.lead_id == INVALID_ENTITY, "no lead");
}

static void test_follow() {
    std::printf("--- follow (slow lead ahead) ---\n");
    EntityPool pool;
    NpcAiConfig cfg;

    // 前车：慢车 7 m/s，前方 30m
    EntityId lead_id = pool.alloc(EntityType::Car);
    Entity& lead = pool[lead_id];
    apply_vehicle_defaults(lead);
    lead.x = 30; lead.y = -1.75; lead.speed = 7.0;
    lead.target_vx = 7.0; lead.heading = 0;
    lead.vx = 7.0; lead.vy = 0;

    // NPC：15 m/s，会减速跟车
    EntityId npc_id = pool.alloc(EntityType::Car);
    Entity& npc = pool[npc_id];
    apply_vehicle_defaults(npc);
    npc.x = 0; npc.y = -1.75; npc.speed = 15.0;
    npc.target_vx = 15.0; npc.heading = 0;

    // 跑 10 秒
    double dt = 0.05;
    for (int i = 0; i < 200; ++i) {
        // 前车匀速前进
        lead.x += 7.0 * dt;
        step_npc_vehicle(npc, pool, dt, cfg);
    }
    std::printf("  after 10s: npc.speed=%.2f lead.x=%.1f npc.x=%.1f gap=%.1f state=%d\n",
                npc.speed, lead.x, npc.x, lead.x - npc.x, (int)npc.ai_state);
    CHECK(npc.speed < 12.0, "NPC decelerated below 12 m/s");
    CHECK(npc.lead_id == lead_id, "found lead");
    CHECK(npc.ai_state == AIState::Follow, "state == Follow");
    // 应保持安全间距（不追尾）
    CHECK(lead.x - npc.x > 5.0, "kept safe gap > 5m");
}

static void test_stop() {
    std::printf("--- stop (stationary obstacle) ---\n");
    EntityPool pool;
    NpcAiConfig cfg;

    // 前方 20m 一个静止障碍
    EntityId obs_id = pool.alloc(EntityType::Car);
    Entity& obs = pool[obs_id];
    apply_vehicle_defaults(obs);
    obs.x = 20; obs.y = -1.75; obs.speed = 0;

    EntityId npc_id = pool.alloc(EntityType::Car);
    Entity& npc = pool[npc_id];
    apply_vehicle_defaults(npc);
    npc.x = 0; npc.y = -1.75; npc.speed = 12.0;
    npc.target_vx = 12.0;

    double dt = 0.05;
    for (int i = 0; i < 200; ++i) {  // 10s
        step_npc_vehicle(npc, pool, dt, cfg);
    }
    std::printf("  after 10s: npc.speed=%.2f npc.x=%.2f gap=%.2f\n",
                npc.speed, npc.x, obs.x - npc.x);
    CHECK(npc.speed < 2.0, "NPC stopped near obstacle");
    CHECK(obs.x - npc.x > 3.0, "didn't collide (gap > 3m)");
}

static void test_different_lane() {
    std::printf("--- different lane (no interaction) ---\n");
    EntityPool pool;
    NpcAiConfig cfg;

    // 邻车：右车道，前方 20m
    EntityId other_id = pool.alloc(EntityType::Car);
    Entity& other = pool[other_id];
    apply_vehicle_defaults(other);
    other.x = 20; other.y = 1.75; other.speed = 5.0;  // 右车道
    other.target_vx = 5.0; other.heading = 0;
    other.vx = 5.0;

    // NPC：左车道，15 m/s
    EntityId npc_id = pool.alloc(EntityType::Car);
    Entity& npc = pool[npc_id];
    apply_vehicle_defaults(npc);
    npc.x = 0; npc.y = -1.75; npc.speed = 15.0;  // 左车道
    npc.target_vx = 15.0; npc.heading = 0;

    double dt = 0.05;
    for (int i = 0; i < 20; ++i) {  // 1s
        other.x += 5.0 * dt;
        step_npc_vehicle(npc, pool, dt, cfg);
    }
    CHECK(npc.lead_id == INVALID_ENTITY, "no lead (different lane)");
    CHECK(npc.ai_state == AIState::Cruise, "state == Cruise (no follow)");
}

static void test_pedestrian() {
    std::printf("--- pedestrian crossing ---\n");
    EntityPool pool;
    NpcAiConfig cfg;

    EntityId ped_id = pool.alloc(EntityType::Pedestrian);
    Entity& ped = pool[ped_id];
    ped.x = 50; ped.y = 0; ped.vx = 0; ped.vy = 0.6;  // 横穿

    // 跑到边界（0.6 m/s × 13s ≈ 7.8m 边界）
    double dt = 0.05;
    for (int i = 0; i < 300; ++i) {  // 15s
        step_npc_pedestrian(ped, dt, cfg);
        if (ped.ped_parked) break;
    }
    std::printf("  after crossing: y=%.2f parked=%d\n", ped.y, ped.ped_parked);
    CHECK(ped.ped_parked == 1, "pedestrian parked at boundary");
    CHECK(std::fabs(ped.y) >= 7.5, "reached boundary |y| >= 7.5");

    // 等待后反向
    for (int i = 0; i < 100; ++i) {  // 5s
        step_npc_pedestrian(ped, dt, cfg);
    }
    std::printf("  after waiting: y=%.2f vy=%.2f parked=%d\n", ped.y, ped.vy, ped.ped_parked);
    CHECK(ped.ped_parked == 0, "pedestrian resumed after wait");
    CHECK(ped.vy < 0, "pedestrian reversed direction");
}

int main() {
    test_cruise();
    test_follow();
    test_stop();
    test_different_lane();
    test_pedestrian();
    std::printf("=== %d failures ===\n", failures);
    return failures == 0 ? 0 : 1;
}
