// Unit test for scene_pub (Phase 2.2 验收).
// 验证 scene/frame JSON 帧格式：道路网络采样、实体序列化、字段完整性。
//
// 用法：./test_scene_pub
// 不需要 esmini/xodr — 用 legacy 道路几何（curve_start_x/length/offset）测试。
// esmini 路径在 Phase 2.3 集成测试里覆盖（需要真实 xodr 文件）。

#include "flowsim/scene_pub.h"
#include "flowsim/entity.h"
#include "flowsim/physics.h"

#include <cjson/cJSON.h>

#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

using flowsim::Entity;
using flowsim::EntityPool;
using flowsim::EntityType;
using flowsim::EntityId;
using flowsim::AIState;
using flowsim::ScenePubConfig;
using flowsim::apply_vehicle_defaults;
using flowsim::build_scene_frame_json;

static int failures = 0;

#define CHECK(cond, msg) do { \
    if (!(cond)) { std::fprintf(stderr, "FAIL: %s (line %d)\n", msg, __LINE__); ++failures; } \
    else { std::printf("ok: %s\n", msg); } \
} while (0)

static bool approx(double a, double b, double eps = 0.01) {
    return std::fabs(a - b) < eps;
}

/* 构造测试用 EntityPool：
 *   index 0: ego (4.6×2.0)
 *   index 1: NPC car (跟随状态)
 *   index 2: pedestrian (横穿中)
 *   index 3: traffic light (红灯，剩余 12.3s)
 *   index 4: ETC gate (closed/progress=0) */
static EntityPool build_test_pool() {
    EntityPool pool;

    EntityId ego_id = pool.alloc(EntityType::Ego);
    Entity& ego = pool[ego_id];
    ego.x = 102.5; ego.y = -1.75; ego.heading = 0.05;
    ego.speed = 8.0; ego.vx = 8.0; ego.vy = 0;
    ego.throttle = 0.3; ego.brake = 0.0; ego.steer = 0.02;
    ego.target_vx = 10.0;
    ego.length = 4.6; ego.width = 2.0;
    apply_vehicle_defaults(ego);

    EntityId car_id = pool.alloc(EntityType::Car);
    Entity& car = pool[car_id];
    car.id = 1;
    car.x = 120.0; car.y = -1.75; car.heading = 0;
    car.speed = 3.0; car.vx = 3.0; car.vy = 0;
    car.length = 4.6; car.width = 2.0;
    car.ai_state = AIState::Follow;
    apply_vehicle_defaults(car);

    EntityId ped_id = pool.alloc(EntityType::Pedestrian);
    Entity& ped = pool[ped_id];
    ped.id = 2;
    ped.x = 140.0; ped.y = 5.0;
    ped.vx = 0; ped.vy = 0.6;
    ped.speed = 0.6;
    ped.ped_parked = 0;

    EntityId tl_id = pool.alloc(EntityType::TrafficLight);
    Entity& tl = pool[tl_id];
    tl.id = 6;
    tl.x = 200.0; tl.y = -1.75;
    tl.phase_state = 2;  /* Red */
    tl.phase_timer = 12.3;

    EntityId gate_id = pool.alloc(EntityType::ETCGate);
    Entity& gate = pool[gate_id];
    gate.id = 12;
    gate.x = 450.0; gate.y = 0;
    gate.ai_state = AIState::Stop;  /* closed */
    gate.phase_timer = 0.0;

    return pool;
}

/* 解析并验证 JSON 帧结构 */
static void validate_json(const char* json_str, const EntityPool& pool) {
    cJSON* root = cJSON_Parse(json_str);
    CHECK(root != nullptr, "JSON parses");

    /* 顶层字段 */
    cJSON* t_us = cJSON_GetObjectItemCaseSensitive(root, "t_us");
    CHECK(t_us && cJSON_IsNumber(t_us), "t_us is number");
    CHECK(t_us && approx(t_us->valuedouble, 12345678.0), "t_us == 12345678");

    cJSON* cycle = cJSON_GetObjectItemCaseSensitive(root, "cycle");
    CHECK(cycle && cJSON_IsNumber(cycle), "cycle is number");
    CHECK(cycle && approx(cycle->valuedouble, 246.0), "cycle == 246");

    /* road_network */
    cJSON* rn = cJSON_GetObjectItemCaseSensitive(root, "road_network");
    CHECK(rn != nullptr, "road_network exists");
    cJSON* edges = cJSON_GetObjectItemCaseSensitive(rn, "edges");
    CHECK(edges && cJSON_IsArray(edges), "edges is array");
    /* legacy 模式：单条 edge */
    CHECK(cJSON_GetArraySize(edges) == 1, "legacy mode: 1 edge");
    cJSON* edge0 = cJSON_GetArrayItem(edges, 0);
    CHECK(edge0 != nullptr, "edge[0] exists");
    cJSON* edge_id = cJSON_GetObjectItemCaseSensitive(edge0, "id");
    CHECK(edge_id && approx(edge_id->valuedouble, 0.0), "edge[0].id == 0");
    cJSON* nodes = cJSON_GetObjectItemCaseSensitive(edge0, "nodes");
    CHECK(nodes && cJSON_IsArray(nodes), "edge[0].nodes is array");
    /* 直道（curve_offset_m==0）只采样 2 个节点 */
    CHECK(cJSON_GetArraySize(nodes) == 2, "legacy straight road: 2 nodes");
    cJSON* node0 = cJSON_GetArrayItem(nodes, 0);
    CHECK(node0 && cJSON_GetArraySize(node0) == 2, "node[0] is [x,y]");
    cJSON* nx = cJSON_GetArrayItem(node0, 0);
    cJSON* ny = cJSON_GetArrayItem(node0, 1);
    CHECK(nx && approx(nx->valuedouble, 0.0), "node[0].x == 0");
    CHECK(ny && approx(ny->valuedouble, 0.0), "node[0].y == 0 (straight road)");

    /* entities */
    cJSON* entities = cJSON_GetObjectItemCaseSensitive(root, "entities");
    CHECK(entities && cJSON_IsArray(entities), "entities is array");
    /* 5 个 active 实体（ego + car + ped + tl + etc_gate） */
    CHECK(cJSON_GetArraySize(entities) == 5, "5 entities in array");

    /* 找 ego（type=="ego"） */
    cJSON* ego_j = nullptr;
    cJSON* car_j = nullptr;
    cJSON* ped_j = nullptr;
    cJSON* tl_j = nullptr;
    cJSON* gate_j = nullptr;
    cJSON* e;
    cJSON_ArrayForEach(e, entities) {
        cJSON* t = cJSON_GetObjectItemCaseSensitive(e, "type");
        if (!t || !cJSON_IsString(t)) continue;
        if (strcmp(t->valuestring, "ego") == 0) ego_j = e;
        else if (strcmp(t->valuestring, "car") == 0) car_j = e;
        else if (strcmp(t->valuestring, "pedestrian") == 0) ped_j = e;
        else if (strcmp(t->valuestring, "tl") == 0) tl_j = e;
        else if (strcmp(t->valuestring, "etc_gate") == 0) gate_j = e;
    }
    CHECK(ego_j != nullptr, "ego entity present");
    CHECK(car_j != nullptr, "car entity present");
    CHECK(ped_j != nullptr, "pedestrian entity present");
    CHECK(tl_j != nullptr, "traffic_light entity present");
    CHECK(gate_j != nullptr, "etc_gate entity present");

    /* ego 字段 */
    cJSON* ego_x = cJSON_GetObjectItemCaseSensitive(ego_j, "x");
    CHECK(ego_x && approx(ego_x->valuedouble, 102.5), "ego.x == 102.5");
    cJSON* ego_spd = cJSON_GetObjectItemCaseSensitive(ego_j, "spd");
    CHECK(ego_spd && approx(ego_spd->valuedouble, 8.0), "ego.spd == 8.0");
    cJSON* ego_thr = cJSON_GetObjectItemCaseSensitive(ego_j, "throttle");
    CHECK(ego_thr && approx(ego_thr->valuedouble, 0.3), "ego.throttle == 0.3");
    cJSON* ego_steer = cJSON_GetObjectItemCaseSensitive(ego_j, "steer");
    CHECK(ego_steer && approx(ego_steer->valuedouble, 0.02), "ego.steer == 0.02");
    cJSON* ego_len = cJSON_GetObjectItemCaseSensitive(ego_j, "len");
    CHECK(ego_len && approx(ego_len->valuedouble, 4.6), "ego.len == 4.6");
    cJSON* ego_tgt = cJSON_GetObjectItemCaseSensitive(ego_j, "tgt");
    CHECK(ego_tgt && approx(ego_tgt->valuedouble, 10.0), "ego.tgt == 10.0");

    /* car 字段（NPC 应带 ai 状态） */
    cJSON* car_ai = cJSON_GetObjectItemCaseSensitive(car_j, "ai");
    CHECK(car_ai && cJSON_IsString(car_ai), "car.ai is string");
    CHECK(car_ai && strcmp(car_ai->valuestring, "follow") == 0, "car.ai == 'follow'");

    /* pedestrian 字段（应带 parked 布尔） */
    cJSON* ped_parked = cJSON_GetObjectItemCaseSensitive(ped_j, "parked");
    CHECK(ped_parked != nullptr, "pedestrian.parked exists");
    CHECK(ped_parked && cJSON_IsBool(ped_parked), "pedestrian.parked is bool");
    CHECK(ped_parked && !cJSON_IsTrue(ped_parked), "pedestrian.parked == false (walking)");

    /* traffic_light 字段 */
    cJSON* tl_state = cJSON_GetObjectItemCaseSensitive(tl_j, "state");
    CHECK(tl_state && cJSON_IsString(tl_state), "tl.state is string");
    CHECK(tl_state && strcmp(tl_state->valuestring, "red") == 0, "tl.state == 'red'");
    cJSON* tl_remain = cJSON_GetObjectItemCaseSensitive(tl_j, "remain_s");
    CHECK(tl_remain && approx(tl_remain->valuedouble, 12.3), "tl.remain_s == 12.3");

    /* etc_gate 字段 */
    cJSON* gate_state = cJSON_GetObjectItemCaseSensitive(gate_j, "state");
    CHECK(gate_state && cJSON_IsString(gate_state), "etc_gate.state is string");
    CHECK(gate_state && strcmp(gate_state->valuestring, "closed") == 0, "etc_gate.state == 'closed'");
    cJSON* gate_prog = cJSON_GetObjectItemCaseSensitive(gate_j, "progress");
    CHECK(gate_prog && approx(gate_prog->valuedouble, 0.0), "etc_gate.progress == 0.0");

    cJSON_Delete(root);
}

/* 验证弯道场景的道路采样（多节点） */
static void test_curve_road_sampling() {
    std::printf("--- 弯道场景道路采样 ---\n");
    EntityPool pool;
    pool.alloc(EntityType::Ego);  /* 只需要 ego，重点验证道路 */

    ScenePubConfig cfg;
    cfg.curve_start_x = 200.0;
    cfg.curve_length_m = 120.0;
    cfg.curve_offset_m = 8.0;
    cfg.lane_width = 3.5;
    cfg.lane_count = 2;
    cfg.roads = nullptr;

    char* json = build_scene_frame_json(pool, cfg, 0, 0);
    CHECK(json != nullptr, "curve scene json built");

    cJSON* root = cJSON_Parse(json);
    CHECK(root != nullptr, "curve json parses");
    cJSON* rn = cJSON_GetObjectItemCaseSensitive(root, "road_network");
    cJSON* edges = cJSON_GetObjectItemCaseSensitive(rn, "edges");
    cJSON* edge0 = cJSON_GetArrayItem(edges, 0);
    cJSON* nodes = cJSON_GetObjectItemCaseSensitive(edge0, "nodes");
    /* 弯道：8 个采样点 */
    CHECK(cJSON_GetArraySize(nodes) == 8, "curve road: 8 sample nodes");
    /* 起点应为 (0, 0) */
    cJSON* node0 = cJSON_GetArrayItem(nodes, 0);
    cJSON* nx = cJSON_GetArrayItem(node0, 0);
    cJSON* ny = cJSON_GetArrayItem(node0, 1);
    CHECK(approx(nx->valuedouble, 0.0), "curve node[0].x == 0");
    CHECK(approx(ny->valuedouble, 0.0), "curve node[0].y == 0");
    /* 终点应为 (320, 8) — curve_start_x + curve_length_m = 320, offset 8 */
    cJSON* node7 = cJSON_GetArrayItem(nodes, 7);
    cJSON* ex = cJSON_GetArrayItem(node7, 0);
    cJSON* ey = cJSON_GetArrayItem(node7, 1);
    CHECK(approx(ex->valuedouble, 320.0), "curve node[7].x == 320");
    CHECK(approx(ey->valuedouble, 8.0), "curve node[7].y == 8 (offset)");

    cJSON_Delete(root);
    free(json);
}

int main() {
    std::printf("=== scene_pub test (Phase 2.2 验收) ===\n");

    EntityPool pool = build_test_pool();
    CHECK(pool.active_count() == 5, "test pool has 5 active entities");

    ScenePubConfig cfg;
    cfg.curve_start_x = 0;
    cfg.curve_length_m = 0;
    cfg.curve_offset_m = 0;
    cfg.lane_width = 3.5;
    cfg.lane_count = 2;
    cfg.roads = nullptr;

    char* json = build_scene_frame_json(pool, cfg, 12345678, 246);
    CHECK(json != nullptr, "build_scene_frame_json returns non-null");

    std::printf("--- JSON 帧结构验证 ---\n");
    validate_json(json, pool);

    /* 打印 JSON 供人工检视 */
    std::printf("\n--- 生成的 JSON 帧（前 800 字符）---\n");
    int len = (int)strlen(json);
    int show = len < 800 ? len : 800;
    std::printf("%.*s%s\n", show, json, len > 800 ? "..." : "");
    free(json);

    test_curve_road_sampling();

    std::printf("\n=== %s (failures=%d) ===\n",
                failures == 0 ? "PASS" : "FAIL", failures);
    return failures == 0 ? 0 : 1;
}
