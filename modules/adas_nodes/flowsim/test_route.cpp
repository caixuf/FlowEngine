// Unit test for Route + NPC 车道跟随（Frenet 推进修复的验收）。
// 验证：
//   1. Route.build 从路网链出有序主 route（段连续、总长>0）
//   2. locate / to_route_s 往返一致；index_of 正确
//   3. NPC 用 step_npc_vehicle 沿 route 行驶：offset 保持（贴车道）、route_s 前进、
//      世界坐标经 world_to_frenet 反投影仍落在同一车道（过弯不飞出）
//   4. 到 route 末端触发回收（route_s 绕回）
//
// 用法：./test_route <scene.xodr>
//   默认 /tmp/flowsim_city_to_highway_full.xodr（demo 跑一次即生成；
//   或 python3 tools/json_to_xodr.py scenarios/city_to_highway_full.json -o <path>）。

#include "flowsim/route.h"
#include "flowsim/road_network.h"
#include "flowsim/npc_ai.h"
#include "flowsim/entity.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>

using namespace flowsim;

static int failures = 0;
#define CHECK(cond, msg) do { \
    if (!(cond)) { std::fprintf(stderr, "FAIL: %s (line %d)\n", msg, __LINE__); ++failures; } \
    else         { std::printf("ok: %s\n", msg); } \
} while (0)

static bool approx(double a, double b, double eps) { return std::fabs(a - b) < eps; }

int main(int argc, char** argv) {
    const char* xodr = argc > 1 ? argv[1] : "/tmp/flowsim_city_to_highway_full.xodr";
    std::printf("=== Route + NPC lane-follow test on %s ===\n", xodr);

    FlowRoadNetwork roads;
    if (!roads.load(xodr)) {
        std::fprintf(stderr, "SKIP: 无法加载 %s（先跑一次 demo 生成，或传入 xodr 路径）\n", xodr);
        return 77;  // 77 = 约定的 skip 码（缺夹具时不判失败）
    }
    CHECK(roads.road_count() > 0, "road_count > 0");

    // ── 1. Route 构建 ──
    Route route;
    CHECK(route.build(roads), "route.build 成功");
    CHECK(route.count() >= 2, "route 至少 2 段（多段主路）");
    CHECK(route.total_length() > 0.0, "route 总长 > 0");
    std::printf("  route: %d 段, 总长 %.1fm\n", route.count(), route.total_length());
    // 段 s_start 单调递增且首段从 0 起
    CHECK(approx(route.seg(0).s_start, 0.0, 1e-6), "首段 s_start == 0");
    bool mono = true;
    for (int i = 1; i < route.count(); ++i)
        if (route.seg(i).s_start <= route.seg(i - 1).s_start) mono = false;
    CHECK(mono, "各段 s_start 单调递增（route 连续）");

    // ── 2. locate / to_route_s 往返 ──
    {
        int idx1 = route.count() / 2;
        double sl_in = route.seg(idx1).length * 0.5;
        double rs = route.to_route_s(idx1, sl_in);
        int rid, ridx; double sl_out;
        route.locate(rs, rid, sl_out, ridx);
        CHECK(ridx == idx1, "locate 段号往返一致");
        CHECK(approx(sl_out, sl_in, 1e-6), "locate 段内 s 往返一致");
        CHECK(route.index_of(route.seg(idx1).road_id) == idx1, "index_of(road_id) 正确");
    }

    // ── 3. NPC 沿车道行驶：过弯不飞出 ──
    {
        EntityPool pool;
        EntityId id = pool.alloc(EntityType::Car);
        Entity& npc = pool[id];
        npc.length = 4.6; npc.width = 2.0; npc.wheelbase = 2.7;
        npc.mass = 1500; npc.drag_coeff = 0.4;
        // 放在首段起点、右车道内（offset<0 = OpenDRIVE 右侧；-1.0 在各段车道内均安全）
        npc.road_id = route.seg(0).road_id;
        npc.s = 5.0;
        npc.offset = -1.0;
        npc.speed = 10.0; npc.target_vx = 10.0;
        npc_init_route(npc, route, +1);
        CHECK(npc.route_dir == 1, "npc_init_route: route_dir=+1");
        CHECK(npc.route_s > 0.0, "npc_init_route: route_s 已定位");

        const double kInitOffset = npc.offset;
        int    wtf_ok = 0, in_lane = 0, samples = 0;
        double max_off = 0.0;
        NpcAiConfig cfg;
        // 步进 1500 步 ×0.05s = 75s ≈ 750m：穿过 city 直道进入 edge3 匝道弯段
        for (int step = 0; step < 1500; ++step) {
            step_npc_vehicle(npc, pool, 0.05, cfg, &roads, &route, /*ego_route_s=*/0.0);
            ++samples;
            // 反投影：NPC 世界坐标 → frenet。成功 = 仍在路网上（原 bug 会飞到 y=-1308，
            // 那种点 world_to_frenet 直接失败）。offset = 距最近车道中心的横向偏差。
            FrenetPos f;
            if (roads.world_to_frenet(npc.x, npc.y, f)) {
                ++wtf_ok;
                double off = std::fabs(f.offset);
                if (off > max_off) max_off = off;
                if (off < 2.5) ++in_lane;   // 在车道内（含采样/车道宽变化容差）
            }
        }
        std::printf("  NPC 75s 后: route_s=%.1f  world=(%.1f,%.1f)  offset=%.2f\n",
                    npc.route_s, npc.x, npc.y, npc.offset);
        std::printf("  在路网采样 %d/%d，车道内 %d/%d，最大离车道中心 %.2fm\n",
                    wtf_ok, samples, in_lane, samples, max_off);
        CHECK(approx(npc.offset, kInitOffset, 1e-9), "NPC offset 全程恒定（车道保持）");
        CHECK(npc.route_s > 400.0, "NPC 沿 route 前进 >400m（穿过 city 进入匝道）");
        CHECK(wtf_ok >= samples * 0.95, "≥95% 采样 NPC 仍在路网上（不飞出——直击原 bug）");
        CHECK(in_lane >= samples * 0.90, "≥90% 采样在车道内（过弯不飞出）");
    }

    // ── 4. 回收：到 route 末端应绕回 ──
    {
        EntityPool pool;
        EntityId id = pool.alloc(EntityType::Car);
        Entity& npc = pool[id];
        npc.road_id = route.seg(0).road_id;
        npc.s = 0; npc.offset = -1.75;
        npc.speed = 12.0; npc.target_vx = 12.0;
        npc_init_route(npc, route, +1);
        // 强制推到接近末端
        npc.route_s = route.total_length() - 1.0;
        NpcAiConfig cfg;
        bool wrapped = false;
        double prev = npc.route_s;
        for (int step = 0; step < 40; ++step) {
            step_npc_vehicle(npc, pool, 0.05, cfg, &roads, &route, /*ego_route_s=*/0.0);
            if (npc.route_s < prev - 5.0) { wrapped = true; break; }  // 回收=route_s 骤降
            prev = npc.route_s;
        }
        CHECK(wrapped, "NPC 到 route 末端后被回收（route_s 绕回）");
        CHECK(npc.route_s >= 0.0 && npc.route_s <= route.total_length(),
              "回收后 route_s 落在 [0,total]");
    }

    std::printf("\n=== %s：%d 失败 ===\n", failures ? "FAIL" : "PASS", failures);
    return failures ? 1 : 0;
}
