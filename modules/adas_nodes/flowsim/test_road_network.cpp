// Unit test for FlowRoadNetwork (Phase 1.2 验收).
// 验证：加载、道路查询、frenet↔world 转换、限速、车道宽度。
//
// 用法：./test_road_network <scene.xodr>
// 期望场景：highway_exit.xodr（含 2 段道路，road0 直道 200m，road1 弯道 120m）

#include "flowsim/road_network.h"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>

using flowsim::FlowRoadNetwork;
using flowsim::RoadInfo;
using flowsim::WorldPos;
using flowsim::FrenetPos;

static int failures = 0;

#define CHECK(cond, msg) do { \
    if (!(cond)) { \
        std::fprintf(stderr, "FAIL: %s (line %d)\n", msg, __LINE__); \
        ++failures; \
    } else { \
        std::printf("ok: %s\n", msg); \
    } \
} while (0)

static bool approx(double a, double b, double eps = 0.01) {
    return std::fabs(a - b) < eps;
}

int main(int argc, char** argv) {
    const char* xodr = argc > 1 ? argv[1] : "/tmp/test_highway.xodr";
    std::printf("=== FlowRoadNetwork test on %s ===\n", xodr);

    FlowRoadNetwork net;
    CHECK(net.load(xodr), "load xodr");
    CHECK(net.loaded(), "loaded() == true");
    CHECK(net.road_count() == 2, "road_count == 2 (urban + curve)");

    // 道路 0：直道 200m，2 个可行驶车道
    RoadInfo r0;
    CHECK(net.road_info(0, r0), "road_info[0]");
    CHECK(r0.length == 200.0, "road[0] length == 200");
    CHECK(r0.drivable_lanes == 2, "road[0] drivable_lanes == 2");

    // 道路 1：弯道 120m
    RoadInfo r1;
    CHECK(net.road_info(1, r1), "road_info[1]");
    CHECK(r1.length == 120.0, "road[1] length == 120");

    // frenet_to_world: road 0, lane -1, s=50, offset=0
    // 期望：x=50（沿 +x 走 50m），y=-1.75（右车道中心），h=0（直道）
    WorldPos w;
    CHECK(net.frenet_to_world(0, -1, 50.0, 0.0, w), "frenet_to_world road0/lane-1/s50");
    std::printf("  → x=%.3f y=%.3f h=%.3f\n", w.x, w.y, w.h);
    CHECK(approx(w.x, 50.0), "x ≈ 50");
    CHECK(approx(w.y, -1.75), "y ≈ -1.75 (right lane center)");
    CHECK(approx(w.h, 0.0), "h ≈ 0 (straight road)");

    // lane -2: 右数第二车道，中心 t=-5.25
    WorldPos w2;
    CHECK(net.frenet_to_world(0, -2, 10.0, 0.0, w2), "frenet_to_world lane-2/s10");
    std::printf("  → x=%.3f y=%.3f\n", w2.x, w2.y);
    CHECK(approx(w2.x, 10.0), "lane-2 x ≈ 10");
    CHECK(approx(w2.y, -5.25), "y ≈ -5.25 (lane -2 center)");

    // Roundtrip: frenet → world → frenet
    FrenetPos f;
    CHECK(net.world_to_frenet(w.x, w.y, f), "world_to_frenet roundtrip");
    std::printf("  → road=%d lane=%d s=%.3f offset=%.3f\n", f.road_id, f.lane_id, f.s, f.offset);
    CHECK(f.road_id == 0, "roundtrip road == 0");
    CHECK(f.lane_id == -1, "roundtrip lane == -1");
    CHECK(approx(f.s, 50.0, 0.1), "roundtrip s ≈ 50");
    CHECK(approx(f.offset, 0.0, 0.1), "roundtrip offset ≈ 0");

    // lane_width: lane -1 宽 3.5m
    double lw = net.lane_width(0, -1, 10.0);
    std::printf("  lane_width(0,-1,10) = %.3f\n", lw);
    CHECK(approx(lw, 3.5), "lane -1 width ≈ 3.5");

    // speed_limit: 应该返回 xodr 里写的 13.89 m/s (50 km/h)
    double sl = net.speed_limit(0, -1, 10.0);
    std::printf("  speed_limit(0,-1,10) = %.3f\n", sl);
    CHECK(approx(sl, 13.89, 0.1), "speed_limit ≈ 13.89");

    // 弯道段位置：road 1, s=60（弯道中点），h 应该非零（弯道有曲率）
    WorldPos wc;
    CHECK(net.frenet_to_world(1, -1, 60.0, 0.0, wc), "frenet_to_world on curve");
    std::printf("  curve → x=%.3f y=%.3f h=%.4f\n", wc.x, wc.y, wc.h);

    // 越界检查：不存在的 road_id
    WorldPos wbad;
    CHECK(!net.frenet_to_world(999, -1, 10.0, 0.0, wbad), "non-existent road returns false");

    // move semantics
    FlowRoadNetwork net2 = std::move(net);
    CHECK(net2.loaded(), "moved-to net is loaded");
    CHECK(!net.loaded(), "moved-from net is not loaded");
    RoadInfo r;
    CHECK(net2.road_info(0, r), "moved-to net still queries");

    std::printf("=== %d failures ===\n", failures);
    return failures == 0 ? 0 : 1;
}
