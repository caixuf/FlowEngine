// Unit test for FlowRoadNetwork (Phase 1.2 验收，2026-07 改为场景无关 invariant)。
// 验证：加载、道路查询、frenet↔world 转换、限速、车道宽度。
// 断言不绑定具体场景（旧版硬编码 zhongkai_road_full 的数字，场景已删除导致
// 测试游离在 CMake 外无法运行）——对任意合法 xodr 验证通用不变式：
//   load 成功、路数 > 0、路长 > 0、车道数 ≥ 1、车道宽 ∈ [2.5,4.0]m、
//   限速 ∈ (0,30]m/s、frenet↔world 往返误差 < 0.1m、越界 road 拒绝。
//
// 用法：./test_road_network <scene.xodr>
// 由 CMake fixture 生成 xodr 后自动运行（见 modules/adas_nodes/CMakeLists.txt）。

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
    const char* xodr = argc > 1 ? argv[1] : "/tmp/flowsim_straight_road.xodr";
    std::printf("=== FlowRoadNetwork test on %s ===\n", xodr);

    FlowRoadNetwork net;
    CHECK(net.load(xodr), "load xodr");
    CHECK(net.loaded(), "loaded() == true");

    const int roads = net.road_count();
    std::printf("  road_count = %d\n", roads);
    CHECK(roads >= 1, "road_count >= 1");

    // 逐条道路：长度 > 0、车道数 ≥ 1
    for (int r = 0; r < roads; ++r) {
        RoadInfo ri;
        CHECK(net.road_info(r, ri), "road_info()");
        char msg[96];
        std::snprintf(msg, sizeof msg, "road[%d].length > 0", r);
        CHECK(ri.length > 0.0, msg);
        std::snprintf(msg, sizeof msg, "road[%d].drivable_lanes >= 1", r);
        CHECK(ri.drivable_lanes >= 1, msg);
    }

    // 通用不变式（对齐 CLAUDE.md 静态 digest）：
    // 车道宽 ∈ [2.5, 4.0]m、限速 ∈ (0, 30]m/s、frenet↔world 往返 < 0.1m
    const int road = 0;
    RoadInfo ri0;
    net.road_info(road, ri0);
    const double road_len = ri0.length;
    const int lanes = net.drivable_lane_count(road, 0.0);
    std::printf("  road[0] drivable lanes (both sides) = %d\n", lanes);
    CHECK(lanes >= 1, "road[0] has >= 1 drivable lane");
    // 右行驶方向车道 id 为负；drivable_lane_count 统计两侧总数，
    // 故从 -1 逐条探测，width <= 0 即该侧车道枚举完毕（对向在 +1..）。
    for (int i = 1;; ++i) {
        const int lane_id = -i;
        const double s = std::min(50.0, road_len / 2.0);
        double lw = net.lane_width(road, lane_id, s);
        if (lw <= 0.0) {
            std::printf("  right side lanes: %d\n", i - 1);
            break;
        }
        char msg[96];
        std::snprintf(msg, sizeof msg, "lane %d width %.3f in [2.5,4.0]m", lane_id, lw);
        CHECK(lw >= 2.5 && lw <= 4.0, msg);

        double sl = net.speed_limit(road, lane_id, s);
        std::snprintf(msg, sizeof msg, "lane %d speed_limit %.3f in (0,30]", lane_id, sl);
        CHECK(sl > 0.0 && sl <= 30.0, msg);

        // 车道中心往返：frenet(s, 0) → world → frenet，应回到同路同车道、|Δs|<0.1、|offset|<0.1
        WorldPos w;
        CHECK(net.frenet_to_world(road, lane_id, s, 0.0, w), "frenet_to_world at lane center");
        FrenetPos f;
        CHECK(net.world_to_frenet(w.x, w.y, f), "world_to_frenet roundtrip");
        std::snprintf(msg, sizeof msg, "lane %d roundtrip: road=%d lane=%d s=%.3f off=%.3f",
                      lane_id, f.road_id, f.lane_id, f.s, f.offset);
        std::printf("  %s\n", msg);
        CHECK(f.road_id == road, "roundtrip same road");
        CHECK(f.lane_id == lane_id, "roundtrip same lane");
        CHECK(approx(f.s, s, 0.1), "roundtrip |Δs| < 0.1");
        CHECK(approx(f.offset, 0.0, 0.1), "roundtrip |Δoffset| < 0.1");
    }

    // 车道中心应在道路另一侧相邻车道外（sanity：-1 车道中心 y < 0）
    WorldPos wc;
    if (net.frenet_to_world(road, -1, 10.0, 0.0, wc)) {
        CHECK(wc.y < 0.0, "lane -1 center is on -y side (right-hand driving)");
    }

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
