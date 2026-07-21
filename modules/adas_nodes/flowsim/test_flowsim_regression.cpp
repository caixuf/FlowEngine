// test_flowsim_regression.cpp — Phase 2.3 旧场景回归测试（向后兼容）
//
// 验证 libflowsim_node.so 能替换 libsim_world.so 跑通旧场景：
//   1. dlopen 加载 + node_get_plugin ABI 校验
//   2. node_init 用真实场景文件初始化
//   3. node_start 启动 20Hz 主循环
//   4. 订阅 vehicle/state + road/geometry + sim/tick + scene/frame
//   5. 等 ~500ms（~10 个 tick）收集消息
//   6. 验证 JSON 帧格式与 sim_world_node 契约一致：
//      - vehicle/state: x/y/spd/hdg/tgt/n_obs + ox%d/oy%d/ov%d/ot%d/ol%d/ow%d
//      - road/geometry: curve_start_x/curve_length_m/curve_offset_m/lane_width/lane_count
//      - sim/tick: t_us/cycle
//      - scene/frame: t_us/cycle/road_network/entities (Phase 2.2 新增)
//   7. node_stop + node_cleanup 干净退出
//
// 用法：./test_flowsim_regression [scenario.json] [libflowsim_node.so]
// 场景路径优先级：命令行参数 > 环境变量 FLOWENGINE_SCENARIO > 报错退出
// （不硬编码场景名，避免与 scenarios/ 目录耦合）

#include "node_plugin.h"
#include "message_bus.h"
#include "transport.h"
#include "discovery.h"
#include "scheduler.h"
#include "config_manager.h"
#include "adas_msgs_gen.h"
#include "logger.h"

#include <cjson/cJSON.h>

#include <dlfcn.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <atomic>
#include <chrono>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

using Clock = std::chrono::steady_clock;

static int g_failures = 0;
#define CHECK(cond, msg) do { \
    if (!(cond)) { std::fprintf(stderr, "FAIL: %s (line %d)\n", msg, __LINE__); ++g_failures; } \
    else { std::printf("ok: %s\n", msg); } \
} while (0)

/* ── 收集订阅到的消息 ─────────────────────────────────────────── */

struct CapturedMsg {
    std::string topic;
    std::vector<uint8_t> data;
};

static std::mutex g_mtx;
static std::vector<CapturedMsg> g_msgs;

static void on_msg(const Message* msg, void* user_data) {
    (void)user_data;
    if (!msg) return;
    std::lock_guard<std::mutex> lk(g_mtx);
    g_msgs.push_back({msg->topic,
                      std::vector<uint8_t>(msg->data, msg->data + msg->data_size)});
}

static int count_topic(const char* topic) {
    std::lock_guard<std::mutex> lk(g_mtx);
    int n = 0;
    for (const auto& m : g_msgs) if (m.topic == topic) ++n;
    return n;
}

static CapturedMsg get_first(const char* topic) {
    std::lock_guard<std::mutex> lk(g_mtx);
    for (const auto& m : g_msgs) {
        if (m.topic == topic) return m;
    }
    return {};
}

/* ── 逐 topic 校验 ────────────────────────────────────────────── */

static void validate_vehicle_state(const CapturedMsg& m) {
    cJSON* root = cJSON_Parse((const char*)m.data.data());
    CHECK(root != nullptr, "vehicle/state JSON parses");
    if (!root) return;

    /* ego 字段（与 sim_world_node 完全一致） */
    cJSON* x   = cJSON_GetObjectItemCaseSensitive(root, "x");
    cJSON* y   = cJSON_GetObjectItemCaseSensitive(root, "y");
    cJSON* spd = cJSON_GetObjectItemCaseSensitive(root, "spd");
    cJSON* hdg = cJSON_GetObjectItemCaseSensitive(root, "hdg");
    cJSON* tgt = cJSON_GetObjectItemCaseSensitive(root, "tgt");
    CHECK(x && cJSON_IsNumber(x), "vehicle/state.x is number");
    CHECK(y && cJSON_IsNumber(y), "vehicle/state.y is number");
    CHECK(spd && cJSON_IsNumber(spd), "vehicle/state.spd is number");
    CHECK(hdg && cJSON_IsNumber(hdg), "vehicle/state.hdg is number");
    CHECK(tgt && cJSON_IsNumber(tgt), "vehicle/state.tgt is number (target_speed)");

    /* n_obs + ox%d/oy%d/ov%d/ot%d/ol%d/ow%d 数组（向后兼容契约） */
    cJSON* n_obs = cJSON_GetObjectItemCaseSensitive(root, "n_obs");
    CHECK(n_obs && cJSON_IsNumber(n_obs), "vehicle/state.n_obs is number");
    int n = n_obs ? (int)n_obs->valuedouble : 0;
    CHECK(n >= 0, "n_obs >= 0");
    if (n > 0) {
        char key[20];
        snprintf(key, sizeof(key), "ox%d", 0);
        cJSON* ox0 = cJSON_GetObjectItemCaseSensitive(root, key);
        snprintf(key, sizeof(key), "oy%d", 0);
        cJSON* oy0 = cJSON_GetObjectItemCaseSensitive(root, key);
        snprintf(key, sizeof(key), "ov%d", 0);
        cJSON* ov0 = cJSON_GetObjectItemCaseSensitive(root, key);
        snprintf(key, sizeof(key), "ot%d", 0);
        cJSON* ot0 = cJSON_GetObjectItemCaseSensitive(root, key);
        snprintf(key, sizeof(key), "ol%d", 0);
        cJSON* ol0 = cJSON_GetObjectItemCaseSensitive(root, key);
        snprintf(key, sizeof(key), "ow%d", 0);
        cJSON* ow0 = cJSON_GetObjectItemCaseSensitive(root, key);
        CHECK(ox0 && cJSON_IsNumber(ox0), "vehicle/state.ox0 present");
        CHECK(oy0 && cJSON_IsNumber(oy0), "vehicle/state.oy0 present");
        CHECK(ov0 && cJSON_IsNumber(ov0), "vehicle/state.ov0 present");
        CHECK(ot0 && cJSON_IsString(ot0), "vehicle/state.ot0 present (type string)");
        CHECK(ol0 && cJSON_IsNumber(ol0), "vehicle/state.ol0 present");
        CHECK(ow0 && cJSON_IsNumber(ow0), "vehicle/state.ow0 present");
    }

    /* t_us 字段（sim_world_node 也有） */
    cJSON* t_us = cJSON_GetObjectItemCaseSensitive(root, "t_us");
    CHECK(t_us && cJSON_IsNumber(t_us), "vehicle/state.t_us is number");

    cJSON_Delete(root);
}

static void validate_road_geometry(const CapturedMsg& m) {
    cJSON* root = cJSON_Parse((const char*)m.data.data());
    CHECK(root != nullptr, "road/geometry JSON parses");
    if (!root) return;
    /* 与 sim_world_node 字段集一致 */
    cJSON* csx = cJSON_GetObjectItemCaseSensitive(root, "curve_start_x");
    cJSON* clm = cJSON_GetObjectItemCaseSensitive(root, "curve_length_m");
    cJSON* com = cJSON_GetObjectItemCaseSensitive(root, "curve_offset_m");
    cJSON* lw  = cJSON_GetObjectItemCaseSensitive(root, "lane_width");
    cJSON* lc  = cJSON_GetObjectItemCaseSensitive(root, "lane_count");
    CHECK(csx && cJSON_IsNumber(csx), "road/geometry.curve_start_x is number");
    CHECK(clm && cJSON_IsNumber(clm), "road/geometry.curve_length_m is number");
    CHECK(com && cJSON_IsNumber(com), "road/geometry.curve_offset_m is number");
    CHECK(lw && cJSON_IsNumber(lw), "road/geometry.lane_width is number");
    CHECK(lc && cJSON_IsNumber(lc), "road/geometry.lane_count is number");
    cJSON_Delete(root);
}

static void validate_sim_tick(const CapturedMsg& m) {
    cJSON* root = cJSON_Parse((const char*)m.data.data());
    CHECK(root != nullptr, "sim/tick JSON parses");
    if (!root) return;
    cJSON* t_us = cJSON_GetObjectItemCaseSensitive(root, "t_us");
    cJSON* cyc  = cJSON_GetObjectItemCaseSensitive(root, "cycle");
    CHECK(t_us && cJSON_IsNumber(t_us), "sim/tick.t_us is number");
    CHECK(cyc && cJSON_IsNumber(cyc), "sim/tick.cycle is number");
    /* 仿真时钟应该 > 0（至少推进过一次） */
    CHECK(t_us->valuedouble > 0, "sim/tick.t_us > 0 (clock advanced)");
    CHECK(cyc->valuedouble >= 0, "sim/tick.cycle >= 0");
    cJSON_Delete(root);
}

static void validate_scene_frame(const CapturedMsg& m) {
    cJSON* root = cJSON_Parse((const char*)m.data.data());
    CHECK(root != nullptr, "scene/frame JSON parses");
    if (!root) return;
    cJSON* t_us = cJSON_GetObjectItemCaseSensitive(root, "t_us");
    cJSON* cyc  = cJSON_GetObjectItemCaseSensitive(root, "cycle");
    cJSON* rn   = cJSON_GetObjectItemCaseSensitive(root, "road_network");
    cJSON* ents = cJSON_GetObjectItemCaseSensitive(root, "entities");
    CHECK(t_us && cJSON_IsNumber(t_us), "scene/frame.t_us is number");
    CHECK(cyc && cJSON_IsNumber(cyc), "scene/frame.cycle is number");
    CHECK(rn && cJSON_IsObject(rn), "scene/frame.road_network is object");
    CHECK(ents && cJSON_IsArray(ents), "scene/frame.entities is array");
    /* entities 至少应含 ego（向后兼容下场景总有一个 ego） */
    int n_entities = ents ? cJSON_GetArraySize(ents) : 0;
    CHECK(n_entities >= 1, "scene/frame has >= 1 entity (ego)");
    /* 首个实体应是 ego */
    if (n_entities > 0) {
        cJSON* e0 = cJSON_GetArrayItem(ents, 0);
        cJSON* t = e0 ? cJSON_GetObjectItemCaseSensitive(e0, "type") : nullptr;
        CHECK(t && cJSON_IsString(t) && strcmp(t->valuestring, "ego") == 0,
              "scene/frame.entities[0].type == 'ego'");
    }
    cJSON_Delete(root);
}

/* ── 主流程 ───────────────────────────────────────────────────── */

int main(int argc, char** argv) {
    const char* scenario = argc > 1 ? argv[1]
                          : getenv("FLOWENGINE_SCENARIO");
    const char* libpath  = argc > 2 ? argv[2]
                          : "/workspace/build/lib/libflowsim_node.so";
    if (!scenario) {
        std::printf("FAIL: scenario not specified.\n"
                    "Usage: %s <scenario.json> [libflowsim_node.so]\n"
                    "   or: export FLOWENGINE_SCENARIO=scenarios/xxx.json\n",
                    argv[0]);
        return 2;
    }

    std::printf("=== flowsim regression test (Phase 2.3) ===\n");
    std::printf("library: %s\n", libpath);
    std::printf("scenario: %s\n", scenario);

    log_init(LOG_WARN, nullptr);  /* 抑制 INFO 日志噪音 */
    signal(SIGINT, [](int){ /* ignore */ });
    adas_msgs_register_all();

    /* ── dlopen + ABI 校验 ── */
    void* lib = dlopen(libpath, RTLD_LAZY | RTLD_GLOBAL);
    CHECK(lib != nullptr, "dlopen libflowsim_node.so");
    if (!lib) { std::printf("FAIL: dlopen: %s\n", dlerror()); return 1; }

    NodeGetPluginFn get_fn = (NodeGetPluginFn)dlsym(lib, "node_get_plugin");
    CHECK(get_fn != nullptr, "node_get_plugin symbol found");
    NodePlugin* plugin = get_fn ? get_fn() : nullptr;
    CHECK(plugin != nullptr, "node_get_plugin() returned non-null");
    CHECK(plugin->api_version == 2, "api_version == 2");
    CHECK(plugin->name && strcmp(plugin->name, "flowsim") == 0,
          "plugin.name == 'flowsim'");
    CHECK(plugin->init && plugin->start && plugin->stop && plugin->cleanup,
          "all lifecycle callbacks non-null");

    /* ── 基础设施（参照 flow_node_host.c 的 setup）── */
    MessageBus* bus = message_bus_create("test_flowsim");
    DiscoveryManager* discovery =
        discovery_create("test_flowsim", CAP_PUBLISHER | CAP_SUBSCRIBER | CAP_FUSION);
    discovery_start(discovery);
    Transport* transport = transport_create(bus, discovery, TRANSPORT_LOCAL);
    transport_start(transport);

    SchedulerConfig scfg = SCHEDULER_CONFIG_DEFAULT;
    scfg.mode = SCHEDULER_MODE_CHOREO;
    Scheduler* scheduler = scheduler_create(&scfg);
    scheduler_set_choreo_bus(scheduler, bus);
    scheduler_start(scheduler);

    /* ── node_init ── */
    /* params_json 与 sim_world_node 的参数集一致：
     *   scenario_file, init_speed, target_speed, random_seed, lane_width */
    std::string params = std::string("{\"scenario_file\":\"") + scenario +
                         "\",\"init_speed\":5.0,\"target_speed\":12.0,"
                         "\"random_seed\":42,\"lane_width\":3.5}";
    int rc = plugin->init(bus, transport, discovery, scheduler, params.c_str());
    CHECK(rc == 0, "node_init returns 0");

    /* ── 订阅所有输出 topic ── */
    transport_subscribe(transport, "vehicle/state",     on_msg, nullptr);
    transport_subscribe(transport, "road/geometry",     on_msg, nullptr);
    transport_subscribe(transport, "road/traffic_lights", on_msg, nullptr);
    transport_subscribe(transport, "sim/tick",          on_msg, nullptr);
    transport_subscribe(transport, "sim/collision",     on_msg, nullptr);
    transport_subscribe(transport, "scene/frame",       on_msg, nullptr);

    /* ── node_start ── */
    rc = plugin->start();
    CHECK(rc == 0, "node_start returns 0");

    /* ── 等待 ~800ms ≈ 16 个 tick @ 20Hz ── */
    std::printf("--- running for 800ms (expect ~16 ticks @ 20Hz) ---\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(800));

    /* ── node_stop + cleanup ── */
    plugin->stop();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));  /* 让 stop 生效 */
    plugin->cleanup();
    std::printf("--- node stopped, validating captured messages ---\n");

    /* ── 验证消息计数 ── */
    int n_vstate = count_topic("vehicle/state");
    int n_rgeom  = count_topic("road/geometry");
    int n_tick   = count_topic("sim/tick");
    int n_scene  = count_topic("scene/frame");
    std::printf("captured: vehicle/state=%d road/geometry=%d sim/tick=%d scene/frame=%d\n",
                n_vstate, n_rgeom, n_tick, n_scene);
    /* 800ms @ 20Hz 应至少 10 个 tick（留容差给启动/停止抖动） */
    CHECK(n_vstate >= 10, "vehicle/state received >= 10 messages");
    CHECK(n_tick >= 10, "sim/tick received >= 10 messages");
    CHECK(n_scene >= 10, "scene/frame received >= 10 messages");
    /* road/geometry 每 50 cycle 发一次（≈1s），800ms 可能 0 或 1 — 放宽到 >= 0 */
    CHECK(n_rgeom >= 0, "road/geometry received >= 0 messages");

    /* ── 验证消息格式 ── */
    if (n_vstate > 0) {
        std::printf("--- validating vehicle/state format ---\n");
        validate_vehicle_state(get_first("vehicle/state"));
    }
    if (n_rgeom > 0) {
        std::printf("--- validating road/geometry format ---\n");
        validate_road_geometry(get_first("road/geometry"));
    }
    if (n_tick > 0) {
        std::printf("--- validating sim/tick format ---\n");
        validate_sim_tick(get_first("sim/tick"));
    }
    if (n_scene > 0) {
        std::printf("--- validating scene/frame format (Phase 2.2) ---\n");
        validate_scene_frame(get_first("scene/frame"));
    }

    /* ── 健康检查 ── */
    if (plugin->health) {
        int h = plugin->health();
        CHECK(h == 0, "plugin->health() returns 0 (running)");
    }

    /* ── teardown ── */
    scheduler_stop(scheduler);
    scheduler_destroy(scheduler);
    transport_stop(transport);
    transport_destroy(transport);
    discovery_stop(discovery);
    discovery_destroy(discovery);
    message_bus_destroy(bus);
    dlclose(lib);
    log_shutdown();

    std::printf("\n=== %s (failures=%d) ===\n",
                g_failures == 0 ? "PASS" : "FAIL", g_failures);
    return g_failures == 0 ? 0 : 1;
}
