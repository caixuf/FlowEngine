/**
 * safety_control_node.cpp — FlowCoro safety gate for control commands
 *
 * Subscribes raw controller output and vehicle state, applies a small safety
 * envelope, then publishes the final control/cmd consumed by sim_world.
 */

#include "coroutine_task.h"
#undef LOG_TRACE
#undef LOG_DEBUG
#undef LOG_INFO
#undef LOG_WARN
#undef LOG_ERROR
#undef LOG_FATAL
#include "logger.h"
#include "node_plugin.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <memory>
#include <pthread.h>
#include <string>

namespace {

constexpr uint32_t CONTROL_RAW_TYPE_ID = 0x2D95C6D3u;
constexpr uint32_t CONTROL_CMD_TYPE_ID = 0x2D95C6D2u;
constexpr uint32_t VEHICLE_STATE_TYPE_ID = 0x1C0E5A7Eu;

struct ControlCmd {
    double throttle{0.0};
    double brake{0.0};
    double steer{0.0};
    double speed{0.0};
    double target{0.0};
    double error{0.0};
    std::string mode{"RAW"};
};

struct VehicleState {
    double x{0.0};
    double y{0.0};
    double speed{0.0};
    double heading{0.0};
    double obs_x[3]{0.0, 0.0, 0.0};
    double obs_y[3]{0.0, 0.0, 0.0};
    double obs_v[3]{0.0, 0.0, 0.0};
    bool obs_valid[3]{false, false, false};
};

struct SafetyParams {
    double max_throttle{0.85};
    double max_brake{1.0};
    double max_steer{0.22};
    double low_speed_steer{0.18};
    double same_lane_tol{2.0};
    double min_gap{6.0};
    double time_headway{1.8};
    double hard_brake_ratio{0.45};
};

struct SafetyContext {
    MessageBus* bus{nullptr};
    Transport* transport{nullptr};
    DiscoveryManager* discovery{nullptr};
    Scheduler* scheduler{nullptr};
    std::unique_ptr<class SafetyControlTask> task;
    pthread_t thread{};
    std::atomic<bool> running{false};
    std::atomic<bool> should_stop{false};
    SafetyParams params;
};

SafetyContext g;

double clamp(double value, double lo, double hi) {
    return std::max(lo, std::min(value, hi));
}

bool scan_double(const char* text, const char* key, double* out) {
    const char* p = std::strstr(text, key);
    if (!p) return false;
    return std::sscanf(p + std::strlen(key), "%lf", out) == 1;
}

std::string scan_mode(const char* text) {
    const char* p = std::strstr(text, "mode=");
    if (!p) return "RAW";
    p += 5;
    char mode[32]{};
    if (std::sscanf(p, "%31s", mode) == 1) return mode;
    return "RAW";
}

ControlCmd parse_control_cmd(const Message& msg) {
    ControlCmd cmd;
    const char* text = reinterpret_cast<const char*>(msg.data);
    if (!text) return cmd;
    scan_double(text, "throttle=", &cmd.throttle);
    scan_double(text, "brake=", &cmd.brake);
    scan_double(text, "steer=", &cmd.steer);
    scan_double(text, "speed=", &cmd.speed);
    scan_double(text, "target=", &cmd.target);
    scan_double(text, "error=", &cmd.error);
    cmd.mode = scan_mode(text);
    return cmd;
}

VehicleState parse_vehicle_state(const Message& msg) {
    VehicleState state;
    const char* text = reinterpret_cast<const char*>(msg.data);
    if (!text) return state;
    scan_double(text, "\"x\":", &state.x);
    scan_double(text, "\"y\":", &state.y);
    scan_double(text, "\"spd\":", &state.speed);
    scan_double(text, "\"hdg\":", &state.heading);
    for (int i = 0; i < 3; ++i) {
        char key[16];
        std::snprintf(key, sizeof(key), "\"ox%d\":", i);
        state.obs_valid[i] = scan_double(text, key, &state.obs_x[i]);
        std::snprintf(key, sizeof(key), "\"oy%d\":", i);
        scan_double(text, key, &state.obs_y[i]);
        std::snprintf(key, sizeof(key), "\"ov%d\":", i);
        scan_double(text, key, &state.obs_v[i]);
    }
    return state;
}

double nearest_same_lane_gap(const VehicleState& state, const SafetyParams& params) {
    double best_gap = 1e9;
    for (int i = 0; i < 3; ++i) {
        if (!state.obs_valid[i]) continue;
        if (std::fabs(state.obs_y[i] - state.y) > params.same_lane_tol) continue;
        double dx = state.obs_x[i] - state.x;
        double gap = dx - 4.6;
        if (dx > 0.0 && gap < best_gap) best_gap = gap;
    }
    return best_gap;
}

class SafetyControlTask : public FlowCoroTask {
public:
    SafetyControlTask(MessageBus* bus, Transport* transport, const SafetyParams& params)
        : FlowCoroTask(bus), transport_(transport), params_(params) {}

protected:
    Task run() override {
        VehicleState state;
        bool has_state = false;
        uint32_t cycle = 0;

        LOG_INFO("safety_control", "FlowCoro safety gate started");
        while (!should_stop()) {
            Message msg = co_await when_any_bus(bus(), {"control/raw_cmd", "vehicle/state"}, &cancel_token_);
            if (std::strcmp(msg.topic, "vehicle/state") == 0) {
                state = parse_vehicle_state(msg);
                has_state = true;
                continue;
            }
            if (std::strcmp(msg.topic, "control/raw_cmd") != 0) continue;

            ControlCmd cmd = parse_control_cmd(msg);
            bool intervened = apply_safety(cmd, state, has_state);
            publish_cmd(cmd, intervened);

            ++cycle;
            if (intervened || cycle % 20 == 1) {
                LOG_INFO("safety_control", "#%u thr=%.2f brk=%.2f st=%.4f spd=%.1f tgt=%.1f %s",
                         cycle, cmd.throttle, cmd.brake, cmd.steer, cmd.speed, cmd.target,
                         intervened ? "INTERVENED" : "pass");
            }
        }
        LOG_INFO("safety_control", "FlowCoro safety gate stopped");
    }

private:
    bool apply_safety(ControlCmd& cmd, const VehicleState& state, bool has_state) const {
        bool changed = false;
        auto set_changed = [&](double& field, double value) {
            if (std::fabs(field - value) > 1e-6) changed = true;
            field = value;
        };

        set_changed(cmd.throttle, clamp(cmd.throttle, 0.0, params_.max_throttle));
        set_changed(cmd.brake, clamp(cmd.brake, 0.0, params_.max_brake));
        double steer_limit = (has_state && state.speed < 3.0) ? params_.low_speed_steer : params_.max_steer;
        set_changed(cmd.steer, clamp(cmd.steer, -steer_limit, steer_limit));

        if (has_state) {
            double gap = nearest_same_lane_gap(state, params_);
            double safe_gap = params_.min_gap + state.speed * params_.time_headway;
            if (gap < safe_gap && gap < 80.0) {
                double ratio = clamp(gap / safe_gap, 0.0, 1.0);
                double limited_throttle = cmd.throttle * ratio;
                set_changed(cmd.throttle, std::min(cmd.throttle, limited_throttle));
                if (ratio < params_.hard_brake_ratio) {
                    set_changed(cmd.brake, std::max(cmd.brake, 1.0 - ratio));
                }
            }
        }
        if (changed && cmd.mode.find("SAFE") == std::string::npos) {
            cmd.mode += "+SAFE";
        }
        return changed;
    }

    void publish_cmd(const ControlCmd& cmd, bool intervened) const {
        char out[320];
        std::snprintf(out, sizeof(out),
                      "throttle=%.2f brake=%.2f steer=%.4f speed=%.1f target=%.1f "
                      "error=%.1f mode=%s safety=%s",
                      cmd.throttle, cmd.brake, cmd.steer, cmd.speed, cmd.target, cmd.error,
                      cmd.mode.c_str(), intervened ? "intervened" : "pass");
        transport_publish(transport_, "control/cmd", out, static_cast<uint32_t>(std::strlen(out) + 1));
    }

    Transport* transport_;
    SafetyParams params_;
};

void* safety_thread(void*) {
    pthread_setname_np(pthread_self(), "safety_ctrl");
    try {
        g.task->execute();
    } catch (...) {
        LOG_ERROR("safety_control", "FlowCoro task failed");
    }
    return nullptr;
}

const char* s_inputs[] = {"control/raw_cmd", "vehicle/state", nullptr};
const char* s_outputs[] = {"control/cmd", nullptr};
extern NodePlugin s_plugin;

int safety_init(MessageBus* bus, Transport* transport, DiscoveryManager* discovery,
                Scheduler* scheduler, const char* params_json) {
    g.bus = bus;
    g.transport = transport;
    g.discovery = discovery;
    g.scheduler = scheduler;
    g.should_stop = false;
    g.running = false;
    g.params = SafetyParams{};

    if (params_json) {
        scan_double(params_json, "\"max_throttle\":", &g.params.max_throttle);
        scan_double(params_json, "\"max_steer\":", &g.params.max_steer);
        scan_double(params_json, "\"low_speed_steer\":", &g.params.low_speed_steer);
        scan_double(params_json, "\"time_headway\":", &g.params.time_headway);
    }

    transport_advertise(transport, "control/cmd", CONTROL_CMD_TYPE_ID);

    discovery_advertise(discovery, "control/raw_cmd", CONTROL_RAW_TYPE_ID, CAP_SUBSCRIBER, 0);
    discovery_advertise(discovery, "vehicle/state", VEHICLE_STATE_TYPE_ID, CAP_SUBSCRIBER, 0);
    discovery_advertise(discovery, "control/cmd", CONTROL_CMD_TYPE_ID, CAP_PUBLISHER, 100.0);

    g.task = std::make_unique<SafetyControlTask>(bus, transport, g.params);
    LOG_INFO("safety_control", "initialized (FlowCoro, max_thr=%.2f max_steer=%.2f)",
             g.params.max_throttle, g.params.max_steer);
    return 0;
}

int safety_start() {
    g.should_stop = false;
    if (!g.task) return -1;
    if (pthread_create(&g.thread, nullptr, safety_thread, nullptr) != 0) return -1;
    g.running = true;
    node_announce_self(g.transport, &s_plugin);
    LOG_INFO("safety_control", "started");
    return 0;
}

void safety_stop() {
    g.should_stop = true;
    if (g.task) g.task->stop();
}

void safety_cleanup() {
    safety_stop();
    if (g.running) {
        pthread_join(g.thread, nullptr);
        g.running = false;
    }
    g.task.reset();
    LOG_INFO("safety_control", "cleanup done");
}

int safety_health() {
    return g.task && !g.task->should_stop() ? 0 : -1;
}

NodePlugin s_plugin = {
    "safety_control",
    "1.0.0",
    "FlowCoro safety envelope for control commands",
    s_inputs,
    s_outputs,
    safety_init,
    safety_start,
    safety_stop,
    safety_cleanup,
    safety_health,
};

} // namespace

extern "C" NodePlugin* node_get_plugin(void) { return &s_plugin; }