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
#include "adas_msgs_gen.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <memory>
#include <pthread.h>
#include <string>

namespace {

constexpr uint32_t CONTROL_RAW_TYPE_ID = 0x871712d1u;  /* CONTROLRAW_TYPE_ID (adas_msgs_gen.h) */
constexpr uint32_t CONTROL_CMD_TYPE_ID = 0x2D95C6D2u;  /* CONTROLCMD_TYPE_ID (adas_msgs_gen.h) */
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
    double obs_x[4]{0.0, 0.0, 0.0, 0.0};
    double obs_y[4]{0.0, 0.0, 0.0, 0.0};
    double obs_v[4]{0.0, 0.0, 0.0, 0.0};
    double obs_vy[4]{0.0, 0.0, 0.0, 0.0};
    bool obs_valid[4]{false, false, false, false};
    char obs_type[4][16]{};   /* "car", "pedestrian", ... */
    int  ped_index{-1};       /* index of first pedestrian obs, -1 if none */
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
    pthread_mutex_t state_mutex = PTHREAD_MUTEX_INITIALIZER;
    VehicleState latest_state;
    bool has_state{false};
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

    /* Try binary deserialization first (serializer path) */
    {
        ControlRaw raw;
        if (ControlRaw_deserialize(&raw, (const uint8_t*)msg.data, msg.data_size) == 0) {
            cmd.throttle = raw.throttle;
            cmd.brake    = raw.brake;
            cmd.steer    = raw.steering;
            cmd.speed    = raw.speed;
            cmd.target   = raw.target;
            cmd.error    = raw.error;
            cmd.mode     = raw.mode;
            return cmd;
        }
    }

    /* Fallback: text format parsing */
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
    for (int i = 0; i < 4; ++i) {
        char key[16];
        std::snprintf(key, sizeof(key), "\"ox%d\":", i);
        state.obs_valid[i] = scan_double(text, key, &state.obs_x[i]);
        std::snprintf(key, sizeof(key), "\"oy%d\":", i);
        scan_double(text, key, &state.obs_y[i]);
        std::snprintf(key, sizeof(key), "\"ov%d\":", i);
        scan_double(text, key, &state.obs_v[i]);
        std::snprintf(key, sizeof(key), "\"ovy%d\":", i);
        scan_double(text, key, &state.obs_vy[i]);
        /* parse obstacle type to detect pedestrian dynamically */
        std::snprintf(key, sizeof(key), "\"ot%d\":\"", i);
        const char* p = std::strstr(text, key);
        if (p) {
            p += std::strlen(key);
            const char* end = std::strchr(p, '"');
            if (end) {
                std::size_t tlen = static_cast<std::size_t>(end - p);
                if (tlen >= sizeof(state.obs_type[i])) tlen = sizeof(state.obs_type[i]) - 1;
                std::memcpy(state.obs_type[i], p, tlen);
                state.obs_type[i][tlen] = '\0';
                if (state.ped_index < 0 && std::strcmp(state.obs_type[i], "pedestrian") == 0)
                    state.ped_index = i;
            }
        }
    }
    return state;
}

void on_vehicle_state(const Message* msg, void*) {
    if (!msg) return;
    VehicleState state = parse_vehicle_state(*msg);
    pthread_mutex_lock(&g.state_mutex);
    g.latest_state = state;
    g.has_state = true;
    pthread_mutex_unlock(&g.state_mutex);
}

double nearest_same_lane_gap(const VehicleState& state, const SafetyParams& params) {
    double best_gap = 1e9;
    for (int i = 0; i < 4; ++i) {
        if (!state.obs_valid[i]) continue;
        if (std::fabs(state.obs_y[i] - state.y) > params.same_lane_tol) continue;
        double dx = state.obs_x[i] - state.x;
        double gap = dx - 4.6;
        if (dx > 0.0 && gap < best_gap) best_gap = gap;
    }
    return best_gap;
}

double pedestrian_collision_gap(const VehicleState& state) {
    int pi = state.ped_index;
    if (pi < 0 || !state.obs_valid[pi]) return 1e9;
    double dx = state.obs_x[pi] - state.x;
    double dy = std::fabs(state.obs_y[pi] - state.y);
    if (std::fabs(dx) > 70.0 || dy > 4.5) return 1e9;
    return std::fabs(dx) - 2.8;
}

double pedestrian_crossing_hold_gap(const VehicleState& state) {
    int pi = state.ped_index;
    if (pi < 0 || !state.obs_valid[pi]) return 1e9;

    const double dx = state.obs_x[pi] - state.x;
    const double dy = std::fabs(state.obs_y[pi] - state.y);
    const double vyy = std::fabs(state.obs_vy[pi]);

    /* Guard zone: if pedestrian is crossing (or very close to lane center),
     * keep ego at least this distance behind the crossing line.
     *
     * Two-tier detection:
     *   dy < 3.0m     → always active (pedestrian ON the road, even if stopped)
     *   vyy > 0.05    → pedestrian is actively moving near the road (crossing intent)
     * Otherwise        → pedestrian is parked at curb → NOT crossing, release hold */
    const bool crossing_active = (dy < 3.0) || (vyy > 0.05 && dy < 6.5);
    if (!crossing_active) return 1e9;
    if (dx < -2.0 || dx > 35.0) return 1e9;

    constexpr double kCrossingBufferM = 6.0;
    return dx - kCrossingBufferM;
}

double min_vehicle_ttc(const VehicleState& state, double* out_dx = nullptr, double* out_dy = nullptr) {
    double best_ttc = 1e9;
    double best_dx = 0.0;
    double best_dy = 0.0;
    for (int i = 0; i < 4; ++i) {
        if (!state.obs_valid[i]) continue;
        const double dx = state.obs_x[i] - state.x;
        const double dy = std::fabs(state.obs_y[i] - state.y);
        if (dx < 0.0 || dx > 35.0 || dy > 2.3) continue;

        const double closing = state.speed - state.obs_v[i];
        if (closing <= 0.4) continue;

        const double clearance = dx - 4.8;
        const double ttc = clearance / std::max(0.1, closing);
        if (ttc < best_ttc) {
            best_ttc = ttc;
            best_dx = dx;
            best_dy = dy;
        }
    }
    if (out_dx) *out_dx = best_dx;
    if (out_dy) *out_dy = best_dy;
    return best_ttc;
}

double nearest_vehicle_lateral_cross_risk(const VehicleState& state, double* out_dx = nullptr, double* out_dy_signed = nullptr) {
    double best = 1e9;
    double best_dx = 0.0;
    double best_dy_signed = 0.0;
    for (int i = 0; i < 4; ++i) {
        if (!state.obs_valid[i]) continue;
        const double dx = state.obs_x[i] - state.x;
        const double dy_signed = state.obs_y[i] - state.y;
        const double dy = std::fabs(dy_signed);
        if (dx < -5.0 || dx > 12.0) continue;
        if (dy > 2.2) continue;
        const double metric = std::fabs(dx) + 2.0 * dy;
        if (metric < best) {
            best = metric;
            best_dx = dx;
            best_dy_signed = dy_signed;
        }
    }
    if (out_dx) *out_dx = best_dx;
    if (out_dy_signed) *out_dy_signed = best_dy_signed;
    return best;
}

class SafetyControlTask : public CoroutineTask {
public:
    SafetyControlTask(MessageBus* bus, Transport* transport, const SafetyParams& params)
        : CoroutineTask(bus), transport_(transport), params_(params) {}

protected:
    Task run() override {
        uint32_t cycle = 0;

        LOG_INFO("safety_control", "safety gate started (synchronous resume)");
        while (!should_stop()) {
            Message msg = co_await when_any_bus(bus(), {"control/raw_cmd", "inference/raw_cmd"}, &cancel_token_);
            if (std::strcmp(msg.topic, "control/raw_cmd") != 0 &&
                std::strcmp(msg.topic, "inference/raw_cmd") != 0) continue;

            ControlCmd cmd = parse_control_cmd(msg);
            VehicleState state;
            bool has_state = false;
            pthread_mutex_lock(&g.state_mutex);
            state = g.latest_state;
            has_state = g.has_state;
            pthread_mutex_unlock(&g.state_mutex);
            bool intervened = apply_safety(cmd, state, has_state);
            publish_cmd(cmd, intervened);

            ++cycle;
            if (intervened || cycle % 20 == 1) {
                LOG_INFO("safety_control", "#%u thr=%.2f brk=%.2f st=%.4f spd=%.1f tgt=%.1f %s",
                         cycle, cmd.throttle, cmd.brake, cmd.steer, cmd.speed, cmd.target,
                         intervened ? "INTERVENED" : "pass");
            }
        }
        LOG_INFO("safety_control", "safety gate stopped");
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

            /* Near-field vehicle guard: brake by TTC to avoid side/front scrape
             * when ego is between lanes and still closing on a lead vehicle. */
            double risk_dx = 0.0;
            double risk_dy = 0.0;
            double ttc = min_vehicle_ttc(state, &risk_dx, &risk_dy);
            if (ttc < 2.2) {
                set_changed(cmd.throttle, 0.0);
                double brake_floor = clamp((2.2 - ttc) / 2.2, 0.45, 1.0);
                if (risk_dx < 8.0 && risk_dy < 2.1) {
                    brake_floor = std::max(brake_floor, 0.85);
                }
                set_changed(cmd.brake, std::max(cmd.brake, brake_floor));
                if (ttc < 1.0 || (risk_dx < 6.5 && risk_dy < 1.9)) {
                    set_changed(cmd.brake, 1.0);
                }
            }

            /* Lateral crossing guard: if another car is near while ego is crossing lanes,
             * suppress steering authority and force stronger braking. */
            double cross_dx = 0.0;
            double cross_dy_signed = 0.0;
            double cross_risk = nearest_vehicle_lateral_cross_risk(state, &cross_dx, &cross_dy_signed);
            const bool crossing_intent = std::fabs(cmd.steer) > 0.08 &&
                                         cmd.mode.find("ROAD_GUARD") == std::string::npos;
            if (crossing_intent && cross_risk < 9.0 && state.speed > 7.0) {
                set_changed(cmd.throttle, 0.0);
                set_changed(cmd.brake, std::max(cmd.brake, 0.65));
                double steer_guard = 0.06;
                const double cross_dy = std::fabs(cross_dy_signed);
                if (std::fabs(cross_dx) < 5.0 && cross_dy < 1.9) {
                    set_changed(cmd.brake, 1.0);
                    steer_guard = 0.03;
                }

                /* 转向安全约束：只在风险车仍在前方时限制转向方向
                 * （防止变道过半后回正方向被错误覆盖——此时风险车已到侧后方，
                 * 自然的回正转向看似"朝向风险车"但实为正确的变道收尾动作）。 */
                if (cross_dx > 0.0) {
                    if (cross_dy_signed < 0.0) {
                        cmd.steer = std::max(cmd.steer, steer_guard);
                    } else {
                        cmd.steer = std::min(cmd.steer, -steer_guard);
                    }
                }
            }

            double ped_gap = pedestrian_collision_gap(state);
            double ped_stop_gap = std::max(24.0, state.speed * 5.0);
            if (ped_gap < ped_stop_gap) {
                double ratio = clamp(ped_gap / ped_stop_gap, 0.0, 1.0);
                set_changed(cmd.throttle, 0.0);
                set_changed(cmd.brake, std::max(cmd.brake, 1.0 - ratio));
                if (ped_gap < ped_stop_gap * 0.55) {
                    set_changed(cmd.brake, 1.0);
                }
            }

            /* Crossing-line hold: do not stop on/near the pedestrian crossing line. */
            double hold_gap = pedestrian_crossing_hold_gap(state);
            if (hold_gap < 10.0) {
                double ratio = clamp(hold_gap / 10.0, 0.0, 1.0);
                set_changed(cmd.throttle, 0.0);
                set_changed(cmd.brake, std::max(cmd.brake, 1.0 - ratio));
                if (hold_gap < 1.5) {
                    set_changed(cmd.brake, 1.0);
                }
            }
        }
        if (changed && cmd.mode.find("SAFE") == std::string::npos) {
            cmd.mode += "+SAFE";
        }
        return changed;
    }

    void publish_cmd(const ControlCmd& cmd, bool intervened) const {
        /* Binary serialized ControlCmd (serializer path) */
        ::ControlCmd bin;
        bin.seq            = 0;
        bin.throttle       = (float)cmd.throttle;
        bin.brake          = (float)cmd.brake;
        bin.steering       = (float)cmd.steer;
        bin.gear           = GEAR_DRIVE;
        bin.emergency_stop = cmd.brake > 0.95;

        uint8_t buf[32];
        size_t len = sizeof(buf);
        ControlCmd_serialize(&bin, buf, &len);
        transport_publish(transport_, "control/cmd", buf, (uint32_t)len);

        /* Text format for logging/backward compat */
        char out[320];
        std::snprintf(out, sizeof(out),
                      "throttle=%.2f brake=%.2f steer=%.4f speed=%.1f target=%.1f "
                      "error=%.1f mode=%s safety=%s",
                      cmd.throttle, cmd.brake, cmd.steer, cmd.speed, cmd.target, cmd.error,
                      cmd.mode.c_str(), intervened ? "intervened" : "pass");
        transport_publish(transport_, "control/cmd/text", out,
                          static_cast<uint32_t>(std::strlen(out) + 1));
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
    g.has_state = false;

    if (params_json) {
        scan_double(params_json, "\"max_throttle\":", &g.params.max_throttle);
        scan_double(params_json, "\"max_steer\":", &g.params.max_steer);
        scan_double(params_json, "\"low_speed_steer\":", &g.params.low_speed_steer);
        scan_double(params_json, "\"time_headway\":", &g.params.time_headway);
    }

    transport_subscribe(transport, "vehicle/state", on_vehicle_state, nullptr);
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
    NODE_PLUGIN_API_VERSION,
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