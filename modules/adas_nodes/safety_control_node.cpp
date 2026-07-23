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
#include "topic_registry.h"
#include "adas_msgs_gen.h"
#include <cjson/cJSON.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <errno.h>
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
    int    turn_signal{0};   /* 0=off, 1=left, 2=right */
    bool   hazard{false};
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
    /* IEEE-754 下 NaN < x 恒为 false，未加防护时 clamp(NaN, 0.0, max_brake) 会
     * 返回 0.0（不刹车），clamp(NaN, -steer_limit, steer_limit) 会返回 -steer_limit
     * （一侧打死）。NaN/Inf 输入直接返回 lo（"不刹车/不转向"安全侧）；brake 的
     * 安全侧在 publish_cmd 里再做一次显式 isfinite 紧急刹车兜底。 */
    if (!std::isfinite(value)) return lo;
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
            cmd.turn_signal = (int)raw.turn_signal;
            cmd.hazard      = raw.hazard;
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
    /* 灯光指令：turn_signal 和 hazard 从 text 解析 */
    {
        double ts = 0, hz = 0;
        if (scan_double(text, "turn_signal=", &ts)) cmd.turn_signal = (int)ts;
        if (scan_double(text, "hazard=", &hz))     cmd.hazard = (hz != 0.0);
    }
    return cmd;
}

void on_fusion(const Message* msg, void*) {
    if (!msg || !msg->data) return;
    cJSON* root = cJSON_Parse((const char*)msg->data);
    if (!root) return;
    pthread_mutex_lock(&g.state_mutex);
    cJSON* j;
    if ((j = cJSON_GetObjectItemCaseSensitive(root, "v")) && cJSON_IsNumber(j))
        g.latest_state.speed = j->valuedouble;
    if ((j = cJSON_GetObjectItemCaseSensitive(root, "x")) && cJSON_IsNumber(j))
        g.latest_state.x = j->valuedouble;
    if ((j = cJSON_GetObjectItemCaseSensitive(root, "y")) && cJSON_IsNumber(j))
        g.latest_state.y = j->valuedouble;
    if ((j = cJSON_GetObjectItemCaseSensitive(root, "heading")) && cJSON_IsNumber(j))
        g.latest_state.heading = j->valuedouble;
    g.has_state = true;
    pthread_mutex_unlock(&g.state_mutex);
    cJSON_Delete(root);
}

void on_perception_obstacles(const Message* msg, void*) {
    if (!msg || !msg->data) return;
    ObstacleList list;
    if (ObstacleList_deserialize(&list, (const uint8_t*)msg->data, msg->data_size) != 0)
        return;

    pthread_mutex_lock(&g.state_mutex);
    VehicleState* state = &g.latest_state;
    state->ped_index = -1;
    double ch = cos(state->heading), sh = sin(state->heading);
    for (int i = 0; i < 4; i++) {
        if (i < (int)list.count) {
            const Obstacle* o = &list.obstacles[i];
            state->obs_x[i] = state->x + o->x * ch - o->y * sh;
            state->obs_y[i] = state->y + o->x * sh + o->y * ch;
            state->obs_v[i]  = o->vx * ch - o->vy * sh;
            state->obs_vy[i] = o->vx * sh + o->vy * ch;
            state->obs_valid[i] = true;
            switch (o->type) {
                case OBJ_TYPE_PEDESTRIAN: strncpy(state->obs_type[i], "pedestrian", sizeof(state->obs_type[i])-1); break;
                case OBJ_TYPE_CYCLIST:    strncpy(state->obs_type[i], "cyclist", sizeof(state->obs_type[i])-1); break;
                default:                  strncpy(state->obs_type[i], "car", sizeof(state->obs_type[i])-1); break;
            }
            if (o->type == OBJ_TYPE_PEDESTRIAN && state->ped_index < 0)
                state->ped_index = i;
        } else {
            state->obs_valid[i] = false;
            state->obs_x[i] = state->obs_y[i] = state->obs_v[i] = state->obs_vy[i] = 0.0;
            state->obs_type[i][0] = '\0';
        }
    }
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

/* Phase 5: 对向来车 TTC.
 * 检查对向车道 (dy > 2.0m) 是否有迎面驶来的车辆 (obs_v < -2 m/s)。
 * head-on closing speed = ego_speed + |obs_v|, 比同向 closing speed 大得多。 */
double min_oncoming_ttc(const VehicleState& state, double* out_dx = nullptr) {
    double best_ttc = 1e9;
    double best_dx = 0.0;
    for (int i = 0; i < 4; ++i) {
        if (!state.obs_valid[i]) continue;
        const double dx = state.obs_x[i] - state.x;
        const double dy = state.obs_y[i] - state.y;
        /* 对向车道: |dy| > 2.0m (双侧车道), 前方 60m */
        if (dx < 0.0 || dx > 60.0 || std::fabs(dy) < 2.0) continue;
        /* 迎面驶来: obs_v < -2 m/s */
        if (state.obs_v[i] > -2.0) continue;

        const double closing = state.speed + std::fabs(state.obs_v[i]);
        const double clearance = dx - 4.0;  /* 车长余量 */
        const double ttc = clearance / std::max(0.1, closing);
        if (ttc < best_ttc) {
            best_ttc = ttc;
            best_dx = dx;
        }
    }
    if (out_dx) *out_dx = best_dx;
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

            /* Phase 5: 对向碰撞安全检查。
             * 对向车道来车 (dy>2.0m, vx<-2m/s) 时计算 head-on TTC。
             * closing speed = ego_v + |obs_v|, 比同向大得多, 需要更早刹车。 */
            double oncoming_dx = 0.0;
            double oncoming_ttc = min_oncoming_ttc(state, &oncoming_dx);
            if (oncoming_ttc < 4.0) {
                set_changed(cmd.throttle, 0.0);
                double brake_floor = clamp((4.0 - oncoming_ttc) / 4.0, 0.5, 1.0);
                if (oncoming_dx < 15.0) brake_floor = std::max(brake_floor, 0.85);
                set_changed(cmd.brake, std::max(cmd.brake, brake_floor));
                if (oncoming_ttc < 1.5 || oncoming_dx < 8.0) {
                    set_changed(cmd.brake, 1.0);  /* 紧急制动 */
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

            /* Low-speed deadlock recovery: if ego has been stuck for too long
             * and the road ahead is clear, ease the brake and allow a small
             * throttle so the planner can creep forward (e.g. stopped at a
             * red light or blocked during a low-speed lane change).
             *
             * 安全复查：恢复前必须确认触发刹停的原因已消失。nearest_same_lane_gap
             * 只看同车道车辆（横向容差 2.0m），不复查行人/TTC/横穿风险——若 ego
             * 因行人而正确刹停，行人多等 5 秒（现实常见）就触发恢复，会把车命令
             * 朝着行人冲过去。故恢复前必须复查行人碰撞、斑马线等待、对向来车 TTC。 */
            static auto last_move_time = std::chrono::steady_clock::now();
            if (state.speed >= 0.5) {
                last_move_time = std::chrono::steady_clock::now();
            } else {
                double elapsed_ms = static_cast<double>(
                    std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::steady_clock::now() - last_move_time)
                        .count());
                if (elapsed_ms > 5000.0) {
                    /* 复查行人碰撞风险：行人仍在危险范围内不恢复 */
                    double ped_gap_recheck = pedestrian_collision_gap(state);
                    double ped_stop_gap_recheck = std::max(24.0, state.speed * 5.0);
                    /* 复查斑马线等待：斑马线区域仍有行人不恢复 */
                    double hold_gap_recheck = pedestrian_crossing_hold_gap(state);
                    /* 复查对向来车 TTC：参考 413-423 行 oncoming 紧急刹车逻辑 */
                    double oncoming_dx_recheck = 0.0;
                    double oncoming_ttc_recheck = min_oncoming_ttc(state, &oncoming_dx_recheck);

                    bool ped_still_dangerous = (ped_gap_recheck < ped_stop_gap_recheck);
                    bool crossing_still_dangerous = (hold_gap_recheck < 10.0);
                    bool oncoming_still_dangerous = (oncoming_ttc_recheck < 4.0 || oncoming_dx_recheck < 8.0);

                    if (!ped_still_dangerous && !crossing_still_dangerous && !oncoming_still_dangerous) {
                        double gap = nearest_same_lane_gap(state, params_);
                        if (gap > 10.0) {
                            fprintf(stderr, "[safety] low-speed recovery: spd=%.2f gap=%.2f -> brake=%.2f throttle=%.2f\n",
                                    state.speed, gap, cmd.brake, cmd.throttle);
                            set_changed(cmd.brake, std::min(cmd.brake, 0.30));
                            set_changed(cmd.throttle, std::max(cmd.throttle, 0.20));
                        }
                    }
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
        bin.gear           = GEAR_DRIVE;

        /* NaN/Inf 兜底：clamp 已把 NaN/Inf 收敛到 lo，但 brake 的 lo=0.0 意味着
         * "不刹车"，对制动不安全。发布前再做一次显式 isfinite 复查，任一字段
         * 非有限 → 强制 emergency_stop（brake=1.0, throttle=0.0, steer=0.0）。 */
        if (!std::isfinite(cmd.throttle) || !std::isfinite(cmd.brake) || !std::isfinite(cmd.steer)) {
            bin.throttle       = 0.0f;
            bin.brake          = 1.0f;
            bin.steering       = 0.0f;
            bin.emergency_stop = true;
            fprintf(stderr, "[safety] NaN/Inf in control cmd, forcing emergency stop\n");
        } else {
            bin.throttle       = (float)cmd.throttle;
            bin.brake          = (float)cmd.brake;
            bin.steering       = (float)cmd.steer;
            bin.emergency_stop = cmd.brake > 0.95;
        }
        bin.turn_signal = (uint8_t)cmd.turn_signal;
        bin.hazard      = cmd.hazard;

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

const char* s_inputs[] = {"control/raw_cmd", TOPIC_FUSION_LOCALIZATION, TOPIC_PERCEPTION_OBSTACLES, nullptr};
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

    transport_subscribe(transport, TOPIC_FUSION_LOCALIZATION, on_fusion, nullptr);
    transport_subscribe(transport, TOPIC_PERCEPTION_OBSTACLES, on_perception_obstacles, nullptr);
    transport_advertise(transport, "control/cmd", CONTROL_CMD_TYPE_ID);

    discovery_advertise(discovery, "control/raw_cmd", CONTROL_RAW_TYPE_ID, CAP_SUBSCRIBER, 0);
    discovery_advertise(discovery, TOPIC_FUSION_LOCALIZATION, 0xF0ED10C0u, CAP_SUBSCRIBER, 0);
    discovery_advertise(discovery, TOPIC_PERCEPTION_OBSTACLES, 0x0B5A010Eu, CAP_SUBSCRIBER, 0);
    discovery_advertise(discovery, "control/cmd", CONTROL_CMD_TYPE_ID, CAP_PUBLISHER, 100.0);

    g.task = std::make_unique<SafetyControlTask>(bus, transport, g.params);
    LOG_INFO("safety_control", "initialized (FlowCoro, max_thr=%.2f max_steer=%.2f)",
             g.params.max_throttle, g.params.max_steer);
    return 0;
}

int safety_start() {
    g.should_stop = false;
    if (!g.task) return -1;
    if (pthread_create(&g.thread, nullptr, safety_thread, nullptr) != 0) {
        LOG_WARN("safety_control", "pthread_create failed: %s", strerror(errno));
        return -1;
    }
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