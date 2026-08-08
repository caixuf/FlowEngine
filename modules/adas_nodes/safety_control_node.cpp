/**
 * safety_control_node.cpp — FlowCoro safety gate for control commands
 *
 * Subscribes raw controller output and vehicle state, applies a small safety
 * envelope, then publishes the final control/cmd consumed by flowsim_node.
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
#include "degrade_ladder.h"
#include "clock_service.h"
#include <cjson/cJSON.h>

#include <algorithm>
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

/* 上游 perception 发布的 ObstacleList 容量。从已包含的 adas_msgs_gen.h →
 * ObstacleList.h 中实际结构体推导，避免与 adas_msgs.h 的 ADAS_MAX_OBSTACLES
 * 冲突（后者会与生成的 ObstacleList 重复定义）。当协议扩容时此常量自动跟随。 */
constexpr int kMaxObs = (int)(sizeof(::ObstacleList::obstacles) / sizeof(::Obstacle));

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
    int    gear{GEAR_DRIVE}; /* 档位，从 control_node 透传 */
};

struct VehicleState {
    double x{0.0};
    double y{0.0};
    double speed{0.0};
    double heading{0.0};
    double obs_x[kMaxObs]{};
    double obs_y[kMaxObs]{};
    double obs_v[kMaxObs]{};
    double obs_vy[kMaxObs]{};
    bool obs_valid[kMaxObs]{};
    char obs_type[kMaxObs][16]{};   /* "car", "pedestrian", ... */
    int  ped_index{-1};             /* index of first pedestrian obs, -1 if none */
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
    /* TaskBase 包装器（由 EXPORT_COROUTINE_TASK 宏创建） */
    struct safety_control_Wrapper* task_wrapper{nullptr};
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
            cmd.gear        = (int)raw.gear;
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
        double ts = 0, hz = 0, gv = 0;
        if (scan_double(text, "turn_signal=", &ts)) cmd.turn_signal = (int)ts;
        if (scan_double(text, "hazard=", &hz))     cmd.hazard = (hz != 0.0);
        if (scan_double(text, "gear=", &gv))       cmd.gear = (int)gv;
    }
    return cmd;
}

void on_fusion(const Message* msg, void*) {
    if (!msg) return;
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
    if (!msg) return;
    ObstacleList list;
    if (ObstacleList_deserialize(&list, (const uint8_t*)msg->data, msg->data_size) != 0)
        return;

    pthread_mutex_lock(&g.state_mutex);
    VehicleState* state = &g.latest_state;
    state->ped_index = -1;
    double ch = cos(state->heading), sh = sin(state->heading);
    for (int i = 0; i < kMaxObs; i++) {
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
                case OBJ_TYPE_CONSTRUCTION: strncpy(state->obs_type[i], "construction", sizeof(state->obs_type[i])-1); break;
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
    /* 方向感知（2026-08-04 掉头返程）：旧实现 dx=obs_x-ego_x 世界 +x 判"前方"，
     * 返程向西时前车在 -x（dx<0）被 skip → 同向 gap 恒 1e9 → 返程跟车失效。
     * 沿车头方向投影 ahead 后前进/返程统一。 */
    const double fwd_x = std::cos(state.heading);
    const double fwd_y = std::sin(state.heading);
    for (int i = 0; i < kMaxObs; ++i) {
        if (!state.obs_valid[i]) continue;
        const double ex = state.obs_x[i] - state.x;
        const double ey = state.obs_y[i] - state.y;
        const double lat = std::fabs(-ex * fwd_y + ey * fwd_x);
        if (lat > params.same_lane_tol) continue;
        const double along = ex * fwd_x + ey * fwd_y;
        const double gap = along - 4.6;
        if (along > 0.0 && gap < best_gap) best_gap = gap;
    }
    return best_gap;
}

double pedestrian_collision_gap(const VehicleState& state) {
    int pi = state.ped_index;
    if (pi < 0 || !state.obs_valid[pi]) return 1e9;
    /* 方向感知（2026-08-05）：旧实现用世界 dx = obs_x-ego_x，掉头返程（heading≈π
     * 朝西）时前方行人 dx<0 被当"后方"、后方行人被当"前方" → 方向反。沿车头方向
     * 投影 along 后前进/返程统一。行人在 ego 后方（along ≤ 0）无前向碰撞风险。 */
    const double fwd_x = std::cos(state.heading);
    const double fwd_y = std::sin(state.heading);
    const double ex = state.obs_x[pi] - state.x;
    const double ey = state.obs_y[pi] - state.y;
    const double along = ex * fwd_x + ey * fwd_y;            /* 沿车头前方距离 */
    const double lat = std::fabs(-ex * fwd_y + ey * fwd_x);  /* 横向（车体系） */
    if (along <= 0.0 || along > 70.0 || lat > 4.5) return 1e9;
    return along - 2.8;
}

double pedestrian_crossing_hold_gap(const VehicleState& state) {
    int pi = state.ped_index;
    if (pi < 0 || !state.obs_valid[pi]) return 1e9;

    /* 方向感知（2026-08-05）：同 pedestrian_collision_gap，沿车头投影，返程自适应 */
    const double fwd_x = std::cos(state.heading);
    const double fwd_y = std::sin(state.heading);
    const double ex = state.obs_x[pi] - state.x;
    const double ey = state.obs_y[pi] - state.y;
    const double along = ex * fwd_x + ey * fwd_y;
    const double dy = std::fabs(-ex * fwd_y + ey * fwd_x);
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
    if (along < -2.0 || along > 35.0) return 1e9;

    constexpr double kCrossingBufferM = 6.0;
    return along - kCrossingBufferM;
}

double min_vehicle_ttc(const VehicleState& state, double* out_dx = nullptr, double* out_dy = nullptr) {
    double best_ttc = 1e9;
    double best_dx = 0.0;
    double best_dy = 0.0;
    /* 方向感知（2026-08-04 掉头返程同向防撞失效）：旧实现用世界 dx=obs_x-ego_x，
     * 返程 ego 向西时前车在 -x（dx<0）被 skip → 同向 TTC 完全失效，返程无防撞。
     * 改为沿车头方向投影 ahead + 沿向速度，前进/返程统一。 */
    const double fwd_x = std::cos(state.heading);
    const double fwd_y = std::sin(state.heading);
    for (int i = 0; i < kMaxObs; ++i) {
        if (!state.obs_valid[i]) continue;
        const double ex = state.obs_x[i] - state.x;
        const double ey = state.obs_y[i] - state.y;
        const double along = ex * fwd_x + ey * fwd_y;      /* 沿车头前方距离 */
        const double lat = std::fabs(-ex * fwd_y + ey * fwd_x);  /* 横向偏移 */
        if (along < 0.0 || along > 35.0 || lat > 2.3) continue;

        const double along_v = state.obs_v[i] * fwd_x + state.obs_vy[i] * fwd_y;
        const double closing = state.speed - along_v;
        if (closing <= 0.4) continue;

        const double clearance = along - 4.8;
        const double ttc = clearance / std::max(0.1, closing);
        if (ttc < best_ttc) {
            best_ttc = ttc;
            best_dx = along;
            best_dy = lat;
        }
    }
    if (out_dx) *out_dx = best_dx;
    if (out_dy) *out_dy = best_dy;
    return best_ttc;
}

/* Phase 5: 对向来车 TTC.
 * 检查相邻对向车道 (2.0 < |dy| ≤ 6.0m) 是否有迎面驶来的车辆。
 * head-on closing speed = ego_speed + |obs_v_along|, 比同向 closing speed 大得多。
 *
 * |dy| 上界 6.0m ≈ 1.5×标准车道宽：只把**相邻车道**的对向车当迎头威胁。
 * 旧逻辑 |dy|>2.0 把对向任意车道都算上，多车道高速上 ego 在 lane3、对向车
 * 在 lane0（横向 10.5m，中间隔两条车道）也触发 → 巡航被压到 0.5~1.0 全刹
 * （2026-07-31 实跑：合并回 lane2 途中遇 2 车道外对向车刹停到 0）。
 *
 * 方向感知（2026-08-04 掉头返程幽灵刹车）：旧实现用世界 obs_v < -2 判"迎面"，
 * 掉头返程 ego 向西（世界 vx<0）时，**同向车**（世界 vx 也 <0）被误判为迎头 →
 * head-on TTC 用 speed+|obs_v| 硬刹 → 不敢超旁边车道同向车。改为沿车头方向
 * 投影沿向速度 along_v：沿向为负（朝 ego 靠近）才算迎头，同向车沿向恒为正。
 */
static constexpr double kOncomingSameLaneLatMax = 2.3;  /* 同车道横向容差（车宽半幅+余量） */
double min_oncoming_ttc(const VehicleState& state, double* out_dx = nullptr) {
    double best_ttc = 1e9;
    double best_dx = 0.0;
    const double fwd_x = std::cos(state.heading);
    const double fwd_y = std::sin(state.heading);
    for (int i = 0; i < kMaxObs; ++i) {
        if (!state.obs_valid[i]) continue;
        const double dx = state.obs_x[i] - state.x;
        const double dy = state.obs_y[i] - state.y;
        /* 2026-08-05 同车道头对头修复：只把**同一条车道**、迎面驶来的车当迎头威胁。
         * 旧逻辑 2.0<|dy|≤6.0 把相邻对向车道（间隔 3.5m 的分隔车道）也当迎头 → 掉头
         * 返程（heading≈π 朝西）每次接近东行对向车 head-on TTC 硬刹 = 幽灵刹车。 */
        const double along = dx * fwd_x + dy * fwd_y;
        if (along < 0.0 || along > 60.0) continue;
        const double lat = std::fabs(-dx * fwd_y + dy * fwd_x); /* 横向（车体系投影） */
        if (lat > kOncomingSameLaneLatMax) continue;           /* 只在同车道才算迎头 */
        /* 沿车头方向速度：负 = 朝 ego 驶来（迎头）。同向/静止车沿向恒 ≥ -2，不算迎头 */
        const double along_v = state.obs_v[i] * fwd_x + state.obs_vy[i] * fwd_y;
        if (along_v > -2.0) continue;

        const double closing = state.speed + std::fabs(along_v);
        const double clearance = along - 4.0;  /* 车长余量 */
        const double ttc = clearance / std::max(0.1, closing);
        if (ttc < best_ttc) {
            best_ttc = ttc;
            best_dx = along;
        }
    }
    if (out_dx) *out_dx = best_dx;
    return best_ttc;
}

double nearest_vehicle_lateral_cross_risk(const VehicleState& state, double* out_dx = nullptr, double* out_dy_signed = nullptr) {
    double best = 1e9;
    double best_dx = 0.0;
    double best_dy_signed = 0.0;
    /* 方向感知（2026-08-04 掉头返程）：旧实现用世界 dx 窗口 [-5,12]，返程
     * 时把车后的障碍算进横向穿越风险 → 误刹。改沿车头投影（ahead 窗口 + 横向）。 */
    const double fwd_x = std::cos(state.heading);
    const double fwd_y = std::sin(state.heading);
    for (int i = 0; i < kMaxObs; ++i) {
        if (!state.obs_valid[i]) continue;
        const double ex = state.obs_x[i] - state.x;
        const double ey = state.obs_y[i] - state.y;
        const double along = ex * fwd_x + ey * fwd_y;
        const double dy_signed = -ex * fwd_y + ey * fwd_x;
        const double dy = std::fabs(dy_signed);
        if (along < -5.0 || along > 12.0) continue;
        if (dy > 2.2) continue;
        const double metric = std::fabs(along) + 2.0 * dy;
        if (metric < best) {
            best = metric;
            best_dx = along;
            best_dy_signed = dy_signed;
        }
    }
    if (out_dx) *out_dx = best_dx;
    if (out_dy_signed) *out_dy_signed = best_dy_signed;
    return best;
}

class SafetyControlTask : public CoroutineTask {
public:
    SafetyControlTask(MessageBus* bus) : CoroutineTask(bus) {}

    void set_params(Transport* transport, const SafetyParams& params) {
        transport_ = transport;
        params_ = params;
    }

protected:
    Task run() override {
        uint32_t cycle = 0;
        uint64_t last_msg_us = clock_now_us();

        /* 常驻订阅桥：替代 when_any_bus_for——该适配器（WhenAnyBusAwaitableT
         * 反复订阅/退订）多次循环后消息与超时 fire 双失效，safety 曾 3 次
         * run 均在启动后 1-3s 永久挂起（control/cmd 断流 → 内置巡航追尾）。
         * 桥订阅生命周期=节点，5ms 轮询取槽，不再依赖事件唤醒。 */
        BusQueueBridge cmd_bridge(bus(), {"control/raw_cmd", "inference/raw_cmd"});

        LOG_INFO("safety_control", "safety gate started (bus bridge polling)");
        while (!should_stop()) {
            std::string topic;
            Message msg;
            if (cmd_bridge.try_take_any(&topic, &msg)) {
                last_msg_us = clock_now_us();

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
            } else {
                /* 无新消息：数据超时检查 + 固定 5ms 轮询 */
                uint64_t now_us = clock_now_us();
                /* 车已停稳（speed<=0.5）时 raw_cmd 停发属正常行为：
                 * 例如红灯前刹停后 control 不再高频发 cmd。此时心跳缺失
                 * 不应判 L3，否则与 degrade_ladder 的自动恢复形成 MRM 拉锯
                 * （车停稳却反复 降级→恢复→降级）。仅当车仍在运动而 2s 无
                 * cmd 时才视为真实失联 → L3。
                 *
                 * 超时从 1s 提升到 2s（2026-08-03）：与 flowsim 的
                 * CONTROL_STALE_TIMEOUT_US(2s) 保持一致。高负载下消息总线
                 * 丢包率高，1s 超时导致 MRM 拉锯循环：
                 *   车停→L3清除→起步加速→speed>0.5→raw_cmd 1s stale→L3
                 *   →刹车→停稳→3s 恢复→起步→... 无限循环车速恒为 0 */
                double cur_speed = 0.0;
                bool has_state = false;
                pthread_mutex_lock(&g.state_mutex);
                cur_speed = g.latest_state.speed;
                has_state = g.has_state;
                pthread_mutex_unlock(&g.state_mutex);
                bool moving = has_state && cur_speed > 0.5;
                if (moving && (now_us - last_msg_us > 2000000ULL)) {
                    /* 行驶中数据超时 > 2s → L3 立即停 */
                    degrade_set_level(DEGRADE_L3, DEGRADE_REASON_HEARTBEAT);
                }
            }
            co_await sleep_us(5000);  /* 5ms 轮询节拍（消息驱动 → 固定周期） */
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

        /* §11.2 降级触发 */
        {
            if (!has_state) {
                /* 传感器融合丢失 → L1 降级 */
                degrade_set_level(DEGRADE_L1, DEGRADE_REASON_FUSION_TO);
            }
        }

        /* ── 机动窗口（掉头/倒车）──
         * control 巡航钳位上限 0.16rad，只有 maneuver_mode（掉头/倒车轨迹）
         * 会发出 |steer|>0.30 或倒挡。此时 safety 的巡航级钳位/TTC 规则会
         * 直接杀死掉头：low_speed_steer=0.18 → 转弯半径 R=L/tan(0.18)≈15m，
         * 14m 路宽物理上掉不过来；施工区/停车被近场 TTC 当前车全刹 → v=0
         * → yaw_rate=v/L·tan(δ)=0 转不动 → 车横漂进对向车道定格"逆行"
         * （2026-08-03 死锁现场）。机动窗口内放行满舵、豁免巡航级 TTC，
         * 只保留 <2.0m 硬碰撞保护——这是 planning 掉头轨迹的执行前提。 */
        const bool maneuver = (cmd.gear == GEAR_REVERSE) ||
                              std::fabs(cmd.steer) > 0.30 ||
                              cmd.mode.find("MANEUVER") != std::string::npos;

        /* 倒挡时油门为负（flowsim 负油门=倒车驱动），钳位下界随挡位放开 */
        const double thr_lo = (cmd.gear == GEAR_REVERSE) ? -params_.max_throttle : 0.0;
        set_changed(cmd.throttle, clamp(cmd.throttle, thr_lo, params_.max_throttle));
        set_changed(cmd.brake, clamp(cmd.brake, 0.0, params_.max_brake));
        double steer_limit = maneuver ? 0.62
                           : (has_state && state.speed < 3.0) ? params_.low_speed_steer
                                                              : params_.max_steer;
        set_changed(cmd.steer, clamp(cmd.steer, -steer_limit, steer_limit));

        if (has_state && maneuver) {
            /* 硬碰撞保护：沿运动方向 2m 内有障碍才全刹，其余放行。
             * 不复用 min_vehicle_ttc——它无候选时返回 dx=0，会被误判
             * "0m 处有障碍" → 恒全刹（2026-08-03 掉头两次死于此）。
             * 方向性：前进只看前方障碍，倒车只看后方——倒车逃离前方
             * 障碍是 Phase 0 腾挪的合法动作，不得拦截。 */
            const bool backing = (cmd.gear == GEAR_REVERSE);
            const double ch = std::cos(state.heading), sh = std::sin(state.heading);
            for (int i = 0; i < kMaxObs; ++i) {
                if (!state.obs_valid[i]) continue;
                /* 施工区是 planning 生成机动轨迹的边界约束，不是动态碰撞体。
                 * Phase 0 的语义正是从施工前缘倒车腾挪；若把墙体感知点纳入
                 * 倒车硬门，墙在车后 4.8m 内时会永久 brake=1，机动直到 40s
                 * TIMEOUT。真实车辆/行人仍保留双向硬碰撞保护。 */
                if (std::strcmp(state.obs_type[i], "construction") == 0) continue;
                const double dx = state.obs_x[i] - state.x;
                const double dy = state.obs_y[i] - state.y;
                /* 车体系投影：掉头转过 90°/180° 后世界系 +x 早已不是"前方" */
                const double lon = dx * ch + dy * sh;
                const double lat = -dx * sh + dy * ch;
                const double ahead = backing ? -lon : lon;
                /* 1.2m 净距 + 3.6m 偏置（半车长 2.4 + 半障碍 1.2，中心距） */
                if (ahead > 0.0 && ahead < 3.6 + 1.2 && std::fabs(lat) < 1.4) {
                    set_changed(cmd.throttle, 0.0);
                    set_changed(cmd.brake, 1.0);
                    break;
                }
            }
        }

        if (has_state && !maneuver) {
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
                /* §11.2 TTC 过低 → L2 MRM 降级 */
                if (ttc < 1.5) {
                    degrade_set_level(DEGRADE_L2, DEGRADE_REASON_COLLISION);
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

            /* 死锁恢复职责归属 control 节点（SPEED_ZERO_RECOVERY / STUCK_RECOVER）。
             *
             * safety_control 此前有独立的 5s low-speed deadlock recovery，与 control
             * 的 SPEED_ZERO_RECOVERY 同时触发后互相矛盾：control 设 throttle=0.15/brake=0，
             * safety 又覆写为 throttle=0.20/brake=0.30 → ego 既前进又刹车，无法移动。
             * 更严重的是 safety 不订阅红绿灯 topic，红灯停车 5s 后会强制蠕行闯红灯。
             *
             * 职责边界：safety 是纯安全闸门（clamp + 碰撞制动覆写），不发起恢复。
             * control 负责所有死锁恢复（已含 target_speed 检查，红灯时不触发）。 */
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
        bin.gear           = (int8_t)cmd.gear;

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
        /* 无条件发布：QoS depth+drop_oldest 兜底，确保最新指令必达。
         * 原反压跳过（topic_is_full 时丢弃本帧）配合 control/cmd 的 depth=1
         * QoS，会在 dispatch 瞬时抖动时持续跳过 → flowsim 断流 → FSAFE/MRM
         * 永久停车（2026-07-31 复现：safety #61 后 flowsim cb 永停）。 */
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

/* ── TaskBase 包装器（宏生成） — 必须在 safety_init 前展开 ─────── */
EXPORT_COROUTINE_TASK(SafetyControlTask, safety_control)

const char* s_inputs[] = {"control/raw_cmd", TOPIC_FUSION_LOCALIZATION, TOPIC_PERCEPTION_OBSTACLES, nullptr};
const char* s_outputs[] = {"control/cmd", nullptr};
extern NodePlugin s_plugin;

int safety_init(MessageBus* bus, Transport* transport, DiscoveryManager* discovery,
                Scheduler* scheduler, const char* params_json) {
    g.bus = bus;
    g.transport = transport;
    g.discovery = discovery;
    g.scheduler = scheduler;
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

    /* §n: 注册 Req/Reply 服务 — 查询安全状态 */
    message_bus_register_service(bus, "safety/status", [](const Message* req, Message* rep, void*) {
        (void)req;
        char buf[128];
        int n = snprintf(buf, sizeof(buf),
            "{\"speed\":%.1f,\"has_state\":%s}",
            g.latest_state.speed,
            g.has_state ? "true" : "false");
        if (n > 0 && (size_t)n < sizeof(buf) && (size_t)n <= sizeof(rep->data)) {
            memcpy(rep->data, buf, (size_t)n);
            rep->data_size = (uint32_t)n;
        }
    }, nullptr);

    discovery_advertise(discovery, "control/raw_cmd", CONTROL_RAW_TYPE_ID, CAP_SUBSCRIBER, 0);
    discovery_advertise(discovery, TOPIC_FUSION_LOCALIZATION, 0xF0ED10C0u, CAP_SUBSCRIBER, 0);
    discovery_advertise(discovery, TOPIC_PERCEPTION_OBSTACLES, OBSTACLELIST_TYPE_ID, CAP_SUBSCRIBER, 0);
    discovery_advertise(discovery, "control/cmd", CONTROL_CMD_TYPE_ID, CAP_PUBLISHER, 100.0);

    /* 创建 TaskBase 包装器（托管模式） */
    TaskConfig tcfg = {};
    snprintf(tcfg.name, sizeof(tcfg.name), "safety_control");
    tcfg.priority = TASK_PRIORITY_NORMAL;
    g.task_wrapper = safety_control_create(&tcfg, bus);
    if (!g.task_wrapper) {
        LOG_ERROR("safety_control", "safety_control_create failed");
        return -1;
    }
    g.task_wrapper->impl->set_params(transport, g.params);
    s_plugin.taskbase = safety_control_get_base(g.task_wrapper);

    LOG_INFO("safety_control", "initialized (FlowCoro, max_thr=%.2f max_steer=%.2f, managed mode)",
             g.params.max_throttle, g.params.max_steer);
    return 0;
}

int safety_start() {
    if (!g.task_wrapper) return -1;
    /* 托管模式：注册到调度器 + 派生工作线程 + 设置 choreo trigger */
    int rc = node_start_managed(&s_plugin, g.scheduler);
    if (rc != 0) {
        LOG_WARN("safety_control", "node_start_managed failed: %d", rc);
    }
    node_announce_self(g.transport, &s_plugin);
    LOG_INFO("safety_control", "started (managed mode)");
    return 0;
}

void safety_stop() {
    if (g.task_wrapper) {
        safety_control_stop(&g.task_wrapper->base);
    }
}

void safety_cleanup() {
    if (g.task_wrapper) {
        safety_control_destroy(g.task_wrapper);
        g.task_wrapper = nullptr;
    }
    s_plugin.taskbase = nullptr;
    LOG_INFO("safety_control", "cleanup done");
}

int safety_health() {
    return g.task_wrapper ? 0 : -1;
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
    nullptr,  /* taskbase: 在 init() 中通过 safety_control_create 设置 */
};

} // namespace

extern "C" NodePlugin* node_get_plugin(void) { return &s_plugin; }