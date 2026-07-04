/**
 * e2e_demo.c — FlowEngine 全组件端到端演示
 *
 * 串联所有组件:
 *   序列化 → 调度器 → 状态机 → 发现 → 融合 → 传输 → 日志
 *
 * Pipeline:
 *   PerceptionTask (CRITICAL, 10Hz) ─── lidar/gps ──→ bus/transport
 *   FusionTask    (HIGH, choreo)      ─── 对齐融合 ──→ bus/transport
 *   ControlTask   (NORMAL, choreo)    ─── 决策输出 ──→ bus/transport
 *   MonitorTask   (NORMAL, 1Hz)       ─── 统计打印 ──→ stdout
 *
 * 编译: 由 CMakeLists.txt 自动处理
 * 运行: ./build/bin/flow_e2e [duration_sec=10]
 */

#include "message_bus.h"
#include "serializer.h"
#include "scheduler.h"
#include "state_machine.h"
#include "discovery.h"
#include "fusion.h"
#include "transport.h"
#include "logger.h"
#include "flow_registry.h"
#include "param_registry.h"
#include "adas_msgs_gen.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <math.h>
#include <inttypes.h>
#include "mcap_writer.h"

/* MCAP channel IDs (set after registration) */
static uint16_t mcap_ch_lidar   = 0;
static uint16_t mcap_ch_camera  = 0;
static uint16_t mcap_ch_gps     = 0;
static uint16_t mcap_ch_fusion  = 0;
static uint16_t mcap_ch_control = 0;
static uint16_t mcap_ch_vehicle = 0;

/* Camera frame reuses LidarFrame struct for demo simplicity */
#define CAMERAFRAME_TYPE_ID  0x4A1B0C2Du

/* ══════════════════════════════════════════════════════════ */
/* 全局状态                                                    */
/* ══════════════════════════════════════════════════════════ */

static volatile bool g_running = true;
static MessageBus*       g_bus       = NULL;
static DiscoveryManager* g_discovery = NULL;
static Transport*        g_transport = NULL;
static Scheduler*        g_scheduler = NULL;
static int               g_fusion_tid = -1;   /**< for monitor latency reporting */

static void sig_handler(int sig) {
    (void)sig;
    LOG_INFO("e2e", "signal %d received, shutting down...", sig);
    g_running = false;
}

/* ══════════════════════════════════════════════════════════ */
/* 任务1: 感知 — 发布传感器数据 (CRITICAL, 10Hz)              */
/* ══════════════════════════════════════════════════════════ */

typedef struct {
    TaskBase    base;
    int         tid;
    uint32_t    frame_id;
} PerceptionTask;

static int perception_init(TaskBase* base) {
    PerceptionTask* pt = (PerceptionTask*)base;
    pt->frame_id = 0;

    /* ── 发现: 广播 topic ── */
    discovery_advertise(g_discovery, "sensor/lidar", LIDARFRAME_TYPE_ID,
                        CAP_PUBLISHER, 20.0);
    discovery_advertise(g_discovery, "sensor/gps", GPSDATA_TYPE_ID,
                        CAP_PUBLISHER, 10.0);
    discovery_advertise(g_discovery, "sensor/camera", LIDARFRAME_TYPE_ID,
                        CAP_PUBLISHER, 20.0);

    /* ── 传输: 广告 topic ── */
    transport_advertise(g_transport, "sensor/lidar", LIDARFRAME_TYPE_ID);
    transport_advertise(g_transport, "sensor/gps", GPSDATA_TYPE_ID);
    transport_advertise(g_transport, "sensor/camera", LIDARFRAME_TYPE_ID);

    LOG_INFO("perception", "initialized (CRITICAL, 20Hz LiDAR + 20Hz Camera + 10Hz GPS)");
    return 0;
}

static int perception_execute(TaskBase* base) {
    PerceptionTask* pt = (PerceptionTask*)base;
    RateControl* rc = scheduler_get_rate_control(g_scheduler, pt->tid);

    while (g_running && !base->should_stop) {
        if (!rate_control_acquire(rc)) { usleep(1000); continue; }

        /* ── LiDAR @20Hz ── */
        LidarFrame lidar = {
            .x = 50.0f - (float)pt->frame_id * 0.05f,
            .y = 0.0f, .z = 0.0f, .intensity = 0.85f,
            .point_count = 64000 + pt->frame_id,
            .frame_id = pt->frame_id
        };
        Message lmsg;
        msg_init_typed(&lmsg, "sensor/lidar", "perception",
                       LIDARFRAME_TYPE_ID, LIDARFRAME_SCHEMA_VERSION,
                       &lidar, sizeof(lidar));
        transport_publish(g_transport, "sensor/lidar", lmsg.data, lmsg.data_size);

        /* ── Camera @20Hz ── */
        LidarFrame cam = {  /* reuse LidarFrame for demo — simulates bounding boxes */
            .x = 48.0f - (float)pt->frame_id * 0.05f,
            .y = 12.0f, .z = 5.0f, .intensity = 0.72f,
            .point_count = 1280 + pt->frame_id % 20,
            .frame_id = pt->frame_id
        };
        Message cmsg;
        msg_init_typed(&cmsg, "sensor/camera", "perception",
                       CAMERAFRAME_TYPE_ID, 1,
                       &cam, sizeof(cam));
        transport_publish(g_transport, "sensor/camera", cmsg.data, cmsg.data_size);

        /* ── GPS @10Hz (every other cycle) ── */
        if (pt->frame_id % 2 == 0) {
            GpsData gps = {
                .latitude = 39.904 + (double)pt->frame_id * 0.00001,
                .longitude = 116.407 + (double)pt->frame_id * 0.00001,
                .speed_mps = 33.0f, .heading_deg = 0.0f, .accuracy_m = 0.5f
            };
            Message gmsg;
            msg_init_typed(&gmsg, "sensor/gps", "perception",
                           GPSDATA_TYPE_ID, GPSDATA_SCHEMA_VERSION,
                           &gps, sizeof(gps));
            transport_publish(g_transport, "sensor/gps", gmsg.data, gmsg.data_size);
            LOG_DEBUG("perception", "GPS: lat=%.6f lon=%.6f",
                      gps.latitude, gps.longitude);
        }

        pt->frame_id++;
    }

    if (statem_current(&base->sm) == SM_STATE_RUNNING)
        statem_send_event(&base->sm, SM_EVENT_STOP, base);
    LOG_INFO("perception", "stopped (%u frames)", pt->frame_id);
    return 0;
}

static void perception_cleanup(TaskBase* base) {
    (void)base;
}

static TaskInterface g_perception_vtable = {
    .initialize = perception_init,
    .execute    = perception_execute,
    .cleanup    = perception_cleanup,
    .health_check = NULL,
    .on_message = NULL,
};

/* ══════════════════════════════════════════════════════════ */
/* 任务2: 融合 — 时间对齐 LiDAR+GPS (HIGH, choreo)            */
/* ══════════════════════════════════════════════════════════ */

typedef struct {
    TaskBase      base;
    int           tid;
    MessageBuffer* lidar_buf;
    MessageBuffer* gps_buf;
    uint32_t      fused_count;
} FusionTask;

static void fusion_on_lidar(const Message* msg, void* user_data) {
    FusionTask* ft = (FusionTask*)user_data;
    message_buffer_push(ft->lidar_buf, msg);
}

static void fusion_on_gps(const Message* msg, void* user_data) {
    FusionTask* ft = (FusionTask*)user_data;
    message_buffer_push(ft->gps_buf, msg);
}

static int fusion_init(TaskBase* base) {
    FusionTask* ft = (FusionTask*)base;

    ft->lidar_buf = message_buffer_create("sensor/lidar", LIDARFRAME_TYPE_ID, 32, 5000000);
    ft->gps_buf   = message_buffer_create("sensor/gps",   GPSDATA_TYPE_ID,    16, 5000000);

    /* ── 订阅原始数据 ── */
    transport_subscribe(g_transport, "sensor/lidar", fusion_on_lidar, ft);
    transport_subscribe(g_transport, "sensor/gps",   fusion_on_gps,   ft);

    /* ── 发现: 广告融合输出 ── */
    discovery_advertise(g_discovery, "fusion/localization", 0xF0ED10C0u,
                        CAP_FUSION | CAP_PUBLISHER, 10.0);
    transport_advertise(g_transport, "fusion/localization", 0xF0ED10C0u);

    /* ── 状态机: 驾驶模式 NA→ACC (条件满足) ── */
    ReflectiveStateMachine mode_sm;
    statem_init(&mode_sm, SM_TABLE_MODE_SWITCHING, SM_MODE_NA, "driving_mode");
    mode_sm.trace_enabled = true;
    statem_send_event(&mode_sm, SM_EVT_CONDITIONS_MET, base);
    LOG_INFO("fusion", "driving mode: %s (%s)",
             statem_mode_name(statem_current(&mode_sm)),
             statem_sub_state_name(SM_SUB_READY));

    /* ── Choreo: 被 LiDAR 消息触发 ── */
    scheduler_choreo_trigger_on(g_scheduler, ft->tid, "sensor/lidar");

    LOG_INFO("fusion", "initialized (HIGH, choreo, TIME_ALIGNED 50ms)");
    return 0;
}

static int fusion_execute(TaskBase* base) {
    FusionTask* ft = (FusionTask*)base;

    while (g_running && !base->should_stop) {
        /* ── Choreo wait: 阻塞直到 LiDAR 消息到达 ── */
        int ret = scheduler_choreo_wait(g_scheduler, ft->tid, 500000);
        if (ret == -2) break;  /* stopped */
        if (ret == -1) continue; /* timeout, no new data */

        /* ── 融合: 时间对齐查找 ── */
        const Message* lidar_msg = message_buffer_latest(ft->lidar_buf);
        if (!lidar_msg) continue;

        uint64_t ref_ts = lidar_msg->timestamp_us;
        const Message* gps_msg = message_buffer_find_nearest(
            ft->gps_buf, ref_ts, 50000); /* 50ms window */

        /* ── 序列化: 类型安全访问 ── */
        const LidarFrame* lidar = (const LidarFrame*)
            _msg_cast_impl(lidar_msg, LIDARFRAME_TYPE_ID, sizeof(LidarFrame), "LidarFrame");
        const GpsData* gps = gps_msg ? (const GpsData*)
            _msg_cast_impl(gps_msg, GPSDATA_TYPE_ID, sizeof(GpsData), "GpsData") : NULL;

        if (!lidar) continue;

        /* ── 输出融合结果 ── */
        char fused[256];
        int off = 0;
        off += snprintf(fused + off, sizeof(fused) - (size_t)off,
                        "pos=(%.1f,%.1f)", lidar->x, lidar->y);
        if (gps) {
            off += snprintf(fused + off, sizeof(fused) - (size_t)off,
                            " gps=(%.6f,%.6f) speed=%.1f dt=%" PRIu64 "us",
                            gps->latitude, gps->longitude, gps->speed_mps,
                            gps_msg ? (gps_msg->timestamp_us > ref_ts ?
                             gps_msg->timestamp_us - ref_ts :
                             ref_ts - gps_msg->timestamp_us) : 0ULL);
        }

        ft->fused_count++;

        /* ── 发布融合结果 ── */
        Message out_msg;
        msg_init_typed(&out_msg, "fusion/localization", "fusion",
                       0xF0ED10C0u, 1, fused, (uint32_t)strlen(fused) + 1);
        out_msg.timestamp_us = ref_ts;
        transport_publish(g_transport, "fusion/localization",
                          out_msg.data, out_msg.data_size);

        LOG_INFO("fusion", "#%u %s", ft->fused_count, fused);

        /* ── Latency tracking ── */
        LatencyTracker* lt = scheduler_get_latency(g_scheduler, ft->tid);
        if (lt) {
            uint64_t now_us = (uint64_t)(gps_msg ? gps_msg->timestamp_us : 0);
            if (now_us > 0) {
                struct timespec ts;
                clock_gettime(CLOCK_MONOTONIC, &ts);
                uint64_t wall = (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;
                latency_tracker_record(lt, wall - now_us);
            }
        }
    }

    if (statem_current(&base->sm) == SM_STATE_RUNNING)
        statem_send_event(&base->sm, SM_EVENT_STOP, base);
    LOG_INFO("fusion", "stopped (%u fused frames)", ft->fused_count);
    return 0;
}

static void fusion_cleanup(TaskBase* base) {
    FusionTask* ft = (FusionTask*)base;
    message_buffer_destroy(ft->lidar_buf);
    message_buffer_destroy(ft->gps_buf);
}

static TaskInterface g_fusion_vtable = {
    .initialize = fusion_init,
    .execute    = fusion_execute,
    .cleanup    = fusion_cleanup,
    .health_check = NULL,
    .on_message = NULL,
};

/* ══════════════════════════════════════════════════════════ */
/* 车辆动力学模型 (简单质点)                                     */
/* ══════════════════════════════════════════════════════════ */

typedef struct {
    double  x;               /**< 纵向位置 (m, 世界系) */
    double  y;               /**< 横向位置 (m, 左正) */
    double  speed;           /**< 当前速度 (m/s) */
    double  target_speed;    /**< 目标速度 (m/s), 来自规划/ACC */
    double  throttle;        /**< 油门 0..1 */
    double  brake;           /**< 刹车 0..1 */
    double  steer;           /**< 前轮转向角 (rad) */
    double  heading;         /**< 航向角 (rad) */
    double  lane_target;     /**< 目标横向位置 (m), 变道用 */
    double  wheelbase;       /**< 轴距 (m) */
    double  mass;            /**< 质量 (kg) */
    double  drag_coeff;      /**< 空气阻力系数 */
} VehicleModel;

static VehicleModel g_vehicle = {
    .x = 0, .y = 0, .speed = 5.0, .target_speed = 10.0,
    .throttle = 0.3, .brake = 0, .steer = 0, .heading = 0,
    .lane_target = 0, .wheelbase = 2.7,
    .mass = 1500.0, .drag_coeff = 0.3
};

/* ── 3D 场景: 障碍物 (真实运动学，供可视化与 ACC 使用) ── */
typedef struct {
    int         id;
    const char* type;        /**< "car" | "pedestrian" */
    double      x, y;        /**< 世界系位置 (m) */
    double      vx, vy;      /**< 速度 (m/s) */
    double      len, wid;    /**< 包围盒 (m) */
} SimObstacle;

#define SIM_OBSTACLE_COUNT 3
static SimObstacle g_obstacles[SIM_OBSTACLE_COUNT] = {
    { 0, "car",         35.0,  0.0,  6.0, 0.0, 4.6, 2.0 }, /* 同车道前车 */
    { 1, "car",         95.0, -3.5, -9.0, 0.0, 4.6, 2.0 }, /* 对向来车 */
    { 2, "pedestrian",  55.0,  8.0,  0.0, 0.6, 0.6, 0.6 }, /* 过街行人 */
};

/* 障碍物运动学 + 循环边界，使场景持续有内容 */
static void obstacles_tick(double dt) {
    for (int i = 0; i < SIM_OBSTACLE_COUNT; i++) {
        SimObstacle* o = &g_obstacles[i];
        o->x += o->vx * dt;
        o->y += o->vy * dt;
        if (strcmp(o->type, "pedestrian") == 0) {
            /* 行人在人行横道上来回走 (y ∈ [-8, 8]) */
            if (o->y >  8.0) { o->y =  8.0; o->vy = -fabs(o->vy); }
            if (o->y < -8.0) { o->y = -8.0; o->vy =  fabs(o->vy); }
        }
        /* 相对自车过远则回收到前方，保持画面始终有障碍物 */
        double rel = o->x - g_vehicle.x;
        if (rel < -30.0) o->x = g_vehicle.x + 80.0 + (double)i * 5.0;
        if (rel >  140.0) o->x = g_vehicle.x + 30.0;
    }
}

/* 与同车道前车的纵向间距 (m)，无则返回大值 */
static double lead_gap(void) {
    double best = 1e9;
    for (int i = 0; i < SIM_OBSTACLE_COUNT; i++) {
        SimObstacle* o = &g_obstacles[i];
        if (o->vx < 0) continue;                 /* 忽略对向车 */
        if (fabs(o->y - g_vehicle.y) > 2.0) continue; /* 不在本车道 */
        double gap = o->x - g_vehicle.x;
        if (gap > 0 && gap < best) best = gap;
    }
    return best;
}

static void vehicle_tick(double dt) {
    /* 驱动力 = 油门 × max_force - 刹车 × max_brake */
    double drive_force  = g_vehicle.throttle * 5000.0;   /* N */
    double brake_force  = g_vehicle.brake    * 8000.0;   /* N */
    double drag_force   = g_vehicle.drag_coeff * g_vehicle.speed * g_vehicle.speed;
    double net_force    = drive_force - brake_force - drag_force;
    double accel        = net_force / g_vehicle.mass;

    g_vehicle.speed += accel * dt;
    if (g_vehicle.speed < 0) g_vehicle.speed = 0;

    /* ── 横向: 简单车道保持/变道控制 → 转向角 ── */
    double y_err   = g_vehicle.lane_target - g_vehicle.y;
    double psi_des = 0.6 * y_err;                 /* 期望航向 (P 控制) */
    double steer   = 1.2 * (psi_des - g_vehicle.heading);
    if (steer >  0.5) steer =  0.5;               /* ±0.5 rad 限幅 */
    if (steer < -0.5) steer = -0.5;
    g_vehicle.steer = steer;

    /* ── Bicycle 模型: 航向随转向角与速度演化 ── */
    g_vehicle.heading += (g_vehicle.speed / g_vehicle.wheelbase)
                         * tan(g_vehicle.steer) * dt;

    g_vehicle.x += g_vehicle.speed * dt * cos(g_vehicle.heading);
    g_vehicle.y += g_vehicle.speed * dt * sin(g_vehicle.heading);

    obstacles_tick(dt);
}

/* ══════════════════════════════════════════════════════════ */
/* 任务3: PID 纵向控制 (NORMAL, choreo)                        */
/*   输入: fusion/localization (当前速度)                       */
/*   输入: planning/trajectory (目标速度)                        */
/*   输出: control/cmd (油门/刹车/转向)                          */
/*   反馈: control/cmd → vehicle_tick() → 闭环                  */
/* ══════════════════════════════════════════════════════════ */

typedef struct {
    TaskBase  base;
    int       tid;
    int       cycle;
    /* PID 状态 */
    double    kp, ki, kd;
    double    integral;
    double    prev_error;
    double    current_speed;
    double    target_speed;
} ControlTask;

static void control_on_fusion(const Message* msg, void* user_data) {
    ControlTask* ct = (ControlTask*)user_data;
    const char* data = (const char*)msg->data;
    if (!data) return;
    /* 从 fusion 输出中提取当前速度 */
    if (strstr(data, "speed="))
        sscanf(data, "speed=%lf", &ct->current_speed);
}

static void control_on_trajectory(const Message* msg, void* user_data) {
    ControlTask* ct = (ControlTask*)user_data;
    const char* data = (const char*)msg->data;
    if (!data) return;
    /* 从规划输出中提取目标速度 */
    if (strstr(data, "speed="))
        sscanf(data, "speed=%lf", &ct->target_speed);
}

static int control_init(TaskBase* base) {
    ControlTask* ct = (ControlTask*)base;

    /* PID 参数: Kp=800, Ki=50, Kd=100 (速度控制器) */
    ct->kp = 800.0; ct->ki = 50.0; ct->kd = 100.0;
    ct->integral = 0; ct->prev_error = 0;
    ct->current_speed = g_vehicle.speed;
    ct->target_speed  = g_vehicle.target_speed;

    transport_subscribe(g_transport, "fusion/localization", control_on_fusion, ct);
    transport_subscribe(g_transport, "planning/trajectory", control_on_trajectory, ct);

    discovery_advertise(g_discovery, "fusion/localization", 0xF0ED10C0u,
                        CAP_SUBSCRIBER, 0);
    discovery_advertise(g_discovery, "planning/trajectory", 0x3A7B1C2Du,
                        CAP_SUBSCRIBER, 0);
    discovery_advertise(g_discovery, "control/cmd", 0x2D95C6D2u,
                        CAP_PUBLISHER, 100.0);

    transport_advertise(g_transport, "control/cmd", 0x2D95C6D2u);

    scheduler_choreo_trigger_on(g_scheduler, ct->tid, "fusion/localization");

    LOG_INFO("control", "initialized (NORMAL, choreo, PID: kp=%.0f ki=%.0f kd=%.0f)",
             ct->kp, ct->ki, ct->kd);
    return 0;
}

static int control_execute(TaskBase* base) {
    ControlTask* ct = (ControlTask*)base;

    while (g_running && !base->should_stop) {
        int ret = scheduler_choreo_wait(g_scheduler, ct->tid, 500000);
        if (ret == -2) break;

        ct->cycle++;

        /* ── ACC: 依据与同车道前车的间距动态限制目标速度 ── */
        double gap       = lead_gap();
        double safe_gap  = 8.0 + ct->current_speed * 1.2;   /* 距离策略 */
        double acc_target = ct->target_speed;
        if (gap < safe_gap) {
            double ratio = gap / safe_gap;
            if (ratio < 0) ratio = 0;
            acc_target = ct->target_speed * ratio;           /* 越近目标越低 */
        }

        /* ── 变道演示: 行驶一段距离后切到左车道再回正 ── */
        g_vehicle.lane_target = (g_vehicle.x > 120.0 && g_vehicle.x < 180.0)
                                ? 3.5 : 0.0;

        /* ── PID 计算 (目标为 ACC 限速后的值) ── */
        double error = acc_target - ct->current_speed;
        ct->integral += error * 0.05;  /* dt ≈ 50ms */
        if (ct->integral > 500)  ct->integral = 500;   /* 积分饱和 */
        if (ct->integral < -200) ct->integral = -200;

        double derivative = (error - ct->prev_error) / 0.05;
        double output = ct->kp * error + ct->ki * ct->integral + ct->kd * derivative;

        /* 输出拆分: >0 → 油门, <0 → 刹车 */
        double throttle = 0, brake = 0;
        const char* mode;
        if (output > 0) {
            throttle = output / 5000.0;
            if (throttle > 1.0) throttle = 1.0;
            brake = 0;
            mode = (error < 1.0) ? "⏺ HOLD" : "🟢 ACCEL";
        } else {
            throttle = 0;
            brake = (-output) / 8000.0;
            if (brake > 1.0) brake = 1.0;
            mode = "🔴 BRAKE";
        }

        /* ── 更新车辆状态 ── */
        g_vehicle.throttle = throttle;
        g_vehicle.brake    = brake;
        vehicle_tick(0.05);  /* 50ms 步长 */

        /* ── 发布控制指令 ── */
        char cmd[128];
        snprintf(cmd, sizeof(cmd),
                 "throttle=%.2f brake=%.2f steer=%.2f "
                 "speed=%.1f target=%.1f error=%.1f mode=%s",
                 throttle, brake, g_vehicle.steer,
                 g_vehicle.speed, acc_target, error, mode);
        Message cmsg;
        msg_init_typed(&cmsg, "control/cmd", "control",
                       0x2D95C6D2u, 1, cmd, (uint32_t)(strlen(cmd) + 1));
        transport_publish(g_transport, "control/cmd", cmsg.data, cmsg.data_size);
        if (mcap_ch_vehicle) mcap_writer_write_json(mcap_writer_global(), mcap_ch_vehicle, 0, "{\"speed\":%.1f,\"target_speed\":%.1f,\"throttle\":%.3f,\"brake\":%.3f,\"x\":%.1f,\"y\":%.1f,\"error\":%.1f}", g_vehicle.speed, g_vehicle.target_speed, g_vehicle.throttle, g_vehicle.brake, g_vehicle.x, g_vehicle.y, g_vehicle.target_speed - g_vehicle.speed);

        /* ── 更新当前速度（闭环: 车辆状态 → 感知 → 融合 → 这里） ── */
        ct->current_speed = g_vehicle.speed;
        ct->prev_error    = error;

        LOG_INFO("control", "#%d spd=%.1f→%.1f err=%.1f thr=%.2f brk=%.2f %s",
                 ct->cycle, ct->current_speed, ct->target_speed,
                 error, throttle, brake, mode);
    }

    if (statem_current(&base->sm) == SM_STATE_RUNNING)
        statem_send_event(&base->sm, SM_EVENT_STOP, base);
    LOG_INFO("control", "stopped (%d cycles, final speed=%.1f m/s)",
             ct->cycle, g_vehicle.speed);
    return 0;
}

static void control_cleanup(TaskBase* base) {
    (void)base;
}

static TaskInterface g_control_vtable = {
    .initialize = control_init,
    .execute    = control_execute,
    .cleanup    = control_cleanup,
    .health_check = NULL,
    .on_message = NULL,
};

/* ══════════════════════════════════════════════════════════ */
/* PlanningTask: 轨迹规划 (NORMAL, choreo)                      */
/*   订阅 fusion/localization → 发布 planning/trajectory        */
/* ══════════════════════════════════════════════════════════ */

typedef struct {
    TaskBase  base;
    int       tid;
    int       plan_count;
} PlanningTask;

static void planning_on_fusion(const Message* msg, void* user_data) {
    PlanningTask* plt = (PlanningTask*)user_data;
    const char* data = (const char*)msg->data;
    if (!data) return;

    /* Simple trajectory: pass-through with speed limit */
    float x = 0, speed = 33.0f;
    if (strstr(data, "pos=(")) sscanf(data, "pos=(%f", &x);
    if (strstr(data, "speed=")) sscanf(data, "speed=%f", &speed);

    char traj[256];
    float target_speed = speed > 40.0f ? 40.0f : speed;
    snprintf(traj, sizeof(traj), "traj=(%.1f,0.0) speed=%.1f lane=center",
             x + 2.0f, target_speed);

    Message pmsg;
    msg_init_typed(&pmsg, "planning/trajectory", "planning",
                   0x3A7B1C2Du, 1, traj, (uint32_t)(strlen(traj) + 1));
    transport_publish(g_transport, "planning/trajectory", pmsg.data, pmsg.data_size);
    plt->plan_count++;
}

static int planning_init(TaskBase* base) {
    PlanningTask* pt = (PlanningTask*)base;

    transport_subscribe(g_transport, "fusion/localization", planning_on_fusion, pt);
    transport_advertise(g_transport, "planning/trajectory", 0x3A7B1C2Du);

    discovery_advertise(g_discovery, "fusion/localization", 0xF0ED10C0u,
                        CAP_SUBSCRIBER, 0);
    discovery_advertise(g_discovery, "planning/trajectory", 0x3A7B1C2Du,
                        CAP_PUBLISHER, 10.0);

    scheduler_choreo_trigger_on(g_scheduler, pt->tid, "fusion/localization");

    LOG_INFO("planning", "initialized (NORMAL, choreo)");
    return 0;
}

static int planning_execute(TaskBase* base) {
    PlanningTask* pt = (PlanningTask*)base;
    RateControl* rc = scheduler_get_rate_control(g_scheduler, pt->tid);

    while (g_running && !base->should_stop) {
        if (!rate_control_acquire(rc)) { usleep(5000); continue; }
        /* planning_on_fusion callback does the actual work */
        task_update_heartbeat(base);
    }

    LOG_INFO("planning", "stopped (%d trajectories)", pt->plan_count);
    return 0;
}

static void planning_cleanup(TaskBase* base) {
    (void)base;
}

static TaskInterface g_planning_vtable = {
    .initialize = planning_init,
    .execute    = planning_execute,
    .cleanup    = planning_cleanup,
    .health_check = NULL,
    .on_message = NULL,
};

/* ══════════════════════════════════════════════════════════ */
/* 任务4: 监控 — 定期打印统计 (NORMAL, 1Hz)                   */
/* ══════════════════════════════════════════════════════════ */

typedef struct {
    TaskBase  base;
    int       tid;
} MonitorTask;

static int monitor_init(TaskBase* base) {
    statem_send_event(&base->sm, SM_EVENT_START, base);
    LOG_INFO("monitor", "initialized (10Hz stats reporter)");
    return 0;
}

static int monitor_execute(TaskBase* base) {
    RateControl* rc = scheduler_get_rate_control(g_scheduler, ((MonitorTask*)base)->tid);

    while (g_running && !base->should_stop) {
        if (!rate_control_acquire(rc)) { usleep(100000); continue; }

        /* ── Bus stats ── */
        uint64_t pub, del, drop;
        message_bus_get_stats(g_bus, &pub, &del, &drop);

        /* ── Transport stats ── */
        TransportStats ts;
        transport_get_stats(g_transport, &ts);

        /* ── Scheduler stats ── */
        int task_count = scheduler_task_count(g_scheduler);

        /* ── Discovery stats ── */
        const TopologyGraph* topo = discovery_get_topology(g_discovery);

        printf("\n┌─── Monitor @ %lu ──────────────────────────┐\n",
               (unsigned long)time(NULL));
        printf("│ Bus:     pub=%lu del=%lu drop=%lu\n",
               (unsigned long)pub, (unsigned long)del, (unsigned long)drop);
        printf("│ Net:     local=%lu/%lu remote=%lu/%lu\n",
               (unsigned long)ts.local_published, (unsigned long)ts.local_delivered,
               (unsigned long)ts.remote_published, (unsigned long)ts.remote_delivered);
        printf("│ Tasks:   %d  Topology: %u nodes\n",
               task_count, topo ? topo->node_count : 0);
        printf("│ Routes:  IPC=%d TCP=%d\n",
               transport_ipc_channel_count(g_transport),
               transport_remote_peer_count(g_transport));
        printf("└───────────────────────────────────────────┘\n");

        /* ── Export for FlowBoard dashboard (atomic write) ── */
        char* topo_json = discovery_export_json(g_discovery);
        if (topo_json) {
            const char* state_path = flowengine_state_file();
            char tmp_path[512];
            snprintf(tmp_path, sizeof(tmp_path), "%s.tmp", state_path);
            FILE* jf = fopen(tmp_path, "w");
            if (jf) {
                /* Wrap discovery JSON with metrics + registry */
                /* discovery_export_json → {"self":"...","nodes":[...]} */
                /* Output: append ,"metrics":{...},"registry":{...}} */
                size_t len = strlen(topo_json);
                fwrite(topo_json, 1, len - 1, jf); /* strip final } */

                fprintf(jf, ",\"metrics\":{"
                        "\"bus\":{\"published\":%lu,\"delivered\":%lu,\"dropped\":%lu},"
                        "\"transport\":{\"local_pub\":%lu,\"remote_pub\":%lu},"
                        "\"scheduler\":{\"tasks\":%d,\"mode\":\"CHOREO\"},",
                        (unsigned long)pub, (unsigned long)del, (unsigned long)drop,
                        (unsigned long)ts.local_published, (unsigned long)ts.remote_published,
                        task_count);
                LatencyStats ls = latency_tracker_stats(
                    g_fusion_tid >= 0 ? scheduler_get_latency(g_scheduler, g_fusion_tid) : NULL);
                fprintf(jf, "\"latency\":{\"avg_us\":%lu,\"p50_us\":%lu,\"p99_us\":%lu}",
                        (unsigned long)ls.avg_us, (unsigned long)ls.p50_us,
                        (unsigned long)ls.p99_us);

                /* Per-topic stats (QoS) */
                TopicStats tstats[16];
                int nt = message_bus_get_all_topic_stats(g_bus, tstats, 16);
                fprintf(jf, ",\"topics\":[");
                for (int ti = 0; ti < nt; ti++) {
                    uint64_t avg_lat = tstats[ti].deliver_count > 0
                        ? tstats[ti].total_latency_us / tstats[ti].deliver_count : 0;
                    fprintf(jf, "%s{\"topic\":\"%s\",\"pub\":%lu,\"del\":%lu,\"drop\":%lu,"
                            "\"lat_us\":%lu,\"freq\":%.1f,\"subs\":%u}",
                            ti > 0 ? "," : "",
                            tstats[ti].topic,
                            (unsigned long)tstats[ti].publish_count,
                            (unsigned long)tstats[ti].deliver_count,
                            (unsigned long)tstats[ti].drop_count,
                            (unsigned long)avg_lat,
                            tstats[ti].frequency_hz,
                            tstats[ti].subscriber_count);
                }
                fprintf(jf, "]");

                /* Vehicle state (PID closed-loop telemetry) */
                fprintf(jf, ",\"vehicle\":{"
                        "\"speed\":%.1f,\"target_speed\":%.1f,"
                        "\"throttle\":%.3f,\"brake\":%.3f,"
                        "\"x\":%.1f,\"error\":%.1f}",
                        g_vehicle.speed, g_vehicle.target_speed,
                        g_vehicle.throttle, g_vehicle.brake,
                        g_vehicle.x,
                        g_vehicle.target_speed - g_vehicle.speed);

                /* ── 3D scene: ego pose + obstacles + LiDAR point cloud ──
                 * 坐标系: 自车系, x 前为正, y 左为正 (米)。前端据此渲染真实
                 * 障碍物包围盒与点云, 而非随机点。 */
                fprintf(jf, ",\"scene\":{");
                fprintf(jf, "\"ego\":{\"x\":%.2f,\"y\":%.2f,\"heading\":%.3f,"
                        "\"speed\":%.2f,\"steer\":%.3f},",
                        g_vehicle.x, g_vehicle.y, g_vehicle.heading,
                        g_vehicle.speed, g_vehicle.steer);
                fprintf(jf, "\"lane\":{\"width\":3.5,\"count\":2},");
                fprintf(jf, "\"obstacles\":[");
                double ch = cos(-g_vehicle.heading), sh = sin(-g_vehicle.heading);
                for (int oi = 0; oi < SIM_OBSTACLE_COUNT; oi++) {
                    SimObstacle* o = &g_obstacles[oi];
                    /* 转换到自车系 */
                    double dx = o->x - g_vehicle.x, dy = o->y - g_vehicle.y;
                    double rx = dx * ch - dy * sh;
                    double ry = dx * sh + dy * ch;
                    fprintf(jf, "%s{\"id\":%d,\"type\":\"%s\",\"x\":%.2f,\"y\":%.2f,"
                            "\"vx\":%.2f,\"len\":%.1f,\"wid\":%.1f}",
                            oi > 0 ? "," : "", o->id, o->type, rx, ry,
                            o->vx, o->len, o->wid);
                }
                fprintf(jf, "],");
                /* LiDAR: 对障碍物表面 + 地面环带做光线投射 (下采样) */
                fprintf(jf, "\"lidar\":[");
                int lp = 0;
                for (int oi = 0; oi < SIM_OBSTACLE_COUNT; oi++) {
                    SimObstacle* o = &g_obstacles[oi];
                    double dx = o->x - g_vehicle.x, dy = o->y - g_vehicle.y;
                    double rx = dx * ch - dy * sh;
                    double ry = dx * sh + dy * ch;
                    if (rx < -20 || rx > 60) continue;   /* 视野外 */
                    for (int k = 0; k < 6; k++) {
                        double a = (double)k / 6.0 * 6.2831853;
                        double px = rx + cos(a) * o->wid * 0.5;
                        double py = ry + sin(a) * o->len * 0.5;
                        fprintf(jf, "%s[%.2f,%.2f,%.2f]", lp++ ? "," : "",
                                px, py, 0.4);
                    }
                }
                for (int k = 0; k < 24; k++) {           /* 地面环 */
                    double a = (double)k / 24.0 * 6.2831853;
                    double r = 12.0 + 6.0 * ((k % 3) - 1);
                    fprintf(jf, "%s[%.2f,%.2f,%.2f]", lp++ ? "," : "",
                            cos(a) * r, sin(a) * r, 0.0);
                }
                fprintf(jf, "]}");


                /* FlowRegistry: tasks, topics, plugins */
                char* reg_json = flow_registry_export_json();
                if (reg_json) {
                    fprintf(jf, ",\"registry\":%s", reg_json);
                    free(reg_json);
                }
                fprintf(jf, "}}\n");
                fclose(jf);

                /* Atomic rename — readers never see partial content */
                rename(tmp_path, state_path);
            }
            free(topo_json);
        }
    }

    statem_send_event(&base->sm, SM_EVENT_STOP, base);
    return 0;
}

static void monitor_cleanup(TaskBase* base) {
    (void)base;
}

static TaskInterface g_monitor_vtable = {
    .initialize = monitor_init,
    .execute    = monitor_execute,
    .cleanup    = monitor_cleanup,
    .health_check = NULL,
    .on_message = NULL,
};

/* ══════════════════════════════════════════════════════════ */
/* Main                                                        */
/* ══════════════════════════════════════════════════════════ */

int main(int argc, char** argv) {
    int duration = 10;
    const char* role = NULL;
    const char* node_name = "e2e_node";

    /* Parse args: flow_e2e [--role <name>] [duration_sec] */
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--role") == 0 && i + 1 < argc) {
            role = argv[++i];
            /* Each role gets a unique node name for discovery */
            char name_buf[64];
            snprintf(name_buf, sizeof(name_buf), "e2e_%s", role);
            node_name = strdup(name_buf);
        } else {
            duration = atoi(argv[i]);
            if (duration <= 0) duration = 10;
        }
    }

    /* ── 日志: 全局初始化 ── */
    log_init(LOG_INFO, NULL);
    if (role) {
        LOG_INFO("e2e", "╔══════════════════════════════════════════╗");
        LOG_INFO("e2e", "║  FlowEngine Multi-Process Demo           ║");
        LOG_INFO("e2e", "║  Role: %-32s ║", role);
        LOG_INFO("e2e", "║  Node: %-32s ║", node_name);
        LOG_INFO("e2e", "╚══════════════════════════════════════════╝");
        LOG_INFO("e2e", "Start other roles in separate terminals:");
        if (strcmp(role, "perception") != 0) LOG_INFO("e2e", "  ./build/bin/flow_e2e --role perception");
        if (strcmp(role, "fusion") != 0)     LOG_INFO("e2e", "  ./build/bin/flow_e2e --role fusion");
        if (strcmp(role, "control") != 0)    LOG_INFO("e2e", "  ./build/bin/flow_e2e --role control");
        if (strcmp(role, "planning") != 0)   LOG_INFO("e2e", "  ./build/bin/flow_e2e --role planning");
        if (strcmp(role, "monitor") != 0)    LOG_INFO("e2e", "  ./build/bin/flow_e2e --role monitor");
    } else {
        LOG_INFO("e2e", "╔══════════════════════════════════════════╗");
        LOG_INFO("e2e", "║  FlowEngine End-to-End Demo (%ds)        ║", duration);
        LOG_INFO("e2e", "║  感知→融合→控制→规划→监控  全组件串联    ║");
        LOG_INFO("e2e", "╚══════════════════════════════════════════╝");
    }

    /* ── 序列化: 注册 ADAS 类型 ── */
    adas_msgs_register_all();
    LOG_INFO("e2e", "serializer: %d types registered", serializer_type_count());

    /* ── 总线 ── */
    g_bus = message_bus_create("e2e_bus");
    LOG_INFO("e2e", "message_bus: created");

    /* ── 发现 ── */
    g_discovery = discovery_create(node_name,
        CAP_PUBLISHER | CAP_SUBSCRIBER | CAP_FUSION);
    discovery_start(g_discovery);
    LOG_INFO("e2e", "discovery: started");

    /* ── 传输 ── */
    g_transport = transport_create(g_bus, g_discovery, TRANSPORT_AUTO);
    transport_start(g_transport);
    LOG_INFO("e2e", "transport: started (AUTO mode)");

    /* ── 调度器 ── */
    SchedulerConfig scfg = SCHEDULER_CONFIG_DEFAULT;
    scfg.mode = SCHEDULER_MODE_CHOREO;
    g_scheduler = scheduler_create(&scfg);

    /* ── 创建任务 (all-in-one 或按 role 单节点) ── */
    #define ROLE_MATCH(n) (!role || strcmp(role, n) == 0)

    PerceptionTask* pt = NULL;
    FusionTask*     ft = NULL;
    ControlTask*    ct = NULL;
    PlanningTask*   plt = NULL;
    MonitorTask*    mt = NULL;

    if (ROLE_MATCH("perception")) {
        pt = (PerceptionTask*)calloc(1, sizeof(PerceptionTask));
        TaskConfig cfg = { .name="perception", .priority=TASK_PRIORITY_CRITICAL,
                           .max_frequency_hz=20.0, .auto_restart=false };
        task_base_init(&pt->base, &g_perception_vtable, &cfg);
        pt->tid = scheduler_register_task(g_scheduler, &pt->base, "perception");
    }
    if (ROLE_MATCH("fusion")) {
        ft = (FusionTask*)calloc(1, sizeof(FusionTask));
        TaskConfig cfg = { .name="fusion", .priority=TASK_PRIORITY_HIGH,
                           .max_frequency_hz=0, .auto_restart=false };
        task_base_init(&ft->base, &g_fusion_vtable, &cfg);
        ft->tid = scheduler_register_task(g_scheduler, &ft->base, "fusion");
        g_fusion_tid = ft->tid;
    }
    if (ROLE_MATCH("control")) {
        ct = (ControlTask*)calloc(1, sizeof(ControlTask));
        TaskConfig cfg = { .name="control", .priority=TASK_PRIORITY_NORMAL,
                           .max_frequency_hz=100.0, .auto_restart=false };
        task_base_init(&ct->base, &g_control_vtable, &cfg);
        ct->tid = scheduler_register_task(g_scheduler, &ct->base, "control");
    }
    if (ROLE_MATCH("planning")) {
        plt = (PlanningTask*)calloc(1, sizeof(PlanningTask));
        TaskConfig cfg = { .name="planning", .priority=TASK_PRIORITY_NORMAL,
                           .max_frequency_hz=10.0, .auto_restart=false };
        task_base_init(&plt->base, &g_planning_vtable, &cfg);
        plt->tid = scheduler_register_task(g_scheduler, &plt->base, "planning");
    }
    if (ROLE_MATCH("monitor")) {
        mt = (MonitorTask*)calloc(1, sizeof(MonitorTask));
        TaskConfig cfg = { .name="monitor", .priority=TASK_PRIORITY_NORMAL,
                           .max_frequency_hz=10.0, .auto_restart=false };
        task_base_init(&mt->base, &g_monitor_vtable, &cfg);
        mt->tid = scheduler_register_task(g_scheduler, &mt->base, "monitor");
    }

    scheduler_set_choreo_bus(g_scheduler, g_bus);
    scheduler_start(g_scheduler);
    LOG_INFO("e2e", "scheduler: %d tasks in CHOREO mode", scheduler_task_count(g_scheduler));

    /* ── FlowRegistry: 注册所用组件 ── */
    if (ROLE_MATCH("perception"))
        flow_registry_register_task("perception", "LiDAR+Camera+GPS sensor simulator",
            "libfake_perception_task.so",
            (const char*[]){NULL},
            (const char*[]){"sensor/lidar","sensor/camera","sensor/gps",NULL}, NULL);
    if (ROLE_MATCH("fusion"))
        flow_registry_register_task("fusion", "Time-aligned sensor fusion",
            "libflowcoro_task.so",
            (const char*[]){"sensor/lidar","sensor/gps",NULL},
            (const char*[]){"fusion/localization",NULL}, NULL);
    if (ROLE_MATCH("control"))
        flow_registry_register_task("control", "Driving decision maker",
            "libfake_control_task.so",
            (const char*[]){"fusion/localization","planning/trajectory",NULL},
            (const char*[]){"control/cmd",NULL}, NULL);
    if (ROLE_MATCH("planning"))
        flow_registry_register_task("planning", "Trajectory planner",
            "libplanning_task.so",
            (const char*[]){"fusion/localization",NULL},
            (const char*[]){"planning/trajectory",NULL}, NULL);
    if (ROLE_MATCH("monitor"))
        flow_registry_register_task("monitor", "System stats reporter",
            "libexample_task.so", NULL, NULL, NULL);

    if (ROLE_MATCH("perception")) {
        flow_registry_register_topic("sensor/lidar", LIDARFRAME_TYPE_ID, NULL);
        flow_registry_register_topic("sensor/gps", GPSDATA_TYPE_ID, NULL);
        flow_registry_register_topic("sensor/camera", CAMERAFRAME_TYPE_ID, NULL);
    }
    if (ROLE_MATCH("fusion") || ROLE_MATCH("control") || ROLE_MATCH("planning"))
        flow_registry_register_topic("fusion/localization", 0xF0ED10C0u, NULL);
    if (ROLE_MATCH("planning"))
        flow_registry_register_topic("planning/trajectory", 0x3A7B1C2Du, NULL);

    flow_registry_register_plugin("fake_perception", "libfake_perception_task.so",
        (const char*[]){"perception",NULL}, (const char*[]){"LidarFrame","GpsData",NULL});
    flow_registry_register_plugin("fake_control", "libfake_control_task.so",
        (const char*[]){"control",NULL}, (const char*[]){"ControlCmd",NULL});

    LOG_INFO("e2e", "registry: %d total entries", flow_registry_total_count());

    /* ── ParamRegistry: 注册运行时参数 ── */
    param_register_int("control.max_speed", 120, 0, 200, "Max speed km/h");
    param_register_float("fusion.max_delta_ms", 50.0, 10.0, 500.0, "Alignment window ms");
    param_register_bool("control.emergency_brake", true, "Enable AEB");
    param_register_int("perception.lidar_rate_hz", 10, 1, 100, "LiDAR scan rate");
    param_enable_hot_reload("control.max_speed");
    param_enable_hot_reload("control.emergency_brake");
    LOG_INFO("e2e", "params: %d registered (%d hot-reloadable)", param_count(),
             /* count hot-reloadable */ 2);

    /* ── 启动任务（先启动消费者，再启动生产者，确保 trigger 就绪）── */
    task_start(&ft->base);  LOG_INFO("e2e", "fusion:     started (HIGH, choreo)");
    task_start(&ct->base);  LOG_INFO("e2e", "control:    started (NORMAL, choreo)");
    task_start(&mt->base);  LOG_INFO("e2e", "monitor:    started (10Hz)");
    usleep(200000);  /* wait 200ms for subscriptions to take effect */
    task_start(&pt->base);  LOG_INFO("e2e", "perception: started (CRITICAL, 10Hz)");

    /* ── 信号处理 ── */
    signal(SIGINT, sig_handler);
    signal(SIGTERM, sig_handler);

    /* ── MCAP 录制 (Foxglove 兼容) ── */
    McapWriter* mcap = mcap_writer_open("demo.mcap", "x-json");
    if (mcap) {
        mcap_ch_lidar = mcap_writer_register_channel(mcap, "sensor/lidar", "LidarFrame",
            "{\"type\":\"object\",\"properties\":{\"x\":{\"type\":\"number\"},\"y\":{\"type\":\"number\"},\"z\":{\"type\":\"number\"},\"intensity\":{\"type\":\"number\"},\"point_count\":{\"type\":\"integer\"},\"frame_id\":{\"type\":\"integer\"}}}");
        mcap_ch_camera = mcap_writer_register_channel(mcap, "sensor/camera", "CameraFrame",
            "{\"type\":\"object\",\"properties\":{\"x\":{\"type\":\"number\"},\"y\":{\"type\":\"number\"},\"z\":{\"type\":\"number\"},\"point_count\":{\"type\":\"integer\"},\"frame_id\":{\"type\":\"integer\"}}}");
        mcap_ch_gps = mcap_writer_register_channel(mcap, "sensor/gps", "GpsData",
            "{\"type\":\"object\",\"properties\":{\"latitude\":{\"type\":\"number\"},\"longitude\":{\"type\":\"number\"},\"speed_mps\":{\"type\":\"number\"},\"heading_deg\":{\"type\":\"number\"}}}");
        mcap_ch_fusion = mcap_writer_register_channel(mcap, "fusion/localization", "Localization",
            "{\"type\":\"object\",\"properties\":{\"x\":{\"type\":\"number\"},\"y\":{\"type\":\"number\"},\"speed\":{\"type\":\"number\"}}}");
        mcap_ch_vehicle = mcap_writer_register_channel(mcap, "vehicle/state", "VehicleState",
            "{\"type\":\"object\",\"properties\":{\"speed\":{\"type\":\"number\"},\"target_speed\":{\"type\":\"number\"},\"throttle\":{\"type\":\"number\"},\"brake\":{\"type\":\"number\"},\"x\":{\"type\":\"number\"},\"y\":{\"type\":\"number\"},\"error\":{\"type\":\"number\"}}}");
        mcap_writer_set_global(mcap);
        LOG_INFO("e2e", "MCAP: recording to demo.mcap (%u channels)", mcap_ch_vehicle);
    }

    /* ── 运行 ── */
    LOG_INFO("e2e", "running for %d seconds... (Ctrl+C to stop)", duration);
    LOG_INFO("e2e", "monitor: start 'flowmond --port 8800' in another terminal for live dashboard");
    sleep((unsigned)duration);
    g_running = false;

    /* ── 等待任务结束 ── */
    LOG_INFO("e2e", "stopping tasks...");
    task_stop(&pt->base);
    task_stop(&ft->base);
    task_stop(&ct->base);
    task_stop(&mt->base);

    /* ── 统计摘要 ── */
    printf("\n╔══════════════════════════════════════════╗\n");
    printf("║  End-to-End Demo Summary                 ║\n");
    printf("╠══════════════════════════════════════════╣\n");

    /* ── 状态机 ── */
    printf("║  State Machines:                         ║\n");
    printf("║    perception: %-26s ║\n",
           statem_state_name(NULL, statem_current(&pt->base.sm)));
    printf("║    fusion:     %-26s ║\n",
           statem_state_name(NULL, statem_current(&ft->base.sm)));
    printf("║    control:    %-26s ║\n",
           statem_state_name(NULL, statem_current(&ct->base.sm)));

    /* ── 延迟 ── */
    LatencyStats ls = latency_tracker_stats(scheduler_get_latency(g_scheduler, ft->tid));
    printf("║  Fusion Latency (us):                    ║\n");
    printf("║    avg=%lu p50=%lu p99=%lu min=%lu max=%lu   ║\n",
           (unsigned long)ls.avg_us, (unsigned long)ls.p50_us,
           (unsigned long)ls.p99_us, (unsigned long)ls.min_us,
           (unsigned long)ls.max_us);

    /* ── 传输统计 ── */
    TransportStats ts;
    transport_get_stats(g_transport, &ts);
    printf("║  Transport:                              ║\n");
    printf("║    local  pub=%lu del=%lu                    ║\n",
           (unsigned long)ts.local_published, (unsigned long)ts.local_delivered);

    /* ── 发现拓扑 ── */
    discovery_print_graph(g_discovery);
    char* topo_json = discovery_export_json(g_discovery);
    printf("║  Topology JSON: %ld bytes                  ║\n",
           (long)strlen(topo_json ? topo_json : "{}"));
    printf("║  (paste into tools/topology_viewer.html)  ║\n");
    free(topo_json);

    printf("╚══════════════════════════════════════════╝\n\n");

    /* ── 清理 ── */
    scheduler_stop(g_scheduler);
    scheduler_destroy(g_scheduler);
    transport_stop(g_transport);
    transport_destroy(g_transport);
    discovery_stop(g_discovery);
    discovery_destroy(g_discovery);
    message_bus_destroy(g_bus);

    free(pt); free(ft); free(ct); free(mt);
    log_shutdown();
    return 0;
}
