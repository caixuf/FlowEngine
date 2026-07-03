/**
 * fake_perception_task.c — 假感知节点
 *
 * 向消息总线模拟发布：
 *   sensor/lidar         LidarFrame   @10 Hz
 *   sensor/gps           GpsData      @ 5 Hz
 *   perception/obstacles ObstacleList @10 Hz
 *   perception/ego_state EgoState     @10 Hz
 *
 * 内置 ADAS 场景（8 秒）：
 *   t=0.5s  前方 50m 出现一辆车，以 ~5 m/s 持续驶近
 *   t=3.0s  左侧行人从路边向路中横穿（持续约 4 秒）
 */

#include "fake_perception_task.h"
#include "adas_msgs.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>

/* ── 内部结构 ───────────────────────────────────────────── */

struct FakePerceptionTask {
    TaskBase    base;
    MessageBus* bus;
    uint64_t    start_us;       /* 任务启动时间（CLOCK_MONOTONIC，微秒） */
    uint32_t    lidar_frame_id;
    uint32_t    gps_tick;
    uint32_t    obs_frame_id;
};

/* ── 工具函数 ────────────────────────────────────────────── */

static uint64_t perc_now_us(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;
}

/* ── 虚函数实现 ─────────────────────────────────────────── */

static int fake_perc_initialize(TaskBase* base) {
    struct FakePerceptionTask* task = TASK_CAST(struct FakePerceptionTask, base);
    task->lidar_frame_id = 0;
    task->gps_tick       = 0;
    task->obs_frame_id   = 0;
    task->start_us       = perc_now_us();
    printf("[感知节点] 初始化完成: %s\n", base->config.name);
    return 0;
}

static int fake_perc_execute(TaskBase* base) {
    struct FakePerceptionTask* task = TASK_CAST(struct FakePerceptionTask, base);

    printf("[感知节点] 开始模拟传感器数据流\n");
    printf("[感知节点] 话题: sensor/lidar(10Hz) | sensor/gps(5Hz) | "
           "perception/obstacles(10Hz) | perception/ego_state(10Hz)\n");
    printf("[感知节点] 场景: 前方车辆从 50m 处驶近 + 行人 3s 后横穿\n\n");

    while (!task_should_stop(base)) {
        uint64_t now      = perc_now_us();
        double elapsed_s  = (double)(now - task->start_us) / 1.0e6;

        /* ── 激光雷达帧 (10 Hz) ─────────────────────────── */
        LidarFrame lidar = {
            .x           = (float)(task->lidar_frame_id % 100) * 0.1f,
            .y           = 0.0f,
            .z           = 0.5f,
            .intensity   = 0.75f,
            .point_count = 64000 + (task->lidar_frame_id % 1000),
            .frame_id    = task->lidar_frame_id
        };
        message_bus_publish(task->bus, "sensor/lidar", "fake_perception",
                            &lidar, sizeof(lidar));

        /* ── GPS 数据 (5 Hz，每 2 帧发一次) ────────────── */
        if (task->lidar_frame_id % 2 == 0) {
            GpsData gps = {
                .latitude    = 39.9042 + (double)task->gps_tick * 9e-6,
                .longitude   = 116.4074 + (double)task->gps_tick * 1.2e-5,
                .speed_mps   = 8.3f,
                .heading_deg = 0.0f,
                .accuracy_m  = 0.5f
            };
            message_bus_publish(task->bus, "sensor/gps", "fake_perception",
                                &gps, sizeof(gps));
            task->gps_tick++;
        }

        /* ── 障碍物感知 (10 Hz) ─────────────────────────── */
        ObstacleList obs = {
            .frame_id     = task->obs_frame_id,
            .timestamp_us = now,
            .count        = 0
        };

        /* 场景 1：前方有车，从 50m 处以 5 m/s 驶近 */
        if (elapsed_s > 0.5) {
            float car_dist = 50.0f - (float)((elapsed_s - 0.5) * 5.0);
            if (car_dist < 0.5f) car_dist = 0.5f;
            obs.obstacles[obs.count++] = (Obstacle){
                .id         = 1,
                .x          = car_dist,
                .y          = 0.2f,    /* 微偏右 */
                .vx         = -5.0f,   /* 相对驶近 */
                .vy         = 0.0f,
                .width      = 1.8f,
                .length     = 4.5f,
                .type       = OBJ_TYPE_VEHICLE,
                .confidence = 0.95f
            };
        }

        /* 场景 2：t=3s，左侧行人横穿（持续 4 秒） */
        if (elapsed_s > 3.0 && elapsed_s < 7.0) {
            float ped_y = -3.5f + (float)((elapsed_s - 3.0) * 0.9f);
            obs.obstacles[obs.count++] = (Obstacle){
                .id         = 2,
                .x          = 18.0f,
                .y          = ped_y,
                .vx         = 0.0f,
                .vy         = 0.9f,    /* 向路中移动 */
                .width      = 0.6f,
                .length     = 0.6f,
                .type       = OBJ_TYPE_PEDESTRIAN,
                .confidence = 0.85f
            };
        }

        message_bus_publish(task->bus, "perception/obstacles", "fake_perception",
                            &obs, sizeof(obs));

        /* ── 自车状态 (10 Hz) ────────────────────────────── */
        EgoState ego = {
            .latitude      = 39.9042 + (double)task->gps_tick * 9e-6,
            .longitude     = 116.4074 + (double)task->gps_tick * 1.2e-5,
            .heading_deg   = 0.0f,
            .speed_mps     = 8.3f,
            .yaw_rate_dps  = 0.0f,
            .acceleration_mss = 0.0f
        };
        message_bus_publish(task->bus, "perception/ego_state", "fake_perception",
                            &ego, sizeof(ego));

        task->lidar_frame_id++;
        task->obs_frame_id++;
        task_update_heartbeat(base);

        usleep(100000);  /* 10 Hz */
    }

    printf("[感知节点] 退出，共发布 %u 帧感知数据\n", task->obs_frame_id);
    return 0;
}

static void fake_perc_cleanup(TaskBase* base) {
    (void)base;
    printf("[感知节点] 清理完成\n");
}

static bool fake_perc_health_check(TaskBase* base) {
    (void)base;
    return true;
}

/* ── 虚函数表 ───────────────────────────────────────────── */

static const TaskInterface fake_perception_vtable = {
    .initialize   = fake_perc_initialize,
    .execute      = fake_perc_execute,
    .cleanup      = fake_perc_cleanup,
    .health_check = fake_perc_health_check,
    /* pause / resume / handle_signal / get_status / on_message = NULL */
};

/* ── 公共接口 ───────────────────────────────────────────── */

FakePerceptionTask* fake_perception_task_create(const TaskConfig* config, MessageBus* bus) {
    if (!config || !bus) return NULL;

    FakePerceptionTask* task = (FakePerceptionTask*)calloc(1, sizeof(FakePerceptionTask));
    if (!task) return NULL;

    task->bus = bus;

    if (task_base_init(&task->base, &fake_perception_vtable, config) != 0) {
        free(task);
        return NULL;
    }
    return task;
}

void fake_perception_task_destroy(FakePerceptionTask* task) {
    if (!task) return;
    task_base_destroy(&task->base);
    free(task);
}

TaskBase* fake_perception_task_get_base(FakePerceptionTask* task) {
    return task ? &task->base : NULL;
}
