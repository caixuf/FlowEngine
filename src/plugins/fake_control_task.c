/**
 * fake_control_task.c — 假控制节点（消息驱动）
 *
 * 订阅：
 *   perception/obstacles  — 每帧触发决策
 *   perception/ego_state  — 更新自车状态（参考用）
 *
 * 发布：
 *   control/cmd           — ControlCmd（每收到一帧障碍物数据发出一条）
 *
 * 决策策略（基于正前方最近障碍物距离）：
 *   dist >= 50m : 正常巡航  throttle=0.40
 *   25m ~ 50m  : 减速巡航  throttle 线性降低
 *   10m ~ 25m  : 制动减速  brake 线性增加
 *   <  10m     : 紧急制动  brake=1.0, emergency_stop=true
 *
 * 转向：简单比例控制，向障碍物对侧轻微转向。
 */

#include "fake_control_task.h"
#include "adas_msgs.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <time.h>
#include <math.h>

/* ── 内部结构 ───────────────────────────────────────────── */

struct FakeControlTask {
    TaskBase base;
    MessageBus* bus;

    /* 最新感知数据（由 on_message 更新，execute 消费）*/
    ObstacleList latest_obs;
    EgoState     latest_ego;
    bool         has_obs;
    bool         has_ego;

    /* 条件变量：新障碍物帧到来时唤醒 execute() */
    pthread_mutex_t msg_mutex;
    pthread_cond_t  msg_cond;
    bool            obs_ready;

    uint32_t cmd_seq;
};

/* ── 工具：符号限幅 ─────────────────────────────────────── */

static float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

/* ── on_message：在总线分发线程中被调用 ───────────────────── */

static void fake_ctrl_on_message(TaskBase* base, const void* raw_msg) {
    struct FakeControlTask* task = TASK_CAST(struct FakeControlTask, base);
    const Message* msg = (const Message*)raw_msg;

    pthread_mutex_lock(&task->msg_mutex);

    if (strcmp(msg->topic, "perception/obstacles") == 0) {
        if (msg->data_size == sizeof(ObstacleList)) {
            task->latest_obs = *(const ObstacleList*)msg->data;
            task->has_obs    = true;
            task->obs_ready  = true;
            pthread_cond_signal(&task->msg_cond);
        }
    } else if (strcmp(msg->topic, "perception/ego_state") == 0) {
        if (msg->data_size == sizeof(EgoState)) {
            task->latest_ego = *(const EgoState*)msg->data;
            task->has_ego    = true;
        }
    }

    pthread_mutex_unlock(&task->msg_mutex);
}

/* ── initialize ─────────────────────────────────────────── */

static int fake_ctrl_initialize(TaskBase* base) {
    struct FakeControlTask* task = TASK_CAST(struct FakeControlTask, base);
    pthread_mutex_init(&task->msg_mutex, NULL);
    pthread_cond_init (&task->msg_cond,  NULL);
    task->obs_ready = false;
    task->has_obs   = false;
    task->has_ego   = false;
    task->cmd_seq   = 0;
    printf("[控制节点] 初始化完成: %s\n", base->config.name);
    return 0;
}

/* ── execute：决策主循环 ─────────────────────────────────── */

static int fake_ctrl_execute(TaskBase* base) {
    struct FakeControlTask* task = TASK_CAST(struct FakeControlTask, base);

    printf("[控制节点] 等待感知数据...\n");
    printf("[控制节点] 订阅: perception/obstacles | perception/ego_state\n");
    printf("[控制节点] 发布: control/cmd\n\n");

    while (!task_should_stop(base)) {

        /* 等待新的障碍物帧 */
        pthread_mutex_lock(&task->msg_mutex);
        while (!task->obs_ready && !task_should_stop(base)) {
            struct timespec ts;
            clock_gettime(CLOCK_REALTIME, &ts);
            ts.tv_sec += 1;   /* 1s 超时，防止错过停止信号 */
            pthread_cond_timedwait(&task->msg_cond, &task->msg_mutex, &ts);
        }
        if (!task->obs_ready) {
            pthread_mutex_unlock(&task->msg_mutex);
            continue;
        }
        ObstacleList obs = task->latest_obs;
        task->obs_ready  = false;
        pthread_mutex_unlock(&task->msg_mutex);

        /* ── 决策：找正前方最近障碍物 ─────────────────── */
        float closest_x   = 1000.0f;
        float steer_offset = 0.0f;   /* 向障碍物对侧转向偏量 */

        for (uint32_t i = 0; i < obs.count; i++) {
            const Obstacle* o = &obs.obstacles[i];
            /* 只考虑正前方扇区（x > 0，横向偏差 < 2.5m）*/
            if (o->x > 0.0f && fabsf(o->y) < 2.5f) {
                if (o->x < closest_x) {
                    closest_x    = o->x;
                    /* y > 0 表示障碍偏右 → 向左（负）转向；反之向右 */
                    steer_offset = clampf(-o->y * 0.04f, -0.3f, 0.3f);
                }
            }
        }

        /* ── 构造控制指令 ────────────────────────────── */
        ControlCmd cmd = {
            .seq            = task->cmd_seq++,
            .throttle       = 0.0f,
            .brake          = 0.0f,
            .steering       = steer_offset,
            .gear           = GEAR_DRIVE,
            .emergency_stop = false
        };

        const char* decision_label;

        if (closest_x < 10.0f) {
            /* 紧急制动 */
            cmd.brake         = 1.0f;
            cmd.emergency_stop = true;
            decision_label    = "⚠  紧急制动";
            printf("[控制节点] %s! dist=%.1fm brake=1.0 | #%u\n",
                   decision_label, closest_x, cmd.seq);
        } else if (closest_x < 25.0f) {
            /* 制动减速：brake 随距离缩短线性增大 */
            float ratio  = (25.0f - closest_x) / 15.0f;  /* [0, 1] */
            cmd.brake    = clampf(ratio * 0.8f, 0.0f, 0.8f);
            decision_label = "🔴 制动减速";
            printf("[控制节点] %s  dist=%.1fm brake=%.2f steer=%+.2f | #%u\n",
                   decision_label, closest_x, cmd.brake, cmd.steering, cmd.seq);
        } else if (closest_x < 50.0f) {
            /* 减速巡航：油门随距离增大线性增大 */
            float ratio   = (closest_x - 25.0f) / 25.0f; /* [0, 1] */
            cmd.throttle  = clampf(ratio * 0.3f, 0.0f, 0.3f);
            decision_label = "🟡 减速巡航";
            printf("[控制节点] %s dist=%.1fm throttle=%.2f steer=%+.2f | #%u\n",
                   decision_label, closest_x, cmd.throttle, cmd.steering, cmd.seq);
        } else {
            /* 正常巡航 */
            cmd.throttle  = 0.4f;
            decision_label = "🟢 正常巡航";
            printf("[控制节点] %s  throttle=%.2f | #%u\n",
                   decision_label, cmd.throttle, cmd.seq);
        }

        message_bus_publish(task->bus, "control/cmd", base->config.name,
                            &cmd, sizeof(cmd));
        task_update_heartbeat(base);
    }

    printf("[控制节点] 退出，共发出 %u 条控制指令\n", task->cmd_seq);
    return 0;
}

/* ── cleanup ────────────────────────────────────────────── */

static void fake_ctrl_cleanup(TaskBase* base) {
    struct FakeControlTask* task = TASK_CAST(struct FakeControlTask, base);
    pthread_mutex_destroy(&task->msg_mutex);
    pthread_cond_destroy (&task->msg_cond);
    printf("[控制节点] 清理完成\n");
}

static bool fake_ctrl_health_check(TaskBase* base) {
    (void)base;
    return true;
}

/* ── 虚函数表 ───────────────────────────────────────────── */

static const TaskInterface fake_control_vtable = {
    .initialize   = fake_ctrl_initialize,
    .execute      = fake_ctrl_execute,
    .cleanup      = fake_ctrl_cleanup,
    .health_check = fake_ctrl_health_check,
    .on_message   = fake_ctrl_on_message,
    /* pause / resume / handle_signal / get_status = NULL */
};

/* ── 公共接口 ───────────────────────────────────────────── */

FakeControlTask* fake_control_task_create(const TaskConfig* config, MessageBus* bus) {
    if (!config || !bus) return NULL;

    FakeControlTask* task = (FakeControlTask*)calloc(1, sizeof(FakeControlTask));
    if (!task) return NULL;

    task->bus = bus;

    if (task_base_init(&task->base, &fake_control_vtable, config) != 0) {
        free(task);
        return NULL;
    }

    /* 订阅两个感知话题，消息到来时触发 on_message */
    task_subscribe(&task->base, bus, "perception/obstacles");
    task_subscribe(&task->base, bus, "perception/ego_state");

    return task;
}

void fake_control_task_destroy(FakeControlTask* task) {
    if (!task) return;
    task_base_destroy(&task->base);
    free(task);
}

TaskBase* fake_control_task_get_base(FakeControlTask* task) {
    return task ? &task->base : NULL;
}
