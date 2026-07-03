/**
 * reactive_task.c — 消息驱动任务示例（Step 4）
 *
 * 与 example_task 的区别：
 *  - 不使用 sleep 轮询，而是等待 message_bus 上的消息
 *  - 收到 "sensor/lidar" 消息后立即处理并 publish 结果到 "fusion/result"
 *  - 演示数据驱动调度模型，端到端延迟 < 1ms
 */

#include "task_interface.h"
#include "message_bus.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <time.h>

/* ── 消息类型（与 bus_demo.c 保持一致）── */
typedef struct {
    float    x, y, z;
    float    intensity;
    uint32_t point_count;
    uint32_t frame_id;
} LidarFrame;

typedef struct {
    uint32_t frame_id;
    float    processed_x, processed_y;
    uint64_t latency_us;    /* 端到端延迟（微秒）*/
} FusionResult;

/* ── 任务结构 ──────────────────────────── */

typedef struct {
    TaskBase    base;
    MessageBus* bus;            /* 关联的总线 */
    uint32_t    msg_received;   /* 已处理消息数 */
    pthread_mutex_t result_mutex;
    pthread_cond_t  result_cond;
    bool        has_msg;
    Message     pending;        /* 待处理消息 */
} ReactiveTask;

/* ── 虚函数实现 ────────────────────────── */

static int reactive_init(TaskBase* base) {
    ReactiveTask* task = TASK_CAST(ReactiveTask, base);
    pthread_mutex_init(&task->result_mutex, NULL);
    pthread_cond_init (&task->result_cond, NULL);
    task->has_msg = false;
    printf("[ReactiveTask] 初始化完成: %s\n", base->config.name);
    return 0;
}

static void reactive_on_message(TaskBase* base, const void* raw_msg) {
    ReactiveTask* task = TASK_CAST(ReactiveTask, base);
    const Message* msg = (const Message*)raw_msg;

    pthread_mutex_lock(&task->result_mutex);
    task->pending  = *msg;   /* copy */
    task->has_msg  = true;
    pthread_cond_signal(&task->result_cond);
    pthread_mutex_unlock(&task->result_mutex);
}

static int reactive_execute(TaskBase* base) {
    ReactiveTask* task = TASK_CAST(ReactiveTask, base);

    printf("[ReactiveTask] 进入消息等待循环: %s\n", base->config.name);

    while (!task_should_stop(base)) {
        pthread_mutex_lock(&task->result_mutex);
        while (!task->has_msg && !task_should_stop(base)) {
            struct timespec ts;
            clock_gettime(CLOCK_REALTIME, &ts);
            ts.tv_sec += 1;   /* 1s timeout to check should_stop */
            pthread_cond_timedwait(&task->result_cond,
                                   &task->result_mutex, &ts);
        }
        if (!task->has_msg) {
            pthread_mutex_unlock(&task->result_mutex);
            continue;
        }
        Message msg = task->pending;
        task->has_msg = false;
        pthread_mutex_unlock(&task->result_mutex);

        if (msg.data_size != sizeof(LidarFrame)) continue;

        struct timespec now_ts;
        clock_gettime(CLOCK_MONOTONIC, &now_ts);
        uint64_t recv_us = (uint64_t)now_ts.tv_sec * 1000000ULL +
                           (uint64_t)(now_ts.tv_nsec / 1000);
        uint64_t latency = (recv_us > msg.timestamp_us)
                           ? (recv_us - msg.timestamp_us) : 0;

        const LidarFrame* f = (const LidarFrame*)msg.data;

        FusionResult result = {
            .frame_id    = f->frame_id,
            .processed_x = f->x * 1.1f,
            .processed_y = f->y * 1.1f,
            .latency_us  = latency
        };

        printf("[ReactiveTask] 处理帧 #%u, 延迟=%lu µs\n",
               f->frame_id, (unsigned long)latency);

        if (task->bus) {
            message_bus_publish(task->bus, "fusion/result", base->config.name,
                                &result, sizeof(result));
        }

        task->msg_received++;
        task_update_heartbeat(base);
    }

    printf("[ReactiveTask] 退出，共处理 %u 条消息\n", task->msg_received);
    return 0;
}

static void reactive_cleanup(TaskBase* base) {
    ReactiveTask* task = TASK_CAST(ReactiveTask, base);
    pthread_mutex_destroy(&task->result_mutex);
    pthread_cond_destroy (&task->result_cond);
    printf("[ReactiveTask] 清理完成\n");
}

static bool reactive_health_check(TaskBase* base) {
    (void)base;
    return true;
}

/* ── 虚函数表 ──────────────────────────── */

static const TaskInterface reactive_vtable = {
    .initialize   = reactive_init,
    .execute      = reactive_execute,
    .cleanup      = reactive_cleanup,
    .health_check = reactive_health_check,
    .on_message   = reactive_on_message,
    /* pause, resume, handle_signal, get_status = NULL (使用默认) */
};

/* ── 公共接口 ──────────────────────────── */

ReactiveTask* reactive_task_create(const TaskConfig* config, MessageBus* bus) {
    if (!config) return NULL;
    ReactiveTask* task = calloc(1, sizeof(ReactiveTask));
    if (!task) return NULL;
    task->bus = bus;
    if (task_base_init(&task->base, &reactive_vtable, config) != 0) {
        free(task);
        return NULL;
    }
    /* Subscribe to lidar topic */
    if (bus) task_subscribe(&task->base, bus, "sensor/lidar");
    return task;
}

void reactive_task_destroy(ReactiveTask* task) {
    if (!task) return;
    task_base_destroy(&task->base);
    free(task);
}

TaskBase* reactive_task_get_base(ReactiveTask* task) {
    return task ? &task->base : NULL;
}
