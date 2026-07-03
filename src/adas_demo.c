/**
 * adas_demo.c — ADAS 完整演示程序
 *
 * 演示了一条完整的「感知 → 消息总线 → 控制」链路：
 *
 *  ┌──────────────────────────────────────────────────────────┐
 *  │                  MessageBus (adas_bus)                    │
 *  │                                                          │
 *  │  ┌─────────────────┐    sensor/lidar          ┌──────┐   │
 *  │  │  FakePerception │ ─→ sensor/gps            │      │   │
 *  │  │     Task        │ ─→ perception/obstacles ─→ 控制 │   │
 *  │  │  (10/5 Hz)      │ ─→ perception/ego_state  │ 节点 │   │
 *  │  └─────────────────┘                          │      │   │
 *  │                                               └──┬───┘   │
 *  │                          control/cmd ←───────────┘       │
 *  └──────────────────────────────────────────────────────────┘
 *
 * 内置 8 秒场景：
 *   t=0.5s  前方 50m 出现车辆，以 5 m/s 持续驶近
 *   t=3.0s  左侧行人开始横穿（约持续 4 秒）
 *
 * 运行：
 *   ./build/bin/adas_demo
 */

#include "fake_perception_task.h"
#include "fake_control_task.h"
#include "message_bus.h"
#include "adas_msgs.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>

/* ── 全局状态 ─────────────────────────────────────────────── */

static volatile int g_running = 1;

static void signal_handler(int sig) {
    (void)sig;
    g_running = 0;
}

/* ── 总监控回调（接收所有话题，仅计数）──────────────────── */

static void on_monitor(const Message* msg, void* user_data) {
    uint64_t* count = (uint64_t*)user_data;
    (void)msg;
    (*count)++;
}

/* ── 主程序 ───────────────────────────────────────────────── */

int main(void) {
    printf("╔══════════════════════════════════════════════════════════╗\n");
    printf("║    ADAS 演示程序  (adas_demo)                            ║\n");
    printf("║    感知节点 + 控制节点 + 消息总线 全链路演示              ║\n");
    printf("╚══════════════════════════════════════════════════════════╝\n\n");

    signal(SIGINT,  signal_handler);
    signal(SIGTERM, signal_handler);

    /* ── 1. 创建消息总线 ──────────────────────────────────── */
    MessageBus* bus = message_bus_create("adas_bus");
    if (!bus) {
        fprintf(stderr, "[错误] 创建消息总线失败\n");
        return 1;
    }
    printf("[总线] 创建成功: adas_bus\n");

    uint64_t monitor_count = 0;
    message_bus_subscribe(bus, "*", on_monitor, &monitor_count);

    /* ── 2. 创建感知节点 ──────────────────────────────────── */
    TaskConfig perc_cfg = {
        .name              = "fake_perception",
        .description       = "假感知节点：模拟 LiDAR / GPS / 障碍物 / 自车状态",
        .priority          = TASK_PRIORITY_HIGH,
        .max_restart_count = 0,
        .heartbeat_interval = 5,
        .auto_restart      = false,
        .enable_stats      = true,
        .custom_config     = NULL
    };

    FakePerceptionTask* perc = fake_perception_task_create(&perc_cfg, bus);
    if (!perc) {
        fprintf(stderr, "[错误] 创建感知节点失败\n");
        message_bus_destroy(bus);
        return 1;
    }
    printf("[感知节点] 创建成功\n");

    /* ── 3. 创建控制节点 ──────────────────────────────────── */
    TaskConfig ctrl_cfg = {
        .name              = "fake_control",
        .description       = "假控制节点：接收感知数据，输出油门/制动/转向指令",
        .priority          = TASK_PRIORITY_CRITICAL,
        .max_restart_count = 0,
        .heartbeat_interval = 5,
        .auto_restart      = false,
        .enable_stats      = true,
        .custom_config     = NULL
    };

    FakeControlTask* ctrl = fake_control_task_create(&ctrl_cfg, bus);
    if (!ctrl) {
        fprintf(stderr, "[错误] 创建控制节点失败\n");
        fake_perception_task_destroy(perc);
        message_bus_destroy(bus);
        return 1;
    }
    printf("[控制节点] 创建成功\n\n");

    /* ── 4. 启动节点（先启控制，再启感知，确保订阅就绪）─── */
    printf("═══════════════ 启动 ADAS 节点 ═══════════════\n\n");

    if (task_start(fake_control_task_get_base(ctrl)) != 0) {
        fprintf(stderr, "[错误] 启动控制节点失败\n");
        goto cleanup;
    }
    usleep(50000);  /* 等待控制节点完成初始化与订阅 */

    if (task_start(fake_perception_task_get_base(perc)) != 0) {
        fprintf(stderr, "[错误] 启动感知节点失败\n");
        goto cleanup;
    }

    /* ── 5. 运行 8 秒，等待场景展示完毕 ─────────────────── */
    printf("\n[主程序] 演示运行中，共 8 秒...（按 Ctrl+C 提前退出）\n\n");

    for (int i = 0; i < 8 && g_running; i++) {
        sleep(1);
        /* 每隔 2 秒打印分隔线，使输出更易读 */
        if ((i + 1) % 2 == 0) {
            printf("\n──────────────── 已运行 %d 秒 ────────────────\n\n", i + 1);
        }
    }

cleanup:
    /* ── 6. 停止所有节点 ──────────────────────────────────── */
    printf("\n═══════════════ 停止 ADAS 节点 ═══════════════\n");

    task_stop(fake_perception_task_get_base(perc));
    task_stop(fake_control_task_get_base(ctrl));

    /* ── 7. 打印统计信息 ──────────────────────────────────── */
    uint64_t pub, del, drop;
    message_bus_get_stats(bus, &pub, &del, &drop);

    printf("\n════════════════ 总线统计 ═══════════════════\n");
    printf("  总发布消息数:   %lu\n",  (unsigned long)pub);
    printf("  总投递次数:     %lu\n",  (unsigned long)del);
    printf("  丢弃消息数:     %lu\n",  (unsigned long)drop);
    printf("  监控接收消息数: %lu\n",  (unsigned long)monitor_count);

    printf("\n════════════════ 任务统计 ═══════════════════\n");
    const TaskStats* ps = task_get_stats(fake_perception_task_get_base(perc));
    const TaskStats* cs = task_get_stats(fake_control_task_get_base(ctrl));
    if (ps)
        printf("  感知节点: 运行时间=%.1fs, 错误次数=%u\n",
               (double)ps->total_run_time / 1.0e6, ps->error_count);
    if (cs)
        printf("  控制节点: 运行时间=%.1fs, 错误次数=%u\n",
               (double)cs->total_run_time / 1.0e6, cs->error_count);

    /* ── 8. 清理资源 ──────────────────────────────────────── */
    fake_perception_task_destroy(perc);
    fake_control_task_destroy(ctrl);
    message_bus_destroy(bus);

    printf("\n[总线] 已销毁，ADAS 演示结束\n");
    return 0;
}
