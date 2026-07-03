#include "task_manager.h"
#include "example_task.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>

static TaskManager* g_manager = NULL;

/**
 * 信号处理函数
 */
static void signal_handler(int sig) {
    printf("收到信号 %d，正在关闭任务管理器...\n", sig);
    
    if (g_manager) {
        task_manager_stop_all(g_manager);
        task_manager_destroy(g_manager);
        g_manager = NULL;
    }
    
    exit(0);
}

/**
 * 任务事件回调函数
 */
static void task_event_callback(const char* task_name, TaskState old_state, TaskState new_state, void* user_data) {
    (void)user_data;
    const char* state_names[] = {
        "UNKNOWN", "INITIALIZED", "RUNNING", 
        "STOPPING", "STOPPED", "ERROR"
    };
    
    printf("任务事件: %s 状态从 %s 变为 %s\n", 
           task_name, state_names[old_state], state_names[new_state]);
}

/**
 * 交互式命令处理
 */
static void handle_interactive_commands(TaskManager* manager) {
    char command[256];
    char task_name[64];
    
    printf("\n=== 任务管理器交互模式 ===\n");
    printf("命令:\n");
    printf("  start <task_name>     - 启动任务\n");
    printf("  stop <task_name>      - 停止任务\n");
    printf("  restart <task_name>   - 重启任务\n");
    printf("  status <task_name>    - 查看任务状态\n");
    printf("  list                  - 列出所有任务\n");
    printf("  stats                 - 查看管理器统计\n");
    printf("  health                - 执行健康检查\n");
    printf("  start_all             - 启动所有任务\n");
    printf("  stop_all              - 停止所有任务\n");
    printf("  quit                  - 退出\n");
    printf("\n> ");
    
    while (fgets(command, sizeof(command), stdin)) {
        // 去掉换行符
        command[strcspn(command, "\n")] = '\0';
        
        if (strncmp(command, "quit", 4) == 0) {
            break;
        } else if (strncmp(command, "start ", 6) == 0) {
            sscanf(command + 6, "%s", task_name);
            int ret = task_manager_start_task(manager, task_name);
            if (ret == 0) {
                printf("任务 %s 启动成功\n", task_name);
            } else {
                printf("任务 %s 启动失败 (错误: %d)\n", task_name, ret);
            }
        } else if (strncmp(command, "stop ", 5) == 0) {
            sscanf(command + 5, "%s", task_name);
            int ret = task_manager_stop_task(manager, task_name);
            if (ret == 0) {
                printf("任务 %s 停止成功\n", task_name);
            } else {
                printf("任务 %s 停止失败 (错误: %d)\n", task_name, ret);
            }
        } else if (strncmp(command, "restart ", 8) == 0) {
            sscanf(command + 8, "%s", task_name);
            int ret = task_manager_restart_task(manager, task_name);
            if (ret == 0) {
                printf("任务 %s 重启成功\n", task_name);
            } else {
                printf("任务 %s 重启失败 (错误: %d)\n", task_name, ret);
            }
        } else if (strncmp(command, "status ", 7) == 0) {
            sscanf(command + 7, "%s", task_name);
            TaskBase* task = task_manager_get_task(manager, task_name);
            if (task) {
                char status_buffer[1024];
                int written = TASK_CALL(task, get_status, status_buffer, sizeof(status_buffer));
                if (written > 0) {
                    printf("%s\n", status_buffer);
                } else {
                    printf("获取任务 %s 状态失败\n", task_name);
                }
            } else {
                printf("未找到任务: %s\n", task_name);
            }
        } else if (strcmp(command, "list") == 0) {
            char names[32][64];
            int count = task_manager_list_tasks(manager, names, 32);
            printf("任务列表 (共 %d 个):\n", count);
            for (int i = 0; i < count; i++) {
                TaskState state = task_manager_get_task_state(manager, names[i]);
                const char* state_names[] = {
                    "UNKNOWN", "INITIALIZED", "RUNNING", 
                    "STOPPING", "STOPPED", "ERROR"
                };
                printf("  %s - %s\n", names[i], state_names[state]);
            }
        } else if (strcmp(command, "stats") == 0) {
            uint32_t total, running, error;
            task_manager_get_stats(manager, &total, &running, &error);
            printf("管理器统计:\n");
            printf("  总任务数: %u\n", total);
            printf("  运行中: %u\n", running);
            printf("  错误任务: %u\n", error);
        } else if (strcmp(command, "health") == 0) {
            int unhealthy = task_manager_health_check(manager);
            printf("健康检查完成，不健康任务数: %d\n", unhealthy);
        } else if (strcmp(command, "start_all") == 0) {
            int failed = task_manager_start_all(manager);
            printf("启动所有任务完成，失败任务数: %d\n", failed);
        } else if (strcmp(command, "stop_all") == 0) {
            int failed = task_manager_stop_all(manager);
            printf("停止所有任务完成，失败任务数: %d\n", failed);
        } else if (strlen(command) > 0) {
            printf("未知命令: %s\n", command);
        }
        
        printf("> ");
    }
}

int main() {
    printf("=== 任务系统演示程序 ===\n");
    
    // 安装信号处理器
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    // 创建任务管理器
    g_manager = task_manager_create();
    if (!g_manager) {
        printf("创建任务管理器失败\n");
        return 1;
    }
    
    // 设置事件回调
    task_manager_set_event_callback(g_manager, task_event_callback, NULL);
    
    // 创建示例任务1
    TaskConfig config1 = {
        .name = "task1",
        .description = "第一个示例任务",
        .priority = TASK_PRIORITY_NORMAL,
        .max_restart_count = 3,
        .heartbeat_interval = 10,
        .auto_restart = true,
        .enable_stats = true,
        .custom_config = NULL
    };
    
    ExampleTaskConfig custom_config1 = {
        .work_interval = 3,
        .use_random_delay = false,
        .message = "我是任务1，每3秒工作一次"
    };
    
    ExampleTask* example1 = example_task_create(&config1, &custom_config1);
    if (example1) {
        task_manager_register(g_manager, example_task_get_base(example1), "task1");
        printf("注册任务1成功\n");
    }
    
    // 创建示例任务2
    TaskConfig config2 = {
        .name = "task2", 
        .description = "第二个示例任务",
        .priority = TASK_PRIORITY_HIGH,
        .max_restart_count = 5,
        .heartbeat_interval = 15,
        .auto_restart = true,
        .enable_stats = true,
        .custom_config = NULL
    };
    
    ExampleTaskConfig custom_config2 = {
        .work_interval = 5,
        .use_random_delay = true,
        .message = "我是任务2，每5秒工作一次(有随机延迟)"
    };
    
    ExampleTask* example2 = example_task_create(&config2, &custom_config2);
    if (example2) {
        task_manager_register(g_manager, example_task_get_base(example2), "task2");
        printf("注册任务2成功\n");
    }
    
    // 启动监控
    if (task_manager_start_monitor(g_manager) == 0) {
        printf("任务监控启动成功\n");
    }
    
    // 进入交互模式
    handle_interactive_commands(g_manager);
    
    // 清理资源
    printf("正在清理资源...\n");
    
    if (g_manager) {
        task_manager_stop_all(g_manager);
        task_manager_destroy(g_manager);
    }
    
    // 销毁任务对象
    if (example1) {
        example_task_destroy(example1);
    }
    if (example2) {
        example_task_destroy(example2);
    }
    
    printf("程序结束\n");
    return 0;
}
