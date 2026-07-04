#include "process_manager.h"
#include "config_manager.h"
#include "logger.h"
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
#include <string.h>

static ProcessManager* g_manager = NULL;

/**
 * 信号处理函数
 */
static void signal_handler(int sig) {
    LOG_INFO("launcher", "Received signal %d, shutting down...", sig);

    if (g_manager) {
        process_manager_stop_all(g_manager);
        process_manager_destroy(g_manager);
        g_manager = NULL;
    }

    log_shutdown();
    exit(0);
}

/**
 * 打印使用帮助
 */
static void print_usage(const char* program_name) {
    printf("Usage: %s <config_file>\n", program_name);
    printf("  config_file: JSON configuration file path\n");
    printf("\nExample config file:\n");
    printf("{\n");
    printf("  \"log_file\": \"launcher.log\",\n");
    printf("  \"log_level\": 1,\n");
    printf("  \"monitor_interval\": 5,\n");
    printf("  \"enable_monitor\": true,\n");
    printf("  \"processes\": [\n");
    printf("    {\n");
    printf("      \"name\": \"example_process\",\n");
    printf("      \"library_path\": \"./plugins/example.so\",\n");
    printf("      \"config_data\": \"{}\",\n");
    printf("      \"priority\": 1,\n");
    printf("      \"auto_start\": true\n");
    printf("    }\n");
    printf("  ]\n");
    printf("}\n");
}

/**
 * 交互式命令处理
 */
static void handle_interactive_commands(ProcessManager* manager) {
    char command[256];
    char process_name[64];
    
    printf("\nLauncher Interactive Mode\n");
    printf("Commands:\n");
    printf("  start <process_name>   - Start a process\n");
    printf("  stop <process_name>    - Stop a process\n");
    printf("  restart <process_name> - Restart a process\n");
    printf("  status <process_name>  - Get process status\n");
    printf("  list                   - List all processes\n");
    printf("  quit                   - Exit launcher\n");
    printf("\n> ");
    
    while (fgets(command, sizeof(command), stdin)) {
        // 去掉换行符
        command[strcspn(command, "\n")] = '\0';
        
        if (strncmp(command, "quit", 4) == 0) {
            break;
        } else if (strncmp(command, "start ", 6) == 0) {
            sscanf(command + 6, "%s", process_name);
            int ret = process_manager_start_process(manager, process_name);
            if (ret == 0) {
                printf("Process %s started successfully\n", process_name);
            } else {
                printf("Failed to start process %s (error: %d)\n", process_name, ret);
            }
        } else if (strncmp(command, "stop ", 5) == 0) {
            sscanf(command + 5, "%s", process_name);
            int ret = process_manager_stop_process(manager, process_name);
            if (ret == 0) {
                printf("Process %s stopped successfully\n", process_name);
            } else {
                printf("Failed to stop process %s (error: %d)\n", process_name, ret);
            }
        } else if (strncmp(command, "restart ", 8) == 0) {
            sscanf(command + 8, "%s", process_name);
            int ret = process_manager_restart_process(manager, process_name);
            if (ret == 0) {
                printf("Process %s restarted successfully\n", process_name);
            } else {
                printf("Failed to restart process %s (error: %d)\n", process_name, ret);
            }
        } else if (strncmp(command, "status ", 7) == 0) {
            sscanf(command + 7, "%s", process_name);
            ProcessState state = process_manager_get_process_state(manager, process_name);
            const char* state_names[] = {"UNKNOWN", "INITIALIZING", "RUNNING", "STOPPING", "STOPPED", "ERROR"};
            printf("Process %s state: %s\n", process_name, state_names[state]);
        } else if (strcmp(command, "list") == 0) {
            printf("Process list functionality not implemented yet\n");
        } else if (strlen(command) > 0) {
            printf("Unknown command: %s\n", command);
        }
        
        printf("> ");
    }
}

int main(int argc, char* argv[]) {
    if (argc != 2) {
        print_usage(argv[0]);
        return 1;
    }
    
    // 安装信号处理器
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    // 加载配置
    LauncherConfig* config = config_load(argv[1]);
    if (!config) {
        printf("Failed to load config file: %s\n", argv[1]);
        return 1;
    }
    
    // 初始化全局日志系统
    log_init((LogLevel)(config->log_level & 0xff), config->log_file);
    LOG_INFO("launcher", "starting...");

    // 创建进程管理器
    g_manager = process_manager_create(default_log_callback);
    if (!g_manager) {
        LOG_ERROR("launcher", "Failed to create process manager");
        log_shutdown();
        config_free(config);
        return 1;
    }

    // 加载所有插件
    int loaded_count = 0;
    for (int i = 0; i < config->process_count; i++) {
        ProcessConfig* proc_config = &config->processes[i];
        int ret = process_manager_load_plugin(g_manager,
                                             proc_config->name,
                                             proc_config->library_path,
                                             proc_config->config_data);
        if (ret == 0) {
            loaded_count++;

            /* ── 应用调度配置 ── */
            SchedulingConfig* sc = &proc_config->scheduling;
            if (sc->priority > 0 || sc->cpu_affinity_mask != 0 || sc->max_frequency_hz > 0) {
                LOG_INFO("launcher", "  scheduling[%s]: prio=%d cpu_mask=0x%lx freq=%.1fHz",
                         proc_config->name, sc->priority,
                         (unsigned long)sc->cpu_affinity_mask,
                         sc->max_frequency_hz);
                process_manager_set_scheduling(g_manager, proc_config->name,
                                               sc->priority,
                                               sc->cpu_affinity_mask,
                                               sc->max_frequency_hz);
            }

            if (proc_config->auto_start) {
                process_manager_start_process(g_manager, proc_config->name);
            }
        } else {
            LOG_ERROR("launcher", "Failed to load plugin %s: %d", proc_config->name, ret);
        }
    }

    LOG_INFO("launcher", "Loaded %d/%d plugins (scheduler: %s mode)",
             loaded_count, config->process_count,
             config->scheduler.mode == 1 ? "choreo" : "classic");

    // 启动监控线程
    if (config->enable_monitor) {
        if (process_manager_start_monitor(g_manager) == 0) {
            LOG_INFO("launcher", "Monitor thread started");
        } else {
            LOG_ERROR("launcher", "Failed to start monitor thread");
        }
    }

    // 进入交互模式
    handle_interactive_commands(g_manager);

    // 清理资源
    LOG_INFO("launcher", "shutting down...");

    if (g_manager) {
        process_manager_stop_all(g_manager);
        process_manager_destroy(g_manager);
    }
    
    config_free(config);
    
    return 0;
}
