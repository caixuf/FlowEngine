#include "process_manager.h"
#include "config_manager.h"
#include "message_bus.h"
#include "param_registry.h"
#include "logger.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <signal.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>

static ProcessManager* g_manager = NULL;
static MessageBus*     g_bus     = NULL;

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
    if (g_bus) {
        message_bus_destroy(g_bus);
        g_bus = NULL;
    }

    log_shutdown();
    exit(0);
}

/**
 * 打印使用帮助
 */
static void print_usage(const char* program_name) {
    printf("Usage: %s [--daemon] <config_file>\n", program_name);
    printf("  --daemon     Run in background (no interactive mode)\n");
    printf("  config_file  JSON configuration file path\n");
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
            ProcessSnapshot procs[64];
            int n = process_manager_list_processes(manager, procs, 64);
            printf("Loaded processes (%d):\n", n);
            printf("  %-20s %-8s %-8s %s\n", "NAME", "STATE", "PRIO", "RESTARTS");
            for (int k = 0; k < n; k++) {
                const char* prio_label = (procs[k].sched_priority == 3) ? "crit" :
                                    (procs[k].sched_priority == 2) ? "high" :
                                    (procs[k].sched_priority == 1) ? "norm" : "low";
                printf("  %-20s %-8s %-8s %u\n",
                       procs[k].name,
                       procs[k].is_running ? "RUNNING" : "STOPPED",
                       prio_label,
                       procs[k].restart_count);
            }
            if (n == 0) printf("  (no processes loaded)\n");
        } else if (strlen(command) > 0) {
            printf("Unknown command: %s\n", command);
        }
        
        printf("> ");
    }
}

/**
 * 将配置文件中声明的 publish QoS 写入消息总线。
 * 支持 depth / policy / reliability / deadline_ms / lifespan_ms。
 */
static void apply_config_qos(MessageBus* bus, const LauncherConfig* cfg) {
    if (!bus || !cfg) return;
    for (int i = 0; i < cfg->process_count; i++) {
        const ProcessConfig* pc = &cfg->processes[i];
        for (int k = 0; k < pc->publish_count; k++) {
            const TopicDecl* td = &pc->publish[k];
            if (td->topic[0] == '\0') continue;
            if (td->qos_depth == 0 && td->qos_policy[0] == '\0') continue;

            TopicQos qos;
            memset(&qos, 0, sizeof(qos));
            qos.depth = td->qos_depth > 0 ? (uint32_t)td->qos_depth : 10;

            if (strcmp(td->qos_policy, "block") == 0)
                qos.policy = QOS_BLOCK;
            else if (strcmp(td->qos_policy, "drop_latest") == 0)
                qos.policy = QOS_DROP_LATEST;
            else
                qos.policy = QOS_DROP_OLDEST;

            if (strcmp(td->qos_policy, "reliable") == 0)
                qos.reliability = QOS_RELIABLE;
            else
                qos.reliability = QOS_BEST_EFFORT;

            message_bus_set_topic_qos(bus, td->topic, &qos);
            LOG_INFO("launcher", "  qos[%s]: depth=%u policy=%s reliability=%s",
                     td->topic, qos.depth,
                     (qos.policy == QOS_BLOCK) ? "block" :
                     (qos.policy == QOS_DROP_LATEST) ? "drop_latest" : "drop_oldest",
                     (qos.reliability == QOS_RELIABLE) ? "reliable" : "best_effort");
        }
    }
}

/**
 * 将配置文件中声明的 subscribe remap 规则写入消息总线。
 */
static void apply_config_remaps(MessageBus* bus, const LauncherConfig* cfg) {
    if (!bus || !cfg) return;
    for (int i = 0; i < cfg->process_count; i++) {
        const ProcessConfig* pc = &cfg->processes[i];
        for (int k = 0; k < pc->subscribe_count; k++) {
            const TopicDecl* td = &pc->subscribe[k];
            if (td->topic[0] == '\0' || td->remap[0] == '\0') continue;
            message_bus_add_remap(bus, td->topic, td->remap);
            LOG_INFO("launcher", "  remap: %s → %s", td->topic, td->remap);
        }
    }
}

/**
 * 将配置文件中声明的 params 写入 param_registry。
 * 格式：每个 process 的 params 字段为 JSON object，key=param名，value=初始值。
 */
static void apply_config_params(const LauncherConfig* cfg) {
    if (!cfg) return;
    for (int i = 0; i < cfg->process_count; i++) {
        const ProcessConfig* pc = &cfg->processes[i];
        if (pc->params[0] == '\0') continue;
        /* params 字段在 config_manager.c 中以原始 JSON object 字符串保存。
         * 以 "{process_name}.params" 为键注册到 param_registry，插件可通过
         * param_get_string("{name}.params") 读取并自行解析 JSON 键值。
         * 示例配置：{"max_speed": 10.5, "sensor_id": 2} */
        char param_name[128];
        snprintf(param_name, sizeof(param_name), "%s.params", pc->name);
        param_register_string(param_name, pc->params, "Process params JSON blob");
    }
}

/**
 * 进入 daemon 模式：fork 到后台，关闭 stdin/stdout/stderr。
 */
static int daemonize(void) {
    pid_t pid = fork();
    if (pid < 0) return -1;
    if (pid > 0) exit(0);  /* parent exits */

    setsid();

    /* Redirect standard fds to /dev/null; handle partial dup2 failure */
    int devnull = open("/dev/null", O_RDWR);
    if (devnull >= 0) {
        if (dup2(devnull, STDIN_FILENO)  < 0 ||
            dup2(devnull, STDOUT_FILENO) < 0 ||
            dup2(devnull, STDERR_FILENO) < 0) {
            close(devnull);
            return -1;
        }
        close(devnull);
    }
    return 0;
}

int main(int argc, char* argv[]) {
    bool daemon_mode = false;
    const char* config_file = NULL;

    /* Parse arguments */
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--daemon") == 0) {
            daemon_mode = true;
        } else if (argv[i][0] != '-') {
            config_file = argv[i];
        }
    }

    if (!config_file) {
        print_usage(argv[0]);
        return 1;
    }
    
    // 安装信号处理器
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    // 加载配置
    LauncherConfig* config = config_load(config_file);
    if (!config) {
        printf("Failed to load config file: %s\n", config_file);
        return 1;
    }

    // daemon 模式：fork 到后台（在日志初始化之前，避免文件描述符问题）
    if (daemon_mode) {
        if (daemonize() != 0) {
            fprintf(stderr, "Failed to daemonize\n");
            config_free(config);
            return 1;
        }
    }
    
    // 初始化全局日志系统
    log_init((LogLevel)(config->log_level & 0xff), config->log_file);
    LOG_INFO("launcher", "starting... (daemon=%s)", daemon_mode ? "yes" : "no");

    // 创建共享消息总线（供 QoS/remap 配置使用）
    g_bus = message_bus_create("launcher_bus");
    if (!g_bus) {
        LOG_WARN("launcher", "Failed to create message bus; QoS/remap config skipped");
    } else {
        // 应用 QoS 配置
        apply_config_qos(g_bus, config);
        // 应用 topic remap 规则
        apply_config_remaps(g_bus, config);
    }

    // 注册进程参数到 param_registry
    apply_config_params(config);

    // 创建进程管理器
    g_manager = process_manager_create(default_log_callback);
    if (!g_manager) {
        LOG_ERROR("launcher", "Failed to create process manager");
        if (g_bus) { message_bus_destroy(g_bus); g_bus = NULL; }
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

    // 交互模式：若是 tty 且非 daemon，进入交互命令行；否则等待信号
    if (!daemon_mode && isatty(STDIN_FILENO)) {
        handle_interactive_commands(g_manager);
    } else {
        LOG_INFO("launcher", "Running in non-interactive mode, waiting for signal...");
        pause();  /* sleep until SIGINT/SIGTERM */
    }

    // 清理资源
    LOG_INFO("launcher", "shutting down...");

    if (g_manager) {
        process_manager_stop_all(g_manager);
        process_manager_destroy(g_manager);
        g_manager = NULL;
    }
    if (g_bus) {
        message_bus_destroy(g_bus);
        g_bus = NULL;
    }
    
    config_free(config);
    
    return 0;
}
