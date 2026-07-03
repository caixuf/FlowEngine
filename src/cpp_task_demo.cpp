#include "task_interface.h"
#include "task_manager.h"
#include "logger.h"
#include "config_manager.h"
#include "process_interface.h" // 添加这个包含LogLevel和LogCallback定义
#include <iostream>
#include <thread>
#include <chrono>
#include <vector>
#include <memory>
#include <functional>
#include <signal.h>
#include <cstring>

// 声明C++任务的创建和销毁函数
extern "C" {
    // cpp_example_task.cpp 中的函数
    struct CppTaskWrapper;
    CppTaskWrapper* cpp_task_create(const TaskConfig* config);
    void cpp_task_destroy(CppTaskWrapper* wrapper);
    TaskBase* cpp_task_get_base(CppTaskWrapper* wrapper);
}

// 全局变量
static volatile bool g_running = true;
static TaskManager* g_task_manager = nullptr;
/* Logger now uses global singleton via LOG_* macros */

// 信号处理函数
void signal_handler(int signum) {
    std::cout << "\n收到信号 " << signum << "，准备退出...\n";
    g_running = false;
    
    if (g_task_manager) {
        task_manager_stop_all(g_task_manager);
    }
}

// 设置信号处理
void setup_signal_handlers() {
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
}

// 创建任务配置
TaskConfig create_task_config(const std::string& task_name, const std::string& config_data = "") {
    TaskConfig config;
    memset(&config, 0, sizeof(config));
    
    strncpy(config.name, task_name.c_str(), sizeof(config.name) - 1);
    config.name[sizeof(config.name) - 1] = '\0';
    
    strncpy(config.description, "C++ Task", sizeof(config.description) - 1);
    config.description[sizeof(config.description) - 1] = '\0';
    
    config.priority = TASK_PRIORITY_NORMAL;
    config.max_restart_count = 3;
    config.heartbeat_interval = 30;
    config.auto_restart = true;
    config.enable_stats = true;
    
    if (!config_data.empty()) {
        config.custom_config = strdup(config_data.c_str());
    }
    
    return config;
}

// 打印任务状态
void print_task_status(TaskManager* manager) {
    (void)manager;
    std::cout << "\n=== 任务管理器状态 ===\n";
    std::cout << "任务状态查询功能开发中...\n";
    std::cout << "========================\n";
}

// 创建并运行C++任务演示
int run_cpp_task_demo() {
    std::cout << "=== C++ 任务演示程序 ===\n\n";
    
    // 初始化日志系统
    // 创建任务管理器
    g_task_manager = task_manager_create();
    if (!g_task_manager) {
        std::cerr << "任务管理器创建失败\n";
        return -1;
    }
    
    std::cout << "[OK] 任务管理器创建完成\n";
    
    // 设置信号处理
    setup_signal_handlers();
    std::cout << "[OK] 信号处理设置完成\n";
    
    try {
        // 1. 创建C++示例任务
        std::cout << "\n--- 创建C++示例任务 ---\n";
        TaskConfig cpp_config = create_task_config("cpp_example_task", 
            R"({"iterations": 100, "delay_ms": 500})");
        
        auto* cpp_wrapper = cpp_task_create(&cpp_config);
        if (!cpp_wrapper) {
            throw std::runtime_error("创建C++示例任务失败");
        }
        
        TaskBase* cpp_task = cpp_task_get_base(cpp_wrapper);
        if (task_manager_register(g_task_manager, cpp_task, "cpp_example_task") != 0) {
            cpp_task_destroy(cpp_wrapper);
            throw std::runtime_error("注册C++示例任务失败");
        }
        
        std::cout << "[OK] C++示例任务创建并注册完成\n";
        
        // 启动所有任务
        std::cout << "\n--- 启动所有任务 ---\n";
        if (task_manager_start_all(g_task_manager) != 0) {
            throw std::runtime_error("启动任务失败");
        }
        
        std::cout << "[OK] 所有任务启动完成\n";
        
        // 主监控循环
        std::cout << "\n--- 开始监控任务状态 ---\n";
        std::cout << "按 Ctrl+C 退出程序...\n\n";
        
        int status_counter = 0;
        while (g_running) {
            std::this_thread::sleep_for(std::chrono::seconds(5));
            
            // 每30秒打印一次状态
            if (++status_counter % 6 == 0) {
                print_task_status(g_task_manager);
            }
            
            // 检查任务健康状态
            if (!task_manager_health_check(g_task_manager)) {
                std::cout << "[WARN] 检测到任务健康状态异常\n";
                print_task_status(g_task_manager);
            }
        }
        
        // 清理资源
        std::cout << "\n--- 开始清理资源 ---\n";
        
        // 停止所有任务
        task_manager_stop_all(g_task_manager);
        std::cout << "[OK] 所有任务已停止\n";
        
        // 销毁任务包装器
        cpp_task_destroy(cpp_wrapper);
        std::cout << "[OK] 任务包装器已销毁\n";
        
        // 销毁任务管理器
        task_manager_destroy(g_task_manager);
        g_task_manager = nullptr;
        std::cout << "[OK] 任务管理器已销毁\n";
        
        // 释放配置内存
        if (cpp_config.custom_config) free((void*)cpp_config.custom_config);
        
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] 程序异常: " << e.what() << std::endl;
        
        if (g_task_manager) {
            task_manager_destroy(g_task_manager);
            g_task_manager = nullptr;
        }
        
        log_shutdown();
        
        return -1;
    }
    
    // 清理日志系统
    log_shutdown();
    std::cout << "[OK] 日志系统已清理\n";
    
    std::cout << "\n=== C++ 任务演示程序正常退出 ===\n";
    return 0;
}

// 交互式菜单
void show_interactive_menu() {
    std::cout << "\n=== 交互式菜单 ===\n";
    std::cout << "1. 查看任务状态\n";
    std::cout << "2. 健康检查\n";
    std::cout << "3. 重启所有任务\n";
    std::cout << "4. 停止所有任务\n";
    std::cout << "5. 启动所有任务\n";
    std::cout << "0. 退出程序\n";
    std::cout << "请输入选项 (0-5): ";
}

int run_interactive_demo() {
    std::cout << "=== C++ 任务交互式演示 ===\n\n";
    
    // 初始化日志系统
    log_init(LOG_INFO, "logs/cpp_interactive_demo.log");

    // 创建任务管理器
    g_task_manager = task_manager_create();
    if (!g_task_manager) {
        std::cerr << "任务管理器创建失败\n";
        log_shutdown();
        return -1;
    }
    
    std::vector<std::function<void()>> cleanup_functions;
    
    try {
        // 创建简化任务（只创建一个C++示例任务）
        TaskConfig config = create_task_config("interactive_cpp_task", 
            R"({"iterations": 50, "delay_ms": 1000})");
        
        auto* wrapper = cpp_task_create(&config);
        if (!wrapper) {
            throw std::runtime_error("创建C++任务失败");
        }
        
        cleanup_functions.push_back([wrapper]() {
            cpp_task_destroy(wrapper);
        });
        
        TaskBase* task = cpp_task_get_base(wrapper);
        if (task_manager_register(g_task_manager, task, "simple_cpp_task") != 0) {
            throw std::runtime_error("注册C++任务失败");
        }
        
        std::cout << "[OK] 任务创建完成\n";
        
        // 交互循环
        int choice;
        while (true) {
            show_interactive_menu();
            
            if (!(std::cin >> choice)) {
                std::cin.clear();
                std::cin.ignore(10000, '\n');
                std::cout << "无效输入，请重新输入\n";
                continue;
            }
            
            switch (choice) {
                case 1:
                    print_task_status(g_task_manager);
                    break;
                    
                case 2:
                    if (task_manager_health_check(g_task_manager)) {
                        std::cout << "[OK] 所有任务健康状态正常\n";
                    } else {
                        std::cout << "[WARN] 检测到任务健康状态异常\n";
                        print_task_status(g_task_manager);
                    }
                    break;
                    
                case 3:
                    std::cout << "重启所有任务...\n";
                    task_manager_stop_all(g_task_manager);
                    std::this_thread::sleep_for(std::chrono::seconds(2));
                    task_manager_start_all(g_task_manager);
                    std::cout << "[OK] 所有任务已重启\n";
                    break;
                    
                case 4:
                    std::cout << "停止所有任务...\n";
                    task_manager_stop_all(g_task_manager);
                    std::cout << "[OK] 所有任务已停止\n";
                    break;
                    
                case 5:
                    std::cout << "启动所有任务...\n";
                    task_manager_start_all(g_task_manager);
                    std::cout << "[OK] 所有任务已启动\n";
                    break;
                    
                case 0:
                    std::cout << "退出程序...\n";
                    goto cleanup;
                    
                default:
                    std::cout << "无效选项，请重新选择\n";
                    break;
            }
        }
        
cleanup:
        // 清理资源
        task_manager_stop_all(g_task_manager);
        
        for (auto& cleanup : cleanup_functions) {
            cleanup();
        }
        
        if (config.custom_config) {
            free((void*)config.custom_config);
        }
        
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] 程序异常: " << e.what() << std::endl;
        
        for (auto& cleanup : cleanup_functions) {
            cleanup();
        }
        
        task_manager_destroy(g_task_manager);
        log_shutdown();
        return -1;
    }
    
    task_manager_destroy(g_task_manager);
    log_shutdown();

    std::cout << "=== 交互式演示程序退出 ===\n";
    return 0;
}

int main(int argc, char* argv[]) {
    std::cout << "C++ 任务系统演示程序\n";
    std::cout << "构建时间: " << __DATE__ << " " << __TIME__ << "\n\n";
    
    std::string mode = "auto";
    if (argc > 1) {
        mode = argv[1];
    }
    
    if (mode == "interactive" || mode == "-i") {
        return run_interactive_demo();
    } else {
        return run_cpp_task_demo();
    }
}
