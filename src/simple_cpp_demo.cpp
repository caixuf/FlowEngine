#include "task_interface.h"
#include "task_manager.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <cstring>
#include <signal.h>

// 声明C++任务的创建和销毁函数
extern "C" {
    // simple_cpp_task.cpp 中的函数
    struct CppTaskWrapper;
    CppTaskWrapper* cpp_task_create(const TaskConfig* config);
    void cpp_task_destroy(CppTaskWrapper* wrapper);
    TaskBase* cpp_task_get_base(CppTaskWrapper* wrapper);
}

// 全局变量
static volatile bool g_running = true;
static TaskManager* g_task_manager = nullptr;

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
TaskConfig create_task_config(const std::string& task_name) {
    TaskConfig config;
    memset(&config, 0, sizeof(config));
    
    strncpy(config.name, task_name.c_str(), sizeof(config.name) - 1);
    config.name[sizeof(config.name) - 1] = '\0';
    
    strncpy(config.description, "C++ Task Demo", sizeof(config.description) - 1);
    config.description[sizeof(config.description) - 1] = '\0';
    
    config.priority = TASK_PRIORITY_NORMAL;
    config.max_restart_count = 3;
    config.heartbeat_interval = 30;
    config.auto_restart = true;
    config.enable_stats = true;
    config.custom_config = nullptr;
    
    return config;
}

int main() {
    std::cout << "=== 简化 C++ 任务演示程序 ===\n\n";
    
    // 创建任务管理器
    g_task_manager = task_manager_create();
    if (!g_task_manager) {
        std::cerr << "[ERROR] 任务管理器创建失败\n";
        return -1;
    }
    
    std::cout << "[OK] 任务管理器创建完成\n";
    
    // 设置信号处理
    setup_signal_handlers();
    std::cout << "[OK] 信号处理设置完成\n";
    
    try {
        // 创建C++任务
        std::cout << "\n--- 创建C++任务 ---\n";
        TaskConfig cpp_config = create_task_config("cpp_example_task");
        
        auto* cpp_wrapper = cpp_task_create(&cpp_config);
        if (!cpp_wrapper) {
            throw std::runtime_error("创建C++任务失败");
        }
        
        TaskBase* cpp_task = cpp_task_get_base(cpp_wrapper);
        if (task_manager_register(g_task_manager, cpp_task, "cpp_example_task") != 0) {
            cpp_task_destroy(cpp_wrapper);
            throw std::runtime_error("注册C++任务失败");
        }
        
        std::cout << "[OK] C++任务创建并注册完成\n";
        
        // 启动任务
        std::cout << "\n--- 启动任务 ---\n";
        if (task_manager_start_task(g_task_manager, "cpp_example_task") != 0) {
            throw std::runtime_error("启动C++任务失败");
        }
        
        std::cout << "[OK] C++任务启动完成\n";
        std::cout << "\n--- 监控任务运行状态 ---\n";
        std::cout << "按 Ctrl+C 退出程序...\n\n";
        
        // 主监控循环
        int counter = 0;
        while (g_running) {
            std::this_thread::sleep_for(std::chrono::seconds(5));
            
            if (++counter % 6 == 0) { // 每30秒输出一次状态
                std::cout << "程序运行中... (已运行 " << counter * 5 << " 秒)\n";
                
                // 简单的健康检查
                if (task_manager_health_check(g_task_manager)) {
                    std::cout << "[OK] 任务健康状态正常\n";
                } else {
                    std::cout << "[WARN] 检测到任务健康状态异常\n";
                }
            }
        }
        
        // 清理资源
        std::cout << "\n--- 开始清理资源 ---\n";
        
        // 停止任务
        task_manager_stop_all(g_task_manager);
        std::cout << "[OK] 任务已停止\n";
        
        // 销毁任务包装器
        cpp_task_destroy(cpp_wrapper);
        std::cout << "[OK] 任务包装器已销毁\n";
        
        // 销毁任务管理器
        task_manager_destroy(g_task_manager);
        g_task_manager = nullptr;
        std::cout << "[OK] 任务管理器已销毁\n";
        
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] 程序异常: " << e.what() << std::endl;
        
        if (g_task_manager) {
            task_manager_destroy(g_task_manager);
            g_task_manager = nullptr;
        }
        
        return -1;
    }
    
    std::cout << "\n=== C++ 任务演示程序正常退出 ===\n";
    return 0;
}
