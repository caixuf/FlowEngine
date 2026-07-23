#include "task_interface.h"
#include "process_interface.h" // LogLevel和LogCallback定义
#include <iostream>
#include <string>
#include <memory>
#include <thread>
#include <chrono>
#include <atomic>
#include <vector>
#include <mutex>
#include <cstring>

/**
 * 简化的C++任务基类
 */
class SimpleCppTask {
public:
    SimpleCppTask() : running_(false), iteration_count_(0) {}
    
    virtual ~SimpleCppTask() {
        stop();
    }
    
    virtual bool initialize(const std::string& config_data) {
        std::cout << "[CppTask] 初始化任务: " << config_data << std::endl;
        return true;
    }
    
    virtual void start() {
        running_ = true;
        std::cout << "[CppTask] 开始执行任务" << std::endl;
        
        // 简单的计数循环
        while (running_ && iteration_count_ < 100) {
            std::cout << "[CppTask] 执行迭代 " << (iteration_count_ + 1) << "/100" << std::endl;
            
            // 模拟工作
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            iteration_count_++;
        }
        
        std::cout << "[CppTask] 任务执行完成，共执行 " << iteration_count_ << " 次迭代" << std::endl;
    }
    
    virtual void stop() {
        if (running_) {
            running_ = false;
            std::cout << "[CppTask] 停止任务" << std::endl;
        }
    }
    
    virtual bool health_check() const {
        return running_;
    }
    
    virtual std::string get_status() const {
        return std::string("C++ Task - Running: ") + 
               (running_ ? "Yes" : "No") + 
               ", Iterations: " + std::to_string(iteration_count_);
    }

private:
    std::atomic<bool> running_;
    std::atomic<int> iteration_count_;
};

// ==============================================================================
// C接口包装
// ==============================================================================

struct SimpleCppTaskWrapper {
    TaskBase base;
    std::unique_ptr<SimpleCppTask> task;
};

static int simple_cpp_task_initialize(TaskBase* base_task) {
    auto* wrapper = reinterpret_cast<SimpleCppTaskWrapper*>(base_task);
    
    wrapper->task = std::make_unique<SimpleCppTask>();
    
    std::string config_data;
    if (base_task->config.custom_config) {
        config_data = static_cast<const char*>(base_task->config.custom_config);
    }
    
    return wrapper->task->initialize(config_data) ? 0 : -1;
}

static int simple_cpp_task_execute(TaskBase* base_task) {
    auto* wrapper = reinterpret_cast<SimpleCppTaskWrapper*>(base_task);
    
    if (!wrapper->task) {
        return -1;
    }
    
    try {
        wrapper->task->start();
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "C++任务执行异常: " << e.what() << std::endl;
        return -1;
    }
}

static void simple_cpp_task_cleanup(TaskBase* base_task) {
    auto* wrapper = reinterpret_cast<SimpleCppTaskWrapper*>(base_task);
    
    if (wrapper->task) {
        wrapper->task->set_stop();
        wrapper->task.reset();
    }
}

static bool simple_cpp_task_health_check(TaskBase* base_task) {
    auto* wrapper = reinterpret_cast<SimpleCppTaskWrapper*>(base_task);
    return wrapper->task ? wrapper->task->health_check() : false;
}

static int simple_cpp_task_get_status(TaskBase* base_task, char* buffer, size_t size) {
    auto* wrapper = reinterpret_cast<SimpleCppTaskWrapper*>(base_task);
    
    if (!wrapper->task || !buffer || size == 0) {
        return -1;
    }
    
    std::string status = wrapper->task->get_status();
    strncpy(buffer, status.c_str(), size - 1);
    buffer[size - 1] = '\0';
    
    return static_cast<int>(status.length());
}

static const TaskInterface simple_cpp_task_vtable = {
    .initialize = simple_cpp_task_initialize,
    .execute = simple_cpp_task_execute,
    .cleanup = simple_cpp_task_cleanup,
    .pause = nullptr,
    .resume = nullptr,
    .handle_signal = nullptr,
    .health_check = simple_cpp_task_health_check,
    .get_status = simple_cpp_task_get_status,
    .on_message = nullptr
};

// ==============================================================================
// C导出函数
// ==============================================================================

extern "C" {

// 使用CppTaskWrapper作为类型别名
typedef SimpleCppTaskWrapper CppTaskWrapper;

CppTaskWrapper* cpp_task_create(const TaskConfig* config) {
    if (!config) {
        return nullptr;
    }
    
    auto* wrapper = static_cast<CppTaskWrapper*>(malloc(sizeof(CppTaskWrapper)));
    if (!wrapper) {
        return nullptr;
    }
    
    if (task_base_init(&wrapper->base, &simple_cpp_task_vtable, config) != 0) {
        free(wrapper);
        return nullptr;
    }
    
    new(&wrapper->task) std::unique_ptr<SimpleCppTask>();
    
    return wrapper;
}

void cpp_task_destroy(CppTaskWrapper* wrapper) {
    if (!wrapper) {
        return;
    }
    
    wrapper->task.~unique_ptr<SimpleCppTask>();
    task_base_destroy(&wrapper->base);
    free(wrapper);
}

TaskBase* cpp_task_get_base(CppTaskWrapper* wrapper) {
    return wrapper ? &wrapper->base : nullptr;
}

} // extern "C"
