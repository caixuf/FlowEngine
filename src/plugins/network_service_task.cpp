#include "task_interface.h"
#include "process_interface.h"
#include <iostream>
#include <string>
#include <memory>
#include <thread>
#include <chrono>
#include <vector>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <random>
#include <cstring>

/**
 * 网络服务任务 - 模拟一个简单的网络服务
 * 演示C++高级特性：线程池、异步处理、RAII等
 */
class NetworkServiceTask {
public:
    struct Request {
        int id;
        std::string data;
        std::chrono::steady_clock::time_point timestamp;
        
        Request(int req_id, const std::string& req_data) 
            : id(req_id), data(req_data), timestamp(std::chrono::steady_clock::now()) {}
    };
    
    struct ServiceConfig {
        int port = 8080;
        int max_connections = 100;
        int worker_threads = 4;
        int request_timeout = 30; // seconds
    };

public:
    NetworkServiceTask() : running_(false), request_counter_(0) {}
    
    ~NetworkServiceTask() {
        stop();
    }
    
    bool initialize(const std::string& config_data, LogCallback log_cb) {
        log_callback_ = log_cb;
        log("初始化网络服务任务");
        
        // 解析配置
        parse_config(config_data);
        
        // 初始化工作线程池
        for (int i = 0; i < config_.worker_threads; ++i) {
            workers_.emplace_back([this, i] { worker_thread(i); });
        }
        
        log("网络服务初始化完成 - 端口: " + std::to_string(config_.port) + 
            ", 工作线程: " + std::to_string(config_.worker_threads));
        return true;
    }
    
    void start() {
        running_ = true;
        log("网络服务开始运行");
        
        // 主服务循环 - 模拟接收请求
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<> dis(100, 1000);
        
        while (running_) {
            // 模拟接收网络请求
            if (request_queue_.size() < static_cast<size_t>(config_.max_connections)) {
                int req_id = ++request_counter_;
                std::string req_data = "请求数据 " + std::to_string(req_id);
                
                {
                    std::lock_guard<std::mutex> lock(queue_mutex_);
                    request_queue_.emplace(req_id, req_data);
                }
                queue_cv_.notify_one();
                
                if (req_id % 10 == 0) {
                    log("已处理 " + std::to_string(req_id) + " 个请求, 当前队列长度: " + 
                        std::to_string(request_queue_.size()));
                }
            }
            
            // 随机延迟模拟网络请求间隔
            std::this_thread::sleep_for(std::chrono::milliseconds(dis(gen)));
            
            // 定期清理超时请求
            cleanup_timeout_requests();
        }
        
        log("网络服务主循环结束");
    }
    
    void stop() {
        if (!running_) return;
        
        log("停止网络服务...");
        running_ = false;
        
        // 通知所有工作线程退出
        queue_cv_.notify_all();
        
        // 等待所有工作线程结束
        for (auto& worker : workers_) {
            if (worker.joinable()) {
                worker.join();
            }
        }
        
        log("网络服务已停止");
    }
    
    bool health_check() const {
        // 检查队列是否过载
        if (request_queue_.size() > static_cast<size_t>(config_.max_connections * 0.8)) {
            return false;
        }
        
        // 检查是否有工作线程在运行
        return running_ && !workers_.empty();
    }
    
    std::string get_status() const {
        std::ostringstream oss;
        oss << "=== 网络服务状态 ===\n";
        oss << "运行状态: " << (running_ ? "运行中" : "已停止") << "\n";
        oss << "监听端口: " << config_.port << "\n";
        oss << "工作线程数: " << config_.worker_threads << "\n";
        oss << "最大连接数: " << config_.max_connections << "\n";
        oss << "当前队列长度: " << request_queue_.size() << "\n";
        oss << "已处理请求数: " << request_counter_.load() << "\n";
        oss << "已完成请求数: " << completed_requests_.load() << "\n";
        return oss.str();
    }

private:
    void parse_config(const std::string& config_data) {
        // 简单的配置解析（实际项目中应使用JSON库）
        if (config_data.find("port") != std::string::npos) {
            // 这里可以实现更复杂的配置解析
            // 为了演示，使用默认配置
        }
    }
    
    void worker_thread(int worker_id) {
        log("工作线程 " + std::to_string(worker_id) + " 启动");
        
        while (running_) {
            std::unique_lock<std::mutex> lock(queue_mutex_);
            
            // 等待请求或退出信号
            queue_cv_.wait(lock, [this] { 
                return !running_ || !request_queue_.empty(); 
            });
            
            if (!running_) break;
            
            if (request_queue_.empty()) continue;
            
            // 获取请求
            Request request = request_queue_.front();
            request_queue_.pop();
            lock.unlock();
            
            // 处理请求
            process_request(request, worker_id);
        }
        
        log("工作线程 " + std::to_string(worker_id) + " 退出");
    }
    
    void process_request(const Request& request, int worker_id) {
        // 模拟请求处理时间
        auto processing_time = std::chrono::milliseconds(50 + (request.id % 200));
        std::this_thread::sleep_for(processing_time);
        
        // 使用C++高级特性处理数据
        std::string response = process_data(request.data);
        
        // 记录处理结果
        ++completed_requests_;
        
        if (request.id % 50 == 0) {
            log("工作线程 " + std::to_string(worker_id) + " 处理请求 " + 
                std::to_string(request.id) + " 完成: " + response);
        }
    }
    
    std::string process_data(const std::string& input_data) {
        // 演示C++字符串处理和STL算法
        std::string processed = input_data;
        
        // 转换为大写
        std::transform(processed.begin(), processed.end(), processed.begin(), 
                      [](unsigned char c) { return std::toupper(c); });
        
        // 添加时间戳
        auto now = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now);
        
        std::ostringstream oss;
        oss << "已处理[" << std::put_time(std::localtime(&time_t), "%H:%M:%S") << "]: " << processed;
        
        return oss.str();
    }
    
    void cleanup_timeout_requests() {
        static auto last_cleanup = std::chrono::steady_clock::now();
        auto now = std::chrono::steady_clock::now();
        
        // 每30秒清理一次
        if (now - last_cleanup < std::chrono::seconds(30)) {
            return;
        }
        
        std::lock_guard<std::mutex> lock(queue_mutex_);
        auto timeout_threshold = now - std::chrono::seconds(config_.request_timeout);
        
        std::queue<Request> new_queue;
        int timeout_count = 0;
        
        while (!request_queue_.empty()) {
            const auto& request = request_queue_.front();
            if (request.timestamp > timeout_threshold) {
                new_queue.push(request);
            } else {
                timeout_count++;
            }
            request_queue_.pop();
        }
        
        request_queue_ = std::move(new_queue);
        last_cleanup = now;
        
        if (timeout_count > 0) {
            log("清理了 " + std::to_string(timeout_count) + " 个超时请求");
        }
    }
    
    void log(const std::string& message) {
        if (log_callback_) {
            std::string full_message = "[NetworkService] " + message;
            log_callback_(LOG_LEVEL_INFO, full_message.c_str());
        } else {
            std::cout << "[NetworkService] " << message << std::endl;
        }
    }

private:
    ServiceConfig config_;
    std::atomic<bool> running_;
    std::atomic<int> request_counter_;
    std::atomic<int> completed_requests_;
    
    std::queue<Request> request_queue_;
    std::mutex queue_mutex_;
    std::condition_variable queue_cv_;
    
    std::vector<std::thread> workers_;
    LogCallback log_callback_ = nullptr;
};

// ==============================================================================
// C接口包装
// ==============================================================================

struct NetworkServiceWrapper {
    TaskBase base;
    std::unique_ptr<NetworkServiceTask> service;
};

static int network_service_initialize(TaskBase* base_task) {
    auto* wrapper = reinterpret_cast<NetworkServiceWrapper*>(base_task);
    
    wrapper->service = std::make_unique<NetworkServiceTask>();
    
    std::string config_data;
    if (base_task->config.custom_config) {
        config_data = static_cast<const char*>(base_task->config.custom_config);
    }
    
    // 设置日志回调
    LogCallback log_cb = [](LogLevel level, const char* message) {
        std::cout << "[NET_LOG] " << message << std::endl;
    };
    
    return wrapper->service->initialize(config_data, log_cb) ? 0 : -1;
}

static int network_service_execute(TaskBase* base_task) {
    auto* wrapper = reinterpret_cast<NetworkServiceWrapper*>(base_task);
    
    if (!wrapper->service) {
        return -1;
    }
    
    try {
        wrapper->service->start();
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "网络服务执行异常: " << e.what() << std::endl;
        return -1;
    }
}

static void network_service_cleanup(TaskBase* base_task) {
    auto* wrapper = reinterpret_cast<NetworkServiceWrapper*>(base_task);
    
    if (wrapper->service) {
        wrapper->service->stop();
        wrapper->service.reset();
    }
}

static bool network_service_health_check(TaskBase* base_task) {
    auto* wrapper = reinterpret_cast<NetworkServiceWrapper*>(base_task);
    return wrapper->service ? wrapper->service->health_check() : false;
}

static int network_service_get_status(TaskBase* base_task, char* buffer, size_t size) {
    auto* wrapper = reinterpret_cast<NetworkServiceWrapper*>(base_task);
    
    if (!wrapper->service || !buffer || size == 0) {
        return -1;
    }
    
    std::string status = wrapper->service->get_status();
    strncpy(buffer, status.c_str(), size - 1);
    buffer[size - 1] = '\0';
    
    return static_cast<int>(status.length());
}

static const TaskInterface network_service_vtable = {
    .initialize = network_service_initialize,
    .execute = network_service_execute,
    .cleanup = network_service_cleanup,
    .pause = nullptr,
    .resume = nullptr,
    .handle_signal = nullptr,
    .health_check = network_service_health_check,
    .get_status = network_service_get_status
};

// ==============================================================================
// C导出函数
// ==============================================================================

extern "C" {

NetworkServiceWrapper* network_service_task_create(const TaskConfig* config) {
    if (!config) {
        return nullptr;
    }
    
    auto* wrapper = static_cast<NetworkServiceWrapper*>(malloc(sizeof(NetworkServiceWrapper)));
    if (!wrapper) {
        return nullptr;
    }
    
    if (task_base_init(&wrapper->base, &network_service_vtable, config) != 0) {
        free(wrapper);
        return nullptr;
    }
    
    new(&wrapper->service) std::unique_ptr<NetworkServiceTask>();
    
    return wrapper;
}

void network_service_task_destroy(NetworkServiceWrapper* wrapper) {
    if (!wrapper) {
        return;
    }
    
    wrapper->service.~unique_ptr<NetworkServiceTask>();
    task_base_destroy(&wrapper->base);
    free(wrapper);
}

TaskBase* network_service_task_get_base(NetworkServiceWrapper* wrapper) {
    return wrapper ? &wrapper->base : nullptr;
}

} // extern "C"
