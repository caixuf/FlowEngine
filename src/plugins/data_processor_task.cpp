#include "task_interface.h"
#include "process_interface.h"
#include <iostream>
#include <string>
#include <memory>
#include <thread>
#include <chrono>
#include <vector>
#include <map>
#include <unordered_map>
#include <set>
#include <queue>
#include <deque>
#include <algorithm>
#include <numeric>
#include <functional>
#include <future>
#include <atomic>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <random>
#include <mutex>
#include <shared_mutex>
#include <cstring>

/**
 * 数据处理任务 - 演示C++高级特性
 * 包含：模板编程、lambda表达式、智能指针、STL容器、并发编程等
 */
class DataProcessorTask {
public:
    // 数据记录结构
    struct DataRecord {
        int id;
        double value;
        std::string category;
        std::chrono::system_clock::time_point timestamp;
        std::map<std::string, std::string> metadata;
        
        DataRecord(int record_id, double record_value, const std::string& cat)
            : id(record_id), value(record_value), category(cat)
            , timestamp(std::chrono::system_clock::now()) {}
    };
    
    // 统计信息
    struct Statistics {
        size_t total_count = 0;
        double sum = 0.0;
        double mean = 0.0;
        double min_value = std::numeric_limits<double>::max();
        double max_value = std::numeric_limits<double>::lowest();
        std::unordered_map<std::string, size_t> category_counts;
    };
    
    // 数据过滤器类型
    using DataFilter = std::function<bool(const DataRecord&)>;
    using DataProcessor = std::function<DataRecord(const DataRecord&)>;

public:
    DataProcessorTask() : running_(false), process_counter_(0) {}
    
    ~DataProcessorTask() {
        stop();
    }
    
    bool initialize(const std::string& config_data, LogCallback log_cb) {
        log_callback_ = log_cb;
        log("初始化数据处理任务");
        
        // 初始化随机数生成器
        std::random_device rd;
        generator_.seed(rd());
        
        // 设置预定义的数据过滤器
        setup_filters();
        
        // 设置预定义的数据处理器
        setup_processors();
        
        log("数据处理任务初始化完成");
        return true;
    }
    
    void start() {
        running_ = true;
        log("数据处理任务开始运行");
        
        // 启动数据生成线程
        data_generator_thread_ = std::thread([this] { data_generator_loop(); });
        
        // 启动数据处理线程
        data_processor_thread_ = std::thread([this] { data_processor_loop(); });
        
        // 启动统计分析线程
        statistics_thread_ = std::thread([this] { statistics_loop(); });
        
        // 主监控循环
        main_monitoring_loop();
    }
    
    void stop() {
        if (!running_) return;
        
        log("停止数据处理任务...");
        running_ = false;
        
        // 等待所有线程结束
        if (data_generator_thread_.joinable()) {
            data_generator_thread_.join();
        }
        if (data_processor_thread_.joinable()) {
            data_processor_thread_.join();
        }
        if (statistics_thread_.joinable()) {
            statistics_thread_.join();
        }
        
        log("数据处理任务已停止");
    }
    
    bool health_check() const {
        std::shared_lock<std::shared_mutex> lock(data_mutex_);
        
        // 检查数据队列是否过载
        if (raw_data_queue_.size() > 10000) {
            return false;
        }
        
        // 检查处理速度
        auto now = std::chrono::steady_clock::now();
        static auto last_check = now;
        static size_t last_count = process_counter_.load();
        
        if (now - last_check > std::chrono::minutes(1)) {
            size_t current_count = process_counter_.load();
            size_t processed_per_minute = current_count - last_count;
            
            last_check = now;
            last_count = current_count;
            
            // 如果每分钟处理少于10个数据，认为不健康
            if (processed_per_minute < 10) {
                return false;
            }
        }
        
        return running_;
    }
    
    std::string get_status() const {
        std::shared_lock<std::shared_mutex> lock(data_mutex_);
        
        std::ostringstream oss;
        oss << "=== 数据处理任务状态 ===\n";
        oss << "运行状态: " << (running_ ? "运行中" : "已停止") << "\n";
        oss << "原始数据队列: " << raw_data_queue_.size() << "\n";
        oss << "已处理数据: " << processed_data_.size() << "\n";
        oss << "处理计数器: " << process_counter_.load() << "\n";
        
        // 统计信息
        if (!current_statistics_.category_counts.empty()) {
            oss << "\n=== 当前统计 ===\n";
            oss << "总数量: " << current_statistics_.total_count << "\n";
            oss << "平均值: " << std::fixed << std::setprecision(2) 
                << current_statistics_.mean << "\n";
            oss << "最小值: " << current_statistics_.min_value << "\n";
            oss << "最大值: " << current_statistics_.max_value << "\n";
            
            oss << "\n分类统计:\n";
            for (const auto& [category, count] : current_statistics_.category_counts) {
                oss << "  " << category << ": " << count << "\n";
            }
        }
        
        return oss.str();
    }
    
    // 添加自定义数据过滤器
    void add_filter(const std::string& name, DataFilter filter) {
        std::unique_lock<std::shared_mutex> lock(data_mutex_);
        filters_[name] = std::move(filter);
        log("添加数据过滤器: " + name);
    }
    
    // 添加自定义数据处理器
    void add_processor(const std::string& name, DataProcessor processor) {
        std::unique_lock<std::shared_mutex> lock(data_mutex_);
        processors_[name] = std::move(processor);
        log("添加数据处理器: " + name);
    }

private:
    void setup_filters() {
        // 值范围过滤器
        filters_["value_positive"] = [](const DataRecord& record) {
            return record.value > 0.0;
        };
        
        // 分类过滤器
        filters_["category_ABC"] = [](const DataRecord& record) {
            return record.category == "A" || record.category == "B" || record.category == "C";
        };
        
        // 时间过滤器（最近1小时）
        filters_["recent_hour"] = [](const DataRecord& record) {
            auto now = std::chrono::system_clock::now();
            auto hour_ago = now - std::chrono::hours(1);
            return record.timestamp > hour_ago;
        };
        
        // ID奇偶性过滤器
        filters_["even_id"] = [](const DataRecord& record) {
            return record.id % 2 == 0;
        };
    }
    
    void setup_processors() {
        // 值归一化处理器
        processors_["normalize"] = [](DataRecord record) {
            record.value = std::tanh(record.value / 100.0);  // 归一化到[-1,1]
            record.metadata["processed_by"] = "normalize";
            return record;
        };
        
        // 值平方处理器
        processors_["square"] = [](DataRecord record) {
            record.value = record.value * record.value;
            record.metadata["processed_by"] = "square";
            return record;
        };
        
        // 分类转换处理器
        processors_["categorize"] = [](DataRecord record) {
            if (record.value > 50.0) record.category = "HIGH";
            else if (record.value > 0.0) record.category = "MEDIUM";
            else record.category = "LOW";
            record.metadata["processed_by"] = "categorize";
            return record;
        };
        
        // 元数据增强处理器
        processors_["enhance_metadata"] = [](DataRecord record) {
            record.metadata["processing_time"] = std::to_string(
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::system_clock::now().time_since_epoch()).count()
            );
            record.metadata["processed_by"] = "enhance_metadata";
            return record;
        };
    }
    
    void data_generator_loop() {
        log("数据生成线程启动");
        
        std::uniform_real_distribution<double> value_dist(-100.0, 100.0);
        std::uniform_int_distribution<int> category_dist(0, 4);
        std::vector<std::string> categories = {"A", "B", "C", "D", "E"};
        
        int id_counter = 1;
        
        while (running_) {
            // 生成随机数据
            double value = value_dist(generator_);
            std::string category = categories[category_dist(generator_)];
            
            DataRecord record(id_counter++, value, category);
            
            // 添加一些随机元数据
            record.metadata["source"] = "generator";
            record.metadata["batch"] = std::to_string((id_counter - 1) / 100);
            
            // 将数据添加到队列
            {
                std::unique_lock<std::shared_mutex> lock(data_mutex_);
                raw_data_queue_.push_back(record);
                
                // 限制队列大小
                if (raw_data_queue_.size() > 5000) {
                    raw_data_queue_.pop_front();
                }
            }
            
            // 控制生成速度
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            
            if (id_counter % 1000 == 0) {
                log("已生成 " + std::to_string(id_counter) + " 条数据记录");
            }
        }
        
        log("数据生成线程退出");
    }
    
    void data_processor_loop() {
        log("数据处理线程启动");
        
        while (running_) {
            std::vector<DataRecord> batch;
            
            // 获取一批数据进行处理
            {
                std::unique_lock<std::shared_mutex> lock(data_mutex_);
                size_t batch_size = std::min(static_cast<size_t>(50), raw_data_queue_.size());
                
                for (size_t i = 0; i < batch_size; ++i) {
                    batch.push_back(raw_data_queue_.front());
                    raw_data_queue_.pop_front();
                }
            }
            
            if (batch.empty()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;
            }
            
            // 并行处理数据批次
            process_data_batch(batch);
            
            process_counter_ += batch.size();
        }
        
        log("数据处理线程退出");
    }
    
    void process_data_batch(const std::vector<DataRecord>& batch) {
        std::vector<std::future<std::vector<DataRecord>>> futures;
        
        // 将批次分割成更小的子批次进行并行处理
        const size_t sub_batch_size = 10;
        size_t num_sub_batches = (batch.size() + sub_batch_size - 1) / sub_batch_size;
        
        for (size_t i = 0; i < num_sub_batches; ++i) {
            size_t start = i * sub_batch_size;
            size_t end = std::min(start + sub_batch_size, batch.size());
            
            std::vector<DataRecord> sub_batch(batch.begin() + start, batch.begin() + end);
            
            // 异步处理子批次
            auto future = std::async(std::launch::async, 
                [this](std::vector<DataRecord> data) {
                    return process_sub_batch(std::move(data));
                }, std::move(sub_batch));
            
            futures.push_back(std::move(future));
        }
        
        // 收集所有处理结果
        std::vector<DataRecord> processed_batch;
        for (auto& future : futures) {
            auto result = future.get();
            processed_batch.insert(processed_batch.end(), result.begin(), result.end());
        }
        
        // 保存处理后的数据
        {
            std::unique_lock<std::shared_mutex> lock(data_mutex_);
            processed_data_.insert(processed_data_.end(), 
                                 processed_batch.begin(), processed_batch.end());
            
            // 限制处理后数据的大小
            if (processed_data_.size() > 10000) {
                processed_data_.erase(processed_data_.begin(), 
                                    processed_data_.begin() + 5000);
            }
        }
    }
    
    std::vector<DataRecord> process_sub_batch(std::vector<DataRecord> data) {
        std::vector<DataRecord> result;
        
        for (auto& record : data) {
            // 应用所有过滤器
            bool passes_filters = true;
            for (const auto& [name, filter] : filters_) {
                if (!filter(record)) {
                    passes_filters = false;
                    break;
                }
            }
            
            if (!passes_filters) {
                continue;
            }
            
            // 应用所有处理器
            DataRecord processed_record = record;
            for (const auto& [name, processor] : processors_) {
                processed_record = processor(processed_record);
            }
            
            result.push_back(processed_record);
        }
        
        return result;
    }
    
    void statistics_loop() {
        log("统计分析线程启动");
        
        while (running_) {
            calculate_statistics();
            std::this_thread::sleep_for(std::chrono::seconds(30));
        }
        
        log("统计分析线程退出");
    }
    
    void calculate_statistics() {
        std::shared_lock<std::shared_mutex> lock(data_mutex_);
        
        if (processed_data_.empty()) {
            return;
        }
        
        Statistics stats;
        stats.total_count = processed_data_.size();
        
        // 使用STL算法计算统计信息
        for (const auto& record : processed_data_) {
            stats.sum += record.value;
            if (record.value < stats.min_value) stats.min_value = record.value;
            if (record.value > stats.max_value) stats.max_value = record.value;
        }
        stats.mean = stats.sum / stats.total_count;
        
        // 计算分类统计
        for (const auto& record : processed_data_) {
            stats.category_counts[record.category]++;
        }
        
        current_statistics_ = stats;
        
        static int stats_counter = 0;
        if (++stats_counter % 10 == 0) {
            log("统计信息更新 - 总数: " + std::to_string(stats.total_count) + 
                ", 平均值: " + std::to_string(stats.mean));
        }
    }
    
    void main_monitoring_loop() {
        log("主监控循环启动");
        
        auto last_report = std::chrono::steady_clock::now();
        
        while (running_) {
            auto now = std::chrono::steady_clock::now();
            
            // 每分钟输出一次状态报告
            if (now - last_report > std::chrono::minutes(1)) {
                auto status = get_status();
                log("\n" + status);
                last_report = now;
            }
            
            std::this_thread::sleep_for(std::chrono::seconds(10));
        }
        
        log("主监控循环结束");
    }
    
    void log(const std::string& message) {
        if (log_callback_) {
            std::string full_message = "[DataProcessor] " + message;
            log_callback_(LOG_LEVEL_INFO, full_message.c_str());
        } else {
            std::cout << "[DataProcessor] " << message << std::endl;
        }
    }

private:
    std::atomic<bool> running_;
    std::atomic<size_t> process_counter_;
    
    mutable std::shared_mutex data_mutex_;
    std::deque<DataRecord> raw_data_queue_;
    std::vector<DataRecord> processed_data_;
    
    std::unordered_map<std::string, DataFilter> filters_;
    std::unordered_map<std::string, DataProcessor> processors_;
    
    Statistics current_statistics_;
    
    std::thread data_generator_thread_;
    std::thread data_processor_thread_;
    std::thread statistics_thread_;
    
    std::mt19937 generator_;
    LogCallback log_callback_ = nullptr;
};

// ==============================================================================
// C接口包装
// ==============================================================================

struct DataProcessorWrapper {
    TaskBase base;
    std::unique_ptr<DataProcessorTask> processor;
};

static int data_processor_initialize(TaskBase* base_task) {
    auto* wrapper = reinterpret_cast<DataProcessorWrapper*>(base_task);
    
    wrapper->processor = std::make_unique<DataProcessorTask>();
    
    std::string config_data;
    if (base_task->config.custom_config) {
        config_data = static_cast<const char*>(base_task->config.custom_config);
    }
    
    LogCallback log_cb = [](LogLevel level, const char* message) {
        std::cout << "[DATA_LOG] " << message << std::endl;
    };
    
    return wrapper->processor->initialize(config_data, log_cb) ? 0 : -1;
}

static int data_processor_execute(TaskBase* base_task) {
    auto* wrapper = reinterpret_cast<DataProcessorWrapper*>(base_task);
    
    if (!wrapper->processor) {
        return -1;
    }
    
    try {
        wrapper->processor->start();
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "数据处理任务执行异常: " << e.what() << std::endl;
        return -1;
    }
}

static void data_processor_cleanup(TaskBase* base_task) {
    auto* wrapper = reinterpret_cast<DataProcessorWrapper*>(base_task);
    
    if (wrapper->processor) {
        wrapper->processor->stop();
        wrapper->processor.reset();
    }
}

static bool data_processor_health_check(TaskBase* base_task) {
    auto* wrapper = reinterpret_cast<DataProcessorWrapper*>(base_task);
    return wrapper->processor ? wrapper->processor->health_check() : false;
}

static int data_processor_get_status(TaskBase* base_task, char* buffer, size_t size) {
    auto* wrapper = reinterpret_cast<DataProcessorWrapper*>(base_task);
    
    if (!wrapper->processor || !buffer || size == 0) {
        return -1;
    }
    
    std::string status = wrapper->processor->get_status();
    strncpy(buffer, status.c_str(), size - 1);
    buffer[size - 1] = '\0';
    
    return static_cast<int>(status.length());
}

static const TaskInterface data_processor_vtable = {
    .initialize = data_processor_initialize,
    .execute = data_processor_execute,
    .cleanup = data_processor_cleanup,
    .pause = nullptr,
    .resume = nullptr,
    .handle_signal = nullptr,
    .health_check = data_processor_health_check,
    .get_status = data_processor_get_status
};

// ==============================================================================
// C导出函数
// ==============================================================================

extern "C" {

DataProcessorWrapper* data_processor_task_create(const TaskConfig* config) {
    if (!config) {
        return nullptr;
    }
    
    auto* wrapper = static_cast<DataProcessorWrapper*>(malloc(sizeof(DataProcessorWrapper)));
    if (!wrapper) {
        return nullptr;
    }
    
    if (task_base_init(&wrapper->base, &data_processor_vtable, config) != 0) {
        free(wrapper);
        return nullptr;
    }
    
    new(&wrapper->processor) std::unique_ptr<DataProcessorTask>();
    
    return wrapper;
}

void data_processor_task_destroy(DataProcessorWrapper* wrapper) {
    if (!wrapper) {
        return;
    }
    
    wrapper->processor.~unique_ptr<DataProcessorTask>();
    task_base_destroy(&wrapper->base);
    free(wrapper);
}

TaskBase* data_processor_task_get_base(DataProcessorWrapper* wrapper) {
    return wrapper ? &wrapper->base : nullptr;
}

} // extern "C"
