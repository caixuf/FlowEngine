#include "task_interface.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <signal.h>
#include <stdarg.h>

/**
 * 示例任务结构体 - "继承"TaskBase
 * 通过将TaskBase作为第一个成员来实现"继承"
 */
typedef struct ExampleTask {
    TaskBase base;              // "基类" - 必须是第一个成员
    
    // "子类"特有的成员变量
    int work_interval;          // 工作间隔(秒)
    int counter;                // 计数器
    char message[256];          // 自定义消息
    bool use_random_delay;      // 是否使用随机延迟
} ExampleTask;

/**
 * 示例任务配置结构体
 */
typedef struct ExampleTaskConfig {
    int work_interval;
    bool use_random_delay;
    char message[256];
} ExampleTaskConfig;

// ============================================================================
// 实现虚函数 - 重写基类的虚函数
// ============================================================================

/**
 * 初始化函数 - 重写基类虚函数
 */
static int example_task_initialize(TaskBase* base_task) {
    // 向下转型为具体任务类型
    ExampleTask* task = TASK_CAST(ExampleTask, base_task);
    
    printf("初始化示例任务: %s\n", task->base.config.name);
    
    // 获取自定义配置
    ExampleTaskConfig* config = (ExampleTaskConfig*)task_get_custom_config(base_task);
    if (config) {
        task->work_interval = config->work_interval;
        task->use_random_delay = config->use_random_delay;
        snprintf(task->message, sizeof(task->message), "%s", config->message);
    } else {
        // 使用默认配置
        task->work_interval = 5;
        task->use_random_delay = false;
        strcpy(task->message, "Hello from example task!");
    }
    
    task->counter = 0;
    
    printf("示例任务初始化完成，工作间隔: %d秒\n", task->work_interval);
    return 0;
}

/**
 * 执行函数 - 重写基类虚函数(任务主循环)
 */
static int example_task_execute(TaskBase* base_task) {
    ExampleTask* task = TASK_CAST(ExampleTask, base_task);
    
    printf("示例任务开始执行: %s\n", task->base.config.name);
    
    while (!task_should_stop(base_task)) {
        // 模拟工作
        task->counter++;
        
        // 更新心跳
        task_update_heartbeat(base_task);
        
        // 更新统计信息
        pthread_mutex_lock(&task->base.mutex);
        task->base.stats.cpu_usage = 10 + (task->counter % 20); // 模拟CPU使用率
        task->base.stats.memory_usage = 1024 * 1024 * (2 + (task->counter % 5)); // 模拟内存使用
        pthread_mutex_unlock(&task->base.mutex);
        
        printf("[%s] 执行第 %d 次: %s\n", 
               task->base.config.name, task->counter, task->message);
        
        // 模拟工作延迟
        int delay = task->work_interval;
        if (task->use_random_delay) {
            delay += (rand() % 3); // 随机增加0-2秒
        }
        
        // 分段休眠，以便及时响应停止信号
        for (int i = 0; i < delay && !task_should_stop(base_task); i++) {
            sleep(1);
        }
    }
    
    printf("示例任务执行完成: %s，总执行次数: %d\n", 
           task->base.config.name, task->counter);
    
    return 0;
}

/**
 * 清理函数 - 重写基类虚函数
 */
static void example_task_cleanup(TaskBase* base_task) {
    ExampleTask* task = TASK_CAST(ExampleTask, base_task);
    
    printf("清理示例任务: %s\n", task->base.config.name);
    
    // 这里可以清理任务特定的资源
    // 例如：关闭文件、释放内存等
    task->counter = 0;
    
    printf("示例任务清理完成\n");
}

/**
 * 暂停函数 - 重写基类虚函数
 */
static int example_task_pause(TaskBase* base_task) {
    ExampleTask* task = TASK_CAST(ExampleTask, base_task);
    
    printf("暂停示例任务: %s\n", task->base.config.name);
    
    // 这里可以实现任务特定的暂停逻辑
    // 例如：保存当前状态、暂停定时器等
    
    return 0;
}

/**
 * 恢复函数 - 重写基类虚函数  
 */
static int example_task_resume(TaskBase* base_task) {
    ExampleTask* task = TASK_CAST(ExampleTask, base_task);
    
    printf("恢复示例任务: %s\n", task->base.config.name);
    
    // 这里可以实现任务特定的恢复逻辑
    // 例如：恢复状态、重启定时器等
    
    return 0;
}

/**
 * 信号处理函数 - 重写基类虚函数
 */
static void example_task_handle_signal(TaskBase* base_task, int signal) {
    ExampleTask* task = TASK_CAST(ExampleTask, base_task);
    
    printf("示例任务 %s 收到信号: %d\n", task->base.config.name, signal);
    
    // 调用默认的信号处理
    // 或者实现自定义信号处理逻辑
    if (signal == SIGTERM || signal == SIGINT) {
        printf("示例任务收到终止信号，准备停止\n");
        pthread_mutex_lock(&task->base.mutex);
        task->base.should_stop = true;
        pthread_mutex_unlock(&task->base.mutex);
    }
}

/**
 * 健康检查函数 - 重写基类虚函数
 */
static bool example_task_health_check(TaskBase* base_task) {
    ExampleTask* task = TASK_CAST(ExampleTask, base_task);
    
    // 实现任务特定的健康检查逻辑
    pthread_mutex_lock(&task->base.mutex);
    
    bool healthy = true;
    
    // 检查任务是否卡死(例如：长时间没有更新计数器)
    uint64_t current_time = time(NULL);
    if (task->base.state == TASK_STATE_RUNNING) {
        // 如果超过工作间隔的3倍时间没有心跳，认为不健康
        if (current_time - task->base.stats.last_heartbeat > (uint64_t)(task->work_interval * 3)) {
            healthy = false;
            printf("示例任务 %s 心跳超时，可能已卡死\n", task->base.config.name);
        }
    }
    
    pthread_mutex_unlock(&task->base.mutex);
    
    return healthy;
}

/**
 * 获取状态函数 - 重写基类虚函数
 */
static int example_task_get_status(TaskBase* base_task, char* buffer, size_t size) {
    ExampleTask* task = TASK_CAST(ExampleTask, base_task);
    
    if (!buffer || size == 0) {
        return -1;
    }
    
    pthread_mutex_lock(&task->base.mutex);
    
    int written = snprintf(buffer, size,
        "=== 示例任务状态 ===\n"
        "任务名称: %s\n"
        "状态: %s\n"
        "执行次数: %d\n"
        "工作间隔: %d秒\n"
        "使用随机延迟: %s\n"
        "消息: %s\n"
        "运行时间: %lu秒\n"
        "CPU使用率: %u%%\n"
        "内存使用: %lu bytes\n"
        "最后心跳: %lu\n",
        task->base.config.name,
        task->base.state == TASK_STATE_RUNNING ? "运行中" : "已停止",
        task->counter,
        task->work_interval,
        task->use_random_delay ? "是" : "否",
        task->message,
        task->base.stats.total_run_time,
        task->base.stats.cpu_usage,
        task->base.stats.memory_usage,
        task->base.stats.last_heartbeat
    );
    
    pthread_mutex_unlock(&task->base.mutex);
    
    return written;
}

// ============================================================================
// 虚函数表 - 定义具体任务的虚函数表
// ============================================================================
static const TaskInterface example_task_vtable = {
    .initialize = example_task_initialize,
    .execute = example_task_execute,
    .cleanup = example_task_cleanup,
    .pause = example_task_pause,
    .resume = example_task_resume,
    .handle_signal = example_task_handle_signal,
    .health_check = example_task_health_check,
    .get_status = example_task_get_status
};

// ============================================================================
// 公共接口 - 任务的"构造函数"和"析构函数"
// ============================================================================

/**
 * 创建示例任务 - "构造函数"
 * @param config 任务配置
 * @param custom_config 自定义配置
 * @return 任务指针，失败返回NULL
 */
ExampleTask* example_task_create(const TaskConfig* config, const ExampleTaskConfig* custom_config) {
    if (!config) {
        return NULL;
    }
    
    // 分配内存
    ExampleTask* task = malloc(sizeof(ExampleTask));
    if (!task) {
        return NULL;
    }
    
    // 初始化基类
    if (task_base_init(&task->base, &example_task_vtable, config) != 0) {
        free(task);
        return NULL;
    }
    
    // 设置自定义配置
    if (custom_config) {
        ExampleTaskConfig* config_copy = malloc(sizeof(ExampleTaskConfig));
        if (config_copy) {
            *config_copy = *custom_config;
            task_set_custom_config(&task->base, config_copy);
        }
    }
    
    return task;
}

/**
 * 销毁示例任务 - "析构函数"
 * @param task 任务指针
 */
void example_task_destroy(ExampleTask* task) {
    if (!task) {
        return;
    }
    
    // 释放自定义配置
    ExampleTaskConfig* custom_config = (ExampleTaskConfig*)task_get_custom_config(&task->base);
    if (custom_config) {
        free(custom_config);
    }
    
    // 销毁基类
    task_base_destroy(&task->base);
    
    // 释放任务内存
    free(task);
}

/**
 * 获取任务基类指针 - 用于向上转型
 * @param task 示例任务指针
 * @return 基类指针
 */
TaskBase* example_task_get_base(ExampleTask* task) {
    return task ? &task->base : NULL;
}
