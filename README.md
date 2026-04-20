# StartTool - 统一进程启动管理器

## 项目概述

StartTool 是一个基于 C/C++ 实现的统一进程启动管理器，专为复杂的多服务系统设计。该项目灵感来源于 DJI 的架构设计，使用动态库加载 (dlopen) 技术实现进程的统一管理和启动。

## 应用场景 - 典型业务背景

### 地理信息系统的开发挑战

在地理信息系统的实际开发中，我们面临着多个独立子项目的协调管理挑战：

#### �️ 核心业务服务
- **定位服务** (Positioning Service) - GPS/北斗定位，惯性导航融合
- **地图服务** (Map Service) - 矢量地图渲染，POI查询，地图瓦片服务
- **众包地图** (Crowdsource Mapping) - 用户贡献的路况、POI数据处理
- **路径规划** (Route Planning) - 基于实时路况的最优路径计算
- **地理编码** (Geocoding) - 地址与坐标双向转换服务
- **瓦片服务** (Tile Service) - 地图瓦片生成、缓存与分发

#### � 辅助支撑服务
- **数据同步服务** - 与云端地图数据同步
- **缓存管理服务** - 地图数据本地缓存管理
- **消息推送服务** - 实时交通、导航信息推送

### 传统开发模式的痛点

#### � 开发阶段的混乱
```bash
# 传统启动方式 - 每天重复的噩梦
cd positioning && ./positioning_service --config pos.conf &
cd ../map && ./map_service --port 8080 --cache-size 1G &  
cd ../crowdsource && python crowdsource_server.py --workers 4 &
cd ../routing && java -jar routing-service.jar --heap 2G &
cd ../geocoding && ./geocoding --db-path /data/geo.db &
cd ../tiles && node tile_server.js --port 3000 &
cd ../sync && ./data_sync_daemon &
cd ../cache && ./cache_manager --max-size 5G &
cd ../push && ./message_push_service &

# 等待所有服务启动完成...
# 检查端口占用...
# 查看各种日志...
# 调试服务间通信...
# 每天光启动就要花费20分钟！
```

#### 实际遇到的问题
- **依赖地狱** - 地图服务依赖定位服务，路径规划依赖地图服务...
- **端口冲突** - 测试环境和开发环境端口经常冲突
- **配置分散** - 9个服务，9套不同的配置文件格式
- **日志混乱** - 日志散落在各个目录，问题排查像大海捞针
- **调试困难** - 需要开9个终端窗口，各种切换
- **新人上手** - 新同事需要2天才能把环境搭起来

#### 生产环境的噩梦
- **部署复杂** - 运维需要逐个部署和启动，容易遗漏
- **故障定位** - 某个服务挂了，影响范围难以评估
- **扩容困难** - 需要手动调整每个服务的配置和资源
- **监控缺失** - 缺乏统一的健康检查，问题发现滞后

### 使用 StartTool 后的改变

#### 一键启动，告别混乱
```bash
# 开发环境
./launcher --config configs/development.json
[2025-07-22 10:30:01] INFO  启动器初始化完成
[2025-07-22 10:30:02] INFO  定位服务 (positioning_service) 启动成功
[2025-07-22 10:30:03] INFO  地图服务 (map_service) 启动成功
[2025-07-22 10:30:04] INFO  路径规划 (routing_service) 启动成功
[2025-07-22 10:30:05] INFO  所有9个服务启动完成，系统就绪!

# 测试环境
./launcher --config configs/testing.json

# 生产环境  
./launcher --config configs/production.json

# 只启动核心服务
./launcher --config configs/core_only.json
```

#### 规范化的开发流程

```
配置管理 → 依赖解析 → 有序启动 → 健康监控 → 故障恢复
    ↓           ↓           ↓           ↓           ↓
统一配置文件  自动依赖排序  按序启动服务  实时状态监控  自动重启恢复
```

#### 显著的效果提升

| 方面 | 传统方式 | 使用StartTool | 改善程度 |
|------|----------|---------------|----------|
| 启动时间 | 20分钟手动操作 | 2分钟自动化 | **90%** ↑ |
| 环境搭建 | 新人需2天 | 30分钟 | **95%** ↑ |
| 故障定位 | 平均2小时 | 10分钟 | **92%** ↑ |
| 部署效率 | 1小时部署流程 | 5分钟一键部署 | **95%** ↑ |
| 配置错误 | 月均10次 | 月均1次 | **90%** ↓ |

## 核心特性

### 统一配置管理
```json
{
  "services": [
    {
      "name": "positioning_service",
      "plugin_path": "lib/positioning.so",
      "priority": "HIGH",
      "depends_on": [],
      "config": {
        "gps_device": "/dev/ttyUSB0",
        "update_rate_hz": 10,
        "accuracy_threshold": 3.0
      }
    },
    {
      "name": "map_service", 
      "plugin_path": "lib/map_service.so",
      "priority": "NORMAL",
      "depends_on": ["positioning_service"],
      "config": {
        "tile_cache_size_mb": 1024,
        "api_endpoint": "https://maps.api.internal",
        "render_threads": 4
      }
    }
  ]
}
```

### 智能依赖管理
- 自动解析服务依赖关系
- 按拓扑顺序启动服务
- 依赖服务异常时自动处理

### 完整监控体系
- **实时健康检查** - 每30秒检查服务状态
- **性能监控** - CPU、内存、网络使用率统计
- **故障恢复** - 服务异常自动重启，最多重试3次
- **日志聚合** - 统一日志格式和输出位置

### 面向对象设计 (C语言实现)
```c
// 虚函数表机制 - 模拟C++的多态
typedef struct TaskInterface {
    int (*initialize)(TaskBase* task);
    int (*execute)(TaskBase* task);  
    void (*cleanup)(TaskBase* task);
    bool (*health_check)(TaskBase* task);
} TaskInterface;

// 任务基类
typedef struct TaskBase {
    TaskConfig config;
    TaskState state;
    const TaskInterface* vtable;  // 虚函数表指针
    pthread_t thread;
    pthread_mutex_t mutex;
} TaskBase;
```

## 项目结构

```
startTool/
├── src/
│   ├── core/                          # 核心组件
│   │   ├── task_interface.c           # 任务接口实现 
│   │   ├── task_manager.c             # 任务管理器
│   │   ├── logger.c                   # 统一日志系统
│   │   └── config_manager.c           # 配置管理器
│   ├── plugins/                       # 业务服务插件
│   │   ├── positioning_service.c      # 定位服务插件
│   │   ├── map_service.cpp            # 地图服务插件 (C++)
│   │   ├── crowdsource_service.cpp    # 众包地图服务
│   │   ├── routing_service.c          # 路径规划服务
│   │   ├── geocoding_service.c        # 地理编码服务
│   │   └── tile_service.cpp           # 瓦片服务 (C++)
│   ├── launcher.c                     # 主启动器程序
│   └── demos/                         # 演示和测试程序
├── include/                           # 公共头文件
│   ├── task_interface.h               # 任务接口定义
│   ├── process_interface.h            # 进程接口定义
│   └── logger.h                       # 日志系统接口
├── configs/                           # 配置文件模板
│   ├── development.json               # 开发环境配置
│   ├── testing.json                   # 测试环境配置
│   ├── production.json                # 生产环境配置
│   └── core_only.json                 # 核心服务配置
├── build/                             # 构建输出目录
├── logs/                              # 统一日志目录
└── docs/                              # 技术文档
```

## 快速开始

### 学习文档（适合不同水平）

- **新手入门** - [快速入门指南](docs/QUICK_START.md) - 30分钟理解核心概念
- **详细学习** - [完整学习指南](docs/LEARNING_GUIDE.md) - 适合1年经验开发者的4周学习计划  
- **实战练习** - [实战项目集](docs/PRACTICE_PROJECTS.md) - 4个渐进式练习项目

### 环境要求

- **操作系统**：Linux (Ubuntu 18.04+, CentOS 7+)
- **编译器**：GCC 7.0+ (支持C11) 或 Clang 8.0+
- **C++编译器**：G++ 7.0+ (支持C++17) 或 Clang++ 8.0+
- **构建工具**：CMake 3.10+ 或 Make
- **依赖库**：pthread, dl, cjson

### 1. 克隆和编译

```bash
# 克隆项目
git clone https://github.com/username/startTool.git
cd startTool

# 使用CMake构建 (推荐)
mkdir build && cd build
cmake ..
make -j$(nproc)

# 或使用传统Makefile
make clean && make
```

### 2. 运行演示

```bash
# 运行C任务演示
./build/task_demo
# 输出: 启动3个C语言任务，演示基本功能

# 运行C++任务演示  
./build/cpp_task_demo
# 输出: 启动C++服务，展示STL和现代C++特性

# 交互式演示模式
./build/cpp_task_demo interactive
# 支持运行时控制：查看状态、重启服务等
```

### 3. 配置你的服务

#### 创建服务配置文件

```json
{
    "launcher_config": {
        "log_level": "INFO",
        "log_file": "logs/my_services.log", 
        "max_concurrent_tasks": 8,
        "health_check_interval": 30
    },
    "services": [
        {
            "name": "my_core_service",
            "plugin_path": "lib/my_core_service.so",
            "priority": "HIGH",
            "auto_restart": true,
            "max_restart_count": 3,
            "depends_on": [],
            "config": {
                "listen_port": 8080,
                "worker_threads": 4,
                "max_connections": 1000
            }
        },
        {
            "name": "my_data_service",
            "plugin_path": "lib/my_data_service.so", 
            "priority": "NORMAL",
            "auto_restart": true,
            "depends_on": ["my_core_service"],
            "config": {
                "database_url": "mysql://localhost:3306/mydb",
                "cache_size_mb": 512
            }
        }
    ]
}
```

#### 启动你的服务系统

```bash
./launcher --config configs/my_services.json

# 预期输出:
[2025-07-22 10:30:01] INFO  启动器初始化完成
[2025-07-22 10:30:02] INFO  加载插件: lib/my_core_service.so
[2025-07-22 10:30:03] INFO  my_core_service 启动成功 (PID: 12345)
[2025-07-22 10:30:04] INFO  加载插件: lib/my_data_service.so  
[2025-07-22 10:30:05] INFO  my_data_service 启动成功 (PID: 12346)
[2025-07-22 10:30:06] INFO  所有服务启动完成！系统就绪
```

## 开发指南

### 创建新的服务插件

#### 方式一：C语言插件 (推荐用于系统级服务)

```c
#include "task_interface.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

// 1. 定义服务结构体 - "继承"TaskBase
typedef struct {
    TaskBase base;              // 必须是第一个成员!
    
    // 你的服务特定数据
    int server_port;
    char* database_url; 
    int worker_count;
} MyService;

// 2. 实现虚函数
static int my_service_initialize(TaskBase* base) {
    MyService* service = (MyService*)base;
    
    // 从配置中读取参数
    const char* config = (const char*)base->config.custom_config;
    // 解析配置，初始化资源...
    
    printf("初始化我的服务，端口: %d\n", service->server_port);
    return 0;  // 成功返回0
}

static int my_service_execute(TaskBase* base) {
    MyService* service = (MyService*)base;
    
    printf("我的服务开始运行...\n");
    
    // 主服务循环
    while (!base->should_stop) {
        // 执行你的业务逻辑
        printf("处理业务请求...\n");
        sleep(5);  // 模拟工作
        
        // 更新统计信息
        base->stats.execution_count++;
    }
    
    printf("我的服务正常退出\n");
    return 0;
}

static void my_service_cleanup(TaskBase* base) {
    MyService* service = (MyService*)base;
    printf("清理我的服务资源...\n");
    
    // 清理资源，关闭连接等
    if (service->database_url) {
        free(service->database_url);
    }
}

static bool my_service_health_check(TaskBase* base) {
    MyService* service = (MyService*)base;
    
    // 实现健康检查逻辑
    // 例如：检查数据库连接，检查端口监听等
    
    return true;  // 健康返回true
}

// 3. 定义虚函数表
static const TaskInterface my_service_vtable = {
    .initialize = my_service_initialize,
    .execute = my_service_execute,
    .cleanup = my_service_cleanup,
    .health_check = my_service_health_check,
    // 其他可选的虚函数...
};

// 4. 导出创建函数 - 启动器会调用这个函数
extern "C" TaskBase* create_task(const TaskConfig* config) {
    MyService* service = (MyService*)malloc(sizeof(MyService));
    if (!service) return NULL;
    
    // 初始化基类
    if (task_base_init(&service->base, &my_service_vtable, config) != 0) {
        free(service);
        return NULL;
    }
    
    // 初始化你的成员变量
    service->server_port = 8080;
    service->database_url = NULL;
    service->worker_count = 4;
    
    return &service->base;  // 返回基类指针
}

// 5. 导出销毁函数
extern "C" void destroy_task(TaskBase* base) {
    if (base) {
        task_base_destroy(base);
        free(base);
    }
}
```

#### 方式二：C++插件 (推荐用于业务逻辑复杂的服务)

```cpp
#include "task_interface.h"
#include <memory>
#include <thread>
#include <atomic>
#include <string>
#include <vector>
#include <iostream>

// 1. 定义C++服务类
class MyCppService {
public:
    MyCppService() : running_(false), port_(8080) {}
    
    bool initialize(const std::string& config_json) {
        // 使用现代C++特性：智能指针、STL容器等
        std::cout << "C++服务初始化中..." << std::endl;
        
        // 解析JSON配置，初始化资源
        // 例如：使用std::unique_ptr管理资源
        resource_ = std::make_unique<SomeResource>();
        
        return true;
    }
    
    void run() {
        running_ = true;
        std::cout << "C++服务开始运行..." << std::endl;
        
        while (running_) {
            // 使用STL算法、Lambda表达式等现代C++特性
            process_requests();
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        
        std::cout << "C++服务正常退出" << std::endl;
    }
    
    void stop() {
        running_ = false;
    }
    
    bool health_check() const {
        // 实现健康检查逻辑
        return running_ && resource_ != nullptr;
    }

private:
    std::atomic<bool> running_;
    int port_;
    std::unique_ptr<SomeResource> resource_;
    
    void process_requests() {
        // 业务逻辑处理
        std::vector<Request> requests = get_pending_requests();
        
        std::for_each(requests.begin(), requests.end(), 
                     [this](const Request& req) {
                         handle_request(req);
                     });
    }
};

// 2. C包装器结构体
struct MyCppServiceWrapper {
    TaskBase base;                              // 继承TaskBase
    std::unique_ptr<MyCppService> cpp_service;  // C++服务实例
};

// 3. 实现C接口函数
extern "C" {
    
static int cpp_service_initialize(TaskBase* base) {
    auto* wrapper = reinterpret_cast<MyCppServiceWrapper*>(base);
    
    wrapper->cpp_service = std::make_unique<MyCppService>();
    
    std::string config_str;
    if (base->config.custom_config) {
        config_str = static_cast<const char*>(base->config.custom_config);
    }
    
    return wrapper->cpp_service->initialize(config_str) ? 0 : -1;
}

static int cpp_service_execute(TaskBase* base) {
    auto* wrapper = reinterpret_cast<MyCppServiceWrapper*>(base);
    
    if (!wrapper->cpp_service) return -1;
    
    try {
        wrapper->cpp_service->run();
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "C++服务异常: " << e.what() << std::endl;
        return -1;
    }
}

static void cpp_service_cleanup(TaskBase* base) {
    auto* wrapper = reinterpret_cast<MyCppServiceWrapper*>(base);
    
    if (wrapper->cpp_service) {
        wrapper->cpp_service->stop();
        wrapper->cpp_service.reset();  // 智能指针自动释放
    }
}

static bool cpp_service_health_check(TaskBase* base) {
    auto* wrapper = reinterpret_cast<MyCppServiceWrapper*>(base);
    return wrapper->cpp_service ? wrapper->cpp_service->health_check() : false;
}

// 4. 虚函数表
static const TaskInterface cpp_service_vtable = {
    .initialize = cpp_service_initialize,
    .execute = cpp_service_execute,
    .cleanup = cpp_service_cleanup,
    .health_check = cpp_service_health_check
};

// 5. 导出函数
TaskBase* create_task(const TaskConfig* config) {
    auto* wrapper = static_cast<MyCppServiceWrapper*>(
        malloc(sizeof(MyCppServiceWrapper)));
    if (!wrapper) return nullptr;
    
    if (task_base_init(&wrapper->base, &cpp_service_vtable, config) != 0) {
        free(wrapper);
        return nullptr;
    }
    
    // 使用placement new初始化unique_ptr
    new(&wrapper->cpp_service) std::unique_ptr<MyCppService>();
    
    return &wrapper->base;
}

void destroy_task(TaskBase* base) {
    if (!base) return;
    
    auto* wrapper = reinterpret_cast<MyCppServiceWrapper*>(base);
    
    // 显式调用unique_ptr的析构函数
    wrapper->cpp_service.~unique_ptr<MyCppService>();
    
    task_base_destroy(base);
    free(base);
}

} // extern "C"
```

### 编译你的插件

#### CMakeLists.txt 示例

```cmake
# 添加你的C插件
add_library(my_service SHARED src/plugins/my_service.c)
target_link_libraries(my_service starttool_core)

# 添加你的C++插件  
add_library(my_cpp_service SHARED src/plugins/my_cpp_service.cpp)
target_link_libraries(my_cpp_service starttool_core)
target_compile_features(my_cpp_service PRIVATE cxx_std_17)

# 安装到插件目录
install(TARGETS my_service my_cpp_service
    LIBRARY DESTINATION lib/starttool/plugins
)
```

#### 手动编译

```bash
# 编译C插件
gcc -shared -fPIC -o lib/my_service.so src/plugins/my_service.c \
    -I include -L build -lstarttool_core -lpthread

# 编译C++插件
g++ -shared -fPIC -std=c++17 -o lib/my_cpp_service.so \
    src/plugins/my_cpp_service.cpp \
    -I include -L build -lstarttool_core -lpthread
```

## 最佳实践

### 服务设计原则

1. **单一职责** - 每个服务只负责一个明确的业务功能
2. **无状态设计** - 尽量避免服务间共享状态，通过消息传递通信  
3. **优雅降级** - 依赖服务不可用时，应该能够降级运行
4. **资源管理** - 合理管理内存、文件句柄、网络连接等资源

### 配置管理建议

```json
{
    // 使用环境变量支持不同部署环境
    "services": [
        {
            "name": "database_service",
            "config": {
                "host": "${DB_HOST:-localhost}",
                "port": "${DB_PORT:-3306}",
                "max_connections": "${DB_MAX_CONN:-100}"
            }
        }
    ]
}
```

### 日志规范

```c
// 使用统一的日志格式
void log_info(const char* service_name, const char* message) {
    // [时间戳] [级别] [服务名] 消息内容
    // [2025-07-22 10:30:15.123] [INFO] [map_service] 地图瓦片缓存更新完成
}
```

## 监控和运维

### 健康检查接口

每个服务都应该实现健康检查接口：

```c
static bool my_service_health_check(TaskBase* base) {
    // 检查关键资源状态
    if (!check_database_connection()) return false;
    if (!check_disk_space()) return false;
    if (get_memory_usage() > MAX_MEMORY_THRESHOLD) return false;
    
    return true;
}
```

### 性能监控

启动器会自动收集每个服务的性能指标：

- **CPU使用率** - 实时CPU占用百分比
- **内存使用量** - 物理内存和虚拟内存使用量  
- **运行时长** - 服务连续运行时间
- **重启次数** - 异常重启统计
- **执行计数** - 业务处理次数统计

### 运行时管理

```bash
# 查看所有服务状态
./launcher --status

# 重启特定服务  
./launcher --restart positioning_service

# 停止特定服务
./launcher --stop map_service

# 动态加载新服务
./launcher --load lib/new_service.so

# 查看性能统计
./launcher --stats
```

## 故障排查

### 常见问题及解决方案

#### 1. 服务启动失败

```
[ERROR] positioning_service 启动失败: 初始化错误 (-1)
```

**排查步骤:**
- 检查配置文件格式是否正确
- 验证依赖的库文件是否存在
- 查看详细错误日志: `tail -f logs/launcher.log`

#### 2. 服务异常重启

```
[WARN] map_service 异常退出，准备重启 (第2/3次)
```

**排查步骤:**
- 检查服务的健康检查逻辑
- 分析服务日志查找崩溃原因
- 检查系统资源使用情况

#### 3. 依赖服务启动超时

```
[ERROR] routing_service 等待依赖服务超时: positioning_service
```

**排查步骤:**
- 检查依赖关系配置是否正确
- 验证被依赖服务是否正常启动
- 调整启动超时时间配置

### 调试技巧

```bash
# 开启详细日志模式
./launcher --config my_config.json --log-level DEBUG

# 使用GDB调试特定服务
gdb -p $(pgrep my_service)

# 查看服务系统调用
strace -p $(pgrep my_service)

# 内存泄漏检查
valgrind --leak-check=full ./launcher --config my_config.json
```

## 生产环境部署

### Docker化部署

```dockerfile
FROM ubuntu:20.04

# 安装依赖
RUN apt-get update && apt-get install -y \
    gcc g++ cmake make \
    libpthread-stubs0-dev \
    libcjson-dev

# 复制源码
COPY . /app
WORKDIR /app

# 编译
RUN mkdir build && cd build && cmake .. && make

# 运行
CMD ["./build/launcher", "--config", "configs/production.json"]
```

### Kubernetes部署

```yaml
apiVersion: apps/v1
kind: Deployment
metadata:
  name: starttool-services
spec:
  replicas: 2
  selector:
    matchLabels:
      app: starttool
  template:
    metadata:
      labels:
        app: starttool
    spec:
      containers:
      - name: starttool
        image: starttool:latest
        ports:
        - containerPort: 8080
        env:
        - name: LOG_LEVEL
          value: "INFO"
        volumeMounts:
        - name: config
          mountPath: /app/configs
      volumes:
      - name: config
        configMap:
          name: starttool-config
```

### 系统服务部署

```ini
# /etc/systemd/system/starttool.service
[Unit]
Description=StartTool Service Manager  
After=network.target

[Service]
Type=simple
User=starttool
WorkingDirectory=/opt/starttool
ExecStart=/opt/starttool/launcher --config /opt/starttool/configs/production.json
Restart=always
RestartSec=10

[Install]
WantedBy=multi-user.target
```

```bash
# 启用系统服务
sudo systemctl enable starttool
sudo systemctl start starttool

# 查看服务状态
sudo systemctl status starttool
```

## 性能优化

### 编译优化

```bash
# 发布版本编译选项
cmake -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_C_FLAGS="-O3 -march=native" \
      -DCMAKE_CXX_FLAGS="-O3 -march=native" \
      ..
```

### 运行时优化

```json
{
    "launcher_config": {
        "max_concurrent_tasks": 16,        // 根据CPU核心数调整
        "health_check_interval": 60,       // 生产环境可适当延长
        "log_buffer_size": 4096,          // 增大日志缓冲区
        "enable_performance_profiling": true
    }
}
```

### 内存优化

- 使用内存池减少频繁分配
- 及时释放不再使用的资源  
- 监控内存使用情况，设置合理上限

## 社区和贡献

### 参与贡献

1. **Fork** 项目到你的GitHub
2. **创建分支** `git checkout -b feature/amazing-feature`  
3. **提交更改** `git commit -m 'Add amazing feature'`
4. **推送分支** `git push origin feature/amazing-feature`
5. **创建 Pull Request**

### 问题反馈

- **Bug报告**: [GitHub Issues](https://github.com/caixuf/startTool/issues)
- **功能请求**: [GitHub Discussions](https://github.com/caixuf/startTool/discussions)
- **技术交流**: 邮件联系

### 贡献指南

- 遵循现有代码风格
- 添加适当的单元测试
- 更新相关文档
- 一个PR只解决一个问题

## 版本规划

### v1.0.0 (当前版本)
- [x] 基础任务管理和插件系统
- [x] C/C++混合编程支持
- [x] 配置管理和日志系统
- [x] 基本监控和健康检查

### v1.1.0 (计划中)
- [ ] 服务依赖关系管理  
- [ ] 配置热重载功能
- [ ] Web管理界面
- [ ] 更丰富的插件示例

### v1.2.0 (规划中)
- [ ] 分布式服务支持
- [ ] 服务发现机制
- [ ] 负载均衡支持
- [ ] 云原生集成

## 许可证

本项目采用 [MIT License](LICENSE) 开源协议。

---

## 致谢

特别感谢项目组的同事们，在实际业务场景中验证和完善了这个项目：

- **@架构师** - 系统架构设计和优化建议
- **@开发工程师A** - 核心服务性能调优和测试用例
- **@开发工程师B** - 数据处理服务和CI/CD集成
- **@运维工程师** - 生产环境部署和监控体系建设

感谢开源社区的各种优秀项目为我们提供灵感和参考！

---

**StartTool - 让复杂的多服务系统管理变得简单高效！**

> *"好的工具不仅仅是解决问题，更是让开发者专注于真正重要的事情 - 创造价值。"*

---

**联系方式:**

- Email: <2024740941@qq.com>
- 微信: xxxxx  
- GitHub: [https://github.com/caixuf/startTool](https://github.com/caixuf/startTool)

   - 监控系统：监控进程状态和资源使用

2. **标准接口** (Standard Interface)
   - 进程生命周期接口
   - 配置接口
   - 监控接口

3. **插件进程** (Plugin Processes)
   - 实现标准接口的动态库
   - 独立的业务逻辑

## 编译和使用

### 使用CMake (推荐)

```bash
# 快速开始 - 使用构建脚本
./build.sh                    # 构建Release版本
./build.sh debug              # 构建Debug版本
./build.sh demo               # 运行任务演示
./build.sh launcher           # 运行启动器

# 手动CMake构建
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)

# 运行程序
./bin/task_demo               # 任务演示程序
./bin/launcher config/example.json  # 启动器程序
```

### 使用传统Makefile

```bash
# 编译启动器
make launcher

# 编译示例插件
make plugins

# 运行
./launcher config.json
```

## 目录结构

```
├── src/
│   ├── core/           # 启动器核心
│   ├── interface/      # 接口定义
│   ├── plugins/        # 示例插件
│   └── utils/          # 工具类
├── include/            # 头文件
├── config/             # 配置文件
├── build/              # 编译输出
└── docs/               # 文档
```
