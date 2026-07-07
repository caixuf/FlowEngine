# 快速开始指南

## 环境要求

- Linux系统 (Ubuntu/CentOS/RHEL等)
- GCC编译器
- libcjson开发库
- jq工具 (用于JSON处理)

## 安装和编译

### 1. 安装系统依赖

```bash
# Ubuntu/Debian系统
sudo apt-get update
sudo apt-get install -y libcjson-dev jq build-essential

# CentOS/RHEL系统  
sudo yum install -y libcjson-devel jq gcc make

# 或者使用Makefile自动安装
make install
```

### 2. 编译项目

```bash
# 编译启动器和示例插件
make all

# 创建示例配置文件
make example_config
```

### 3. 运行示例

```bash
# 直接运行示例
make run

# 或者手动运行
cd build
./launcher ../config/example.json
```

## 交互式命令

启动器运行后，你可以使用以下命令：

```bash
# 启动进程
start example_process

# 查看进程状态  
status example_process

# 停止进程
stop example_process

# 重启进程
restart example_process

# 退出启动器
quit
```

## 开发自己的插件

### 1. 创建插件源文件

在 `src/plugins/` 目录下创建你的插件文件，例如 `my_process.c`：

```c
#include "process_interface.h"
#include <stdio.h>
#include <unistd.h>

// 进程信息
static ProcessInfo g_info = {
    .name = "my_process",
    .version = "1.0.0", 
    .description = "我的自定义进程",
    .priority = 1,
    .restart_count = 3,
    .auto_restart = true
};

// 实现必需的接口函数...
// (参考example_process.c的完整实现)

// 导出函数
ProcessInterface* get_process_interface(void) {
    return &g_interface;
}

uint32_t get_interface_version(void) {
    return PROCESS_INTERFACE_VERSION;
}
```

### 2. 编译插件

```bash
make plugins
```

### 3. 更新配置文件

修改 `config/example.json`，添加你的插件：

```json
{
  "processes": [
    {
      "name": "my_process",
      "library_path": "./build/plugins/my_process.so",
      "config_data": "{}",
      "priority": 1,
      "auto_start": true
    }
  ]
}
```

## 配置文件说明

```json
{
  "log_file": "launcher.log",        // 日志文件路径
  "log_level": 1,                   // 日志级别 (0=DEBUG, 1=INFO, 2=WARN, 3=ERROR, 4=FATAL)
  "monitor_interval": 5,            // 监控检查间隔(秒)
  "enable_monitor": true,           // 是否启用进程监控
  "processes": [                    // 进程配置列表
    {
      "name": "process_name",       // 进程名称
      "library_path": "./xxx.so",   // 动态库路径
      "config_data": "{}",          // 传递给进程的配置数据
      "priority": 1,                // 启动优先级
      "auto_start": true            // 是否自动启动
    }
  ]
}
```

## 常见问题

### Q: 编译时找不到cjson库
A: 确保已安装libcjson开发包：
```bash
sudo apt-get install libcjson-dev  # Ubuntu
sudo yum install libcjson-devel    # CentOS
```

### Q: 插件加载失败
A: 检查以下几点：
1. 动态库路径是否正确
2. 是否实现了必需的导出函数
3. 接口版本是否匹配
4. 查看日志文件获取详细错误信息

### Q: 进程无法启动
A: 检查：
1. 插件的initialize()函数返回值
2. 配置数据格式是否正确
3. 进程权限是否足够

### Q: 进程频繁重启
A: 检查：
1. health_check()函数实现
2. 进程主循环是否有异常退出
3. 调整重启次数限制

## 目录结构

```
FlowEngine/
├── src/
│   ├── launcher.c              # 主启动器
│   ├── core/                   # 核心模块
│   │   ├── process_manager.c   # 进程管理器
│   │   ├── config_manager.c    # 配置管理器
│   │   └── logger.c           # 日志系统
│   └── plugins/               # 插件目录
│       └── example_process.c   # 示例插件
├── include/                   # 头文件
│   ├── process_interface.h    # 核心接口定义
│   ├── process_manager.h      # 进程管理器
│   ├── config_manager.h       # 配置管理器
│   └── logger.h              # 日志系统
├── build/                    # 编译输出 (自动生成)
├── config/                   # 配置文件 (自动生成)
├── docs/                     # 文档
├── Makefile                  # 构建脚本
└── README.md                 # 项目说明
```

现在你可以开始使用这个启动器框架来管理你的进程了！
