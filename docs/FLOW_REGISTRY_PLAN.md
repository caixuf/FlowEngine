# FlowRegistry — 统一元信息注册中心设计计划

> 状态：**已实现**（2026-07-09）
> FlowRegistry (`src/core/flow_registry.c`, `include/flow_registry.h`) 和 ParamRegistry
> (`src/core/param_registry.c`, `include/param_registry.h`) 已完成并集成到 `msg_schema`、
> `task_manager`、`process_manager` 和 `flowctl`。本文档保留作为原始设计记录，实际 API 以头文件为准。

## 1. 问题分析：当前元信息分散现状

经过代码调研，当前项目元信息分布在 6 个独立注册表中，彼此不感知：

| 模块 | 头文件 | 注册了什么 | 查询方式 |
|------|--------|-----------|---------|
| msg_schema | `include/msg_schema.h` | topic → struct_size | `msg_schema_check()` |
| serializer | `include/serializer.h` | type_id + 序列化函数 | `serializer_lookup_type()` |
| state_machine | `include/state_machine.h` | 状态转移表（每个 task 独立持有） | `statem_dump_table()` |
| discovery | `include/discovery.h` | 节点 + topic 拓扑 | `discovery_get_topology()` |
| task_manager | `include/task_manager.h` | 任务名 → TaskBase* | `task_manager_get_task()` |
| process_manager | `include/process_manager.h` | 插件库 + ProcessNode | `process_manager_load_plugin()` |

问题：
- 同一个 topic 的类型信息在 msg_schema、serializer、discovery 三处各存一份，且字段不同
- 同一个 task 的元信息在 task_manager、process_manager、state_machine 三处各管一段
- 没有统一查询入口，flowctl / topology viewer / bag info 各自需要遍历多套注册表
- 插件加载后无法向统一注册中心声明自己发布的 topic、订阅的 topic、参数定义

## 2. 设计目标

1. **单一真相源**：每种元信息（Task / Topic / Type / Param / Plugin）只在一个地方注册
2. **向后兼容**：现有 `msg_schema_register`、`serializer_register_type` 等 API 保持可用，内部委托给 FlowRegistry
3. **插件自声明**：插件加载时可以向注册中心声明自己的 topic / param / 状态机，无需 launcher 手动配置
4. **统一查询**：提供 `flow_registry_dump_json()` 等接口，供 flowctl / topology viewer 直接使用
5. **线程安全**：注册中心内部加锁，支持运行时动态注册/注销

## 3. 核心数据结构

### 3.1 FlowRegistry 主结构

```c
typedef struct FlowRegistry FlowRegistry;

/* 获取全局单例（进程内唯一，插件共享） */
FlowRegistry* flow_registry_get_instance(void);

/* 销毁全局单例（程序退出时调用） */
void flow_registry_destroy(void);
```

### 3.2 TaskMeta — 任务元信息

```c
typedef struct {
    char        name[64];            /* 任务名 */
    char        plugin_name[64];     /* 来源插件名 */
    TaskState   state;               /* 当前运行状态 */
    TaskPriority priority;
    char        description[256];

    /* 发布/订阅的 topic 名列表 */
    char        pub_topics[8][64];
    int         pub_count;
    char        sub_topics[8][64];
    int         sub_count;

    /* 依赖任务名列表 */
    char        dependencies[8][64];
    int         dep_count;

    /* 运行统计快照 */
    TaskStats   stats;
} TaskMeta;

/* 注册/查询 */
int  flow_registry_register_task(FlowRegistry* reg, const TaskMeta* meta);
int  flow_registry_unregister_task(FlowRegistry* reg, const char* name);
const TaskMeta* flow_registry_lookup_task(FlowRegistry* reg, const char* name);
int  flow_registry_list_tasks(FlowRegistry* reg, TaskMeta* out, int max);
int  flow_registry_update_task_state(FlowRegistry* reg, const char* name,
                                      TaskState new_state);
```

### 3.3 TopicMeta — 话题元信息

```c
typedef struct {
    char        name[64];            /* topic 名 */
    uint32_t    type_id;             /* FNV-1a 类型 ID */
    char        type_name[64];       /* 类型名字符串 */
    uint8_t     schema_version;
    size_t      struct_size;

    /* 发布者/订阅者任务名 */
    char        publishers[8][64];
    int         pub_count;
    char        subscribers[8][64];
    int         sub_count;

    /* QoS 占位（后续扩展） */
    uint8_t     reliability;         /* 0=best_effort, 1=reliable */
    uint32_t    depth;               /* 历史深度 */
    double      frequency_hz;        /* 期望频率 */
} TopicMeta;

int  flow_registry_register_topic(FlowRegistry* reg, const TopicMeta* meta);
const TopicMeta* flow_registry_lookup_topic(FlowRegistry* reg, const char* name);
int  flow_registry_list_topics(FlowRegistry* reg, TopicMeta* out, int max);
int  flow_registry_add_publisher(FlowRegistry* reg, const char* topic,
                                 const char* task_name);
int  flow_registry_add_subscriber(FlowRegistry* reg, const char* topic,
                                  const char* task_name);
```

### 3.4 TypeMeta — 类型元信息

```c
typedef struct {
    uint32_t        type_id;          /* FNV-1a hash */
    char            type_name[64];
    uint8_t         schema_version;
    size_t          struct_size;
    SerializeFunc   serialize;
    DeserializeFunc deserialize;
    EndianSwapFunc  endian_swap;
} TypeMeta;

int  flow_registry_register_type(FlowRegistry* reg, const TypeMeta* meta);
const TypeMeta* flow_registry_lookup_type(FlowRegistry* reg, uint32_t type_id);
const TypeMeta* flow_registry_lookup_type_by_name(FlowRegistry* reg,
                                                   const char* name);
int  flow_registry_list_types(FlowRegistry* reg, TypeMeta* out, int max);
```

### 3.5 ParamMeta — 参数元信息

```c
typedef enum {
    PARAM_INT,
    PARAM_DOUBLE,
    PARAM_STRING,
    PARAM_BOOL,
} ParamType;

typedef struct {
    char        name[64];
    ParamType  type;
    char        description[256];

    /* 当前值（联合体） */
    union {
        long    int_val;
        double  dbl_val;
        char    str_val[256];
        bool    bool_val;
    } value;

    /* 默认值（联合体，同上结构） */
    union {
        long    int_val;
        double  dbl_val;
        char    str_val[256];
        bool    bool_val;
    } default_value;

    char        owner_task[64];      /* 参数所属任务 */
    bool        is_dynamic;          /* 是否允许运行时修改 */
} ParamMeta;

int  flow_registry_register_param(FlowRegistry* reg, const ParamMeta* meta);
int  flow_registry_set_param(FlowRegistry* reg, const char* name,
                             ParamType type, const void* value);
int  flow_registry_get_param(FlowRegistry* reg, const char* name,
                             ParamType* type, void* out_value);
const ParamMeta* flow_registry_lookup_param(FlowRegistry* reg, const char* name);
int  flow_registry_list_params(FlowRegistry* reg, ParamMeta* out, int max);
```

### 3.6 PluginMeta — 插件元信息

```c
typedef struct {
    char        name[64];            /* 插件名 */
    char        library_path[256];   /* .so 文件路径 */
    char        version[32];
    char        description[256];
    uint32_t    interface_version;   /* PROCESS_INTERFACE_VERSION */

    /* 插件提供的任务列表 */
    char        tasks[16][64];
    int         task_count;

    /* 插件状态 */
    bool        is_loaded;
    uint32_t    restart_count;
} PluginMeta;

int  flow_registry_register_plugin(FlowRegistry* reg, const PluginMeta* meta);
int  flow_registry_unregister_plugin(FlowRegistry* reg, const char* name);
const PluginMeta* flow_registry_lookup_plugin(FlowRegistry* reg, const char* name);
int  flow_registry_list_plugins(FlowRegistry* reg, PluginMeta* out, int max);
```

### 3.7 统一导出

```c
/* 导出整个注册中心为 JSON（供 flowctl / topology viewer 使用） */
char* flow_registry_dump_json(FlowRegistry* reg);

/* 导出指定维度的 JSON */
char* flow_registry_dump_tasks_json(FlowRegistry* reg);
char* flow_registry_dump_topics_json(FlowRegistry* reg);
char* flow_registry_dump_types_json(FlowRegistry* reg);
char* flow_registry_dump_params_json(FlowRegistry* reg);
char* flow_registry_dump_plugins_json(FlowRegistry* reg);

/* 拓扑图：pub/sub 关系矩阵 */
char* flow_registry_dump_topology_json(FlowRegistry* reg);
```

## 4. 实现计划

### 阶段 1：FlowRegistry 核心（新建文件）

| 步骤 | 文件 | 内容 |
|------|------|------|
| 1.1 | `include/flow_registry.h` | 全部公共 API 声明 + 数据结构定义 |
| 1.2 | `src/core/flow_registry.c` | 注册中心实现：内部用数组 + pthread_mutex，单例模式 |
| 1.3 | `CMakeLists.txt` | 将 `flow_registry.c` 加入 `CORE_SOURCES` |

### 阶段 2：适配层 — 旧 API 委托给 FlowRegistry

| 步骤 | 文件 | 内容 |
|------|------|------|
| 2.1 | `src/core/msg_schema.c` | `msg_schema_register()` 内部调用 `flow_registry_register_topic()` |
| 2.2 | `src/core/serializer.c` | `serializer_register_type()` 内部调用 `flow_registry_register_type()` |
| 2.3 | `src/core/task_manager.c` | `task_manager_register()` 内部调用 `flow_registry_register_task()` |
| 2.4 | `src/core/process_manager.c` | `process_manager_load_plugin()` 内部调用 `flow_registry_register_plugin()` |

适配层原则：
- 旧 API 签名不变，不破坏现有插件
- 旧 API 内部委托给 FlowRegistry，同时保留旧的全局表（如有）做兼容
- 新代码直接使用 FlowRegistry 新 API

### 阶段 3：插件自声明机制

| 步骤 | 文件 | 内容 |
|------|------|------|
| 3.1 | `include/flow_registry.h` | 增加 `FLOW_REGISTRY_DECLARE_PLUGIN()` 宏 |
| 3.2 | 插件示例改造 | 在 `example_task.c` 等插件中演示自声明 |

```c
/* 插件 .so 加载后自动执行的声明宏 */
#define FLOW_REGISTRY_DECLARE_PLUGIN(plugin_name, version, desc, ...) \
    __attribute__((constructor)) \
    static void _flow_plugin_declare_##plugin_name(void) { \
        FlowRegistry* reg = flow_registry_get_instance(); \
        PluginMeta meta = {0}; \
        strncpy(meta.name, #plugin_name, sizeof(meta.name)-1); \
        strncpy(meta.version, version, sizeof(meta.version)-1); \
        strncpy(meta.description, desc, sizeof(meta.description)-1); \
        flow_registry_register_plugin(reg, &meta); \
        __VA_ARGS__ \
    }
```

### 阶段 4：单元测试

| 步骤 | 文件 | 内容 |
|------|------|------|
| 4.1 | `tests/test_registry.c` | 注册/查询/注销各维度元信息 |
| 4.2 | `CMakeLists.txt` | 加入 `registry_test` 目标 |

测试用例：
- 注册 task → 查询 → 注销 → 查询返回 NULL
- 注册 topic → add_publisher / add_subscriber → 拓扑正确
- 注册 type → 按 type_id 和 name 两种方式查询
- 注册 param → set/get 往返一致
- 注册 plugin → 关联 task → 查询 plugin 的 task 列表
- dump_json 输出非空且可解析
- 线程安全：多线程并发注册不崩溃
- 适配层：`msg_schema_register` 后 `flow_registry_lookup_topic` 能查到

### 阶段 5：集成到现有 demo 验证

| 步骤 | 文件 | 内容 |
|------|------|------|
| 5.1 | `src/core/flow_registry.c` | 启动时调用 `flow_registry_dump_json()` 打印注册中心状态 |
| 5.2 | `tools/topology_viewer.html` | 验证能消费 registry JSON |

## 5. 文件清单

新增文件：
- `include/flow_registry.h` — 公共头文件
- `src/core/flow_registry.c` — 实现
- `tests/test_registry.c` — 单元测试

修改文件：
- `CMakeLists.txt` — 添加新源文件和测试目标
- `src/core/msg_schema.c` — 委托给 registry
- `src/core/serializer.c` — 委托给 registry
- `src/core/task_manager.c` — 委托给 registry
- `src/core/process_manager.c` — 委托给 registry
- `src/flow_launcher.c` — 集成验证

## 6. 容量与约束

| 维度 | 最大容量 | 理由 |
|------|---------|------|
| Task | 64 | 单进程中间件合理上限 |
| Topic | 64 | 对齐 MSG_BUS_MAX_TOPICS |
| Type | 128 | 对齐 SERIALIZER_MAX_TYPE_ENTRIES |
| Param | 128 | 足够覆盖多任务参数 |
| Plugin | 32 | 动态库插件合理上限 |
| 每个 topic 的 pub/sub | 8 | 大多数场景足够 |

## 7. 不做什么（scope 边界）

- 不引入动态内存分配的哈希表，用定长数组 + 线性查找（简单、可预测、嵌入式友好）
- 不做跨进程注册中心同步（Discovery 模块继续负责网络拓扑，Registry 只管进程内）
- 不做 schema 字段级反射（仅记录 type_id + size，字段级留给未来 IDL 生成器）
- 不替换旧 API，只做适配委托
- 不修改 MessageBus / IPC / Bag 内部实现
