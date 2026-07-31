# Skill 02 - 插件化架构（dlopen 动态加载）

## 核心思想

通过 `dlopen` / `dlsym` 在运行时动态加载共享库（`.so`），框架无需在编译时知道业务逻辑，新增服务只需编写插件并修改配置文件，**不需要重新编译框架**。

## 工作原理

```
配置文件 (JSON)
   └─ plugin_path: "lib/my_service.so"
         │
         ▼
  dlopen("lib/my_service.so", RTLD_LAZY)
         │
         ▼
  dlsym(handle, "create_task")   ← 查找工厂函数
         │
         ▼
  create_task(&config)            ← 创建任务对象
         │
         ▼
  TaskBase* task                  ← 框架统一管理
```

## 关键 API

```c
#include <dlfcn.h>

// 加载动态库
void* handle = dlopen("lib/my_service.so", RTLD_LAZY | RTLD_LOCAL);
if (!handle) {
    fprintf(stderr, "dlopen 失败: %s\n", dlerror());
}

// 查找符号（工厂函数）
typedef TaskBase* (*CreateTaskFn)(const TaskConfig*);
CreateTaskFn create_fn = (CreateTaskFn)dlsym(handle, "create_task");

char* err = dlerror();
if (err) {
    fprintf(stderr, "dlsym 失败: %s\n", err);
    dlclose(handle);
}

// 调用工厂函数创建任务
TaskBase* task = create_fn(&config);

// 卸载（任务停止后）
dlclose(handle);
```

## 插件合约

每个插件 `.so` 必须导出以下符号：

```c
// 工厂函数 —— 框架通过此函数创建任务实例
TaskBase* create_task(const TaskConfig* config);

// 可选：销毁函数（默认使用 task_base_destroy）
void destroy_task(TaskBase* task);
```

## 插件编写模板

```c
/* my_plugin.c */
#include "task_interface.h"
#include <stdlib.h>

typedef struct {
    TaskBase base;   // 父类必须第一
    /* 插件私有数据 */
} MyPlugin;

static int  my_init   (TaskBase* b) { return 0; }
static int  my_execute(TaskBase* b) {
    while (!b->should_stop) { sleep(1); }
    return 0;
}
static void my_cleanup(TaskBase* b) {}

static const TaskInterface my_vtable = {
    .initialize = my_init,
    .execute    = my_execute,
    .cleanup    = my_cleanup,
};

/* 框架查找此符号 */
TaskBase* create_task(const TaskConfig* config) {
    MyPlugin* p = calloc(1, sizeof(MyPlugin));
    if (!p) return NULL;
    task_base_init(&p->base, &my_vtable, config);
    return &p->base;
}
```

## CMakeLists.txt 配置

```cmake
# 将插件编译为动态库
add_library(my_plugin SHARED my_plugin.c)
target_link_libraries(my_plugin flowengine_core)

# 安装到插件目录
install(TARGETS my_plugin
    LIBRARY DESTINATION lib/flowengine/plugins
)
```

## 插件配置（JSON）

```json
{
  "services": [
    {
      "name": "my_service",
      "plugin_path": "lib/flowengine/plugins/my_plugin.so",
      "priority": "NORMAL",
      "auto_restart": true,
      "max_restart_count": 3,
      "depends_on": []
    }
  ]
}
```

## 依赖排序（拓扑排序）

框架在启动时对 `depends_on` 字段做拓扑排序，保证依赖服务先启动：

```
A → B → C      启动顺序：A, B, C
     ↘ D       启动顺序：A, B, C 和 D（B 的两个依赖并行可行）
```

若检测到循环依赖则报错退出。

## 注意事项

1. **符号可见性** — 插件内部函数加 `static`，避免符号污染主进程命名空间。
2. **`RTLD_LOCAL`** — 插件符号不对其他插件可见，防止符号冲突。
3. **错误处理** — `dlopen` 和 `dlsym` 后必须检查 `dlerror()`。
4. **生命周期** — `dlclose` 必须在插件对象完全销毁后调用，否则会 SIGSEGV。

## 参考文件

- `src/core/process_manager.c` — dlopen 加载与管理逻辑
- `src/launcher.c` — 主启动器，读取配置并依次加载插件
- `src/plugins/example_process.c` — 最简进程插件示例
- `cmake/config.json.in` — 配置文件模板
