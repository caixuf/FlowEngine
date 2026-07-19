# FlowEngine — 项目概览

轻量级自动驾驶中间件，核心是一个 Pub/Sub 消息总线 + 调度器 + 传输层。

> **开发流程：** 设计 → 执行 → 测试 → 迭代 → 清理 → 文档。详见 `.claude/skills/SKILL.md`（入口路由）。
> 改完代码后必跑：`/verify` → `/code-review` → `/simplify` → commit → 更新文档。

## 架构

```
sim_world → sensor_model → perception → fusion → planning → control → safety_control → monitor
     ↓            ↓             ↓           ↓          ↓          ↓            ↓            ↓
 vehicle/state  sensor/lidar perception/  fusion/  planning/  control/raw  control/cmd  dashboard
                sensor/gps  obstacles   localization trajectory  _cmd                     JSON
                     ↓             ↓           ↓          ↓          ↓            ↓
              ════════════════ Message Bus ════════════════════
                                    ↓
                             Transport (IPC/TCP) → Discovery → FlowRegistry
                                    ↓
                             flowmond (IPC stats bridge + HTTP/SSE) → DashBoard
```

## 关键文件

| 文件 | 作用 |
|------|------|
| `src/core/message_bus.c` | 进程内 Pub/Sub 总线 |
| `src/core/transport.c` | 统一传输抽象（local/IPC/TCP） |
| `src/core/scheduler.c` | 任务调度器（classic/choreo 模式） |
| `src/flowmond.c` | 监控守护进程（HTTP 仪表盘 + IPC 统计/仪表盘桥接 + 自动重连） |
| `src/core/monitor_server.c` | 内嵌 HTTP 服务器（多线程连接、/tools 静态资源、JSON 安全转义、过期缓存自动 fallback） |
| `src/core/stats_bridge.c` | 跨进程 topic 统计 IPC 桥接 |
| `src/core/dashboard_bridge.c` | 跨进程仪表盘 JSON IPC 桥接（分块传输协议） |
| `modules/adas_nodes/sensor_model_node.c` | 传感器模型（LiDAR/GPS/Camera，FOV/遮挡/噪声） |
| `modules/adas_nodes/safety_control_node.cpp` | FlowCoro 协程安全控制（TTC/横向交叉/行人防护） |
| `tools/flowboard/index.html` | 前端仪表盘（3D+2D+图表+D3 拓扑，ES modules） |
| `tools/foxglove_bridge.py` | Foxglove Studio WebSocket 桥接 |
| `tools/demo_evaluator.py` | 回归评估器：采样 JSON 并自动评分（碰撞/偏航/停滞/频率） |
| `scripts/demo.sh` | 一键启动脚本 |
| `src/flow_launcher.c` | 配置驱动启动器（读取 pipeline.json，dlopen 加载插件节点） |
| `src/flowctl.c` | CLI 工具（list/inspect/dashboard/param/bag 等子命令） |
| `src/core/flow_registry.c` | 统一元信息注册中心（Task/Topic/Type/Plugin/Schema） |
| `src/core/param_registry.c` | 参数系统（int/float/bool/string，范围校验，hot-reload） |
| `src/core/scenario_loader.c` | 场景 JSON 加载器（actor 定义 + ego 配置） |
| `tools/bag_check.c` | Bag 文件完整性检查器 |
| `scenarios/pedestrian_crossing.json` | 行人横穿场景 |
| `scenarios/highway_overtake.json` | 高速超车场景 |
| `modules/adas_nodes/data_recorder_node.c` | 训练样本采集（Learning Loop Stage 0） |
| `modules/adas_nodes/inference_node.c` | tiny-MLP 影子推理（Learning Loop Stage 2） |
| `modules/adas_nodes/tiny_mlp.h` | 纯 C 单隐层 MLP 推理内核 |
| `tools/train/train.py` | 离线训练脚本（scikit-learn MLP） |
| `docs/LEARNING_LOOP.md` | 车端学习闭环架构 |

> 深入教程见 `skills/` 目录（13 篇，覆盖 OOP in C、插件系统、消息总线、IPC、Bag、Clock、
> Serializer、State Machine、Discovery、Fusion、Coroutine、Demo Evaluator、E2E Learning Loop）。

## 运行

```bash
bash scripts/demo.sh [duration]          # 启动演示
bash scripts/demo.sh --no-browser 15     # 不打开浏览器
```

仪表盘: `http://localhost:8800`
3D 桥接: `ws://localhost:8765`

## 验证

```bash
# 每次改动 pipeline 链路上的节点后，跑评估器
python3 tools/demo_evaluator.py --duration 45 --interval 0.5

# 仅分析当前数据，不重新启动 demo
python3 tools/demo_evaluator.py --no-run
```

评估器采样 `/tmp/flow_topology.json`，自动检查：拓扑完整性、topic 频率、碰撞、路沿偏离、停滞、变道次数、偏航抖动、NPC 瞬移。WARN 是已知问题可忽略，FAIL 必须修复。

## 编码规范（统一 API — 2026-07 重构后强制执行）

> 以下 API 是本项目的**唯一合法入口**。禁止绕过它们直接调用底层函数。

### JSON 序列化/反序列化 → `cjson/cJSON.h`

```c
#include <cjson/cJSON.h>

// 序列化（替代手拼 snprintf）
cJSON* root = cJSON_CreateObject();
cJSON_AddNumberToObject(root, "speed", v);
cJSON_AddStringToObject(root, "mode", "highway");
char* s = cJSON_PrintUnformatted(root);   // topic publish 用 compact
// char* s = cJSON_Print(root);           // 文件写入用 formatted
transport_publish(t, "topic", (const uint8_t*)s, (uint32_t)strlen(s) + 1);
free(s);
cJSON_Delete(root);

// 反序列化（替代 strstr + sscanf）
cJSON* root = cJSON_Parse((const char*)msg->data);
if (root) {
    cJSON* j = cJSON_GetObjectItemCaseSensitive(root, "speed");
    if (cJSON_IsNumber(j)) double v = j->valuedouble;
    cJSON_Delete(root);
}
```

- ❌ **禁止** `snprintf("{\"key\":%d}", ...)` 手拼 JSON
- ❌ **禁止** `strstr(msg, "\"key\":")` + `sscanf` 手写解析
- ✅ **必须** 用 cJSON 序列化/反序列化所有 topic 消息和文件
- 参考：`src/core/scenario_loader.c`（已有成熟 pattern）

### 时间 → `clock_service.h`

```c
#include "clock_service.h"

uint64_t now = clock_now_us();               // CLOCK_MONOTONIC，仿真/回放模式下可注入
uint64_t real = clock_now_realtime_us();     // CLOCK_REALTIME，始终真实墙钟（训练样本、仪表盘）
```

- ❌ **禁止** `static uint64_t monotonic_us(void) { clock_gettime(...); }` 重复造轮子
- ❌ **禁止** `clock_gettime(CLOCK_MONOTONIC, &ts)` 裸调用
- ✅ **必须** 用 `clock_now_us()`；需绝对时间用 `clock_now_realtime_us()`

### 参数解析 → cJSON_Parse

```c
// 替代 strstr(params_json, "\"target_speed\":")
cJSON* p = cJSON_Parse(params_json);
cJSON* j = cJSON_GetObjectItemCaseSensitive(p, "target_speed");
if (cJSON_IsNumber(j)) cfg.target_speed = j->valuedouble;
cJSON_Delete(p);
```

- ❌ **禁止** `strstr(params_json, "\"param_name\":")` + `sscanf` 手写参数解析
- ✅ **必须** 用 `cJSON_Parse(params_json)` 解析所有节点参数

### 错误码 → `error_codes.h`（modules 中待推广）

```c
#include "error_codes.h"
return ERR_INVALID_PARAM;  // 替代 return -1
```

### 违反以上规范的代码不会被合并。

## 常见故障模式

| 现象 | 根因 | 位置 |
|------|------|------|
| 仪表盘/3D 一直 "Waiting for data"，curl 却有数据 | 仪表盘 JSON 是 cJSON_Print 多行格式，SSE 单 `data:` 帧发送被 EventSource 按行丢弃，浏览器只收到 45 字节前缀。已在发送前压平为单行，详见 [排查文档](docs/TROUBLESHOOTING_3D_DASHBOARD.md) | `monitor_server.c` handle_sse |
| 3D 场景整屏黑（curl 有数据、console 报 `Unexpected token 'export'`） | MVC 重构（c5e4ba9）拆 Controller 层时 `_renderFrame` 相机块漏闭合一个 `}`，scene3d.js 顶层 `export` 被当块内语句、整模块编译失败不执行 → `init3DScene` 未导出。已补回 | `scene3d.js:2159` 附近 |
| 仪表盘所有请求挂死（端口在监听） | 终端对前台 demo.sh 按了 Ctrl+Z，整个进程组 `T (stopped)`。Ctrl+C 结束或后台运行 | `scripts/demo.sh` |
| 车速降到 0 后永久卡死 | ROAD_GUARD 低俗恢复条件要求 `|y|>=road_center_limit`，但车可在任意 `2.1<|y|<2.5` 停下。改为只要 `speed<2.5` 就给小油门 | `control_node.c:534` |
| 变道冲出车道 | Stanley heading 阻尼硬编码 0.5，pipeline.json 的 `lat_kd_heading` 未生效 | `control_node.c:548` |
| NPC 瞬移 | 障碍物回收逻辑放入 100m 外（设计如此，非 bug） | `sim_world_node.c:204` |
| NPC/车飞出路面、不在车道上、坐标飞到几千米外 | flowsim NPC 用 `step_bicycle(steer=0)` 世界系直线积分、不跟道路几何，路一拐弯就直线冲出路网。已改为中央 `Route`（把 esmini 各 road 连成有序主路）+ Frenet 沿车道推进 + 到头回收 | `npc_ai.cpp` step_npc_vehicle / `flowsim/route.cpp` |
| 感知降频 | DBSCAN 点云过多时聚类耗时超过 deadline | `perception_node.c` |

## 最新 tag

`v0.1.0` — 创始版本，8 节点全链路稳定运行

---

# 可视化架构

详见 [可视化架构](docs/VISUALIZATION_ARCHITECTURE.md)。

可视化由统一的 C 监控守护进程 **flowmond**（`src/flowmond.c` → `build/bin/flowmond`）提供，
内置 HTTP 仪表盘，同时启用两条等价数据链路，按可用性自动回退：

```
monitor_node → 10Hz 写 /tmp/flow_topology.json → flowmond :8800 → 浏览器 (文件桥接回退)
monitor_node → stats_bridge / dashboard_bridge → IPC SHM → flowmond :8800 → 浏览器 (IPC 桥接)
```

- 前端 `tools/flowboard/index.html` 由 flowmond 通过 `--html-path` 加载并托管。
- `modules/adas_nodes/flowmond_node.cpp` 是 flowmond 的 `NodePlugin` 包装版，可作节点插件在 pipeline 内运行。
- 启动：`./build/bin/flowmond --html-path tools/flowboard/index.html`（或通过 `scripts/demo.sh`，已改为调用 flowmond）。

| 组件 | 端口 | 说明 |
|------|------|------|
| `flowmond` (C) | 8800 | HTTP 仪表盘：IPC 桥接（首选）+ 文件桥接（`/tmp/flow_topology.json` 回退）+ 自动重连 |
| `foxglove_bridge.py` | 8765 | Foxglove Studio 3D 桥接 |
