# FlowEngine 整体修复计划

> 制定日期：2026-07-21
> 基于：本次性能排查（SSE 卡顿根因分析 + 调度/刷新机制审查 + 项目整体评估）

---

## 已完成的修复（本次会话）

| # | 文件 | 问题 | 修复 |
|---|------|------|------|
| 1 | `monitor_node.c` | 每帧写文件后又 fopen/fread 读回做 IPC 发布（双重 I/O） | 直接用内存中的 `json_str` 发布 |
| 2 | `monitor_node.c` | `cJSON_Print` 格式化 JSON 体积膨胀 30-40% | 改用 `cJSON_PrintUnformatted` |
| 3 | `foxglove_bridge.py` | 每帧重发 channel 广告 + `await writer.drain()` 阻塞事件循环 | channel 只在变化时广告；drain 改为 `ensure_future` 非阻塞 |
| 4 | `monitor_server.c` | SSE 500ms 轮询 → 浏览器只能以 2Hz 收到数据 | 条件变量事件驱动：dashboard_bridge 收到数据立即唤醒 SSE 线程 |

---

## Phase 0：性能兜底（预计 1-2 天）

这些是本次排查中发现但未修复的次级性能问题，不改会持续消耗 CPU/IO。

### 0.1 降低 monitor_node 导出频率

**现状：** `g.frequency_hz = 10.0`，每 100ms 构建完整 cJSON 树（含 scene_entities、LiDAR 点云、障碍物数组、road network、sysmon、registry 等）。

**问题：** 仪表盘不需要 10Hz 刷新。cJSON 构建 + 序列化 + 文件写入 + IPC 发布，四步都是 CPU 密集操作。

**方案：** 降到 5Hz（200ms）。仪表盘 5fps 完全够用，3D 场景有 dead reckoning 兜底。

```c
// monitor_node.c:859
g.frequency_hz = 5.0;  // 从 10.0 降到 5.0
```

### 0.2 关闭 flowmond 文件桥接线程（当 IPC 通道正常时）

**现状：** [flowmond.c#L159-L190](file:///workspace/src/flowmond.c#L159-L190) 的 `dashboard_file_watcher_fn` 线程以 5Hz 轮询 `/tmp/flow_topology.json`，即使 IPC 桥接已经正常工作。三条路径（IPC 桥接 + 文件轮询 + stats 桥接）同时跑，IPC 才是主路径。

**问题：** 多余的 `stat()` + `fopen`/`fread` 磁盘 I/O。多进程模式下文件轮询还有可能和 IPC 桥接产生数据竞争（同一个 `cached_json_version` 被两个线程同时更新）。

**方案：** 文件桥接线程加一个开关——IPC 桥接连接成功后自动暂停文件轮询，IPC 断开后恢复。

### 0.3 移除 foxglove_bridge.py 中多余的 `time.sleep(0.001)`

**现状：** [foxglove_bridge.py watcher.poll()](file:///workspace/tools/foxglove_bridge.py) 内部可能有高频轮询。

**方案：** 确认 watcher 实现，确保用 inotify/文件修改时间检测而非忙等。

---

## Phase 1：C 节点调度落地（预计 3-5 天）

这是 C 侧最大的架构问题。当前 8 个 ADAS 节点全部裸线程 `usleep`，没有统一的频率控制、延迟统计、资源配额。

### 1.1 让 C 节点接入 scheduler 的 RateControl

**目标：** 每个 C 节点的 execute 循环不再自己 `usleep(period)`，而是调用 `rate_control_acquire()` 决定是否执行。

**涉及文件：**
- `modules/adas_nodes/perception_node.c`
- `modules/adas_nodes/fusion_node.c`
- `modules/adas_nodes/planning_node.cpp`
- `modules/adas_nodes/control_node.cpp`
- `modules/adas_nodes/monitor_node.c`
- `modules/adas_nodes/flowsim_node.cpp`

**做法：** 每个节点的 `execute()` 函数开头加一行：

```c
if (!rate_control_acquire(&g.rate_ctrl)) return;
```

`g.rate_ctrl` 在 `init()` 中通过 `rate_control_init(&g.rate_ctrl, g.frequency_hz)` 初始化。

**效果：** 统一的频率控制，不再依赖 `usleep` 精度。后续可以通过 `flowctl` 动态调整每个节点的频率。

### 1.2 让 C 节点接入 scheduler 的 LatencyTracker

**目标：** 每个 C 节点记录每次 execute 的耗时，输出 P50/P99 延迟。

**做法：** 在 `execute()` 中增加：

```c
uint64_t t0 = clock_now_us();
// ... 原有逻辑 ...
uint64_t elapsed = clock_now_us() - t0;
latency_tracker_record(&g.latency, elapsed);
```

**效果：** 所有节点延迟可观测，为性能回归基线提供数据。

### 1.3 统一 C 节点的 execute 循环模式

**目标：** 消除每个节点自己写 `while(running) { execute(); usleep(period); }` 的模式。

**做法：** 在 `scheduler.c` 中提供 `scheduler_run_loop(Scheduler* s, int task_id)` 函数，封装标准的事件循环：

```c
void scheduler_run_loop(Scheduler* sched, int task_id, 
                        bool (*should_stop)(void*), void* ctx) {
    RateControl* rc = scheduler_get_rate_control(sched, task_id);
    LatencyTracker* lt = scheduler_get_latency(sched, task_id);
    while (!should_stop(ctx)) {
        if (rate_control_acquire(rc)) {
            uint64_t t0 = clock_now_us();
            scheduler_execute_task(sched, task_id);
            latency_tracker_record(lt, clock_now_us() - t0);
        }
        usleep(1000); // 1ms 粒度，避免 CPU 空转
    }
}
```

---

## Phase 2：测试补全（预计 5-7 天）

### 2.1 核心模块单元测试

**优先级最高（频繁出问题、改动风险高）：**

| 模块 | 文件 | 测试内容 |
|------|------|----------|
| message_bus | `tests/test_message_bus.c` | pub/sub 正确性、多订阅者分发、QoS drop 策略、退订竞态 |
| ipc_channel | `tests/test_ipc_channel.c` | 单/多分块重组、seq 乱序、超时、重连 |
| monitor_server | `tests/test_monitor_server.c` | SSE 推送正确性、条件变量唤醒、版本号去重、心跳 |
| dashboard_bridge | `tests/test_dashboard_bridge.c` | (已有 `test_bridges.c`，但需扩展) 断连重连、分块边界 |

**次优先级：**

| 模块 | 测试内容 |
|------|----------|
| config_manager | JSON 解析、参数覆盖、默认值 |
| discovery | 节点注册/注销、拓扑变化通知 |
| param_registry | 参数读写、类型校验、schema 验证 |

### 2.2 前端 JS 测试

| 文件 | 测试内容 |
|------|----------|
| `deadreckon.js` | 位置外推正确性、平滑收敛、π 跳变 |
| `scene2d.js` | 坐标变换、视野裁剪 |
| `app.js` | SSE 数据解析、updateAll 渲染管线（集成测试） |

### 2.3 重新启用 TSAN

**现状：** EVOLUTION_ROADMAP 提到 TSAN 被禁用。

**方案：**
1. 补 suppression 文件（白名单已知的 false positive）
2. 在 CI 中加入 `-fsanitize=thread` 构建
3. 跑 e2e_smoke + e2e_stability 15 分钟

---

## Phase 3：架构清理（预计 3-4 天）

### 3.1 消除 `write()` 返回值忽略

**现状：** [monitor_server.c](file:///workspace/src/core/monitor_server.c) 中大量 `(void)w;` 忽略 write 返回值。SSE 长连接对端断开会触发 EPIPE，当前代码无法优雅处理。

**方案：** 封装一个 `safe_write()` 辅助函数，统一处理 EPIPE/ECONNRESET：

```c
static int safe_write(int fd, const void* buf, size_t len) {
    ssize_t w = write(fd, buf, len);
    if (w < 0 && (errno == EPIPE || errno == ECONNRESET)) return -1;
    return (int)w;
}
```

### 3.2 flowmond 文件桥接按需启停

如 Phase 0.2 所述，IPC 桥接正常时关闭文件轮询。

### 3.3 清理死代码

**待移除或标记为 deprecated：**
- `benchmark.c` / `benchmark_coro.cpp`（demo.sh 不走它们）
- `flow_coro` / `flow_ipc` / `flow_bus` 独立二进制（功能已被 flow_launcher 覆盖）
- `src/flow_e2e.c`（如果存在且未被引用）

### 3.4 大文件拆分

**monitor_node.c（~900 行）：**
- `monitor_node_json.c` — JSON 构建逻辑（export_dashboard_json）
- `monitor_node_subscribe.c` — 订阅回调（on_obstacles, on_vehicle_state, ...）
- `monitor_node.c` — 主体 execute + NodePlugin 接口

**monitor_server.c（~1000 行）：**
- `monitor_server_http.c` — HTTP 路由、SSE 处理、静态文件
- `monitor_server_json.c` — build_sse_json、build_cached_dashboard_json
- `monitor_server.c` — 主体 create/start/stop/destroy + IPC 桥接

---

## Phase 4：性能回归基线（预计 2-3 天）

### 4.1 建立端到端延迟基线

**指标：**
- monitor_node JSON 构建耗时（P50/P99）
- SSE 端到端延迟（monitor_node 产出 → 浏览器收到）
- 3D 渲染帧率（FPS，含 dead reckoning 平滑效果）
- 消息总线 dispatch 延迟（pub → sub 回调）

**工具：**
- 复用已有的 `LatencyTracker` + `CoroStats`
- 新增 `scripts/perf_bench.py` 跑 demo.sh 30s 采集指标

### 4.2 CI 性能回归检测

在 CI nightly job 中加入：
1. 跑 `demo.sh --duration 30`
2. 采集 SSE 端到端延迟的 P99
3. 与基线比较，超过 2x 告警

---

## Phase 5：C++ 协程迁移（可选，预计 5-10 天）

### 5.1 将 C 节点逐步迁移到 FlowCoroTask

**目标：** demo 的 8 个 ADAS 节点从 C 裸线程迁移到 C++ 协程模型。

**优先级：**
1. `monitor_node` → `MonitorCoroTask`（产出 dashboard JSON，影响面最大）
2. `perception_node` → `PerceptionCoroTask`
3. `fusion_node` → `FusionCoroTask`
4. 其余节点

**优势：**
- 数据驱动调度：消息到达才唤醒，不再 `usleep` 盲等
- CancelToken 优雅停止：`stop()` 直接唤醒协程，无需等 sleep 结束
- 线程池执行：FLOWCORO 模式下不阻塞总线分发线程
- 可观测性：CoroStats 提供 resume 次数和挂起时长分布

**风险：** 改动量大，需要逐个节点迁移并回归测试。建议 Phase 1（C 调度落地）先做，Phase 5 作为后续优化。

---

## 优先级总结

```
Phase 0  ████░░░░░░  性能兜底          (1-2 天)  立竿见影
Phase 1  ██████░░░░  C 节点调度落地     (3-5 天)  解决架构问题
Phase 2  ████████░░  测试补全           (5-7 天)  降低改动风险
Phase 3  ████░░░░░░  架构清理           (3-4 天)  提高可维护性
Phase 4  ███░░░░░░░  性能回归基线       (2-3 天)  防止性能退化
Phase 5  ██████████  协程迁移（可选）   (5-10 天) 长期架构升级
```

建议先做 Phase 0 + Phase 1，这两项改动最小、收益最大，能直接解决当前 "界面卡顿" + "调度缺失" 两个核心问题。