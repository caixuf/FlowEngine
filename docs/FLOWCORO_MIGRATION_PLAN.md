# ADAS 节点 FlowCoro 迁移方案

> 目标：把适合的 C NodePlugin 节点迁移到 C++ FlowCoro 协程框架，统一并发模型，消除手写 pthread + condvar + 轮询的复杂度。
>
> 范围：本方案为**评审稿**，不含落地代码。先评审，再按节点逐个实施。

---

## 1. 背景与现状

`modules/adas_nodes/` 下 14 个节点中，**仅 [safety_control_node.cpp](file:///workspace/modules/adas_nodes/safety_control_node.cpp) 真正使用 FlowCoro 协程**（`CoroutineTask` + `co_await when_any_bus`）。其余 12 个均为 C 的 `NodePlugin`，采用 pthread + `transport_subscribe` 回调 + 手写同步原语。[flowmond_node.cpp](file:///workspace/modules/adas_nodes/flowmond_node.cpp) 虽是 C++，但只是普通 NodePlugin 包装，无协程。

| 节点 | 语言 | 线程模型 | 订阅 topics | 适配 FlowCoro |
|------|------|---------|-------------|--------------|
| safety_control | C++ | FlowCoro 协程 | control/raw_cmd, vehicle/state | 已迁移（范式） |
| **fusion** | C | condvar 事件驱动 | sensor/lidar, sensor/gps | **最高** |
| perception | C | usleep 定时轮询 | vehicle/state | 高 |
| planning | C | usleep 定时轮询 | fusion/localization, vehicle/state, road/geometry | 高 |
| control | C | usleep 定时轮询 | fusion/localization, planning/trajectory, vehicle/state, road/geometry | 中 |
| inference | C | usleep 定时轮询 | control/cmd, fusion/localization, perception/obstacles, planning/trajectory | 中 |
| monitor | C | usleep 定时轮询 | 6 个 topic | 低 |
| data_recorder / learner / model_ota | C | usleep / I/O | 多 topic | 低 |
| sim_world / sensor_model | C | 固定频率自驱动 | — | 不建议 |

**关键约束**：`pipeline.json` 对 C/C++ 节点完全透明——`flow_launcher` 通过 `dlopen + node_get_plugin` 统一加载，只要 `.so` 名和导出符号不变，迁移**无需改 pipeline.json**。

---

## 2. 迁移范式（safety_control 模板）

safety_control 已验证的迁移路径，四要素：

1. **保留 NodePlugin 外壳**：`s_plugin` 结构体 + `extern "C" NodePlugin* node_get_plugin(void)`
2. **init 构造协程任务**：`g.task = std::make_unique<XxxTask>(bus, transport, params)`
3. **start 启动协程宿主线程**：`pthread_create(..., []{ g.task->execute(); })`
4. **run 用 co_await 等待消息**：`while (!should_stop()) co_await when_any_bus(...)`

骨架（参考 [safety_control_node.cpp:275-308](file:///workspace/modules/adas_nodes/safety_control_node.cpp#L275-L308)）：

```cpp
class XxxTask : public CoroutineTask {
public:
    XxxTask(MessageBus* bus, Transport* transport)
        : CoroutineTask(bus), transport_(transport) {}

protected:
    Task run() override {
        while (!should_stop()) {
            // 替代 pthread_cond_wait / usleep 轮询
            Message msg = co_await when_any_bus(
                bus(), {"topic/a", "topic/b"}, &cancel_token_);
            if (should_stop()) break;
            // ... 业务逻辑 ...
            transport_publish(transport_, "out/topic", buf, len);
        }
    }

private:
    Transport* transport_;
};
```

**生命周期映射**：

| C 节点 | FlowCoro 节点 |
|--------|--------------|
| `pthread_create(thread_fn)` | `pthread_create([]{ task->execute(); })` |
| `pthread_cond_timedwait` / `usleep` | `co_await when_any_bus` / `co_await delay_ms` |
| `volatile should_stop` + `cond_signal` | `task->stop()` 触发 CancelToken 唤醒挂起的 awaitable |
| `pthread_join` | `pthread_join` + `task.reset()` |

---

## 3. fusion 迁移详步（首要目标）

### 3.1 当前实现分析（[fusion_node.c](file:///workspace/modules/adas_nodes/fusion_node.c)）

- **事件驱动**：`on_lidar` 回调 push 到 `lidar_buf` + `cond_signal`；`fusion_thread` 在 `pthread_cond_timedwait`（100ms watchdog 超时）上等待
- **时间对齐**：取 `lidar_buf` 最新帧 → `message_buffer_find_nearest(gps_buf, ref_ts, 50ms)` 找最近 GPS
- **EKF**：predict → update_lidar → update_gps → get_state/covariance → publish `fusion/localization`
- **延迟**：每 20 帧 publish `fusion/latency`
- **同步原语**：`pthread_mutex_t lidar_mu` + `pthread_cond_t lidar_cv`（CLOCK_MONOTONIC）

### 3.2 两种迁移方案

**方案 A：复用 [FusionNodeCpp](file:///workspace/include/fusion.h#L183-L212) 基类**
- 继承 `FusionNodeCpp`，实现 `Message Fuse(const SyncedFrame&) override`
- **问题**：当前 [fusion_cpp.cpp:58-70](file:///workspace/src/cpp/fusion_cpp.cpp#L58-L70) 的 `run()` 是空挂起半成品，且基类内部仍走 C 侧 `fusion_node_*`（create/start/callback），协程与 C 回调耦合，需重写框架代码
- 工作量大，要改 `src/cpp/fusion_cpp.cpp` + `include/fusion.h`，风险波及框架

**方案 B：仿 safety_control 新写 CoroutineTask 子类（推荐）**
- 新建 `fusion_node.cpp`（替换 `fusion_node.c`），自包含，不碰框架
- 保留 `MessageBuffer` 时间对齐逻辑（on_lidar/on_gps 回调 push 到 buf），仅用 `co_await when_any_bus` 替代 condvar 等待
- 与 safety_control 同构，范式已验证，风险最低

### 3.3 方案 B 代码骨架

```cpp
class FusionTask : public CoroutineTask {
public:
    FusionTask(MessageBus* bus, Transport* transport, MessageBuffer* lidar_buf,
               MessageBuffer* gps_buf, EkfFusion* ekf, LatencyTracker* lat)
        : CoroutineTask(bus), transport_(transport),
          lidar_buf_(lidar_buf), gps_buf_(gps_buf),
          ekf_(ekf), lat_tracker_(lat), fused_count_(0) {}

protected:
    Task run() override {
        LOG_INFO("fusion", "FlowCoro fusion started");
        while (!should_stop()) {
            // 替代 pthread_cond_timedwait(100ms)：100ms 超时作 watchdog
            auto res = co_await when_any_bus_for(
                bus(), {"sensor/lidar", "sensor/gps"}, 100000, &cancel_token_);
            if (should_stop()) break;

            const Message* lidar_msg = message_buffer_latest(lidar_buf_);
            if (!lidar_msg) continue;
            uint64_t ref_ts = lidar_msg->timestamp_us;
            const Message* gps_msg = message_buffer_find_nearest(gps_buf_, ref_ts, 50000);

            const LidarFrame* lidar = msg_cast<LidarFrame>(lidar_msg);
            const GpsData* gps = gps_msg ? msg_cast<GpsData>(gps_msg) : nullptr;
            if (!lidar) continue;

            // ── EKF（直搬 fusion_thread 行 126-164）──
            ekf_fusion_predict(ekf_);
            ekf_fusion_update_lidar(ekf_, lidar->x, lidar->y, nullptr);
            if (gps) {
                double hdg = gps->heading_deg * M_PI / 180.0;
                ekf_fusion_update_gps(ekf_, gps->speed_mps, hdg, nullptr);
            }
            double x, y, v, h, yr, diag[5];
            ekf_fusion_get_state(ekf_, &x, &y, &v, &h, &yr);
            ekf_fusion_get_covariance_diag(ekf_, diag);
            ++fused_count_;
            if (ekf_->diverged && fused_count_ % 10 == 0) ekf_fusion_reset(ekf_);

            // ── publish fusion/localization（直搬行 152-173）──
            Localization loc; /* 填充字段... */
            uint8_t buf[128]; size_t len = sizeof(buf);
            Localization_serialize(&loc, buf, &len);
            transport_publish(transport_, "fusion/localization", buf, (uint32_t)len);

            // ── 每 20 帧 publish fusion/latency（直搬行 182-201）──
            // ...
        }
        LOG_INFO("fusion", "FlowCoro fusion stopped (%u frames)", fused_count_);
    }

private:
    Transport* transport_;
    MessageBuffer* lidar_buf_, *gps_buf_;
    EkfFusion* ekf_;
    LatencyTracker* lat_tracker_;
    uint32_t fused_count_;
};
```

### 3.4 逐行映射表

| fusion_node.c (C) | fusion_node.cpp (FlowCoro) |
|--------------------|---------------------------|
| `on_lidar` push buf + `cond_signal` | `on_lidar` push buf（**去掉 cond_signal**，when_any_bus 内部订阅负责唤醒） |
| `on_gps` push buf | `on_gps` push buf（不变） |
| `pthread_cond_timedwait(100ms)` | `co_await when_any_bus_for(..., 100000, &cancel_token_)` |
| `should_stop` + `cond_broadcast` | `task->stop()`（CancelToken 唤醒） |
| `pthread_mutex_destroy` / `cond_destroy` | 删除（协程框架接管） |
| EKF / publish 逻辑 | **原样搬入 run()**（业务逻辑零改动） |

### 3.5 注意点

- `MessageBuffer` 保留：`when_any_bus` 只负责唤醒，时间对齐仍由 `message_buffer_find_nearest` 完成。这比用 `BusChannel` 重写时间对齐更稳妥（BusChannel 是队列消费模型，无 nearest 查找）。
- `on_lidar` 回调仍需注册（push 到 buf），但**删除 cond_signal**——`when_any_bus` 的内部订阅回调会直接唤醒协程，双重唤醒冗余。
- watchdog 超时保留：lidar 停发时 100ms 超时让协程醒来检查 `should_stop`，与原逻辑等价。

---

## 4. 其它候选节点迁移要点

### perception（[perception_node.c](file:///workspace/modules/adas_nodes/perception_node.c)）
- **现状**：`usleep(period_us)` 定时轮询，从 `g.ego_*`（vehicle/state 回调更新）生成点云 + DBSCAN
- **迁移**：`co_await subscribe_once(bus(), "vehicle/state", &cancel_token_)` 事件驱动；DBSCAN 逻辑原样搬入 `run()`
- **收益**：消除固定频率轮询，按 vehicle/state 到达节奏处理

### planning（[planning_node.c](file:///workspace/modules/adas_nodes/planning_node.c)）
- **现状**：`usleep(50000)` 20Hz 轮询，检查 `g.has_fusion`；订阅 fusion/localization + vehicle/state + road/geometry
- **迁移**：`co_await when_any_bus({"fusion/localization","vehicle/state","road/geometry"})`；驾驶模式仲裁 + 路线变道逻辑原样搬入
- **复杂点**：已有 frenet_bridge.cpp C++ 运行时基底，迁移友好；但状态机逻辑较长（行 252-300+），需仔细搬移

### control（[control_node.c](file:///workspace/modules/adas_nodes/control_node.c)）
- **现状**：订阅 4 topic，PID 控制对时序敏感
- **迁移**：仿 safety_control 选**同步 resume**（CoroutineTask，非 FlowCoroTask），避免线程池饥饿导致 control/cmd 陈旧（参考 [CMakeLists.txt:138-141](file:///workspace/modules/adas_nodes/CMakeLists.txt#L138-L141) 注释）
- **风险**：control loop 延迟敏感，迁移后必须验证端到端延迟不退化

### inference（[inference_node.c](file:///workspace/modules/adas_nodes/inference_node.c)）
- **现状**：`usleep(sleep_us)` 定频，shadow 推理 + OTA 热重载
- **迁移**：`co_await when_any_bus({"fusion/localization","perception/obstacles",...})`；OTA reload 检查改为 `co_await delay_ms` 周期触发
- **收益**：shadow 推理协程化后可与主控解耦

---

## 5. CMake 改动

子项目 [modules/adas_nodes/CMakeLists.txt](file:///workspace/modules/adas_nodes/CMakeLists.txt) 第 33 行**已包含 flowcoro 头文件目录**：

```cmake
include_directories(
    ...
    ${FLOWENGINE_BUILD}/_deps/flowcoro-src/include   # ← 已存在
)
```

### 5.1 同步 resume 模式（推荐，仿 safety_control）

仅把 `.c` 换成 `.cpp` + 加 `stdc++`：

```cmake
# 原（行 93-94）：
add_library(fusion_node SHARED fusion_node.c)
target_link_libraries(fusion_node ${NODE_LINK_LIBS})

# 改为：
add_library(fusion_node SHARED fusion_node.cpp)
target_link_libraries(fusion_node ${NODE_LINK_LIBS} stdc++)
```

`.so` 名不变（`libfusion_node.so`），`pipeline.json` 无需改。

### 5.2 线程池 resume 模式（FlowCoroTask，可选）

若重算法节点要用线程池 resume，需额外：

```cmake
target_compile_definitions(fusion_node PRIVATE FLOWCORO_INTEGRATION)
# 子项目无 flowcoro_headers target，需手动 link flowcoro 库：
find_library(FLOWCORO_LIB NAMES flowcoro HINTS ${FLOWENGINE_BUILD}/_deps/flowcoro-build)
target_link_libraries(fusion_node ${NODE_LINK_LIBS} stdc++ ${FLOWCORO_LIB})
```

**建议**：除 control 外，所有候选节点先用同步 resume（CoroutineTask），与 safety_control 一致，CMake 改动最小、风险最低。

---

## 6. 风险点

| 风险 | 影响 | 缓解 |
|------|------|------|
| **resume 模式选错** | FlowCoroTask 线程池 resume 可能导致 control/cmd 陈旧（safety_control 已踩坑） | 默认用 CoroutineTask 同步 resume；control 节点必须用同步 |
| **时间对齐语义丢失** | fusion 的 `message_buffer_find_nearest` 依赖缓冲而非队列消费，若误用 BusChannel 替代会破坏对齐 | 方案 B 保留 MessageBuffer，when_any_bus 仅作唤醒 |
| **回调双重唤醒** | on_lidar 的 cond_signal 与 when_any_bus 订阅同时存在会冗余唤醒 | 迁移时删除 cond_signal/cond_wait，仅保留 push 到 buf |
| **向后兼容** | `.so` 名或导出符号变化会导致 flow_launcher 加载失败 | 保持 `lib<name>.so` 名 + `node_get_plugin` 符号不变；pipeline.json 不改 |
| **协程取消不彻底** | `stop()` 后协程仍挂在 `co_await` 上导致 `pthread_join` 卡死 | 用 `task->stop()` 触发 CancelToken；cleanup 中先 stop 再 join（仿 safety_control 行 494-507） |
| **C++ 异常** | 协程内未捕获异常会泄漏到 `execute()` | `run()` 外层套 try/catch（仿 safety_thread 行 441-445） |
| **性能回归** | 同步 resume 在 bus 分发线程执行业务，重算法会阻塞总线分发 | fusion EKF 轻量（<100us）安全；重节点评估后考虑 FlowCoroTask |

---

## 7. 验证策略

每个节点迁移后执行：

1. **单元编译**：`./build.sh release` 确认 .so 生成
2. **场景回归**：`python3 tools/scenario_regression.py` 全 11 场景 PASS
3. **频率检查**：`demo_evaluator.py` 的 `TOPIC_MIN_FREQ` 确认输出频率不退化
4. **延迟对比**：fusion/latency 的 p50/p99 迁移前后对比
5. **ctest**：13/13 PASS（含 e2e_stability 10 分钟压测）
6. **长时间稳定性**：单场景跑 5 分钟观察无死锁/内存泄漏

---

## 8. 推荐执行顺序

1. **fusion**（方案 B）—— 验证事件驱动节点迁移范式，最高收益
2. **perception** —— 验证单 topic 轮询节点迁移范式
3. **planning** —— 验证多 topic + 状态机节点迁移
4. **control**（同步 resume，谨慎）—— 时序敏感，迁移后必跑延迟对比
5. **inference** —— shadow 推理，收益中等

每步独立提交，回归通过后再进入下一步。fusion 完成后即可确认范式是否值得推广。
