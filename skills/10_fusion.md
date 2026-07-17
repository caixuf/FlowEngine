# 10 — 数据融合框架

## 核心组件

```
MessageBuffer (环形缓冲) → find_nearest (时间对齐) → Fuse 逻辑 → publish
```

## 生产范式 — FlowCoroTask + MessageBuffer

项目生产融合节点（modules/adas_nodes/fusion_node.cpp）采用此范式：
回调仅 push 到 MessageBuffer，协程体用 select_for 等待输入 + message_buffer_find_nearest
做时间对齐，再执行融合逻辑并 transport_publish。

```cpp
#include "fusion.h"
#include "coroutine_task.h"
#include "adas_msgs_gen.h"

class SensorFusion : public FlowCoroTask {
public:
    SensorFusion(MessageBus* bus, Transport* t,
                 MessageBuffer* lidar_buf, MessageBuffer* gps_buf)
        : FlowCoroTask(bus), transport_(t),
          lidar_buf_(lidar_buf), gps_buf_(gps_buf) {}

protected:
    Task run() override {
        while (!should_stop()) {
            // 等任一输入到达或 100ms 超时（超时作 watchdog，防止 lidar 停发卡死）
            co_await select_for({"sensor/lidar", "sensor/gps"}, 100000);
            if (should_stop()) break;

            const Message* lidar_msg = message_buffer_latest(lidar_buf_);
            if (!lidar_msg) continue;
            uint64_t ref_ts = lidar_msg->timestamp_us;

            // 时间对齐：找 50ms 窗口内最近的 GPS
            const Message* gps_msg = message_buffer_find_nearest(gps_buf_, ref_ts, 50000);

            const LidarFrame* lidar = (const LidarFrame*)lidar_msg->data;
            // ... 融合计算 ...
            transport_publish(transport_, "fusion/localization", out_buf, out_len);
        }
    }

private:
    Transport* transport_;
    MessageBuffer* lidar_buf_;
    MessageBuffer* gps_buf_;
};

// 订阅回调：仅 push 到 buf，不做计算（避免阻塞总线分发线程）
static void on_lidar(const Message* m, void*) { message_buffer_push(g.lidar_buf, m); }
static void on_gps(const Message* m, void*)   { message_buffer_push(g.gps_buf,   m); }
```

## 时间对齐算法

```
1. select_for 等待任一输入到达（或超时）
2. 取主输入最新帧的 timestamp_us 作 reference_ts
3. 对其他输入，message_buffer_find_nearest(buf, ref_ts, max_delta_us)
4. 在 max_delta 内的视为有效，参与融合
```

## API 速查

| 函数 | 用途 |
|------|------|
| `message_buffer_create()` | 创建传感器环形缓冲 |
| `message_buffer_push()` | 回调里推入消息（值拷贝，线程安全） |
| `message_buffer_find_nearest()` | 时间戳最近邻查找（时间对齐核心） |
| `message_buffer_latest()` | 取最新消息 |
| `message_buffer_destroy()` | 销毁缓冲 |
| `FlowCoroTask::run()` | 重写协程体，co_await select_for 等输入 |
| `FlowCoroTask::select_for()` | 等任一 topic 消息或超时（自动注入 CancelToken） |

## 历史 API（不推荐新代码使用）

core/fusion.c 另提供 FusionNode C API 与 FusionNodeCpp C++ 基类（事件驱动时间对齐模型）。
生产代码已迁移到上述 FlowCoroTask 范式，这两个 API 保留供参考，新融合节点请参照
modules/adas_nodes/fusion_node.cpp。
