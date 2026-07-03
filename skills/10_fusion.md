# 10 — 数据融合框架

## 核心组件

```
MessageBuffer (环形缓冲) → find_nearest (时间对齐) → SyncedFrame → Fuse() → publish
```

## 快速开始 — C++ 协程方式

```cpp
#include "fusion.h"
#include "adas_msgs_gen.h"

class SensorFusion : public FusionNodeCpp {
public:
    SensorFusion(MessageBus* bus)
        : FusionNodeCpp(bus, FUSION_POLICY_TIME_ALIGNED)
    {
        AddSensorInput("sensor/lidar", LIDARFRAME_TYPE_ID, 32);
        AddSensorInput("sensor/gps",   GPSDATA_TYPE_ID,    16);
        SetOutputTopic("fusion/localization", FUSEDLOC_TYPE_ID);
    }

protected:
    Message Fuse(const SyncedFrame& frame) override {
        const auto* lidar = msg_cast<LidarFrame>(&frame.inputs[0]);
        const auto* gps   = msg_cast<GpsData>(&frame.inputs[1]);

        FusedLocalization out = {};
        out.timestamp_us = frame.reference_ts;
        if (lidar) { out.x = lidar->x; out.y = lidar->y; }
        if (gps)   { out.lat = gps->latitude; out.lon = gps->longitude; }

        Message msg;
        msg_init_typed(&msg, "fusion/localization", "fusion",
                       FUSEDLOC_TYPE_ID, 1, &out, sizeof(out));
        return msg;
    }
};
```

## 快速开始 — C API

```c
FusionNode* fn = fusion_node_create("my_fusion", bus, &policy);
fusion_node_add_input(fn, "sensor/lidar", LIDARFRAME_TYPE_ID, 32);
fusion_node_add_input(fn, "sensor/gps",   GPSDATA_TYPE_ID,    16);
fusion_node_set_output(fn, "fusion/output", FUSEDOUT_TYPE_ID);
fusion_node_set_callback(fn, my_fusion_callback, user_data);
fusion_node_start(fn);

// 消息到达时自动触发回调 → build_synced_frame → callback
```

## 融合策略

| 策略 | 描述 |
|------|------|
| `FUSION_TIME_ALIGNED` | 时间对齐到最新帧（默认，max_delta_us） |
| `FUSION_LATEST_WINS` | 使用每个输入的最新值 |
| `FUSION_WEIGHTED_AVERAGE` | 加权平均（同类型传感器） |
| `FUSION_KALMAN_SLOT` | 卡尔曼滤波器槽位 |

## 时间对齐算法

```
1. 找到所有缓冲中最新的时间戳作为 reference_ts
2. 对每个输入，find_nearest(buffer, reference_ts, max_delta_us)
3. 所有在 max_delta 内的 → SyncedFrame.input_valid[i] = true
4. 至少一个有效 → 触发 callback / Fuse()
```

## API 速查

| 函数 | 用途 |
|------|------|
| `message_buffer_create()` | 创建传感器缓冲 |
| `message_buffer_push()` | 推入消息 |
| `message_buffer_find_nearest()` | 时间戳最近查找 |
| `message_buffer_latest()` | 获取最新消息 |
| `fusion_node_create()` | 创建融合节点 (C) |
| `fusion_node_add_input()` | 添加传感器输入 |
| `fusion_node_start()` | 启动融合线程 |
| `FusionNodeCpp::Fuse()` | 重写实现融合逻辑 (C++) |
