# Skill 05 - 数据录制与回放（Bag）

## 核心思想

Bag 模块将运行时的消息流录制到文件，之后可以**离线回放**，让系统以为数据仍然来自真实传感器。这是机器人/自动驾驶领域的标准调试手段（类比 ROS bag）。

## 核心价值

- **复现线上问题** — 将现场数据录制下来，回到办公室离线调试
- **算法迭代** — 同一份数据反复跑不同版本的算法，比较效果
- **回归测试** — 将典型场景的 bag 作为测试用例，自动化验证
- **数据标注** — 离线慢速回放，方便人工标注

## 文件格式

```
┌──────────────────────────────┐
│   Bag File Header            │  魔数 + 版本 + 时间戳 + 话题索引偏移
├──────────────────────────────┤
│   Record 0                   │  时间戳 | topic_len | topic | data_len | data
├──────────────────────────────┤
│   Record 1                   │
│   ...                        │
├──────────────────────────────┤
│   Topic Index                │  话题名 → 文件偏移列表（可选，加速随机访问）
└──────────────────────────────┘
```

## 录制

```c
#include "bag.h"

// 打开 bag 文件用于录制
BagWriter* writer = bag_writer_open("flight_20250722.bag");

// 在消息总线回调中录制
void on_gps(const char* topic, const void* data, size_t size, void* ud) {
    BagWriter* w = (BagWriter*)ud;
    bag_writer_write(w, topic, data, size);   // 自动记录当前时间戳
}

message_bus_subscribe(bus, "sensor/gps", on_gps, writer);
message_bus_subscribe(bus, "sensor/imu", on_gps, writer);  // 同一个回调

// 程序退出时关闭
bag_writer_close(writer);
```

## 回放

```c
#include "bag.h"
#include "clock_service.h"

// 切换到模拟时钟（由 bag 驱动时间）
clock_service_set_mode(CLOCK_MODE_SIM);

// 打开 bag 文件用于回放
BagReader* reader = bag_reader_open("flight_20250722.bag");

// 回放：按录制时间顺序投递消息到总线，1× 速度
bag_reader_play(reader, bus, 1.0f);

// 加速回放（3× 速度）
bag_reader_play(reader, bus, 3.0f);

// 只回放特定话题
const char* topics[] = {"sensor/gps", NULL};
bag_reader_play_filtered(reader, bus, topics, 1.0f);

bag_reader_close(reader);
```

## 与统一时钟的配合

回放时必须让系统时钟与 bag 内的时间戳同步，否则依赖时间的算法会出错：

```c
// 回放开始时，将模拟时钟设置为 bag 的起始时间
uint64_t start_ts = bag_reader_start_time(reader);
clock_service_set_sim_time(start_ts);

// 每投递一条消息后，推进模拟时钟到该消息的时间戳
clock_service_set_sim_time(record->timestamp);

// 算法代码始终通过 clock_service_now() 获取时间，
// 无需感知当前是录制还是回放模式
uint64_t now = clock_service_now();
```

## 注意事项

1. **时间戳精度** — 建议使用 `CLOCK_MONOTONIC_RAW`（纳秒级），避免系统时间跳变影响录制。
2. **文件大小** — 高频话题（如 100Hz 图像）会迅速产生大文件，录制前估算磁盘空间。
3. **话题过滤** — 只录制需要的话题，减小文件体积。
4. **索引** — 对长 bag 文件建立话题索引，支持按时间随机跳转。
5. **压缩** — 可选对数据块 LZ4 压缩，在 CPU 可接受范围内大幅减小文件体积。

## 参考文件

- `include/bag.h` — API 定义
- `src/core/bag.c` — 实现
- `src/bag_demo.c` — 录制与回放完整演示
- `include/clock_service.h` — 统一时钟 API
