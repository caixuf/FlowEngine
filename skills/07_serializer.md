# 07 — 类型安全序列化层

## 问题

原始 FlowEngine 的消息传递是 `void* + size`，没有类型安全：

```c
// 旧方式 — 容易出错
const LidarFrame* f = (const LidarFrame*)msg->data;  // 如果 data 是 GpsData 呢？
```

## 解决方案

**类型 ID (FNV-1a hash)** + **代码生成器** + **运行时注册表**

```
.msg IDL → msg_codegen.py → _gen.h (struct + type_id + serialize/deserialize)
                          → 运行时 TypeRegistry
```

## 快速开始

### 1. 定义消息类型 (`msg/my_msgs.msg`)

```
struct SensorData {
    float   value
    uint32  seq
    float64 timestamp
}
```

### 2. 生成代码

```bash
python3 tools/msg_codegen.py msg/my_msgs.msg build/gen/my_msgs_gen.h
```

### 3. 使用类型安全访问

```c
#include "my_msgs_gen.h"

// 初始化时注册类型
sensor_data_register_type();

// 发送带类型 ID 的消息
SensorData sd = { .value = 42.0f, .seq = 1, .timestamp = 123456.0 };
Message msg;
msg_init_typed(&msg, "sensor/data", "my_node",
               SENSORDATA_TYPE_ID, SENSORDATA_SCHEMA_VERSION,
               &sd, sizeof(sd));
message_bus_publish(bus, msg.topic, msg.sender, msg.data, msg.data_size);

// 接收时类型安全访问
const SensorData* sd = msg_cast<SensorData>(msg);  // C++
const SensorData* sd = msg_cast(msg, SENSORDATA_TYPE_ID, sizeof(SensorData));  // C
if (sd) {
    printf("value=%f\n", sd->value);
}
```

## API 速查

| 函数 | 用途 |
|------|------|
| `msg_cast<T>(msg)` | C++ 类型安全转换 |
| `msg_cast(msg, type_id, size)` | C 类型安全转换 |
| `msg_init_typed()` | 构造带类型 ID 的消息 |
| `serializer_register_type()` | 运行时注册类型 |
| `serializer_lookup_type()` | 按 type_id 查找 |
| `fnv1a_hash()` | FNV-1a 32-bit hash |

## 字节序

- LE 平台：`endian_marker` = `0x12`
- BE 平台：`endian_marker` = `0x21`
- `serializer_ensure_endian()` 自动转换
- Bag v2 格式保存字节序标记，跨平台回放自动处理
