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
| `serializer_lookup_by_name()` | 按类型名查找 |
| `serializer_check_compat()` | 跨版本 schema 兼容性判定 |
| `fnv1a_hash()` | FNV-1a 32-bit hash |

## 字段级 schema、版本与哈希

每个类型除了 `type_id`（按类型名派生的稳定标识）外，codegen 还生成：

- **`<TYPE>_SCHEMA_VERSION`** — 显式版本号（默认 1），字段发生增删时手动递增。
- **`<TYPE>_SCHEMA_HASH`** — 字段级布局的 FNV-1a 哈希（字段名 + 类型 + 数组长度）。
  布局改变时哈希自动改变，无需人工维护。
- **`<Type>_fields[]`** — `FieldDesc` 字段描述表（名称 / 种类 / 偏移 / 单元素大小 / 数组长度），
  注册进 `TypeRegistryEntry`，实现自描述。

`flowctl schema <type>` 可查看类型的版本、哈希与全部字段：

```bash
flowctl schema LidarFrame     # 打印 type_id / schema 版本 / hash / 字段表
```

### 跨版本兼容性策略

`serializer_check_compat(type_name, their_version, their_hash)` 返回：

| 结果 | 条件 | 含义 |
|------|------|------|
| `SCHEMA_IDENTICAL` | hash 相同 | 完全一致 |
| `SCHEMA_COMPATIBLE` | hash 不同且**版本号不同** | 视为字段尾部增删的版本演进，反序列化端补零/截断尽力兼容 |
| `SCHEMA_INCOMPATIBLE` | hash 不同但**版本号相同** | 未升版的破坏性变更，拒绝 |
| `SCHEMA_UNKNOWN` | 本地未注册该类型 | 无法判定 |

**约定**：任何改动字段布局的变更都必须递增 `SCHEMA_VERSION`；否则会被判定为
`SCHEMA_INCOMPATIBLE`（防止“悄悄改布局”导致的静默数据损坏）。

## 字节序

- LE 平台：`endian_marker` = `0x12`
- BE 平台：`endian_marker` = `0x21`
- `serializer_ensure_endian()` 自动转换
- Bag v2 格式保存字节序标记，跨平台回放自动处理

## Bag 自描述回放

Bag v2 的每条记录与索引项都保存 `type_id` + `schema_version`，因此录制文件是
**自描述**的，无需外部 schema 即可回放并识别类型：

```bash
flowctl bag info out.bag      # 每个 topic 显示 type_id 与 schema 版本
```

