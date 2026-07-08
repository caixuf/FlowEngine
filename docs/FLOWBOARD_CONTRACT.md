# FlowBoard 数据契约 (Data Contract)

> 可视化链路 `monitor_node.c → /tmp/flow_topology.json → flowboard_server.py → /api/topology → flowboard.html` 的**唯一事实来源**。
> 机器可读定义见 [`tools/schema/flowboard.schema.json`](../tools/schema/flowboard.schema.json)。

## 为什么需要契约

后端 `normalize_live_data()` 需要同时兼容三种输入格式:

| 来源 | 顶层形状 | 说明 |
|------|---------|------|
| `monitor_node.c` | `{self, nodes, metrics:{bus,latency,topics,vehicle,sysmon,...}, registry, scene}` | 生产链路的主格式 |
| `discovery` | `{self, nodes:[{name, caps, topics:[...]}]}` | peer 发现导出 |
| `NodePlugin` | `{nodes:[{name, inputs:[...], outputs:[...]}]}` | 插件拓扑,`inputs/outputs` 会被展开为 `topics[]` |

前端历史上到处用 `x || 0` / `x || []` 兜底,契约是隐式的。本文件把它显式化,让**三端(C 生产者 / Python 归一化 / 前端渲染)对齐同一份 schema**,并作为回归测试基准。

## 归一化输出 (`/api/topology`)

`normalize_live_data()` 把上述任意输入统一成:

```jsonc
{
  "self": "flow_launcher",
  "source": "live",          // live | stale | demo
  "stale": false,
  "age_sec": 0.1,
  "timestamp": 1700000000.0,
  "nodes":       [ /* node[]      */ ],
  "metrics":     { /* bus,latency,vehicle,sysmon,topics... */ },
  "endpoints":   [ /* (node,topic,role) 扁平表 */ ],
  "topic_roles": [ /* {topic,publishers[],subscribers[]} */ ]
}
```

字段类型与必填约束以 schema 为准。schema 刻意**允许 `additionalProperties`**:生产者可以增加字段而不破坏消费者,只有仪表盘真正依赖的字段才被约束。

## `source` 三态语义

| source | 含义 | 触发条件 | 前端表现 |
|--------|------|---------|---------|
| **live** | 实时新鲜数据 | 状态文件在 `STALE_AFTER_SEC` (5s) 内刷新过 | `● live` 绿色 |
| **stale** | server 可达但生产者停更 | 距上次成功读取 > 5s(**保留最后一帧真实数据**,不换假数据) | `● stale Ns` 黄色 |
| **demo** | 内置模拟数据 | `--demo` 或连接彻底失败 3 次后 fallback | `● demo` + 页面水印 |

关键原则:**stale 时继续显示最后的真实数据**,而不是静默切换到模拟值——避免把"生产者停了"误判为"链路坏了",也避免把假数字当成真的。

## 健康检查 (`/api/health`)

```jsonc
{
  "status": "ok",
  "source": "live",          // live | stale | demo
  "age_sec": 0.1,            // null when demo
  "read_failures": 0         // 累计状态文件读取失败次数 (JSON 损坏/半截写入等)
}
```

`read_failures` 单调递增,用于观测"半截 JSON / 文件竞态"的发生频率。理想情况下应保持为 0(生产侧已用 `.tmp` + `rename()` 原子写)。

## 原子写约定 (生产者侧)

`monitor_node.c::export_dashboard_json()` **必须**:

1. 写入 `"<state_file>.tmp"`;
2. `fclose()` 之后 `rename(tmp, state_file)`。

`rename(2)` 在同一文件系统上是原子的,保证 server 端永远读到完整 JSON,不会读到写一半的内容。任何新的状态文件生产者都应遵循此约定。

## 校验与降级

- **server 侧**:`normalize_live_data()` 入口对原始数据做一次轻量 schema 校验(`validate_payload()`)。校验失败**不会**让脏数据流进渲染——记录原因并退化到空/上一帧,而不是抛异常。
- **前端侧**:收到数据后集中做一次兜底(缺字段补默认值),替代散落各处的 `|| 0`。
- **测试侧**:`tools/tests/` 下 pytest 用本 schema 校验 Python sample data、前端 demo data 与归一化输出,防止某一端偷偷改格式导致联调断裂。
