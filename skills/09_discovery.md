# 09 — 服务发现与拓扑管理

## 协议

UDP 组播 `239.255.0.100:5500`，消息：HELLO / HEARTBEAT / GOODBYE / QUERY

```
[magic:DISC(4B)|ver(1B)|type(1B)|name(64B)|pid(4B)|caps(1B)|
 topic_count(2B)|topics(N)|ipv4(4B)|port(2B)|crc32(4B)]
```

- 心跳: 每 2 秒
- 超时: 10 秒标记死亡
- 启动: 发 QUERY 获取已有节点

## 快速开始

```c
// 创建并启动
DiscoveryManager* dm = discovery_create("my_node",
    CAP_PUBLISHER | CAP_SUBSCRIBER);

// 广播本节点的 topics
discovery_advertise(dm, "sensor/lidar", LIDARFRAME_TYPE_ID, CAP_PUBLISHER, 10.0);
discovery_advertise(dm, "control/cmd",  CONTROLCMD_TYPE_ID,  CAP_SUBSCRIBER, 0);

discovery_start(dm);

// 等待依赖上线
const char* deps[] = {"perception_node", "fusion_node"};
discovery_wait_for_deps(dm, deps, 2, 30000);  // 30秒超时

// 查询拓扑
const TopologyGraph* g = discovery_get_topology(dm);
for (uint32_t i = 0; i < g->node_count; i++) {
    printf("  %s (pid=%u) %s\n", g->nodes[i].name, g->nodes[i].pid,
           g->nodes[i].alive ? "●" : "○");
}

// 导出/打印
char* json = discovery_export_json(dm);  // 可视化
discovery_print_graph(dm);               // ASCII 拓扑图

// 注册拓扑变更回调
void on_change(const TopologyGraph* g, const NodeInfo* node,
               bool joined, void* data) {
    printf("Node '%s' %s\n", node->name, joined ? "joined" : "left");
}
discovery_set_change_callback(dm, on_change, NULL);

// 自动创建 IPC 通道
int count = discovery_create_ipc_channels(dm, 32);
printf("Created %d IPC channels\n", count);

// 清理
discovery_stop(dm);
discovery_destroy(dm);
```

## 能力标志

| 标志 | 含义 |
|------|------|
| `CAP_PUBLISHER` | 发布消息 |
| `CAP_SUBSCRIBER` | 订阅消息 |
| `CAP_SERVICE` | 提供 Req/Reply 服务 |
| `CAP_FUSION` | 传感器融合节点 |

## 多进程集成演示

```bash
# 3 个终端分别运行：
./build/bin/flow_fullstack --role perception
./build/bin/flow_fullstack --role fusion
./build/bin/flow_fullstack --role control

# 或一键启动：
bash scripts/fullstack_demo.sh
```

拓扑输出示例：
```
[拓扑: control_node] 2 节点
  perception_node  ● topics=[sensor/lidar/pub, sensor/gps/pub]
  fusion_node      ● topics=[sensor/lidar/sub, sensor/gps/sub, fusion/localization/pub]
  relations:
    perception_node -> fusion_node (pub/sub)
```

> ⚠️ **实现状态提示**：Discovery 模块的 UDP 组播协议和拓扑追踪功能已实现，
> 但**基于 Discovery 的跨进程 topic 数据转发（IPC/TCP bridge）当前未实现**。
> 因此 `flowmond` 等监控节点虽然能通过 Discovery 发现业务节点，但无法获取
> 业务节点内部的 topic 统计数据。当前可用的跨进程监控链路为"状态文件 +
> `flowboard_server.py`"，详见 `docs/VISUALIZATION_ARCHITECTURE.md`。
