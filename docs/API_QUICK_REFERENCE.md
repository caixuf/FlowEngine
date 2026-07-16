# FlowEngine API 快速参考

## Message Bus

```c
MessageBus* bus = message_bus_create("name");

// 发布/订阅
message_bus_publish(bus, "topic", "sender", &data, sizeof(data));
message_bus_subscribe(bus, "topic", callback, user_data);

// 零拷贝
message_bus_publish_zero_copy(bus, "topic", "sender", &data, sizeof(data));
message_bus_subscribe_zero_copy(bus, "topic", zc_callback, user_data);

// 请求/应答
message_bus_register_service(bus, "service", handler, user_data);
message_bus_request(bus, "service", "sender", &req, sizeof(req), &reply, 2000);

// QoS
message_bus_set_topic_qos(bus, "topic", &(TopicQos){.depth=32, .policy=QOS_DROP_OLDEST});
message_bus_get_topic_stats(bus, "topic", &stats);
message_bus_list_topics(bus, topics, 64);

// 统计
message_bus_get_stats(bus, &pub, &del, &drop);
message_bus_destroy(bus);
```

## Serializer

```c
adas_msgs_register_all();  // 注册 ADAS 类型

// 类型安全访问
const LidarFrame* f = msg_cast<LidarFrame>(msg);       // C++
const LidarFrame* f = msg_cast(msg, TYPE_ID, sizeof(LidarFrame)); // C

// 类型化消息构造
msg_init_typed(&msg, "topic", "sender", TYPE_ID, SCHEMA_VERSION, &data, sizeof(data));

// 运行时注册表
serializer_register_type(&entry);
serializer_lookup_type(type_id);
serializer_type_count();
```

## Scheduler

```c
Scheduler* sched = scheduler_create(&(SchedulerConfig){
    .mode = SCHEDULER_MODE_CHOREO
});
scheduler_set_choreo_bus(sched, bus);
scheduler_register_task(sched, task, "name");
scheduler_set_params(sched, tid, TASK_PRIORITY_CRITICAL, 0x01, 10.0);

// Choreo
scheduler_choreo_trigger_on(sched, tid, "topic");
scheduler_choreo_wait(sched, tid, 500000);

// 监控
scheduler_get_latency(sched, tid);
scheduler_get_rate_control(sched, tid);
scheduler_start(sched);
```

## State Machine

```c
statem_init(&sm, SM_TABLE_STANDARD, SM_STATE_INITIALIZED, "task");
statem_send_event(&sm, SM_EVENT_START, task);

// 反射
statem_can_transition(&sm, SM_EVENT_STOP);
statem_allowed_events(&sm, events, 16);
statem_dump_table(&sm);          // 打印完整表
statem_export_json(&sm);         // JSON 导出

// 动态控制
statem_set_guard(&sm, my_guard);
statem_add_transition(&sm, from, ev, to, "desc");
statem_set_trace(&sm, true);
```

## Transport（统一）

```c
Transport* t = transport_create(bus, discovery, TRANSPORT_AUTO);
transport_advertise(t, "topic", TYPE_ID);
transport_publish(t, "topic", &data, sizeof(data));
transport_subscribe(t, "topic", callback, user_data);
transport_start(t);
```

## Discovery

```c
DiscoveryManager* dm = discovery_create("node", CAP_PUBLISHER);
discovery_advertise(dm, "topic", TYPE_ID, CAP_PUBLISHER, 10.0);
discovery_start(dm);
discovery_get_topology(dm);
discovery_export_json(dm);
discovery_print_graph(dm);
```

## Fusion

```c
FusionNode* fn = fusion_node_create("name", bus, &policy);
fusion_node_add_input(fn, "sensor/lidar", TYPE_ID, 32);
fusion_node_set_output(fn, "fusion/out", TYPE_ID);
fusion_node_set_callback(fn, my_callback, user_data);
fusion_node_start(fn);

// C++ 协程基类
class MyFusion : public FusionNodeCpp {
    Message Fuse(const SyncedFrame& f) override { ... }
};
```

## Logger

```c
log_init(LOG_INFO, NULL);
LOG_INFO("module", "message %d", value);
LOG_WARN("module", "warning: %s", reason);
LOG_ERROR("module", "error: %d", code);
log_set_module_level("discovery", LOG_WARN);
log_shutdown();
```

## FlowRegistry

```c
flow_registry_register_task("name", "desc", "plugin.so", inputs, outputs, params);
flow_registry_register_topic("name", TYPE_ID, NULL);
flow_registry_register_plugin("name", "path.so", tasks, types);
flow_registry_get_task("name");
flow_registry_export_json();
```

## ParamRegistry

```c
param_register_int("control.max_speed", 120, 0, 200, "Max speed km/h");
param_register_float("fusion.max_delta_ms", 50.0, 10.0, 500.0, "Window");
param_register_bool("control.aeb", true, "Enable AEB");

int speed = param_get_int("control.max_speed");
param_set_int("control.max_speed", 100);  // 已校验
param_enable_hot_reload("control.max_speed");
```

## Stats Bridge（跨进程 IPC）

```c
// 发布端 —— 在业务进程启动时调用一次
IpcChannel* ch = stats_bridge_publisher_open();

// 序列化 MessageBus 统计并发送给 flowmond（周期性调用，例如每 5 秒）
stats_bridge_publish(ch, bus, "flow_launcher");

// 订阅端 —— 在 flowmond 中调用；在发布端打开通道前返回 NULL
IpcChannel* sub = stats_bridge_subscriber_open(on_stats_callback, user_data);
if (sub) ipc_channel_start(sub);   // 启动非阻塞后台接收线程

// 清理
ipc_channel_close(ch);
ipc_channel_close(sub);
```

回调签名：

```c
void on_stats_callback(const Message* msg, void* user_data) {
    const StatsPacket* pkt = (const StatsPacket*)msg->data;
    // pkt->source_name —— 发送进程（如 "flow_launcher"）
    // pkt->topic_count —— pkt->topics[] 中的条目数
    // pkt->bus_pub / bus_del / bus_drop —— 总线聚合计数器
}
```

## Bag

```c
BagWriter* w = bag_writer_open("out.bag");
bag_writer_attach(w, bus);
bag_writer_close(w);

BagReader* r = bag_reader_open("out.bag");
bag_reader_info(r, &count, &duration);
bag_reader_play(r, bus, 1.0f);
bag_reader_get_topics(r, topics, 64, counts);
bag_reader_close(r);
```

## CLI

```bash
flowctl list tasks|topics|plugins
flowctl graph
flowctl state <task>
flowctl topic stats <topic>
flowctl bag info|check <file>
flowctl schema <type>
flowctl param list|get|set
flowctl registry
flowctl dashboard
flowctl version
```

## 启动配置

```json
{
  "scheduler": {"mode": "choreo"},
  "nodes": [{
    "name": "perception",
    "plugin": "lib/libperception.so",
    "publish": [{"topic": "sensor/lidar", "qos": {"depth": 32, "policy": "drop_oldest"}}],
    "scheduling": {"priority": "critical", "cpu_affinity": [0], "max_frequency_hz": 10.0},
    "resources": {"max_memory_mb": 256}
  }]
}
```
