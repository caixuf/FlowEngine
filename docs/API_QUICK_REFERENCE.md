# FlowEngine API Quick Reference

## Message Bus

```c
MessageBus* bus = message_bus_create("name");

// Pub/Sub
message_bus_publish(bus, "topic", "sender", &data, sizeof(data));
message_bus_subscribe(bus, "topic", callback, user_data);

// Zero-copy
message_bus_publish_zero_copy(bus, "topic", "sender", &data, sizeof(data));
message_bus_subscribe_zero_copy(bus, "topic", zc_callback, user_data);

// Req/Reply
message_bus_register_service(bus, "service", handler, user_data);
message_bus_request(bus, "service", "sender", &req, sizeof(req), &reply, 2000);

// QoS
message_bus_set_topic_qos(bus, "topic", &(TopicQos){.depth=32, .policy=QOS_DROP_OLDEST});
message_bus_get_topic_stats(bus, "topic", &stats);
message_bus_list_topics(bus, topics, 64);

// Stats
message_bus_get_stats(bus, &pub, &del, &drop);
message_bus_destroy(bus);
```

## Serializer

```c
adas_msgs_register_all();  // register ADAS types

// Type-safe access
const LidarFrame* f = msg_cast<LidarFrame>(msg);       // C++
const LidarFrame* f = msg_cast(msg, TYPE_ID, sizeof(LidarFrame)); // C

// Typed message construction
msg_init_typed(&msg, "topic", "sender", TYPE_ID, SCHEMA_VERSION, &data, sizeof(data));

// Runtime registry
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

// Monitoring
scheduler_get_latency(sched, tid);
scheduler_get_rate_control(sched, tid);
scheduler_start(sched);
```

## State Machine

```c
statem_init(&sm, SM_TABLE_STANDARD, SM_STATE_INITIALIZED, "task");
statem_send_event(&sm, SM_EVENT_START, task);

// Reflection
statem_can_transition(&sm, SM_EVENT_STOP);
statem_allowed_events(&sm, events, 16);
statem_dump_table(&sm);          // print full table
statem_export_json(&sm);         // JSON export

// Dynamic control
statem_set_guard(&sm, my_guard);
statem_add_transition(&sm, from, ev, to, "desc");
statem_set_trace(&sm, true);
```

## Transport (Unified)

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

// C++ coroutine base
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
param_set_int("control.max_speed", 100);  // validated
param_enable_hot_reload("control.max_speed");
```

## Stats Bridge (Cross-process IPC)

```c
// Publisher side — call once at startup in the business process
IpcChannel* ch = stats_bridge_publisher_open();

// Serialize MessageBus stats and send to flowmond (call periodically, e.g. every 5 s)
stats_bridge_publish(ch, bus, "flow_e2e");

// Subscriber side — call in flowmond; returns NULL until publisher opens the channel
IpcChannel* sub = stats_bridge_subscriber_open(on_stats_callback, user_data);
if (sub) ipc_channel_start(sub);   // starts non-blocking background receive thread

// Cleanup
ipc_channel_close(ch);
ipc_channel_close(sub);
```

Callback signature:

```c
void on_stats_callback(const Message* msg, void* user_data) {
    const StatsPacket* pkt = (const StatsPacket*)msg->data;
    // pkt->source_name — sending process (e.g. "flow_e2e")
    // pkt->topic_count — number of entries in pkt->topics[]
    // pkt->bus_pub / bus_del / bus_drop — aggregate bus counters
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

## Launch Config

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
