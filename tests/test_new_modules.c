/**
 * test_new_modules.c — 新增模块单元测试（Task Manager / IPC / Discovery / Bag / FlowRegistry）
 *
 * 与 test_modules.c 分开编译，避免链接时与 message_bus 的符号冲突。
 * 编译为 C（非 CXX），保持与核心库一致的 ABI。
 */

#include "serializer.h"
#include "state_machine.h"
#include "message_bus.h"
#include "error_codes.h"
#include "task_manager.h"
#include "process_manager.h"
#include "ipc_channel.h"
#include "discovery.h"
#include "bag.h"
#include "flow_registry.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <pthread.h>

/* ── 测试宏 ─────────────────────────────────────────────── */

static int g_passed = 0;
static int g_failed = 0;

#define TEST(name) \
    do { printf("  %-50s ", name); fflush(stdout); } while (0)

#define PASS() \
    do { printf("\xE2\x9C\x85 PASS\n"); g_passed++; } while (0)

#define FAIL(fmt, ...) \
    do { printf("\xE2\x9D\x8C FAIL: " fmt "\n", ##__VA_ARGS__); g_failed++; } while (0)

#define ASSERT(cond, fmt, ...) \
    do { if (!(cond)) { FAIL(fmt, ##__VA_ARGS__); return; } } while (0)

#define ASSERT_EQ(a, b, fmt, ...) \
    do { \
        int _a = (int)(a); \
        int _b = (int)(b); \
        if (_a != _b) { FAIL(fmt " (%d vs %d)", ##__VA_ARGS__, _a, _b); return; } \
    } while (0)

/* ── 辅助变量 ────────────────────────────────────────────── */

/* Minimal vtable for test tasks (task_base_init requires non-NULL vtable) */
static const TaskInterface g_test_vtable = {
    /* All fields are NULL — the task is a stub that does nothing */
    NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL
};

/* ── 辅助类型 ────────────────────────────────────────────── */

typedef struct {
    pthread_mutex_t m;
    int count;
    int max_val;
    int slow_us;
} BusCounter;

static void bus_counter_cb(const Message* msg, void* ud) {
    BusCounter* c = (BusCounter*)ud;
    int val = 0;
    if (msg->data_size >= sizeof(int)) memcpy(&val, msg->data, sizeof(int));
    if (c->slow_us > 0) usleep((unsigned int)c->slow_us);
    pthread_mutex_lock(&c->m);
    c->count++;
    if (val > c->max_val) c->max_val = val;
    pthread_mutex_unlock(&c->m);
}

/* ══════════════════════════════════════════════════════════ */
/* Message Bus — Basic                                        */
/* ══════════════════════════════════════════════════════════ */

static void test_bus_pub_sub_basic(void) {
    TEST("bus basic pub/sub");
    MessageBus* bus = message_bus_create("test_basic");
    ASSERT(bus != NULL, "bus create failed");

    BusCounter c = { PTHREAD_MUTEX_INITIALIZER, 0, 0, 0 };
    message_bus_subscribe(bus, "t/basic", bus_counter_cb, &c);

    int val = 42;
    message_bus_publish(bus, "t/basic", "tester", &val, sizeof(val));
    usleep(50000);

    ASSERT(c.count == 1, "expected 1 message, got %d", c.count);
    ASSERT(c.max_val == 42, "expected payload 42, got %d", c.max_val);

    message_bus_destroy(bus);
    PASS();
}

static void test_bus_pub_sub_multi(void) {
    TEST("bus pub/sub multiple topics");
    MessageBus* bus = message_bus_create("test_multi");
    BusCounter ca = { PTHREAD_MUTEX_INITIALIZER, 0, 0, 0 };
    BusCounter cb = { PTHREAD_MUTEX_INITIALIZER, 0, 0, 0 };
    message_bus_subscribe(bus, "t/a", bus_counter_cb, &ca);
    message_bus_subscribe(bus, "t/b", bus_counter_cb, &cb);

    int v = 1;
    message_bus_publish(bus, "t/a", "tester", &v, sizeof(v));
    v = 2;
    message_bus_publish(bus, "t/b", "tester", &v, sizeof(v));
    usleep(50000);

    ASSERT(ca.count == 1, "topic a: expected 1, got %d", ca.count);
    ASSERT(cb.count == 1, "topic b: expected 1, got %d", cb.count);

    message_bus_destroy(bus);
    PASS();
}

static void test_bus_wildcard_subscribe(void) {
    TEST("bus wildcard subscribe matches all");
    MessageBus* bus = message_bus_create("test_wild");
    BusCounter c = { PTHREAD_MUTEX_INITIALIZER, 0, 0, 0 };
    message_bus_subscribe(bus, "*", bus_counter_cb, &c);

    int v = 1;
    message_bus_publish(bus, "t/x", "tester", &v, sizeof(v));
    message_bus_publish(bus, "t/y", "tester", &v, sizeof(v));
    usleep(50000);

    ASSERT(c.count == 2, "wildcard should receive 2, got %d", c.count);
    message_bus_destroy(bus);
    PASS();
}

static void test_bus_list_topics(void) {
    TEST("bus list_topics returns topics");
    MessageBus* bus = message_bus_create("test_list");
    int v = 1;
    message_bus_publish(bus, "sensor/lidar", "tester", &v, sizeof(v));
    message_bus_publish(bus, "sensor/gps", "tester", &v, sizeof(v));
    usleep(50000);

    char topics[32][64];
    int n = message_bus_list_topics(bus, topics, 32);
    ASSERT(n >= 2, "expected >= 2 topics, got %d", n);

    int found_lidar = 0, found_gps = 0;
    for (int i = 0; i < n; i++) {
        if (strcmp(topics[i], "sensor/lidar") == 0) found_lidar = 1;
        if (strcmp(topics[i], "sensor/gps") == 0) found_gps = 1;
    }
    ASSERT(found_lidar && found_gps, "expected both lidar and gps topics");

    message_bus_destroy(bus);
    PASS();
}

static void test_bus_remap(void) {
    TEST("bus topic remap routes correctly");
    MessageBus* bus = message_bus_create("test_remap");
    BusCounter c = { PTHREAD_MUTEX_INITIALIZER, 0, 0, 0 };
    message_bus_subscribe(bus, "t/dst", bus_counter_cb, &c);
    message_bus_add_remap(bus, "t/src", "t/dst");

    int v = 7;
    message_bus_publish(bus, "t/src", "tester", &v, sizeof(v));
    usleep(50000);

    ASSERT(c.count == 1, "remap: expected 1 message on dst, got %d", c.count);
    message_bus_destroy(bus);
    PASS();
}

static void test_bus_zero_copy_basic(void) {
    TEST("bus zero-copy publish delivers");
    MessageBus* bus = message_bus_create("test_zc");
    BusCounter c = { PTHREAD_MUTEX_INITIALIZER, 0, 0, 0 };
    message_bus_subscribe(bus, "t/zc", bus_counter_cb, &c);

    int v = 55;
    int delivered = message_bus_publish_zero_copy(bus, "t/zc", "tester", &v, sizeof(v));
    ASSERT(delivered >= 0, "zero-copy publish should succeed");

    usleep(50000);
    ASSERT(c.count == 1, "zero-copy: expected 1 delivery, got %d", c.count);
    ASSERT(c.max_val == 55, "zero-copy: expected payload 55, got %d", c.max_val);

    message_bus_destroy(bus);
    PASS();
}

static void echo_service_handler(const Message* req, Message* reply, void* user_data) {
    (void)user_data;
    snprintf(reply->topic, 64, "%s", req->topic);
    reply->data_size = req->data_size;
    if (req->data_size > 0) memcpy(reply->data, req->data, req->data_size);
}

static void test_bus_req_reply(void) {
    TEST("bus req/reply round-trip");
    MessageBus* bus = message_bus_create("test_req");
    ASSERT(bus != NULL, "bus create failed");

    message_bus_register_service(bus, "svc/echo", echo_service_handler, NULL);

    int req_val = 99;
    Message reply;
    memset(&reply, 0, sizeof(reply));
    int rc = message_bus_request(bus, "svc/echo", "caller",
                                  &req_val, sizeof(req_val), &reply, 2000);
    ASSERT_EQ(rc, 0, "request failed");
    ASSERT(reply.data_size == sizeof(req_val), "reply size mismatch");
    int rep_val = 0;
    memcpy(&rep_val, reply.data, sizeof(rep_val));
    ASSERT(rep_val == 99, "echo value mismatch: got %d", rep_val);

    message_bus_destroy(bus);
    PASS();
}

/* ══════════════════════════════════════════════════════════ */
/* Task Manager Tests                                         */
/* ══════════════════════════════════════════════════════════ */

static void test_tmgr_create_destroy(void) {
    TEST("task_manager create/destroy");
    TaskManager* mgr = task_manager_create();
    ASSERT(mgr != NULL, "create failed");

    uint32_t total, running, error;
    task_manager_get_stats(mgr, &total, &running, &error);
    ASSERT(total == 0 && running == 0 && error == 0, "empty stats should be zero");

    task_manager_destroy(mgr);
    PASS();
}

static void test_tmgr_register(void) {
    TEST("task_manager register task");
    TaskManager* mgr = task_manager_create();
    TaskBase task;
    TaskConfig cfg;
    memset(&cfg, 0, sizeof(cfg));
    snprintf(cfg.name, 64, "tmgr_test");
    task_base_init(&task, &g_test_vtable, &cfg);

    ASSERT_EQ(task_manager_register(mgr, &task, "tmgr_test"), 0, "register failed");
    ASSERT(task_manager_get_task(mgr, "tmgr_test") != NULL, "get_task should find it");

    uint32_t total;
    task_manager_get_stats(mgr, &total, NULL, NULL);
    ASSERT(total == 1, "total should be 1, got %u", total);

    task_manager_destroy(mgr);
    PASS();
}

static void test_tmgr_list_tasks(void) {
    TEST("task_manager list_tasks");
    TaskManager* mgr = task_manager_create();
    TaskBase t1, t2;
    TaskConfig cfg;
    memset(&cfg, 0, sizeof(cfg));
    snprintf(cfg.name, 64, "t1");
    task_base_init(&t1, &g_test_vtable, &cfg);
    snprintf(cfg.name, 64, "t2");
    task_base_init(&t2, &g_test_vtable, &cfg);

    task_manager_register(mgr, &t1, "t1");
    task_manager_register(mgr, &t2, "t2");

    char names[8][64];
    int n = task_manager_list_tasks(mgr, names, 8);
    ASSERT(n == 2, "expected 2 tasks, got %d", n);

    int found1 = 0, found2 = 0;
    for (int i = 0; i < n; i++) {
        if (strcmp(names[i], "t1") == 0) found1 = 1;
        if (strcmp(names[i], "t2") == 0) found2 = 1;
    }
    ASSERT(found1 && found2, "should find both tasks");

    task_manager_destroy(mgr);
    PASS();
}

static void test_tmgr_health_check(void) {
    TEST("task_manager health_check all ok");
    TaskManager* mgr = task_manager_create();
    TaskBase task;
    TaskConfig cfg;
    memset(&cfg, 0, sizeof(cfg));
    snprintf(cfg.name, 64, "healthy");
    task_base_init(&task, &g_test_vtable, &cfg);
    task_manager_register(mgr, &task, "healthy");

    int unhealthy = task_manager_health_check(mgr);
    ASSERT(unhealthy == 0, "expected 0 unhealthy, got %d", unhealthy);

    task_manager_destroy(mgr);
    PASS();
}

static void test_tmgr_dependency(void) {
    TEST("task_manager add_dependency");
    TaskManager* mgr = task_manager_create();
    TaskBase t1, t2;
    TaskConfig cfg;
    memset(&cfg, 0, sizeof(cfg));
    snprintf(cfg.name, 64, "t1");
    task_base_init(&t1, &g_test_vtable, &cfg);
    snprintf(cfg.name, 64, "t2");
    task_base_init(&t2, &g_test_vtable, &cfg);

    task_manager_register(mgr, &t1, "t1");
    task_manager_register(mgr, &t2, "t2");
    ASSERT_EQ(task_manager_add_dependency(mgr, "t2", "t1"), 0, "add_dependency failed");

    task_manager_destroy(mgr);
    PASS();
}

/* ══════════════════════════════════════════════════════════ */
/* IPC Channel Tests                                          */
/* ══════════════════════════════════════════════════════════ */

static void test_ipc_open_close(void) {
    TEST("ipc_channel open/close pub");
    IpcChannel* ch = ipc_channel_open("test_ipc_open", IPC_ROLE_PUBLISHER, 8);
    ASSERT(ch != NULL, "ipc pub open failed");
    ipc_channel_close(ch);
    PASS();

    TEST("ipc_channel open/close sub");
    IpcChannel* pub = ipc_channel_open("test_ipc_open2", IPC_ROLE_PUBLISHER, 8);
    ASSERT(pub != NULL, "pub open failed");
    IpcChannel* sub = ipc_channel_open("test_ipc_open2", IPC_ROLE_SUBSCRIBER, 8);
    ASSERT(sub != NULL, "sub open failed (pub must exist first)");
    ipc_channel_close(sub);
    ipc_channel_close(pub);
    PASS();
}

static void test_ipc_pub_sub(void) {
    TEST("ipc pub/sub message delivery");
    IpcChannel* pub = ipc_channel_open("test_ipc_ps", IPC_ROLE_PUBLISHER, 8);
    ASSERT(pub != NULL, "pub open failed");
    IpcChannel* sub = ipc_channel_open("test_ipc_ps", IPC_ROLE_SUBSCRIBER, 8);
    ASSERT(sub != NULL, "sub open failed");

    BusCounter c = { PTHREAD_MUTEX_INITIALIZER, 0, 0, 0 };
    ipc_channel_subscribe(sub, bus_counter_cb, &c);
    ipc_channel_start(sub);

    int v = 88;
    ASSERT_EQ(ipc_channel_publish(pub, "t/ipc", "pub", &v, sizeof(v)), 0, "publish failed");
    usleep(100000);

    ipc_channel_stop(sub);
    ASSERT(c.count == 1, "expected 1 ipc message, got %d", c.count);
    ASSERT(c.max_val == 88, "expected payload 88, got %d", c.max_val);

    ipc_channel_close(sub);
    ipc_channel_close(pub);
    PASS();
}

/* ══════════════════════════════════════════════════════════ */
/* Discovery Tests                                            */
/* ══════════════════════════════════════════════════════════ */

static void test_disc_create_destroy(void) {
    TEST("discovery create/destroy");
    DiscoveryManager* dm = discovery_create("test_node", 0);
    ASSERT(dm != NULL, "discovery create failed");
    if (dm) discovery_destroy(dm);
    PASS();
}

static void test_disc_advertise_topic(void) {
    TEST("discovery advertise topic");
    DiscoveryManager* dm = discovery_create("test_advert", 0);
    ASSERT(dm != NULL, "create failed");

    ASSERT_EQ(discovery_advertise(dm, "sensor/lidar", 0x1234, 0, 10.0), 0, "advertise failed");

    discovery_destroy(dm);
    PASS();
}

static void test_disc_topology(void) {
    TEST("discovery topology is non-NULL");
    DiscoveryManager* dm = discovery_create("test_topo", 0);
    ASSERT(dm != NULL, "create failed");

    const TopologyGraph* topo = discovery_get_topology(dm);
    ASSERT(topo != NULL, "topology should be non-NULL");

    discovery_destroy(dm);
    PASS();
}

/* ══════════════════════════════════════════════════════════ */
/* Bag Tests                                                  */
/* ══════════════════════════════════════════════════════════ */

static void test_bag_write_read(void) {
    TEST("bag write and read");
    const char* path = "/tmp/test_bag_write_read.bag";
    BagWriter* w = bag_writer_open(path);
    ASSERT(w != NULL, "bag_writer_open failed");

    Message msg;
    memset(&msg, 0, sizeof(msg));
    snprintf(msg.topic, 64, "t/bag_test");
    msg.data_size = 4;
    int val = 123;
    memcpy(msg.data, &val, sizeof(val));
    msg.timestamp_us = 1000000;
    ASSERT_EQ(bag_writer_write(w, &msg), 0, "bag_writer_write failed");

    val = 456;
    msg.timestamp_us = 2000000;
    memcpy(msg.data, &val, sizeof(val));
    ASSERT_EQ(bag_writer_write(w, &msg), 0, "bag_writer_write 2 failed");
    bag_writer_close(w);

    BagReader* r = bag_reader_open(path);
    ASSERT(r != NULL, "bag_reader_open failed");

    uint64_t msg_count = 0, duration_us = 0;
    ASSERT_EQ(bag_reader_info(r, &msg_count, &duration_us), 0, "bag_reader_info failed");
    ASSERT(msg_count == 2, "expected 2 messages, got %llu", (unsigned long long)msg_count);

    char topics[8][64];
    int n = bag_reader_get_topics(r, topics, 8, NULL);
    ASSERT(n >= 1, "expected >= 1 topic, got %d", n);

    bag_reader_close(r);
    remove(path);
    PASS();
}

static void test_bag_filtered_play(void) {
    TEST("bag filtered playback");
    const char* path = "/tmp/test_bag_filter.bag";
    BagWriter* w = bag_writer_open(path);
    ASSERT(w != NULL, "open failed");

    Message msg;
    memset(&msg, 0, sizeof(msg));
    snprintf(msg.topic, 64, "t/keep");
    msg.data_size = 4;
    msg.timestamp_us = 1000000;
    bag_writer_write(w, &msg);
    snprintf(msg.topic, 64, "t/skip");
    msg.timestamp_us = 2000000;
    bag_writer_write(w, &msg);
    bag_writer_close(w);

    BagReader* r = bag_reader_open(path);
    ASSERT(r != NULL, "reader open failed");

    int played = bag_reader_play_filtered(r, NULL, 0.0f, "t/keep", 0, 0);
    ASSERT(played == 1, "filtered play expected 1, got %d", played);

    bag_reader_close(r);
    remove(path);
    PASS();
}

/* ══════════════════════════════════════════════════════════ */
/* FlowRegistry Tests                                         */
/* ══════════════════════════════════════════════════════════ */

static void test_freg_register_task(void) {
    TEST("flow_registry register/get task");
    const char* inputs[] = {"sensor/lidar", "sensor/gps", NULL};
    const char* outputs[] = {"perception/obstacles", NULL};
    flow_registry_register_task("perception", "Sensor fusion task", "libperception.so",
        inputs, outputs, NULL);

    const TaskMeta* t = flow_registry_get_task("perception");
    ASSERT(t != NULL, "task should be found");
    ASSERT(strcmp(t->name, "perception") == 0, "name mismatch");
    ASSERT(t->input_count == 2, "expected 2 inputs, got %d", t->input_count);
    ASSERT(t->output_count == 1, "expected 1 output, got %d", t->output_count);

    PASS();
}

static void test_freg_list_tasks(void) {
    TEST("flow_registry list_tasks");
    TaskMeta buf[8];
    int n = flow_registry_list_tasks(buf, 8);
    ASSERT(n >= 1, "expected >= 1 task, got %d", n);
    PASS();
}

static void test_freg_topic(void) {
    TEST("flow_registry register/get topic");
    TopicQos qos;
    memset(&qos, 0, sizeof(qos));
    qos.depth = 16;
    qos.policy = QOS_DROP_LATEST;
    flow_registry_register_topic("sensor/lidar", 0xDEADBEEF, &qos);

    const TopicMeta* t = flow_registry_get_topic("sensor/lidar");
    ASSERT(t != NULL, "topic should be found");
    ASSERT(t->type_id == 0xDEADBEEF, "type_id mismatch");
    ASSERT(t->qos.depth == 16, "qos depth mismatch");

    PASS();
}

static void test_freg_schema(void) {
    TEST("flow_registry register/get schema");
    flow_registry_register_schema("sensor/lidar", 64, "LidarFrame");

    const FlowSchemaEntry* s = flow_registry_get_schema("sensor/lidar");
    ASSERT(s != NULL, "schema should be found");
    ASSERT(s->struct_size == 64, "struct_size mismatch");
    ASSERT(strcmp(s->type_name, "LidarFrame") == 0, "type_name mismatch");

    PASS();
}

static void test_freg_list_schemas(void) {
    TEST("flow_registry list_schemas");
    FlowSchemaEntry buf[16];
    int n = flow_registry_list_schemas(buf, 16);
    ASSERT(n >= 1, "expected >= 1 schema, got %d", n);
    PASS();
}

static void test_freg_plugin(void) {
    TEST("flow_registry register/get plugin");
    const char* tasks[] = {"perception_task", NULL};
    const char* types[] = {"LidarFrame", "ObstacleList", NULL};
    flow_registry_register_plugin("perception", "libperception.so", tasks, types);

    const PluginMeta* p = flow_registry_get_plugin("perception");
    ASSERT(p != NULL, "plugin should be found");
    ASSERT(p->task_count == 1, "expected 1 task, got %d", p->task_count);
    ASSERT(p->type_count == 2, "expected 2 types, got %d", p->type_count);

    PASS();
}

static void test_freg_json_export(void) {
    TEST("flow_registry JSON export");
    char* json = flow_registry_export_json();
    ASSERT(json != NULL, "JSON export should return non-NULL");
    ASSERT(strstr(json, "\"tasks\"") != NULL, "JSON should contain tasks");
    ASSERT(strstr(json, "\"topics\"") != NULL, "JSON should contain topics");
    ASSERT(strstr(json, "\"plugins\"") != NULL, "JSON should contain plugins");
    ASSERT(strstr(json, "\"schemas\"") != NULL, "JSON should contain schemas");
    ASSERT(strstr(json, "\"summary\"") != NULL, "JSON should contain summary");
    free(json);
    PASS();
}

static void test_freg_total_count(void) {
    TEST("flow_registry total_count positive");
    int n = flow_registry_total_count();
    ASSERT(n > 0, "total_count should be positive, got %d", n);
    PASS();
}

/* ══════════════════════════════════════════════════════════ */
/* Main                                                        */
/* ══════════════════════════════════════════════════════════ */

int main(void) {
    printf("\n\xE2\x95\x94\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x97\n");
    printf("\xE2\x95\x91  FlowEngine New Module Tests                    \xE2\x95\x91\n");
    printf("\xE2\x95\x9A\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x9D\n");

    /* ── Message Bus ────────────────────────── */
    printf("\n═══ Message Bus / Basic ═══\n");
    test_bus_pub_sub_basic();
    test_bus_pub_sub_multi();
    test_bus_wildcard_subscribe();
    test_bus_list_topics();
    test_bus_remap();
    test_bus_zero_copy_basic();
    test_bus_req_reply();

    /* ── Task Manager ───────────────────────── */
    printf("\n═══ Task Manager ═══\n");
    test_tmgr_create_destroy();
    test_tmgr_register();
    test_tmgr_list_tasks();
    test_tmgr_health_check();
    test_tmgr_dependency();

    /* ── IPC Channel ────────────────────────── */
    printf("\n═══ IPC Channel ═══\n");
    test_ipc_open_close();
    test_ipc_pub_sub();

    /* ── Discovery ──────────────────────────── */
    printf("\n═══ Discovery ═══\n");
    test_disc_create_destroy();
    test_disc_advertise_topic();
    test_disc_topology();

    /* ── Bag ────────────────────────────────── */
    printf("\n═══ Bag ═══\n");
    test_bag_write_read();
    test_bag_filtered_play();

    /* ── FlowRegistry ───────────────────────── */
    printf("\n═══ FlowRegistry ═══\n");
    test_freg_register_task();
    test_freg_list_tasks();
    test_freg_topic();
    test_freg_schema();
    test_freg_list_schemas();
    test_freg_plugin();
    test_freg_json_export();
    test_freg_total_count();

    /* ── Summary ────────────────────────────── */
    printf("\n╔════════════════════════════════╗\n");
    printf("║  Tests: %3d passed, %3d failed  ║\n", g_passed, g_failed);
    printf("╚════════════════════════════════╝\n");

    return g_failed > 0 ? 1 : 0;
}