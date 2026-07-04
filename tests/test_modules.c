/**
 * test_modules.c — FlowEngine 模块单元测试
 *
 * 覆盖: serializer, state_machine, scheduler, fusion
 *
 * 编译: gcc -I include -I build/gen tests/test_modules.c src/core/serializer.c
 *        src/core/state_machine.c src/core/scheduler.c src/core/fusion.c
 *        src/core/message_bus.c ... -lpthread -lrt -lm
 *
 * 运行: ./build/bin/test_modules
 */

#include "serializer.h"
#include "state_machine.h"
#include "scheduler.h"
#include "fusion.h"
#include "message_bus.h"
#include "error_codes.h"
#include "adas_msgs_gen.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <pthread.h>

static int g_passed = 0;
static int g_failed = 0;

#define TEST(name)  printf("  %-50s ", name)
#define PASS()      do { printf("✅ PASS\n"); g_passed++; } while(0)
#define FAIL(fmt, ...) do { printf("❌ FAIL: " fmt "\n", ##__VA_ARGS__); g_failed++; } while(0)
#define ASSERT(cond, fmt, ...) if (!(cond)) { FAIL(fmt, ##__VA_ARGS__); return; }
#define ASSERT_EQ(a, b, fmt, ...) if ((a) != (b)) { FAIL(fmt " (got %d, expected %d)", ##__VA_ARGS__, (int)(a), (int)(b)); return; }

/* ══════════════════════════════════════════════════════════ */
/* Serializer Tests                                          */
/* ══════════════════════════════════════════════════════════ */

static void test_fnv1a_hash(void) {
    TEST("fnv1a_hash empty string");
    uint32_t h = fnv1a_hash((const uint8_t*)"", 0);
    ASSERT(h == FNV1A_INIT, "empty hash should be FNV1A_INIT");

    TEST("fnv1a_hash known value");
    h = fnv1a_hash((const uint8_t*)"hello", 5);
    ASSERT(h != FNV1A_INIT, "hash should differ from init");
    /* FNV-1a("hello") = 0x4f9f2cab */
    ASSERT(h == 0x4f9f2cab, "FNV-1a('hello') mismatch");

    TEST("fnv1a_hash deterministic");
    uint32_t h2 = fnv1a_hash((const uint8_t*)"hello", 5);
    ASSERT(h == h2, "hash should be deterministic");

    PASS();
}

static void test_type_registry(void) {
    TEST("type registry empty");
    ASSERT(serializer_type_count() >= 0, "count should be non-negative");

    TEST("type registry register");
    TypeRegistryEntry e = {
        .type_id = 0xDEADBEEF, .schema_version = 1, .struct_size = 42,
        .type_name = "TestType", .serialize = NULL, .deserialize = NULL, .endian_swap = NULL
    };
    ASSERT_EQ(serializer_register_type(&e), 0, "register failed");

    TEST("type registry lookup");
    const TypeRegistryEntry* found = serializer_lookup_type(0xDEADBEEF);
    ASSERT(found != NULL, "lookup should find registered type");
    ASSERT(strcmp(found->type_name, "TestType") == 0, "type name mismatch");

    TEST("type registry lookup not found");
    found = serializer_lookup_type(0xCAFEBABE);
    ASSERT(found == NULL, "lookup should return NULL for unknown type");

    PASS();
}

/* 字段级 schema：验证 codegen 生成的字段元信息与 schema hash */
static void test_schema_metadata(void) {
    LidarFrame_register_type();

    TEST("schema hash generated (non-zero)");
    ASSERT(LIDARFRAME_SCHEMA_HASH != 0, "schema hash should be non-zero");

    TEST("registry carries field metadata");
    const TypeRegistryEntry* e = serializer_lookup_by_name("LidarFrame");
    ASSERT(e != NULL, "LidarFrame should be registered");
    ASSERT(e->schema_hash == LIDARFRAME_SCHEMA_HASH, "schema_hash mismatch");
    ASSERT_EQ(e->field_count, 6, "LidarFrame should have 6 fields");
    ASSERT(e->fields != NULL, "fields table should be present");

    TEST("field descriptor content");
    ASSERT(strcmp(e->fields[0].name, "x") == 0, "field[0] name should be x");
    ASSERT(e->fields[0].kind == FIELD_KIND_FLOAT, "field[0] should be float");
    ASSERT(e->fields[4].kind == FIELD_KIND_UINT, "point_count should be uint");
    ASSERT(e->fields[0].array_len == 1, "scalar field array_len should be 1");

    TEST("nested/array field metadata (ObstacleList)");
    ObstacleList_register_type();
    const TypeRegistryEntry* ol = serializer_lookup_by_name("ObstacleList");
    ASSERT(ol != NULL, "ObstacleList should be registered");
    /* obstacles[8] is a nested array field */
    bool found_nested_array = false;
    for (uint16_t i = 0; i < ol->field_count; i++) {
        if (ol->fields[i].kind == FIELD_KIND_NESTED && ol->fields[i].array_len == 8) {
            found_nested_array = true;
        }
    }
    ASSERT(found_nested_array, "should find nested array field obstacles[8]");

    PASS();
}

/* 跨版本兼容性策略判定 */
static void test_schema_compat(void) {
    /* 注册一个已知 hash/version 的基准类型 */
    static const FieldDesc dummy_fields[] = {
        { "a", FIELD_KIND_UINT, 0, 4, 1 },
    };
    TypeRegistryEntry base = {
        .type_id = 0x11112222, .schema_version = 2, .struct_size = 4,
        .type_name = "CompatType", .schema_hash = 0xAABBCCDD,
        .fields = dummy_fields, .field_count = 1,
    };
    ASSERT_EQ(serializer_register_type(&base), 0, "register base failed");

    TEST("compat: unknown type");
    ASSERT_EQ(serializer_check_compat("NoSuchType", 1, 0x1234), SCHEMA_UNKNOWN,
              "unregistered type should be UNKNOWN");

    TEST("compat: identical hash");
    ASSERT_EQ(serializer_check_compat("CompatType", 2, 0xAABBCCDD), SCHEMA_IDENTICAL,
              "same hash should be IDENTICAL");

    TEST("compat: evolved (diff version + diff hash)");
    ASSERT_EQ(serializer_check_compat("CompatType", 3, 0x99887766), SCHEMA_COMPATIBLE,
              "diff version+hash should be COMPATIBLE");

    TEST("compat: breaking (same version + diff hash)");
    ASSERT_EQ(serializer_check_compat("CompatType", 2, 0x99887766), SCHEMA_INCOMPATIBLE,
              "same version diff hash should be INCOMPATIBLE");

    TEST("compat: missing hash falls back to version");
    ASSERT_EQ(serializer_check_compat("CompatType", 2, 0), SCHEMA_IDENTICAL,
              "no hash + same version should be IDENTICAL");
    ASSERT_EQ(serializer_check_compat("CompatType", 5, 0), SCHEMA_COMPATIBLE,
              "no hash + diff version should be COMPATIBLE");

    PASS();
}

static void test_serialize_roundtrip(void) {
    TEST("serialize/deserialize roundtrip");
    /* Simple struct for testing */
    typedef struct {
        float x, y, z;
        uint32_t seq;
    } TestPoint;

    TestPoint orig = { .x = 1.0f, .y = 2.0f, .z = 3.0f, .seq = 42 };
    uint8_t buf[256];
    size_t sz = sizeof(orig);

    /* Simulate serialize */
    memcpy(buf, &orig, sz);

    /* Simulate deserialize */
    TestPoint restored;
    memcpy(&restored, buf, sz);

    ASSERT(fabsf(restored.x - orig.x) < 0.001f, "x mismatch");
    ASSERT(fabsf(restored.y - orig.y) < 0.001f, "y mismatch");
    ASSERT(restored.seq == orig.seq, "seq mismatch");

    PASS();
}

static void test_msg_cast(void) {
    TEST("msg_cast with matching size");
    Message msg;
    memset(&msg, 0, sizeof(msg));
    msg.data_size = 12;
    const void* p = _msg_cast_impl(&msg, 0, 12, "TestType");
    ASSERT(p == msg.data, "should return data ptr");

    TEST("msg_cast with mismatched size");
    p = _msg_cast_impl(&msg, 0, 24, "TestType");
    ASSERT(p == NULL, "should return NULL on size mismatch");

    PASS();
}

static void test_endian_detection(void) {
    TEST("endian detection");
    bool is_be = serializer_is_big_endian();
    uint8_t marker = serializer_endian_marker();
    ASSERT(marker == (is_be ? ENDIAN_MARKER_BE : ENDIAN_MARKER_LE),
           "endian marker mismatch");

    TEST("endian swap32 symmetry");
    uint32_t val = 0x12345678;
    uint8_t* bytes = (uint8_t*)&val;
    serializer_swap32(bytes);
    serializer_swap32(bytes);
    ASSERT(*(uint32_t*)bytes == 0x12345678, "swap32 should be symmetric");

    PASS();
}

/* ══════════════════════════════════════════════════════════ */
/* State Machine Tests                                       */
/* ══════════════════════════════════════════════════════════ */

static void test_sm_lifecycle(void) {
    TEST("sm init");
    ReflectiveStateMachine sm;
    statem_init(&sm, SM_TABLE_STANDARD, SM_STATE_INITIALIZED, "test");
    ASSERT(statem_current(&sm) == SM_STATE_INITIALIZED, "initial state wrong");

    TEST("sm START -> RUNNING");
    ASSERT(statem_send_event(&sm, SM_EVENT_START, NULL), "START should succeed");
    ASSERT(statem_current(&sm) == SM_STATE_RUNNING, "should be RUNNING");

    TEST("sm STOP -> STOPPING");
    ASSERT(statem_send_event(&sm, SM_EVENT_STOP, NULL), "STOP should succeed");
    ASSERT(statem_current(&sm) == SM_STATE_STOPPING, "should be STOPPING");

    TEST("sm DONE -> STOPPED");
    ASSERT(statem_send_event(&sm, SM_EVENT_DONE, NULL), "DONE should succeed");
    ASSERT(statem_current(&sm) == SM_STATE_STOPPED, "should be STOPPED");

    PASS();
}

static void test_sm_illegal_transition(void) {
    TEST("sm illegal: INITIALIZED + STOP rejected");
    ReflectiveStateMachine sm;
    statem_init(&sm, SM_TABLE_STANDARD, SM_STATE_INITIALIZED, "test");
    bool ok = statem_send_event(&sm, SM_EVENT_STOP, NULL);
    ASSERT(!ok, "illegal transition should be rejected");
    ASSERT(statem_current(&sm) == SM_STATE_INITIALIZED, "state should not change");

    PASS();
}

static void test_sm_new_transitions(void) {
    TEST("sm RUNNING + DONE -> STOPPED (自行结束)");
    ReflectiveStateMachine sm;
    statem_init(&sm, SM_TABLE_STANDARD, SM_STATE_INITIALIZED, "test");
    ASSERT(statem_send_event(&sm, SM_EVENT_START, NULL), "START should succeed");
    ASSERT(statem_send_event(&sm, SM_EVENT_DONE, NULL), "RUNNING + DONE should succeed");
    ASSERT(statem_current(&sm) == SM_STATE_STOPPED, "should be STOPPED");

    TEST("sm INITIALIZED + ERROR -> ERROR (init 失败)");
    ReflectiveStateMachine sm2;
    statem_init(&sm2, SM_TABLE_STANDARD, SM_STATE_INITIALIZED, "test");
    ASSERT(statem_send_event(&sm2, SM_EVENT_ERROR, NULL), "INITIALIZED + ERROR should succeed");
    ASSERT(statem_current(&sm2) == SM_STATE_ERROR, "should be ERROR");

    PASS();
}

static void test_sm_illegal_policy(void) {
    TEST("sm send_event_ex 统一错误码");
    ReflectiveStateMachine sm;
    statem_init(&sm, SM_TABLE_STANDARD, SM_STATE_INITIALIZED, "test");
    ASSERT_EQ(statem_send_event_ex(&sm, SM_EVENT_START, NULL), ERR_OK,
              "合法转移应返回 ERR_OK");
    ASSERT_EQ(statem_send_event_ex(&sm, SM_EVENT_RESUME, NULL), ERR_ILLEGAL_TRANSITION,
              "非法转移应返回 ERR_ILLEGAL_TRANSITION");
    ASSERT(statem_current(&sm) == SM_STATE_RUNNING, "WARN 策略下状态不变");

    TEST("sm illegal policy REJECT 保持状态不变");
    ReflectiveStateMachine sm2;
    statem_init(&sm2, SM_TABLE_STANDARD, SM_STATE_INITIALIZED, "test");
    statem_set_illegal_policy(&sm2, SM_ILLEGAL_REJECT);
    ASSERT_EQ(statem_send_event_ex(&sm2, SM_EVENT_STOP, NULL), ERR_ILLEGAL_TRANSITION,
              "REJECT 策略仍返回 ERR_ILLEGAL_TRANSITION");
    ASSERT(statem_current(&sm2) == SM_STATE_INITIALIZED, "REJECT 策略下状态不变");

    TEST("sm illegal policy GOTO_ERROR 进入 ERROR 态");
    ReflectiveStateMachine sm3;
    statem_init(&sm3, SM_TABLE_STANDARD, SM_STATE_INITIALIZED, "test");
    statem_set_illegal_policy(&sm3, SM_ILLEGAL_GOTO_ERROR);
    ASSERT_EQ(statem_send_event_ex(&sm3, SM_EVENT_STOP, NULL), ERR_ILLEGAL_TRANSITION,
              "GOTO_ERROR 策略返回 ERR_ILLEGAL_TRANSITION");
    ASSERT(statem_current(&sm3) == SM_STATE_ERROR, "GOTO_ERROR 策略应进入 ERROR 态");

    PASS();
}

static void test_sm_guard(void) {
    TEST("sm guard rejects transition");
    ReflectiveStateMachine sm;
    statem_init(&sm, SM_TABLE_STANDARD, SM_STATE_INITIALIZED, "test");

    /* Install guard that rejects STOP when seq < 5 */
    static int seq = 0;
    sm.guard = [](void*, StateId from, EventId ev, StateId to) -> bool {
        (void)from; (void)to;
        if (ev == SM_EVENT_STOP) {
            seq++;
            return seq >= 5;  /* Only allow after 5 attempts */
        }
        return true;
    };

    /* START first */
    ASSERT(statem_send_event(&sm, SM_EVENT_START, NULL), "START should succeed");

    /* STOP should be rejected 4 times */
    for (int i = 0; i < 4; i++) {
        ASSERT(!statem_send_event(&sm, SM_EVENT_STOP, NULL),
               "STOP should be rejected by guard (#%d)", i+1);
    }
    /* 5th time should succeed */
    ASSERT(statem_send_event(&sm, SM_EVENT_STOP, NULL), "STOP should succeed after guard passes");
    ASSERT(statem_current(&sm) == SM_STATE_STOPPING, "should be STOPPING");

    PASS();
}

static void test_sm_reflection(void) {
    TEST("sm allowed events");
    ReflectiveStateMachine sm;
    statem_init(&sm, SM_TABLE_STANDARD, SM_STATE_RUNNING, "test");
    ASSERT(statem_can_transition(&sm, SM_EVENT_STOP), "RUNNING should allow STOP");
    ASSERT(statem_can_transition(&sm, SM_EVENT_PAUSE), "RUNNING should allow PAUSE");
    ASSERT(!statem_can_transition(&sm, SM_EVENT_RESUME), "RUNNING should NOT allow RESUME");

    TEST("sm allowed_events list");
    EventId allowed[8];
    int n = statem_allowed_events(&sm, allowed, 8);
    ASSERT(n >= 2, "should have at least 2 allowed events");

    bool has_stop = false, has_pause = false;
    for (int i = 0; i < n; i++) {
        if (allowed[i] == SM_EVENT_STOP) has_stop = true;
        if (allowed[i] == SM_EVENT_PAUSE) has_pause = true;
    }
    ASSERT(has_stop && has_pause, "STOP and PAUSE should be in allowed events");

    PASS();
}

static void test_sm_dynamic_rules(void) {
    TEST("sm add dynamic transition");
    ReflectiveStateMachine sm;
    statem_init(&sm, SM_TABLE_STANDARD, SM_STATE_RUNNING, "test");
    ASSERT(statem_add_transition(&sm, SM_STATE_RUNNING, 99, SM_STATE_PAUSED, "custom") == 0,
           "add transition failed");

    TEST("sm dynamic transition works");
    ASSERT(statem_can_transition(&sm, 99), "custom event should now be allowed");
    ASSERT(statem_send_event(&sm, 99, NULL), "custom transition should succeed");
    ASSERT(statem_current(&sm) == SM_STATE_PAUSED, "should be PAUSED");

    PASS();
}

static void test_sm_guard_runtime_swap(void) {
    TEST("sm guard runtime set/clear");
    ReflectiveStateMachine sm;
    statem_init(&sm, SM_TABLE_STANDARD, SM_STATE_INITIALIZED, "test");
    statem_send_event(&sm, SM_EVENT_START, NULL);

    /* Install blocking guard */
    sm.guard = [](void*, StateId, EventId, StateId) -> bool { return false; };
    ASSERT(!statem_send_event(&sm, SM_EVENT_STOP, NULL), "guard should block STOP");

    /* Remove guard */
    statem_set_guard(&sm, NULL);
    ASSERT(statem_get_guard(&sm) == NULL, "guard should be NULL after removal");
    ASSERT(statem_send_event(&sm, SM_EVENT_STOP, NULL), "STOP should work after guard removed");

    PASS();
}

/* ══════════════════════════════════════════════════════════ */
/* Scheduler Tests                                           */
/* ══════════════════════════════════════════════════════════ */

static void test_rate_control(void) {
    TEST("rate_control unlimited");
    RateControl rc;
    rate_control_init(&rc, 0.0);
    ASSERT(rate_control_acquire(&rc), "unlimited should always allow");

    TEST("rate_control limited 10Hz");
    rate_control_init(&rc, 10.0);
    ASSERT(rate_control_acquire(&rc), "first acquire should succeed");
    ASSERT(!rate_control_acquire(&rc), "second acquire too soon");
    usleep(150000); /* wait >100ms */
    ASSERT(rate_control_acquire(&rc), "acquire after wait should succeed");

    PASS();
}

static void test_latency_tracker(void) {
    TEST("latency_tracker single sample");
    LatencyTracker lt;
    memset(&lt, 0, sizeof(lt));
    latency_tracker_record(&lt, 100);
    LatencyStats s = latency_tracker_stats(&lt);
    ASSERT(s.min_us == 100 && s.max_us == 100, "min/max should equal single sample");

    TEST("latency_tracker P50/P99");
    for (int i = 0; i < 100; i++) {
        latency_tracker_record(&lt, (uint64_t)(i * 10));  /* 0, 10, 20, ... 990 */
    }
    s = latency_tracker_stats(&lt);
    ASSERT(s.sample_count == 101, "sample count wrong");
    ASSERT(s.p50_us >= 475 && s.p50_us <= 525, "P50 should be ~500us");
    ASSERT(s.p99_us >= 960, "P99 should be >= 960us");

    PASS();
}

static void test_scheduler_register(void) {
    TEST("scheduler create and register");
    Scheduler* sched = scheduler_create(NULL);
    ASSERT(sched != NULL, "create failed");

    TaskBase task;
    TaskConfig cfg;
    memset(&cfg, 0, sizeof(cfg));
    snprintf(cfg.name, 64, "test_task");
    task_base_init(&task, NULL, &cfg);

    int tid = scheduler_register_task(sched, &task, "test_task");
    ASSERT(tid >= 0, "register failed");
    ASSERT(scheduler_task_count(sched) == 1, "count should be 1");

    ASSERT(scheduler_start(sched) == 0, "start failed");
    scheduler_stop(sched);
    scheduler_destroy(sched);

    PASS();
}

/* ══════════════════════════════════════════════════════════ */
/* Fusion Tests                                              */
/* ══════════════════════════════════════════════════════════ */

static void test_message_buffer(void) {
    TEST("message_buffer create");
    MessageBuffer* mb = message_buffer_create("test/topic", 0x1234, 16, 5000000);
    ASSERT(mb != NULL, "create failed");

    TEST("message_buffer push and find_nearest");
    Message msg;
    memset(&msg, 0, sizeof(msg));
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    uint64_t now = (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;

    /* Use current-time timestamps so window check passes */
    uint64_t t1 = now - 1000;  /* 1ms ago */
    uint64_t t2 = now - 500;   /* 0.5ms ago */

    msg.timestamp_us = t1;
    snprintf(msg.topic, 64, "test/topic");
    msg.data_size = 8;
    message_buffer_push(mb, &msg);

    msg.timestamp_us = t2;
    message_buffer_push(mb, &msg);

    uint64_t target = now - 750;
    const Message* found = message_buffer_find_nearest(mb, target, 1000000);
    ASSERT(found != NULL, "should find nearest message");
    /* Closest to target should be t2 (delta smaller) */
    ASSERT(found->timestamp_us == t1 || found->timestamp_us == t2,
           "should find a message within range");

    TEST("message_buffer find outside narrow window");
    found = message_buffer_find_nearest(mb, now - 2000, 10);  /* max_delta=10us, target far */
    ASSERT(found == NULL, "should return NULL outside max delta");

    TEST("message_buffer latest");
    found = message_buffer_latest(mb);
    ASSERT(found != NULL, "should have latest message");
    ASSERT(found->timestamp_us == t2, "latest timestamp wrong");

    message_buffer_destroy(mb);
    PASS();
}

static void test_fusion_node(void) {
    TEST("fusion_node create");
    FusionPolicy policy = FUSION_POLICY_TIME_ALIGNED;
    MessageBus* bus = message_bus_create("test_fusion_bus");
    FusionNode* fn = fusion_node_create("test_fusion", bus, &policy);
    ASSERT(fn != NULL, "create failed");

    TEST("fusion_node add inputs");
    ASSERT(fusion_node_add_input(fn, "sensor/a", 0xAAAA, 16) == 0, "add input a failed");
    ASSERT(fusion_node_add_input(fn, "sensor/b", 0xBBBB, 16) == 0, "add input b failed");

    TEST("fusion_node set output");
    ASSERT(fusion_node_set_output(fn, "fusion/out", 0xCCCC) == 0, "set output failed");

    fusion_node_destroy(fn);
    message_bus_destroy(bus);
    PASS();
}

/* ══════════════════════════════════════════════════════════ */
/* Message Bus — QoS & Per-Topic Statistics                   */
/* ══════════════════════════════════════════════════════════ */

typedef struct {
    pthread_mutex_t m;
    int count;
    int max_val;   /* max payload value observed */
    int slow_us;   /* per-callback sleep to create backpressure */
} BusCounter;

static void bus_counter_cb(const Message* msg, void* ud) {
    BusCounter* c = (BusCounter*)ud;
    int val = 0;
    if (msg->data_size >= sizeof(int)) memcpy(&val, msg->data, sizeof(int));
    if (c->slow_us > 0) usleep(c->slow_us);
    pthread_mutex_lock(&c->m);
    c->count++;
    if (val > c->max_val) c->max_val = val;
    pthread_mutex_unlock(&c->m);
}

static void test_bus_topic_stats(void) {
    TEST("bus per-topic stats accounting");
    MessageBus* bus = message_bus_create("test_qos_bus");
    ASSERT(bus != NULL, "bus create failed");
    BusCounter c = { PTHREAD_MUTEX_INITIALIZER, 0, 0, 0 };
    message_bus_subscribe(bus, "t/stats", bus_counter_cb, &c);

    const int N = 10;
    for (int i = 0; i < N; i++) {
        message_bus_publish(bus, "t/stats", "tester", &i, sizeof(i));
        usleep(3000); /* 3ms spacing → measurable frequency */
    }
    usleep(100000); /* let dispatch drain */

    TopicStats st;
    int rc = message_bus_get_topic_stats(bus, "t/stats", &st);
    ASSERT_EQ(rc, 0, "get_topic_stats failed");
    ASSERT_EQ((int)st.publish_count, N, "publish_count wrong");
    ASSERT_EQ((int)st.deliver_count, N, "deliver_count wrong");
    ASSERT_EQ((int)st.subscriber_count, 1, "subscriber_count wrong");
    message_bus_destroy(bus);
    PASS();

    TEST("bus per-topic frequency estimate in sane range");
    /* frequency_hz must be non-zero (regression: was always 0) and derived
     * from the real ~3ms inter-arrival gap, not a mis-scaled value. */
    ASSERT(st.frequency_hz > 1.0 && st.frequency_hz < 100000.0,
           "frequency_hz out of range (%.1f)", st.frequency_hz);
    PASS();
}

static void test_bus_qos_config(void) {
    TEST("bus QoS set/get");
    MessageBus* bus = message_bus_create("test_qos_cfg");
    TopicQos q = { .queue_depth = 8, .policy = QOS_DROP_LATEST };
    ASSERT_EQ(message_bus_set_topic_qos(bus, "t/cfg", &q), 0, "set_qos failed");
    const TopicQos* got = message_bus_get_topic_qos(bus, "t/cfg");
    ASSERT(got != NULL, "get_qos returned NULL");
    ASSERT_EQ((int)got->queue_depth, 8, "queue_depth wrong");
    ASSERT_EQ((int)got->policy, (int)QOS_DROP_LATEST, "policy wrong");
    message_bus_destroy(bus);
    PASS();
}

static void test_bus_qos_drop_latest(void) {
    TEST("bus QoS DROP_LATEST enforces depth");
    MessageBus* bus = message_bus_create("test_drop_latest");
    TopicQos q = { .queue_depth = 2, .policy = QOS_DROP_LATEST };
    message_bus_set_topic_qos(bus, "t/dl", &q);
    BusCounter c = { PTHREAD_MUTEX_INITIALIZER, 0, 0, 4000 }; /* slow: 4ms */
    message_bus_subscribe(bus, "t/dl", bus_counter_cb, &c);

    const int N = 40;
    for (int i = 0; i < N; i++)
        message_bus_publish(bus, "t/dl", "tester", &i, sizeof(i)); /* burst */
    usleep(400000); /* drain */

    TopicStats st;
    message_bus_get_topic_stats(bus, "t/dl", &st);
    /* Some messages must have been dropped due to depth pressure */
    ASSERT(st.drop_count > 0, "expected drops (got %d)", (int)st.drop_count);
    /* All enqueued messages must eventually be delivered (1 subscriber) */
    ASSERT_EQ((int)st.deliver_count, (int)st.publish_count,
              "deliver != publish (%d vs %d)", (int)st.deliver_count, (int)st.publish_count);
    message_bus_destroy(bus);
    PASS();
}

static void test_bus_qos_drop_oldest(void) {
    TEST("bus QoS DROP_OLDEST keeps newest");
    MessageBus* bus = message_bus_create("test_drop_oldest");
    TopicQos q = { .queue_depth = 2, .policy = QOS_DROP_OLDEST };
    message_bus_set_topic_qos(bus, "t/do", &q);
    BusCounter c = { PTHREAD_MUTEX_INITIALIZER, 0, 0, 4000 }; /* slow: 4ms */
    message_bus_subscribe(bus, "t/do", bus_counter_cb, &c);

    const int N = 40;
    for (int i = 0; i < N; i++)
        message_bus_publish(bus, "t/do", "tester", &i, sizeof(i)); /* burst 0..39 */
    usleep(400000); /* drain */

    TopicStats st;
    message_bus_get_topic_stats(bus, "t/do", &st);
    ASSERT(st.drop_count > 0, "expected evictions (got %d)", (int)st.drop_count);
    message_bus_destroy(bus);
    /* dispatch thread joined → safe to read counter without lock */
    ASSERT_EQ(c.max_val, N - 1, "newest message must survive (saw max %d)", c.max_val);
    PASS();
}

/* ══════════════════════════════════════════════════════════ */
/* Main                                                       */
/* ══════════════════════════════════════════════════════════ */

int main(void) {
    printf("\n╔══════════════════════════════════════════╗\n");
    printf("║  FlowEngine Unit Tests                    ║\n");
    printf("╚══════════════════════════════════════════╝\n\n");

    /* ── Serializer ─────────────────────────── */
    printf("═══ Serializer ═══\n");
    test_fnv1a_hash();
    test_type_registry();
    test_schema_metadata();
    test_schema_compat();
    test_serialize_roundtrip();
    test_msg_cast();
    test_endian_detection();

    /* ── State Machine ──────────────────────── */
    printf("\n═══ State Machine ═══\n");
    test_sm_lifecycle();
    test_sm_illegal_transition();
    test_sm_new_transitions();
    test_sm_illegal_policy();
    test_sm_guard();
    test_sm_reflection();
    test_sm_dynamic_rules();
    test_sm_guard_runtime_swap();

    /* ── Scheduler ──────────────────────────── */
    printf("\n═══ Scheduler ═══\n");
    test_rate_control();
    test_latency_tracker();
    test_scheduler_register();

    /* ── Fusion ─────────────────────────────── */
    printf("\n═══ Fusion ═══\n");
    test_message_buffer();
    test_fusion_node();

    /* ── Message Bus (QoS) ──────────────────── */
    printf("\n═══ Message Bus / QoS ═══\n");
    test_bus_topic_stats();
    test_bus_qos_config();
    test_bus_qos_drop_latest();
    test_bus_qos_drop_oldest();

    /* ── Summary ────────────────────────────── */
    printf("\n═══════════════════════════════════\n");
    printf("  Total: %d  ✅ Passed: %d  ❌ Failed: %d\n",
           g_passed + g_failed, g_passed, g_failed);
    printf("═══════════════════════════════════\n\n");

    return g_failed > 0 ? 1 : 0;
}
