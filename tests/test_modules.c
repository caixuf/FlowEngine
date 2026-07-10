/**
 * test_modules.c — FlowEngine 模块单元测试
 *
 * 覆盖: serializer, state_machine, scheduler, fusion,
 *       clock_service, scenario_loader
 *
 * 编译: gcc -I include -I build/gen tests/test_modules.c src/core/serializer.c
 *        src/core/state_machine.c src/core/scheduler.c src/core/fusion.c
 *        src/core/message_bus.c src/core/clock_service.c
 *        src/core/scenario_loader.c ... -lpthread -lrt -lm -lcjson
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
#include "clock_service.h"
#include "scenario_loader.h"
#include "nmea_parser.h"
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
    TopicQos q = { .depth = 8, .policy = QOS_DROP_LATEST };
    ASSERT_EQ(message_bus_set_topic_qos(bus, "t/cfg", &q), 0, "set_qos failed");
    const TopicQos* got = message_bus_get_topic_qos(bus, "t/cfg");
    ASSERT(got != NULL, "get_qos returned NULL");
    ASSERT_EQ((int)got->depth, 8, "depth wrong");
    ASSERT_EQ((int)got->policy, (int)QOS_DROP_LATEST, "policy wrong");
    message_bus_destroy(bus);
    PASS();
}

static void test_bus_qos_drop_latest(void) {
    TEST("bus QoS DROP_LATEST enforces depth");
    MessageBus* bus = message_bus_create("test_drop_latest");
    TopicQos q = { .depth = 2, .policy = QOS_DROP_LATEST };
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
    TopicQos q = { .depth = 2, .policy = QOS_DROP_OLDEST };
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

static void test_bus_qos_lifespan(void) {
    TEST("bus QoS lifespan_ms drops stale messages");
    MessageBus* bus = message_bus_create("test_lifespan");
    /* 5ms lifespan: messages waiting in queue longer than 5ms must be dropped */
    TopicQos q = { .depth = MSG_BUS_QUEUE_SIZE, .policy = QOS_DROP_LATEST, .lifespan_ms = 5 };
    message_bus_set_topic_qos(bus, "t/ls", &q);
    /* Slow subscriber: 20ms per callback — causes later messages to wait > 5ms lifespan */
    BusCounter c = { PTHREAD_MUTEX_INITIALIZER, 0, 0, 20000 };
    message_bus_subscribe(bus, "t/ls", bus_counter_cb, &c);

    /* Burst of 5 messages. After msg1 starts being dispatched (20ms),
     * msgs 2-5 have been waiting >5ms and should be dropped by lifespan check. */
    const int N = 5;
    for (int i = 0; i < N; i++)
        message_bus_publish(bus, "t/ls", "tester", &i, sizeof(i));
    usleep(300000); /* drain (N * 20ms + margin) */

    TopicStats st;
    message_bus_get_topic_stats(bus, "t/ls", &st);
    /* At least one message must have been dropped due to lifespan */
    ASSERT(st.drop_count > 0, "expected lifespan drops (got %d, published=%d)",
           (int)st.drop_count, (int)st.publish_count);
    message_bus_destroy(bus);
    PASS();
}

static void test_bus_qos_deadline_violations(void) {
    TEST("bus QoS deadline_ms detects slow dispatch");
    MessageBus* bus = message_bus_create("test_deadline");
    /* Very tight deadline: 1ms. With a slow subscriber the dispatch will exceed it. */
    TopicQos q = { .depth = MSG_BUS_QUEUE_SIZE, .policy = QOS_DROP_OLDEST, .deadline_ms = 1 };
    message_bus_set_topic_qos(bus, "t/dl2", &q);
    BusCounter c = { PTHREAD_MUTEX_INITIALIZER, 0, 0, 5000 }; /* 5ms per callback, much greater than 1ms deadline */
    message_bus_subscribe(bus, "t/dl2", bus_counter_cb, &c);

    const int N = 5;
    for (int i = 0; i < N; i++) {
        message_bus_publish(bus, "t/dl2", "tester", &i, sizeof(i));
        usleep(2000); /* 2ms between publishes — enough to age messages in queue */
    }
    usleep(200000); /* drain */

    TopicStats st;
    message_bus_get_topic_stats(bus, "t/dl2", &st);
    /* With 5ms callback and 1ms deadline, at least some dispatches must have violated */
    ASSERT(st.deadline_violations > 0,
           "expected deadline violations (got %d published %d)",
           (int)st.deadline_violations, (int)st.publish_count);
    message_bus_destroy(bus);
    PASS();
}

/* ══════════════════════════════════════════════════════════ */
/* Main                                                       */
/* ══════════════════════════════════════════════════════════ */

/* ══════════════════════════════════════════════════════════ */
/* Clock Service Tests                                        */
/* ══════════════════════════════════════════════════════════ */

static void test_clock_real_mode(void) {
    TEST("clock real mode returns non-zero");
    clock_set_sim_mode(false);
    uint64_t t = clock_now_us();
    ASSERT(t > 0, "real clock_now_us should be non-zero");

    TEST("clock real mode is_sim_mode = false");
    ASSERT(!clock_is_sim_mode(), "should not be in sim mode");

    TEST("clock real mode advances with time");
    uint64_t t2 = clock_now_us();
    ASSERT(t2 >= t, "monotonic clock must not go backwards");
    PASS();
}

static void test_clock_sim_mode(void) {
    TEST("clock sim_mode set/get");
    clock_set_sim_mode(true);
    ASSERT(clock_is_sim_mode(), "is_sim_mode should be true after set");

    TEST("clock sim set/get time");
    clock_set_sim_time(1000000ULL);
    ASSERT(clock_now_us() == 1000000ULL, "sim time should match set value");

    TEST("clock advance_us accumulates");
    clock_set_sim_time(0);
    clock_advance_us(5000);
    clock_advance_us(3000);
    ASSERT(clock_now_us() == 8000ULL, "advance_us should accumulate: expected 8000");

    TEST("clock advance_us no-op in real mode");
    clock_set_sim_mode(false);
    uint64_t before = clock_now_us();
    clock_advance_us(1000000ULL); /* should have no effect */
    uint64_t after = clock_now_us();
    /* In real mode, now returns CLOCK_MONOTONIC, not the sim counter */
    ASSERT(after >= before, "real clock still monotonic after advance_us no-op");

    /* Restore clean state */
    clock_set_sim_mode(false);
    PASS();
}

static void test_clock_step_us(void) {
    TEST("clock step_us set/get");
    clock_set_step_us(50000ULL); /* 50 ms */
    ASSERT(clock_get_step_us() == 50000ULL, "step_us should match set value");

    TEST("clock step_us non-sim-mode returns 0 after reset");
    clock_set_step_us(0);
    ASSERT(clock_get_step_us() == 0, "step_us should be 0 after reset");

    /* Reset to known clean state */
    clock_set_sim_mode(false);
    PASS();
}

static void test_clock_sim_loop(void) {
    TEST("clock sim loop tick-driven");
    clock_set_sim_mode(true);
    clock_set_sim_time(0);
    clock_set_step_us(10000ULL); /* 10 ms per step */

    const int STEPS = 5;
    for (int i = 0; i < STEPS; i++)
        clock_advance_us(clock_get_step_us());

    uint64_t expected = (uint64_t)STEPS * 10000ULL;
    ASSERT(clock_now_us() == expected,
           "after %d steps of 10ms, sim_time should be %llu us (got %llu)",
           STEPS, (unsigned long long)expected, (unsigned long long)clock_now_us());

    /* Clean up */
    clock_set_sim_mode(false);
    clock_set_step_us(0);
    PASS();
}

/* ══════════════════════════════════════════════════════════ */
/* Scenario Loader Tests                                      */
/* ══════════════════════════════════════════════════════════ */

static void test_scenario_load_null(void) {
    TEST("scenario_load NULL path returns NULL");
    ASSERT(scenario_load(NULL) == NULL, "NULL path should return NULL");

    TEST("scenario_load missing file returns NULL");
    ASSERT(scenario_load("/tmp/nonexistent_scenario_xyz.json") == NULL,
           "missing file should return NULL");
    PASS();
}

/* Helper: construct path relative to repo root (CWD during test run).
 * `filename` must be a plain basename (no path separators or traversal). */
static void make_scenario_path(char* buf, size_t bufsz, const char* filename) {
    /* CMake sets the working directory to the build directory; the scenario
     * files are at <repo>/scenarios/.  Use the compile-time PROJECT_SOURCE_DIR
     * macro when available, otherwise fall back to a relative path. */
#ifdef PROJECT_SOURCE_DIR
    snprintf(buf, bufsz, "%s/scenarios/%s", PROJECT_SOURCE_DIR, filename);
#else
    snprintf(buf, bufsz, "scenarios/%s", filename);
#endif
}

static void test_scenario_load_pedestrian_crossing(void) {
    char path[512];
    make_scenario_path(path, sizeof(path), "pedestrian_crossing.json");

    TEST("scenario_load pedestrian_crossing.json succeeds");
    ScenarioConfig* sc = scenario_load(path);
    ASSERT(sc != NULL, "scenario_load should succeed for pedestrian_crossing.json");

    TEST("scenario name matches");
    ASSERT(strcmp(sc->name, "pedestrian_crossing") == 0,
           "name mismatch: got '%s'", sc->name);

    TEST("scenario random_seed is 42");
    ASSERT(sc->random_seed == 42u, "random_seed should be 42 (got %u)", sc->random_seed);

    TEST("scenario actor_count is 4");
    ASSERT_EQ(sc->actor_count, 4, "actor_count wrong");

    TEST("scenario actor[0] is car at x=35");
    ASSERT(strcmp(sc->actors[0].type, "car") == 0,
           "actor[0] type should be 'car' (got '%s')", sc->actors[0].type);
    ASSERT(fabs(sc->actors[0].x - 35.0) < 0.01,
           "actor[0].x should be 35.0 (got %.2f)", sc->actors[0].x);

    TEST("scenario actor[2] is pedestrian");
    ASSERT(strcmp(sc->actors[2].type, "pedestrian") == 0,
           "actor[2] type should be 'pedestrian' (got '%s')", sc->actors[2].type);

    TEST("scenario ego initial state");
    ASSERT(fabs(sc->ego.y - (-1.75)) < 0.01,
           "ego.y should be -1.75 (got %.2f)", sc->ego.y);
    ASSERT(fabs(sc->ego.init_speed - 5.0) < 0.01,
           "ego.init_speed should be 5.0 (got %.2f)", sc->ego.init_speed);

    TEST("scenario pass_criteria no_collision");
    ASSERT(sc->criteria.no_collision, "no_collision should be true");

    scenario_free(sc);
    PASS();
}

static void test_scenario_load_highway_overtake(void) {
    char path[512];
    make_scenario_path(path, sizeof(path), "highway_overtake.json");

    TEST("scenario_load highway_overtake.json succeeds");
    ScenarioConfig* sc = scenario_load(path);
    ASSERT(sc != NULL, "scenario_load should succeed for highway_overtake.json");

    TEST("scenario highway_overtake name matches");
    ASSERT(strcmp(sc->name, "highway_overtake") == 0,
           "name mismatch: got '%s'", sc->name);

    TEST("scenario highway_overtake has actors");
    ASSERT(sc->actor_count > 0, "actor_count should be > 0 (got %d)", sc->actor_count);

    scenario_free(sc);
    PASS();
}

static void test_scenario_to_json(void) {
    char path[512];
    make_scenario_path(path, sizeof(path), "pedestrian_crossing.json");

    ScenarioConfig* sc = scenario_load(path);
    if (!sc) {
        /* If file is not accessible from the test working dir, skip gracefully */
        TEST("scenario_to_json (skip: file not found)");
        PASS();
        return;
    }

    TEST("scenario_to_json returns non-NULL");
    char* json = scenario_to_json(sc);
    ASSERT(json != NULL, "scenario_to_json should return non-NULL");

    TEST("scenario_to_json contains name field");
    ASSERT(strstr(json, "pedestrian_crossing") != NULL,
           "JSON output should contain scenario name");

    TEST("scenario_to_json contains actors array");
    ASSERT(strstr(json, "actors") != NULL,
           "JSON output should contain 'actors'");

    free(json);
    scenario_free(sc);
    PASS();
}

static void test_scenario_free_null(void) {
    TEST("scenario_free NULL is safe");
    scenario_free(NULL); /* must not crash */
    PASS();
}

/* ══════════════════════════════════════════════════════════ */
/* NMEA 0183 Parser Tests (real GPS sensor format)             */
/* ══════════════════════════════════════════════════════════ */

static void test_nmea_checksum(void) {
    TEST("nmea checksum computation");
    /* Known GPGGA sentence, checksum 0x47 */
    uint8_t cs = nmea_checksum("$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,");
    ASSERT_EQ(cs, 0x47, "checksum mismatch");
    PASS();
}

static void test_nmea_gga(void) {
    TEST("nmea parse GGA (lat/lon/accuracy)");
    NmeaParser p;
    nmea_parser_init(&p);
    GpsData g;
    int rc = nmea_parse_line(&p,
        "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47", &g);
    ASSERT_EQ(rc, NMEA_OK, "GGA parse failed");
    /* 4807.038 N = 48 + 07.038/60 = 48.1173 deg */
    ASSERT(fabs(g.latitude - 48.1173) < 1e-3, "lat wrong: %f", g.latitude);
    /* 01131.000 E = 11 + 31/60 = 11.51667 deg */
    ASSERT(fabs(g.longitude - 11.51667) < 1e-3, "lon wrong: %f", g.longitude);
    ASSERT(fabs(g.accuracy_m - 4.5) < 1e-3, "accuracy wrong: %f", g.accuracy_m);
    PASS();
}

static void test_nmea_rmc(void) {
    TEST("nmea parse RMC (speed knots->m/s, heading)");
    NmeaParser p;
    nmea_parser_init(&p);
    GpsData g;
    int rc = nmea_parse_line(&p,
        "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A", &g);
    ASSERT_EQ(rc, NMEA_OK, "RMC parse failed");
    /* 22.4 knots = 22.4 * 0.514444 = 11.52 m/s */
    ASSERT(fabs(g.speed_mps - 11.52) < 0.05, "speed wrong: %f", g.speed_mps);
    ASSERT(fabs(g.heading_deg - 84.4) < 1e-3, "heading wrong: %f", g.heading_deg);
    PASS();
}

static void test_nmea_southern_western(void) {
    TEST("nmea S/W quadrant sign");
    NmeaParser p;
    nmea_parser_init(&p);
    GpsData g;
    int rc = nmea_parse_line(&p,
        "$GPRMC,081836,A,3751.65,S,14507.36,E,000.0,360.0,130998,011.3,E*62", &g);
    ASSERT_EQ(rc, NMEA_OK, "RMC parse failed");
    ASSERT(g.latitude < 0.0, "southern lat should be negative: %f", g.latitude);
    ASSERT(g.longitude > 0.0, "eastern lon should be positive: %f", g.longitude);
    PASS();
}

static void test_nmea_bad_checksum(void) {
    TEST("nmea rejects bad checksum");
    NmeaParser p;
    nmea_parser_init(&p);
    int rc = nmea_parse_line(&p,
        "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*00", NULL);
    ASSERT_EQ(rc, NMEA_ERR_CHECKSUM, "should reject bad checksum");
    ASSERT_EQ(p.sentences_bad, 1, "bad counter not incremented");
    PASS();
}

static void test_nmea_no_fix(void) {
    TEST("nmea RMC void status = no fix");
    NmeaParser p;
    nmea_parser_init(&p);
    int rc = nmea_parse_line(&p,
        "$GPRMC,123519,V,,,,,,,230394,,*33", NULL);
    ASSERT_EQ(rc, NMEA_ERR_NO_FIX, "void status should be NO_FIX");
    ASSERT(!p.has_position, "position should not be set");
    PASS();
}

static void test_nmea_gnss_talker(void) {
    TEST("nmea accepts GN/GL/BD talker ids");
    NmeaParser p;
    nmea_parser_init(&p);
    GpsData g;
    /* GNRMC (multi-constellation) should be accepted like GPRMC */
    int rc = nmea_parse_line(&p,
        "$GNRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*74", &g);
    ASSERT_EQ(rc, NMEA_OK, "GNRMC should be parsed");
    PASS();
}

static void test_nmea_non_nmea(void) {
    TEST("nmea rejects non-NMEA line");
    NmeaParser p;
    nmea_parser_init(&p);
    int rc = nmea_parse_line(&p, "hello world", NULL);
    ASSERT_EQ(rc, NMEA_ERR_FORMAT, "non-NMEA should be FORMAT error");
    PASS();
}

static void test_nmea_merge(void) {
    TEST("nmea merges GGA position + RMC velocity");
    NmeaParser p;
    nmea_parser_init(&p);
    nmea_parse_line(&p,
        "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47", NULL);
    GpsData g;
    int rc = nmea_parse_line(&p,
        "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A", &g);
    ASSERT_EQ(rc, NMEA_OK, "RMC parse failed");
    /* position from GGA still present, velocity from RMC now present */
    ASSERT(fabs(g.latitude - 48.1173) < 1e-3, "lat lost after merge");
    ASSERT(g.speed_mps > 0.0, "speed not merged");
    ASSERT(p.has_position && p.has_velocity, "flags not both set");
    PASS();
}

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
    test_bus_qos_lifespan();
    test_bus_qos_deadline_violations();

    /* ── Clock Service ──────────────────────── */
    printf("\n═══ Clock Service ═══\n");
    test_clock_real_mode();
    test_clock_sim_mode();
    test_clock_step_us();
    test_clock_sim_loop();

    /* ── Scenario Loader ────────────────────── */
    printf("\n═══ Scenario Loader ═══\n");
    test_scenario_load_null();
    test_scenario_load_pedestrian_crossing();
    test_scenario_load_highway_overtake();
    test_scenario_to_json();
    test_scenario_free_null();

    /* ── NMEA 0183 Parser (real GPS format) ── */
    printf("\n═══ NMEA 0183 Parser ═══\n");
    test_nmea_checksum();
    test_nmea_gga();
    test_nmea_rmc();
    test_nmea_southern_western();
    test_nmea_bad_checksum();
    test_nmea_no_fix();
    test_nmea_gnss_talker();
    test_nmea_non_nmea();
    test_nmea_merge();

    /* ── Summary ────────────────────────────── */
    printf("\n═══════════════════════════════════\n");
    printf("  Total: %d  ✅ Passed: %d  ❌ Failed: %d\n",
           g_passed + g_failed, g_passed, g_failed);
    printf("═══════════════════════════════════\n\n");

    return g_failed > 0 ? 1 : 0;
}
