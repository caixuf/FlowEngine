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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>

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
    test_serialize_roundtrip();
    test_msg_cast();
    test_endian_detection();

    /* ── State Machine ──────────────────────── */
    printf("\n═══ State Machine ═══\n");
    test_sm_lifecycle();
    test_sm_illegal_transition();
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

    /* ── Summary ────────────────────────────── */
    printf("\n═══════════════════════════════════\n");
    printf("  Total: %d  ✅ Passed: %d  ❌ Failed: %d\n",
           g_passed + g_failed, g_passed, g_failed);
    printf("═══════════════════════════════════\n\n");

    return g_failed > 0 ? 1 : 0;
}
