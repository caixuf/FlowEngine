/**
 * test_bridges.c — 传输/可视化桥接栈单元测试
 *
 * 覆盖 EVOLUTION_ROADMAP 下一阶段优先级 #1（质量收敛）中点名的三个模块：
 *   - transport.c        统一传输抽象（此处测 LOCAL 策略，进程内闭环）
 *   - stats_bridge.c     跨进程 topic 统计 IPC 桥接
 *   - dashboard_bridge.c 跨进程仪表盘 JSON IPC 桥接（含分块/重组协议）
 *
 * stats/dashboard 桥接原本用于跨进程，但同一进程内 publisher + subscriber
 * 打开同名 POSIX 共享内存通道即可端到端验证序列化/重组逻辑，无需 fork。
 *
 * 编译为 C，链接 flowengine_core，保持与核心库一致的 ABI。
 */

#include "transport.h"
#include "stats_bridge.h"
#include "dashboard_bridge.h"
#include "message_bus.h"
#include "ipc_channel.h"
#include "error_codes.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>

/* ── 测试宏（与 test_new_modules.c 保持一致风格）──────────── */

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

/* ══════════════════════════════════════════════════════════ */
/* Transport (LOCAL policy)                                    */
/* ══════════════════════════════════════════════════════════ */

typedef struct {
    pthread_mutex_t m;
    int count;
    int last_val;
} LocalCounter;

static void local_counter_cb(const Message* msg, void* ud) {
    LocalCounter* c = (LocalCounter*)ud;
    int val = 0;
    if (msg->data_size >= sizeof(int)) memcpy(&val, msg->data, sizeof(int));
    pthread_mutex_lock(&c->m);
    c->count++;
    c->last_val = val;
    pthread_mutex_unlock(&c->m);
}

static void test_transport_local_create(void) {
    TEST("transport create/start/stop (LOCAL)");
    MessageBus* bus = message_bus_create("t_local");
    ASSERT(bus != NULL, "bus create failed");

    Transport* t = transport_create(bus, NULL, TRANSPORT_LOCAL);
    ASSERT(t != NULL, "transport create failed");

    ASSERT(transport_start(t) == 0, "transport start failed");
    transport_stop(t);

    transport_destroy(t);
    message_bus_destroy(bus);
    PASS();
}

static void test_transport_local_pubsub(void) {
    TEST("transport LOCAL publish/subscribe delivery");
    MessageBus* bus = message_bus_create("t_ps");
    ASSERT(bus != NULL, "bus create failed");
    Transport* t = transport_create(bus, NULL, TRANSPORT_LOCAL);
    ASSERT(t != NULL, "transport create failed");

    LocalCounter c = { PTHREAD_MUTEX_INITIALIZER, 0, 0 };
    transport_advertise(t, "test/topic", 0);
    ASSERT(transport_subscribe(t, "test/topic", local_counter_cb, &c) == 0,
           "subscribe failed");

    int v = 42;
    ASSERT(transport_publish(t, "test/topic", &v, sizeof(v)) == 0,
           "publish failed");
    usleep(50000);  /* bus delivery is async */

    ASSERT(c.count == 1, "expected 1 delivery, got %d", c.count);
    ASSERT(c.last_val == 42, "expected payload 42, got %d", c.last_val);

    /* Route for a LOCAL-policy transport must be ROUTE_LOCAL */
    ASSERT(transport_route_type(t, "test/topic") == ROUTE_LOCAL,
           "expected ROUTE_LOCAL");

    transport_destroy(t);
    message_bus_destroy(bus);
    PASS();
}

static void test_transport_stats(void) {
    TEST("transport stats count local publishes");
    MessageBus* bus = message_bus_create("t_stats");
    ASSERT(bus != NULL, "bus create failed");
    Transport* t = transport_create(bus, NULL, TRANSPORT_LOCAL);
    ASSERT(t != NULL, "transport create failed");

    LocalCounter c = { PTHREAD_MUTEX_INITIALIZER, 0, 0 };
    transport_subscribe(t, "test/count", local_counter_cb, &c);

    int v = 7;
    for (int i = 0; i < 3; i++) transport_publish(t, "test/count", &v, sizeof(v));
    usleep(50000);  /* bus delivery is async */

    TransportStats st;
    memset(&st, 0, sizeof(st));
    transport_get_stats(t, &st);
    ASSERT(st.local_published >= 3, "expected >=3 local_published, got %llu",
           (unsigned long long)st.local_published);
    ASSERT(c.count == 3, "expected 3 deliveries, got %d", c.count);

    transport_destroy(t);
    message_bus_destroy(bus);
    PASS();
}

static void test_transport_unsubscribe(void) {
    TEST("transport unsubscribe stops delivery");
    MessageBus* bus = message_bus_create("t_unsub");
    ASSERT(bus != NULL, "bus create failed");
    Transport* t = transport_create(bus, NULL, TRANSPORT_LOCAL);
    ASSERT(t != NULL, "transport create failed");

    LocalCounter c = { PTHREAD_MUTEX_INITIALIZER, 0, 0 };
    transport_subscribe(t, "test/unsub", local_counter_cb, &c);

    int v = 1;
    transport_publish(t, "test/unsub", &v, sizeof(v));
    usleep(50000);  /* bus delivery is async */
    ASSERT(c.count == 1, "expected 1 before unsub, got %d", c.count);

    transport_unsubscribe(t, "test/unsub", local_counter_cb);
    transport_publish(t, "test/unsub", &v, sizeof(v));
    usleep(50000);
    ASSERT(c.count == 1, "expected still 1 after unsub, got %d", c.count);

    transport_destroy(t);
    message_bus_destroy(bus);
    PASS();
}

/* ══════════════════════════════════════════════════════════ */
/* stats_bridge                                                */
/* ══════════════════════════════════════════════════════════ */

typedef struct {
    pthread_mutex_t m;
    int received;
    StatsPacket last;
} StatsSink;

static void stats_sink_cb(const Message* msg, void* ud) {
    StatsSink* s = (StatsSink*)ud;
    if (msg->data_size < sizeof(StatsPacket)) return;
    pthread_mutex_lock(&s->m);
    s->received++;
    memcpy(&s->last, msg->data, sizeof(StatsPacket));
    pthread_mutex_unlock(&s->m);
}

static void test_stats_bridge_open(void) {
    TEST("stats_bridge publisher/subscriber open");
    IpcChannel* pub = stats_bridge_publisher_open();
    ASSERT(pub != NULL, "publisher open failed");

    StatsSink s;
    memset(&s, 0, sizeof(s));
    pthread_mutex_init(&s.m, NULL);
    IpcChannel* sub = stats_bridge_subscriber_open(stats_sink_cb, &s);
    ASSERT(sub != NULL, "subscriber open failed (publisher must exist first)");

    ipc_channel_close(sub);
    ipc_channel_close(pub);
    PASS();
}

static void test_stats_bridge_roundtrip(void) {
    TEST("stats_bridge serialize/deliver bus stats");
    IpcChannel* pub = stats_bridge_publisher_open();
    ASSERT(pub != NULL, "publisher open failed");

    StatsSink s;
    memset(&s, 0, sizeof(s));
    pthread_mutex_init(&s.m, NULL);
    IpcChannel* sub = stats_bridge_subscriber_open(stats_sink_cb, &s);
    ASSERT(sub != NULL, "subscriber open failed");
    ipc_channel_start(sub);

    /* Generate some real bus traffic so stats are non-empty */
    MessageBus* bus = message_bus_create("t_statsbus");
    ASSERT(bus != NULL, "bus create failed");
    LocalCounter c = { PTHREAD_MUTEX_INITIALIZER, 0, 0 };
    message_bus_subscribe(bus, "sensor/lidar", local_counter_cb, &c);
    int v = 5;
    for (int i = 0; i < 4; i++)
        message_bus_publish(bus, "sensor/lidar", "test", &v, sizeof(v));

    ASSERT(stats_bridge_publish(pub, bus, "unit_test") == 0,
           "stats_bridge_publish failed");
    usleep(150000);

    ipc_channel_stop(sub);

    pthread_mutex_lock(&s.m);
    int got = s.received;
    StatsPacket pkt = s.last;
    pthread_mutex_unlock(&s.m);

    ASSERT(got == 1, "expected 1 stats packet, got %d", got);
    ASSERT(strcmp(pkt.source_name, "unit_test") == 0,
           "expected source 'unit_test', got '%s'", pkt.source_name);
    ASSERT(pkt.topic_count >= 1, "expected >=1 topic, got %u", pkt.topic_count);

    int found = 0;
    for (uint32_t i = 0; i < pkt.topic_count; i++) {
        if (strcmp(pkt.topics[i].topic, "sensor/lidar") == 0) {
            found = 1;
            ASSERT(pkt.topics[i].publish_count >= 4,
                   "expected >=4 publishes, got %llu",
                   (unsigned long long)pkt.topics[i].publish_count);
        }
    }
    ASSERT(found, "sensor/lidar topic missing from packet");

    message_bus_destroy(bus);
    ipc_channel_close(sub);
    ipc_channel_close(pub);
    PASS();
}

/* ══════════════════════════════════════════════════════════ */
/* dashboard_bridge                                            */
/* ══════════════════════════════════════════════════════════ */

typedef struct {
    pthread_mutex_t m;
    int   received;
    char* last;
    size_t last_len;
} JsonSink;

static void json_sink_cb(const char* json, size_t len, void* ud) {
    JsonSink* s = (JsonSink*)ud;
    pthread_mutex_lock(&s->m);
    s->received++;
    free(s->last);
    s->last = (char*)malloc(len + 1);
    if (s->last) {
        memcpy(s->last, json, len);
        s->last[len] = '\0';
    }
    s->last_len = len;
    pthread_mutex_unlock(&s->m);
}

static void test_dashboard_bridge_single_chunk(void) {
    TEST("dashboard_bridge single-chunk roundtrip");
    IpcChannel* pub = dashboard_bridge_publisher_open();
    ASSERT(pub != NULL, "publisher open failed");

    JsonSink s;
    memset(&s, 0, sizeof(s));
    pthread_mutex_init(&s.m, NULL);
    IpcChannel* sub = dashboard_bridge_subscriber_open(json_sink_cb, &s);
    ASSERT(sub != NULL, "subscriber open failed");
    ipc_channel_start(sub);

    const char* json = "{\"vehicle\":{\"x\":1.5,\"y\":2.0},\"ok\":true}";
    ASSERT(dashboard_bridge_publish(pub, json, strlen(json)) == 0,
           "publish failed");
    usleep(150000);

    ipc_channel_stop(sub);

    pthread_mutex_lock(&s.m);
    int got = s.received;
    int match = (s.last && strcmp(s.last, json) == 0);
    pthread_mutex_unlock(&s.m);

    ASSERT(got == 1, "expected 1 snapshot, got %d", got);
    ASSERT(match, "reassembled JSON mismatch: '%s'", s.last ? s.last : "(null)");

    free(s.last);
    ipc_channel_close(sub);
    ipc_channel_close(pub);
    PASS();
}

static void test_dashboard_bridge_multi_chunk(void) {
    TEST("dashboard_bridge multi-chunk reassembly");
    IpcChannel* pub = dashboard_bridge_publisher_open();
    ASSERT(pub != NULL, "publisher open failed");

    JsonSink s;
    memset(&s, 0, sizeof(s));
    pthread_mutex_init(&s.m, NULL);
    IpcChannel* sub = dashboard_bridge_subscriber_open(json_sink_cb, &s);
    ASSERT(sub != NULL, "subscriber open failed");
    ipc_channel_start(sub);

    /* Build a JSON payload larger than the legacy 4096 stack buffer to
     * exercise the single-chunk path with a large payload.
     * DASHBOARD_CHUNK_DATA_SIZE = MSG_BUS_MAX_DATA_SIZE - 12 = 65524,
     * so 10000 bytes → single chunk, but > 4096 → catches stack truncation. */
    const size_t big_len = 10000;
    char* big = (char*)malloc(big_len + 1);
    ASSERT(big != NULL, "alloc failed");
    for (size_t i = 0; i < big_len; i++) {
        /* printable, non-zero ASCII so the reassembler's length probe works */
        big[i] = (char)('A' + (int)(i % 26));
    }
    big[big_len] = '\0';

    ASSERT(dashboard_bridge_publish(pub, big, big_len) == 0, "publish failed");
    usleep(250000);

    ipc_channel_stop(sub);

    pthread_mutex_lock(&s.m);
    int got = s.received;
    size_t got_len = s.last_len;
    int match = (s.last && s.last_len == big_len && memcmp(s.last, big, big_len) == 0);
    pthread_mutex_unlock(&s.m);

    ASSERT(got == 1, "expected 1 snapshot, got %d", got);
    ASSERT(got_len == big_len, "expected len %zu, got %zu", big_len, got_len);
    ASSERT(match, "reassembled multi-chunk JSON mismatch");

    free(big);
    free(s.last);
    ipc_channel_close(sub);
    ipc_channel_close(pub);
    PASS();
}

/* ══════════════════════════════════════════════════════════ */
/* main                                                        */
/* ══════════════════════════════════════════════════════════ */

int main(void) {
    printf("\n╔════════════════════════════════════════════╗\n");
    printf("║  Bridge / Transport stack unit tests        ║\n");
    printf("╚════════════════════════════════════════════╝\n");

    printf("\n═══ Transport (LOCAL) ═══\n");
    test_transport_local_create();
    test_transport_local_pubsub();
    test_transport_stats();
    test_transport_unsubscribe();

    printf("\n═══ stats_bridge ═══\n");
    test_stats_bridge_open();
    test_stats_bridge_roundtrip();

    printf("\n═══ dashboard_bridge ═══\n");
    test_dashboard_bridge_single_chunk();
    test_dashboard_bridge_multi_chunk();

    printf("\n╔════════════════════════════════╗\n");
    printf("║  Tests: %3d passed, %3d failed  ║\n", g_passed, g_failed);
    printf("╚════════════════════════════════╝\n");

    return g_failed > 0 ? 1 : 0;
}
