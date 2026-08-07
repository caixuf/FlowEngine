/**
 * stress_test.c — FlowEngine 消息总线压测（对标 ROS2/CyberRT 高并发场景）
 *
 * 与 src/benchmark.c 的"单生产者串行微基准"不同，这里压的是真实负载形态：
 *   - 多生产者在同一主题上并发发布（突发持续）
 *   - 多订阅者扇出（1 条消息 → N 个消费者）
 *   - 满载下测持续吞吐、丢包率、端到端 P99 延迟
 *
 * 用法:
 *   stress_test [duration_ms] [producers] [subscribers] [payload_bytes]
 *   默认: stress_test 5000 4 8 64
 */

#include "message_bus.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdatomic.h>
#include <pthread.h>
#include <sched.h>
#include <time.h>
#include <unistd.h>

/* ── 计时 ─────────────────────────────────────────────── */

static uint64_t now_ns(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ULL + (uint64_t)ts.tv_nsec;
}

/* ── 运行参数 ──────────────────────────────────────────── */

static int      opt_duration_ms = 5000;
static int      opt_producers   = 4;
static int      opt_subscribers = 8;
static int      opt_payload     = 64;

static MessageBus* g_bus;

static atomic_uint_fast64_t g_pub_attempts;
static atomic_uint_fast64_t g_pub_ok;
static atomic_uint_fast64_t g_delivered;

/* ── 延迟采样（仅订阅者 0 记录，环形缓冲） ─────────────── */

#define LAT_RING 65536
static uint64_t g_lat_ring[LAT_RING];
static atomic_uint_fast64_t g_lat_pos;
static atomic_uint_fast64_t g_lat_min;
static atomic_uint_fast64_t g_lat_max;

static int cmp_u64(const void* a, const void* b) {
    uint64_t x = *(const uint64_t*)a;
    uint64_t y = *(const uint64_t*)b;
    return (x > y) - (x < y);
}

/* 订阅回调：计数；订阅者 0 额外采样端到端延迟 */
static void on_msg(const Message* m, void* user_data) {
    int sub_idx = (int)(intptr_t)user_data;
    atomic_fetch_add(&g_delivered, 1);

    if (sub_idx == 0) {
        uint64_t now_us = now_ns() / 1000;
        uint64_t lat_us = m->timestamp_us ? (now_us - m->timestamp_us) : 0;

        uint64_t pos = atomic_fetch_add(&g_lat_pos, 1);
        g_lat_ring[pos % LAT_RING] = lat_us;

        uint64_t mn = atomic_load(&g_lat_min);
        uint64_t mx = atomic_load(&g_lat_max);
        if (lat_us < mn) atomic_store(&g_lat_min, lat_us);
        if (lat_us > mx) atomic_store(&g_lat_max, lat_us);
    }
}

/* 生产者：满载持续发布，记录成功/失败 */
typedef struct { int id; } ProducerArg;

static void* producer(void* arg) {
    ProducerArg* pa = (ProducerArg*)arg;
    uint8_t* payload = malloc((size_t)opt_payload);
    if (!payload) return NULL;
    memset(payload, 0xAB, (size_t)opt_payload);

    char topic[32];
    snprintf(topic, sizeof(topic), "stress/prod%d", pa->id);

    uint64_t t_end = now_ns() + (uint64_t)opt_duration_ms * 1000000ULL;
    while (now_ns() < t_end) {
        atomic_fetch_add(&g_pub_attempts, 1);
        if (message_bus_publish(g_bus, topic, "stress",
                                payload, (uint32_t)opt_payload) == 0)
            atomic_fetch_add(&g_pub_ok, 1);
    }
    free(payload);
    return NULL;
}

/* ── 主程序 ───────────────────────────────────────────── */

int main(int argc, char** argv) {
    if (argc > 1) opt_duration_ms = atoi(argv[1]);
    if (argc > 2) opt_producers   = atoi(argv[2]);
    if (argc > 3) opt_subscribers = atoi(argv[3]);
    if (argc > 4) opt_payload     = atoi(argv[4]);

    if (opt_duration_ms <= 0 || opt_payload <= 0 ||
        opt_producers < 1 || opt_subscribers < 1) {
        fprintf(stderr, "用法: %s [duration_ms] [producers] [subscribers] [payload_bytes]\n", argv[0]);
        return 2;
    }

    printf("╔══════════════════════════════════════════════════════════════════╗\n");
    printf("║        FlowEngine 消息总线压测 (对标 ROS2/CyberRT)               ║\n");
    printf("╚══════════════════════════════════════════════════════════════════╝\n");
    printf("  运行时长:   %d ms\n", opt_duration_ms);
    printf("  生产者:     %d (并发突发发布)\n", opt_producers);
    printf("  订阅者:     %d (每主题扇出)\n", opt_subscribers);
    printf("  负载大小:   %d 字节\n\n", opt_payload);

    g_bus = message_bus_create("stress");
    if (!g_bus) { fprintf(stderr, "创建总线失败\n"); return 1; }

    /* 每个 producer 一个独立 topic，避免跨 topic 干扰；每个 topic 挂 S 个订阅者 */
    char topics[16][MSG_BUS_MAX_TOPIC_LEN];
    for (int p = 0; p < opt_producers && p < 16; p++) {
        snprintf(topics[p], sizeof(topics[p]), "stress/prod%d", p);
        for (int s = 0; s < opt_subscribers; s++)
            message_bus_subscribe(g_bus, topics[p], on_msg, (void*)(intptr_t)s);
    }
    if (opt_producers > 16) {
        fprintf(stderr, "警告: 生产者上限 16，已截断\n");
        opt_producers = 16;
    }

    atomic_store(&g_pub_attempts, 0);
    atomic_store(&g_pub_ok, 0);
    atomic_store(&g_delivered, 0);
    atomic_store(&g_lat_pos, 0);
    atomic_store(&g_lat_min, UINT64_MAX);
    atomic_store(&g_lat_max, 0);

    /* 预热：让订阅表就绪 */
    for (int p = 0; p < opt_producers; p++)
        message_bus_publish(g_bus, topics[p], "stress", "warm", 4);
    usleep(20000);

    pthread_t th[16];
    ProducerArg args[16];
    for (int p = 0; p < opt_producers; p++) {
        args[p].id = p;
        pthread_create(&th[p], NULL, producer, &args[p]);
    }

    uint64_t t0 = now_ns();
    for (int p = 0; p < opt_producers; p++) pthread_join(th[p], NULL);
    uint64_t t1 = now_ns();

    /* 排空队列 */
    usleep(50000);

    double elapsed_s = (t1 - t0) / 1e9;
    uint64_t attempts = atomic_load(&g_pub_attempts);
    uint64_t ok       = atomic_load(&g_pub_ok);
    uint64_t deliv    = atomic_load(&g_delivered);

    uint64_t stat_pub=0, stat_del=0, stat_drop=0;
    message_bus_get_stats(g_bus, &stat_pub, &stat_del, &stat_drop);

    /* 延迟分位数 */
    uint64_t n_lat = atomic_load(&g_lat_pos);
    uint64_t lat_min = atomic_load(&g_lat_min);
    uint64_t lat_max = atomic_load(&g_lat_max);
    uint64_t lat_avg = 0, lat_p50 = 0, lat_p99 = 0;
    if (n_lat > 0) {
        uint64_t cnt = n_lat < LAT_RING ? n_lat : LAT_RING;
        uint64_t* sorted = malloc(cnt * sizeof(uint64_t));
        if (sorted) {
            for (uint64_t i = 0; i < cnt; i++)
                sorted[i] = g_lat_ring[i % LAT_RING];
            qsort(sorted, cnt, sizeof(uint64_t), cmp_u64);
            uint64_t sum = 0;
            for (uint64_t i = 0; i < cnt; i++) sum += sorted[i];
            lat_avg = sum / cnt;
            lat_p50 = sorted[cnt / 2];
            lat_p99 = sorted[(uint64_t)((cnt - 1) * 0.99)];
            free(sorted);
        }
    }

    printf("─────────────────────────────────────────────────────────────────────\n");
    printf("  发布尝试:   %12llu 次\n", (unsigned long long)attempts);
    printf("  发布成功:   %12llu 次 (入队)\n", (unsigned long long)ok);
    printf("  投递成功:   %12llu 次 (跨 %d 订阅者扇出)\n",
           (unsigned long long)deliv, opt_subscribers);
    printf("  总线丢弃:   %12llu 次 (队列满)\n", (unsigned long long)stat_drop);
    printf("  持续吞吐(入队):  %10.0f 消息/秒\n", ok / elapsed_s);
    printf("  持续吞吐(投递):  %10.0f 消息/秒\n", deliv / elapsed_s);
    printf("  端到端延迟:  avg=%6.1f  p50=%6.1f  p99=%6.1f  min=%6llu  max=%6llu  (µs)\n",
           (double)lat_avg, (double)lat_p50, (double)lat_p99,
           (unsigned long long)lat_min, (unsigned long long)lat_max);
    printf("─────────────────────────────────────────────────────────────────────\n");
    printf("  丢包率: %.6f%%\n",
           ok > 0 ? (double)stat_drop / (double)ok * 100.0 : 0.0);

    message_bus_destroy(g_bus);
    return 0;
}