/**
 * benchmark.c — FlowEngine 核心组件性能基准测试
 *
 * 测试项目:
 *  1. 消息发布 API 调用速率（Raw Publish Rate）
 *  2. 持续端到端吞吐量（批次发布 + 等待交付）
 *  3. 普通路径端到端发布延迟（Pub → 分发线程 → Callback）
 *  4. 零拷贝 vs 普通路径延迟对比
 *  5. Req/Reply 往返延迟（RTT）
 */

#include "message_bus.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdatomic.h>
#include <semaphore.h>
#include <sched.h>
#include <time.h>
#include <pthread.h>
#include <unistd.h>

/* ── 计时工具 ─────────────────────────────────────────────── */

static uint64_t now_ns(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ULL + (uint64_t)ts.tv_nsec;
}

/* ── 统计计算 ─────────────────────────────────────────────── */

static int cmp_u64(const void* a, const void* b) {
    uint64_t x = *(const uint64_t*)a;
    uint64_t y = *(const uint64_t*)b;
    return (x > y) - (x < y);
}

typedef struct {
    uint64_t min_ns;
    uint64_t max_ns;
    uint64_t avg_ns;
    uint64_t p50_ns;
    uint64_t p99_ns;
} Stats;

static Stats calc_stats(uint64_t* samples, int n) {
    uint64_t* sorted = malloc((size_t)n * sizeof(uint64_t));
    if (!sorted) {
        fprintf(stderr, "calc_stats: malloc failed, results will be zero\n");
        Stats empty = {0};
        return empty;
    }
    memcpy(sorted, samples, (size_t)n * sizeof(uint64_t));
    qsort(sorted, (size_t)n, sizeof(uint64_t), cmp_u64);

    uint64_t sum = 0;
    for (int i = 0; i < n; i++) sum += sorted[i];

    Stats s = {
        .min_ns = sorted[0],
        .max_ns = sorted[n - 1],
        .avg_ns = sum / (uint64_t)n,
        .p50_ns = sorted[n / 2],
        .p99_ns = sorted[(int)((n - 1) * 0.99)],
    };
    free(sorted);
    return s;
}

/* ── 输出格式 ─────────────────────────────────────────────── */

static void print_sep(void) {
    printf("─────────────────────────────────────────────────────────────────────\n");
}

static void print_bench_header(const char* title) {
    printf("\n");
    print_sep();
    printf("  %s\n", title);
    print_sep();
}

static void print_latency_row(const char* label, Stats s) {
    printf("  %-38s min=%6.1f  avg=%6.1f  p50=%6.1f  p99=%6.1f  max=%6.1f  (µs)\n",
           label,
           s.min_ns / 1e3, s.avg_ns / 1e3,
           s.p50_ns / 1e3, s.p99_ns / 1e3, s.max_ns / 1e3);
}

/* ══════════════════════════════════════════════════════════
 * Benchmark 1: Raw Publish API Rate
 * 测量 message_bus_publish() 的调用开销（不含分发等待）
 * ══════════════════════════════════════════════════════════ */

static void bench_raw_publish_rate(void) {
    print_bench_header("Benchmark 1: 消息发布 API 调用速率 (Raw Publish Rate)");

    MessageBus* bus = message_bus_create("bench1");

    const int N = 500000;
    uint8_t payload[64];
    memset(payload, 0xAB, sizeof(payload));

    /* 预热 */
    for (int i = 0; i < 256; i++)
        message_bus_publish(bus, "bench/raw", "bench", payload, sizeof(payload));
    usleep(50000); /* 等待队列排空 */

    uint64_t t0 = now_ns();
    for (int i = 0; i < N; i++)
        message_bus_publish(bus, "bench/raw", "bench", payload, sizeof(payload));
    uint64_t t1 = now_ns();

    double elapsed_s  = (t1 - t0) / 1e9;
    double rate        = N / elapsed_s;
    double per_call_ns = (t1 - t0) / (double)N;

    printf("  发布调用次数:  %d\n", N);
    printf("  总耗时:        %.3f ms\n", elapsed_s * 1000.0);
    printf("  API 调用速率:  %.0f 次/秒\n", rate);
    printf("  单次调用耗时:  %.1f ns\n", per_call_ns);
    printf("  (注: 队列大小=%d，队列满后消息被丢弃，不影响 API 耗时测量)\n",
           MSG_BUS_QUEUE_SIZE);

    message_bus_destroy(bus);
}

/* ══════════════════════════════════════════════════════════
 * Benchmark 2: Sustained End-to-End Throughput
 * 批次发布 + 等待全部交付，测量实际持续吞吐量
 * ══════════════════════════════════════════════════════════ */

static atomic_uint_fast64_t g_thru_count;

static void thru_cb(const Message* m, void* u) {
    (void)m; (void)u;
    atomic_fetch_add(&g_thru_count, 1);
}

static void bench_sustained_throughput(void) {
    print_bench_header("Benchmark 2: 持续端到端吞吐量 (Sustained Throughput)");

    MessageBus* bus = message_bus_create("bench2");
    message_bus_subscribe(bus, "bench/thru", thru_cb, NULL);

    /* 每批 BATCH 条（远小于队列 256），共 ROUNDS 批 */
    const int BATCH  = 100;
    const int ROUNDS = 1000;
    const int TOTAL  = BATCH * ROUNDS;

    uint8_t payload[64];
    memset(payload, 0, sizeof(payload));
    atomic_store(&g_thru_count, 0);

    uint64_t t0 = now_ns();
    for (int r = 0; r < ROUNDS; r++) {
        uint64_t target = (uint64_t)(r + 1) * BATCH;
        for (int i = 0; i < BATCH; i++)
            message_bus_publish(bus, "bench/thru", "bench", payload, sizeof(payload));
        /* 等待这批全部被分发处理 */
        while (atomic_load(&g_thru_count) < target)
            sched_yield();
    }
    uint64_t t1 = now_ns();

    double elapsed_s  = (t1 - t0) / 1e9;
    double throughput  = TOTAL / elapsed_s;

    printf("  总消息数:       %d\n", TOTAL);
    printf("  总耗时:         %.3f ms\n", elapsed_s * 1000.0);
    printf("  端到端吞吐量:   %.0f 消息/秒\n", throughput);
    printf("  平均端到端延迟: %.1f µs/消息\n", elapsed_s * 1e6 / TOTAL);

    message_bus_destroy(bus);
}

/* ══════════════════════════════════════════════════════════
 * Benchmark 3: Publish-to-Callback End-to-End Latency
 * 每次发布一条消息，等待回调触发后再发下一条，精确测量延迟分布
 * ══════════════════════════════════════════════════════════ */

static sem_t    g_lat_sem;
static uint64_t g_lat_recv_ns;

static void lat_cb(const Message* m, void* u) {
    (void)m; (void)u;
    g_lat_recv_ns = now_ns();
    sem_post(&g_lat_sem);
}

static void bench_publish_latency(void) {
    print_bench_header("Benchmark 3: 普通路径端到端延迟 (Pub → Dispatch Thread → Callback)");

    MessageBus* bus = message_bus_create("bench3");
    sem_init(&g_lat_sem, 0, 0);
    message_bus_subscribe(bus, "bench/lat", lat_cb, NULL);

    const int N = 5000;
    uint8_t payload[64];
    memset(payload, 0, sizeof(payload));
    uint64_t* samples = malloc((size_t)N * sizeof(uint64_t));
    if (!samples) {
        fprintf(stderr, "bench_publish_latency: malloc failed\n");
        sem_destroy(&g_lat_sem);
        message_bus_destroy(bus);
        return;
    }

    /* 预热 */
    for (int i = 0; i < 100; i++) {
        message_bus_publish(bus, "bench/lat", "bench", payload, sizeof(payload));
        sem_wait(&g_lat_sem);
    }

    for (int i = 0; i < N; i++) {
        uint64_t t0 = now_ns();
        message_bus_publish(bus, "bench/lat", "bench", payload, sizeof(payload));
        sem_wait(&g_lat_sem);
        samples[i] = g_lat_recv_ns - t0;
    }

    Stats s = calc_stats(samples, N);
    print_latency_row("普通路径延迟 (N=5000)", s);
    printf("  等效最大吞吐量: %.0f 消息/秒 (单串行流)\n", 1e9 / s.avg_ns);

    free(samples);
    sem_destroy(&g_lat_sem);
    message_bus_destroy(bus);
}

/* ══════════════════════════════════════════════════════════
 * Benchmark 4: Zero-Copy vs Copy Latency Comparison
 * 零拷贝回调在发布线程内同步触发，与普通异步路径对比
 * ══════════════════════════════════════════════════════════ */

static uint64_t g_zc_recv_ns;

static void zc_lat_cb(const char* t, const char* s, uint32_t id,
                      uint64_t ts, const void* data, uint32_t size, void* u) {
    (void)t; (void)s; (void)id; (void)ts; (void)data; (void)size; (void)u;
    g_zc_recv_ns = now_ns();
}

static void bench_zerocopy_vs_copy(void) {
    print_bench_header("Benchmark 4: 零拷贝 vs 普通路径 延迟对比");

    const int N = 5000;
    uint8_t payload[64];
    memset(payload, 0, sizeof(payload));
    uint64_t* samples = malloc((size_t)N * sizeof(uint64_t));
    if (!samples) {
        fprintf(stderr, "bench_zerocopy_vs_copy: malloc failed\n");
        return;
    }

    /* — 零拷贝路径：回调在发布线程内同步调用 — */
    {
        MessageBus* bus = message_bus_create("bench4_zc");
        message_bus_subscribe_zero_copy(bus, "bench/zc", zc_lat_cb, NULL);

        /* 预热 */
        for (int i = 0; i < 100; i++)
            message_bus_publish_zero_copy(bus, "bench/zc", "bench", payload, sizeof(payload));

        for (int i = 0; i < N; i++) {
            uint64_t t0 = now_ns();
            message_bus_publish_zero_copy(bus, "bench/zc", "bench", payload, sizeof(payload));
            samples[i] = g_zc_recv_ns - t0; /* 回调同步触发，无需 sem_wait */
        }

        Stats s = calc_stats(samples, N);
        print_latency_row("零拷贝路径 (N=5000)", s);

        message_bus_destroy(bus);
    }

    /* — 普通路径（复用 Benchmark 3 的测量结果做参考）— */
    {
        MessageBus* bus = message_bus_create("bench4_copy");
        sem_init(&g_lat_sem, 0, 0);
        message_bus_subscribe(bus, "bench/copy", lat_cb, NULL);

        /* 预热 */
        for (int i = 0; i < 100; i++) {
            message_bus_publish(bus, "bench/copy", "bench", payload, sizeof(payload));
            sem_wait(&g_lat_sem);
        }

        for (int i = 0; i < N; i++) {
            uint64_t t0 = now_ns();
            message_bus_publish(bus, "bench/copy", "bench", payload, sizeof(payload));
            sem_wait(&g_lat_sem);
            samples[i] = g_lat_recv_ns - t0;
        }

        Stats s = calc_stats(samples, N);
        print_latency_row("普通路径 (N=5000)", s);

        sem_destroy(&g_lat_sem);
        message_bus_destroy(bus);
    }

    free(samples);
}

/* ══════════════════════════════════════════════════════════
 * Benchmark 5: Req/Reply Round-Trip Latency
 * 同步 RPC 风格请求的往返延迟
 * ══════════════════════════════════════════════════════════ */

static void echo_service(const Message* req, Message* reply, void* u) {
    (void)u;
    memcpy(reply->data, req->data, req->data_size);
    reply->data_size = req->data_size;
    strncpy(reply->sender, "echo_svc", sizeof(reply->sender) - 1);
}

static void bench_req_reply_latency(void) {
    print_bench_header("Benchmark 5: Req/Reply 往返延迟 (RTT)");

    MessageBus* bus = message_bus_create("bench5");
    message_bus_register_service(bus, "bench/echo", echo_service, NULL);

    const int N = 2000;
    uint8_t req_data[32];
    memset(req_data, 0x42, sizeof(req_data));
    uint64_t* samples = malloc((size_t)N * sizeof(uint64_t));
    if (!samples) {
        fprintf(stderr, "bench_req_reply_latency: malloc failed\n");
        message_bus_destroy(bus);
        return;
    }
    Message   reply;

    /* 预热 */
    for (int i = 0; i < 50; i++)
        message_bus_request(bus, "bench/echo", "bench",
                            req_data, sizeof(req_data), &reply, 1000);

    for (int i = 0; i < N; i++) {
        uint64_t t0 = now_ns();
        message_bus_request(bus, "bench/echo", "bench",
                            req_data, sizeof(req_data), &reply, 1000);
        samples[i] = now_ns() - t0;
    }

    Stats s = calc_stats(samples, N);
    print_latency_row("Req/Reply RTT (N=2000)", s);
    printf("  最大 QPS: %.0f 次/秒\n", 1e9 / s.avg_ns);

    free(samples);
    message_bus_destroy(bus);
}

/* ── 主程序 ─────────────────────────────────────────────── */

int main(void) {
    printf("╔═══════════════════════════════════════════════════════════════════╗\n");
    printf("║             FlowEngine 核心组件性能基准测试                       ║\n");
    printf("╚═══════════════════════════════════════════════════════════════════╝\n");

    bench_raw_publish_rate();
    bench_sustained_throughput();
    bench_publish_latency();
    bench_zerocopy_vs_copy();
    bench_req_reply_latency();

    printf("\n");
    print_sep();
    printf("  基准测试完成\n");
    print_sep();
    printf("\n");
    return 0;
}
