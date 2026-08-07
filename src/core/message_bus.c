/**
 * message_bus.c — 轻量级进程内消息总线实现
 *
 * 特性：
 *  - Pub/Sub：异步环形缓冲队列 + 后台分发线程
 *  - Req/Reply：同步 RPC，带超时
 *  - 零拷贝：发布者线程直接调用订阅者回调，无内存拷贝
 *  - 通配符 "*" 订阅所有主题
 */

#include "message_bus.h"
#include "error_codes.h"
#include "clock_service.h"
#include "logger.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <pthread.h>
#include <time.h>
#include <stdatomic.h>
#include <unistd.h>

/* ── 分片分发线程数 ─────────────────────────────────────
 * 用 N 个分发线程并行处理回调，把单分发线程的吞吐上限抬起来。
 * 原理：多个生产者 message_bus_publish 按 msg_id % N 分片入队，
 * 每个分发线程独占一条队列（各自锁），并发 pop 并分发回调。
 * 各分片队列之间无竞争；订阅表/统计仍由 sub_mutex/topic_mutex 保护，
 * 因此并发安全依赖这些既有互斥锁，改动风险低。 */
#define MSG_BUS_DISPATCH_THREADS 4

/* ── QoS 决策位（打包进 topic_entries[].qos_flags，热路径免锁读） ──
 * 仅发布端做 depth 判断需要这几个位；lifespan 在 dispatch 侧读 qos 结构。
 * 默认（无任何位）= QOS_DROP_OLDEST + best_effort。 */
#define QOS_FLAG_RELIABLE    (1u << 0)   /**< reliability == QOS_RELIABLE → 等效 QOS_BLOCK */
#define QOS_FLAG_DROP_LATEST (1u << 1)   /**< policy == QOS_DROP_LATEST */
#define QOS_FLAG_BLOCK       (1u << 2)   /**< policy == QOS_BLOCK */

/* ── Subscriber entry ─────────────────────────────────── */

typedef struct {
    char            topic[MSG_BUS_MAX_TOPIC_LEN];
    MessageCallback callback;
    void*           user_data;
    bool            active;
    atomic_int      in_flight;  /**< 当前在途分发回调数：dispatch_message 持有
                                  *   快照期间 +1，回调返回后 -1。unsubscribe_ex
                                  *   等到 0 才放行，防止回调访问已释放的 user_data
                                  *   （协程 awaitable 在 await_resume 反注册后即析构）。
                                  *   改为原子：多分发线程并发快照时须原子增减。*/
} SubEntry;

typedef struct {
    char             topic[MSG_BUS_MAX_TOPIC_LEN];
    ZeroCopyCallback callback;
    void*            user_data;
    bool             active;
} ZcSubEntry;

typedef struct {
    char           topic[MSG_BUS_MAX_TOPIC_LEN];
    ServiceHandler handler;
    void*          user_data;
    bool           active;
} SvcEntry;

/* ── Ring buffer for async messages ──────────────────── */

typedef struct {
    /* 存 Message* 而非 Message 值：Message 内联 data[MSB_BUS_MAX_DATA_SIZE]=64KB，
     * 若按值存储，入队/出队各拷贝一次完整 64KB（memset 也 64KB），是压测吞吐瓶颈
     * （~70K 条/秒 ≈ 9GB/s memcpy）。改为堆分配 + 指针传递后，队列只搬 8 字节指针，
     * 拷贝从 128KB/条 降到 ~0，吞吐转为受 mutex/线程约束。 */
    Message*   msgs[MSG_BUS_QUEUE_SIZE];
    uint32_t   head;      /* next write position */
    uint32_t   tail;      /* next read position  */
    uint32_t   count;
    pthread_mutex_t mutex;
    pthread_cond_t  not_empty;
    pthread_cond_t  not_full;
} RingBuffer;

/* ── Req/Reply state ──────────────────────────────────── */

typedef struct {
    uint32_t  req_id;
    Message   reply;
    bool      done;
    pthread_mutex_t mutex;
    pthread_cond_t  cond;
} ReplySlot;

#define MAX_PENDING_REPLIES 16

/* ── Remap table entry ────────────────────────────────── */

#define BUS_MAX_REMAPS 32

typedef struct {
    char from[MSG_BUS_MAX_TOPIC_LEN];
    char to[MSG_BUS_MAX_TOPIC_LEN];
    bool active;
} RemapEntry;

/* ── Message 池 ───────────────────────────────────────────
 * Message 内联 data[MSG_BUS_MAX_DATA_SIZE]=64KB，每消息 malloc/free 一次 64KB
 * 是发布/分发两端隐藏的热路径开销（单生产者实测 ~8µs/条，几乎全耗在分配器）。
 * 用互斥保护的空闲链表复用已消费的 Message 块，把每消息一次 64KB 分配摊薄为
 * 一次短临界区（取/还都是 O(1)）。
 *
 * 注意：早期版本用 Treiber 无锁栈（CAS on head），在高并发（4 生产者 + 4 分发
 * 线程 + 驱逐）下触发经典 ABA：同一节点被并发 pop 两次 → double-alloc → 
 * double-free（glibc "double free or corruption" 崩溃）。互斥空闲链表无 ABA，
 * 正确性优先；临界区 ~ns 级，相比 64KB 分配仍是数量级提升。 */
#define MSG_POOL_MAX 1024

typedef struct {
    Message*        head;    /* 空闲链表头 */
    int             count;   /* 池内块数（上限判定） */
    pthread_mutex_t mutex;   /* 短临界区互斥，杜绝 ABA/double-free */
} MsgPool;

/* 归还一块已消费的 Message 到池（池满则释放，避免内存无界增长） */
static void msg_pool_push(MsgPool* p, Message* m) {
    pthread_mutex_lock(&p->mutex);
    if (p->count >= MSG_POOL_MAX) {
        pthread_mutex_unlock(&p->mutex);
        free(m);
        return;
    }
    m->_pool_next = p->head;
    p->head = m;
    p->count++;
    pthread_mutex_unlock(&p->mutex);
}

/* 从池取一块（池空返回 NULL，由调用方 malloc 兜底） */
static Message* msg_pool_take(MsgPool* p) {
    pthread_mutex_lock(&p->mutex);
    Message* m = p->head;
    if (m) {
        p->head = m->_pool_next;
        p->count--;
    }
    pthread_mutex_unlock(&p->mutex);
    return m;
}

/* ── Per-topic latency ring buffer (for p50/p99) ─────── */

#define BUS_LATENCY_RING_SIZE 128

typedef struct {
    uint64_t samples[BUS_LATENCY_RING_SIZE];
    uint32_t head;   /**< next write position */
    uint32_t count;  /**< number of valid entries (≤ BUS_LATENCY_RING_SIZE) */
} LatencyRing;

static void lat_ring_push(LatencyRing* r, uint64_t latency_us) {
    r->samples[r->head] = latency_us;
    r->head = (r->head + 1) % BUS_LATENCY_RING_SIZE;
    if (r->count < BUS_LATENCY_RING_SIZE) r->count++;
}

static int cmp_u64_bus(const void* a, const void* b) {
    uint64_t ua = *(const uint64_t*)a;
    uint64_t ub = *(const uint64_t*)b;
    return (ua > ub) - (ua < ub);
}

/** Compute p50/p99 from ring buffer. Must be called with topic_mutex held. */
static void lat_ring_percentiles(const LatencyRing* r,
                                  uint64_t* out_p50, uint64_t* out_p99) {
    *out_p50 = 0;
    *out_p99 = 0;
    if (r->count == 0) return;

    uint64_t sorted[BUS_LATENCY_RING_SIZE];
    uint32_t n = r->count;
    uint32_t start = (r->head + BUS_LATENCY_RING_SIZE - n) % BUS_LATENCY_RING_SIZE;
    for (uint32_t i = 0; i < n; i++)
        sorted[i] = r->samples[(start + i) % BUS_LATENCY_RING_SIZE];

    qsort(sorted, n, sizeof(uint64_t), cmp_u64_bus);

    /* Use (n-1)*percentile to avoid out-of-bounds on full arrays and to give
     * a lower-bound "nearest rank" result that works correctly for small n. */
    *out_p50 = sorted[(uint32_t)((n - 1) * 0.50)];
    *out_p99 = sorted[(uint32_t)((n - 1) * 0.99)];
}

/* ── MessageBus ───────────────────────────────────────── */

struct MessageBus {
    char name[64];

    /* Pub/Sub subscribers */
    SubEntry   subs[MSG_BUS_MAX_SUBSCRIBERS];
    int        sub_count;
    pthread_mutex_t sub_mutex;   /**< 写锁：subscribe/unsubscribe 修改订阅表 + 等待在途 */
    pthread_cond_t  sub_cv;   /**< 通知 unsubscribe_ex 在途分发已结束（配合 in_flight）*/
    atomic_uint subs_seq;      /**< 订阅表 seqlock：偶数=稳定，奇数=正在写。
                                 *   分发线程无锁快照订阅表；写者在修改前后递增。
                                 *   避免 N 个分发线程在每条消息上争抢 sub_mutex。 */

    /* Zero-copy subscribers */
    ZcSubEntry zc_subs[MSG_BUS_MAX_SUBSCRIBERS];
    int        zc_sub_count;
    pthread_mutex_t zc_mutex;

    /* Services */
    SvcEntry   svcs[MSG_BUS_MAX_TOPICS];
    int        svc_count;
    pthread_mutex_t svc_mutex;

    /* Async queue（分片） */
    RingBuffer shards[MSG_BUS_DISPATCH_THREADS];

    /* Dispatch threads（每个分片一条） */
    pthread_t  dispatch_threads[MSG_BUS_DISPATCH_THREADS];
    atomic_bool running;

    /* 每个分发线程的上下文（create 时填充，供线程函数定位自身分片） */
    struct { MessageBus* bus; RingBuffer* shard; int tid; } dispatch_ctx[MSG_BUS_DISPATCH_THREADS];

    /* Message ID counter */
    atomic_uint_fast32_t msg_id_counter;

    /* Reply slots for req/reply */
    ReplySlot  reply_slots[MAX_PENDING_REPLIES];
    pthread_mutex_t reply_mutex;

    /* Stats */
    atomic_uint_fast64_t stat_published;
    atomic_uint_fast64_t stat_delivered;
    atomic_uint_fast64_t stat_dropped;
    atomic_uint_fast64_t stat_zc_published;
    atomic_uint_fast64_t stat_zc_delivered;

    /* ── Per-topic tracking (QoS) ──────────────────────── */
    #define BUS_MAX_TOPIC_ENTRIES 128
    struct {
        char       topic[MSG_BUS_MAX_TOPIC_LEN];
        TopicQos   qos;
        TopicStats stats;              /**< 对外可见统计（读时合并无锁字段） */
        LatencyRing lat_ring;          /**< 兼容字段：读时合并各线程环后写入（勿在热路径直写） */
        atomic_uint_fast64_t prev_publish_us;  /**< 上一次发布时间戳（用于频率估算，无锁 CAS 更新） */
        atomic_uint pending_count;     /**< 当前在途（已入队未分发）消息数（原子，热路径免锁） */
        bool       active;
        /* ── 分发侧无锁累积（各分发线程原子更新，读时合并到 stats）── */
        atomic_uint_fast64_t deliver_count;       /**< 累计投递次数 */
        atomic_uint_fast64_t total_latency_us;    /**< 累计端到端延迟（算均值） */
        atomic_uint_fast64_t min_latency_us;      /**< 最小延迟 */
        atomic_uint_fast64_t max_latency_us;      /**< 最大延迟 */
        atomic_uint_fast64_t deadline_violations; /**< deadline 超时违规次数 */
        LatencyRing lat_ring_t[MSG_BUS_DISPATCH_THREADS]; /**< 每分发线程的延迟样本环，读时合并 */
        /* ── 发布侧无锁累积（各生产者线程原子更新，读时合并到 stats）──
         * 目标：发布热路径不再取全局 topic_mutex，全部改原子。 */
        atomic_uint_fast64_t pub_count;         /**< 累计发布次数 */
        atomic_uint_fast64_t last_publish_us;   /**< 最近发布时间 */
        atomic_uint_fast64_t freq_mhz;          /**< 估算发布频率，定点 1e-3 Hz（无锁） */
        atomic_uint drop_count;                 /**< 累计丢弃（含驱逐/满/过载）次数 */
        atomic_uint depth_eff;                  /**< 有效队列深度（QoS 配置时写入，热路径免锁读） */
        atomic_uint qos_flags;                  /**< QoS 决策位打包（见 QOS_FLAG_*，热路径免锁读） */
    } topic_entries[BUS_MAX_TOPIC_ENTRIES];
    atomic_int topic_count;
    pthread_mutex_t topic_mutex;
    atomic_bool has_lifespan_topics;   /**< 是否任一 topic 启用 lifespan。为 false 时
                                        *   dispatch 热路径跳过 lifespan 检查，避免加锁。 */

    /* Remap table entry */
    RemapEntry remaps[BUS_MAX_REMAPS];
    int        remap_count;
    pthread_mutex_t remap_mutex;
    atomic_bool remap_active;    /**< 是否已有 remap 规则。为 false（常见）时发布
                                  *   热路径跳过 remap_mutex，直接按原 topic 路由。 */

    /* Message 池（复用 64KB Message 块，免每消息 malloc/free） */
    MsgPool pool;
};

/* 分配一块 Message：优先复用池块，池空才 malloc（避免每条 64KB 分配） */
static Message* msg_alloc(MessageBus* bus) {
    Message* m = msg_pool_take(&bus->pool);
    return m ? m : (Message*)malloc(sizeof(Message));
}

/* ── Helpers ──────────────────────────────────────────── */

static bool topic_match(const char* pattern, const char* topic) {
    if (strcmp(pattern, "*") == 0) return true;
    return strcmp(pattern, topic) == 0;
}

/* 按 msg_id 选分片队列（round-robin），把并发生产者+分发线程分散到互不竞争的队列上 */
static RingBuffer* shard_for(RingBuffer* shards, uint32_t msg_id) {
    return &shards[msg_id % MSG_BUS_DISPATCH_THREADS];
}

/* ── 订阅表 seqlock 写辅助 ───────────────────────────────
 * 写者（subscribe/unsubscribe/unsubscribe_ex）在 sub_mutex 下修改 subs[]/sub_count，
 * 修改前后各递增一次 subs_seq：递增到奇数=写者进行中（读者自旋），
 * 再次递增回偶数=修改完成。读者据此判断快照是否一致。 */
static void subs_write_begin(MessageBus* bus) { atomic_fetch_add(&bus->subs_seq, 1); }
static void subs_write_end(MessageBus* bus)   { atomic_fetch_add(&bus->subs_seq, 1); }

/* 重算某 topic 的活跃订阅者数并写入 topic 统计（调用方须持 sub_mutex）。
 * 从每消息热路径移到 subscribe/unsubscribe 时调用，避免分发线程扫描订阅表。 */
static void update_subscriber_count(MessageBus* bus, const char* topic) {
    int c = 0;
    for (int i = 0; i < bus->sub_count; i++)
        if (bus->subs[i].active && topic_match(bus->subs[i].topic, topic)) c++;
    pthread_mutex_lock(&bus->topic_mutex);
    for (int i = 0; i < atomic_load(&bus->topic_count); i++)
        if (strcmp(bus->topic_entries[i].topic, topic) == 0)
            bus->topic_entries[i].stats.subscriber_count = (uint32_t)c;
    pthread_mutex_unlock(&bus->topic_mutex);
}

/* 无锁统计某 topic 的活跃订阅者数（seqlock 读订阅表，不取 sub_mutex）。
 * 供 publish 创建 topic 条目时初始化 subscriber_count。不能复用持 sub_mutex 的
 * update_subscriber_count：publish 此时已持 topic_mutex，若再取 sub_mutex 会与
 * subscribe 的 sub_mutex→topic_mutex 构成 AB-BA 死锁。 */
static int count_active_subscribers(MessageBus* bus, const char* topic) {
    uint32_t s1;
    int c;
    for (;;) {
        while ((s1 = atomic_load(&bus->subs_seq)) & 1u) { /* 写者进行中，自旋 */ }
        c = 0;
        for (int i = 0; i < bus->sub_count; i++)
            if (bus->subs[i].active && topic_match(bus->subs[i].topic, topic)) c++;
        if (atomic_load(&bus->subs_seq) == s1) return c;
    }
}

/* ── Ring buffer ops ─────────────────────────────────── */

static void rb_init(RingBuffer* rb) {
    memset(rb, 0, sizeof(*rb));
    pthread_mutex_init(&rb->mutex, NULL);
    pthread_cond_init(&rb->not_empty, NULL);
    pthread_cond_init(&rb->not_full, NULL);
}

static void rb_destroy(RingBuffer* rb) {
    /* 释放队列中残留的未消费消息，避免泄漏 */
    pthread_mutex_lock(&rb->mutex);
    for (uint32_t i = 0; i < rb->count; i++) {
        uint32_t idx = (rb->tail + i) % MSG_BUS_QUEUE_SIZE;
        free(rb->msgs[idx]);
        rb->msgs[idx] = NULL;
    }
    rb->count = 0;
    pthread_mutex_unlock(&rb->mutex);
    pthread_mutex_destroy(&rb->mutex);
    pthread_cond_destroy(&rb->not_empty);
    pthread_cond_destroy(&rb->not_full);
}

/* Returns 0 on success, -1 if full. 仅搬 8 字节指针，不再拷贝 64KB Message。 */
static int rb_push(RingBuffer* rb, Message* msg) {
    pthread_mutex_lock(&rb->mutex);
    if (rb->count >= MSG_BUS_QUEUE_SIZE) {
        pthread_mutex_unlock(&rb->mutex);
        return ERR_OVERFLOW;
    }
    rb->msgs[rb->head] = msg;
    rb->head = (rb->head + 1) % MSG_BUS_QUEUE_SIZE;
    rb->count++;
    pthread_cond_signal(&rb->not_empty);
    pthread_mutex_unlock(&rb->mutex);
    return 0;
}

/* Evict the oldest queued message matching `topic`.
 * Caller must NOT hold rb->mutex. Returns true if one was removed.
 * O(queue length) in the worst case; acceptable for the bounded 1024-slot queue. */
static bool rb_evict_oldest_topic(RingBuffer* rb, MsgPool* pool, const char* topic) {
    bool evicted = false;
    pthread_mutex_lock(&rb->mutex);
    /* Scan from oldest (tail) toward newest for the first topic match */
    for (uint32_t i = 0; i < rb->count; i++) {
        uint32_t idx = (rb->tail + i) % MSG_BUS_QUEUE_SIZE;
        if (strcmp(rb->msgs[idx]->topic, topic) == 0) {
            /* 被移除的是匹配元素（原 tail+i）。移位前先捕获其指针： */
            Message* to_free = rb->msgs[idx];
            /* 把更旧的元素 [0..i-1] 向上移一位（向 idx 处的空洞靠拢），
             * 使空洞移到 tail，同时覆盖掉匹配元素。 */
            for (uint32_t j = i; j > 0; j--) {
                uint32_t dst = (rb->tail + j) % MSG_BUS_QUEUE_SIZE;
                uint32_t src = (rb->tail + j - 1) % MSG_BUS_QUEUE_SIZE;
                rb->msgs[dst] = rb->msgs[src];
            }
            /* 移位后 tail 处是 E0 的重复副本（E0 已被复制到 tail+1），
             * 只清空该槽并推进 tail，绝不能 free —— 否则会 free 掉仍在
             * tail+1 存活的指针，造成 UAF / double-free。 */
            rb->msgs[rb->tail] = NULL;
            msg_pool_push(pool, to_free);   /* 归还池而非 free，避免 64KB 分配 */
            rb->tail = (rb->tail + 1) % MSG_BUS_QUEUE_SIZE;
            rb->count--;
            pthread_cond_signal(&rb->not_full);
            evicted = true;
            break;
        }
    }
    pthread_mutex_unlock(&rb->mutex);
    return evicted;
}

/* QOS_DROP_OLDEST 跨分片驱逐：消息按 msg_id 分片，需在 N 个分片中找一条
 * 该 topic 的最旧消息驱逐。按序扫分片，命中即驱逐一条（保证只清理一条）。
 * publish 调用方已做 pending 计数与 drop 计数（原子），此处仅做队列级驱逐。 */
static bool rb_evict_oldest_topic_all(RingBuffer* shards, MsgPool* pool, int n,
                                      const char* topic) {
    bool evicted = false;
    for (int s = 0; s < n && !evicted; s++) {
        if (rb_evict_oldest_topic(&shards[s], pool, topic)) evicted = true;
    }
    return evicted;
}

/* Blocks until a message is available or bus stops.
 * 返回堆分配的 Message*（所有权移交调用方，处理后须 free），无消息返回 NULL。 */
static Message* rb_pop(RingBuffer* rb, atomic_bool* running) {
    pthread_mutex_lock(&rb->mutex);
    while (rb->count == 0 && atomic_load(running)) {
        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        ts.tv_nsec += 100000000LL; /* 100ms */
        if (ts.tv_nsec >= 1000000000LL) {
            ts.tv_sec++;
            ts.tv_nsec -= 1000000000LL;
        }
        pthread_cond_timedwait(&rb->not_empty, &rb->mutex, &ts);
    }
    if (rb->count == 0) {
        pthread_mutex_unlock(&rb->mutex);
        return NULL;
    }
    Message* m = rb->msgs[rb->tail];
    rb->msgs[rb->tail] = NULL;
    rb->tail = (rb->tail + 1) % MSG_BUS_QUEUE_SIZE;
    rb->count--;
    pthread_mutex_unlock(&rb->mutex);
    return m;
}

/* ── Dispatch message to subscribers ─────────────────── */

static void dispatch_message(MessageBus* bus, const Message* msg, int tid) {
    int delivered = 0;

    /* ── Lifespan check: drop expired messages before dispatching ──
     * 仅当任一 topic 启用 lifespan 时才进入（has_lifespan 原子标志）。绝大多数
     * 运行无 lifespan（默认 0），此路径被跳过，热路径不加 topic_mutex。
     * 命中时用 msg->topic_idx 免锁定位条目；lifespan 是低频显式开关，qos 的
     * 并发改写属可接受的良性竞争。 */
    if (atomic_load(&bus->has_lifespan_topics) &&
        msg->type == MSG_TYPE_PUBLISH && msg->timestamp_us > 0 &&
        msg->topic_idx >= 0 && msg->topic_idx < atomic_load(&bus->topic_count)) {
        typeof(bus->topic_entries[0])* e = &bus->topic_entries[msg->topic_idx];
        TopicQos* q = &e->qos;
        if (q->lifespan_ms > 0) {
            /* lifespan 衡量消息在队列里"真实"滞留时长，须用墙钟时间：
             * 仿真模式下 clock_now_us() 是按固定步长推进的逻辑时间。 */
            uint64_t age_ms = (clock_now_monotonic_wall_us() - msg->timestamp_us) / 1000ULL;
            if (age_ms > (uint64_t)q->lifespan_ms) {
                /* 良性竞争：drop_count 为低频监控计数，lifespan 过期罕见。
                 * 计入原子 drop_count 而非 stats.drop_count——get_topic_stats 经
                 * merge_topic_stats 读的是原子字段，二者必须一致才可见。 */
                uint64_t dc = atomic_fetch_add(&e->drop_count, 1) + 1;
                if (dc % 100 == 1) {
                    LOG_WARN("message_bus", "topic '%s' lifespan expired (%lu ms > %d ms), drop_count=%lu",
                             msg->topic, (unsigned long)age_ms, q->lifespan_ms,
                             (unsigned long)dc);
                }
                atomic_fetch_sub(&e->pending_count, 1);
                return; /* message expired — skip dispatch entirely */
            }
        }
    }

    /* Snapshot matching subscribers under lock to avoid holding lock during callbacks.
     * This prevents deadlock when a callback calls subscribe/unsubscribe on the bus.
     * Stack allocation is safe: MSG_BUS_MAX_SUBSCRIBERS is 32 (256 bytes max).
     *
     * in_flight 引用计数：快照时 +1，回调返回后 -1。unsubscribe_ex 会等到
     * in_flight 归零才放行，从而保证回调期间 user_data（如协程 awaitable）
     * 不会被析构释放——杜绝"快照后、回调前 awaitable 被销毁"的 UAF。 */
    typedef struct { MessageCallback cb; void* ud; SubEntry* src; } CbSnap;
    CbSnap snap[MSG_BUS_MAX_SUBSCRIBERS];

    /* 无锁快照订阅表（seqlock + 原子 in_flight），避免 N 个分发线程在每条
     * 消息上争抢 sub_mutex。写者（subscribe/unsubscribe）在 sub_mutex 下修改
     * subs[] 并递增 subs_seq；读者循环：
     *   读 seq（奇数=写者进行中则自旋）→ 快照匹配订阅并原子 in_flight++ →
     *   重读 seq 校验未变，变了则回滚已增 in_flight 并重试。
     * in_flight++ 必须落在 seqlock 临界区内：保证 unsubscribe_ex 将 active 置
     * false 之后、等待 in_flight 归零之前，读者要么重试看到非 active、要么已持
     * in_flight 引用，从而杜绝"快照后、回调前 awaitable 被销毁"的 UAF。 */
    int snap_count;
retry_snapshot:
    snap_count = 0;
    uint32_t s1;
    while ((s1 = atomic_load(&bus->subs_seq)) & 1u) { /* 写者进行中，自旋 */ }
    for (int i = 0; i < bus->sub_count; i++) {
        SubEntry* s = &bus->subs[i];
        if (!s->active) continue;
        if (topic_match(s->topic, msg->topic)) {
            atomic_fetch_add(&s->in_flight, 1);
            snap[snap_count++] = (CbSnap){ s->callback, s->user_data, s };
        }
    }
    if (atomic_load(&bus->subs_seq) != s1) {
        /* 快照期间写者插入：回滚已增加的 in_flight，重试 */
        for (int i = 0; i < snap_count; i++)
            atomic_fetch_sub(&snap[i].src->in_flight, 1);
        goto retry_snapshot;
    }
    /* 2026-07-31 断流诊断：control/cmd 每 200 条打印一次订阅者画像——
     * 确认 dispatch 是否分发给 flowsim（回调指针对比）。多分发线程并发，
     * 用原子计数避免数据竞争。 */
    if (msg->type == MSG_TYPE_PUBLISH &&
        strcmp(msg->topic, "control/cmd") == 0) {
        static atomic_uint_fast64_t s_cmd_diag = 0;
        uint64_t n = atomic_fetch_add(&s_cmd_diag, 1) + 1;
        if (n % 200 == 1) {
            char who[128] = "";
            for (int i = 0; i < snap_count && i < 6; i++) {
                char tmp[24];
                snprintf(tmp, sizeof tmp, " %s%p", i > 0 ? "," : "",
                         (void*)snap[i].cb);
                strncat(who, tmp, sizeof who - strlen(who) - 1);
            }
            LOG_WARN("message_bus", "[CMD_DIAG] control/cmd dispatch #%llu: "
                     "%d subscribers (cb:%s)", (unsigned long long)n,
                     snap_count, who);
        }
    }

    for (int i = 0; i < snap_count; i++) {
        snap[i].cb(msg, snap[i].ud);
        atomic_fetch_add(&bus->stat_delivered, 1);
        delivered++;
    }

    /* 回调全部返回后原子扣减在途计数，并唤醒可能在 unsubscribe_ex 中等待的
     * 线程。广播无需持有 sub_mutex：unsubscribe_ex 在 sub_mutex 下循环重查
     * 原子 in_flight，丢失的唤醒会被条件重查兜住（标准 condvar 循环模式）。 */
    for (int i = 0; i < snap_count; i++)
        atomic_fetch_sub(&snap[i].src->in_flight, 1);
    pthread_cond_broadcast(&bus->sub_cv);

    /* Per-topic tracking: decrement in-flight count + update delivery stats.
     * 全部改为无锁：pending_count 与各计数原子，latency 样本推入当前分发线程
     * 自己的环（lat_ring_t[tid]），读时再合并——避免 N 个分发线程争抢 topic_mutex。 */
    if (msg->type == MSG_TYPE_PUBLISH &&
        msg->topic_idx >= 0 && msg->topic_idx < atomic_load(&bus->topic_count)) {
        typeof(bus->topic_entries[0])* e = &bus->topic_entries[msg->topic_idx];
        /* Message left the queue — free one slot of in-flight budget */
        atomic_fetch_sub(&e->pending_count, 1);

        if (delivered > 0) {
            atomic_fetch_add(&e->deliver_count, (uint64_t)delivered);
            /* 延迟用墙钟单调时间，避免仿真模式下逻辑时钟按固定步长推进失真。 */
            uint64_t now = clock_now_monotonic_wall_us();
            uint64_t lat = now - msg->timestamp_us;
            atomic_fetch_add(&e->total_latency_us, lat);
            /* min/max：CAS 更新（监控级，竞态仅轻微影响统计值） */
            uint64_t mn = atomic_load(&e->min_latency_us);
            while (mn == 0 || lat < mn) {
                if (atomic_compare_exchange_weak(&e->min_latency_us, &mn, lat)) break;
            }
            uint64_t mx = atomic_load(&e->max_latency_us);
            while (lat > mx) {
                if (atomic_compare_exchange_weak(&e->max_latency_us, &mx, lat)) break;
            }
            /* 推入当前分发线程自己的延迟样本环（无竞争） */
            lat_ring_push(&e->lat_ring_t[tid], lat);

            /* Deadline violation: end-to-end dispatch latency exceeded deadline_ms */
            TopicQos* q = &e->qos;
            if (q->deadline_ms > 0 && lat > (uint64_t)q->deadline_ms * 1000ULL)
                atomic_fetch_add(&e->deadline_violations, 1);
        }
    }

    /* Handle reply if this is a REPLY message */
    if (msg->type == MSG_TYPE_REPLY) {
        for (int i = 0; i < MAX_PENDING_REPLIES; i++) {
            ReplySlot* slot = &bus->reply_slots[i];
            pthread_mutex_lock(&slot->mutex);
            if (slot->req_id == msg->msg_id && !slot->done) {
                slot->reply = *msg;
                slot->done  = true;
                pthread_cond_signal(&slot->cond);
                pthread_mutex_unlock(&slot->mutex);
                break;
            }
            pthread_mutex_unlock(&slot->mutex);
        }
    }
}

/* ── Dispatch thread ─────────────────────────────────── */

static void* dispatch_thread_fn(void* arg) {
    /* 每个分发线程独占一个分片队列，无跨分片锁竞争 */
    struct { MessageBus* bus; RingBuffer* shard; int tid; }* dc =
        (struct { MessageBus* bus; RingBuffer* shard; int tid; }*)arg;
    MessageBus* bus = dc->bus;
    int tid = dc->tid;

    while (atomic_load(&bus->running)) {
        /* Pass the live atomic flag (not a stale snapshot) so a shutdown
         * request during the blocking wait is observed immediately. */
        Message* msg = rb_pop(dc->shard, &bus->running);
        if (!msg) continue;

        if (msg->type == MSG_TYPE_REQUEST) {
            /* Dispatch to service handler */
            pthread_mutex_lock(&bus->svc_mutex);
            SvcEntry* found = NULL;
            for (int i = 0; i < bus->svc_count; i++) {
                if (bus->svcs[i].active &&
                    strcmp(bus->svcs[i].topic, msg->topic) == 0) {
                    found = &bus->svcs[i];
                    break;
                }
            }
            if (found) {
                Message reply;
                memset(&reply, 0, sizeof(reply));
                snprintf(reply.topic, MSG_BUS_MAX_TOPIC_LEN, "%s", msg->topic);
                reply.msg_id    = msg->msg_id;
                reply.type      = MSG_TYPE_REPLY;
                reply.timestamp_us = clock_now_monotonic_wall_us();
                found->handler(msg, &reply, found->user_data);
                pthread_mutex_unlock(&bus->svc_mutex);

                /* Deliver reply to waiting caller */
                for (int i = 0; i < MAX_PENDING_REPLIES; i++) {
                    ReplySlot* slot = &bus->reply_slots[i];
                    pthread_mutex_lock(&slot->mutex);
                    if (slot->req_id == msg->msg_id && !slot->done) {
                        slot->reply = reply;
                        slot->done  = true;
                        pthread_cond_signal(&slot->cond);
                        pthread_mutex_unlock(&slot->mutex);
                        break;
                    }
                    pthread_mutex_unlock(&slot->mutex);
                }
            } else {
                pthread_mutex_unlock(&bus->svc_mutex);
            }
            msg_pool_push(&bus->pool, msg);
        } else {
            dispatch_message(bus, msg, tid);
            msg_pool_push(&bus->pool, msg);
        }
    }
    return NULL;
}

/* ── Lifecycle ────────────────────────────────────────── */

MessageBus* message_bus_create(const char* bus_name) {
    MessageBus* bus = (MessageBus*)calloc(1, sizeof(MessageBus));
    if (!bus) return NULL;

    if (bus_name)
        snprintf(bus->name, sizeof(bus->name), "%s", bus_name);

    for (int i = 0; i < MSG_BUS_DISPATCH_THREADS; i++)
        rb_init(&bus->shards[i]);
    pthread_mutex_init(&bus->sub_mutex, NULL);
    pthread_cond_init(&bus->sub_cv, NULL);
    atomic_init(&bus->subs_seq, 0);
    pthread_mutex_init(&bus->zc_mutex, NULL);
    pthread_mutex_init(&bus->svc_mutex, NULL);
    pthread_mutex_init(&bus->reply_mutex, NULL);

    for (int i = 0; i < MAX_PENDING_REPLIES; i++) {
        pthread_mutex_init(&bus->reply_slots[i].mutex, NULL);
        pthread_cond_init(&bus->reply_slots[i].cond, NULL);
    }

    atomic_init(&bus->msg_id_counter, 1);
    atomic_init(&bus->stat_published, 0);
    atomic_init(&bus->stat_delivered, 0);
    atomic_init(&bus->stat_dropped, 0);
    atomic_init(&bus->stat_zc_published, 0);
    atomic_init(&bus->stat_zc_delivered, 0);
    pthread_mutex_init(&bus->topic_mutex, NULL);
    atomic_init(&bus->has_lifespan_topics, false);
    pthread_mutex_init(&bus->remap_mutex, NULL);
    atomic_init(&bus->remap_active, false);
    pthread_mutex_init(&bus->pool.mutex, NULL);
    bus->pool.head = NULL;
    bus->pool.count = 0;

    atomic_store(&bus->running, true);
    for (int i = 0; i < MSG_BUS_DISPATCH_THREADS; i++) {
        bus->dispatch_ctx[i].bus   = bus;
        bus->dispatch_ctx[i].shard = &bus->shards[i];
        bus->dispatch_ctx[i].tid   = i;
        if (pthread_create(&bus->dispatch_threads[i], NULL,
                           dispatch_thread_fn, &bus->dispatch_ctx[i]) != 0) {
            atomic_store(&bus->running, false);
            /* 唤醒已启动的线程，避免 destroy 时 join 卡住 */
            for (int j = 0; j < i; j++) {
                pthread_mutex_lock(&bus->shards[j].mutex);
                pthread_cond_broadcast(&bus->shards[j].not_empty);
                pthread_mutex_unlock(&bus->shards[j].mutex);
            }
            for (int j = 0; j < i; j++) pthread_join(bus->dispatch_threads[j], NULL);
            free(bus);
            return NULL;
        }
    }
    return bus;
}

void message_bus_destroy(MessageBus* bus) {
    if (!bus) return;
    atomic_store(&bus->running, false);
    /* Wake all dispatch threads if they are blocked waiting so they observe
     * the stop request promptly instead of after the wait timeout. */
    for (int i = 0; i < MSG_BUS_DISPATCH_THREADS; i++) {
        pthread_mutex_lock(&bus->shards[i].mutex);
        pthread_cond_broadcast(&bus->shards[i].not_empty);
        pthread_mutex_unlock(&bus->shards[i].mutex);
        pthread_join(bus->dispatch_threads[i], NULL);
    }

    for (int i = 0; i < MSG_BUS_DISPATCH_THREADS; i++)
        rb_destroy(&bus->shards[i]);
    pthread_mutex_destroy(&bus->sub_mutex);
    pthread_cond_destroy(&bus->sub_cv);
    pthread_mutex_destroy(&bus->zc_mutex);
    pthread_mutex_destroy(&bus->svc_mutex);
    pthread_mutex_destroy(&bus->reply_mutex);
    pthread_mutex_destroy(&bus->topic_mutex);
    pthread_mutex_destroy(&bus->remap_mutex);

    for (int i = 0; i < MAX_PENDING_REPLIES; i++) {
        pthread_mutex_destroy(&bus->reply_slots[i].mutex);
        pthread_cond_destroy(&bus->reply_slots[i].cond);
    }

    /* 释放消息池中残留块 */
    Message* pm = bus->pool.head;
    while (pm) {
        Message* next = pm->_pool_next;
        free(pm);
        pm = next;
    }
    pthread_mutex_destroy(&bus->pool.mutex);
    free(bus);
}

/* ── Pub/Sub ─────────────────────────────────────────── */

int message_bus_publish(MessageBus* bus, const char* topic, const char* sender,
                        const void* data, uint32_t size) {
    if (!bus || !topic) return ERR_INVALID_PARAM;
    if (size > MSG_BUS_MAX_DATA_SIZE) return ERR_OVERFLOW;

    /* ── Remap: resolve topic to its routing target ──
     * 无 remap 规则（remap_active==false，常见）时热路径零加锁直接路由。 */
    char resolved_topic[MSG_BUS_MAX_TOPIC_LEN];
    snprintf(resolved_topic, MSG_BUS_MAX_TOPIC_LEN, "%s", topic);
    if (atomic_load(&bus->remap_active)) {
        pthread_mutex_lock(&bus->remap_mutex);
        for (int i = 0; i < bus->remap_count; i++) {
            if (bus->remaps[i].active &&
                strcmp(bus->remaps[i].from, topic) == 0) {
                snprintf(resolved_topic, MSG_BUS_MAX_TOPIC_LEN, "%s", bus->remaps[i].to);
                break;
            }
        }
        pthread_mutex_unlock(&bus->remap_mutex);
    }

    /* ── topic 条目无锁查找 ──
     * topic_entries 只追加、不删除：条目一经创建其 topic 名不再变化，且注册
     * 线程在写满所有字段后才以 release 递增 topic_count。读侧用 acquire 读
     * topic_count，即可安全地无锁遍历 [0, count) 已完整初始化的条目。
     * 热路径（topic 已注册）零加锁。 */
    int ti = -1;
    {
        int tc = atomic_load_explicit(&bus->topic_count, memory_order_acquire);
        for (int i = 0; i < tc && i < BUS_MAX_TOPIC_ENTRIES; i++) {
            if (strcmp(bus->topic_entries[i].topic, resolved_topic) == 0) { ti = i; break; }
        }
    }
    if (ti < 0) {
        /* 慢路径：注册新 topic（罕见）。取锁避免并发 double-create。 */
        pthread_mutex_lock(&bus->topic_mutex);
        for (int i = 0; i < atomic_load(&bus->topic_count); i++) {
            if (strcmp(bus->topic_entries[i].topic, resolved_topic) == 0) { ti = i; break; }
        }
        if (ti < 0) {
            /* 先在锁内把条目完整初始化，最后才 release 递增 topic_count：
             * 发布热路径用 acquire 读 topic_count 后无锁遍历条目，若先递增
             * count 再写字段，读者会看到半初始化（topic 为垃圾）的条目，
             * 造成错误匹配/崩溃。先写后 publish 保证读者只见完整条目。 */
            int tc = atomic_load(&bus->topic_count);
            if (tc < BUS_MAX_TOPIC_ENTRIES) {
                ti = tc;
                memset(&bus->topic_entries[ti], 0, sizeof(bus->topic_entries[ti]));
                snprintf(bus->topic_entries[ti].topic, MSG_BUS_MAX_TOPIC_LEN, "%s", resolved_topic);
                bus->topic_entries[ti].active = true;
                bus->topic_entries[ti].qos.policy = QOS_DROP_OLDEST;
                bus->topic_entries[ti].stats.subscriber_count =
                    (uint32_t)count_active_subscribers(bus, resolved_topic);
                atomic_store(&bus->topic_entries[ti].depth_eff, MSG_BUS_QUEUE_SIZE);
                atomic_store(&bus->topic_entries[ti].qos_flags, 0u); /* DROP_OLDEST + best_effort */
                /* 其余原子字段（pending/pub_count/deliver_count/...）依赖 memset 的 0 初值，
                 * 与既有代码对 pending_count/deliver_count 的零初始化约定一致。 */
                atomic_store_explicit(&bus->topic_count, tc + 1, memory_order_release);
            }
        }
        pthread_mutex_unlock(&bus->topic_mutex);
    }

    /* ── QoS: per-topic queue depth enforcement（无锁，原子 pending/depth）── */
    bool should_drop = false;
    bool need_evict  = false;
    typeof(bus->topic_entries[0])* e = NULL;
    if (ti >= 0) {
        e = &bus->topic_entries[ti];
        uint32_t depth = atomic_load(&e->depth_eff);
        uint32_t qf    = atomic_load(&e->qos_flags);
        if (atomic_load(&e->pending_count) >= depth) {
            /* QOS_RELIABLE 覆盖策略：总是阻塞以保证送达 */
            QosPolicy effective_policy = (qf & QOS_FLAG_RELIABLE)
                                         ? QOS_BLOCK
                                         : (qf & QOS_FLAG_DROP_LATEST) ? QOS_DROP_LATEST
                                         : (qf & QOS_FLAG_BLOCK)       ? QOS_BLOCK
                                                                       : QOS_DROP_OLDEST;
            switch (effective_policy) {
                case QOS_DROP_OLDEST:
                    need_evict = true;   /* 驱逐在下方做（无锁，O(queue)） */
                    break;
                case QOS_DROP_LATEST:
                    should_drop = true;
                    {
                        uint64_t dc = atomic_fetch_add(&e->drop_count, 1) + 1;
                        if (dc % 100 == 1) {
                            LOG_WARN("message_bus", "topic '%s' QOS_DROP_LATEST depth=%u full, drop_count=%lu",
                                     resolved_topic, depth, (unsigned long)dc);
                        }
                    }
                    break;
                case QOS_BLOCK: {
                    /* 无锁忙等 pending 降到 depth 以下（有界）。
                     * QOS_RELIABLE 等 5s，其余 1s，超时丢弃，避免消费者消失时死锁。 */
                    int max_waits = (qf & QOS_FLAG_RELIABLE) ? 5000 : 1000;
                    int waits = 0;
                    while (atomic_load(&e->pending_count) >= depth && waits < max_waits) {
                        usleep(1000);  /* 1ms */
                        waits++;
                    }
                    if (atomic_load(&e->pending_count) >= depth) {
                        should_drop = true;
                        uint64_t dc = atomic_fetch_add(&e->drop_count, 1) + 1;
                        if (dc % 100 == 1) {
                            LOG_WARN("message_bus", "topic '%s' QOS_BLOCK timeout after %d ms, drop_count=%lu",
                                     resolved_topic, max_waits, (unsigned long)dc);
                        }
                    }
                    break;
                }
            }
        }
    }

    if (should_drop) {
        atomic_fetch_add(&bus->stat_dropped, 1);
        return 0;  /* dropped per QoS policy, not a hard error */
    }

    /* DROP_OLDEST: make room by removing the oldest queued message of this topic. */
    if (need_evict && e &&
        rb_evict_oldest_topic_all(bus->shards, &bus->pool,
                                  MSG_BUS_DISPATCH_THREADS, resolved_topic)) {
        if (atomic_load(&e->pending_count) > 0)
            atomic_fetch_sub(&e->pending_count, 1);
        uint64_t dc = atomic_fetch_add(&e->drop_count, 1) + 1;
        if (dc % 100 == 1) {
            LOG_WARN("message_bus", "topic '%s' QOS_DROP_OLDEST evicted oldest, drop_count=%lu",
                     resolved_topic, (unsigned long)dc);
        }
        atomic_fetch_add(&bus->stat_dropped, 1);
    }

    /* 取 Message（优先复用池块），只初始化头部字段（避免整块 64KB memset）。 */
    Message* msg = msg_alloc(bus);
    if (!msg) return ERR_NOMEM;
    uint64_t msg_ts = clock_now_monotonic_wall_us();  /* 入队前缓存，rb_push 后 msg 可能已被分发线程回收 */
    snprintf(msg->topic, MSG_BUS_MAX_TOPIC_LEN, "%s", resolved_topic);
    if (sender) snprintf(msg->sender, MSG_BUS_MAX_SENDER_LEN, "%s", sender);
    msg->msg_id       = atomic_fetch_add(&bus->msg_id_counter, 1);
    msg->type         = MSG_TYPE_PUBLISH;
    msg->timestamp_us = msg_ts;
    msg->topic_idx    = ti;
    msg->data_size    = size;
    msg->type_id      = 0;
    msg->schema_version = 0;
    msg->endian_marker  = 0;
    if (data && size > 0) memcpy(msg->data, data, size);

    int ret = rb_push(shard_for(bus->shards, msg->msg_id), msg);
    if (ret != 0) msg_pool_push(&bus->pool, msg);   /* 入队失败归还池 */

    if (ret == 0) {
        atomic_fetch_add(&bus->stat_published, 1);
    } else {
        atomic_fetch_add(&bus->stat_dropped, 1);
        uint64_t total = atomic_load(&bus->stat_dropped);
        if (total % 100 == 1) {
            LOG_WARN("message_bus", "ring buffer full (topic '%s'), total_dropped=%lu",
                     resolved_topic, (unsigned long)total);
        }
    }

    /* ── Per-topic tracking（无锁） ── */
    if (ret == 0 && ti >= 0) {
        atomic_fetch_add(&e->pending_count, 1);
        atomic_fetch_add(&e->pub_count, 1);
        atomic_store(&e->last_publish_us, msg_ts);
        /* Frequency: EWMA from inter-arrival gap（原子 prev，近似即可） */
        uint64_t prev = atomic_load(&e->prev_publish_us);
        if (prev != 0 && msg_ts > prev) {
            uint64_t elapsed = msg_ts - prev;
            if (elapsed > 500) {  /* 拒绝突发/启动噪声 */
                double instant_hz = 1000000.0 / (double)elapsed;
                double inst_mhz = instant_hz * 1000.0;
                uint64_t cur = atomic_load(&e->freq_mhz);
                double next = (cur == 0) ? inst_mhz : ((double)cur * 0.8 + inst_mhz * 0.2);
                atomic_store(&e->freq_mhz, (uint64_t)next);
            }
        }
        atomic_store(&e->prev_publish_us, msg_ts);
    } else if (ret != 0 && ti >= 0) {
        uint64_t dc = atomic_fetch_add(&e->drop_count, 1) + 1;
        if (dc % 100 == 1) {
            LOG_WARN("message_bus", "topic '%s' ring buffer push failed, drop_count=%lu",
                     resolved_topic, (unsigned long)dc);
        }
    }

    return ret;
}

int message_bus_subscribe(MessageBus* bus, const char* topic,
                          MessageCallback callback, void* user_data) {
    if (!bus || !topic || !callback) return ERR_INVALID_PARAM;

    pthread_mutex_lock(&bus->sub_mutex);
    subs_write_begin(bus);

    /* First try to reuse an inactive (previously unsubscribed) slot */
    SubEntry* e = NULL;
    for (int i = 0; i < bus->sub_count; i++) {
        if (!bus->subs[i].active) {
            e = &bus->subs[i];
            break;
        }
    }

    /* No free slot found — allocate a new one */
    if (!e) {
        if (bus->sub_count >= MSG_BUS_MAX_SUBSCRIBERS) {
            /* 2026-07-31 safety_control 协程挂起事故候选：订阅表 32 槽满时
             * 静默失败（WhenAnyBusAwaitableT::await_suspend 不检查返回值），
             * 节点收不到消息且无任何日志。此处打 WARN 暴露。 */
            LOG_WARN("message_bus", "SUB_OVERFLOW: topic '%s' cb=%p ud=%p — subscriber table "
                     "full (%d/%d), subscription silently dropped!",
                     topic, (void*)callback, user_data,
                     bus->sub_count, MSG_BUS_MAX_SUBSCRIBERS);
            subs_write_end(bus);
            pthread_mutex_unlock(&bus->sub_mutex);
            return ERR_OVERFLOW;
        }
        e = &bus->subs[bus->sub_count++];
    }

    snprintf(e->topic, MSG_BUS_MAX_TOPIC_LEN, "%s", topic);
    e->callback  = callback;
    e->user_data = user_data;
    atomic_store(&e->in_flight, 0);
    e->active    = true;
    update_subscriber_count(bus, topic);
    subs_write_end(bus);
    pthread_mutex_unlock(&bus->sub_mutex);
    return 0;
}

int message_bus_unsubscribe(MessageBus* bus, const char* topic, MessageCallback callback) {
    if (!bus || !topic || !callback) return ERR_INVALID_PARAM;
    int found = -1;
    pthread_mutex_lock(&bus->sub_mutex);
    subs_write_begin(bus);
    for (int i = 0; i < bus->sub_count; i++) {
        if (bus->subs[i].active &&
            strcmp(bus->subs[i].topic, topic) == 0 &&
            bus->subs[i].callback == callback) {
            bus->subs[i].active = false;
            found = 0;
            break;
        }
    }
    if (found == 0) update_subscriber_count(bus, topic);
    subs_write_end(bus);
    pthread_mutex_unlock(&bus->sub_mutex);
    return found;
}

int message_bus_unsubscribe_ex(MessageBus* bus, const char* topic,
                               MessageCallback callback, void* user_data) {
    if (!bus || !topic || !callback) return ERR_INVALID_PARAM;
    int found = -1;
    /* 分发线程自身的回调内反注册时不等待 in_flight（否则会因等待自己而
     * 自死锁）；此时 user_data 必然仍存活（回调正在使用它），可安全放行。
     * 多分发线程下须比对全部分发线程，而非旧单线程的 bus->dispatch_thread，
     * 否则回调运行在非 threads[0] 的分发线程上自我反注册会误判而自死锁。*/
    bool on_dispatch_thread = false;
    pthread_t self = pthread_self();
    for (int t = 0; t < MSG_BUS_DISPATCH_THREADS; t++) {
        if (pthread_equal(self, bus->dispatch_threads[t])) {
            on_dispatch_thread = true;
            break;
        }
    }
    pthread_mutex_lock(&bus->sub_mutex);
    subs_write_begin(bus);
    int found_idx = -1;
    for (int i = 0; i < bus->sub_count; i++) {
        SubEntry* s = &bus->subs[i];
        if (s->active &&
            strcmp(s->topic, topic) == 0 &&
            s->callback  == callback &&
            s->user_data == user_data) {
            s->active = false;
            found = 0;
            found_idx = i;
            break;
        }
    }
    if (found == 0) update_subscriber_count(bus, topic);
    /* 结束 seqlock 写，读者才能看到 active=false（否则读者会一直自旋）。 */
    subs_write_end(bus);

    /* 等待该订阅上所有在途分发回调完成：dispatch_message 在回调返回前持有
     * in_flight 引用，此处等到 0 才放行，保证回调不会访问已释放的 user_data
     * （协程 awaitable 在 await_resume 反注册后即析构）。in_flight 是原子，
     * 在 sub_mutex 下循环重查 + condvar 等待；丢失的唤醒由条件重查兜住。 */
    if (found == 0 && !on_dispatch_thread) {
        SubEntry* s = &bus->subs[found_idx];
        while (atomic_load(&s->in_flight) > 0)
            pthread_cond_wait(&bus->sub_cv, &bus->sub_mutex);
    }
    pthread_mutex_unlock(&bus->sub_mutex);
    return found;
}

/* ── Req/Reply ───────────────────────────────────────── */

int message_bus_request(MessageBus* bus, const char* topic, const char* sender,
                        const void* data, uint32_t size,
                        Message* reply, uint32_t timeout_ms) {
    if (!bus || !topic || !reply) return ERR_OVERFLOW;
    if (size > MSG_BUS_MAX_DATA_SIZE) return ERR_OVERFLOW;

    uint32_t req_id = atomic_fetch_add(&bus->msg_id_counter, 1);

    /* Allocate a reply slot */
    ReplySlot* slot = NULL;
    for (int i = 0; i < MAX_PENDING_REPLIES; i++) {
        ReplySlot* s = &bus->reply_slots[i];
        pthread_mutex_lock(&s->mutex);
        if (!s->done && s->req_id == 0) {
            slot = s;
            slot->req_id = req_id;
            slot->done   = false;
            break; /* keep slot->mutex locked */
        }
        pthread_mutex_unlock(&s->mutex);
    }
    if (!slot) return ERR_OVERFLOW;

    /* Build and enqueue request */
    Message* req = malloc(sizeof(Message));
    if (!req) {
        pthread_mutex_unlock(&slot->mutex);
        return ERR_NOMEM;
    }
    snprintf(req->topic, MSG_BUS_MAX_TOPIC_LEN, "%s", topic);
    if (sender) snprintf(req->sender, MSG_BUS_MAX_SENDER_LEN, "%s", sender);
    req->msg_id       = req_id;
    req->type         = MSG_TYPE_REQUEST;
    req->timestamp_us = clock_now_monotonic_wall_us();
    req->data_size    = size;
    req->type_id      = 0;
    req->schema_version = 0;
    req->endian_marker  = 0;
    if (data && size > 0) memcpy(req->data, data, size);

    if (rb_push(shard_for(bus->shards, req_id), req) != 0) {
        free(req);
        slot->req_id = 0;
        pthread_mutex_unlock(&slot->mutex);
        return ERR_OVERFLOW;
    }

    /* Wait for reply (slot->mutex already held) */
    int ret = 0;
    if (!slot->done) {
        if (timeout_ms == 0) {
            pthread_cond_wait(&slot->cond, &slot->mutex);
        } else {
            struct timespec ts;
            clock_gettime(CLOCK_REALTIME, &ts);
            ts.tv_sec  += timeout_ms / 1000;
            ts.tv_nsec += (timeout_ms % 1000) * 1000000LL;
            if (ts.tv_nsec >= 1000000000LL) {
                ts.tv_sec++;
                ts.tv_nsec -= 1000000000LL;
            }
            int r = pthread_cond_timedwait(&slot->cond, &slot->mutex, &ts);
            if (r != 0 || !slot->done) ret = -1;
        }
    }
    if (ret == 0) *reply = slot->reply;
    slot->done   = false;
    slot->req_id = 0;
    pthread_mutex_unlock(&slot->mutex);
    return ret;
}

int message_bus_register_service(MessageBus* bus, const char* topic,
                                  ServiceHandler handler, void* user_data) {
    if (!bus || !topic || !handler) return ERR_OVERFLOW;
    pthread_mutex_lock(&bus->svc_mutex);
    /* Check for duplicate */
    for (int i = 0; i < bus->svc_count; i++) {
        if (bus->svcs[i].active && strcmp(bus->svcs[i].topic, topic) == 0) {
            pthread_mutex_unlock(&bus->svc_mutex);
            return ERR_OVERFLOW;
        }
    }
    if (bus->svc_count >= MSG_BUS_MAX_TOPICS) {
        pthread_mutex_unlock(&bus->svc_mutex);
        return ERR_OVERFLOW;
    }
    SvcEntry* e = &bus->svcs[bus->svc_count++];
    snprintf(e->topic, MSG_BUS_MAX_TOPIC_LEN, "%s", topic);
    e->handler   = handler;
    e->user_data = user_data;
    e->active    = true;
    pthread_mutex_unlock(&bus->svc_mutex);
    return 0;
}

int message_bus_unregister_service(MessageBus* bus, const char* topic) {
    if (!bus || !topic) return ERR_INVALID_PARAM;
    pthread_mutex_lock(&bus->svc_mutex);
    int found = -1;
    for (int i = 0; i < bus->svc_count; i++) {
        if (bus->svcs[i].active && strcmp(bus->svcs[i].topic, topic) == 0) {
            bus->svcs[i].active = false;
            found = 0;
            break;
        }
    }
    pthread_mutex_unlock(&bus->svc_mutex);
    return found;
}

/* ── Zero-Copy ───────────────────────────────────────── */

int message_bus_subscribe_zero_copy(MessageBus* bus, const char* topic,
                                     ZeroCopyCallback callback, void* user_data) {
    if (!bus || !topic || !callback) return ERR_INVALID_PARAM;
    pthread_mutex_lock(&bus->zc_mutex);
    if (bus->zc_sub_count >= MSG_BUS_MAX_SUBSCRIBERS) {
        pthread_mutex_unlock(&bus->zc_mutex);
        return ERR_OVERFLOW;
    }
    ZcSubEntry* e = &bus->zc_subs[bus->zc_sub_count++];
    snprintf(e->topic, MSG_BUS_MAX_TOPIC_LEN, "%s", topic);
    e->callback  = callback;
    e->user_data = user_data;
    e->active    = true;
    pthread_mutex_unlock(&bus->zc_mutex);
    return 0;
}

int message_bus_unsubscribe_zero_copy(MessageBus* bus, const char* topic,
                                       ZeroCopyCallback callback) {
    if (!bus || !topic || !callback) return ERR_INVALID_PARAM;
    pthread_mutex_lock(&bus->zc_mutex);
    int found = -1;
    for (int i = 0; i < bus->zc_sub_count; i++) {
        if (bus->zc_subs[i].active &&
            strcmp(bus->zc_subs[i].topic, topic) == 0 &&
            bus->zc_subs[i].callback == callback) {
            bus->zc_subs[i].active = false;
            found = 0;
            break;
        }
    }
    pthread_mutex_unlock(&bus->zc_mutex);
    return found;
}

int message_bus_publish_zero_copy(MessageBus* bus, const char* topic,
                                   const char* sender,
                                   const void* data, uint32_t data_size) {
    if (!bus || !topic) return ERR_INVALID_PARAM;

    uint32_t  msg_id = atomic_fetch_add(&bus->msg_id_counter, 1);
    uint64_t  ts     = clock_now_monotonic_wall_us();
    int       count  = 0;

    atomic_fetch_add(&bus->stat_zc_published, 1);

    /* Snapshot matching zero-copy subscribers under lock, then invoke
     * callbacks outside the lock — prevents deadlock if a callback tries
     * to subscribe/unsubscribe_zero_copy (same non-recursive mutex). */
    typedef struct { ZeroCopyCallback cb; void* ud; } ZcSnap;
    ZcSnap snap[MSG_BUS_MAX_SUBSCRIBERS];
    int snap_count = 0;

    pthread_mutex_lock(&bus->zc_mutex);
    for (int i = 0; i < bus->zc_sub_count; i++) {
        ZcSubEntry* s = &bus->zc_subs[i];
        if (!s->active) continue;
        if (!topic_match(s->topic, topic)) continue;
        snap[snap_count++] = (ZcSnap){ s->callback, s->user_data };
    }
    pthread_mutex_unlock(&bus->zc_mutex);

    for (int i = 0; i < snap_count; i++) {
        snap[i].cb(topic, sender ? sender : "", msg_id, ts, data, data_size, snap[i].ud);
        count++;
        atomic_fetch_add(&bus->stat_zc_delivered, 1);
    }

    /* Also push a copy-based message for regular subscribers */
    message_bus_publish(bus, topic, sender, data, data_size);

    return count;
}

/* ── Stats ───────────────────────────────────────────── */

void message_bus_get_stats(MessageBus* bus,
                           uint64_t* published_count,
                           uint64_t* delivered_count,
                           uint64_t* dropped_count) {
    if (!bus) return;
    if (published_count) *published_count = atomic_load(&bus->stat_published);
    if (delivered_count) *delivered_count = atomic_load(&bus->stat_delivered);
    if (dropped_count)   *dropped_count   = atomic_load(&bus->stat_dropped);
}

void message_bus_get_zc_stats(MessageBus* bus,
                               uint64_t* zc_published, uint64_t* zc_delivered) {
    if (!bus) return;
    if (zc_published) *zc_published = atomic_load(&bus->stat_zc_published);
    if (zc_delivered) *zc_delivered = atomic_load(&bus->stat_zc_delivered);
}

/* ══════════════════════════════════════════════════════════ */
/* QoS & Per-Topic Statistics                                */
/* ══════════════════════════════════════════════════════════ */

int message_bus_set_topic_qos(MessageBus* bus, const char* topic,
                              const TopicQos* qos) {
    if (!bus || !topic || !qos) return ERR_INVALID_PARAM;

    pthread_mutex_lock(&bus->topic_mutex);
    /* Find or create topic entry */
    for (int i = 0; i < atomic_load(&bus->topic_count); i++) {
        if (strcmp(bus->topic_entries[i].topic, topic) == 0) {
            bus->topic_entries[i].qos = *qos;
            atomic_store(&bus->topic_entries[i].depth_eff,
                         qos->depth > 0 ? qos->depth : MSG_BUS_QUEUE_SIZE);
            uint32_t qf = (qos->reliability == QOS_RELIABLE) ? QOS_FLAG_RELIABLE : 0u;
            if (qos->policy == QOS_DROP_LATEST) qf |= QOS_FLAG_DROP_LATEST;
            if (qos->policy == QOS_BLOCK)       qf |= QOS_FLAG_BLOCK;
            atomic_store(&bus->topic_entries[i].qos_flags, qf);
            if (qos->lifespan_ms > 0) atomic_store(&bus->has_lifespan_topics, true);
            pthread_mutex_unlock(&bus->topic_mutex);
            return 0;
        }
    }
    /* New topic：先在锁内完整初始化，再 release 递增 topic_count（理由同
     * publish 慢路径：发布热路径无锁读 topic_count，须避免读者见半初始化条目）。 */
    int tc = atomic_load(&bus->topic_count);
    if (tc < BUS_MAX_TOPIC_ENTRIES) {
        int idx = tc;
        memset(&bus->topic_entries[idx], 0, sizeof(bus->topic_entries[idx]));
        snprintf(bus->topic_entries[idx].topic, MSG_BUS_MAX_TOPIC_LEN, "%s", topic);
        bus->topic_entries[idx].active = true;
        bus->topic_entries[idx].qos = *qos;
        atomic_store(&bus->topic_entries[idx].depth_eff,
                     qos->depth > 0 ? qos->depth : MSG_BUS_QUEUE_SIZE);
        uint32_t qf = (qos->reliability == QOS_RELIABLE) ? QOS_FLAG_RELIABLE : 0u;
        if (qos->policy == QOS_DROP_LATEST) qf |= QOS_FLAG_DROP_LATEST;
        if (qos->policy == QOS_BLOCK)       qf |= QOS_FLAG_BLOCK;
        atomic_store(&bus->topic_entries[idx].qos_flags, qf);
        if (qos->lifespan_ms > 0) atomic_store(&bus->has_lifespan_topics, true);
        atomic_store_explicit(&bus->topic_count, tc + 1, memory_order_release);
        pthread_mutex_unlock(&bus->topic_mutex);
        return 0;
    }
    pthread_mutex_unlock(&bus->topic_mutex);
    return ERR_OVERFLOW;
}

const TopicQos* message_bus_get_topic_qos(MessageBus* bus, const char* topic) {
    if (!bus || !topic) return NULL;
    pthread_mutex_lock(&bus->topic_mutex);
    for (int i = 0; i < atomic_load(&bus->topic_count); i++) {
        if (strcmp(bus->topic_entries[i].topic, topic) == 0) {
            const TopicQos* q = &bus->topic_entries[i].qos;
            pthread_mutex_unlock(&bus->topic_mutex);
            return q;
        }
    }
    pthread_mutex_unlock(&bus->topic_mutex);
    return NULL;
}

/* 读时合并各无锁计数到对外 TopicStats（调用方须持 topic_mutex）。
 * 发布侧 pub_count/last_publish/freq/drop 与分发侧 deliver/latency 均为原子，
 * 此处汇总成单值。lat_ring_t[t] 由分发线程无锁写入，监控路径容忍轻微不一致。 */
static void merge_topic_stats(MessageBus* bus, int i, TopicStats* out) {
    typeof(bus->topic_entries[0])* e = &bus->topic_entries[i];
    *out = e->stats;                     /* 基础字段（subscriber_count 等） */
    out->qos = e->qos;
    snprintf(out->topic, MSG_BUS_MAX_TOPIC_LEN, "%s", e->topic);
    out->publish_count       = atomic_load(&e->pub_count);
    out->deliver_count       = atomic_load(&e->deliver_count);
    out->drop_count          = atomic_load(&e->drop_count);
    out->deadline_violations = atomic_load(&e->deadline_violations);
    out->total_latency_us    = atomic_load(&e->total_latency_us);
    out->min_latency_us      = atomic_load(&e->min_latency_us);
    out->max_latency_us      = atomic_load(&e->max_latency_us);
    out->last_publish_us     = atomic_load(&e->last_publish_us);
    out->frequency_hz        = (double)atomic_load(&e->freq_mhz) / 1000.0;

    /* p50/p99：合并各分发线程的延迟样本环 */
    LatencyRing combined;
    memset(&combined, 0, sizeof(combined));
    for (int t = 0; t < MSG_BUS_DISPATCH_THREADS; t++) {
        const LatencyRing* r = &e->lat_ring_t[t];
        uint32_t start = (r->head + BUS_LATENCY_RING_SIZE - r->count) % BUS_LATENCY_RING_SIZE;
        for (uint32_t k = 0; k < r->count; k++)
            lat_ring_push(&combined, r->samples[(start + k) % BUS_LATENCY_RING_SIZE]);
    }
    lat_ring_percentiles(&combined, &out->p50_latency_us, &out->p99_latency_us);
}

int message_bus_get_topic_stats(MessageBus* bus, const char* topic,
                                TopicStats* stats) {
    if (!bus || !topic || !stats) return ERR_INVALID_PARAM;
    pthread_mutex_lock(&bus->topic_mutex);
    for (int i = 0; i < atomic_load(&bus->topic_count); i++) {
        if (strcmp(bus->topic_entries[i].topic, topic) == 0) {
            merge_topic_stats(bus, i, stats);
            pthread_mutex_unlock(&bus->topic_mutex);
            return 0;
        }
    }
    pthread_mutex_unlock(&bus->topic_mutex);
    return ERR_NOT_FOUND;
}

int message_bus_topic_pending(MessageBus* bus, const char* topic) {
    if (!bus || !topic) return -1;
    pthread_mutex_lock(&bus->topic_mutex);
    for (int i = 0; i < atomic_load(&bus->topic_count); i++) {
        if (strcmp(bus->topic_entries[i].topic, topic) == 0) {
            int pending = (int)atomic_load(&bus->topic_entries[i].pending_count);
            pthread_mutex_unlock(&bus->topic_mutex);
            return pending;
        }
    }
    pthread_mutex_unlock(&bus->topic_mutex);
    return -1;  /* topic not found */
}

int message_bus_topic_is_full(MessageBus* bus, const char* topic) {
    if (!bus || !topic) return -1;
    pthread_mutex_lock(&bus->topic_mutex);
    for (int i = 0; i < atomic_load(&bus->topic_count); i++) {
        if (strcmp(bus->topic_entries[i].topic, topic) == 0) {
            uint32_t depth = bus->topic_entries[i].qos.depth > 0
                           ? bus->topic_entries[i].qos.depth
                           : MSG_BUS_QUEUE_SIZE;
            int full = atomic_load(&bus->topic_entries[i].pending_count) >= depth ? 1 : 0;
            pthread_mutex_unlock(&bus->topic_mutex);
            return full;
        }
    }
    pthread_mutex_unlock(&bus->topic_mutex);
    return -1;  /* topic not found */
}

/* ── 主动读取：按 topic 窥视最新一条消息（非阻塞，不消费） ──────
 * 2026-07-31：flowsim 的 select_for + transport 回调在 control/cmd 高频
 * （发布频率 >> 消费频率）下曾双双断流 15s，内置巡航接管导致追尾。
 * 本接口给高频关键指令一个不依赖被动唤醒的主动通道：
 *   - 扫描队列复制最后一条匹配 topic 的消息（depth=1 drop_oldest 下即最新）
 *   - 只读不消费：广播订阅回调仍由 dispatch 线程正常分发，不破坏语义
 *   - 消费者重复解析同一条消息是幂等的（指令值不变），用于刷新新鲜度
 * @return 0=取到, ERR_NOT_FOUND=队列中无该 topic 消息
 */
int message_bus_peek_latest(MessageBus* bus, const char* topic, Message* out) {
    if (!bus || !topic || !out) return ERR_INVALID_PARAM;
    /* 消息分片存储。跨全部分片找最后一条匹配 topic 的消息（按 msg_id 最大）。
     * 一次性锁住所有分片（按序 0..N-1，与分发线程各自只锁自己的分片不构成环，
     * 无死锁），在锁内完成扫描与拷贝——与所有分发线程的 rb_pop 互斥，杜绝
     * 分发线程并发 free 该消息导致的 UAF。peek 是低频兜底通道，全锁可接受。 */
    Message* latest = NULL;
    for (int s = 0; s < MSG_BUS_DISPATCH_THREADS; s++)
        pthread_mutex_lock(&bus->shards[s].mutex);

    for (int s = 0; s < MSG_BUS_DISPATCH_THREADS; s++) {
        RingBuffer* q = &bus->shards[s];
        for (uint32_t i = 0; i < q->count; i++) {
            uint32_t idx = (q->tail + i) % MSG_BUS_QUEUE_SIZE;
            Message* m   = q->msgs[idx];
            if (strcmp(m->topic, topic) == 0 &&
                (!latest || m->msg_id > latest->msg_id)) latest = m;
        }
    }
    int ret = ERR_NOT_FOUND;
    if (latest) { *out = *latest; ret = 0; }

    for (int s = MSG_BUS_DISPATCH_THREADS - 1; s >= 0; s--)
        pthread_mutex_unlock(&bus->shards[s].mutex);
    return ret;
}

int message_bus_list_topics(MessageBus* bus, char topics[][64], int max) {
    if (!bus || !topics || max <= 0) return ERR_INVALID_PARAM;
    pthread_mutex_lock(&bus->topic_mutex);
    int tc = atomic_load(&bus->topic_count);
    int n = (tc < max) ? tc : max;
    for (int i = 0; i < n; i++)
        snprintf(topics[i], 64, "%s", bus->topic_entries[i].topic);
    pthread_mutex_unlock(&bus->topic_mutex);
    return n;
}

int message_bus_get_all_topic_stats(MessageBus* bus, TopicStats* stats, int max) {
    if (!bus || !stats || max <= 0) return ERR_INVALID_PARAM;
    pthread_mutex_lock(&bus->topic_mutex);
    int tc = atomic_load(&bus->topic_count);
    int n = (tc < max) ? tc : max;
    for (int i = 0; i < n; i++)
        merge_topic_stats(bus, i, &stats[i]);
    pthread_mutex_unlock(&bus->topic_mutex);
    return n;
}

/* ══════════════════════════════════════════════════════════ */
/* Topic Remap                                                */
/* ══════════════════════════════════════════════════════════ */

int message_bus_add_remap(MessageBus* bus, const char* from, const char* to) {
    if (!bus || !from || !to) return ERR_INVALID_PARAM;
    pthread_mutex_lock(&bus->remap_mutex);

    /* Check for existing rule — update in place */
    for (int i = 0; i < bus->remap_count; i++) {
        if (bus->remaps[i].active &&
            strcmp(bus->remaps[i].from, from) == 0) {
            snprintf(bus->remaps[i].to, MSG_BUS_MAX_TOPIC_LEN, "%s", to);
            pthread_mutex_unlock(&bus->remap_mutex);
            return 0;
        }
    }

    if (bus->remap_count >= BUS_MAX_REMAPS) {
        pthread_mutex_unlock(&bus->remap_mutex);
        return ERR_OVERFLOW;
    }

    RemapEntry* e = &bus->remaps[bus->remap_count++];
    snprintf(e->from, MSG_BUS_MAX_TOPIC_LEN, "%s", from);
    snprintf(e->to,   MSG_BUS_MAX_TOPIC_LEN, "%s", to);
    e->active = true;
    atomic_store(&bus->remap_active, true);   /* 发布热路径据此决定是否取 remap_mutex */
    pthread_mutex_unlock(&bus->remap_mutex);
    return 0;
}

int message_bus_remove_remap(MessageBus* bus, const char* from) {
    if (!bus || !from) return ERR_INVALID_PARAM;
    pthread_mutex_lock(&bus->remap_mutex);
    for (int i = 0; i < bus->remap_count; i++) {
        if (bus->remaps[i].active &&
            strcmp(bus->remaps[i].from, from) == 0) {
            bus->remaps[i].active = false;
            /* 若无剩余 active 规则则关闭热路径标志，恢复零加锁发布 */
            bool any = false;
            for (int j = 0; j < bus->remap_count; j++)
                if (bus->remaps[j].active) { any = true; break; }
            atomic_store(&bus->remap_active, any);
            pthread_mutex_unlock(&bus->remap_mutex);
            return 0;
        }
    }
    pthread_mutex_unlock(&bus->remap_mutex);
    return ERR_NOT_FOUND;
}

void message_bus_resolve_topic(MessageBus* bus, const char* topic, char* out_topic) {
    if (!bus || !topic || !out_topic) return;
    pthread_mutex_lock(&bus->remap_mutex);
    for (int i = 0; i < bus->remap_count; i++) {
        if (bus->remaps[i].active &&
            strcmp(bus->remaps[i].from, topic) == 0) {
            snprintf(out_topic, MSG_BUS_MAX_TOPIC_LEN, "%s", bus->remaps[i].to);
            pthread_mutex_unlock(&bus->remap_mutex);
            return;
        }
    }
    pthread_mutex_unlock(&bus->remap_mutex);
    snprintf(out_topic, MSG_BUS_MAX_TOPIC_LEN, "%s", topic);
}
