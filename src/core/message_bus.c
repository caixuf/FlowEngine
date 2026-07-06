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
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <pthread.h>
#include <time.h>
#include <stdatomic.h>
#include <unistd.h>

/* ── Subscriber entry ─────────────────────────────────── */

typedef struct {
    char            topic[MSG_BUS_MAX_TOPIC_LEN];
    MessageCallback callback;
    void*           user_data;
    bool            active;
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
    Message    msgs[MSG_BUS_QUEUE_SIZE];
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

/* ── MessageBus ───────────────────────────────────────── */

struct MessageBus {
    char name[64];

    /* Pub/Sub subscribers */
    SubEntry   subs[MSG_BUS_MAX_SUBSCRIBERS];
    int        sub_count;
    pthread_mutex_t sub_mutex;

    /* Zero-copy subscribers */
    ZcSubEntry zc_subs[MSG_BUS_MAX_SUBSCRIBERS];
    int        zc_sub_count;
    pthread_mutex_t zc_mutex;

    /* Services */
    SvcEntry   svcs[MSG_BUS_MAX_TOPICS];
    int        svc_count;
    pthread_mutex_t svc_mutex;

    /* Async queue */
    RingBuffer queue;

    /* Dispatch thread */
    pthread_t  dispatch_thread;
    atomic_bool running;

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
        char      topic[MSG_BUS_MAX_TOPIC_LEN];
        TopicQos  qos;
        TopicStats stats;
        uint64_t  prev_publish_us;  /**< 上一次发布时间戳（用于频率估算） */
        uint32_t  pending_count;    /**< 当前在途（已入队未分发）消息数 */
        bool      active;
    } topic_entries[BUS_MAX_TOPIC_ENTRIES];
    int        topic_count;
    pthread_mutex_t topic_mutex;

    /* ── Topic remap table ─────────────────────────────── */
    RemapEntry remaps[BUS_MAX_REMAPS];
    int        remap_count;
    pthread_mutex_t remap_mutex;
};

/* ── Helpers ──────────────────────────────────────────── */

static uint64_t monotonic_us(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;
}

static bool topic_match(const char* pattern, const char* topic) {
    if (strcmp(pattern, "*") == 0) return true;
    return strcmp(pattern, topic) == 0;
}

/* ── Ring buffer ops ─────────────────────────────────── */

static void rb_init(RingBuffer* rb) {
    memset(rb, 0, sizeof(*rb));
    pthread_mutex_init(&rb->mutex, NULL);
    pthread_cond_init(&rb->not_empty, NULL);
    pthread_cond_init(&rb->not_full, NULL);
}

static void rb_destroy(RingBuffer* rb) {
    pthread_mutex_destroy(&rb->mutex);
    pthread_cond_destroy(&rb->not_empty);
    pthread_cond_destroy(&rb->not_full);
}

/* Returns 0 on success, -1 if full */
static int rb_push(RingBuffer* rb, const Message* msg) {
    pthread_mutex_lock(&rb->mutex);
    if (rb->count >= MSG_BUS_QUEUE_SIZE) {
        pthread_mutex_unlock(&rb->mutex);
        return ERR_OVERFLOW;
    }
    rb->msgs[rb->head] = *msg;
    rb->head = (rb->head + 1) % MSG_BUS_QUEUE_SIZE;
    rb->count++;
    pthread_cond_signal(&rb->not_empty);
    pthread_mutex_unlock(&rb->mutex);
    return 0;
}

/* Evict the oldest queued message matching `topic`.
 * Caller must NOT hold rb->mutex. Returns true if one was removed.
 * O(queue length) in the worst case; acceptable for the bounded 256-slot queue. */
static bool rb_evict_oldest_topic(RingBuffer* rb, const char* topic) {
    bool evicted = false;
    pthread_mutex_lock(&rb->mutex);
    /* Scan from oldest (tail) toward newest for the first topic match */
    for (uint32_t i = 0; i < rb->count; i++) {
        uint32_t idx = (rb->tail + i) % MSG_BUS_QUEUE_SIZE;
        if (strcmp(rb->msgs[idx].topic, topic) == 0) {
            /* Remove logical element i: shift older elements [0..i-1] up by
             * one slot, then advance tail. Preserves FIFO order. */
            for (uint32_t j = i; j > 0; j--) {
                uint32_t dst = (rb->tail + j) % MSG_BUS_QUEUE_SIZE;
                uint32_t src = (rb->tail + j - 1) % MSG_BUS_QUEUE_SIZE;
                rb->msgs[dst] = rb->msgs[src];
            }
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

/* Blocks until a message is available or bus stops */
static bool rb_pop(RingBuffer* rb, Message* out, atomic_bool* running) {
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
        return false;
    }
    *out = rb->msgs[rb->tail];
    rb->tail = (rb->tail + 1) % MSG_BUS_QUEUE_SIZE;
    rb->count--;
    pthread_mutex_unlock(&rb->mutex);
    return true;
}

/* ── Dispatch message to subscribers ─────────────────── */

static void dispatch_message(MessageBus* bus, const Message* msg) {
    int delivered = 0;

    /* ── Lifespan check: drop expired messages before dispatching ── */
    if (msg->type == MSG_TYPE_PUBLISH && msg->timestamp_us > 0) {
        pthread_mutex_lock(&bus->topic_mutex);
        for (int i = 0; i < bus->topic_count; i++) {
            if (strcmp(bus->topic_entries[i].topic, msg->topic) == 0) {
                TopicQos* q = &bus->topic_entries[i].qos;
                if (q->lifespan_ms > 0) {
                    uint64_t age_ms = (monotonic_us() - msg->timestamp_us) / 1000ULL;
                    if (age_ms > (uint64_t)q->lifespan_ms) {
                        bus->topic_entries[i].stats.drop_count++;
                        if (bus->topic_entries[i].pending_count > 0)
                            bus->topic_entries[i].pending_count--;
                        pthread_mutex_unlock(&bus->topic_mutex);
                        return; /* message expired — skip dispatch entirely */
                    }
                }
                break;
            }
        }
        pthread_mutex_unlock(&bus->topic_mutex);
    }

    pthread_mutex_lock(&bus->sub_mutex);
    for (int i = 0; i < bus->sub_count; i++) {
        SubEntry* s = &bus->subs[i];
        if (!s->active) continue;
        if (topic_match(s->topic, msg->topic)) {
            s->callback(msg, s->user_data);
            atomic_fetch_add(&bus->stat_delivered, 1);
            delivered++;
        }
    }
    pthread_mutex_unlock(&bus->sub_mutex);

    /* Per-topic tracking: decrement in-flight count + update delivery stats */
    if (msg->type == MSG_TYPE_PUBLISH) {
        pthread_mutex_lock(&bus->topic_mutex);
        for (int i = 0; i < bus->topic_count; i++) {
            if (strcmp(bus->topic_entries[i].topic, msg->topic) == 0) {
                /* Message left the queue — free one slot of in-flight budget */
                if (bus->topic_entries[i].pending_count > 0)
                    bus->topic_entries[i].pending_count--;

                if (delivered > 0) {
                    TopicStats* s = &bus->topic_entries[i].stats;
                    s->deliver_count += (uint64_t)delivered;
                    uint64_t now = monotonic_us();
                    uint64_t lat = now - msg->timestamp_us;
                    s->total_latency_us += lat;
                    if (s->min_latency_us == 0 || lat < s->min_latency_us) s->min_latency_us = lat;
                    if (lat > s->max_latency_us) s->max_latency_us = lat;

                    /* Deadline violation: end-to-end dispatch latency exceeded deadline_ms */
                    TopicQos* q = &bus->topic_entries[i].qos;
                    if (q->deadline_ms > 0 && lat > (uint64_t)q->deadline_ms * 1000ULL)
                        s->deadline_violations++;

                    /* Update subscriber count */
                    int subc = 0;
                    pthread_mutex_lock(&bus->sub_mutex);
                    for (int j = 0; j < bus->sub_count; j++)
                        if (bus->subs[j].active && topic_match(bus->subs[j].topic, msg->topic)) subc++;
                    pthread_mutex_unlock(&bus->sub_mutex);
                    s->subscriber_count = (uint32_t)subc;
                }
                break;
            }
        }
        pthread_mutex_unlock(&bus->topic_mutex);
    }

    /* Handle reply if this is a REPLY message */
    if (msg->type == MSG_TYPE_REPLY) {
        pthread_mutex_lock(&bus->reply_mutex);
        for (int i = 0; i < MAX_PENDING_REPLIES; i++) {
            ReplySlot* slot = &bus->reply_slots[i];
            if (slot->req_id == msg->msg_id && !slot->done) {
                pthread_mutex_lock(&slot->mutex);
                slot->reply = *msg;
                slot->done  = true;
                pthread_cond_signal(&slot->cond);
                pthread_mutex_unlock(&slot->mutex);
                break;
            }
        }
        pthread_mutex_unlock(&bus->reply_mutex);
    }
}

/* ── Dispatch thread ─────────────────────────────────── */

static void* dispatch_thread_fn(void* arg) {
    MessageBus* bus = (MessageBus*)arg;
    Message msg;

    while (atomic_load(&bus->running)) {
        /* Pass the live atomic flag (not a stale snapshot) so a shutdown
         * request during the blocking wait is observed immediately. */
        if (!rb_pop(&bus->queue, &msg, &bus->running)) continue;

        if (msg.type == MSG_TYPE_REQUEST) {
            /* Dispatch to service handler */
            pthread_mutex_lock(&bus->svc_mutex);
            SvcEntry* found = NULL;
            for (int i = 0; i < bus->svc_count; i++) {
                if (bus->svcs[i].active &&
                    strcmp(bus->svcs[i].topic, msg.topic) == 0) {
                    found = &bus->svcs[i];
                    break;
                }
            }
            if (found) {
                Message reply;
                memset(&reply, 0, sizeof(reply));
                snprintf(reply.topic, MSG_BUS_MAX_TOPIC_LEN, "%s", msg.topic);
                reply.msg_id    = msg.msg_id;
                reply.type      = MSG_TYPE_REPLY;
                reply.timestamp_us = monotonic_us();
                found->handler(&msg, &reply, found->user_data);
                pthread_mutex_unlock(&bus->svc_mutex);

                /* Deliver reply to waiting caller */
                pthread_mutex_lock(&bus->reply_mutex);
                for (int i = 0; i < MAX_PENDING_REPLIES; i++) {
                    ReplySlot* slot = &bus->reply_slots[i];
                    if (slot->req_id == msg.msg_id && !slot->done) {
                        pthread_mutex_lock(&slot->mutex);
                        slot->reply = reply;
                        slot->done  = true;
                        pthread_cond_signal(&slot->cond);
                        pthread_mutex_unlock(&slot->mutex);
                        break;
                    }
                }
                pthread_mutex_unlock(&bus->reply_mutex);
            } else {
                pthread_mutex_unlock(&bus->svc_mutex);
            }
        } else {
            dispatch_message(bus, &msg);
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

    rb_init(&bus->queue);
    pthread_mutex_init(&bus->sub_mutex, NULL);
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
    pthread_mutex_init(&bus->remap_mutex, NULL);

    atomic_store(&bus->running, true);
    if (pthread_create(&bus->dispatch_thread, NULL, dispatch_thread_fn, bus) != 0) {
        atomic_store(&bus->running, false);
        free(bus);
        return NULL;
    }
    return bus;
}

void message_bus_destroy(MessageBus* bus) {
    if (!bus) return;
    atomic_store(&bus->running, false);
    /* Wake the dispatch thread if it is blocked waiting for messages so it
     * observes the stop request promptly instead of after the wait timeout. */
    pthread_mutex_lock(&bus->queue.mutex);
    pthread_cond_broadcast(&bus->queue.not_empty);
    pthread_mutex_unlock(&bus->queue.mutex);
    pthread_join(bus->dispatch_thread, NULL);

    rb_destroy(&bus->queue);
    pthread_mutex_destroy(&bus->sub_mutex);
    pthread_mutex_destroy(&bus->zc_mutex);
    pthread_mutex_destroy(&bus->svc_mutex);
    pthread_mutex_destroy(&bus->reply_mutex);
    pthread_mutex_destroy(&bus->topic_mutex);
    pthread_mutex_destroy(&bus->remap_mutex);

    for (int i = 0; i < MAX_PENDING_REPLIES; i++) {
        pthread_mutex_destroy(&bus->reply_slots[i].mutex);
        pthread_cond_destroy(&bus->reply_slots[i].cond);
    }
    free(bus);
}

/* ── Pub/Sub ─────────────────────────────────────────── */

int message_bus_publish(MessageBus* bus, const char* topic, const char* sender,
                        const void* data, uint32_t size) {
    if (!bus || !topic) return ERR_INVALID_PARAM;
    if (size > MSG_BUS_MAX_DATA_SIZE) return ERR_OVERFLOW;

    /* ── Remap: resolve topic to its routing target ── */
    char resolved_topic[MSG_BUS_MAX_TOPIC_LEN];
    snprintf(resolved_topic, MSG_BUS_MAX_TOPIC_LEN, "%s", topic);
    pthread_mutex_lock(&bus->remap_mutex);
    for (int i = 0; i < bus->remap_count; i++) {
        if (bus->remaps[i].active &&
            strcmp(bus->remaps[i].from, topic) == 0) {
            snprintf(resolved_topic, MSG_BUS_MAX_TOPIC_LEN, "%s", bus->remaps[i].to);
            break;
        }
    }
    pthread_mutex_unlock(&bus->remap_mutex);

    Message msg;
    memset(&msg, 0, sizeof(msg));
    snprintf(msg.topic, MSG_BUS_MAX_TOPIC_LEN, "%s", resolved_topic);
    if (sender) snprintf(msg.sender, MSG_BUS_MAX_SENDER_LEN, "%s", sender);
    msg.msg_id       = atomic_fetch_add(&bus->msg_id_counter, 1);
    msg.type         = MSG_TYPE_PUBLISH;
    msg.timestamp_us = monotonic_us();
    msg.data_size    = size;
    if (data && size > 0) memcpy(msg.data, data, size);

    /* ── QoS: per-topic queue depth enforcement ── */
    pthread_mutex_lock(&bus->topic_mutex);
    int ti = -1;
    for (int i = 0; i < bus->topic_count; i++) {
        if (strcmp(bus->topic_entries[i].topic, resolved_topic) == 0) { ti = i; break; }
    }
    /* Auto-register topic if new */
    if (ti < 0 && bus->topic_count < BUS_MAX_TOPIC_ENTRIES) {
        ti = bus->topic_count++;
        memset(&bus->topic_entries[ti], 0, sizeof(bus->topic_entries[ti]));
        snprintf(bus->topic_entries[ti].topic, MSG_BUS_MAX_TOPIC_LEN, "%s", resolved_topic);
        bus->topic_entries[ti].active = true;
        bus->topic_entries[ti].qos.policy = QOS_DROP_OLDEST;
    }

    bool should_drop = false;
    bool need_evict  = false;
    if (ti >= 0) {
        TopicStats* s = &bus->topic_entries[ti].stats;
        TopicQos*   q = &bus->topic_entries[ti].qos;
        uint32_t depth = q->depth > 0 ? q->depth : MSG_BUS_QUEUE_SIZE;

        /* Lifespan expiry is checked at dispatch time (in dispatch_message),
         * where the message has had a chance to age in the queue.
         * Here at publish time the message age is always ~0. */

        /* Enforce per-topic depth using accurate in-flight (pending) count */
        if (bus->topic_entries[ti].pending_count >= depth) {
            /* QOS_RELIABLE overrides policy: always block to guarantee delivery */
            QosPolicy effective_policy = (q->reliability == QOS_RELIABLE)
                                         ? QOS_BLOCK
                                         : q->policy;
            switch (effective_policy) {
                case QOS_DROP_OLDEST:
                    /* Evict the oldest queued message of this topic (done below,
                     * outside topic_mutex, to preserve lock ordering) */
                    need_evict = true;
                    break;
                case QOS_DROP_LATEST:
                    should_drop = true;  /* Drop this new message */
                    s->drop_count++;
                    break;
                case QOS_BLOCK: {
                    /* Block until the topic has drained below depth (bounded).
                     * For QOS_RELIABLE: waits up to 5s (5000 x 1ms) before
                     * giving up — avoids deadlock if consumer is gone. */
                    int max_waits = (q->reliability == QOS_RELIABLE) ? 5000 : 1000;
                    int waits = 0;
                    while (bus->topic_entries[ti].pending_count >= depth && waits < max_waits) {
                        pthread_mutex_unlock(&bus->topic_mutex);
                        usleep(1000);  /* 1ms */
                        pthread_mutex_lock(&bus->topic_mutex);
                        waits++;
                    }
                    if (bus->topic_entries[ti].pending_count >= depth) {
                        should_drop = true;
                        s->drop_count++;
                    }
                    break;
                }
            }
        }
    }
    pthread_mutex_unlock(&bus->topic_mutex);

    if (should_drop) {
        atomic_fetch_add(&bus->stat_dropped, 1);
        return 0;  /* dropped per QoS policy, not a hard error */
    }

    /* DROP_OLDEST: make room by removing the oldest queued message of this topic */
    if (need_evict && rb_evict_oldest_topic(&bus->queue, resolved_topic)) {
        pthread_mutex_lock(&bus->topic_mutex);
        if (ti >= 0) {
            if (bus->topic_entries[ti].pending_count > 0)
                bus->topic_entries[ti].pending_count--;
            bus->topic_entries[ti].stats.drop_count++;
        }
        pthread_mutex_unlock(&bus->topic_mutex);
        atomic_fetch_add(&bus->stat_dropped, 1);
    }

    int ret = rb_push(&bus->queue, &msg);
    if (ret == 0) {
        atomic_fetch_add(&bus->stat_published, 1);
    } else {
        atomic_fetch_add(&bus->stat_dropped, 1);
    }

    /* Per-topic tracking */
    pthread_mutex_lock(&bus->topic_mutex);
    if (ti >= 0) {
        TopicStats* s = &bus->topic_entries[ti].stats;
        if (ret == 0) {
            bus->topic_entries[ti].pending_count++;
            /* Frequency: derive from inter-arrival gap (prev -> current) */
            uint64_t prev = bus->topic_entries[ti].prev_publish_us;
            bus->topic_entries[ti].prev_publish_us = msg.timestamp_us;
            s->publish_count++;
            s->last_publish_us = msg.timestamp_us;
            if (prev != 0) {
                uint64_t elapsed = msg.timestamp_us - prev;
                if (elapsed > 0) s->frequency_hz = 1000000.0 / (double)elapsed;
            }
        } else {
            s->drop_count++;
        }
    }
    pthread_mutex_unlock(&bus->topic_mutex);

    return ret;
}

int message_bus_subscribe(MessageBus* bus, const char* topic,
                          MessageCallback callback, void* user_data) {
    if (!bus || !topic || !callback) return ERR_INVALID_PARAM;

    pthread_mutex_lock(&bus->sub_mutex);

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
            pthread_mutex_unlock(&bus->sub_mutex);
            return ERR_OVERFLOW;
        }
        e = &bus->subs[bus->sub_count++];
    }

    snprintf(e->topic, MSG_BUS_MAX_TOPIC_LEN, "%s", topic);
    e->callback  = callback;
    e->user_data = user_data;
    e->active    = true;
    pthread_mutex_unlock(&bus->sub_mutex);
    return 0;
}

int message_bus_unsubscribe(MessageBus* bus, const char* topic, MessageCallback callback) {
    if (!bus || !topic || !callback) return ERR_INVALID_PARAM;
    int found = -1;
    pthread_mutex_lock(&bus->sub_mutex);
    for (int i = 0; i < bus->sub_count; i++) {
        if (bus->subs[i].active &&
            strcmp(bus->subs[i].topic, topic) == 0 &&
            bus->subs[i].callback == callback) {
            bus->subs[i].active = false;
            found = 0;
            break;
        }
    }
    pthread_mutex_unlock(&bus->sub_mutex);
    return found;
}

int message_bus_unsubscribe_ex(MessageBus* bus, const char* topic,
                               MessageCallback callback, void* user_data) {
    if (!bus || !topic || !callback) return ERR_INVALID_PARAM;
    int found = -1;
    pthread_mutex_lock(&bus->sub_mutex);
    for (int i = 0; i < bus->sub_count; i++) {
        SubEntry* s = &bus->subs[i];
        if (s->active &&
            strcmp(s->topic, topic) == 0 &&
            s->callback  == callback &&
            s->user_data == user_data) {
            s->active = false;
            found = 0;
            break;
        }
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
    pthread_mutex_lock(&bus->reply_mutex);
    ReplySlot* slot = NULL;
    for (int i = 0; i < MAX_PENDING_REPLIES; i++) {
        if (!bus->reply_slots[i].done && bus->reply_slots[i].req_id == 0) {
            slot = &bus->reply_slots[i];
            slot->req_id = req_id;
            slot->done   = false;
            break;
        }
    }
    pthread_mutex_unlock(&bus->reply_mutex);
    if (!slot) return ERR_OVERFLOW;

    /* Build and enqueue request */
    Message req;
    memset(&req, 0, sizeof(req));
    snprintf(req.topic, MSG_BUS_MAX_TOPIC_LEN, "%s", topic);
    if (sender) snprintf(req.sender, MSG_BUS_MAX_SENDER_LEN, "%s", sender);
    req.msg_id       = req_id;
    req.type         = MSG_TYPE_REQUEST;
    req.timestamp_us = monotonic_us();
    req.data_size    = size;
    if (data && size > 0) memcpy(req.data, data, size);

    if (rb_push(&bus->queue, &req) != 0) {
        pthread_mutex_lock(&bus->reply_mutex);
        slot->req_id = 0;
        pthread_mutex_unlock(&bus->reply_mutex);
        return ERR_OVERFLOW;
    }

    /* Wait for reply */
    pthread_mutex_lock(&slot->mutex);
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
    uint64_t  ts     = monotonic_us();
    int       count  = 0;

    atomic_fetch_add(&bus->stat_zc_published, 1);

    pthread_mutex_lock(&bus->zc_mutex);
    for (int i = 0; i < bus->zc_sub_count; i++) {
        ZcSubEntry* s = &bus->zc_subs[i];
        if (!s->active) continue;
        if (!topic_match(s->topic, topic)) continue;
        s->callback(topic, sender ? sender : "", msg_id, ts, data, data_size, s->user_data);
        count++;
        atomic_fetch_add(&bus->stat_zc_delivered, 1);
    }
    pthread_mutex_unlock(&bus->zc_mutex);

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
    for (int i = 0; i < bus->topic_count; i++) {
        if (strcmp(bus->topic_entries[i].topic, topic) == 0) {
            bus->topic_entries[i].qos = *qos;
            pthread_mutex_unlock(&bus->topic_mutex);
            return 0;
        }
    }
    /* New topic */
    if (bus->topic_count < BUS_MAX_TOPIC_ENTRIES) {
        int idx = bus->topic_count++;
        memset(&bus->topic_entries[idx], 0, sizeof(bus->topic_entries[idx]));
        snprintf(bus->topic_entries[idx].topic, MSG_BUS_MAX_TOPIC_LEN, "%s", topic);
        bus->topic_entries[idx].active = true;
        bus->topic_entries[idx].qos = *qos;
        pthread_mutex_unlock(&bus->topic_mutex);
        return 0;
    }
    pthread_mutex_unlock(&bus->topic_mutex);
    return ERR_OVERFLOW;
}

const TopicQos* message_bus_get_topic_qos(MessageBus* bus, const char* topic) {
    if (!bus || !topic) return NULL;
    pthread_mutex_lock(&bus->topic_mutex);
    for (int i = 0; i < bus->topic_count; i++) {
        if (strcmp(bus->topic_entries[i].topic, topic) == 0) {
            const TopicQos* q = &bus->topic_entries[i].qos;
            pthread_mutex_unlock(&bus->topic_mutex);
            return q;
        }
    }
    pthread_mutex_unlock(&bus->topic_mutex);
    return NULL;
}

int message_bus_get_topic_stats(MessageBus* bus, const char* topic,
                                TopicStats* stats) {
    if (!bus || !topic || !stats) return ERR_INVALID_PARAM;
    pthread_mutex_lock(&bus->topic_mutex);
    for (int i = 0; i < bus->topic_count; i++) {
        if (strcmp(bus->topic_entries[i].topic, topic) == 0) {
            *stats = bus->topic_entries[i].stats;
            stats->qos = bus->topic_entries[i].qos;
            snprintf(stats->topic, MSG_BUS_MAX_TOPIC_LEN, "%s", topic);
            pthread_mutex_unlock(&bus->topic_mutex);
            return 0;
        }
    }
    pthread_mutex_unlock(&bus->topic_mutex);
    return ERR_NOT_FOUND;
}

int message_bus_list_topics(MessageBus* bus, char topics[][64], int max) {
    if (!bus || !topics || max <= 0) return ERR_INVALID_PARAM;
    pthread_mutex_lock(&bus->topic_mutex);
    int n = (bus->topic_count < max) ? bus->topic_count : max;
    for (int i = 0; i < n; i++)
        snprintf(topics[i], 64, "%s", bus->topic_entries[i].topic);
    pthread_mutex_unlock(&bus->topic_mutex);
    return n;
}

int message_bus_get_all_topic_stats(MessageBus* bus, TopicStats* stats, int max) {
    if (!bus || !stats || max <= 0) return ERR_INVALID_PARAM;
    pthread_mutex_lock(&bus->topic_mutex);
    int n = (bus->topic_count < max) ? bus->topic_count : max;
    for (int i = 0; i < n; i++) {
        stats[i] = bus->topic_entries[i].stats;
        stats[i].qos = bus->topic_entries[i].qos;
        snprintf(stats[i].topic, MSG_BUS_MAX_TOPIC_LEN, "%s", bus->topic_entries[i].topic);
    }
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
