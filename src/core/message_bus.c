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
    bool       running;

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

/* Blocks until a message is available or bus stops */
static bool rb_pop(RingBuffer* rb, Message* out, volatile bool* running) {
    pthread_mutex_lock(&rb->mutex);
    while (rb->count == 0 && *running) {
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
    pthread_mutex_lock(&bus->sub_mutex);
    for (int i = 0; i < bus->sub_count; i++) {
        SubEntry* s = &bus->subs[i];
        if (!s->active) continue;
        if (topic_match(s->topic, msg->topic)) {
            s->callback(msg, s->user_data);
            atomic_fetch_add(&bus->stat_delivered, 1);
        }
    }
    pthread_mutex_unlock(&bus->sub_mutex);

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

    while (bus->running) {
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

    bus->running = true;
    if (pthread_create(&bus->dispatch_thread, NULL, dispatch_thread_fn, bus) != 0) {
        bus->running = false;
        free(bus);
        return NULL;
    }
    return bus;
}

void message_bus_destroy(MessageBus* bus) {
    if (!bus) return;
    bus->running = false;
    pthread_join(bus->dispatch_thread, NULL);

    rb_destroy(&bus->queue);
    pthread_mutex_destroy(&bus->sub_mutex);
    pthread_mutex_destroy(&bus->zc_mutex);
    pthread_mutex_destroy(&bus->svc_mutex);
    pthread_mutex_destroy(&bus->reply_mutex);

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

    Message msg;
    memset(&msg, 0, sizeof(msg));
    snprintf(msg.topic, MSG_BUS_MAX_TOPIC_LEN, "%s", topic);
    if (sender) snprintf(msg.sender, MSG_BUS_MAX_SENDER_LEN, "%s", sender);
    msg.msg_id       = atomic_fetch_add(&bus->msg_id_counter, 1);
    msg.type         = MSG_TYPE_PUBLISH;
    msg.timestamp_us = monotonic_us();
    msg.data_size    = size;
    if (data && size > 0) memcpy(msg.data, data, size);

    int ret = rb_push(&bus->queue, &msg);
    if (ret == 0) {
        atomic_fetch_add(&bus->stat_published, 1);
    } else {
        atomic_fetch_add(&bus->stat_dropped, 1);
    }
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
