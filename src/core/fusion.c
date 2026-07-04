/**
 * fusion.c — 多传感器数据融合框架实现 (FlowEngine Phase 4)
 */

#include "fusion.h"
#include "error_codes.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <time.h>

/* ── 单调时钟 ────────────────────────────────────────────── */

static uint64_t monotonic_us(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;
}

/* ══════════════════════════════════════════════════════════ */
/* MessageBuffer                                              */
/* ══════════════════════════════════════════════════════════ */

MessageBuffer* message_buffer_create(const char* topic, uint32_t type_id,
                                     uint32_t capacity, uint64_t window_us) {
    if (!topic || capacity == 0) return NULL;

    MessageBuffer* mb = (MessageBuffer*)calloc(1, sizeof(MessageBuffer));
    if (!mb) return NULL;

    snprintf(mb->topic, MSG_BUS_MAX_TOPIC_LEN, "%s", topic);
    mb->type_id   = type_id;
    mb->capacity  = capacity;
    mb->window_us = window_us;
    mb->buffer    = (Message*)calloc(capacity, sizeof(Message));
    if (!mb->buffer) { free(mb); return NULL; }

    pthread_mutex_init(&mb->mutex, NULL);
    return mb;
}

int message_buffer_push(MessageBuffer* mb, const Message* msg) {
    if (!mb || !msg) return ERR_INVALID_PARAM;

    pthread_mutex_lock(&mb->mutex);

    /* Store at head position (circular) */
    mb->buffer[mb->head] = *msg;
    mb->head = (mb->head + 1) % mb->capacity;
    if (mb->count < mb->capacity) mb->count++;

    /* Evict messages older than window_us */
    uint64_t cutoff = monotonic_us() - mb->window_us;
    for (uint32_t i = 0; i < mb->count; i++) {
        uint32_t idx = (mb->head + mb->capacity - mb->count + i) % mb->capacity;
        if (mb->buffer[idx].timestamp_us < cutoff) {
            /* This is the oldest — but we don't compact in the simple case.
             * The find_nearest will skip messages outside the window. */
        }
    }

    pthread_mutex_unlock(&mb->mutex);
    return 0;
}

const Message* message_buffer_find_nearest(MessageBuffer* mb, uint64_t target_us,
                                           uint64_t max_delta_us) {
    if (!mb) return NULL;

    pthread_mutex_lock(&mb->mutex);

    const Message* best = NULL;
    uint64_t best_delta = UINT64_MAX;

    for (uint32_t i = 0; i < mb->count; i++) {
        uint32_t idx = (mb->head + mb->capacity - mb->count + i) % mb->capacity;
        const Message* m = &mb->buffer[idx];

        /* Check window */
        uint64_t now = monotonic_us();
        if (now - m->timestamp_us > mb->window_us) continue;

        /* Find closest */
        uint64_t delta = (target_us > m->timestamp_us)
                         ? (target_us - m->timestamp_us)
                         : (m->timestamp_us - target_us);
        if (delta <= max_delta_us && delta < best_delta) {
            best_delta = delta;
            best = m;
        }
    }

    pthread_mutex_unlock(&mb->mutex);
    return best;
}

const Message* message_buffer_latest(MessageBuffer* mb) {
    if (!mb || mb->count == 0) return NULL;

    pthread_mutex_lock(&mb->mutex);
    uint32_t idx = (mb->head + mb->capacity - 1) % mb->capacity;
    const Message* m = &mb->buffer[idx];
    pthread_mutex_unlock(&mb->mutex);
    return m;
}

void message_buffer_destroy(MessageBuffer* mb) {
    if (!mb) return;
    pthread_mutex_destroy(&mb->mutex);
    free(mb->buffer);
    free(mb);
}

/* ══════════════════════════════════════════════════════════ */
/* FusionNode C API                                           */
/* ══════════════════════════════════════════════════════════ */

struct FusionNode {
    char          name[64];
    MessageBus*   bus;
    FusionPolicy  policy;

    /* Input buffers */
    MessageBuffer* buffers[FUSION_MAX_INPUTS];
    int            buffer_count;

    /* Output */
    char          output_topic[MSG_BUS_MAX_TOPIC_LEN];
    uint32_t      output_type_id;

    /* Callback */
    FusionCallback callback;
    void*          cb_user_data;

    /* Thread */
    pthread_t     worker_thread;
    bool          running;
    pthread_mutex_t mutex;
};

FusionNode* fusion_node_create(const char* name, MessageBus* bus,
                               const FusionPolicy* policy) {
    FusionNode* fn = (FusionNode*)calloc(1, sizeof(FusionNode));
    if (!fn) return NULL;

    if (name) snprintf(fn->name, sizeof(fn->name), "%s", name);
    fn->bus = bus;
    if (policy) fn->policy = *policy;
    else        fn->policy = (FusionPolicy)FUSION_POLICY_TIME_ALIGNED;

    pthread_mutex_init(&fn->mutex, NULL);
    return fn;
}

int fusion_node_add_input(FusionNode* fn, const char* topic,
                          uint32_t type_id, uint32_t buffer_capacity) {
    if (!fn || !topic || fn->buffer_count >= FUSION_MAX_INPUTS) return ERR_OVERFLOW;

    if (buffer_capacity == 0) buffer_capacity = 64;

    MessageBuffer* mb = message_buffer_create(topic, type_id,
                                              buffer_capacity,
                                              FUSION_DEFAULT_WINDOW_US);
    if (!mb) return ERR_INVALID_PARAM;

    pthread_mutex_lock(&fn->mutex);
    fn->buffers[fn->buffer_count++] = mb;
    pthread_mutex_unlock(&fn->mutex);

    return 0;
}

int fusion_node_set_output(FusionNode* fn, const char* topic, uint32_t type_id) {
    if (!fn || !topic) return ERR_INVALID_PARAM;
    snprintf(fn->output_topic, MSG_BUS_MAX_TOPIC_LEN, "%s", topic);
    fn->output_type_id = type_id;
    return 0;
}

void fusion_node_set_callback(FusionNode* fn, FusionCallback cb, void* user_data) {
    if (!fn) return;
    fn->callback      = cb;
    fn->cb_user_data  = user_data;
}

/* ── Time alignment algorithm ────────────────────────────── */

/**
 * Build a SyncedFrame from the current state of all input buffers.
 * Uses the latest arriving message's timestamp as reference.
 * Returns true if at least one input has valid data.
 */
static bool build_synced_frame(FusionNode* fn, SyncedFrame* frame) {
    memset(frame, 0, sizeof(*frame));

    /* Find the latest timestamp across all buffers as reference */
    uint64_t latest_ts = 0;
    for (int i = 0; i < fn->buffer_count; i++) {
        const Message* m = message_buffer_latest(fn->buffers[i]);
        if (m && m->timestamp_us > latest_ts) latest_ts = m->timestamp_us;
    }
    if (latest_ts == 0) return false;

    frame->reference_ts = latest_ts;

    /* For each input, find the nearest message in time */
    bool any_valid = false;
    uint64_t max_delta = fn->policy.max_timestamp_delta_us > 0
                         ? fn->policy.max_timestamp_delta_us
                         : FUSION_DEFAULT_MAX_DELTA_US;

    for (int i = 0; i < fn->buffer_count; i++) {
        const Message* m = message_buffer_find_nearest(
            fn->buffers[i], latest_ts, max_delta);
        if (m) {
            frame->inputs[i]       = *m;
            frame->input_valid[i]  = true;
            frame->input_timestamps[i] = m->timestamp_us;
            frame->input_deltas_us[i]  = (double)((int64_t)m->timestamp_us - (int64_t)latest_ts);
            any_valid = true;
        } else {
            frame->input_valid[i] = false;
        }
    }

    frame->input_count = (uint32_t)fn->buffer_count;
    return any_valid;
}

/* ── Bus subscription callback ───────────────────────────── */

typedef struct {
    FusionNode* fn;
    int         buffer_idx;
} FusionSubCtx;

static void fusion_on_message(const Message* msg, void* user_data) {
    FusionSubCtx* ctx = (FusionSubCtx*)user_data;
    FusionNode* fn = ctx->fn;
    if (!fn->running) return;

    message_buffer_push(fn->buffers[ctx->buffer_idx], msg);

    /* Try to build a synced frame */
    SyncedFrame frame;
    if (build_synced_frame(fn, &frame)) {
        /* Call user callback */
        if (fn->callback) {
            fn->callback(&frame, fn->bus, fn->output_topic,
                         fn->output_type_id, fn->cb_user_data);
        }
    }
}

/* ── Worker thread ───────────────────────────────────────── */

static void* fusion_worker_fn(void* arg) {
    FusionNode* fn = (FusionNode*)arg;

    /* Subscribe to all input topics */
    FusionSubCtx* ctxs = (FusionSubCtx*)calloc((size_t)fn->buffer_count, sizeof(FusionSubCtx));
    for (int i = 0; i < fn->buffer_count; i++) {
        ctxs[i].fn         = fn;
        ctxs[i].buffer_idx = i;
        message_bus_subscribe(fn->bus, fn->buffers[i]->topic,
                              fusion_on_message, &ctxs[i]);
    }

    printf("[fusion:%s] started with %d inputs, output=%s, strategy=%d\n",
           fn->name, fn->buffer_count,
           fn->output_topic[0] ? fn->output_topic : "(none)",
           fn->policy.strategy);

    /* Spin — messages are processed in callbacks */
    while (fn->running) {
        usleep(100000); /* 100ms wake-up for stop check */
    }

    /* Cleanup subscriptions */
    for (int i = 0; i < fn->buffer_count; i++) {
        message_bus_unsubscribe(fn->bus, fn->buffers[i]->topic,
                                fusion_on_message);
    }
    free(ctxs);

    return NULL;
}

int fusion_node_start(FusionNode* fn) {
    if (!fn || fn->running) return ERR_INVALID_PARAM;

    fn->running = true;
    pthread_create(&fn->worker_thread, NULL, fusion_worker_fn, fn);
    return 0;
}

void fusion_node_stop(FusionNode* fn) {
    if (!fn || !fn->running) return;
    fn->running = false;
    pthread_join(fn->worker_thread, NULL);
    printf("[fusion:%s] stopped\n", fn->name);
}

void fusion_node_destroy(FusionNode* fn) {
    if (!fn) return;
    if (fn->running) fusion_node_stop(fn);
    for (int i = 0; i < fn->buffer_count; i++) {
        message_buffer_destroy(fn->buffers[i]);
    }
    pthread_mutex_destroy(&fn->mutex);
    free(fn);
}
