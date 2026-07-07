/**
 * dashboard_bridge.c — Cross-process dashboard JSON IPC bridge
 *
 * Ships the full dashboard JSON (vehicle, scene, sysmon, registry, etc.)
 * from monitor_node to flowmond via a POSIX shared-memory IPC channel.
 *
 * Chunking protocol:
 *   - Single-chunk messages have seq=idx=count=0.
 *   - Multi-chunk messages set seq to a monotonic counter, count>1,
 *     and idx from 0 to count-1. The receiver waits for all chunks of
 *     the same seq before delivering the reassembled JSON to the callback.
 */

#include "dashboard_bridge.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/* ── Private reassembly state (per subscriber instance) ──── */

typedef struct {
    DashboardJsonCallback callback;
    void*                 user_data;

    /* Reassembly buffer (heap, grows as needed) */
    char*    buf;
    size_t   buf_cap;
    size_t   buf_len;

    /* Current snapshot being assembled */
    uint32_t current_seq;
    int      chunks_received;
    int      chunks_expected;
} BridgeSubState;

/* ── Publisher API ──────────────────────────────────────── */

IpcChannel* dashboard_bridge_publisher_open(void) {
    IpcChannel* ch = ipc_channel_open(DASHBOARD_BRIDGE_CHANNEL,
                                      IPC_ROLE_PUBLISHER,
                                      DASHBOARD_BRIDGE_QUEUE_DEPTH);
    if (!ch)
        fprintf(stderr, "[dashboard_bridge] failed to open publisher channel '%s'\n",
                DASHBOARD_BRIDGE_CHANNEL);
    return ch;
}

int dashboard_bridge_publish(IpcChannel* ch, const char* json, size_t len) {
    if (!ch || !json || len == 0) return -1;

    static uint32_t g_seq = 0;

    const size_t chunk_max = DASHBOARD_CHUNK_DATA_SIZE;
    uint16_t total_chunks = (uint16_t)((len + chunk_max - 1) / chunk_max);
    if (total_chunks == 0) total_chunks = 1;

    uint32_t seq;
    if (total_chunks > 1) {
        seq = ++g_seq;
    } else {
        seq = 0;  /* single-chunk: seq=0 signals no reassembly needed */
    }

    for (uint16_t idx = 0; idx < total_chunks; idx++) {
        DashboardChunk chunk;
        chunk.seq   = seq;
        chunk.idx   = idx;
        chunk.count = total_chunks;

        size_t offset = (size_t)idx * chunk_max;
        size_t remain = len - offset;
        size_t copy   = remain < chunk_max ? remain : chunk_max;

        if (copy > 0) {
            memcpy(chunk.data, json + offset, copy);
        }
        /* Zero-fill the remainder (safety) */
        if (copy < chunk_max) {
            memset(chunk.data + copy, 0, chunk_max - copy);
        }

        int ret = ipc_channel_publish(ch, DASHBOARD_BRIDGE_TOPIC,
                                      "monitor_node", &chunk, sizeof(chunk));
        if (ret != 0) return ret;
    }

    return 0;
}

/* ── Subscriber: IPC raw message → reassembly callback ── */

static void on_raw_chunk(const Message* msg, void* user_data) {
    if (!msg || msg->data_size < sizeof(DashboardChunk)) return;

    BridgeSubState* st = (BridgeSubState*)user_data;
    const DashboardChunk* chunk = (const DashboardChunk*)msg->data;

    uint32_t seq   = chunk->seq;
    uint16_t count = chunk->count;
    uint16_t idx   = chunk->idx;

    const size_t chunk_data_size = DASHBOARD_CHUNK_DATA_SIZE;
    size_t payload_len = chunk_data_size;
    /* Estimate the actual payload length (find first zero or use chunk_data_size) */
    {
        size_t i;
        for (i = 0; i < chunk_data_size; i++) {
            if (chunk->data[i] == '\0') break;
        }
        payload_len = i;
    }

    /* Single-chunk message: deliver immediately */
    if (count == 1 && seq == 0) {
        if (st->callback) {
            /* Pass a null-terminated copy */
            char tmp[4096];
            size_t copy_len = payload_len < sizeof(tmp) - 1 ? payload_len : sizeof(tmp) - 1;
            memcpy(tmp, chunk->data, copy_len);
            tmp[copy_len] = '\0';
            st->callback(tmp, copy_len, st->user_data);
        }
        return;
    }

    /* Multi-chunk: first chunk of a new sequence — allocate/reset buffer */
    if (idx == 0 || seq != st->current_seq) {
        st->current_seq    = seq;
        st->chunks_received = 0;
        st->chunks_expected = (int)count;
        st->buf_len = 0;

        /* Allocate or grow buffer for the full JSON */
        size_t needed = (size_t)count * chunk_data_size + 1;
        if (st->buf_cap < needed) {
            char* newbuf = (char*)realloc(st->buf, needed);
            if (!newbuf) return;  /* OOM — skip this snapshot */
            st->buf     = newbuf;
            st->buf_cap = needed;
        }
    }

    /* Discard if sequence changed (e.g., dropped a chunk) */
    if (seq != st->current_seq) return;

    /* Copy chunk data into buffer */
    size_t offset = (size_t)idx * chunk_data_size;
    if (offset + payload_len <= st->buf_cap) {
        memcpy(st->buf + offset, chunk->data, payload_len);
        st->chunks_received++;
    }

    /* If we have all chunks, null-terminate and deliver */
    if (st->chunks_received >= st->chunks_expected) {
        /* Calculate actual total length (last chunk may be shorter) */
        size_t total_len = st->buf_len;
        /* Find actual end: the total length is offset + payload_len from the last chunk */
        total_len = (size_t)(st->chunks_expected - 1) * chunk_data_size + payload_len;
        st->buf[total_len] = '\0';

        if (st->callback) {
            st->callback(st->buf, total_len, st->user_data);
        }

        /* Reset for next snapshot */
        st->current_seq = 0;
        st->chunks_received = 0;
        st->chunks_expected = 0;
        st->buf_len = 0;
    }
}

IpcChannel* dashboard_bridge_subscriber_open(DashboardJsonCallback callback,
                                              void* user_data) {
    BridgeSubState* st = (BridgeSubState*)calloc(1, sizeof(BridgeSubState));
    if (!st) return NULL;
    st->callback  = callback;
    st->user_data = user_data;

    IpcChannel* ch = ipc_channel_open(DASHBOARD_BRIDGE_CHANNEL,
                                      IPC_ROLE_SUBSCRIBER,
                                      DASHBOARD_BRIDGE_QUEUE_DEPTH);
    if (!ch) {
        free(st);
        return NULL;
    }

    if (ipc_channel_subscribe(ch, on_raw_chunk, st) != 0) {
        ipc_channel_close(ch);
        free(st);
        return NULL;
    }

    return ch;
}
