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
#include <stdatomic.h>

/* ── Private reassembly state (per subscriber instance) ──── */

typedef struct {
    DashboardJsonCallback callback;
    void*                 user_data;

    /* Reassembly buffer (heap, grows as needed) */
    char*    buf;
    size_t   buf_cap;
    size_t   buf_len;

    /* Current snapshot being assembled.
     *
     * Reassembly completeness is tracked with a bitmap (received_mask) rather
     * than a counter, so that a missing middle chunk can never be masked by a
     * duplicate or out-of-order delivery: completion requires every expected
     * bit set, exactly. The maximum supported chunk count is 64 (≈4 MB JSON
     * at DASHBOARD_CHUNK_DATA_SIZE ≈ 65 KB/chunk, far beyond anything the
     * dashboard produces); larger counts are rejected as malformed.
     *
     * last_chunk_len records the payload_len of the idx == count-1 chunk so
     * the final total length can be computed even if chunks arrive out of
     * order — completion may be triggered by any chunk, not necessarily the
     * last one. */
    uint32_t current_seq;
    uint64_t received_mask;
    int      chunks_expected;
    size_t   last_chunk_len;
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

    static _Atomic uint32_t g_seq = 0;

    const size_t chunk_max = DASHBOARD_CHUNK_DATA_SIZE;
    uint16_t total_chunks = (uint16_t)((len + chunk_max - 1) / chunk_max);
    if (total_chunks == 0) total_chunks = 1;

    uint32_t seq;
    if (total_chunks > 1) {
        seq = atomic_fetch_add(&g_seq, 1u) + 1u;
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
            /* 动态分配：MSG_BUS_MAX_DATA_SIZE=65536 后 single-chunk 可能远大于
             * 旧的 4096，固定栈 buffer 会截断 payload。用 malloc 避免截断和栈溢出。 */
            char* tmp = (char*)malloc(payload_len + 1);
            if (tmp) {
                memcpy(tmp, chunk->data, payload_len);
                tmp[payload_len] = '\0';
                st->callback(tmp, payload_len, st->user_data);
                free(tmp);
            }
        }
        return;
    }

    /* Validate chunk header BEFORE any allocation. A count of 0, a count
     * larger than the bitmap width (64), or an idx outside [0,count) would
     * all be malformed — drop the chunk rather than risk a massive realloc
     * or size_t overflow. */
    if (count == 0 || count > 64 || idx >= count) return;

    /* Multi-chunk: first chunk of a new sequence — allocate/reset buffer.
     * Also reset if idx==0 of the same seq arrives again (publisher retry). */
    if (idx == 0 || seq != st->current_seq) {
        st->current_seq      = seq;
        st->received_mask    = 0;
        st->chunks_expected  = (int)count;
        st->last_chunk_len   = 0;
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

    /* Record the last chunk's actual payload length so we can compute the
     * true total length on completion regardless of arrival order. */
    if (idx == (uint16_t)count - 1) {
        st->last_chunk_len = payload_len;
    }

    /* Copy chunk data into buffer and mark its bit. Copying is idempotent:
     * a duplicate chunk simply overwrites the same region without affecting
     * the mask, so duplicates can never falsely trigger completion. */
    size_t offset = (size_t)idx * chunk_data_size;
    if (offset + payload_len <= st->buf_cap) {
        memcpy(st->buf + offset, chunk->data, payload_len);
        st->received_mask |= (1ULL << idx);
    }

    /* Complete iff every expected chunk bit is set. With the old counter
     * this would have been `received >= expected`, which a missing middle
     * chunk plus a later duplicate could satisfy with a gap in the data;
     * the bitmap makes that impossible. */
    uint64_t expected_mask = (count == 64) ? ~0ULL : ((1ULL << count) - 1);
    if (st->received_mask == expected_mask) {
        /* Total length = full chunks before the last + last chunk's actual
         * payload length (which may be shorter than chunk_data_size). */
        size_t total_len = (size_t)(st->chunks_expected - 1) * chunk_data_size
                           + st->last_chunk_len;
        if (total_len < st->buf_cap) {
            st->buf[total_len] = '\0';
        } else if (st->buf_cap > 0) {
            st->buf[st->buf_cap - 1] = '\0';
        }

        if (st->callback) {
            st->callback(st->buf, total_len, st->user_data);
        }

        /* Reset for next snapshot */
        st->current_seq     = 0;
        st->received_mask   = 0;
        st->chunks_expected = 0;
        st->last_chunk_len  = 0;
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
