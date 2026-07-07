#ifndef DASHBOARD_BRIDGE_H
#define DASHBOARD_BRIDGE_H

/**
 * @file dashboard_bridge.h
 * @brief Cross-process dashboard JSON IPC bridge
 *
 * Enables monitor_node to ship the full dashboard JSON (vehicle, scene,
 * sysmon, registry, etc.) to flowmond over a POSIX shared-memory IPC channel.
 *
 * The JSON may exceed MSG_BUS_MAX_DATA_SIZE (4096 bytes), so the bridge
 * transparently chunks the payload on the publisher side and reassembles
 * on the subscriber side.
 *
 * Architecture:
 *   monitor_node ── DashboardChunk[] ──► IPC shm ──► flowmond ──► /api/topology
 *
 * Publisher side (monitor_node):
 *   IpcChannel* ch = dashboard_bridge_publisher_open();
 *   dashboard_bridge_publish(ch, json_str, json_len);
 *
 * Subscriber side (flowmond):
 *   IpcChannel* ch = dashboard_bridge_subscriber_open(on_dashboard_json, user_data);
 *   if (ch) ipc_channel_start(ch);
 */

#include "message_bus.h"
#include "ipc_channel.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ── Constants ──────────────────────────────────────────── */

/** Well-known IPC channel name for dashboard JSON */
#define DASHBOARD_BRIDGE_CHANNEL     "flow_dashboard"

/** Topic name used inside the IPC Message envelope */
#define DASHBOARD_BRIDGE_TOPIC       "_dashboard"

/** IPC ring-buffer depth (chunks, not full snapshots) */
#define DASHBOARD_BRIDGE_QUEUE_DEPTH 8

/** Max bytes of payload data per chunk */
#define DASHBOARD_CHUNK_DATA_SIZE    (MSG_BUS_MAX_DATA_SIZE - 12)

/* ── Chunk protocol ────────────────────────────────────── */

/**
 * Each IPC Message carries one DashboardChunk as its payload.
 * seq=idx=count=0 indicates a single-chunk message (no reassembly needed).
 * count>1 means the receiver must wait for all chunks of the same seq.
 */
typedef struct {
    uint32_t seq;                        /**< Monotonic snapshot sequence number */
    uint16_t idx;                        /**< Chunk index (0-based) */
    uint16_t count;                      /**< Total chunks in this snapshot */
    char     data[DASHBOARD_CHUNK_DATA_SIZE]; /**< Partial JSON payload (NOT null-terminated) */
} DashboardChunk;

/* Compile-time check: one chunk fits in one IPC message */
typedef char dashboard_chunk_size_check[
    sizeof(DashboardChunk) <= MSG_BUS_MAX_DATA_SIZE ? 1 : -1];

/* ── Callback type for subscriber ──────────────────────── */

/**
 * Called when a complete dashboard JSON snapshot has been reassembled.
 * @param json      Null-terminated JSON string (valid only during callback)
 * @param len       Length of json (excluding null terminator)
 * @param user_data User pointer registered at subscription time
 */
typedef void (*DashboardJsonCallback)(const char* json, size_t len, void* user_data);

/* ── API ─────────────────────────────────────────────────── */

/**
 * Open the IPC channel as publisher (creates shared memory).
 * Call once at startup in monitor_node.
 * @return IpcChannel pointer, or NULL on failure.
 */
IpcChannel* dashboard_bridge_publisher_open(void);

/**
 * Publish a full dashboard JSON snapshot via IPC.
 * Transparently chunks if len > DASHBOARD_CHUNK_DATA_SIZE.
 * Safe to call from any thread. Silently drops if channel is full.
 * @param ch   Publisher channel from dashboard_bridge_publisher_open()
 * @param json The JSON string to publish
 * @param len  Length of json (must be > 0)
 * @return 0 on success, -1 on failure
 */
int dashboard_bridge_publish(IpcChannel* ch, const char* json, size_t len);

/**
 * Open the IPC channel as subscriber with a reassembly callback.
 * Returns NULL if the publisher process has not started yet; caller should retry.
 * @param callback   Called with each fully reassembled dashboard JSON
 * @param user_data  Passed through to callback
 * @return IpcChannel pointer, or NULL if channel not yet available.
 */
IpcChannel* dashboard_bridge_subscriber_open(DashboardJsonCallback callback,
                                              void* user_data);

#ifdef __cplusplus
}
#endif

#endif /* DASHBOARD_BRIDGE_H */
