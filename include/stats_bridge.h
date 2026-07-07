#ifndef STATS_BRIDGE_H
#define STATS_BRIDGE_H

/**
 * @file stats_bridge.h
 * @brief Cross-process MessageBus stats IPC bridge
 *
 * Enables flowmond to aggregate topic statistics from other processes
 * (e.g., flow_e2e) running on the same machine via a POSIX shared-memory
 * IPC channel.
 *
 * Architecture:
 *   flow_e2e  ── StatsPacket ──► IPC shm channel ──► flowmond ──► dashboard
 *
 * Publisher side (e.g., flow_e2e):
 *   IpcChannel* ch = stats_bridge_publisher_open();
 *   stats_bridge_publish(ch, g_bus, "flow_e2e");
 *   ipc_channel_close(ch);
 *
 * Subscriber side (e.g., flowmond):
 *   IpcChannel* ch = stats_bridge_subscriber_open(on_stats_pkt, user_data);
 *   if (ch) ipc_channel_start(ch);  // non-blocking background thread
 *   ...
 *   ipc_channel_close(ch);
 *
 * Notes:
 *   - The publisher must open the channel before the subscriber.
 *   - flowmond should retry stats_bridge_subscriber_open() until flow_e2e starts.
 *   - StatsPacket fits within MSG_BUS_MAX_DATA_SIZE (< 2 KB for 16 topics).
 */

#include "message_bus.h"
#include "ipc_channel.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ── Constants ──────────────────────────────────────────── */

/** Well-known IPC channel name for stats aggregation */
#define STATS_BRIDGE_CHANNEL     "flow_stats_bridge"

/** Topic name used inside the IPC Message envelope */
#define STATS_BRIDGE_TOPIC       "_stats"

/** Maximum topics included in one StatsPacket */
#define STATS_BRIDGE_MAX_TOPICS  16

/** IPC ring-buffer depth (packets) */
#define STATS_BRIDGE_QUEUE_DEPTH 8

/* ── Compact per-topic stats (IPC transport format) ─────── */

typedef struct {
    char     topic[64];         /**< Topic name */
    uint64_t publish_count;
    uint64_t deliver_count;
    uint64_t drop_count;
    uint64_t total_latency_us;
    double   frequency_hz;
    uint32_t subscriber_count;
    uint32_t reserved;          /**< Alignment padding */
} RemoteTopicStat;

/* ── Stats packet sent over IPC ─────────────────────────── */

typedef struct {
    char            source_name[64]; /**< Sending process name (e.g., "flow_e2e") */
    uint32_t        topic_count;     /**< Number of valid entries in topics[] */
    uint32_t        _reserved;
    uint64_t        bus_pub;         /**< Total published messages on that bus */
    uint64_t        bus_del;         /**< Total delivered messages */
    uint64_t        bus_drop;        /**< Total dropped messages */
    RemoteTopicStat topics[STATS_BRIDGE_MAX_TOPICS];
} StatsPacket;

/* ── API ─────────────────────────────────────────────────── */

/**
 * Open the IPC channel as publisher (creates shared memory).
 * Call once at startup in the business process.
 * @return IpcChannel pointer, or NULL on failure.
 */
IpcChannel* stats_bridge_publisher_open(void);

/**
 * Serialize bus stats into a StatsPacket and publish via IPC.
 * Safe to call from any thread. Silently skips if channel is full.
 * @param ch          Publisher channel from stats_bridge_publisher_open()
 * @param bus         The MessageBus whose stats to export
 * @param source_name Process identifier shown in the dashboard
 * @return 0 on success, -1 on failure
 */
int stats_bridge_publish(IpcChannel* ch, MessageBus* bus, const char* source_name);

/**
 * Open the IPC channel as subscriber with a callback.
 * Returns NULL if the publisher process has not started yet; caller should retry.
 * @param callback   Called for every received StatsPacket message
 * @param user_data  Passed through to callback
 * @return IpcChannel pointer, or NULL if channel not yet available.
 */
IpcChannel* stats_bridge_subscriber_open(MessageCallback callback, void* user_data);

#ifdef __cplusplus
}
#endif
#endif /* STATS_BRIDGE_H */
