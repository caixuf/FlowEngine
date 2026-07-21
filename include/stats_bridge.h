#ifndef STATS_BRIDGE_H
#define STATS_BRIDGE_H

/**
 * @file stats_bridge.h
 * @brief Cross-process MessageBus stats IPC bridge
 *
 * Enables flowmond to aggregate topic statistics from other processes
 * (e.g., flow_launcher) running on the same machine via a POSIX shared-memory
 * IPC channel.
 *
 * Architecture:
 *   flow_launcher  ── StatsPacket ──► IPC shm channel ──► flowmond ──► dashboard
 *
 * Publisher side (e.g., flow_launcher):
 *   IpcChannel* ch = stats_bridge_publisher_open();
 *   stats_bridge_publish(ch, g_bus, "flow_launcher");
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
 *   - flowmond should retry stats_bridge_subscriber_open() until flow_launcher starts.
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

/** Maximum topics for monitor server (for compatibility) */
#define MONITOR_MAX_TOPICS       STATS_BRIDGE_MAX_TOPICS

/** Maximum nodes for monitor server */
#define MONITOR_MAX_NODES        8

/** IPC ring-buffer depth (packets) */
#define STATS_BRIDGE_QUEUE_DEPTH 8

/* ── Compact per-topic stats (IPC transport format) ─────── */

typedef struct {
    char     topic[64];         /**< Topic name */
    uint64_t publish_count;
    uint64_t deliver_count;
    uint64_t drop_count;
    uint64_t total_latency_us;
    uint64_t p50_latency_us;    /**< 50th percentile latency */
    uint64_t p99_latency_us;    /**< 99th percentile latency */
    uint64_t deadline_violations;
    double   frequency_hz;
    uint32_t subscriber_count;
    uint32_t reserved;          /**< Alignment padding */
    uint32_t pub_count;         /**< Alias for publish_count (monitor_server compatibility) */
    uint32_t del_count;         /**< Alias for deliver_count (monitor_server compatibility) */
    uint32_t avg_lat_us;        /**< Alias for p50_latency_us (monitor_server compatibility) */
    double   freq_hz;           /**< Alias for frequency_hz (monitor_server compatibility) */
    uint32_t sub_count;         /**< Alias for subscriber_count (monitor_server compatibility) */
    char     qos_profile[32];   /**< QoS reliability profile (monitor_server) */
    int32_t  deadline_ms;       /**< Deadline in ms (monitor_server) */
    char     transport[32];     /**< Transport type (monitor_server) */
    bool     active;            /**< Whether this entry is active (monitor_server) */
} RemoteTopicStat;

/* ── Monitor Server internal structures ──────────────────── */

typedef struct {
    char     topic[64];
    char     role[16];
    int      caps;
    uint32_t type_id;
    double   freq;
} LocalTopicInfo;

typedef struct {
    char     topic[64];
    char     role[16];
    int      caps;
    uint32_t type_id;
    double   freq;
} RemoteTopicInfo;

typedef struct {
    char            name[64];
    int             pid;
    int             caps;
    bool            active;
    int             topic_count;
    LocalTopicInfo  topics[MONITOR_MAX_TOPICS];
} LocalNode;

typedef struct {
    char             name[64];
    int              pid;
    int              caps;
    bool             active;
    int              topic_count;
    RemoteTopicInfo  topics[MONITOR_MAX_TOPICS];
} RemoteNode;

typedef struct {
    char     topic[64];
    uint32_t pub_count;
    uint32_t del_count;
    uint32_t drop_count;
    uint32_t avg_lat_us;
    double   freq_hz;
    int      sub_count;
    char     qos_profile[32];
    int      deadline_ms;
    char     transport[32];
    bool     active;
} LocalTopicStat;

typedef struct {
    uint32_t pub_total;
    uint32_t del_total;
    uint32_t drop_total;
    uint32_t avg_lat_us;
    uint32_t max_lat_us;
    uint32_t vehicle_speed;
    uint32_t vehicle_target;
    uint32_t vehicle_throttle;
    uint32_t vehicle_brake;
    uint32_t vehicle_x;
    uint32_t vehicle_error;
    char     driver_mode[32];
} LocalStats;

typedef struct {
    uint32_t pub_total;
    uint32_t del_total;
    uint32_t drop_total;
} RemoteStats;

typedef struct MonitorServer {
    char*             self_name;
    char              html_path[512];
    uint16_t          port;
    int               listen_fd;
    volatile bool     running;
    uint64_t          start_time_us;
    pthread_t         server_thread;
    pthread_mutex_t   local_mutex;
    pthread_mutex_t   remote_mutex;
    pthread_mutex_t   client_mutex;
    pthread_mutex_t   cached_mutex;
    pthread_cond_t    cached_cond;
    int               active_clients;
    char*             cached_json;
    uint64_t          cached_json_version;
    LocalStats        local_stats;
    RemoteStats       remote_stats;
    int               local_node_count;
    LocalNode         local_nodes[MONITOR_MAX_NODES];
    int               remote_node_count;
    RemoteNode        remote_nodes[MONITOR_MAX_NODES];
    int               local_topic_count;
    LocalTopicStat    local_topic_stats[MONITOR_MAX_TOPICS];
    int               remote_topic_count;
    RemoteTopicStat   remote_topic_stats[MONITOR_MAX_TOPICS];
} MonitorServer;

/* ── Stats packet sent over IPC ─────────────────────────── */

typedef struct {
    char            source_name[64]; /**< Sending process name (e.g., "flow_launcher") */
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
