/**
 * stats_bridge.c — Cross-process MessageBus stats IPC bridge
 *
 * Serializes MessageBus TopicStats into compact StatsPacket structs and
 * ships them over an IPC shared-memory channel so that flowmond can
 * aggregate statistics from other running processes without sharing a bus.
 */

#include "stats_bridge.h"
#include <string.h>
#include <stdio.h>

/* Compile-time guard: StatsPacket must fit in a single IPC Message payload */
typedef char _stats_packet_size_check[
    sizeof(StatsPacket) <= MSG_BUS_MAX_DATA_SIZE ? 1 : -1];

/* ── Public API ─────────────────────────────────────────── */

IpcChannel* stats_bridge_publisher_open(void) {
    IpcChannel* ch = ipc_channel_open(STATS_BRIDGE_CHANNEL,
                                      IPC_ROLE_PUBLISHER,
                                      STATS_BRIDGE_QUEUE_DEPTH);
    if (!ch)
        fprintf(stderr, "[stats_bridge] failed to open publisher channel '%s'\n",
                STATS_BRIDGE_CHANNEL);
    return ch;
}

int stats_bridge_publish(IpcChannel* ch, MessageBus* bus, const char* source_name) {
    if (!ch || !bus || !source_name) return -1;

    StatsPacket pkt;
    memset(&pkt, 0, sizeof(pkt));
    snprintf(pkt.source_name, sizeof(pkt.source_name), "%s", source_name);

    message_bus_get_stats(bus, &pkt.bus_pub, &pkt.bus_del, &pkt.bus_drop);

    TopicStats tstats[STATS_BRIDGE_MAX_TOPICS];
    int nt = message_bus_get_all_topic_stats(bus, tstats, STATS_BRIDGE_MAX_TOPICS);
    pkt.topic_count = (uint32_t)nt;

    for (int i = 0; i < nt; i++) {
        snprintf(pkt.topics[i].topic, sizeof(pkt.topics[i].topic),
                 "%s", tstats[i].topic);
        pkt.topics[i].publish_count    = tstats[i].publish_count;
        pkt.topics[i].deliver_count    = tstats[i].deliver_count;
        pkt.topics[i].drop_count       = tstats[i].drop_count;
        pkt.topics[i].total_latency_us = tstats[i].total_latency_us;
        pkt.topics[i].frequency_hz     = tstats[i].frequency_hz;
        pkt.topics[i].subscriber_count = tstats[i].subscriber_count;
    }

    return ipc_channel_publish(ch, STATS_BRIDGE_TOPIC, source_name,
                               &pkt, sizeof(pkt));
}

IpcChannel* stats_bridge_subscriber_open(MessageCallback callback, void* user_data) {
    IpcChannel* ch = ipc_channel_open(STATS_BRIDGE_CHANNEL,
                                      IPC_ROLE_SUBSCRIBER,
                                      STATS_BRIDGE_QUEUE_DEPTH);
    if (!ch) return NULL;  /* publisher not started yet — caller should retry */

    if (ipc_channel_subscribe(ch, callback, user_data) != 0) {
        ipc_channel_close(ch);
        return NULL;
    }
    return ch;
}
