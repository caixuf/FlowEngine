/**
 * topic_bridge.c — 跨进程 Topic 桥接实现
 *
 * 复用 ipc_channel (SHM + semaphore) 作传输层，把本进程 MessageBus 上的
 * 指定 topic 单向/双向镜像到另一个进程的 MessageBus。
 *
 * PUB 方向: 本地 bus 订阅 → ipc_channel_publish → 对端进程
 * SUB 方向: ipc_channel recv 线程 → message_bus_publish → 本地 bus
 * BIDIR:    以 "bridge:" 前缀标记转发消息, 防止回环重发
 */

#include "topic_bridge.h"
#include "ipc_channel.h"
#include "error_codes.h"
#include "logger.h"
#include "clock_service.h"

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <pthread.h>
#include <time.h>

/* ── 单 topic 转发记录 ─────────────────────────────────────── */

typedef struct {
    char     topic[64];
    uint32_t type_id;
    TopicBridgeStats stats;
} BridgeEntry;

/* ── TopicBridge 实体 ──────────────────────────────────────── */

struct TopicBridge {
    MessageBus*          bus;
    char                 channel_name[TOPIC_BRIDGE_CHANNEL_LEN];
    TopicBridgeDirection direction;

    BridgeEntry entries[TOPIC_BRIDGE_MAX_TOPICS];
    int         n_topics;

    /* PUB 方向: 为每个 topic 分配的回调上下文, 由 destroy 统一释放。
     * 类型为 void* 以避免前向声明依赖; 实际指向 PubCallbackCtx。 */
    void* pub_ctx[TOPIC_BRIDGE_MAX_TOPICS];

    IpcChannel*  ipc;
    volatile int running;
    volatile int should_stop;
};

/* ── PUB 方向: 本地 bus 回调 → IPC 转发 ─────────────────────
 * user_data 指向对应的 BridgeEntry（含所属 bridge 引用）。       */

typedef struct {
    TopicBridge* bridge;
    int          entry_idx;
} PubCallbackCtx;

static void pub_on_topic(const Message* msg, void* user_data) {
    PubCallbackCtx* ctx = (PubCallbackCtx*)user_data;
    TopicBridge*  bridge = ctx->bridge;
    BridgeEntry*  entry  = &bridge->entries[ctx->entry_idx];

    if (!bridge->running || bridge->should_stop) return;

    /* BIDIR 防回环: 跳过来自桥接自身的消息 */
    if (bridge->direction == TOPIC_BRIDGE_BIDIR &&
        msg->sender[0] != '\0' &&
        strncmp(msg->sender, "bridge:", 7) == 0) {
        return;
    }

    int rc = ipc_channel_publish(bridge->ipc,
                                  msg->topic, msg->sender,
                                  msg->data, msg->data_size);
    if (rc == 0) {
        entry->stats.forwarded++;
        entry->stats.bytes        += msg->data_size;
        entry->stats.last_ts_us    = clock_now_us();
    } else {
        entry->stats.dropped++;
        LOG_WARN("topic_bridge", "[PUB] drop on '%s' (ipc full)", msg->topic);
    }
}

/* ── SUB 方向: IPC 回调 → 本地 bus 发布 ─────────────────────  */

static void sub_on_ipc(const Message* msg, void* user_data) {
    TopicBridge* bridge = (TopicBridge*)user_data;
    if (!bridge->running || bridge->should_stop) return;

    /* 找对应 entry */
    BridgeEntry* entry = NULL;
    for (int i = 0; i < bridge->n_topics; i++) {
        if (strcmp(bridge->entries[i].topic, msg->topic) == 0) {
            entry = &bridge->entries[i];
            break;
        }
    }
    if (!entry) return;  /* 收到未注册的 topic，忽略 */

    /* BIDIR 防回环: 用 "bridge:" 前缀作为 sender 注入本地 bus */
    char sender[MSG_BUS_MAX_SENDER_LEN];
    snprintf(sender, sizeof(sender), "bridge:%s", msg->sender);

    int rc = message_bus_publish(bridge->bus,
                                  msg->topic, sender,
                                  msg->data, msg->data_size);
    if (rc == 0) {
        entry->stats.forwarded++;
        entry->stats.bytes        += msg->data_size;
        entry->stats.last_ts_us    = clock_now_us();
    } else {
        entry->stats.dropped++;
        LOG_WARN("topic_bridge", "[SUB] publish failed on '%s'", msg->topic);
    }
}

/* ══════════════════════════════════════════════════════════ */
/* 生命周期                                                    */
/* ══════════════════════════════════════════════════════════ */

TopicBridge* topic_bridge_create(MessageBus* bus, const char* channel_name,
                                  TopicBridgeDirection direction) {
    if (!bus || !channel_name) return NULL;
    if (direction != TOPIC_BRIDGE_PUB &&
        direction != TOPIC_BRIDGE_SUB &&
        direction != TOPIC_BRIDGE_BIDIR) return NULL;

    TopicBridge* b = (TopicBridge*)calloc(1, sizeof(*b));
    if (!b) return NULL;

    b->bus       = bus;
    b->direction = direction;
    snprintf(b->channel_name, sizeof(b->channel_name), "%s", channel_name);
    return b;
}

int topic_bridge_add_topic(TopicBridge* bridge, const char* topic, uint32_t type_id) {
    if (!bridge || !topic) return ERR_INVALID_PARAM;
    if (bridge->n_topics >= TOPIC_BRIDGE_MAX_TOPICS) return ERR_OVERFLOW;

    BridgeEntry* e = &bridge->entries[bridge->n_topics];
    snprintf(e->topic, sizeof(e->topic), "%s", topic);
    e->type_id = type_id;
    memset(&e->stats, 0, sizeof(e->stats));
    bridge->n_topics++;
    return ERR_OK;
}

int topic_bridge_start(TopicBridge* bridge) {
    if (!bridge) return ERR_INVALID_PARAM;
    if (bridge->running) return ERR_BUSY;

    IpcRole role = (bridge->direction == TOPIC_BRIDGE_PUB) ? IPC_ROLE_PUBLISHER
                                                            : IPC_ROLE_SUBSCRIBER;
    /* BIDIR: PUB 侧先建通道; SUB 侧由对方进程建 (约定) */
    if (bridge->direction == TOPIC_BRIDGE_BIDIR) role = IPC_ROLE_PUBLISHER;

    bridge->ipc = ipc_channel_open(bridge->channel_name, role, 64);
    if (!bridge->ipc) {
        LOG_WARN("topic_bridge", "ipc_channel_open('%s', role=%d) failed",
                 bridge->channel_name, (int)role);
        return ERR_IO;
    }

    bridge->should_stop = 0;
    bridge->running     = 1;

    if (bridge->direction == TOPIC_BRIDGE_PUB ||
        bridge->direction == TOPIC_BRIDGE_BIDIR) {
        /* PUB: 先分配所有 ctx, 确保失败时可以整体回退 */
        for (int i = 0; i < bridge->n_topics; i++) {
            PubCallbackCtx* ctx = (PubCallbackCtx*)malloc(sizeof(PubCallbackCtx));
            if (!ctx) {
                /* 释放已分配的 ctx, 防止内存泄露 */
                for (int j = 0; j < i; j++) { free(bridge->pub_ctx[j]); bridge->pub_ctx[j] = NULL; }
                bridge->running = 0;
                ipc_channel_close(bridge->ipc);
                bridge->ipc = NULL;
                return ERR_NOMEM;
            }
            ctx->bridge    = bridge;
            ctx->entry_idx = i;
            bridge->pub_ctx[i] = ctx;
        }
        /* 订阅本地 bus 上所有注册 topic */
        for (int i = 0; i < bridge->n_topics; i++) {
            int rc = message_bus_subscribe(bridge->bus,
                                           bridge->entries[i].topic,
                                           pub_on_topic, bridge->pub_ctx[i]);
            if (rc != ERR_OK) {
                LOG_WARN("topic_bridge", "[PUB] subscribe '%s' failed (%d)",
                         bridge->entries[i].topic, rc);
            }
        }
        LOG_INFO("topic_bridge", "[PUB] started on channel '%s' (%d topics)",
                 bridge->channel_name, bridge->n_topics);
    }

    if (bridge->direction == TOPIC_BRIDGE_SUB) {
        /* SUB: 注册 IPC 回调并启动后台接收线程 */
        ipc_channel_subscribe(bridge->ipc, sub_on_ipc, bridge);
        int rc = ipc_channel_start(bridge->ipc);
        if (rc != 0) {
            LOG_WARN("topic_bridge", "[SUB] ipc_channel_start failed");
            bridge->running = 0;
            ipc_channel_close(bridge->ipc);
            bridge->ipc = NULL;
            return ERR_IO;
        }
        LOG_INFO("topic_bridge", "[SUB] started on channel '%s' (%d topics)",
                 bridge->channel_name, bridge->n_topics);
    }

    return ERR_OK;
}

int topic_bridge_stop(TopicBridge* bridge) {
    if (!bridge || !bridge->running) return ERR_OK;
    bridge->should_stop = 1;
    if (bridge->ipc) {
        ipc_channel_stop(bridge->ipc);
    }
    bridge->running = 0;
    LOG_INFO("topic_bridge", "stopped ('%s')", bridge->channel_name);
    return ERR_OK;
}

void topic_bridge_destroy(TopicBridge* bridge) {
    if (!bridge) return;
    topic_bridge_stop(bridge);
    if (bridge->ipc) {
        ipc_channel_close(bridge->ipc);
        bridge->ipc = NULL;
    }
    /* 释放 PUB 方向的 callback 上下文 */
    for (int i = 0; i < bridge->n_topics; i++) {
        if (bridge->pub_ctx[i]) {
            free(bridge->pub_ctx[i]);
            bridge->pub_ctx[i] = NULL;
        }
    }
    free(bridge);
}

/* ══════════════════════════════════════════════════════════ */
/* 内省                                                        */
/* ══════════════════════════════════════════════════════════ */

int topic_bridge_get_stats(const TopicBridge* bridge, const char* topic,
                            TopicBridgeStats* out) {
    if (!bridge || !topic || !out) return ERR_INVALID_PARAM;
    for (int i = 0; i < bridge->n_topics; i++) {
        if (strcmp(bridge->entries[i].topic, topic) == 0) {
            *out = bridge->entries[i].stats;
            return ERR_OK;
        }
    }
    return ERR_NOT_FOUND;
}

int topic_bridge_topic_count(const TopicBridge* bridge) {
    if (!bridge) return 0;
    return bridge->n_topics;
}
