/**
 * transport.c — 统一传输抽象层实现
 *
 * 路由策略:
 *   AUTO:  同进程→bus, 同机跨进程→SHM IPC, 跨机→TCP
 *   LOCAL: 仅 bus
 *   IPC:   仅 SHM
 *   REMOTE: 仅 TCP
 *
 * 设计: 对每个订阅的 topic，追踪"发布者在哪"，自动选择最优传输方式。
 */

#include "transport.h"
#include "error_codes.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <pthread.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>

/* ── Topic 路由条目 ──────────────────────────────────────── */

#define TRANSPORT_MAX_TOPICS 64

typedef struct {
    char        topic[MSG_BUS_MAX_TOPIC_LEN];
    RouteType   route;           /**< 当前路由方式 */
    bool        is_publisher;    /**< 本节点是发布者 */
    bool        is_subscriber;   /**< 本节点是订阅者 */

    /* 本地订阅 */
    MessageCallback  local_cb;
    void*            local_user_data;

    /* IPC 通道（跨进程同机） */
    IpcChannel*      ipc_channel;

    /* 远端连接（跨机） */
    bool             remote_bridged;
} TopicRoute;

/* ── Transport 内部结构 ──────────────────────────────────── */

struct Transport {
    MessageBus*        bus;
    DiscoveryManager*  discovery;
    TransportPolicy    policy;
    NetworkTransport*  net_transport;    /**< TCP 传输层 */
    bool               running;

    /* Topic 路由表 */
    TopicRoute         routes[TRANSPORT_MAX_TOPICS];
    int                route_count;
    pthread_mutex_t    mutex;
};

/* ── 内部辅助 ────────────────────────────────────────────── */

static TopicRoute* find_or_create_route(Transport* t, const char* topic) {
    for (int i = 0; i < t->route_count; i++) {
        if (strcmp(t->routes[i].topic, topic) == 0)
            return &t->routes[i];
    }
    if (t->route_count >= TRANSPORT_MAX_TOPICS) return NULL;
    TopicRoute* r = &t->routes[t->route_count++];
    memset(r, 0, sizeof(*r));
    snprintf(r->topic, MSG_BUS_MAX_TOPIC_LEN, "%s", topic);
    return r;
}

/**
 * 根据 discovery 拓扑判断发布者在哪，选择最优路由。
 */
/* ── IPC 通道名称（topic 中 '/' 替换为 '_'）─────────────── */
static void topic_to_ipc_name(const char* topic, char* out, size_t out_sz) {
    snprintf(out, out_sz, "flow_%s", topic);
    for (char* p = out; *p; p++) {
        if (*p == '/' || *p == ' ') *p = '_';
    }
}

static RouteType determine_route(Transport* t, const char* topic) {
    if (t->policy == TRANSPORT_LOCAL)  return ROUTE_LOCAL;
    if (t->policy == TRANSPORT_IPC)    return ROUTE_IPC;
    if (t->policy == TRANSPORT_REMOTE) return ROUTE_REMOTE;

    /* AUTO: 查询 discovery 拓扑 */
    if (!t->discovery) return ROUTE_LOCAL;

    const TopologyGraph* g = discovery_get_topology(t->discovery);
    if (!g) return ROUTE_LOCAL;

    /* 查找发布此 topic 的其他节点 */
    bool has_remote = false;
    for (uint32_t i = 0; i < g->node_count; i++) {
        const NodeInfo* n = &g->nodes[i];
        if (!n->alive) continue;
        for (uint32_t j = 0; j < n->topic_count; j++) {
            if (strcmp(n->topics[j].topic, topic) == 0 &&
                (n->topics[j].capabilities & CAP_PUBLISHER)) {
                /* 检查是否远程 */
                struct in_addr a;
                a.s_addr = n->ipv4_address;
                const char* ip = inet_ntoa(a);
                if (strcmp(ip, "127.0.0.1") != 0 && strcmp(ip, "0.0.0.0") != 0) {
                    has_remote = true;
                }
            }
        }
    }

    return has_remote ? ROUTE_REMOTE : ROUTE_LOCAL;
}

/* ── 发布回调（用于 IPC 和 TCP 转发）─────────────────────── */

/* 本地总线发布总是执行，IPC/TCP 转发按需 */

/* ══════════════════════════════════════════════════════════ */
/* 公开 API                                                    */
/* ══════════════════════════════════════════════════════════ */

Transport* transport_create(MessageBus* bus, DiscoveryManager* discovery,
                            TransportPolicy policy) {
    Transport* t = (Transport*)calloc(1, sizeof(Transport));
    if (!t) return NULL;

    t->bus       = bus;
    t->discovery = discovery;
    t->policy    = policy;
    pthread_mutex_init(&t->mutex, NULL);

    /* 如果是 REMOTE 或 AUTO 模式，创建 TCP 传输层 */
    if (policy == TRANSPORT_REMOTE || policy == TRANSPORT_AUTO) {
        t->net_transport = net_transport_create("0.0.0.0", 0, bus, discovery);
    }

    printf("[transport] created (policy=%d, bus=%p, discovery=%p)\n",
           (int)policy, (void*)bus, (void*)discovery);
    return t;
}

void transport_destroy(Transport* t) {
    if (!t) return;
    if (t->running) transport_stop(t);

    /* 关闭所有 IPC 通道 */
    for (int i = 0; i < t->route_count; i++) {
        if (t->routes[i].ipc_channel) {
            ipc_channel_close(t->routes[i].ipc_channel);
        }
    }

    if (t->net_transport) {
        net_transport_destroy(t->net_transport);
    }

    pthread_mutex_destroy(&t->mutex);
    free(t);
    printf("[transport] destroyed\n");
}

int transport_start(Transport* t) {
    if (!t || t->running) return ERR_INVALID_PARAM;
    t->running = true;

    /* 启动 TCP 传输层 */
    if (t->net_transport) {
        net_transport_start(t->net_transport);
    }

    /* 自动创建 IPC 通道（根据 discovery 拓扑） */
    if (t->discovery && (t->policy == TRANSPORT_AUTO || t->policy == TRANSPORT_IPC)) {
        int created = discovery_create_ipc_channels(t->discovery, 32);
        printf("[transport] auto-created %d IPC channels\n", created);
    }

    printf("[transport] started (routes=%d)\n", t->route_count);
    return 0;
}

void transport_stop(Transport* t) {
    if (!t || !t->running) return;
    t->running = false;

    if (t->net_transport) {
        net_transport_stop(t->net_transport);
    }
    printf("[transport] stopped\n");
}

int transport_advertise(Transport* t, const char* topic, uint32_t type_id) {
    if (!t || !topic) return ERR_INVALID_PARAM;

    pthread_mutex_lock(&t->mutex);
    TopicRoute* r = find_or_create_route(t, topic);
    if (!r) { pthread_mutex_unlock(&t->mutex); return ERR_INVALID_PARAM; }
    r->is_publisher = true;
    r->route = determine_route(t, topic);

    /* IPC policy: create publisher channel so cross-process subscribers can read */
    if (t->policy == TRANSPORT_IPC && !r->ipc_channel) {
        char ch_name[256];
        topic_to_ipc_name(topic, ch_name, sizeof(ch_name));
        r->ipc_channel = ipc_channel_open(ch_name, IPC_ROLE_PUBLISHER, 32);
        if (r->ipc_channel) r->route = ROUTE_IPC;
    }
    pthread_mutex_unlock(&t->mutex);

    /* Advertise via discovery */
    if (t->discovery) {
        discovery_advertise(t->discovery, topic, type_id, CAP_PUBLISHER, 0);
    }

    printf("[transport] advertise '%s' (route=%d, type_id=0x%08x)\n",
           topic, (int)r->route, type_id);
    return 0;
}

int transport_publish(Transport* t, const char* topic,
                      const void* data, uint32_t size) {
    if (!t || !topic) return ERR_INVALID_PARAM;

    /* Always publish to local bus first */
    int ret = message_bus_publish(t->bus, topic, "transport", data, size);

    /* If we have an IPC publisher channel, also push to shared memory */
    pthread_mutex_lock(&t->mutex);
    for (int i = 0; i < t->route_count; i++) {
        if (strcmp(t->routes[i].topic, topic) == 0) {
            if (t->routes[i].ipc_channel && t->routes[i].is_publisher) {
                ipc_channel_publish(t->routes[i].ipc_channel, topic, "transport",
                                    data, size);
            }
            break;
        }
    }
    pthread_mutex_unlock(&t->mutex);

    return ret;
}

/* ── IPC → 本地 bus 中继（relay）──────────────────────── */
/* IPC 订阅者方收到消息后，重新发布到本地 bus，这样:
 * 1) choreo 调度触发器正常运作（它监听的是本地 bus）
 * 2) 所有已经通过 message_bus_subscribe 注册的回调自动被调用
 * 因此在 ipc_channel_subscribe 中不需要直接传入用户回调 */
typedef struct { MessageBus* bus; } IpcRelayCtx;

static void ipc_to_bus_relay(const Message* msg, void* user_data) {
    IpcRelayCtx* ctx = (IpcRelayCtx*)user_data;
    message_bus_publish(ctx->bus, msg->topic, msg->sender, msg->data, msg->data_size);
}

int transport_subscribe(Transport* t, const char* topic,
                        MessageCallback callback, void* user_data) {
    if (!t || !topic || !callback) return ERR_INVALID_PARAM;

    pthread_mutex_lock(&t->mutex);
    TopicRoute* r = find_or_create_route(t, topic);
    if (!r) { pthread_mutex_unlock(&t->mutex); return ERR_INVALID_PARAM; }
    r->is_subscriber  = true;
    r->local_cb       = callback;
    r->local_user_data = user_data;
    r->route = determine_route(t, topic);
    pthread_mutex_unlock(&t->mutex);

    /* Subscribe to local bus (always, as fallback) */
    message_bus_subscribe(t->bus, topic, callback, user_data);

    /* For REMOTE routes, bridge via TCP */
    if (r->route == ROUTE_REMOTE && t->net_transport) {
        net_transport_bridge_topic(t->net_transport, topic);
        r->remote_bridged = true;
    }

    /* For IPC policy/routes, set up IPC subscriber */
    if ((t->policy == TRANSPORT_IPC || r->route == ROUTE_IPC) && !r->ipc_channel) {
        char ch_name[256];
        topic_to_ipc_name(topic, ch_name, sizeof(ch_name));
        /* Retry up to 20 times (2s) waiting for publisher to create the channel */
        for (int attempt = 0; attempt < 20; attempt++) {
            r->ipc_channel = ipc_channel_open(ch_name, IPC_ROLE_SUBSCRIBER, 32);
            if (r->ipc_channel) break;
            usleep(100000); /* 100ms */
        }
        if (r->ipc_channel) {
            /* 使用 relay 而不是用户回调: IPC 消息转御到本地 bus,
             * 这样 choreo trigger 和所有本地订阅者自动被调用 */
            IpcRelayCtx* ctx = (IpcRelayCtx*)malloc(sizeof(IpcRelayCtx));
            if (ctx) {
                ctx->bus = t->bus;
                ipc_channel_subscribe(r->ipc_channel, ipc_to_bus_relay, ctx);
            }
            ipc_channel_start(r->ipc_channel);
        } else {
            printf("[transport] WARNING: IPC channel '%s' not available, falling back to bus\n",
                   ch_name);
        }
    }

    printf("[transport] subscribe '%s' (route=%d)\n", topic, (int)r->route);
    return 0;
}

int transport_unsubscribe(Transport* t, const char* topic,
                          MessageCallback callback) {
    if (!t || !topic) return ERR_INVALID_PARAM;

    message_bus_unsubscribe(t->bus, topic, callback);

    pthread_mutex_lock(&t->mutex);
    for (int i = 0; i < t->route_count; i++) {
        if (strcmp(t->routes[i].topic, topic) == 0) {
            if (t->routes[i].remote_bridged && t->net_transport) {
                net_transport_unbridge_topic(t->net_transport, topic);
            }
            if (t->routes[i].ipc_channel) {
                ipc_channel_close(t->routes[i].ipc_channel);
                t->routes[i].ipc_channel = NULL;
            }
            t->routes[i].is_subscriber = false;
            break;
        }
    }
    pthread_mutex_unlock(&t->mutex);
    return 0;
}

RouteType transport_route_type(Transport* t, const char* topic) {
    if (!t || !topic) return ROUTE_LOCAL;
    pthread_mutex_lock(&t->mutex);
    for (int i = 0; i < t->route_count; i++) {
        if (strcmp(t->routes[i].topic, topic) == 0) {
            RouteType rt = t->routes[i].route;
            pthread_mutex_unlock(&t->mutex);
            return rt;
        }
    }
    pthread_mutex_unlock(&t->mutex);
    return ROUTE_LOCAL;
}

int transport_ipc_channel_count(Transport* t) {
    if (!t) return 0;
    int count = 0;
    pthread_mutex_lock(&t->mutex);
    for (int i = 0; i < t->route_count; i++) {
        if (t->routes[i].ipc_channel) count++;
    }
    pthread_mutex_unlock(&t->mutex);
    return count;
}

int transport_remote_peer_count(Transport* t) {
    if (!t || !t->net_transport) return 0;
    return net_transport_connection_count(t->net_transport);
}

void transport_get_stats(Transport* t, TransportStats* stats) {
    if (!t || !stats) return;
    memset(stats, 0, sizeof(*stats));

    uint64_t pub, del, drop;
    message_bus_get_stats(t->bus, &pub, &del, &drop);
    stats->local_published = pub;
    stats->local_delivered = del;

    if (t->net_transport) {
        NetTransportStats ns;
        net_transport_get_stats(t->net_transport, &ns);
        stats->remote_published = ns.msgs_sent;
        stats->remote_delivered = ns.msgs_received;
    }
}
