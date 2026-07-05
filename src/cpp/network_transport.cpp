/**
 * network_transport.cpp — 分布式 TCP 传输层 (基于 flowcoro::net)
 */

#include "network_transport.h"
#include <flowcoro/net.h>
#include <flowcoro/thread_pool.h>

#include <string>
#include <vector>
#include <unordered_map>
#include <mutex>
#include <atomic>
#include <thread>
#include <cstring>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <unistd.h>
#include <fcntl.h>

/* ══════════════════════════════════════════════════════════ */
/* 网络帧协议                                                  */
/* ══════════════════════════════════════════════════════════ */

static std::string serialize_frame(const Message* msg) {
    uint32_t payload = sizeof(Message);
    uint32_t net_len = htonl(payload);
    std::string frame;
    frame.append((const char*)&net_len, 4);
    frame.append((const char*)msg, sizeof(Message));
    return frame;
}

static bool deserialize_frame(const uint8_t* data, size_t len, Message* msg) {
    if (len < sizeof(Message)) return false;
    memcpy(msg, data, sizeof(Message));
    return true;
}

static ssize_t read_full(int fd, void* buf, size_t count) {
    size_t total = 0;
    while (total < count) {
        ssize_t n = read(fd, (char*)buf + total, count - total);
        if (n <= 0) return n;
        total += (size_t)n;
    }
    return (ssize_t)total;
}

/* ══════════════════════════════════════════════════════════ */
/* Bridge subscription (needs to be outside NetworkTransport)  */
/* ══════════════════════════════════════════════════════════ */

struct NetworkTransport;  /* fwd */

struct BridgeSub {
    NetworkTransport* t;
    std::string       topic;
};

/* ══════════════════════════════════════════════════════════ */
/* 连接状态                                                    */
/* ══════════════════════════════════════════════════════════ */

struct PeerConn {
    std::string            host;
    uint16_t               port;
    int                    fd;          // -1 = disconnected
    std::atomic<bool>      active;
    std::mutex             write_mutex; // protect concurrent writes
};

struct NetworkTransport {
    std::string             bind_addr;
    uint16_t                port;
    MessageBus*             bus;
    DiscoveryManager*       discovery;

    int                     listen_fd;
    std::atomic<bool>       running;

    std::vector<PeerConn*>  peers;
    std::mutex              peers_mutex;

    std::vector<std::string> bridged_topics;
    std::mutex              bridge_mutex;

    /* Background threads */
    std::thread             accept_thread;
    std::thread             recv_thread;

    NetTransportStats       stats;

    /* Bridge subscriptions */
    std::vector<BridgeSub*> subs;
};

/* ══════════════════════════════════════════════════════════ */
/* 出站桥接回调                                                */
/* ══════════════════════════════════════════════════════════ */

static void bridge_outbound_cb(const Message* msg, void* user_data) {
    auto* sub = (BridgeSub*)user_data;
    auto* t   = sub->t;

    std::string frame = serialize_frame(msg);

    std::lock_guard<std::mutex> lock(t->peers_mutex);
    for (auto* peer : t->peers) {
        if (!peer->active || peer->fd < 0) continue;

        std::lock_guard<std::mutex> wlock(peer->write_mutex);
        ssize_t n = write(peer->fd, frame.data(), frame.size());
        if (n > 0) {
            t->stats.bytes_sent += (uint64_t)n;
            t->stats.msgs_sent++;
        } else {
            t->stats.send_errors++;
            peer->active = false;
            close(peer->fd);
            peer->fd = -1;
        }
    }
}

/* ══════════════════════════════════════════════════════════ */
/* 入站接收线程                                                */
/* ══════════════════════════════════════════════════════════ */

static void recv_thread_fn(NetworkTransport* t) {
    uint8_t buf[sizeof(Message) + 4];

    while (t->running) {
        /* Messages received this cycle, published AFTER releasing peers_mutex.
         * Publishing under the lock is unsafe: message_bus delivery can re-enter
         * the transport (e.g. bridge_outbound_cb also locks peers_mutex), which
         * would self-deadlock and permanently starve every other peers_mutex
         * user (accept, outbound bridge, and connection_count used by the
         * monitor/state-file writer). */
        std::vector<Message> inbound;

        {
            /* Poll all peer connections — lock held only for the poll loop,
             * never across usleep() or bus publish. */
            std::lock_guard<std::mutex> lock(t->peers_mutex);
            for (auto* peer : t->peers) {
                if (!peer->active || peer->fd < 0) continue;

                /* Non-blocking read of length prefix */
                uint32_t net_len = 0;
                ssize_t n = recv(peer->fd, &net_len, 4, MSG_DONTWAIT);
                if (n == 0) {
                    peer->active = false;
                    close(peer->fd);
                    peer->fd = -1;
                    t->stats.disconnects++;
                    printf("[net_transport] peer %s:%u disconnected\n",
                           peer->host.c_str(), peer->port);
                    continue;
                }
                if (n < 0) {
                    if (errno == EAGAIN || errno == EWOULDBLOCK) continue;
                    peer->active = false;
                    close(peer->fd);
                    peer->fd = -1;
                    t->stats.disconnects++;
                    continue;
                }

                uint32_t payload = ntohl(net_len);
                if (payload > sizeof(Message)) continue;

                /* Read payload. fd is non-blocking, so read_full does not block:
                 * on a short read the underlying read() returns EAGAIN (n<=0) and
                 * read_full returns early, so the frame is simply dropped (n !=
                 * payload below). Payload is also bounded by sizeof(Message)
                 * (checked above), so the lock is never held for long. */
                n = read_full(peer->fd, buf, payload);
                if (n != (ssize_t)payload) continue;

                t->stats.bytes_received += 4 + payload;
                t->stats.msgs_received++;

                Message msg;
                if (deserialize_frame(buf, payload, &msg)) {
                    inbound.push_back(msg);
                }
            }
        }

        /* Publish outside the lock. */
        for (const auto& msg : inbound) {
            message_bus_publish(t->bus, msg.topic, msg.sender,
                                msg.data, msg.data_size);
        }

        usleep(10000); /* 10ms poll interval */
    }
}

/* ══════════════════════════════════════════════════════════ */
/* Accept 线程                                                 */
/* ══════════════════════════════════════════════════════════ */

static void accept_thread_fn(NetworkTransport* t) {
    while (t->running) {
        struct sockaddr_in client_addr;
        socklen_t addr_len = sizeof(client_addr);
        int client_fd = accept(t->listen_fd, (struct sockaddr*)&client_addr, &addr_len);
        if (client_fd < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                usleep(100000);
                continue;
            }
            break;
        }

        /* Set TCP_NODELAY for low-latency messaging */
        int flag = 1;
        setsockopt(client_fd, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));
        fcntl(client_fd, F_SETFL, O_NONBLOCK);

        /* Create peer entry for this connection */
        auto* peer = new PeerConn();
        peer->host   = inet_ntoa(client_addr.sin_addr);
        peer->port   = ntohs(client_addr.sin_port);
        peer->fd     = client_fd;
        peer->active = true;

        {
            std::lock_guard<std::mutex> lock(t->peers_mutex);
            t->peers.push_back(peer);
        }
        t->stats.connects++;

        printf("[net_transport] accepted connection from %s:%u (total=%zu)\n",
               peer->host.c_str(), peer->port, t->peers.size());
    }
}

/* ══════════════════════════════════════════════════════════ */
/* Discovery 集成                                              */
/* ══════════════════════════════════════════════════════════ */

static void discovery_cb(const TopologyGraph* graph, const NodeInfo* node,
                         bool joined, void* user_data) {
    (void)graph;
    auto* t = (NetworkTransport*)user_data;
    if (!joined || !node->alive || node->ipv4_address == 0) return;

    struct in_addr a;
    a.s_addr = node->ipv4_address;
    const char* host = inet_ntoa(a);
    if (strcmp(host, "127.0.0.1") == 0 || strcmp(host, "0.0.0.0") == 0) return;

    uint16_t port = node->unicast_port ? node->unicast_port : NET_DEFAULT_PORT;
    printf("[net_transport] discovered %s → auto-connect %s:%u\n",
           node->name, host, port);
    net_transport_connect(t, host, port);
}

/* ══════════════════════════════════════════════════════════ */
/* 公开 API                                                    */
/* ══════════════════════════════════════════════════════════ */

extern "C" {

NetworkTransport* net_transport_create(const char* bind_addr, uint16_t port,
                                       MessageBus* bus, DiscoveryManager* dm) {
    auto* t = new NetworkTransport();
    memset(t, 0, sizeof(*t));
    t->bind_addr  = bind_addr ? bind_addr : "0.0.0.0";
    t->port       = port ? port : NET_DEFAULT_PORT;
    t->bus        = bus;
    t->discovery  = dm;
    t->listen_fd  = -1;

    if (dm) discovery_set_change_callback(dm, discovery_cb, t);
    return t;
}

void net_transport_destroy(NetworkTransport* t) {
    if (!t) return;
    if (t->running) net_transport_stop(t);
    for (auto* sub : t->subs) { delete sub; }
    for (auto* p : t->peers) {
        if (p->fd >= 0) close(p->fd);
        delete p;
    }
    delete t;
}

int net_transport_start(NetworkTransport* t) {
    if (!t || t->running) return -1;

    t->listen_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (t->listen_fd < 0) return -1;

    int reuse = 1;
    setsockopt(t->listen_fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    fcntl(t->listen_fd, F_SETFL, O_NONBLOCK);

    struct sockaddr_in addr = {};
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = inet_addr(t->bind_addr.c_str());
    addr.sin_port        = htons(t->port);

    if (bind(t->listen_fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        close(t->listen_fd);
        return -1;
    }
    listen(t->listen_fd, 32);

    t->running = true;
    t->accept_thread = std::thread(accept_thread_fn, t);
    t->recv_thread   = std::thread(recv_thread_fn, t);

    printf("[net_transport] listening on %s:%u\n", t->bind_addr.c_str(), t->port);
    return 0;
}

void net_transport_stop(NetworkTransport* t) {
    if (!t || !t->running) return;
    t->running = false;
    if (t->listen_fd >= 0) { close(t->listen_fd); t->listen_fd = -1; }
    if (t->accept_thread.joinable()) t->accept_thread.join();
    if (t->recv_thread.joinable())   t->recv_thread.join();
    printf("[net_transport] stopped\n");
}

int net_transport_bridge_topic(NetworkTransport* t, const char* topic) {
    if (!t || !topic) return -1;
    auto* sub = new BridgeSub();
    sub->t = t; sub->topic = topic;
    message_bus_subscribe(t->bus, topic, bridge_outbound_cb, sub);
    t->subs.push_back(sub);
    return 0;
}

int net_transport_unbridge_topic(NetworkTransport* t, const char* topic) {
    if (!t || !topic) return -1;
    message_bus_unsubscribe(t->bus, topic, bridge_outbound_cb);
    return 0;
}

int net_transport_connect(NetworkTransport* t, const char* host, uint16_t port) {
    if (!t || !host) return -1;

    int fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) return -1;

    struct sockaddr_in addr = {};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = inet_addr(host);
    addr.sin_port = htons(port);

    if (connect(fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        close(fd);
        return -1;
    }

    int flag = 1;
    setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));
    fcntl(fd, F_SETFL, O_NONBLOCK);

    auto* peer = new PeerConn();
    peer->host   = host;
    peer->port   = port;
    peer->fd     = fd;
    peer->active = true;

    {
        std::lock_guard<std::mutex> lock(t->peers_mutex);
        t->peers.push_back(peer);
    }
    t->stats.connects++;
    return 0;
}

int net_transport_disconnect(NetworkTransport* t, const char* host, uint16_t port) {
    if (!t) return -1;
    std::lock_guard<std::mutex> lock(t->peers_mutex);
    for (auto* p : t->peers) {
        if (p->host == host && p->port == port) {
            p->active = false;
            if (p->fd >= 0) { close(p->fd); p->fd = -1; }
            t->stats.disconnects++;
            return 0;
        }
    }
    return -1;
}

int net_transport_connection_count(NetworkTransport* t) {
    if (!t) return 0;
    int n = 0;
    std::lock_guard<std::mutex> lock(t->peers_mutex);
    for (auto* p : t->peers) if (p->active) n++;
    return n;
}

void net_transport_get_stats(NetworkTransport* t, NetTransportStats* stats) {
    if (t && stats) *stats = t->stats;
}

} /* extern "C" */
