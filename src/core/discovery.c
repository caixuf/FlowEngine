/**
 * discovery.c — UDP 组播服务发现实现 (FlowEngine Phase 3)
 */

#include "discovery.h"
#include "ipc_channel.h"
#include "error_codes.h"
#include "clock_service.h"
#include <cjson/cJSON.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <pthread.h>
#include <unistd.h>
#include <time.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>

/* ── 协议常量 ─────────────────────────────────────────────── */

#define DISC_MAGIC      0x43534944u  /* "DISC" in LE */
#define DISC_VERSION    1
#define DISC_MSG_HELLO      0
#define DISC_MSG_HEARTBEAT  1
#define DISC_MSG_GOODBYE    2
#define DISC_MSG_QUERY      3

/* ── 简易 CRC32 ──────────────────────────────────────────── */

static uint32_t crc32(const uint8_t* data, size_t len) {
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++)
            crc = (crc >> 1) ^ ((crc & 1) ? 0xEDB88320u : 0);
    }
    return ~crc;
}

/* ══════════════════════════════════════════════════════════ */
/* 内部结构                                                   */
/* ══════════════════════════════════════════════════════════ */

struct DiscoveryManager {
    /* 本节点信息 */
    char     my_name[DISC_NODE_NAME_LEN];
    uint32_t my_pid;
    uint8_t  my_capabilities;
    TopicAdvert my_topics[DISC_MAX_TOPICS_PER_NODE];
    uint32_t my_topic_count;

    /* 拓扑 */
    TopologyGraph topology;
    pthread_mutex_t topo_mutex;

    /* 网络 */
    int      sock_fd;
    struct sockaddr_in mcast_addr;
    bool     running;

    /* 线程 */
    pthread_t beacon_thread;
    pthread_t recv_thread;

    /* 回调 */
    TopologyChangeCallback change_cb;
    void*                  change_user_data;
};

/* ══════════════════════════════════════════════════════════ */
/* 序列化/反序列化 Beacon                                      */
/* ══════════════════════════════════════════════════════════ */

static int serialize_beacon(uint8_t* buf, size_t buf_size,
                            uint8_t msg_type, const DiscoveryManager* dm) {
    if (buf_size < 128) return ERR_INVALID_PARAM;
    size_t off = 0;

    uint32_t magic = DISC_MAGIC;
    memcpy(buf + off, &magic, 4); off += 4;
    buf[off++] = DISC_VERSION;
    buf[off++] = msg_type;
    memcpy(buf + off, dm->my_name, DISC_NODE_NAME_LEN); off += DISC_NODE_NAME_LEN;
    memcpy(buf + off, &dm->my_pid, 4); off += 4;
    buf[off++] = dm->my_capabilities;

    /* Topic count + topics */
    uint16_t tcount = (uint16_t)dm->my_topic_count;
    memcpy(buf + off, &tcount, 2); off += 2;
    for (uint16_t i = 0; i < tcount; i++) {
        if (off + sizeof(TopicAdvert) > buf_size) break;
        memcpy(buf + off, &dm->my_topics[i], sizeof(TopicAdvert));
        off += sizeof(TopicAdvert);
    }

    /* IP + port (placeholder for now) */
    uint32_t ip = 0; /* will be filled by receiver from recvaddr */
    memcpy(buf + off, &ip, 4); off += 4;
    uint16_t port = DISC_MULTICAST_PORT;
    memcpy(buf + off, &port, 2); off += 2;

    /* CRC32 (over everything except magic) */
    uint32_t crc = crc32(buf + 4, off - 4);
    memcpy(buf + off, &crc, 4); off += 4;

    return (int)off;
}

static int deserialize_beacon(const uint8_t* buf, size_t len,
                              uint8_t* msg_type, NodeInfo* node) {
    if (len < 128) return ERR_INVALID_PARAM;

    uint32_t magic;
    memcpy(&magic, buf, 4);
    if (magic != DISC_MAGIC) return ERR_INVALID_PARAM;

    size_t off = 4;
    uint8_t ver = buf[off++];
    (void)ver;
    *msg_type = buf[off++];

    memset(node, 0, sizeof(*node));
    memcpy(node->name, buf + off, DISC_NODE_NAME_LEN);
    node->name[DISC_NODE_NAME_LEN - 1] = '\0';
    off += DISC_NODE_NAME_LEN;

    memcpy(&node->pid, buf + off, 4); off += 4;
    node->capabilities = buf[off++];

    if (off + 2 > len) return ERR_INVALID_PARAM;
    uint16_t tcount;
    memcpy(&tcount, buf + off, 2); off += 2;
    if (tcount > DISC_MAX_TOPICS_PER_NODE) tcount = DISC_MAX_TOPICS_PER_NODE;

    /* Validate topic data and CRC fit within packet before copying */
    if (off + tcount * sizeof(TopicAdvert) + 4 > len) return ERR_INVALID_PARAM;

    node->topic_count = tcount;
    for (uint16_t i = 0; i < tcount; i++) {
        memcpy(&node->topics[i], buf + off, sizeof(TopicAdvert));
        off += sizeof(TopicAdvert);
    }

    memcpy(&node->ipv4_address, buf + off, 4); off += 4;
    memcpy(&node->unicast_port, buf + off, 2); off += 2;

    /* Verify CRC32 (covers everything after the 4-byte magic) */
    uint32_t received_crc;
    memcpy(&received_crc, buf + off, 4);
    uint32_t computed_crc = crc32(buf + 4, off - 4);
    if (received_crc != computed_crc) {
        /* Corrupt or truncated packet — silently discard to avoid ghost nodes */
        return ERR_INVALID_PARAM;
    }

    node->alive = true;
    return 0;
}

/* ══════════════════════════════════════════════════════════ */
/* 拓扑更新                                                   */
/* ══════════════════════════════════════════════════════════ */

static void topology_update(DiscoveryManager* dm, const NodeInfo* node,
                            bool* is_new) {
    pthread_mutex_lock(&dm->topo_mutex);

    /* Find existing or create new */
    int idx = -1;
    for (uint32_t i = 0; i < dm->topology.node_count; i++) {
        if (strcmp(dm->topology.nodes[i].name, node->name) == 0 &&
            dm->topology.nodes[i].pid == node->pid) {
            idx = (int)i;
            break;
        }
    }

    if (idx < 0) {
        /* New node */
        if (dm->topology.node_count >= DISC_MAX_NODES) {
            pthread_mutex_unlock(&dm->topo_mutex);
            return;
        }
        idx = (int)dm->topology.node_count++;
        *is_new = true;
    }

    dm->topology.nodes[idx] = *node;
    dm->topology.nodes[idx].last_heartbeat_us = clock_now_us() / 1000ULL;

    /* Update relations: mark pub/sub matches between nodes */
    for (uint32_t i = 0; i < dm->topology.node_count; i++) {
        if (i == (uint32_t)idx) continue;
        /* Check if node i publishes something that node idx subscribes to */
        for (uint32_t ai = 0; ai < dm->topology.nodes[i].topic_count; ai++) {
            for (uint32_t aj = 0; aj < dm->topology.nodes[idx].topic_count; aj++) {
                if (strcmp(dm->topology.nodes[i].topics[ai].topic,
                           dm->topology.nodes[idx].topics[aj].topic) == 0) {
                    if ((dm->topology.nodes[i].topics[ai].capabilities & CAP_PUBLISHER) &&
                        (dm->topology.nodes[idx].topics[aj].capabilities & CAP_SUBSCRIBER)) {
                        dm->topology.relation[i][idx] |= 0x01; /* pub/sub */
                    }
                }
            }
        }
    }

    pthread_mutex_unlock(&dm->topo_mutex);
}

/* ══════════════════════════════════════════════════════════ */
/* 超时检测                                                   */
/* ══════════════════════════════════════════════════════════ */

static void check_timeouts(DiscoveryManager* dm) {
    uint64_t now = clock_now_us() / 1000ULL;
    pthread_mutex_lock(&dm->topo_mutex);

    for (uint32_t i = 0; i < dm->topology.node_count; i++) {
        NodeInfo* n = &dm->topology.nodes[i];
        if (n->alive && (now - n->last_heartbeat_us) > DISC_NODE_TIMEOUT_MS) {
            n->alive = false;
            printf("[discovery:%s] node '%s' (pid=%u) timed out\n",
                   dm->my_name, n->name, n->pid);
            if (dm->change_cb) {
                pthread_mutex_unlock(&dm->topo_mutex);
                dm->change_cb(&dm->topology, n, false, dm->change_user_data);
                pthread_mutex_lock(&dm->topo_mutex);
            }
        }
    }

    pthread_mutex_unlock(&dm->topo_mutex);
}

/* ══════════════════════════════════════════════════════════ */
/* 网络线程                                                   */
/* ══════════════════════════════════════════════════════════ */

static void* beacon_thread_fn(void* arg) {
    DiscoveryManager* dm = (DiscoveryManager*)arg;
    uint8_t buf[DISC_BEACON_MAX_SIZE];

    /* Send initial HELLO */
    int len = serialize_beacon(buf, sizeof(buf), DISC_MSG_HELLO, dm);
    if (len > 0)
        sendto(dm->sock_fd, buf, (size_t)len, 0,
               (const struct sockaddr*)&dm->mcast_addr, sizeof(dm->mcast_addr));

    while (dm->running) {
        usleep(DISC_HEARTBEAT_MS * 1000);

        /* Send HEARTBEAT */
        len = serialize_beacon(buf, sizeof(buf), DISC_MSG_HEARTBEAT, dm);
        if (len > 0)
            sendto(dm->sock_fd, buf, (size_t)len, 0,
                   (const struct sockaddr*)&dm->mcast_addr, sizeof(dm->mcast_addr));

        /* Check for dead nodes */
        check_timeouts(dm);
    }
    return NULL;
}

static void* recv_thread_fn(void* arg) {
    DiscoveryManager* dm = (DiscoveryManager*)arg;
    uint8_t buf[DISC_BEACON_MAX_SIZE];
    struct sockaddr_in from_addr;
    socklen_t from_len = sizeof(from_addr);

    while (dm->running) {
        ssize_t n = recvfrom(dm->sock_fd, buf, sizeof(buf), MSG_DONTWAIT,
                             (struct sockaddr*)&from_addr, &from_len);
        if (n <= 0) {
            usleep(100000); /* 100ms poll */
            continue;
        }

        uint8_t msg_type;
        NodeInfo node;
        if (deserialize_beacon(buf, (size_t)n, &msg_type, &node) != 0) continue;

        /* Skip our own messages */
        if (strcmp(node.name, dm->my_name) == 0 && node.pid == dm->my_pid) continue;

        /* Fill in source IP */
        node.ipv4_address = from_addr.sin_addr.s_addr;

        switch (msg_type) {
            case DISC_MSG_HELLO: {
                bool is_new = false;
                topology_update(dm, &node, &is_new);
                printf("[discovery:%s] discovered '%s' (pid=%u, caps=0x%02x, %u topics)\n",
                       dm->my_name, node.name, node.pid,
                       node.capabilities, node.topic_count);
                if (dm->change_cb)
                    dm->change_cb(&dm->topology, &node, true, dm->change_user_data);
                break;
            }
            case DISC_MSG_HEARTBEAT: {
                bool is_new = false;
                topology_update(dm, &node, &is_new);
                break;
            }
            case DISC_MSG_GOODBYE: {
                pthread_mutex_lock(&dm->topo_mutex);
                for (uint32_t i = 0; i < dm->topology.node_count; i++) {
                    if (strcmp(dm->topology.nodes[i].name, node.name) == 0) {
                        dm->topology.nodes[i].alive = false;
                        printf("[discovery:%s] '%s' left gracefully\n",
                               dm->my_name, node.name);
                        if (dm->change_cb)
                            dm->change_cb(&dm->topology, &node, false, dm->change_user_data);
                        break;
                    }
                }
                pthread_mutex_unlock(&dm->topo_mutex);
                break;
            }
            case DISC_MSG_QUERY: {
                /* Respond with our HELLO to the unicast address */
                int rlen = serialize_beacon(buf, sizeof(buf), DISC_MSG_HELLO, dm);
                if (rlen > 0) {
                    sendto(dm->sock_fd, buf, (size_t)rlen, 0,
                           (const struct sockaddr*)&from_addr, sizeof(from_addr));
                }
                break;
            }
        }
    }
    return NULL;
}

/* ══════════════════════════════════════════════════════════ */
/* 公开 API                                                    */
/* ══════════════════════════════════════════════════════════ */

DiscoveryManager* discovery_create(const char* node_name, uint8_t capabilities) {
    DiscoveryManager* dm = (DiscoveryManager*)calloc(1, sizeof(DiscoveryManager));
    if (!dm) return NULL;

    if (node_name) snprintf(dm->my_name, DISC_NODE_NAME_LEN, "%s", node_name);
    else snprintf(dm->my_name, DISC_NODE_NAME_LEN, "node_%d", getpid());
    dm->my_pid = (uint32_t)getpid();
    dm->my_capabilities = capabilities;

    pthread_mutex_init(&dm->topo_mutex, NULL);

    /* Setup UDP multicast socket */
    dm->sock_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (dm->sock_fd < 0) {
        free(dm);
        return NULL;
    }

    int reuse = 1;
    setsockopt(dm->sock_fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    /* Bind to multicast port */
    memset(&dm->mcast_addr, 0, sizeof(dm->mcast_addr));
    dm->mcast_addr.sin_family      = AF_INET;
    dm->mcast_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    dm->mcast_addr.sin_port        = htons(DISC_MULTICAST_PORT);
    bind(dm->sock_fd, (struct sockaddr*)&dm->mcast_addr, sizeof(dm->mcast_addr));

    /* Join multicast group */
    struct ip_mreq mreq;
    mreq.imr_multiaddr.s_addr = inet_addr(DISC_MULTICAST_GROUP);
    mreq.imr_interface.s_addr = htonl(INADDR_ANY);
    setsockopt(dm->sock_fd, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof(mreq));

    /* Set multicast TTL */
    uint8_t ttl = 1;
    setsockopt(dm->sock_fd, IPPROTO_IP, IP_MULTICAST_TTL, &ttl, sizeof(ttl));

    /* Set destination address for sending */
    dm->mcast_addr.sin_addr.s_addr = inet_addr(DISC_MULTICAST_GROUP);

    return dm;
}

void discovery_destroy(DiscoveryManager* dm) {
    if (!dm) return;

    if (dm->running) discovery_stop(dm);

    /* Send GOODBYE */
    uint8_t buf[DISC_BEACON_MAX_SIZE];
    int len = serialize_beacon(buf, sizeof(buf), DISC_MSG_GOODBYE, dm);
    if (len > 0) {
        sendto(dm->sock_fd, buf, (size_t)len, 0,
               (const struct sockaddr*)&dm->mcast_addr, sizeof(dm->mcast_addr));
    }

    if (dm->sock_fd >= 0) close(dm->sock_fd);
    pthread_mutex_destroy(&dm->topo_mutex);
    free(dm);
}

int discovery_start(DiscoveryManager* dm) {
    if (!dm || dm->running) return ERR_INVALID_PARAM;

    dm->running = true;
    pthread_create(&dm->beacon_thread, NULL, beacon_thread_fn, dm);
    pthread_create(&dm->recv_thread, NULL, recv_thread_fn, dm);

    printf("[discovery:%s] started (group=%s:%d, caps=0x%02x)\n",
           dm->my_name, DISC_MULTICAST_GROUP, DISC_MULTICAST_PORT,
           dm->my_capabilities);
    return 0;
}

void discovery_stop(DiscoveryManager* dm) {
    if (!dm || !dm->running) return;
    dm->running = false;
    pthread_join(dm->beacon_thread, NULL);
    pthread_join(dm->recv_thread, NULL);
    printf("[discovery:%s] stopped\n", dm->my_name);
}

int discovery_advertise(DiscoveryManager* dm, const char* topic,
                        uint32_t type_id, uint8_t capabilities,
                        double freq_hz) {
    if (!dm || !topic || dm->my_topic_count >= DISC_MAX_TOPICS_PER_NODE) return ERR_INVALID_PARAM;

    pthread_mutex_lock(&dm->topo_mutex);
    TopicAdvert* ta = &dm->my_topics[dm->my_topic_count++];
    memset(ta, 0, sizeof(*ta));
    snprintf(ta->topic, DISC_TOPIC_NAME_LEN, "%s", topic);
    ta->type_id      = type_id;
    ta->capabilities = capabilities;
    ta->frequency_hz = freq_hz;
    pthread_mutex_unlock(&dm->topo_mutex);

    return 0;
}

int discovery_unadvertise(DiscoveryManager* dm, const char* topic) {
    if (!dm || !topic) return ERR_INVALID_PARAM;

    pthread_mutex_lock(&dm->topo_mutex);
    for (uint32_t i = 0; i < dm->my_topic_count; i++) {
        if (strcmp(dm->my_topics[i].topic, topic) == 0) {
            /* Shift remaining */
            if (i < dm->my_topic_count - 1) {
                memmove(&dm->my_topics[i], &dm->my_topics[i + 1],
                        (dm->my_topic_count - i - 1) * sizeof(TopicAdvert));
            }
            dm->my_topic_count--;
            break;
        }
    }
    pthread_mutex_unlock(&dm->topo_mutex);
    return 0;
}

const TopologyGraph* discovery_get_topology(DiscoveryManager* dm) {
    return dm ? &dm->topology : NULL;
}

int discovery_copy_topology(const DiscoveryManager* dm, TopologyGraph* out) {
    if (!dm || !out) return ERR_INVALID_PARAM;
    pthread_mutex_lock(&dm->topo_mutex);
    memcpy(out, &dm->topology, sizeof(TopologyGraph));
    pthread_mutex_unlock(&dm->topo_mutex);
    return 0;
}

void discovery_set_change_callback(DiscoveryManager* dm,
                                   TopologyChangeCallback cb, void* user_data) {
    if (!dm) return;
    dm->change_cb        = cb;
    dm->change_user_data = user_data;
}

/* ══════════════════════════════════════════════════════════ */
/* 自省 / 导出                                                */
/* ══════════════════════════════════════════════════════════ */

char* discovery_export_json(DiscoveryManager* dm) {
    if (!dm) return strdup("{}");

    pthread_mutex_lock(&dm->topo_mutex);

    cJSON* root = cJSON_CreateObject();
    if (!root) { pthread_mutex_unlock(&dm->topo_mutex); return NULL; }

    cJSON_AddStringToObject(root, "self", dm->my_name);

    cJSON* nodes = cJSON_CreateArray();
    for (uint32_t i = 0; i < dm->topology.node_count; i++) {
        NodeInfo* n = &dm->topology.nodes[i];
        cJSON* node = cJSON_CreateObject();
        cJSON_AddStringToObject(node, "name", n->name);
        cJSON_AddNumberToObject(node, "pid", (double)n->pid);
        cJSON_AddBoolToObject(node, "alive", n->alive);
        cJSON_AddNumberToObject(node, "caps", n->capabilities);

        cJSON* topics = cJSON_CreateArray();
        for (uint32_t j = 0; j < n->topic_count; j++) {
            cJSON* topic = cJSON_CreateObject();
            cJSON_AddStringToObject(topic, "topic", n->topics[j].topic);
            char type_id_str[16];
            snprintf(type_id_str, sizeof(type_id_str), "0x%08x", n->topics[j].type_id);
            cJSON_AddStringToObject(topic, "type_id", type_id_str);
            cJSON_AddNumberToObject(topic, "freq", n->topics[j].frequency_hz);
            cJSON_AddNumberToObject(topic, "caps", n->topics[j].capabilities);
            const char* role = "unknown";
            bool is_pub = (n->topics[j].capabilities & CAP_PUBLISHER) != 0;
            bool is_sub = (n->topics[j].capabilities & CAP_SUBSCRIBER) != 0;
            if (is_pub && is_sub) role = "pubsub";
            else if (is_pub) role = "pub";
            else if (is_sub) role = "sub";
            cJSON_AddStringToObject(topic, "role", role);
            cJSON_AddItemToArray(topics, topic);
        }
        cJSON_AddItemToObject(node, "topics", topics);

        cJSON_AddItemToArray(nodes, node);
    }
    cJSON_AddItemToObject(root, "nodes", nodes);

    char* out = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);

    pthread_mutex_unlock(&dm->topo_mutex);
    return out;
}

void discovery_print_graph(DiscoveryManager* dm) {
    if (!dm) return;

    pthread_mutex_lock(&dm->topo_mutex);
    printf("\n[拓扑: %s] %u 节点\n", dm->my_name, dm->topology.node_count);
    for (uint32_t i = 0; i < dm->topology.node_count; i++) {
        NodeInfo* n = &dm->topology.nodes[i];
        printf("  %s (pid=%u) %s", n->name, n->pid, n->alive ? "●" : "○");
        if (n->topic_count > 0) {
            printf(" topics=[");
            for (uint32_t j = 0; j < n->topic_count && j < 4; j++) {
                const char* cap_str = (n->topics[j].capabilities & CAP_PUBLISHER) ? "pub" : "sub";
                printf("%s/%s", n->topics[j].topic, cap_str);
                if (j < n->topic_count - 1 && j < 3) printf(", ");
            }
            if (n->topic_count > 4) printf(" +%u more", n->topic_count - 4);
            printf("]");
        }
        printf("\n");
    }

    /* Relations */
    bool has_rel = false;
    for (uint32_t i = 0; i < dm->topology.node_count; i++) {
        for (uint32_t j = 0; j < dm->topology.node_count; j++) {
            if (dm->topology.relation[i][j]) {
                if (!has_rel) { printf("  relations:\n"); has_rel = true; }
                printf("    %s -> %s (pub/sub)\n",
                       dm->topology.nodes[i].name, dm->topology.nodes[j].name);
            }
        }
    }
    printf("\n");
    pthread_mutex_unlock(&dm->topo_mutex);
}

/* ══════════════════════════════════════════════════════════ */
/* 依赖等待 / 自动 IPC                                        */
/* ══════════════════════════════════════════════════════════ */

int discovery_wait_for_deps(DiscoveryManager* dm, const char** deps,
                            int dep_count, uint32_t timeout_ms) {
    if (!dm || !deps || dep_count <= 0) return 0;

    uint64_t deadline = clock_now_us() / 1000ULL + timeout_ms;

    while (clock_now_us() / 1000ULL < deadline) {
        int found = 0;
        pthread_mutex_lock(&dm->topo_mutex);
        for (int d = 0; d < dep_count; d++) {
            for (uint32_t i = 0; i < dm->topology.node_count; i++) {
                if (strcmp(dm->topology.nodes[i].name, deps[d]) == 0 &&
                    dm->topology.nodes[i].alive) {
                    found++;
                    break;
                }
            }
        }
        pthread_mutex_unlock(&dm->topo_mutex);

        if (found >= dep_count) {
            printf("[discovery:%s] all %d dependencies online\n", dm->my_name, dep_count);
            return 0;
        }

        usleep(500000); /* 500ms poll */
    }

    fprintf(stderr, "[discovery:%s] timeout waiting for %d dependencies\n",
            dm->my_name, dep_count);
    return ERR_INVALID_PARAM;
}

int discovery_create_ipc_channels(DiscoveryManager* dm, uint32_t max_depth) {
    if (!dm) return ERR_INVALID_PARAM;
    int created = 0;

    pthread_mutex_lock(&dm->topo_mutex);

    for (uint32_t i = 0; i < dm->topology.node_count; i++) {
        for (uint32_t j = 0; j < dm->topology.node_count; j++) {
            if (i == j) continue;
            if (!(dm->topology.relation[i][j] & 0x01)) continue;

            /* Node i publishes to node j — create IPC channel */
            NodeInfo* pub = &dm->topology.nodes[i];
            NodeInfo* sub = &dm->topology.nodes[j];

            /* Find the matching topic */
            for (uint32_t ai = 0; ai < pub->topic_count; ai++) {
                for (uint32_t aj = 0; aj < sub->topic_count; aj++) {
                    if (strcmp(pub->topics[ai].topic, sub->topics[aj].topic) != 0)
                        continue;

                    /* Create channel name: <topic>/<pub>_to_<sub> */
                    char ch_name[256];
                    snprintf(ch_name, sizeof(ch_name), "%s_%s_to_%s",
                             pub->topics[ai].topic, pub->name, sub->name);

                    /* Determine our role: publisher, subscriber, or both */
                    IpcRole role;
                    bool we_are_pub = (strcmp(pub->name, dm->my_name) == 0);
                    bool we_are_sub = (strcmp(sub->name, dm->my_name) == 0);

                    /* Create channel on our side only */
                    if (we_are_pub) {
                        role = IPC_ROLE_PUBLISHER;
                    } else if (we_are_sub) {
                        role = IPC_ROLE_SUBSCRIBER;
                    } else {
                        continue; /* Not our channel to create */
                    }

                    IpcChannel* ch = ipc_channel_open(ch_name, role, (uint32_t)max_depth);
                    if (ch) {
                        printf("[discovery:%s] opened IPC channel '%s' for %s (role=%s)\n",
                               dm->my_name, ch_name, pub->topics[ai].topic,
                               (role == IPC_ROLE_PUBLISHER) ? "pub" : "sub");
                        created++;
                        ipc_channel_start(ch);
                    }
                }
            }
        }
    }

    pthread_mutex_unlock(&dm->topo_mutex);

    printf("[discovery:%s] auto-created %d IPC channels\n", dm->my_name, created);
    return created;
}

int discovery_find_publishers(const TopologyGraph* g, const char* topic,
                              const NodeInfo** out, int max) {
    if (!g || !topic || !out || max <= 0) return 0;
    int count = 0;

    for (uint32_t i = 0; i < g->node_count && count < max; i++) {
        const NodeInfo* n = &g->nodes[i];
        if (!n->alive) continue;
        for (uint32_t j = 0; j < n->topic_count; j++) {
            if (strcmp(n->topics[j].topic, topic) == 0 &&
                (n->topics[j].capabilities & CAP_PUBLISHER)) {
                out[count++] = n;
                break;
            }
        }
    }
    return count;
}
