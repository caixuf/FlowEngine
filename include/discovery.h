#ifndef DISCOVERY_H
#define DISCOVERY_H

/**
 * @file discovery.h
 * @brief UDP 组播服务发现与拓扑管理 (FlowEngine Phase 3)
 *
 * 协议：
 *   组播地址 239.255.0.100:5500，消息格式：
 *     [magic:DISC(4B)|ver(1B)|type(1B)|name(64B)|pid(4B)|caps(1B)|
 *      topic_count(2B)|topics(N)|ipv4(4B)|port(2B)|crc32(4B)]
 *
 * 消息类型：HELLO(0)/HEARTBEAT(1)/GOODBYE(2)/QUERY(3)
 * 心跳间隔 2s，超时 10s 标记死亡
 *
 * 典型用法：
 *   DiscoveryManager* dm = discovery_create("perception_node", CAP_PUBLISHER);
 *   discovery_advertise(dm, "sensor/lidar", LIDARFRAME_TYPE_ID, CAP_PUBLISHER, 10.0);
 *   discovery_start(dm);
 *   // ... 其他节点自动发现 ...
 *   const TopologyGraph* g = discovery_get_topology(dm);
 *   char* json = discovery_export_json(dm);  // 可视化
 *   discovery_stop(dm);
 *   discovery_destroy(dm);
 */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── 常量 ────────────────────────────────────────────────── */

#define DISC_MAX_NODES            64
#define DISC_MAX_TOPICS_PER_NODE  32
#define DISC_NODE_NAME_LEN        64
#define DISC_TOPIC_NAME_LEN       64
#define DISC_MULTICAST_GROUP      "239.255.0.100"
#define DISC_MULTICAST_PORT       5500
#define DISC_HEARTBEAT_MS         2000
#define DISC_NODE_TIMEOUT_MS      10000
#define DISC_BEACON_MAX_SIZE      2048

/* ── 能力标志 ─────────────────────────────────────────────── */

typedef enum {
    CAP_PUBLISHER   = 1 << 0,
    CAP_SUBSCRIBER  = 1 << 1,
    CAP_SERVICE     = 1 << 2,   /**< Req/Reply service */
    CAP_FUSION      = 1 << 3,   /**< Sensor fusion node */
} NodeCapability;

/* ── Topic 广播 ───────────────────────────────────────────── */

typedef struct {
    char     topic[DISC_TOPIC_NAME_LEN];
    uint32_t type_id;          /**< FNV-1a type ID (0 = unknown) */
    uint8_t  capabilities;     /**< Bitmask of NodeCapability */
    double   frequency_hz;     /**< Expected rate (0 = unknown) */
} TopicAdvert;

/* ── 节点信息 ─────────────────────────────────────────────── */

typedef struct {
    char     name[DISC_NODE_NAME_LEN];
    uint32_t pid;
    uint8_t  capabilities;
    uint32_t topic_count;
    TopicAdvert topics[DISC_MAX_TOPICS_PER_NODE];
    uint64_t last_heartbeat_us;
    bool     alive;
    uint32_t ipv4_address;
    uint16_t unicast_port;
} NodeInfo;

/* ── 拓扑图 ───────────────────────────────────────────────── */

typedef struct {
    uint32_t node_count;
    NodeInfo nodes[DISC_MAX_NODES];
    uint8_t  relation[DISC_MAX_NODES][DISC_MAX_NODES];
    /* relation bits: 0x01=pub/sub match, 0x02=srv/cli match, 0x04=depends */
} TopologyGraph;

/* ── 变更回调 ──────────────────────────────────────────────── */

typedef void (*TopologyChangeCallback)(const TopologyGraph* graph,
                                       const NodeInfo* changed_node,
                                       bool joined,  /* true=joined, false=left/died */
                                       void* user_data);

/* ── DiscoveryManager ────────────────────────────────────── */

typedef struct DiscoveryManager DiscoveryManager;

/** 创建发现管理器 */
DiscoveryManager* discovery_create(const char* node_name, uint8_t capabilities);

/** 销毁（发送 GOODBYE + 清理） */
void discovery_destroy(DiscoveryManager* dm);

/** 启动发现服务（创建 socket + 心跳线程 + 接收线程） */
int discovery_start(DiscoveryManager* dm);

/** 停止发现服务 */
void discovery_stop(DiscoveryManager* dm);

/** 广播本节点提供的 topic */
int discovery_advertise(DiscoveryManager* dm, const char* topic,
                        uint32_t type_id, uint8_t capabilities,
                        double freq_hz);

/** 取消广播 topic */
int discovery_unadvertise(DiscoveryManager* dm, const char* topic);

/** 获取当前拓扑快照 */
const TopologyGraph* discovery_get_topology(DiscoveryManager* dm);

/** 导出拓扑为 JSON（调用者需 free） */
char* discovery_export_json(DiscoveryManager* dm);

/** 打印拓扑到 stdout */
void discovery_print_graph(DiscoveryManager* dm);

/** 注册拓扑变更回调 */
void discovery_set_change_callback(DiscoveryManager* dm,
                                   TopologyChangeCallback cb, void* user_data);

/**
 * 等待依赖节点上线（阻塞 + 超时）。
 * @param deps      依赖节点名数组
 * @param dep_count 依赖数量
 * @param timeout_ms 超时（毫秒）
 * @return 0 = 全部上线, -1 = 超时
 */
int discovery_wait_for_deps(DiscoveryManager* dm, const char** deps,
                            int dep_count, uint32_t timeout_ms);

/**
 * 自动为跨进程的 pub/sub 匹配创建 IPC 通道。
 * @return 创建的通道数, -1 = 失败
 */
int discovery_create_ipc_channels(DiscoveryManager* dm, uint32_t max_depth);

/**
 * 查找发布指定 topic 的节点。
 * @return 节点数量
 */
int discovery_find_publishers(const TopologyGraph* g, const char* topic,
                              const NodeInfo** out, int max);

#ifdef __cplusplus
}
#endif

#endif /* DISCOVERY_H */
