#ifndef MESSAGE_BUS_H
#define MESSAGE_BUS_H

/**
 * @file message_bus.h
 * @brief 轻量级进程内消息总线（通信总线第一轮）
 *
 * 支持两种通信模式：
 *  - Pub/Sub（发布/订阅）：异步、一对多、基于主题路由
 *  - Req/Reply（请求/回复）：同步、一对一、RPC风格
 *
 * 典型用法（发布）：
 *   MessageBus* bus = message_bus_create("adas");
 *   message_bus_subscribe(bus, "sensor/lidar", my_callback, ctx);
 *   message_bus_publish(bus, "sensor/lidar", "lidar_driver", &frame, sizeof(frame));
 *
 * 典型用法（请求）：
 *   Message reply;
 *   message_bus_request(bus, "service/path_planning", "ctrl",
 *                       &req, sizeof(req), &reply, 2000);
 */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── 常量 ───────────────────────────────────────────────── */

#define MSG_BUS_MAX_TOPIC_LEN    64
#define MSG_BUS_MAX_SENDER_LEN   64
#define MSG_BUS_MAX_DATA_SIZE    4096
#define MSG_BUS_MAX_TOPICS       64
#define MSG_BUS_MAX_SUBSCRIBERS  32
#define MSG_BUS_QUEUE_SIZE       256

/* ── 消息结构 ────────────────────────────────────────────── */

typedef enum {
    MSG_TYPE_PUBLISH = 0,   /**< 发布消息（异步） */
    MSG_TYPE_REQUEST,       /**< 请求消息（同步） */
    MSG_TYPE_REPLY          /**< 回复消息（内部使用） */
} MessageType;

typedef struct {
    char        topic[MSG_BUS_MAX_TOPIC_LEN];    /**< 主题 */
    char        sender[MSG_BUS_MAX_SENDER_LEN];  /**< 发送者名称 */
    uint32_t    msg_id;                           /**< 消息唯一ID */
    MessageType type;                             /**< 消息类型 */
    uint64_t    timestamp_us;                     /**< 时间戳（微秒，CLOCK_MONOTONIC） */
    uint32_t    data_size;                        /**< 有效数据字节数 */

    /* ── 类型安全序列化字段 (Phase 1) ─────────────────────── */
    uint32_t    type_id;          /**< FNV-1a hash 类型标识 (0=raw/unknown) */
    uint8_t     schema_version;   /**< schema 版本号 (0=unknown, 1=initial) */
    uint8_t     endian_marker;    /**< 字节序标记: 0x12=LE, 0x21=BE, 0=unknown */
    uint8_t     _reserved[6];     /**< 对齐保留，便于后续扩展 */

    uint8_t     data[MSG_BUS_MAX_DATA_SIZE];      /**< 负载数据 */
} Message;

/* ── 回调类型 ────────────────────────────────────────────── */

/**
 * 订阅回调：收到匹配主题消息时调用
 */
typedef void (*MessageCallback)(const Message* msg, void* user_data);

/**
 * 服务处理函数（req/reply 服务端）
 * @param request  收到的请求消息
 * @param reply    需要填充的回复消息（topic/msg_id 已预填）
 * @param user_data 注册时传入的用户数据
 */
typedef void (*ServiceHandler)(const Message* request, Message* reply, void* user_data);

/* ── 总线句柄（不透明类型）────────────────────────────────── */

typedef struct MessageBus MessageBus;

/* ── 生命周期 ────────────────────────────────────────────── */

/**
 * 创建消息总线并启动后台分发线程
 * @param bus_name 总线名称（仅用于调试标识）
 * @return 总线指针，失败返回 NULL
 */
MessageBus* message_bus_create(const char* bus_name);

/**
 * 停止总线并释放所有资源
 */
void message_bus_destroy(MessageBus* bus);

/* ── Pub/Sub ─────────────────────────────────────────────── */

/**
 * 发布消息到指定主题（非阻塞）
 * @param bus    总线
 * @param topic  主题名（如 "sensor/lidar"）
 * @param sender 发送者名称
 * @param data   数据指针（NULL 表示无负载）
 * @param size   数据字节数（<= MSG_BUS_MAX_DATA_SIZE）
 * @return 0 成功，-1 失败（队列已满或参数非法）
 */
int message_bus_publish(MessageBus* bus, const char* topic, const char* sender,
                        const void* data, uint32_t size);

/**
 * 订阅主题
 * @param topic    主题名；传 "*" 可订阅所有主题
 * @param callback 消息到达时的回调
 * @param user_data 透传给回调的用户指针
 * @return 0 成功，-1 失败
 */
int message_bus_subscribe(MessageBus* bus, const char* topic,
                          MessageCallback callback, void* user_data);

/**
 * 取消订阅
 * @param callback 与订阅时相同的回调指针
 * @return 0 成功，-1 未找到
 */
int message_bus_unsubscribe(MessageBus* bus, const char* topic, MessageCallback callback);

/**
 * 取消订阅（同时匹配 callback 和 user_data）
 * 用于多个订阅者使用相同回调函数但不同 user_data 的场景。
 * @return 0 成功，-1 未找到
 */
int message_bus_unsubscribe_ex(MessageBus* bus, const char* topic,
                               MessageCallback callback, void* user_data);

/* ── Req/Reply ───────────────────────────────────────────── */

/**
 * 发起同步请求并等待回复
 * @param topic      服务主题（与 register_service 对应）
 * @param sender     调用方名称
 * @param data       请求数据
 * @param size       数据大小
 * @param reply      输出参数：接收回复消息
 * @param timeout_ms 超时毫秒（0 = 永不超时）
 * @return 0 成功，-1 超时或错误
 */
int message_bus_request(MessageBus* bus, const char* topic, const char* sender,
                        const void* data, uint32_t size,
                        Message* reply, uint32_t timeout_ms);

/**
 * 注册服务（req/reply 服务端）
 * @return 0 成功，-1 主题已注册或参数非法
 */
int message_bus_register_service(MessageBus* bus, const char* topic,
                                  ServiceHandler handler, void* user_data);

/**
 * 注销服务
 * @return 0 成功，-1 未找到
 */
int message_bus_unregister_service(MessageBus* bus, const char* topic);

/* ── 零拷贝（Zero-Copy）────────────────────────────────── */

/**
 * 零拷贝订阅回调：直接接收原始数据指针，无内存拷贝开销
 *
 * @param topic        消息主题
 * @param sender       发送者名称
 * @param msg_id       消息唯一 ID
 * @param timestamp_us 时间戳（微秒，CLOCK_MONOTONIC）
 * @param data         指向发布者原始数据的指针（仅在回调执行期间有效！不可异步保存）
 * @param data_size    数据字节数
 * @param user_data    注册时传入的用户指针
 *
 * @warning data 指针的生命周期仅限于本次回调调用，严禁在回调返回后继续访问！
 */
typedef void (*ZeroCopyCallback)(const char*  topic,
                                  const char*  sender,
                                  uint32_t     msg_id,
                                  uint64_t     timestamp_us,
                                  const void*  data,
                                  uint32_t     data_size,
                                  void*        user_data);

/**
 * 注册零拷贝订阅者
 *
 * 当发布方调用 message_bus_publish_zero_copy() 时，匹配的零拷贝订阅者会在
 * 发布者线程中被同步调用，原始数据指针直接传递，不发生任何内存拷贝。
 *
 * @param topic    主题名；传 "*" 可订阅所有主题
 * @param callback 零拷贝消息到达时的回调
 * @param user_data 透传给回调的用户指针
 * @return 0 成功，-1 失败
 */
int message_bus_subscribe_zero_copy(MessageBus*      bus,
                                     const char*      topic,
                                     ZeroCopyCallback callback,
                                     void*            user_data);

/**
 * 取消零拷贝订阅
 * @return 0 成功，-1 未找到
 */
int message_bus_unsubscribe_zero_copy(MessageBus*      bus,
                                       const char*      topic,
                                       ZeroCopyCallback callback);

/**
 * 零拷贝发布（同步）
 *
 * 与 message_bus_publish() 的区别：
 *  - 零拷贝订阅者（通过 subscribe_zero_copy 注册）在调用者线程中被同步调用，
 *    data 指针直接传递，零内存拷贝，零队列延迟。
 *  - 普通 copy-based 订阅者仍然通过异步队列收到通知（需一次拷贝）。
 *
 * 适用场景：大块数据（点云、图像帧等）对延迟敏感的场景。
 *
 * @param data      指向待发布数据的指针（函数返回前始终有效）
 * @param data_size 数据字节数（<= MSG_BUS_MAX_DATA_SIZE）
 * @return 成功通知的零拷贝订阅者数量，-1 表示参数非法
 */
int message_bus_publish_zero_copy(MessageBus*  bus,
                                   const char*  topic,
                                   const char*  sender,
                                   const void*  data,
                                   uint32_t     data_size);

/* ── 统计 ────────────────────────────────────────────────── */

/**
 * 获取总线运行统计
 * @param published_count  已投入队列的消息总数（输出，可为 NULL）
 * @param delivered_count  已成功投递给订阅者的次数（输出，可为 NULL）
 * @param dropped_count    因队列满而丢弃的消息数（输出，可为 NULL）
 */
void message_bus_get_stats(MessageBus* bus,
                           uint64_t* published_count,
                           uint64_t* delivered_count,
                           uint64_t* dropped_count);

/**
 * 获取零拷贝专项统计
 * @param zc_published  零拷贝发布调用次数（输出，可为 NULL）
 * @param zc_delivered  零拷贝成功投递给订阅者的次数（输出，可为 NULL）
 */
void message_bus_get_zc_stats(MessageBus* bus,
                               uint64_t*   zc_published,
                               uint64_t*   zc_delivered);

/* ── QoS & Per-Topic Statistics ─────────────────────────── */

/** 可靠性 */
typedef enum {
    QOS_BEST_EFFORT = 0,  /**< 尽力传输（允许丢帧，遵循 QosPolicy） */
    QOS_RELIABLE    = 1,  /**< 可靠传输（保证送达，自动升级为 QOS_BLOCK，不丢帧） */
} QosReliability;

/** 队列溢出策略 */
typedef enum {
    QOS_DROP_OLDEST = 0,  /**< 丢弃最旧消息（默认） */
    QOS_DROP_LATEST = 1,  /**< 丢弃最新消息（保留旧数据） */
    QOS_BLOCK       = 2,  /**< 阻塞发布者直到队列有空间 */
} QosPolicy;

/** 传输类型 */
typedef enum {
    TRANSPORT_INTRA = 0,  /**< 进程内总线 (零拷贝) */
    TRANSPORT_SHM   = 1,  /**< 共享内存 IPC */
    TRANSPORT_TCP   = 2,  /**< TCP 网络 */
    TRANSPORT_DDS   = 3,  /**< DDS 协议 (预留) */
} TopicTransport;

/** Topic QoS 配置 (DDS 风格) */
typedef struct {
    QosReliability reliability;  /**< 可靠性: best_effort / reliable */
    uint32_t       depth;        /**< 队列深度 */
    QosPolicy      policy;       /**< 溢出策略 */
    uint32_t       deadline_ms;  /**< 截止时间 (ms)，超时告警。0=不设置 */
    uint32_t       lifespan_ms;  /**< 消息生存期 (ms)，过期丢弃。0=永不过期 */
    TopicTransport transport;    /**< 传输方式 */
} TopicQos;

/** Per-topic 统计 */
typedef struct {
    char      topic[MSG_BUS_MAX_TOPIC_LEN];
    uint64_t  publish_count;    /**< 发布次数 */
    uint64_t  deliver_count;    /**< 投递次数 */
    uint64_t  drop_count;       /**< 丢弃次数 */
    uint64_t  deadline_violations; /**< deadline 超时违规次数（端到端分发延迟超过 deadline_ms）*/
    uint64_t  total_latency_us; /**< 累计延迟（用于计算平均值） */
    uint64_t  min_latency_us;   /**< 最小延迟 */
    uint64_t  max_latency_us;   /**< 最大延迟 */
    uint64_t  last_publish_us;  /**< 最近发布时间 */
    uint32_t  subscriber_count; /**< 当前订阅者数 */
    double    frequency_hz;     /**< 估算发布频率 */
    TopicQos  qos;              /**< 当前 QoS 配置 */
} TopicStats;

/**
 * 为指定 topic 设置 QoS 策略。
 * 必须在首次 publish 之前调用以生效。
 * @return 0 成功, -1 topic 名非法
 */
int message_bus_set_topic_qos(MessageBus* bus, const char* topic,
                              const TopicQos* qos);

/** 获取 topic 的 QoS 配置 */
const TopicQos* message_bus_get_topic_qos(MessageBus* bus, const char* topic);

/**
 * 获取 topic 的运行时统计。
 * @param stats 输出缓冲区
 * @return 0 成功, -1 topic 不存在
 */
int message_bus_get_topic_stats(MessageBus* bus, const char* topic,
                                TopicStats* stats);

/**
 * 列出总线上所有活跃 topic。
 * @param topics 输出缓冲区（每个 64 字节）
 * @param max    最多返回数量
 * @return 实际 topic 数量
 */
int message_bus_list_topics(MessageBus* bus, char topics[][64], int max);

/**
 * 获取所有 topic 的统计摘要（用于 flowctl/监控）。
 * @param stats 输出数组
 * @param max   最多返回数量
 * @return 实际 topic 数量
 */
int message_bus_get_all_topic_stats(MessageBus* bus, TopicStats* stats, int max);

/* ── Topic Remap ─────────────────────────────────────────── */

/**
 * 添加话题重映射规则：发布到 `from` 的消息将被路由到 `to`。
 *
 * 常用于多节点复用同一插件、测试时将 topic 重定向，或兼容命名迁移。
 * 重映射在 publish 路径上透明执行，订阅者按 `to` 名称注册即可。
 *
 * @param from  原始 topic 名（发布者使用的名称）
 * @param to    目标 topic 名（订阅者注册的名称）
 * @return 0 成功，-1 参数非法或表已满
 */
int message_bus_add_remap(MessageBus* bus, const char* from, const char* to);

/**
 * 删除重映射规则。
 * @return 0 成功，-1 未找到
 */
int message_bus_remove_remap(MessageBus* bus, const char* from);

/**
 * 查询 topic 的实际路由目标（若无重映射则返回原名）。
 * @param out_topic 输出缓冲区（至少 MSG_BUS_MAX_TOPIC_LEN 字节）
 */
void message_bus_resolve_topic(MessageBus* bus, const char* topic, char* out_topic);

#ifdef __cplusplus
}
#endif

#endif /* MESSAGE_BUS_H */
