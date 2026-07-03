/**
 * bus_demo.c — 通信总线演示程序
 *
 * 演示内容：
 *  1. 创建消息总线
 *  2. 多个订阅者订阅不同主题（激光雷达、GPS）
 *  3. 通配符订阅者监控所有消息
 *  4. 发布者线程模拟传感器数据流
 *  5. req/reply 模式调用路径规划服务
 *  6. 打印总线统计信息
 */

#include "message_bus.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <signal.h>
#include <math.h>

/* ── 业务数据结构 ────────────────────────────────────────── */

typedef struct {
    float    x, y, z;          /* 点云中心坐标（米） */
    float    intensity;         /* 平均反射强度 */
    uint32_t point_count;       /* 点云数量 */
    uint32_t frame_id;          /* 帧序号 */
} LidarFrame;

typedef struct {
    double latitude;            /* 纬度（度） */
    double longitude;           /* 经度（度） */
    float  speed_mps;           /* 速度（m/s） */
    float  heading_deg;         /* 航向角（度） */
} GpsData;

typedef struct {
    float origin_x, origin_y;   /* 起点（米） */
    float target_x, target_y;   /* 终点（米） */
} PathRequest;

typedef struct {
    float distance_m;           /* 路径总长度（米） */
    float eta_seconds;          /* 预计到达时间（秒） */
    int   waypoint_count;       /* 路径点数量 */
} PathResponse;

/* ── 全局状态 ─────────────────────────────────────────────── */

static volatile int g_running = 1;
static MessageBus*  g_bus     = NULL;

static void signal_handler(int sig) {
    (void)sig;
    g_running = 0;
}

/* ══════════════════════════════════════════════════════════
 * 订阅者回调
 * ══════════════════════════════════════════════════════════ */

static void on_lidar(const Message* msg, void* user_data) {
    const char* module = (const char*)user_data;
    if (msg->data_size != sizeof(LidarFrame)) return;

    const LidarFrame* f = (const LidarFrame*)msg->data;
    printf("[%s] 激光雷达帧 #%u: 点云=%u, 中心=(%.1f, %.1f, %.1f)\n",
           module, f->frame_id, f->point_count, f->x, f->y, f->z);
}

static void on_gps(const Message* msg, void* user_data) {
    const char* module = (const char*)user_data;
    if (msg->data_size != sizeof(GpsData)) return;

    const GpsData* g = (const GpsData*)msg->data;
    printf("[%s] GPS: 纬度=%.6f, 经度=%.6f, 速度=%.1f km/h\n",
           module, g->latitude, g->longitude, g->speed_mps * 3.6f);
}

static void on_all(const Message* msg, void* user_data) {
    (void)user_data;
    printf("[监控] topic=%-25s sender=%-15s id=%-6u size=%u bytes\n",
           msg->topic, msg->sender, msg->msg_id, msg->data_size);
}

/* ══════════════════════════════════════════════════════════
 * 服务处理函数（req/reply）
 * ══════════════════════════════════════════════════════════ */

static void path_planning_service(const Message* req, Message* reply, void* user_data) {
    (void)user_data;
    if (req->data_size != sizeof(PathRequest)) return;

    const PathRequest* r = (const PathRequest*)req->data;
    float dx = r->target_x - r->origin_x;
    float dy = r->target_y - r->origin_y;

    PathResponse resp;
    resp.distance_m    = sqrtf(dx * dx + dy * dy);
    resp.eta_seconds   = resp.distance_m / 10.0f;   /* 假设 10 m/s */
    resp.waypoint_count = (int)(resp.distance_m / 5.0f) + 1;

    printf("[路径规划] 计算完成: %.1f m, ETA=%.1f s, 路径点=%d\n",
           resp.distance_m, resp.eta_seconds, resp.waypoint_count);

    memcpy(reply->data, &resp, sizeof(resp));
    reply->data_size = sizeof(resp);
    strncpy(reply->sender, "path_planner", sizeof(reply->sender) - 1);
}

/* ══════════════════════════════════════════════════════════
 * 发布者线程
 * ══════════════════════════════════════════════════════════ */

static void* lidar_thread(void* arg) {
    MessageBus* bus = (MessageBus*)arg;
    uint32_t frame_id = 0;

    printf("[激光雷达驱动] 启动，发布频率 10 Hz\n");
    while (g_running) {
        LidarFrame f = {
            .x           = (float)(frame_id % 100) * 0.1f,
            .y           = (float)(frame_id % 50)  * 0.2f,
            .z           = 0.0f,
            .intensity   = 0.75f,
            .point_count = 64000 + frame_id % 1000,
            .frame_id    = frame_id
        };
        message_bus_publish(bus, "sensor/lidar", "lidar_driver", &f, sizeof(f));
        frame_id++;
        usleep(100000);   /* 10 Hz */
    }
    printf("[激光雷达驱动] 停止\n");
    return NULL;
}

static void* gps_thread(void* arg) {
    MessageBus* bus = (MessageBus*)arg;
    uint32_t tick = 0;

    printf("[GPS驱动] 启动，发布频率 5 Hz\n");
    while (g_running) {
        GpsData g = {
            .latitude    = 39.9042 + tick * 0.00001,
            .longitude   = 116.4074 + tick * 0.00002,
            .speed_mps   = 8.3f + (float)(tick % 5) * 0.5f,
            .heading_deg = (float)((tick * 3) % 360)
        };
        message_bus_publish(bus, "sensor/gps", "gps_driver", &g, sizeof(g));
        tick++;
        usleep(200000);   /* 5 Hz */
    }
    printf("[GPS驱动] 停止\n");
    return NULL;
}

/* ══════════════════════════════════════════════════════════
 * 主程序
 * ══════════════════════════════════════════════════════════ */

int main(void) {
    printf("╔══════════════════════════════════════╗\n");
    printf("║    通信总线演示程序 (bus_demo)        ║\n");
    printf("╚══════════════════════════════════════╝\n\n");

    signal(SIGINT,  signal_handler);
    signal(SIGTERM, signal_handler);

    /* 1. 创建总线 */
    g_bus = message_bus_create("adas_bus");
    if (!g_bus) { fprintf(stderr, "创建总线失败\n"); return 1; }
    printf("[总线] 创建成功: adas_bus\n\n");

    /* 2. 注册订阅者 */
    message_bus_subscribe(g_bus, "sensor/lidar", on_lidar, (void*)"感知模块");
    message_bus_subscribe(g_bus, "sensor/gps",   on_gps,   (void*)"定位模块");
    message_bus_subscribe(g_bus, "*",            on_all,   NULL);
    printf("[总线] 已注册订阅者: sensor/lidar, sensor/gps, *(通配符)\n");

    /* 3. 注册服务 */
    message_bus_register_service(g_bus, "service/path_planning",
                                 path_planning_service, NULL);
    printf("[总线] 已注册服务: service/path_planning\n\n");

    /* 4. 启动传感器发布线程 */
    pthread_t t_lidar, t_gps;
    if (pthread_create(&t_lidar, NULL, lidar_thread, g_bus) != 0) {
        fprintf(stderr, "创建激光雷达线程失败\n");
        message_bus_destroy(g_bus);
        return 1;
    }
    if (pthread_create(&t_gps, NULL, gps_thread, g_bus) != 0) {
        fprintf(stderr, "创建GPS线程失败\n");
        g_running = 0;
        pthread_join(t_lidar, NULL);
        message_bus_destroy(g_bus);
        return 1;
    }

    /* 等待收到几帧后演示 req/reply */
    sleep(1);

    /* 5. Req/Reply 演示：路径规划 */
    printf("\n────────── Req/Reply 演示 ──────────\n");
    PathRequest path_req = { .origin_x = 0, .origin_y = 0,
                             .target_x = 100, .target_y = 200 };
    Message reply;
    int ret = message_bus_request(g_bus, "service/path_planning", "main_ctrl",
                                  &path_req, sizeof(path_req), &reply, 2000);
    if (ret == 0 && reply.data_size == sizeof(PathResponse)) {
        const PathResponse* r = (const PathResponse*)reply.data;
        printf("[主控] 路径规划结果: 距离=%.1f m, ETA=%.1f s, 路径点=%d\n",
               r->distance_m, r->eta_seconds, r->waypoint_count);
    } else {
        printf("[主控] 路径规划请求失败或超时 (ret=%d)\n", ret);
    }
    printf("────────────────────────────────────\n\n");

    /* 运行 3 秒后退出 */
    sleep(3);
    g_running = 0;

    pthread_join(t_lidar, NULL);
    pthread_join(t_gps,   NULL);

    /* 6. 打印统计 */
    uint64_t pub, del, drop;
    message_bus_get_stats(g_bus, &pub, &del, &drop);
    printf("\n════ 总线统计 ════\n");
    printf("  发布消息: %lu\n", (unsigned long)pub);
    printf("  投递次数: %lu\n", (unsigned long)del);
    printf("  丢弃消息: %lu\n", (unsigned long)drop);

    /* 清理 */
    message_bus_destroy(g_bus);
    printf("\n[总线] 已销毁，程序结束\n");
    return 0;
}
