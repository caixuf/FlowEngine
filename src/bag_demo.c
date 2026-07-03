/**
 * bag_demo.c — 消息录制与回放演示（Step 3 + Step 5）
 *
 * 用法：
 *   ./bag_demo --record out.bag   # 录制 3 秒
 *   ./bag_demo --play   out.bag   # 以录制时间回放
 */

#include "message_bus.h"
#include "bag.h"
#include "clock_service.h"
#include "msg_schema.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <signal.h>
#include <math.h>

/* ── Demo data types ─────────────────────────────────── */

typedef struct {
    float    x, y, z;
    uint32_t point_count;
    uint32_t frame_id;
} LidarFrame;

typedef struct {
    double latitude, longitude;
    float  speed_mps;
} GpsData;

/* ── Publisher threads ───────────────────────────────── */

static volatile int g_running = 1;
static void sighandler(int sig) { (void)sig; g_running = 0; }

static void* lidar_pub_thread(void* arg) {
    MessageBus* bus = (MessageBus*)arg;
    uint32_t fid = 0;
    while (g_running) {
        LidarFrame f = { .x = fid * 0.1f, .y = fid * 0.2f, .z = 0,
                         .point_count = 64000, .frame_id = fid };
        MSG_CHECK_SIZE("sensor/lidar", sizeof(LidarFrame));
        message_bus_publish(bus, "sensor/lidar", "lidar_drv", &f, sizeof(f));
        fid++;
        usleep(100000);   /* 10 Hz */
    }
    return NULL;
}

static void* gps_pub_thread(void* arg) {
    MessageBus* bus = (MessageBus*)arg;
    uint32_t tick = 0;
    while (g_running) {
        GpsData g = { .latitude  = 39.9042 + tick * 0.00001,
                      .longitude = 116.4074 + tick * 0.00002,
                      .speed_mps = 8.3f };
        MSG_CHECK_SIZE("sensor/gps", sizeof(GpsData));
        message_bus_publish(bus, "sensor/gps", "gps_drv", &g, sizeof(g));
        tick++;
        usleep(200000);   /* 5 Hz */
    }
    return NULL;
}

/* ── Playback subscriber ─────────────────────────────── */

static void on_playback(const Message* msg, void* user_data) {
    (void)user_data;
    uint64_t now = clock_now_us();   /* sim_mode 下返回录制时间戳 */
    printf("[PLAY] ts=%-20lu topic=%-20s size=%u  (sim_now=%lu)\n",
           (unsigned long)msg->timestamp_us, msg->topic,
           msg->data_size, (unsigned long)now);
}

/* ── Record mode ─────────────────────────────────────── */

static int do_record(const char* path) {
    printf("录制模式: 输出文件 = %s\n", path);

    /* Register type schemas */
    MSG_REGISTER_TYPE("sensor/lidar", LidarFrame);
    MSG_REGISTER_TYPE("sensor/gps",   GpsData);

    MessageBus* bus = message_bus_create("bag_bus");
    BagWriter*  w   = bag_writer_open(path);
    if (!bus || !w) {
        fprintf(stderr, "初始化失败\n");
        if (bus) message_bus_destroy(bus);
        if (w)   bag_writer_close(w);
        return 1;
    }

    bag_writer_attach(w, bus);   /* 订阅 "*"，自动录制所有消息 */

    pthread_t t1, t2;
    pthread_create(&t1, NULL, lidar_pub_thread, bus);
    pthread_create(&t2, NULL, gps_pub_thread,   bus);

    printf("录制中... 3 秒后自动停止（或按 Ctrl+C）\n");
    for (int i = 0; i < 30 && g_running; i++) usleep(100000);
    g_running = 0;

    pthread_join(t1, NULL);
    pthread_join(t2, NULL);

    bag_writer_close(w);
    message_bus_destroy(bus);
    printf("录制完成。\n");
    return 0;
}

/* ── Playback mode ───────────────────────────────────── */

static int do_play(const char* path) {
    printf("回放模式: 输入文件 = %s\n", path);

    MessageBus* bus = message_bus_create("play_bus");
    BagReader*  r   = bag_reader_open(path);
    if (!bus || !r) {
        fprintf(stderr, "初始化失败\n");
        if (bus) message_bus_destroy(bus);
        if (r)   bag_reader_close(r);
        return 1;
    }

    message_bus_subscribe(bus, "*", on_playback, NULL);

    /* 启用仿真时钟，回放时 clock_now_us() 返回录制时间戳 */
    clock_set_sim_mode(true);

    printf("开始回放（1× 速度）...\n");
    int count = bag_reader_play(r, bus, 1.0f);
    printf("回放完成，共 %d 条消息\n", count);

    clock_set_sim_mode(false);

    bag_reader_close(r);
    message_bus_destroy(bus);
    return 0;
}

/* ── main ────────────────────────────────────────────── */

int main(int argc, char* argv[]) {
    signal(SIGINT,  sighandler);
    signal(SIGTERM, sighandler);

    if (argc < 3) {
        fprintf(stderr, "用法: %s --record <file.bag>\n", argv[0]);
        fprintf(stderr, "      %s --play   <file.bag>\n", argv[0]);
        return 1;
    }

    if (strcmp(argv[1], "--record") == 0) return do_record(argv[2]);
    if (strcmp(argv[1], "--play")   == 0) return do_play  (argv[2]);

    fprintf(stderr, "未知选项: %s\n", argv[1]);
    return 1;
}
