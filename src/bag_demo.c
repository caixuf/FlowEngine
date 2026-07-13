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
#include "dashboard_bridge.h"
#include "ipc_channel.h"
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

/* ── Global state ────────────────────────────────────── */

static volatile int g_running = 1;
static void sighandler(int sig) { (void)sig; g_running = 0; }

/* IPC dashboard bridge publisher for --replay mode */
static IpcChannel* g_replay_dashboard_ch = NULL;

/* ── Publisher threads ───────────────────────────────── */

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
    uint64_t now = clock_now_us();
    printf("[PLAY] ts=%-20lu topic=%-20s size=%u\n",
           (unsigned long)msg->timestamp_us, msg->topic, msg->data_size);
}

/* ── Replay + Dashboard subscriber ────────────────────── */

static void on_replay_state(const Message* msg, void* user_data) {
    (void)user_data;
    if (!msg || msg->data_size < 2) return;
    /* Only handle JSON text data — binary struct data would cause
     * strstr to overread the buffer beyond its bounds. */
    if (msg->data[0] != '{') return;

    /* Build dashboard JSON in memory so it can be sent to both
     * /tmp/flow_topology.json (foxglove_bridge.py) and the IPC
     * dashboard bridge (flowmond — makes the browser visualization update). */
    char dash_buf[8192];
    int off = 0;

#define RBUF_APPEND(fmt, ...) \
    do { \
        if (off >= 0 && (size_t)off < sizeof(dash_buf)) \
            off += snprintf(dash_buf + off, sizeof(dash_buf) - (size_t)off, \
                            fmt, ##__VA_ARGS__); \
    } while (0)

    const char* d = (const char*)msg->data;
    const char* p;
    p = strstr(d, "\"spd\":");   double spd = p ? atof(p + 6) : 0;
    p = strstr(d, "\"tgt\":");   double tgt = p ? atof(p + 6) : 0;
    p = strstr(d, "\"x\":");     double x   = p ? atof(p + 4) : 0;
    p = strstr(d, "\"y\":");     double ey  = p ? atof(p + 5) : -1.75;
    p = strstr(d, "\"hdg\":");   double hdg = p ? atof(p + 7) : 0.0;
    p = strstr(d, "\"st\":");    double st  = p ? atof(p + 6) : 0.0;
    p = strstr(d, "\"n_obs\":"); int n_obs  = p ? atoi(p + 8) : 0;

    RBUF_APPEND("{\"self\":\"replay\",\"timestamp\":0,\"nodes\":[],"
        "\"metrics\":{"
        "\"bus\":{\"published\":0,\"delivered\":0,\"dropped\":0},"
        "\"transport\":{\"local_pub\":0,\"remote_pub\":0},"
        "\"scheduler\":{\"tasks\":0,\"mode\":\"REPLAY\"},"
        "\"latency\":{\"avg_us\":0,\"p50_us\":0,\"p99_us\":0},"
        "\"topics\":[]},"
        "\"vehicle\":{\"speed\":%.1f,\"target_speed\":%.1f,\"x\":%.1f},"
        "\"scene\":{\"ego\":{"
        "\"x\":%.2f,\"y\":%.2f,\"heading\":%.3f,\"speed\":%.2f,\"steer\":%.3f},"
        "\"lane\":{\"width\":3.5,\"count\":2},\"obstacles\":[",
        spd, tgt, x, x, ey, hdg, spd, st);

    for (int i = 0; i < n_obs && i < 16; i++) {
        char kx[16]; snprintf(kx, 16, "\"ox%d\":", i);
        char ky[16]; snprintf(ky, 16, "\"oy%d\":", i);
        double ox = 0, oy = 0;
        const char* pp = strstr(d, kx); if (pp) ox = atof(pp + strlen(kx));
        pp = strstr(d, ky); if (pp) oy = atof(pp + strlen(ky));
        RBUF_APPEND("%s{\"x\":%.2f,\"y\":%.2f}", i > 0 ? "," : "", ox, oy);
    }

    RBUF_APPEND("]}}");
#undef RBUF_APPEND

    /* Write to state file for foxglove_bridge.py */
    const char* jf_path = "/tmp/flow_topology.json";
    FILE* jf = fopen(jf_path, "w");
    if (jf) { fputs(dash_buf, jf); fclose(jf); }

    /* Publish via IPC dashboard bridge so flowmond updates the browser.
     * Skip if JSON was truncated due to buffer overflow. */
    if (g_replay_dashboard_ch && off > 0 && off < (int)sizeof(dash_buf))
        dashboard_bridge_publish(g_replay_dashboard_ch, dash_buf, (size_t)off);
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
        fprintf(stderr, "      %s --replay <file.bag>  # 回放+输出dashboard JSON\n", argv[0]);
        return 1;
    }

    if (strcmp(argv[1], "--record") == 0) return do_record(argv[2]);
    if (strcmp(argv[1], "--play")   == 0) return do_play  (argv[2]);
    if (strcmp(argv[1], "--replay") == 0) {
        /* Replay mode with dashboard JSON output.
         * Subscribes to vehicle/state during replay, writes
         * /tmp/flow_topology.json for foxglove_bridge.py, and also
         * publishes via the IPC dashboard bridge so flowmond updates
         * the browser visualization in real time. */
        printf("Replay+Dashboard mode: %s\n", argv[2]);
        MessageBus* bus = message_bus_create("replay_bus");
        BagReader*  r   = bag_reader_open(argv[2]);
        if (!bus || !r) { fprintf(stderr, "init failed\n"); return 1; }

        /* Open IPC dashboard bridge publisher (flowmond subscribes to this) */
        g_replay_dashboard_ch = dashboard_bridge_publisher_open();
        if (!g_replay_dashboard_ch)
            fprintf(stderr, "[replay] dashboard bridge unavailable — "
                    "flowmond visualization will not update\n");

        /* Subscribe vehicle/state: write dashboard JSON on each message */
        message_bus_subscribe(bus, "vehicle/state", on_replay_state, NULL);
        clock_set_sim_mode(true);
        int count = bag_reader_play(r, bus, 1.0f);
        printf("Replay done: %d messages\n", count);
        clock_set_sim_mode(false);

        if (g_replay_dashboard_ch) {
            ipc_channel_close(g_replay_dashboard_ch);
            g_replay_dashboard_ch = NULL;
        }
        bag_reader_close(r);
        message_bus_destroy(bus);
        return 0;
    }

    fprintf(stderr, "未知选项: %s\n", argv[1]);
    return 1;
}
