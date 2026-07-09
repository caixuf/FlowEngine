/**
 * bag_demo.c / flow_bag — Bag 录制、回放、SIL仿真
 *
 * 用法：
 *   flow_bag --record out.bag                # 录制
 *   flow_bag --play   in.bag                 # 全部topic回放
 *   flow_bag --replay in.bag                 # 回放+仪表盘JSON
 *   flow_bag --sil    in.bag pipeline.json   # SIL仿真(传感器数据→算法pipeline)
 */

#include "message_bus.h"
#include "bag.h"
#include "config_manager.h"
#include "clock_service.h"
#include "msg_schema.h"
#include "node_plugin.h"
#include "transport.h"
#include "discovery.h"
#include "scheduler.h"
#include "logger.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <signal.h>
#include <sys/wait.h>
#include <dlfcn.h>
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
    uint64_t now = clock_now_us();
    printf("[PLAY] ts=%-20lu topic=%-20s size=%u\n",
           (unsigned long)msg->timestamp_us, msg->topic, msg->data_size);
}

/* ── Replay + Dashboard subscriber ────────────────────── */

static void on_replay_state(const Message* msg, void* user_data) {
    (void)user_data;
    if (!msg || !msg->data) return;
    /* Write vehicle/state as dashboard JSON — flowmond reads this file */
    const char* jf_path = "/tmp/flow_topology.json";
    FILE* jf = fopen(jf_path, "w");
    if (!jf) return;
    fprintf(jf, "{\"self\":\"replay\",\"timestamp\":0,\"nodes\":[],");
    fprintf(jf, "\"metrics\":{\"bus\":{\"published\":0,\"delivered\":0,\"dropped\":0},");
    fprintf(jf, "\"transport\":{\"local_pub\":0,\"remote_pub\":0},");
    fprintf(jf, "\"scheduler\":{\"tasks\":0,\"mode\":\"REPLAY\"},");
    fprintf(jf, "\"latency\":{\"avg_us\":0,\"p50_us\":0,\"p99_us\":0},");
    fprintf(jf, "\"topics\":[]},");
    /* Embed the raw vehicle state into the dashboard JSON */
    fprintf(jf, "\"vehicle\":{");
    const char* d = (const char*)msg->data;
    /* Extract speed from JSON */
    const char* p = strstr(d, "\"spd\":");
    double spd = p ? atof(p + 6) : 0;
    p = strstr(d, "\"tgt\":");
    double tgt = p ? atof(p + 6) : 0;
    p = strstr(d, "\"x\":");
    double x = p ? atof(p + 4) : 0;
    fprintf(jf, "\"speed\":%.1f,\"target_speed\":%.1f,\"x\":%.1f}", spd, tgt, x);
    fprintf(jf, ",\"scene\":{\"ego\":{\"x\":%.1f,\"y\":-1.75,\"heading\":0,\"speed\":%.1f,\"steer\":0}}}", x, spd);
    fclose(jf);
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

/* ── SIL 仿真: bag传感器数据 → 算法pipeline ──────────────── */

static int do_sil(const char* bagfile, const char* pipeline_cfg) {
    printf("SIL仿真: bag=%s  config=%s\n", bagfile, pipeline_cfg);

    /* Load pipeline config to know which nodes to run */
    LauncherConfig* cfg = config_load(pipeline_cfg);
    if (!cfg) { fprintf(stderr, "failed to load %s\n", pipeline_cfg); return 1; }

    BagReader* r = bag_reader_open(bagfile);
    if (!r) { fprintf(stderr, "cannot open %s\n", bagfile); config_free(cfg); return 1; }

    /* Init infrastructure */
    log_init(LOG_INFO, NULL);
    MessageBus*       bus       = message_bus_create("sil_bus");
    DiscoveryManager* discovery = discovery_create("sil", CAP_PUBLISHER | CAP_SUBSCRIBER);
    discovery_start(discovery);
    Transport* transport = transport_create(bus, discovery, TRANSPORT_LOCAL);
    transport_start(transport);
    SchedulerConfig scfg = SCHEDULER_CONFIG_DEFAULT;
    scfg.mode = SCHEDULER_MODE_CHOREO;
    Scheduler* scheduler = scheduler_create(&scfg);
    scheduler_set_choreo_bus(scheduler, bus);
    scheduler_start(scheduler);

    /* Load pipeline nodes from config, skip sim_world + sensor_model
     * (bag provides sensor/lidar, sensor/gps, vehicle/state). */
    #define SIL_MAX_NODES 8
    NodePlugin* plugins[SIL_MAX_NODES] = {NULL};
    void* handles[SIL_MAX_NODES] = {NULL};
    int n_loaded = 0;

    for (int i = 0; i < cfg->process_count && n_loaded < SIL_MAX_NODES; i++) {
        ProcessConfig* pc = &cfg->processes[i];
        /* Skip data source nodes — bag provides their output */
        if (strcmp(pc->name, "sim_world") == 0 || strcmp(pc->name, "sensor_model") == 0)
            continue;

        handles[n_loaded] = dlopen(pc->library_path, RTLD_LAZY | RTLD_GLOBAL);
        if (!handles[n_loaded]) {
            char alt[512];
            const char* bn = strrchr(pc->library_path, '/');
            bn = bn ? bn + 1 : pc->library_path;
            snprintf(alt, sizeof(alt), "build/lib/%s", bn);
            handles[n_loaded] = dlopen(alt, RTLD_LAZY | RTLD_GLOBAL);
        }
        if (!handles[n_loaded]) {
            fprintf(stderr, "  skip %s: %s\n", pc->name, dlerror());
            continue;
        }
        NodeGetPluginFn get_fn = (NodeGetPluginFn)dlsym(handles[n_loaded], NODE_PLUGIN_SYMBOL);
        if (!get_fn) { dlclose(handles[n_loaded]); handles[n_loaded] = NULL; continue; }
        plugins[n_loaded] = get_fn();
        if (!plugins[n_loaded]) { dlclose(handles[n_loaded]); handles[n_loaded] = NULL; continue; }

        if (plugins[n_loaded]->init(bus, transport, discovery, scheduler,
            pc->params[0] ? pc->params : NULL) == 0) {
            plugins[n_loaded]->start();
            printf("  [%d] %s\n", n_loaded + 1, plugins[n_loaded]->name);
            n_loaded++;
            usleep(50000);
        }
    }
    config_free(cfg);
    printf("SIL pipeline: %d nodes (bag provides sensor data)\n", n_loaded);

    /* Replay bag → pipeline runs on real data */
    uint64_t msg_count = 0, dur_us = 0;
    bag_reader_info(r, &msg_count, &dur_us);
    printf("Replaying %llu msgs (%.1fs)...\n", (unsigned long long)msg_count, dur_us / 1e6);
    clock_set_sim_mode(true);
    int replayed = bag_reader_play(r, bus, 1.0f);
    clock_set_sim_mode(false);
    printf("SIL done: %d msgs replayed\n", replayed);

    sleep(1);
    for (int i = n_loaded - 1; i >= 0; i--) {
        if (plugins[i]) { plugins[i]->stop(); plugins[i]->cleanup(); }
    }
    bag_reader_close(r);
    scheduler_stop(scheduler); scheduler_destroy(scheduler);
    transport_stop(transport); transport_destroy(transport);
    discovery_stop(discovery); discovery_destroy(discovery);
    message_bus_destroy(bus);
    for (int i = 0; i < n_loaded; i++) if (handles[i]) dlclose(handles[i]);
    log_shutdown();
    return 0;
}

/* ── main ────────────────────────────────────────────── */

int main(int argc, char* argv[]) {
    signal(SIGINT,  sighandler);
    signal(SIGTERM, sighandler);

    const char* cmd      = NULL;
    const char* bagfile  = NULL;
    const char* sil_cfg  = NULL;
    int dashboard_mode   = 0;

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--dashboard") == 0) dashboard_mode = 1;
        else if (!cmd) cmd = argv[i];
        else if (!bagfile) bagfile = argv[i];
        else if (!sil_cfg) sil_cfg = argv[i];
    }

    if (!cmd || !bagfile) {
        fprintf(stderr, "用法: %s [--dashboard] --record <file.bag>\n", argv[0]);
        fprintf(stderr, "      %s [--dashboard] --play   <file.bag>\n", argv[0]);
        fprintf(stderr, "      %s [--dashboard] --replay <file.bag>\n", argv[0]);
        fprintf(stderr, "      %s --sil <file.bag> <pipeline.json>\n", argv[0]);
        return 1;
    }

    /* Auto-start flowmond if --dashboard flag given */
    pid_t flowmond_pid = 0;
    if (dashboard_mode) {
        flowmond_pid = fork();
        if (flowmond_pid == 0) {
            /* Child: start flowmond (redirect output to /dev/null) */
            freopen("/dev/null", "w", stdout);
            freopen("/dev/null", "w", stderr);
            execlp("flowmond", "flowmond", "--port", "8800", NULL);
            /* If execlp fails, try relative path */
            execl("./build/bin/flowmond", "flowmond", "--port", "8800", NULL);
            _exit(1);
        }
        usleep(500000);  /* 500ms for flowmond to start */
        printf("[dashboard] flowmond started (pid=%d) → http://localhost:8800\n",
               (int)flowmond_pid);
    }

    int ret = 0;
    if (strcmp(cmd, "--record") == 0) ret = do_record(bagfile);
    else if (strcmp(cmd, "--play") == 0) ret = do_play(bagfile);
    else if (strcmp(cmd, "--sil") == 0) {
        if (!sil_cfg) sil_cfg = "config/pipeline.json";
        ret = do_sil(bagfile, sil_cfg);
    } else if (strcmp(cmd, "--replay") == 0) {
        printf("Replay+Dashboard mode: %s\n", bagfile);
        MessageBus* bus = message_bus_create("replay_bus");
        BagReader*  r   = bag_reader_open(bagfile);
        if (!bus || !r) { fprintf(stderr, "init failed\n"); ret = 1; goto cleanup; }
        message_bus_subscribe(bus, "vehicle/state", on_replay_state, NULL);
        clock_set_sim_mode(true);
        int count = bag_reader_play(r, bus, 1.0f);
        printf("Replay done: %d messages\n", count);
        clock_set_sim_mode(false);
        bag_reader_close(r);
        message_bus_destroy(bus);
    } else { fprintf(stderr, "未知选项: %s\n", cmd); ret = 1; }

cleanup:
    if (flowmond_pid > 0) {
        kill(flowmond_pid, SIGTERM);
        waitpid(flowmond_pid, NULL, 0);
    }
    return ret;
}
