/**
 * fullstack_demo.c — FlowEngine 全栈多进程集成演示
 *
 * 架构：
 *   perception（发布sensor数据）
 *       ↓ UDP discovery + IPC
 *   fusion（FusionNodeCpp 时间对齐 + 融合）
 *       ↓ bus publish
 *   control（订阅融合结果，决策）
 *
 * 每个进程都有：状态机追踪 + 服务发现 + 拓扑感知
 *
 * 用法：
 *   ./flow_fullstack --role perception [--duration 10]
 *   ./flow_fullstack --role fusion      [--duration 10]
 *   ./flow_fullstack --role control     [--duration 10]
 *
 * 一键启动：
 *   bash scripts/fullstack_demo.sh
 */

#include "message_bus.h"
#include "discovery.h"
#include "fusion.h"
#include "state_machine.h"
#include "serializer.h"
#include "adas_msgs_gen.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <time.h>
#include <math.h>

static bool g_running = true;

static void sig_handler(int sig) {
    (void)sig;
    g_running = false;
}

/* ══════════════════════════════════════════════════════════ */
/* 感知节点 — 发布 sensor/lidar + sensor/gps                 */
/* ══════════════════════════════════════════════════════════ */

static void run_perception(int duration_sec) {
    printf("\n╔══════════════════════════════════════════╗\n");
    printf("║  感知节点 (Perception Node)              ║\n");
    printf("╚══════════════════════════════════════════╝\n\n");

    /* Init state machine */
    ReflectiveStateMachine sm;
    statem_init(&sm, SM_TABLE_STANDARD, SM_STATE_INITIALIZED, "perception");
    sm.trace_enabled = true;

    /* Create message bus */
    MessageBus* bus = message_bus_create("perception_bus");

    /* Start discovery */
    DiscoveryManager* dm = discovery_create("perception_node", CAP_PUBLISHER);
    discovery_advertise(dm, "sensor/lidar", LIDARFRAME_TYPE_ID, CAP_PUBLISHER, 10.0);
    discovery_advertise(dm, "sensor/gps",   GPSDATA_TYPE_ID,    CAP_PUBLISHER, 5.0);
    discovery_start(dm);

    /* Enter RUNNING state */
    statem_send_event(&sm, SM_EVENT_START, NULL);
    statem_print_status(&sm);

    printf("[perception] Publishing sensor data...\n");

    uint32_t frame_id = 0;
    time_t start = time(NULL);

    while (g_running && (time(NULL) - start) < duration_sec) {
        /* LiDAR frame @10Hz */
        LidarFrame lidar = {
            .x = 50.0f - (float)frame_id * 0.1f,  /* approaching vehicle */
            .y = 0.0f,
            .z = 0.0f,
            .intensity = 0.85f,
            .point_count = 64000 + frame_id,
            .frame_id = frame_id
        };
        message_bus_publish(bus, "sensor/lidar", "perception",
                            &lidar, sizeof(lidar));
        printf("[perception] LiDAR #%u: center=(%.1f, %.1f)\n",
               frame_id, lidar.x, lidar.y);

        /* GPS frame @5Hz (every other cycle) */
        if (frame_id % 2 == 0) {
            GpsData gps = {
                .latitude = 39.904 + (double)frame_id * 0.00001,
                .longitude = 116.407 + (double)frame_id * 0.00001,
                .speed_mps = 33.0f,
                .heading_deg = 0.0f,
                .accuracy_m = 0.5f
            };
            message_bus_publish(bus, "sensor/gps", "perception",
                                &gps, sizeof(gps));
            printf("[perception] GPS: lat=%.6f lon=%.6f speed=%.1f m/s\n",
                   gps.latitude, gps.longitude, gps.speed_mps);
        }

        frame_id++;
        usleep(100000); /* 10 Hz */
    }

    /* Cleanup */
    statem_send_event(&sm, SM_EVENT_STOP, NULL);
    statem_print_status(&sm);

    uint64_t pub, del, drop;
    message_bus_get_stats(bus, &pub, &del, &drop);
    printf("[perception] Stats: published=%lu delivered=%lu dropped=%lu\n",
           (unsigned long)pub, (unsigned long)del, (unsigned long)drop);

    discovery_print_graph(dm);
    discovery_stop(dm);
    discovery_destroy(dm);
    message_bus_destroy(bus);
    printf("[perception] Done.\n");
}

/* ══════════════════════════════════════════════════════════ */
/* 融合节点 — 时间对齐 LiDAR + GPS → 融合定位                */
/* ══════════════════════════════════════════════════════════ */

/* Simple fused localization output */
typedef struct {
    uint64_t timestamp_us;
    float    position_x, position_y;
    double   latitude, longitude;
    float    speed_mps;
} FusedLocalization;

#define FUSEDLOC_TYPE_ID  0xF0ED10C0u  /* hand-picked */

static void fusion_output_callback(const SyncedFrame* frame, MessageBus* bus,
                                   const char* topic, uint32_t type_id,
                                   void* user_data) {
    (void)type_id;
    (void)user_data;

    FusedLocalization out;
    memset(&out, 0, sizeof(out));
    out.timestamp_us = frame->reference_ts;

    /* Extract LiDAR */
    if (frame->input_valid[0]) {
        const LidarFrame* lidar = (const LidarFrame*)frame->inputs[0].data;
        out.position_x = lidar->x;
        out.position_y = lidar->y;
    }

    /* Extract GPS */
    if (frame->input_valid[1]) {
        const GpsData* gps = (const GpsData*)frame->inputs[1].data;
        out.latitude  = gps->latitude;
        out.longitude = gps->longitude;
        out.speed_mps = gps->speed_mps;
    }

    message_bus_publish(bus, topic, "fusion", &out, sizeof(out));

    printf("[fusion] frame: pos=(%.1f,%.1f) gps=(%.6f,%.6f) speed=%.1f dt=[%.0f, %.0f]us\n",
           out.position_x, out.position_y,
           out.latitude, out.longitude, out.speed_mps,
           frame->input_deltas_us[0], frame->input_deltas_us[1]);
}

static void run_fusion(int duration_sec) {
    printf("\n╔══════════════════════════════════════════╗\n");
    printf("║  融合节点 (Fusion Node)                  ║\n");
    printf("╚══════════════════════════════════════════╝\n\n");

    ReflectiveStateMachine sm;
    statem_init(&sm, SM_TABLE_STANDARD, SM_STATE_INITIALIZED, "fusion");
    sm.trace_enabled = true;

    MessageBus* bus = message_bus_create("fusion_bus");

    DiscoveryManager* dm = discovery_create("fusion_node",
                                            CAP_SUBSCRIBER | CAP_PUBLISHER | CAP_FUSION);
    discovery_advertise(dm, "sensor/lidar",      LIDARFRAME_TYPE_ID, CAP_SUBSCRIBER, 0);
    discovery_advertise(dm, "sensor/gps",        GPSDATA_TYPE_ID,    CAP_SUBSCRIBER, 0);
    discovery_advertise(dm, "fusion/localization",FUSEDLOC_TYPE_ID,  CAP_PUBLISHER, 10.0);
    discovery_start(dm);

    /* Create fusion node */
    FusionPolicy policy = FUSION_POLICY_TIME_ALIGNED;
    FusionNode* fn = fusion_node_create("adas_fusion", bus, &policy);
    fusion_node_add_input(fn, "sensor/lidar", LIDARFRAME_TYPE_ID, 32);
    fusion_node_add_input(fn, "sensor/gps",   GPSDATA_TYPE_ID,    16);
    fusion_node_set_output(fn, "fusion/localization", FUSEDLOC_TYPE_ID);
    fusion_node_set_callback(fn, fusion_output_callback, NULL);

    /* Wait for perception node to be online */
    const char* deps[] = {"perception_node"};
    printf("[fusion] Waiting for dependencies...\n");
    discovery_wait_for_deps(dm, deps, 1, 10000);

    statem_send_event(&sm, SM_EVENT_START, NULL);
    statem_print_status(&sm);

    fusion_node_start(fn);

    printf("[fusion] Running...\n");
    time_t start = time(NULL);
    while (g_running && (time(NULL) - start) < duration_sec) {
        usleep(500000);
    }

    fusion_node_stop(fn);
    statem_send_event(&sm, SM_EVENT_STOP, NULL);

    discovery_print_graph(dm);
    discovery_stop(dm);
    fusion_node_destroy(fn);
    discovery_destroy(dm);
    message_bus_destroy(bus);
    printf("[fusion] Done.\n");
}

/* ══════════════════════════════════════════════════════════ */
/* 控制节点 — 订阅融合结果，做驾驶决策                       */
/* ══════════════════════════════════════════════════════════ */

static void control_on_message(const Message* msg, void* user_data) {
    int* count = (int*)user_data;
    const FusedLocalization* loc = (const FusedLocalization*)msg->data;

    const char* decision = "CRUISE";
    if (loc->position_x < 20.0f)  decision = "BRAKE";
    else if (loc->position_x < 40.0f) decision = "SLOW_DOWN";

    printf("[control] #%d dist=%.1fm speed=%.1fm/s → %s\n",
           ++(*count), loc->position_x, loc->speed_mps, decision);
}

static void run_control(int duration_sec) {
    printf("\n╔══════════════════════════════════════════╗\n");
    printf("║  控制节点 (Control Node)                 ║\n");
    printf("╚══════════════════════════════════════════╝\n\n");

    ReflectiveStateMachine sm;
    statem_init(&sm, SM_TABLE_STANDARD, SM_STATE_INITIALIZED, "control");
    sm.trace_enabled = true;

    MessageBus* bus = message_bus_create("control_bus");

    DiscoveryManager* dm = discovery_create("control_node", CAP_SUBSCRIBER);
    discovery_advertise(dm, "fusion/localization", FUSEDLOC_TYPE_ID, CAP_SUBSCRIBER, 0);
    discovery_start(dm);

    const char* deps[] = {"fusion_node"};
    printf("[control] Waiting for dependencies...\n");
    discovery_wait_for_deps(dm, deps, 1, 10000);

    int msg_count = 0;
    message_bus_subscribe(bus, "fusion/localization", control_on_message, &msg_count);

    statem_send_event(&sm, SM_EVENT_START, NULL);
    statem_print_status(&sm);

    printf("[control] Monitoring...\n");
    time_t start = time(NULL);
    while (g_running && (time(NULL) - start) < duration_sec) {
        usleep(200000);
    }

    statem_send_event(&sm, SM_EVENT_STOP, NULL);

    uint64_t pub, del, drop;
    message_bus_get_stats(bus, &pub, &del, &drop);
    printf("[control] Received %d msgs | stats: delivered=%lu dropped=%lu\n",
           msg_count, (unsigned long)del, (unsigned long)drop);

    discovery_print_graph(dm);
    discovery_stop(dm);
    discovery_destroy(dm);
    message_bus_destroy(bus);
    printf("[control] Done.\n");
}

/* ══════════════════════════════════════════════════════════ */
/* Main                                                        */
/* ══════════════════════════════════════════════════════════ */

static void print_usage(const char* prog) {
    printf("Usage: %s --role <perception|fusion|control> [--duration <sec>]\n", prog);
    printf("\n  Full-stack multi-process demo for FlowEngine.\n");
    printf("  Launch in 3 terminals, or use: bash scripts/fullstack_demo.sh\n");
}

int main(int argc, char** argv) {
    const char* role = NULL;
    int duration = 10;

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--role") == 0 && i + 1 < argc)
            role = argv[++i];
        else if (strcmp(argv[i], "--duration") == 0 && i + 1 < argc)
            duration = atoi(argv[++i]);
        else if (strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-h") == 0) {
            print_usage(argv[0]);
            return 0;
        }
    }

    if (!role) {
        print_usage(argv[0]);
        return 1;
    }

    signal(SIGINT, sig_handler);
    signal(SIGTERM, sig_handler);

    /* Register ADAS types */
    adas_msgs_register_all();

    if (strcmp(role, "perception") == 0)
        run_perception(duration);
    else if (strcmp(role, "fusion") == 0)
        run_fusion(duration);
    else if (strcmp(role, "control") == 0)
        run_control(duration);
    else {
        fprintf(stderr, "Unknown role: %s\n", role);
        print_usage(argv[0]);
        return 1;
    }

    return 0;
}
