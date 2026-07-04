/**
 * e2e_demo.c — FlowEngine 全组件端到端演示
 *
 * 串联所有组件:
 *   序列化 → 调度器 → 状态机 → 发现 → 融合 → 传输 → 日志
 *
 * Pipeline:
 *   PerceptionTask (CRITICAL, 10Hz) ─── lidar/gps ──→ bus/transport
 *   FusionTask    (HIGH, choreo)      ─── 对齐融合 ──→ bus/transport
 *   ControlTask   (NORMAL, choreo)    ─── 决策输出 ──→ bus/transport
 *   MonitorTask   (NORMAL, 1Hz)       ─── 统计打印 ──→ stdout
 *
 * 编译: 由 CMakeLists.txt 自动处理
 * 运行: ./build/bin/flow_e2e [duration_sec=10]
 */

#include "message_bus.h"
#include "serializer.h"
#include "scheduler.h"
#include "state_machine.h"
#include "discovery.h"
#include "fusion.h"
#include "transport.h"
#include "logger.h"
#include "flow_registry.h"
#include "param_registry.h"
#include "adas_msgs_gen.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <math.h>
#include <inttypes.h>

/* ══════════════════════════════════════════════════════════ */
/* 全局状态                                                    */
/* ══════════════════════════════════════════════════════════ */

static volatile bool g_running = true;
static MessageBus*       g_bus       = NULL;
static DiscoveryManager* g_discovery = NULL;
static Transport*        g_transport = NULL;
static Scheduler*        g_scheduler = NULL;
static int               g_fusion_tid = -1;   /**< for monitor latency reporting */

static void sig_handler(int sig) {
    (void)sig;
    LOG_INFO("e2e", "signal %d received, shutting down...", sig);
    g_running = false;
}

/* ══════════════════════════════════════════════════════════ */
/* 任务1: 感知 — 发布传感器数据 (CRITICAL, 10Hz)              */
/* ══════════════════════════════════════════════════════════ */

typedef struct {
    TaskBase    base;
    int         tid;
    uint32_t    frame_id;
} PerceptionTask;

static int perception_init(TaskBase* base) {
    PerceptionTask* pt = (PerceptionTask*)base;
    pt->frame_id = 0;

    /* ── 发现: 广播 topic ── */
    discovery_advertise(g_discovery, "sensor/lidar", LIDARFRAME_TYPE_ID,
                        CAP_PUBLISHER, 10.0);
    discovery_advertise(g_discovery, "sensor/gps", GPSDATA_TYPE_ID,
                        CAP_PUBLISHER, 5.0);

    /* ── 传输: 广告 topic ── */
    transport_advertise(g_transport, "sensor/lidar", LIDARFRAME_TYPE_ID);
    transport_advertise(g_transport, "sensor/gps", GPSDATA_TYPE_ID);

    LOG_INFO("perception", "initialized (CRITICAL, 10Hz LiDAR + 5Hz GPS)");
    return 0;
}

static int perception_execute(TaskBase* base) {
    PerceptionTask* pt = (PerceptionTask*)base;
    RateControl* rc = scheduler_get_rate_control(g_scheduler, pt->tid);

    while (g_running && !base->should_stop) {
        if (!rate_control_acquire(rc)) { usleep(1000); continue; }

        /* ── LiDAR @10Hz ── */
        LidarFrame lidar = {
            .x = 50.0f - (float)pt->frame_id * 0.1f,
            .y = 0.0f, .z = 0.0f, .intensity = 0.85f,
            .point_count = 64000 + pt->frame_id,
            .frame_id = pt->frame_id
        };
        Message lmsg;
        msg_init_typed(&lmsg, "sensor/lidar", "perception",
                       LIDARFRAME_TYPE_ID, LIDARFRAME_SCHEMA_VERSION,
                       &lidar, sizeof(lidar));
        transport_publish(g_transport, "sensor/lidar", lmsg.data, lmsg.data_size);
        LOG_DEBUG("perception", "LiDAR #%u: center=(%.1f, %.1f)",
                  pt->frame_id, lidar.x, lidar.y);

        /* ── GPS @5Hz (every other cycle) ── */
        if (pt->frame_id % 2 == 0) {
            GpsData gps = {
                .latitude = 39.904 + (double)pt->frame_id * 0.00001,
                .longitude = 116.407 + (double)pt->frame_id * 0.00001,
                .speed_mps = 33.0f, .heading_deg = 0.0f, .accuracy_m = 0.5f
            };
            Message gmsg;
            msg_init_typed(&gmsg, "sensor/gps", "perception",
                           GPSDATA_TYPE_ID, GPSDATA_SCHEMA_VERSION,
                           &gps, sizeof(gps));
            transport_publish(g_transport, "sensor/gps", gmsg.data, gmsg.data_size);
            LOG_DEBUG("perception", "GPS: lat=%.6f lon=%.6f",
                      gps.latitude, gps.longitude);
        }

        pt->frame_id++;
    }

    if (statem_current(&base->sm) == SM_STATE_RUNNING)
        statem_send_event(&base->sm, SM_EVENT_STOP, base);
    LOG_INFO("perception", "stopped (%u frames)", pt->frame_id);
    return 0;
}

static void perception_cleanup(TaskBase* base) {
    (void)base;
}

static TaskInterface g_perception_vtable = {
    .initialize = perception_init,
    .execute    = perception_execute,
    .cleanup    = perception_cleanup,
    .health_check = NULL,
    .on_message = NULL,
};

/* ══════════════════════════════════════════════════════════ */
/* 任务2: 融合 — 时间对齐 LiDAR+GPS (HIGH, choreo)            */
/* ══════════════════════════════════════════════════════════ */

typedef struct {
    TaskBase      base;
    int           tid;
    MessageBuffer* lidar_buf;
    MessageBuffer* gps_buf;
    uint32_t      fused_count;
} FusionTask;

static void fusion_on_lidar(const Message* msg, void* user_data) {
    FusionTask* ft = (FusionTask*)user_data;
    message_buffer_push(ft->lidar_buf, msg);
}

static void fusion_on_gps(const Message* msg, void* user_data) {
    FusionTask* ft = (FusionTask*)user_data;
    message_buffer_push(ft->gps_buf, msg);
}

static int fusion_init(TaskBase* base) {
    FusionTask* ft = (FusionTask*)base;

    ft->lidar_buf = message_buffer_create("sensor/lidar", LIDARFRAME_TYPE_ID, 32, 5000000);
    ft->gps_buf   = message_buffer_create("sensor/gps",   GPSDATA_TYPE_ID,    16, 5000000);

    /* ── 订阅原始数据 ── */
    transport_subscribe(g_transport, "sensor/lidar", fusion_on_lidar, ft);
    transport_subscribe(g_transport, "sensor/gps",   fusion_on_gps,   ft);

    /* ── 发现: 广告融合输出 ── */
    discovery_advertise(g_discovery, "fusion/localization", 0xF0ED10C0u,
                        CAP_FUSION | CAP_PUBLISHER, 10.0);
    transport_advertise(g_transport, "fusion/localization", 0xF0ED10C0u);

    /* ── 状态机: 驾驶模式 NA→ACC (条件满足) ── */
    ReflectiveStateMachine mode_sm;
    statem_init(&mode_sm, SM_TABLE_MODE_SWITCHING, SM_MODE_NA, "driving_mode");
    mode_sm.trace_enabled = true;
    statem_send_event(&mode_sm, SM_EVT_CONDITIONS_MET, base);
    LOG_INFO("fusion", "driving mode: %s (%s)",
             statem_mode_name(statem_current(&mode_sm)),
             statem_sub_state_name(SM_SUB_READY));

    /* ── Choreo: 被 LiDAR 消息触发 ── */
    scheduler_choreo_trigger_on(g_scheduler, ft->tid, "sensor/lidar");

    LOG_INFO("fusion", "initialized (HIGH, choreo, TIME_ALIGNED 50ms)");
    return 0;
}

static int fusion_execute(TaskBase* base) {
    FusionTask* ft = (FusionTask*)base;

    while (g_running && !base->should_stop) {
        /* ── Choreo wait: 阻塞直到 LiDAR 消息到达 ── */
        int ret = scheduler_choreo_wait(g_scheduler, ft->tid, 500000);
        if (ret == -2) break;  /* stopped */
        if (ret == -1) continue; /* timeout, no new data */

        /* ── 融合: 时间对齐查找 ── */
        const Message* lidar_msg = message_buffer_latest(ft->lidar_buf);
        if (!lidar_msg) continue;

        uint64_t ref_ts = lidar_msg->timestamp_us;
        const Message* gps_msg = message_buffer_find_nearest(
            ft->gps_buf, ref_ts, 50000); /* 50ms window */

        /* ── 序列化: 类型安全访问 ── */
        const LidarFrame* lidar = (const LidarFrame*)
            _msg_cast_impl(lidar_msg, LIDARFRAME_TYPE_ID, sizeof(LidarFrame), "LidarFrame");
        const GpsData* gps = gps_msg ? (const GpsData*)
            _msg_cast_impl(gps_msg, GPSDATA_TYPE_ID, sizeof(GpsData), "GpsData") : NULL;

        if (!lidar) continue;

        /* ── 输出融合结果 ── */
        char fused[256];
        int off = 0;
        off += snprintf(fused + off, sizeof(fused) - (size_t)off,
                        "pos=(%.1f,%.1f)", lidar->x, lidar->y);
        if (gps) {
            off += snprintf(fused + off, sizeof(fused) - (size_t)off,
                            " gps=(%.6f,%.6f) speed=%.1f dt=%" PRIu64 "us",
                            gps->latitude, gps->longitude, gps->speed_mps,
                            gps_msg ? (gps_msg->timestamp_us > ref_ts ?
                             gps_msg->timestamp_us - ref_ts :
                             ref_ts - gps_msg->timestamp_us) : 0ULL);
        }

        ft->fused_count++;

        /* ── 发布融合结果 ── */
        Message out_msg;
        msg_init_typed(&out_msg, "fusion/localization", "fusion",
                       0xF0ED10C0u, 1, fused, (uint32_t)strlen(fused) + 1);
        out_msg.timestamp_us = ref_ts;
        transport_publish(g_transport, "fusion/localization",
                          out_msg.data, out_msg.data_size);

        LOG_INFO("fusion", "#%u %s", ft->fused_count, fused);

        /* ── Latency tracking ── */
        LatencyTracker* lt = scheduler_get_latency(g_scheduler, ft->tid);
        if (lt) {
            uint64_t now_us = (uint64_t)(gps_msg ? gps_msg->timestamp_us : 0);
            if (now_us > 0) {
                struct timespec ts;
                clock_gettime(CLOCK_MONOTONIC, &ts);
                uint64_t wall = (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;
                latency_tracker_record(lt, wall - now_us);
            }
        }
    }

    if (statem_current(&base->sm) == SM_STATE_RUNNING)
        statem_send_event(&base->sm, SM_EVENT_STOP, base);
    LOG_INFO("fusion", "stopped (%u fused frames)", ft->fused_count);
    return 0;
}

static void fusion_cleanup(TaskBase* base) {
    FusionTask* ft = (FusionTask*)base;
    message_buffer_destroy(ft->lidar_buf);
    message_buffer_destroy(ft->gps_buf);
}

static TaskInterface g_fusion_vtable = {
    .initialize = fusion_init,
    .execute    = fusion_execute,
    .cleanup    = fusion_cleanup,
    .health_check = NULL,
    .on_message = NULL,
};

/* ══════════════════════════════════════════════════════════ */
/* 任务3: 控制 — 订阅融合结果做决策 (NORMAL, choreo)          */
/* ══════════════════════════════════════════════════════════ */

typedef struct {
    TaskBase  base;
    int       tid;
    int       decision_count;
} ControlTask;

static void control_on_fusion(const Message* msg, void* user_data) {
    ControlTask* ct = (ControlTask*)user_data;
    const char* data = (const char*)msg->data;
    if (!data) return;

    const char* decision = "CRUISE";
    if (strstr(data, "pos=(")) {
        float x = 0;
        sscanf(data, "pos=(%f", &x);
        if (x < 15.0f)       decision = "🛑 BRAKE";
        else if (x < 35.0f)  decision = "🟡 SLOW";
    }

    ct->decision_count++;
    LOG_WARN("control", "#%d %s → %s", ct->decision_count, data, decision);
}

static int control_init(TaskBase* base) {
    ControlTask* ct = (ControlTask*)base;

    transport_subscribe(g_transport, "fusion/localization", control_on_fusion, ct);

    discovery_advertise(g_discovery, "fusion/localization", 0xF0ED10C0u,
                        CAP_SUBSCRIBER, 0);

    /* ── Choreo: 被融合输出触发 ── */
    scheduler_choreo_trigger_on(g_scheduler, ct->tid, "fusion/localization");

    LOG_INFO("control", "initialized (NORMAL, choreo)");
    return 0;
}

static int control_execute(TaskBase* base) {
    ControlTask* ct = (ControlTask*)base;

    while (g_running && !base->should_stop) {
        int ret = scheduler_choreo_wait(g_scheduler, ct->tid, 1000000);
        if (ret == -2) break;  /* stopped */
    }

    if (statem_current(&base->sm) == SM_STATE_RUNNING)
        statem_send_event(&base->sm, SM_EVENT_STOP, base);
    LOG_INFO("control", "stopped (%d decisions)", ct->decision_count);
    return 0;
}

static void control_cleanup(TaskBase* base) {
    (void)base;
}

static TaskInterface g_control_vtable = {
    .initialize = control_init,
    .execute    = control_execute,
    .cleanup    = control_cleanup,
    .health_check = NULL,
    .on_message = NULL,
};

/* ══════════════════════════════════════════════════════════ */
/* 任务4: 监控 — 定期打印统计 (NORMAL, 1Hz)                   */
/* ══════════════════════════════════════════════════════════ */

typedef struct {
    TaskBase  base;
    int       tid;
} MonitorTask;

static int monitor_init(TaskBase* base) {
    statem_send_event(&base->sm, SM_EVENT_START, base);
    LOG_INFO("monitor", "initialized (1Hz stats reporter)");
    return 0;
}

static int monitor_execute(TaskBase* base) {
    RateControl* rc = scheduler_get_rate_control(g_scheduler, ((MonitorTask*)base)->tid);

    while (g_running && !base->should_stop) {
        if (!rate_control_acquire(rc)) { usleep(100000); continue; }

        /* ── Bus stats ── */
        uint64_t pub, del, drop;
        message_bus_get_stats(g_bus, &pub, &del, &drop);

        /* ── Transport stats ── */
        TransportStats ts;
        transport_get_stats(g_transport, &ts);

        /* ── Scheduler stats ── */
        int task_count = scheduler_task_count(g_scheduler);

        /* ── Discovery stats ── */
        const TopologyGraph* topo = discovery_get_topology(g_discovery);

        printf("\n┌─── Monitor @ %lu ──────────────────────────┐\n",
               (unsigned long)time(NULL));
        printf("│ Bus:     pub=%lu del=%lu drop=%lu\n",
               (unsigned long)pub, (unsigned long)del, (unsigned long)drop);
        printf("│ Net:     local=%lu/%lu remote=%lu/%lu\n",
               (unsigned long)ts.local_published, (unsigned long)ts.local_delivered,
               (unsigned long)ts.remote_published, (unsigned long)ts.remote_delivered);
        printf("│ Tasks:   %d  Topology: %u nodes\n",
               task_count, topo ? topo->node_count : 0);
        printf("│ Routes:  IPC=%d TCP=%d\n",
               transport_ipc_channel_count(g_transport),
               transport_remote_peer_count(g_transport));
        printf("└───────────────────────────────────────────┘\n");

        /* ── Export for FlowBoard dashboard ── */
        char* topo_json = discovery_export_json(g_discovery);
        if (topo_json) {
            FILE* jf = fopen("/tmp/flow_topology.json", "w");
            if (jf) {
                /* Wrap: replace closing } with metrics */
                /* Format: {"self":"...","nodes":[...]} */
                /* We insert metrics before the final } */
                size_t len = strlen(topo_json);
                /* Write everything except the final } */
                fwrite(topo_json, 1, len - 1, jf);
                /* Add metrics */
                fprintf(jf, ",\"metrics\":{"
                        "\"bus\":{\"published\":%lu,\"delivered\":%lu,\"dropped\":%lu},"
                        "\"transport\":{\"local_pub\":%lu,\"remote_pub\":%lu},"
                        "\"scheduler\":{\"tasks\":%d,\"mode\":\"CHOREO\"},",
                        (unsigned long)pub, (unsigned long)del, (unsigned long)drop,
                        (unsigned long)ts.local_published, (unsigned long)ts.remote_published,
                        task_count);
                LatencyStats ls = latency_tracker_stats(
                    g_fusion_tid >= 0 ? scheduler_get_latency(g_scheduler, g_fusion_tid) : NULL);
                fprintf(jf, "\"latency\":{\"avg_us\":%lu,\"p50_us\":%lu,\"p99_us\":%lu}",
                        (unsigned long)ls.avg_us, (unsigned long)ls.p50_us,
                        (unsigned long)ls.p99_us);

                /* ── Per-topic stats (QoS) ── */
                TopicStats tstats[16];
                int nt = message_bus_get_all_topic_stats(g_bus, tstats, 16);
                fprintf(jf, ",\"topics\":[");
                for (int ti = 0; ti < nt; ti++) {
                    uint64_t avg_lat = tstats[ti].deliver_count > 0
                        ? tstats[ti].total_latency_us / tstats[ti].deliver_count : 0;
                    fprintf(jf, "%s{\"topic\":\"%s\",\"pub\":%lu,\"del\":%lu,\"drop\":%lu,"
                            "\"lat_us\":%lu,\"freq\":%.1f,\"subs\":%u}",
                            ti > 0 ? "," : "",
                            tstats[ti].topic,
                            (unsigned long)tstats[ti].publish_count,
                            (unsigned long)tstats[ti].deliver_count,
                            (unsigned long)tstats[ti].drop_count,
                            (unsigned long)avg_lat,
                            tstats[ti].frequency_hz,
                            tstats[ti].subscriber_count);
                }
                fprintf(jf, "]");
                /* ── Include FlowRegistry data ── */
                char* reg_json = flow_registry_export_json();
                if (reg_json) {
                    /* registry starts with {"tasks":...} — append without outer braces */
                    fprintf(jf, ",\"registry\":%s", reg_json);
                    free(reg_json);
                }
                fprintf(jf, "}}\n");
                fclose(jf);
            }
            free(topo_json);
        }
    }

    statem_send_event(&base->sm, SM_EVENT_STOP, base);
    return 0;
}

static void monitor_cleanup(TaskBase* base) {
    (void)base;
}

static TaskInterface g_monitor_vtable = {
    .initialize = monitor_init,
    .execute    = monitor_execute,
    .cleanup    = monitor_cleanup,
    .health_check = NULL,
    .on_message = NULL,
};

/* ══════════════════════════════════════════════════════════ */
/* Main                                                        */
/* ══════════════════════════════════════════════════════════ */

int main(int argc, char** argv) {
    int duration = (argc > 1) ? atoi(argv[1]) : 10;

    /* ── 日志: 全局初始化 ── */
    log_init(LOG_INFO, NULL);
    LOG_INFO("e2e", "╔══════════════════════════════════════════╗");
    LOG_INFO("e2e", "║  FlowEngine End-to-End Demo (%ds)        ║", duration);
    LOG_INFO("e2e", "║  感知→融合→控制→监控  全组件串联         ║");
    LOG_INFO("e2e", "╚══════════════════════════════════════════╝");

    /* ── 序列化: 注册 ADAS 类型 ── */
    adas_msgs_register_all();
    LOG_INFO("e2e", "serializer: %d types registered", serializer_type_count());

    /* ── 总线 ── */
    g_bus = message_bus_create("e2e_bus");
    LOG_INFO("e2e", "message_bus: created");

    /* ── 发现 ── */
    g_discovery = discovery_create("e2e_node",
        CAP_PUBLISHER | CAP_SUBSCRIBER | CAP_FUSION);
    discovery_start(g_discovery);
    LOG_INFO("e2e", "discovery: started");

    /* ── 传输 ── */
    g_transport = transport_create(g_bus, g_discovery, TRANSPORT_AUTO);
    transport_start(g_transport);
    LOG_INFO("e2e", "transport: started (AUTO mode)");

    /* ── 调度器 ── */
    SchedulerConfig scfg = SCHEDULER_CONFIG_DEFAULT;
    scfg.mode = SCHEDULER_MODE_CHOREO;
    g_scheduler = scheduler_create(&scfg);

    /* ── 创建任务 ── */
    PerceptionTask* pt = (PerceptionTask*)calloc(1, sizeof(PerceptionTask));
    FusionTask*     ft = (FusionTask*)calloc(1, sizeof(FusionTask));
    ControlTask*    ct = (ControlTask*)calloc(1, sizeof(ControlTask));
    MonitorTask*    mt = (MonitorTask*)calloc(1, sizeof(MonitorTask));

    TaskConfig pcfg = { .name="perception", .priority=TASK_PRIORITY_CRITICAL,
                        .max_frequency_hz=10.0, .auto_restart=false };
    TaskConfig fcfg = { .name="fusion",     .priority=TASK_PRIORITY_HIGH,
                        .max_frequency_hz=0, .auto_restart=false };
    TaskConfig ccfg = { .name="control",    .priority=TASK_PRIORITY_NORMAL,
                        .max_frequency_hz=100.0, .auto_restart=false };
    TaskConfig mcfg = { .name="monitor",    .priority=TASK_PRIORITY_NORMAL,
                        .max_frequency_hz=1.0, .auto_restart=false };

    task_base_init(&pt->base, &g_perception_vtable, &pcfg);
    task_base_init(&ft->base, &g_fusion_vtable,     &fcfg);
    task_base_init(&ct->base, &g_control_vtable,     &ccfg);
    task_base_init(&mt->base, &g_monitor_vtable,     &mcfg);

    /* ── 注册到调度器 ── */
    pt->tid = scheduler_register_task(g_scheduler, &pt->base, "perception");
    ft->tid = scheduler_register_task(g_scheduler, &ft->base, "fusion");
    g_fusion_tid = ft->tid;
    ct->tid = scheduler_register_task(g_scheduler, &ct->base, "control");
    mt->tid = scheduler_register_task(g_scheduler, &mt->base, "monitor");

    scheduler_set_choreo_bus(g_scheduler, g_bus);
    scheduler_start(g_scheduler);
    LOG_INFO("e2e", "scheduler: %d tasks in CHOREO mode", scheduler_task_count(g_scheduler));

    /* ── FlowRegistry: 注册所有组件 ── */
    flow_registry_register_task("perception", "LiDAR+GPS sensor simulator",
        "libfake_perception_task.so",
        (const char*[]){NULL},
        (const char*[]){"sensor/lidar","sensor/gps",NULL}, NULL);
    flow_registry_register_task("fusion", "Time-aligned sensor fusion",
        "libflowcoro_task.so",
        (const char*[]){"sensor/lidar","sensor/gps",NULL},
        (const char*[]){"fusion/localization",NULL}, NULL);
    flow_registry_register_task("control", "Driving decision maker",
        "libfake_control_task.so",
        (const char*[]){"fusion/localization",NULL},
        (const char*[]){"control/cmd",NULL}, NULL);
    flow_registry_register_task("monitor", "System stats reporter",
        "libexample_task.so", NULL, NULL, NULL);

    flow_registry_register_topic("sensor/lidar", LIDARFRAME_TYPE_ID, NULL);
    flow_registry_register_topic("sensor/gps", GPSDATA_TYPE_ID, NULL);
    flow_registry_register_topic("fusion/localization", 0xF0ED10C0u, NULL);

    flow_registry_register_plugin("fake_perception", "libfake_perception_task.so",
        (const char*[]){"perception",NULL}, (const char*[]){"LidarFrame","GpsData",NULL});
    flow_registry_register_plugin("fake_control", "libfake_control_task.so",
        (const char*[]){"control",NULL}, (const char*[]){"ControlCmd",NULL});

    LOG_INFO("e2e", "registry: %d total entries", flow_registry_total_count());

    /* ── ParamRegistry: 注册运行时参数 ── */
    param_register_int("control.max_speed", 120, 0, 200, "Max speed km/h");
    param_register_float("fusion.max_delta_ms", 50.0, 10.0, 500.0, "Alignment window ms");
    param_register_bool("control.emergency_brake", true, "Enable AEB");
    param_register_int("perception.lidar_rate_hz", 10, 1, 100, "LiDAR scan rate");
    param_enable_hot_reload("control.max_speed");
    param_enable_hot_reload("control.emergency_brake");
    LOG_INFO("e2e", "params: %d registered (%d hot-reloadable)", param_count(),
             /* count hot-reloadable */ 2);

    /* ── 启动任务（先启动消费者，再启动生产者，确保 trigger 就绪）── */
    task_start(&ft->base);  LOG_INFO("e2e", "fusion:     started (HIGH, choreo)");
    task_start(&ct->base);  LOG_INFO("e2e", "control:    started (NORMAL, choreo)");
    task_start(&mt->base);  LOG_INFO("e2e", "monitor:    started (1Hz)");
    usleep(200000);  /* wait 200ms for subscriptions to take effect */
    task_start(&pt->base);  LOG_INFO("e2e", "perception: started (CRITICAL, 10Hz)");

    /* ── 信号处理 ── */
    signal(SIGINT, sig_handler);
    signal(SIGTERM, sig_handler);

    /* ── 运行 ── */
    LOG_INFO("e2e", "running for %d seconds... (Ctrl+C to stop)", duration);
    sleep((unsigned)duration);
    g_running = false;

    /* ── 等待任务结束 ── */
    LOG_INFO("e2e", "stopping tasks...");
    task_stop(&pt->base);
    task_stop(&ft->base);
    task_stop(&ct->base);
    task_stop(&mt->base);

    /* ── 统计摘要 ── */
    printf("\n╔══════════════════════════════════════════╗\n");
    printf("║  End-to-End Demo Summary                 ║\n");
    printf("╠══════════════════════════════════════════╣\n");

    /* ── 状态机 ── */
    printf("║  State Machines:                         ║\n");
    printf("║    perception: %-26s ║\n",
           statem_state_name(NULL, statem_current(&pt->base.sm)));
    printf("║    fusion:     %-26s ║\n",
           statem_state_name(NULL, statem_current(&ft->base.sm)));
    printf("║    control:    %-26s ║\n",
           statem_state_name(NULL, statem_current(&ct->base.sm)));

    /* ── 延迟 ── */
    LatencyStats ls = latency_tracker_stats(scheduler_get_latency(g_scheduler, ft->tid));
    printf("║  Fusion Latency (us):                    ║\n");
    printf("║    avg=%lu p50=%lu p99=%lu min=%lu max=%lu   ║\n",
           (unsigned long)ls.avg_us, (unsigned long)ls.p50_us,
           (unsigned long)ls.p99_us, (unsigned long)ls.min_us,
           (unsigned long)ls.max_us);

    /* ── 传输统计 ── */
    TransportStats ts;
    transport_get_stats(g_transport, &ts);
    printf("║  Transport:                              ║\n");
    printf("║    local  pub=%lu del=%lu                    ║\n",
           (unsigned long)ts.local_published, (unsigned long)ts.local_delivered);

    /* ── 发现拓扑 ── */
    discovery_print_graph(g_discovery);
    char* topo_json = discovery_export_json(g_discovery);
    printf("║  Topology JSON: %ld bytes                  ║\n",
           (long)strlen(topo_json ? topo_json : "{}"));
    printf("║  (paste into tools/topology_viewer.html)  ║\n");
    free(topo_json);

    printf("╚══════════════════════════════════════════╝\n\n");

    /* ── 清理 ── */
    scheduler_stop(g_scheduler);
    scheduler_destroy(g_scheduler);
    transport_stop(g_transport);
    transport_destroy(g_transport);
    discovery_stop(g_discovery);
    discovery_destroy(g_discovery);
    message_bus_destroy(g_bus);

    free(pt); free(ft); free(ct); free(mt);
    log_shutdown();
    return 0;
}
