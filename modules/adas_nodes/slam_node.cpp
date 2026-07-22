/**
 * slam_node.cpp — SLAM 适配层节点插件 (LiDAR + IMU → 2D 位姿 Pose2D)
 *
 * 为 GPS 丢失场景（室内/隧道/地下停车场）提供定位。订阅 sensor/lidar 与
 * sensor/imu，运行 SLAM 算法发布 2D 位姿到 sensor/pose。这是 FlowEngine 在
 * GPS 不可用环境下的定位入口，与 gps_driver_node 互补：
 *
 *   有 GPS: [gps_driver_node] → sensor/gps → fusion_node (主定位)
 *   无 GPS: [slam_node] → sensor/pose → fusion_node (接管定位)
 *
 * 核心设计——SLAM 算法作为可替换钩子：
 *   本节点是**骨架/适配层**，真实 SLAM 算法（FAST-LIO2 / EKF-SLAM）作为外部库
 *   通过 slam_update() 钩子接入。默认实现一个**简易里程计（dead reckoning）**：
 *     - 用 IMU 陀螺仪 gyro_z 积分 heading
 *     - 假设恒速（或用加速度计 accel 积分速度）推算 x/y
 *   dead reckoning 精度有限，只适合短时间/低速，误差随时间累积发散。
 *
 * ── 集成 EKF-SLAM（轻量内置实现） ──
 *   无需外部依赖，直接启用：algo="ekf_slam"
 *   状态向量: [x, y, heading, v, omega]
 *   IMU 预测: 陀螺仪积分 heading，加速度计积分速度
 *   协方差传播: 完整 5×5 状态转移矩阵 + 过程噪声 Q
 *   量测更新: 支持位置/航向观测校正（LiDAR 特征或仿真真值）
 *
 * ── 集成 FAST-LIO2（LiDAR+IMU 紧耦合）步骤 ──
 *   1. git clone https://github.com/hku-mars/FAST_LIO.git
 *      cd FAST_LIO && mkdir build && cd build
 *      cmake .. && make -j          # 产出 libfast_lio2.a
 *   2. 在本文件顶部 #include "IKFoM_toolkit/esekfom/esekfom.hpp"
 *      在 slam_update() 里替换默认 dead reckoning：
 *        a. esekfom::init()                  初始化误差状态卡尔曼滤波
 *        b. 把 last_lidar 点云喂给 process() 做点云配准
 *        c. 把 last_imu 喂给 predict() 做状态预测
 *        d. 取 esekfom 状态填到 Pose2D (x/y/heading + 协方差)
 *   3. CMakeLists.txt 增加：
 *        find_package(PCL REQUIRED)
 *        find_package(Eigen3 REQUIRED)
 *        find_package(Sophus REQUIRED)
 *        target_link_libraries(slam_node ${NODE_LINK_LIBS}
 *            fast_lio2 PCL::PCL Eigen3::Eigen Sophus::Sophus)
 *
 * ── 也可作为纯里程计节点 ──
 *   source=POSE_SOURCE_ODOMETRY(3) 时，下游 fusion_node 把本节点输出当作
 *   里程计观测与 GPS 融合；source=POSE_SOURCE_SLAM(2) 时视为独立定位源。
 *
 * 话题契约：
 *   输入: sensor/lidar (LidarFrame 二进制, type_id=0xd712aa51)
 *         sensor/imu   (ImuData  二进制, type_id=0x7dc626af)
 *   输出: sensor/pose  (Pose2D   二进制, type_id=0x026c6093)
 */

#include "node_plugin.h"
#include "adas_msgs_gen.h"
#include "transport.h"
#include "discovery.h"
#include "logger.h"
#include "clock_service.h"
#include "ekf_slam.h"
#include <cjson/cJSON.h>

#include <math.h>
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define POSE_SOURCE_SLAM      2u
#define POSE_SOURCE_ODOMETRY  3u

static struct {
    Transport*        transport;
    DiscoveryManager* discovery;
    Scheduler*        scheduler;

    int    enabled;
    int    publish_hz;
    int    dry_run;
    int    use_lidar;
    int    use_imu;
    float  initial_x;
    float  initial_y;
    float  initial_heading;
    char   algo[32];

    float  pose_x;
    float  pose_y;
    float  pose_heading;
    float  pose_cov_xx;
    float  pose_cov_yy;
    float  pose_cov_hh;
    float  speed;

    LidarFrame last_lidar;
    uint64_t   last_lidar_us;
    int        have_lidar;
    ImuData    last_imu;
    uint64_t   last_imu_us;
    int        have_imu;
    uint64_t   last_slam_update_us;

    pthread_mutex_t lock;

    uint64_t poses_published;
    uint64_t lidar_frames_received;
    uint64_t imu_samples_received;
    time_t   last_imu_time;

    EkfSlam ekf_slam_state;

    TaskBase   taskbase;
} g;

static void on_lidar(const Message* msg, void* user_data);
static void on_imu(const Message* msg, void* user_data);
static void slam_update_dead_reckon(Pose2D* pose);
static void slam_update_ekf_slam(Pose2D* pose);
static void slam_update(Pose2D* pose);
static int slam_execute(TaskBase* task);
static int slam_init(MessageBus* bus, Transport* transport,
                     DiscoveryManager* discovery, Scheduler* scheduler,
                     const char* params_json);
static int slam_start(void);
static void slam_stop(void);
static void slam_cleanup(void);
static int slam_health(void);
extern NodePlugin s_plugin;

static void on_lidar(const Message* msg, void* user_data) {
    (void)user_data;
    if (!msg || !g.enabled) return;
    if (msg->data_size == 0) return;

    LidarFrame frame;
    if (LidarFrame_deserialize(&frame, (const uint8_t*)msg->data, msg->data_size) != 0) {
        LOG_WARN("slam", "LidarFrame deserialize failed (size=%u)", msg->data_size);
        return;
    }

    pthread_mutex_lock(&g.lock);
    g.last_lidar    = frame;
    g.last_lidar_us = clock_now_us();
    g.have_lidar    = 1;
    g.lidar_frames_received++;
    pthread_mutex_unlock(&g.lock);
}

static void on_imu(const Message* msg, void* user_data) {
    (void)user_data;
    if (!msg || !g.enabled) return;
    if (msg->data_size == 0) return;

    ImuData imu;
    if (ImuData_deserialize(&imu, (const uint8_t*)msg->data, msg->data_size) != 0) {
        LOG_WARN("slam", "ImuData deserialize failed (size=%u)", msg->data_size);
        return;
    }

    uint64_t now = clock_now_us();

    pthread_mutex_lock(&g.lock);
    if (g.have_imu) {
        float dt = (float)((double)(now - g.last_imu_us) / 1000000.0);
        if (dt > 0.0f && dt < 1.0f) {
            g.pose_heading += imu.gyro_z * dt;
            while (g.pose_heading >  (float)M_PI) g.pose_heading -= 2.0f * (float)M_PI;
            while (g.pose_heading < -(float)M_PI) g.pose_heading += 2.0f * (float)M_PI;
        }
    }
    g.last_imu    = imu;
    g.last_imu_us = now;
    g.have_imu    = 1;
    g.imu_samples_received++;
    g.last_imu_time = time(NULL);
    pthread_mutex_unlock(&g.lock);
}

static void slam_update_dead_reckon(Pose2D* pose);

#ifdef HAVE_FAST_LIO2

typedef struct {
    int placeholder;
} FastLio2State;

static FastLio2State g_fast_lio2;

static int fast_lio2_init(void) {
    memset(&g_fast_lio2, 0, sizeof(g_fast_lio2));
    LOG_INFO("slam", "FAST-LIO2 backend initialized (占位实现，需替换为真实 ESKF 调用)");
    return 0;
}

static void fast_lio2_cleanup(void) {
    memset(&g_fast_lio2, 0, sizeof(g_fast_lio2));
}

static void slam_update_fast_lio2(Pose2D* pose) {
    (void)pose;
    static int warned = 0;
    if (!warned) {
        LOG_WARN("slam", "FAST-LIO2 占位实现：slam_update_fast_lio2() 未填充真实 ESKF 调用，"
                 "本帧降级到 dead reckoning。请按 slam_node.cpp 文件头注释替换实现。");
        warned = 1;
    }
    slam_update_dead_reckon(pose);
}

#endif

static int ekf_slam_init_impl(float initial_x, float initial_y, float initial_heading) {
    ekf_slam_init(&g.ekf_slam_state, initial_x, initial_y, initial_heading);
    LOG_INFO("slam", "EKF-SLAM backend initialized (5-state: x,y,heading,v,omega)");
    return 0;
}

static void ekf_slam_cleanup_impl(void) {
    memset(&g.ekf_slam_state, 0, sizeof(g.ekf_slam_state));
}

static void slam_update_ekf_slam(Pose2D* pose) {
    uint64_t now = clock_now_us();

    pthread_mutex_lock(&g.lock);

    if (g.have_imu) {
        ekf_slam_predict(&g.ekf_slam_state, g.last_imu.accel_x, g.last_imu.gyro_z, now);
    }

    if (g.have_lidar) {
        ekf_slam_update(&g.ekf_slam_state,
                        g.last_lidar.x, g.last_lidar.y, g.pose_heading);
    }

    ekf_slam_get_pose(&g.ekf_slam_state,
                      &g.pose_x, &g.pose_y, &g.pose_heading,
                      &g.pose_cov_xx, &g.pose_cov_yy, &g.pose_cov_hh);

    g.speed = g.ekf_slam_state.x.v;
    g.last_slam_update_us = now;

    pthread_mutex_unlock(&g.lock);

    pose->x         = g.pose_x;
    pose->y         = g.pose_y;
    pose->heading    = g.pose_heading;
    pose->cov_xx    = g.pose_cov_xx;
    pose->cov_yy    = g.pose_cov_yy;
    pose->cov_hh    = g.pose_cov_hh;
    pose->converged = true;
    pose->source    = POSE_SOURCE_SLAM;
}

static void slam_update_dead_reckon(Pose2D* pose) {
    uint64_t now = clock_now_us();
    float dt = 0.0f;
    if (g.last_slam_update_us == 0) {
        g.last_slam_update_us = now;
    } else {
        dt = (float)((double)(now - g.last_slam_update_us) / 1000000.0);
        g.last_slam_update_us = now;
        if (dt <= 0.0f || dt > 1.0f) dt = 0.0f;
    }

    pthread_mutex_lock(&g.lock);

    if (g.have_imu && dt > 0.0f) {
        g.speed += g.last_imu.accel_x * dt;
        if (g.speed >  50.0f) g.speed =  50.0f;
        if (g.speed < -10.0f) g.speed = -10.0f;
    }

    if (dt > 0.0f) {
        g.pose_x += cosf(g.pose_heading) * g.speed * dt;
        g.pose_y += sinf(g.pose_heading) * g.speed * dt;

        g.pose_cov_xx += 0.05f * dt;
        g.pose_cov_yy += 0.05f * dt;
        g.pose_cov_hh += 0.01f * dt;
        if (g.pose_cov_xx > 100.0f) g.pose_cov_xx = 100.0f;
        if (g.pose_cov_yy > 100.0f) g.pose_cov_yy = 100.0f;
        if (g.pose_cov_hh >  10.0f) g.pose_cov_hh =  10.0f;
    }

    pose->x         = g.pose_x;
    pose->y         = g.pose_y;
    pose->heading    = g.pose_heading;
    pose->cov_xx    = g.pose_cov_xx;
    pose->cov_yy    = g.pose_cov_yy;
    pose->cov_hh    = g.pose_cov_hh;
    pose->converged = true;
    pose->source    = POSE_SOURCE_ODOMETRY;

    pthread_mutex_unlock(&g.lock);
}

static void slam_update(Pose2D* pose) {
#ifdef HAVE_FAST_LIO2
    if (strcmp(g.algo, "fast_lio2") == 0) {
        slam_update_fast_lio2(pose);
        return;
    }
#endif
    if (strcmp(g.algo, "ekf_slam") == 0) {
        slam_update_ekf_slam(pose);
        return;
    }
    (void)g;
    slam_update_dead_reckon(pose);
}

static int slam_execute(TaskBase* task) {
    pthread_setname_np(pthread_self(), "slam_node");

    long period_ms = 1000L / (g.publish_hz > 0 ? g.publish_hz : 20);

    while (!task->should_stop) {
        if (!g.enabled) {
            usleep((unsigned long)period_ms * 1000UL);
            continue;
        }

        long t0 = (long)(clock_now_realtime_us()/1000);

        Pose2D pose;
        memset(&pose, 0, sizeof(pose));

        if (g.dry_run) {
            double t = (double)g.poses_published / (double)(g.publish_hz > 0 ? g.publish_hz : 20);
            float  R = 10.0f;
            pose.x         = R * (float)cos(t);
            pose.y         = R * (float)sin(t);
            pose.heading    = (float)t + (float)(M_PI / 2.0);
            pose.cov_xx    = 0.1f;
            pose.cov_yy    = 0.1f;
            pose.cov_hh    = 0.05f;
            pose.converged = true;
            pose.source    = POSE_SOURCE_SLAM;
        } else {
            slam_update(&pose);
        }

        uint8_t buf[64];
        size_t  len = 0;
        if (Pose2D_serialize(&pose, buf, &len) == 0 && len > 0) {
            transport_publish(g.transport, "sensor/pose", buf, (uint32_t)len);
            g.poses_published++;
        }

        if (g.poses_published % 100 == 1) {
            LOG_INFO("slam", "pose #%lu x=%.2f y=%.2f hdg=%.3f spd=%.2f cov=[%.3f,%.3f,%.3f] %s",
                     (unsigned long)g.poses_published,
                     pose.x, pose.y, pose.heading, g.dry_run ? 0.0f : g.speed,
                     pose.cov_xx, pose.cov_yy, pose.cov_hh,
                     g.dry_run ? "[DRY-RUN]" : (g.have_imu ? "[LIVE]" : "[NO-IMU]"));
        }

        long elapsed = (long)(clock_now_realtime_us()/1000) - t0;
        long remain  = period_ms - elapsed;
        if (remain > 0) usleep((unsigned long)remain * 1000UL);
    }
    return 0;
}

static const TaskInterface slam_vtable = {
    .execute = slam_execute,
};

static const char* s_inputs[]  = { "sensor/lidar", "sensor/imu", NULL };
static const char* s_outputs[] = { "sensor/pose", NULL };

static int slam_init(MessageBus* bus, Transport* transport,
                     DiscoveryManager* discovery, Scheduler* scheduler,
                     const char* params_json) {
    (void)bus;
    memset(&g, 0, sizeof(g));
    g.transport = transport;
    g.discovery = discovery;
    g.scheduler = scheduler;

    pthread_mutex_init(&g.lock, NULL);

    g.enabled         = 1;
    g.publish_hz      = 20;
    g.dry_run         = 0;
    g.use_lidar       = 1;
    g.use_imu         = 1;
    g.initial_x       = 0.0f;
    g.initial_y       = 0.0f;
    g.initial_heading = 0.0f;
    snprintf(g.algo, sizeof(g.algo), "dead_reckon");

    if (params_json) {
        cJSON* root = cJSON_Parse(params_json);
        if (root) {
            cJSON* j;
            g.enabled = 1;
            if ((j = cJSON_GetObjectItem(root, "enable")) && cJSON_IsNumber(j))
                g.enabled = j->valueint;
            g.publish_hz = 20;
            if ((j = cJSON_GetObjectItem(root, "publish_hz")) && cJSON_IsNumber(j))
                g.publish_hz = j->valueint;
            g.dry_run = 0;
            if ((j = cJSON_GetObjectItem(root, "dry_run")) && cJSON_IsNumber(j))
                g.dry_run = j->valueint;
            g.use_lidar = 1;
            if ((j = cJSON_GetObjectItem(root, "use_lidar")) && cJSON_IsNumber(j))
                g.use_lidar = j->valueint;
            g.use_imu = 1;
            if ((j = cJSON_GetObjectItem(root, "use_imu")) && cJSON_IsNumber(j))
                g.use_imu = j->valueint;
            g.initial_x = (float)0.0;
            if ((j = cJSON_GetObjectItem(root, "initial_x")) && cJSON_IsNumber(j))
                g.initial_x = (float)j->valuedouble;
            g.initial_y = (float)0.0;
            if ((j = cJSON_GetObjectItem(root, "initial_y")) && cJSON_IsNumber(j))
                g.initial_y = (float)j->valuedouble;
            g.initial_heading = (float)0.0;
            if ((j = cJSON_GetObjectItem(root, "initial_heading")) && cJSON_IsNumber(j))
                g.initial_heading = (float)j->valuedouble;
            snprintf(g.algo, sizeof(g.algo), "%s", "dead_reckon");
            if ((j = cJSON_GetObjectItem(root, "algo")) && cJSON_IsString(j))
                snprintf(g.algo, sizeof(g.algo), "%s", j->valuestring);
            cJSON_Delete(root);
        }
    }

    if (strcmp(g.algo, "dead_reckon") == 0) {
    } else if (strcmp(g.algo, "ekf_slam") == 0) {
        ekf_slam_init_impl(g.initial_x, g.initial_y, g.initial_heading);
        LOG_INFO("slam", "EKF-SLAM 后端已启用（5维状态估计器：x,y,heading,v,omega）");
    } else if (strcmp(g.algo, "fast_lio2") == 0) {
#ifdef HAVE_FAST_LIO2
        if (fast_lio2_init() != 0) {
            LOG_WARN("slam", "FAST-LIO2 init 失败，降级到 dead_reckon");
            snprintf(g.algo, sizeof(g.algo), "dead_reckon");
        } else {
            LOG_INFO("slam", "FAST-LIO2 后端已启用（占位实现，需填充真实 ESKF 调用）");
        }
#else
        LOG_WARN("slam", "algo='fast_lio2' 未编译：需 cmake -DENABLE_FAST_LIO2=ON "
                 "(并预装 PCL/Eigen/Sophus + libfast_lio2.a)。降级到 dead_reckon。");
        snprintf(g.algo, sizeof(g.algo), "dead_reckon");
#endif
    } else if (strcmp(g.algo, "lio_sam") == 0) {
        LOG_WARN("slam", "algo='lio_sam' 暂未提供编译开关，降级到 dead_reckon。");
        snprintf(g.algo, sizeof(g.algo), "dead_reckon");
    } else {
        LOG_WARN("slam", "未知 algo='%s'，使用 dead_reckon", g.algo);
        snprintf(g.algo, sizeof(g.algo), "dead_reckon");
    }

    g.pose_x       = g.initial_x;
    g.pose_y       = g.initial_y;
    g.pose_heading = g.initial_heading;
    g.pose_cov_xx  = 0.01f;
    g.pose_cov_yy  = 0.01f;
    g.pose_cov_hh  = 0.01f;
    g.speed        = 0.0f;

    g.last_imu_time = time(NULL);

    if (!g.enabled) {
        LOG_INFO("slam", "disabled by config (enable=0), will not subscribe/publish");
        return 0;
    }

    if (g.use_lidar) {
        transport_subscribe(transport, "sensor/lidar", on_lidar, NULL);
        discovery_advertise(discovery, "sensor/lidar",
                            LIDARFRAME_TYPE_ID, CAP_SUBSCRIBER, 0);
    }
    if (g.use_imu) {
        transport_subscribe(transport, "sensor/imu", on_imu, NULL);
        discovery_advertise(discovery, "sensor/imu",
                            IMUDATA_TYPE_ID, CAP_SUBSCRIBER, 0);
    }

    discovery_advertise(discovery, "sensor/pose",
                        POSE2D_TYPE_ID, CAP_PUBLISHER, (double)g.publish_hz);

    TaskConfig cfg;
    memset(&cfg, 0, sizeof(cfg));
    snprintf(cfg.name, sizeof(cfg.name), "slam");
    cfg.priority         = TASK_PRIORITY_NORMAL;
    cfg.max_frequency_hz = (double)g.publish_hz;
    cfg.enable_stats     = true;
    if (task_base_init(&g.taskbase, &slam_vtable, &cfg) != 0) {
        LOG_WARN("slam", "task_base_init failed");
        return -1;
    }

    LOG_INFO("slam", "initialized: algo=%s hz=%d lidar=%d imu=%d "
             "init=[%.1f,%.1f,%.3f] %s",
             g.algo, g.publish_hz, g.use_lidar, g.use_imu,
             g.initial_x, g.initial_y, g.initial_heading,
             g.dry_run ? "[DRY-RUN]" : "[LIVE]");

    if (g.dry_run) {
        LOG_INFO("slam", "DRY-RUN 模式：生成圆形轨迹模拟位姿(R=10m, ω=1rad/s)发布到 "
                 "sensor/pose，供 fusion/planning 算法链测通。"
                 "真车部署时关闭 dry_run，连接 LiDAR+IMU 并替换 slam_update() "
                 "为 EKF-SLAM/FAST-LIO2/LIO-SAM（集成方式见文件头注释）。");
    } else if (strcmp(g.algo, "ekf_slam") == 0) {
        LOG_INFO("slam", "EKF-SLAM 模式：基于 IMU 的 5 维状态估计器，支持 LiDAR "
                 "观测校正。协方差随状态转移传播，精度优于 dead reckoning。");
    } else {
        LOG_INFO("slam", "默认 dead reckoning 占位实现，精度有限（只适合短时间/低速）。"
                 "建议启用 algo='ekf_slam' 或编译 ENABLE_FAST_LIO2。");
    }

    return 0;
}

static int slam_start(void) {
    if (!g.enabled) return 0;
    int rc = node_start_managed(&s_plugin, g.scheduler);
    if (rc != 0) {
        LOG_WARN("slam", "node_start_managed failed: %d", rc);
        return rc;
    }
    LOG_INFO("slam", "started (managed) (publish %dHz, algo=%s, %s)",
             g.publish_hz, g.algo, g.dry_run ? "dry-run" : "live");
    node_announce_self(g.transport, &s_plugin);
    return 0;
}

static void slam_stop(void) {
    task_stop(&g.taskbase);
}

static void slam_cleanup(void) {
    task_stop(&g.taskbase);
    task_base_destroy(&g.taskbase);
    pthread_mutex_destroy(&g.lock);
    if (strcmp(g.algo, "ekf_slam") == 0) {
        ekf_slam_cleanup_impl();
    }
#ifdef HAVE_FAST_LIO2
    if (strcmp(g.algo, "fast_lio2") == 0) {
        fast_lio2_cleanup();
    }
#endif
    LOG_INFO("slam", "cleanup: poses=%lu lidar_frames=%lu imu_samples=%lu",
             (unsigned long)g.poses_published,
             (unsigned long)g.lidar_frames_received,
             (unsigned long)g.imu_samples_received);
}

static int slam_health(void) {
    if (!g.enabled) return 0;
    if (g.dry_run) return 0;
    if (!g.use_imu) return 0;
    time_t now = time(NULL);
    if (g.imu_samples_received == 0 && (now - g.last_imu_time) > 5) return -1;
    return 0;
}

NodePlugin s_plugin = {
    .api_version   = NODE_PLUGIN_API_VERSION,
    .name          = "slam",
    .version       = "1.0.0",
    .description   = "SLAM adapter (LiDAR + IMU → Pose2D on sensor/pose, dead-reckon default, EKF-SLAM/FAST-LIO2 hook)",
    .input_topics  = s_inputs,
    .output_topics = s_outputs,
    .init          = slam_init,
    .start         = slam_start,
    .stop          = slam_stop,
    .cleanup       = slam_cleanup,
    .health        = slam_health,
    .taskbase      = &g.taskbase,
};

NodePlugin* node_get_plugin(void) { return &s_plugin; }
