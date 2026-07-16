/**
 * slam_node.c — SLAM 适配层节点插件 (LiDAR + IMU → 2D 位姿 Pose2D)
 *
 * 为 GPS 丢失场景（室内/隧道/地下停车场）提供定位。订阅 sensor/lidar 与
 * sensor/imu，运行 SLAM 算法发布 2D 位姿到 sensor/pose。这是 FlowEngine 在
 * GPS 不可用环境下的定位入口，与 gps_driver_node 互补：
 *
 *   有 GPS: [gps_driver_node] → sensor/gps → fusion_node (主定位)
 *   无 GPS: [slam_node] → sensor/pose → fusion_node (接管定位)
 *
 * 核心设计——SLAM 算法作为可替换钩子：
 *   本节点是**骨架/适配层**，真实 SLAM 算法（FAST-LIO2 / LIO-SAM /
 *   Cartographer）作为外部库通过 slam_update() 钩子接入。默认实现一个
 *   **简易里程计（dead reckoning）**：
 *     - 用 IMU 陀螺仪 gyro_z 积分 heading
 *     - 假设恒速（或用加速度计 accel 积分速度）推算 x/y
 *   dead reckoning 精度有限，只适合短时间/低速，误差随时间累积发散。
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
 *   4. 把 .c 改为 .cpp（FAST-LIO2 是 C++，需要 ESKF 模板），
 *      或在 slam_update() 里用 extern "C++" 桥接。
 *
 * ── 集成 LIO-SAM 步骤类似 ──
 *   LIO-SAM 在因子图上融合 LiDAR里程计/IMU/GPS/回环，精度更高但依赖 GTSAM。
 *   同样在 slam_update() 里调用其 optimize()，结果填到 Pose2D。
 *
 * ── 也可作为纯里程计节点 ──
 *   source=POSE_SOURCE_ODOMETRY(3) 时，下游 fusion_node 把本节点输出当作
 *   里程计观测与 GPS 融合；source=POSE_SOURCE_SLAM(2) 时视为独立定位源。
 *
 * 话题契约：
 *   输入: sensor/lidar (LidarFrame 二进制, type_id=0xd712aa51)
 *         sensor/imu   (ImuData  二进制, type_id=0x7dc626af)
 *   输出: sensor/pose  (Pose2D   二进制, type_id=0x026c6093)
 *
 * 典型 pipeline.json 配置:
 *   {
 *     "name": "slam",
 *     "library": "libslam_node.so",
 *     "params": {
 *       "enable": true,
 *       "publish_hz": 20,
 *       "dry_run": false,
 *       "use_lidar": true,
 *       "use_imu": true,
 *       "initial_x": 0.0,
 *       "initial_y": 0.0,
 *       "initial_heading": 0.0,
 *       "algo": "dead_reckon"
 *     }
 *   }
 *
 * 编译依赖: adas_msgs_gen.h (LidarFrame/ImuData/Pose2D 序列化), node_plugin.h
 */

#include "node_plugin.h"
#include "adas_msgs_gen.h"
#include "transport.h"
#include "discovery.h"
#include "logger.h"
#include "clock_service.h"

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

/* ── Pose2D.source 取值（定位来源标识，下游 fusion 区分信任度） ── */
#define POSE_SOURCE_SLAM      2u   /* SLAM 紧耦合定位（高精度，FAST-LIO2/LIO-SAM） */
#define POSE_SOURCE_ODOMETRY  3u   /* 里程计/航位推算（低精度，dead reckoning） */

/* ── 节点状态 ─────────────────────────────────────────────── */

static struct {
    Transport*        transport;
    DiscoveryManager* discovery;

    /* 配置参数（pipeline.json 注入） */
    int    enabled;            /* 总开关，false=不订阅也不发布 */
    int    publish_hz;        /* 位姿发布频率，默认 20Hz */
    int    dry_run;           /* true=生成圆形轨迹模拟位姿，不依赖传感器（开发调试） */
    int    use_lidar;         /* true=订阅 sensor/lidar（FAST-LIO2 需要） */
    int    use_imu;           /* true=订阅 sensor/imu（dead reckoning 必需） */
    float  initial_x;         /* 初始位姿 x (m)，默认 0 */
    float  initial_y;         /* 初始位姿 y (m)，默认 0 */
    float  initial_heading;   /* 初始位姿 heading (rad)，默认 0 */
    char   algo[32];          /* SLAM 算法选择，默认 "dead_reckon" */

    /* 当前位姿（slam_update 维护，mutex 保护） */
    float  pose_x;
    float  pose_y;
    float  pose_heading;      /* rad，[-pi, pi] */
    float  pose_cov_xx;       /* x 不确定度方差，随时间累积 */
    float  pose_cov_yy;       /* y 不确定度方差 */
    float  pose_cov_hh;       /* heading 不确定度方差 */
    float  speed;             /* 当前速度估计 (m/s)，dead reckoning 用 */

    /* 最新缓存输入（订阅回调写入，slam_update 读取，mutex 保护） */
    LidarFrame last_lidar;
    uint64_t   last_lidar_us;
    int        have_lidar;
    ImuData    last_imu;
    uint64_t   last_imu_us;
    int        have_imu;
    uint64_t   last_slam_update_us;   /* 上次 slam_update 时刻（us），用于算 dt */

    /* 线程同步：保护 last_imu/last_lidar/pose_* 等共享状态 */
    pthread_mutex_t lock;

    /* 统计 */
    uint64_t poses_published;
    uint64_t lidar_frames_received;
    uint64_t imu_samples_received;
    time_t   last_imu_time;   /* 最近一次收到 IMU 的墙上时间（health 用） */

    /* 工作线程 */
    pthread_t thread;
    volatile int thread_running;
    volatile int should_stop;
} g;

/* ── 参数解析（复用 lidar_driver_node.c / actuator_node.c 的模式） ──── */

static double parse_double(const char* json, const char* key, double default_val) {
    if (!json || !key) return default_val;
    char pat[64];
    snprintf(pat, sizeof(pat), "\"%s\"", key);
    const char* p = strstr(json, pat);
    if (!p) return default_val;
    p = strchr(p + strlen(pat), ':');
    if (!p) return default_val;
    p++;
    while (*p == ' ' || *p == '\t') p++;
    return strtod(p, NULL);
}

static int parse_int(const char* json, const char* key, int default_val) {
    return (int)parse_double(json, key, (double)default_val);
}

static void parse_string(const char* json, const char* key, char* out, size_t out_sz, const char* default_val) {
    if (!json || !key || !out || out_sz == 0) {
        if (default_val) snprintf(out, out_sz, "%s", default_val);
        return;
    }
    char pat[64];
    snprintf(pat, sizeof(pat), "\"%s\"", key);
    const char* p = strstr(json, pat);
    if (!p) { if (default_val) snprintf(out, out_sz, "%s", default_val); return; }
    p = strchr(p + strlen(pat), ':');
    if (!p) { if (default_val) snprintf(out, out_sz, "%s", default_val); return; }
    p++;
    while (*p == ' ' || *p == '\t') p++;
    if (*p != '"') { if (default_val) snprintf(out, out_sz, "%s", default_val); return; }
    p++;
    size_t n = 0;
    while (*p && *p != '"' && n < out_sz - 1) out[n++] = *p++;
    out[n] = '\0';
}

/* ── 订阅回调：收到 sensor/lidar ─────────────────────────────
 * 反序列化 LidarFrame，缓存到 g.last_lidar（带时间戳）。
 * dead reckoning 不直接使用 LiDAR；真实 SLAM 算法（FAST-LIO2）会在
 * slam_update() 里读取 g.last_lidar 做点云配准。
 */
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

/* ── 订阅回调：收到 sensor/imu ───────────────────────────────
 * 反序列化 ImuData，缓存到 g.last_imu（带时间戳），同时用 gyro_z 高频率
 * 积分更新 heading（IMU 采样率通常高于 publish_hz，这里逐样本积分更准）。
 *
 * 注意：heading 积分放在 on_imu（高频率），位置积分放在 slam_update（按
 * publish_hz 周期），避免双重积分。若在 slam_update 里重新实现 heading 积分，
 * 注释掉本函数中的积分块即可。
 */
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
    /* 用陀螺仪 gyro_z 积分 heading（逐 IMU 样本，精度高于按 publish_hz 积分） */
    if (g.have_imu) {
        float dt = (float)((double)(now - g.last_imu_us) / 1000000.0);
        if (dt > 0.0f && dt < 1.0f) {  /* 丢弃异常跳变（首帧/回绕） */
            g.pose_heading += imu.gyro_z * dt;
            /* 归一化到 [-pi, pi] */
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

/* ── SLAM 算法钩子：slam_update ───────────────────────────────
 *
 * *** 这是 SLAM 算法的替换点 ***
 *
 * 本函数是分发器：按 g.algo 路由到具体实现：
 *   - "dead_reckon"  → slam_update_dead_reckon()（默认，总是编译）
 *   - "fast_lio2"    → slam_update_fast_lio2()（仅 HAVE_FAST_LIO2 编译时可用）
 *
 * 默认实现：dead reckoning（航位推算）。
 *   - heading 已由 on_imu 高频率积分维护，本函数读取最新 heading
 *   - 用加速度计 accel_x 积分速度（简化：假设 accel_x 为前向加速度，已去重力）
 *   - 沿 heading 方向推算 x/y：x += cos(heading)*speed*dt
 *   - 协方差随时间累积（dead reckoning 误差发散特性）
 *   - converged=true（dead reckoning 总是"收敛"但精度低，不代表高精度）
 *   - source=POSE_SOURCE_ODOMETRY(3)
 *
 * 精度限制：dead reckoning 是占位实现，只适合短时间/低速。真车部署应启用
 * HAVE_FAST_LIO2（LiDAR+IMU 紧耦合），集成步骤见 HARDWARE_DEPLOYMENT.md
 * "FAST-LIO2 集成"章节与文件头注释。
 *
 * @param pose  输出位姿（调用方已 memset 清零）
 */

/* ── FAST-LIO2 占位实现（仅 HAVE_FAST_LIO2 编译时启用） ────────
 *
 * 启用方式：cmake -DENABLE_FAST_LIO2=ON ...
 *   CMakeLists 会 target_compile_definitions(slam_node PRIVATE HAVE_FAST_LIO2)
 *   并链接 PCL/Eigen/Sophus/fast_lio2（参见 CMakeLists.txt 中 ENABLE_FAST_LIO2 块）。
 *
 * 集成模板：把下面的占位代码替换为真实 FAST-LIO2 调用。
 *   1. 顶部 #include "IKFoM_toolkit/esekfom/esekfom.hpp" + PCL 头
 *   2. fast_lio2_init(): esekfom_ = std::make_shared<esekfom::ESKF>();
 *      加载 LiDAR 内参/外参 (config/<lidar>.yaml)
 *   3. fast_lio2_update():
 *      esekfom_->predict(g.last_imu);          // IMU 预测（高频）
 *      esekfom_->update(g.last_lidar);         // LiDAR 配准更新
 *      state = esekfom_->get_state();
 *      pose->x = state.pos(0); pose->y = state.pos(1);
 *      pose->heading = state.rot.yaw();
 *      memcpy(pose->cov_*, state.cov, ...);
 *      pose->converged = true;
 *      pose->source = POSE_SOURCE_SLAM;        // 高精度源
 */

/* 前向声明：slam_update_fast_lio2 占位实现降级时会调 dead_reckon，而后者
 * 定义在下方，需提前声明避免 implicit declaration 警告。 */
static void slam_update_dead_reckon(Pose2D* pose);

#ifdef HAVE_FAST_LIO2

/* 真实编译时：用户在这里放 FAST-LIO2 的状态结构（esekfom 指针、点云缓冲等）。
 * 当前为占位，编译能过但运行时会 LOG_WARN 并降级到 dead reckoning。 */
typedef struct {
    int placeholder;  /* 替换为 esekfom::ESKF::Ptr esekfom_; 等 */
} FastLio2State;

static FastLio2State g_fast_lio2;

static int fast_lio2_init(void) {
    /* TODO: 初始化 ESKF、加载 LiDAR 内参/外参、分配点云缓冲 */
    memset(&g_fast_lio2, 0, sizeof(g_fast_lio2));
    LOG_INFO("slam", "FAST-LIO2 backend initialized (占位实现，需替换为真实 ESKF 调用)");
    return 0;
}

static void fast_lio2_cleanup(void) {
    /* TODO: 释放 ESKF/点云缓冲 */
    memset(&g_fast_lio2, 0, sizeof(g_fast_lio2));
}

static void slam_update_fast_lio2(Pose2D* pose) {
    /* TODO: 替换为真实 FAST-LIO2 调用：
     *   esekfom_->predict(g.last_imu);
     *   esekfom_->update(g.last_lidar);
     *   state = esekfom_->get_state();
     *   填 pose->x/y/heading/cov_* */
    (void)pose;
    /* 占位：首次调用警告一次，后续静默降级到 dead reckoning */
    static int warned = 0;
    if (!warned) {
        LOG_WARN("slam", "FAST-LIO2 占位实现：slam_update_fast_lio2() 未填充真实 ESKF 调用，"
                 "本帧降级到 dead reckoning。请按 slam_node.c 文件头注释替换实现。");
        warned = 1;
    }
    slam_update_dead_reckon(pose);
}

#endif /* HAVE_FAST_LIO2 */

/* ── dead reckoning 实现（默认，总是编译） ─────────────────── */
static void slam_update_dead_reckon(Pose2D* pose) {
    uint64_t now = clock_now_us();
    float dt = 0.0f;
    if (g.last_slam_update_us == 0) {
        g.last_slam_update_us = now;  /* 首次：只记录时刻，不积分 */
    } else {
        dt = (float)((double)(now - g.last_slam_update_us) / 1000000.0);
        g.last_slam_update_us = now;
        if (dt <= 0.0f || dt > 1.0f) dt = 0.0f;  /* 丢弃异常跳变 */
    }

    pthread_mutex_lock(&g.lock);

    /* 速度积分：从加速度计 accel_x 推算（简化模型）。
     * 注意：真实 IMU 的 accel 含重力分量，需先做重力补偿与坐标系变换。
     * 这里假设 accel_x 已是水平前向加速度（IMU 水平安装且去重）。
     * speed 未知时（无 IMU）dt=0，speed 保持 0，位置不变。 */
    if (g.have_imu && dt > 0.0f) {
        g.speed += g.last_imu.accel_x * dt;
        /* 速度限幅，避免积分漂移发散 */
        if (g.speed >  50.0f) g.speed =  50.0f;
        if (g.speed < -10.0f) g.speed = -10.0f;
    }

    /* 位置推算：沿 heading 方向匀速运动 */
    if (dt > 0.0f) {
        g.pose_x += cosf(g.pose_heading) * g.speed * dt;
        g.pose_y += sinf(g.pose_heading) * g.speed * dt;

        /* 不确定性随时间累积（dead reckoning 误差发散特性） */
        g.pose_cov_xx += 0.05f * dt;   /* ~0.05 m^2/s 位置发散率 */
        g.pose_cov_yy += 0.05f * dt;
        g.pose_cov_hh += 0.01f * dt;   /* heading 发散更慢 */
        /* 协方差上限，避免数值溢出 */
        if (g.pose_cov_xx > 100.0f) g.pose_cov_xx = 100.0f;
        if (g.pose_cov_yy > 100.0f) g.pose_cov_yy = 100.0f;
        if (g.pose_cov_hh >  10.0f) g.pose_cov_hh =  10.0f;
    }

    /* 填充输出位姿 */
    pose->x         = g.pose_x;
    pose->y         = g.pose_y;
    pose->heading    = g.pose_heading;
    pose->cov_xx    = g.pose_cov_xx;
    pose->cov_yy    = g.pose_cov_yy;
    pose->cov_hh    = g.pose_cov_hh;
    pose->converged = true;                 /* dead reckoning 总是"收敛"（但精度低） */
    pose->source    = POSE_SOURCE_ODOMETRY;  /* 标记为里程计源，下游 fusion 据此降权 */

    pthread_mutex_unlock(&g.lock);
}

/* ── 分发器：按 g.algo 路由到具体 SLAM 实现 ────────────────── */
static void slam_update(Pose2D* pose) {
#ifdef HAVE_FAST_LIO2
    if (strcmp(g.algo, "fast_lio2") == 0) {
        slam_update_fast_lio2(pose);
        return;
    }
#endif
    (void)g;  /* dead_reckon 不需要 g.algo，避免未用变量警告 */
    slam_update_dead_reckon(pose);
}

/* ── 工作线程：周期调用 slam_update → 序列化 → 发布 sensor/pose ── */

static void* slam_thread(void* arg) {
    (void)arg;
    pthread_setname_np(pthread_self(), "slam_node");

    long period_ms = 1000L / (g.publish_hz > 0 ? g.publish_hz : 20);

    while (g.thread_running && !g.should_stop) {
        if (!g.enabled) {
            usleep((unsigned long)period_ms * 1000UL);
            continue;
        }

        long t0 = (long)(clock_now_realtime_us()/1000);

        Pose2D pose;
        memset(&pose, 0, sizeof(pose));

        if (g.dry_run) {
            /* dry-run：生成沿圆形轨迹运动的模拟位姿，让下游链路能测通。
             * x = R*cos(t), y = R*sin(t), heading = t + pi/2（圆切线方向）
             * 角速度 1 rad/s，半径 R=10m，周期 ~6.28s。 */
            double t = (double)g.poses_published / (double)(g.publish_hz > 0 ? g.publish_hz : 20);
            float  R = 10.0f;
            pose.x         = R * (float)cos(t);
            pose.y         = R * (float)sin(t);
            pose.heading    = (float)t + (float)(M_PI / 2.0);
            pose.cov_xx    = 0.1f;   /* 模拟位姿低不确定度 */
            pose.cov_yy    = 0.1f;
            pose.cov_hh    = 0.05f;
            pose.converged = true;
            pose.source    = POSE_SOURCE_SLAM;  /* 模拟 SLAM 输出 */
        } else {
            /* 真实 SLAM / dead reckoning */
            slam_update(&pose);
        }

        /* 序列化 + 发布到 sensor/pose */
        uint8_t buf[64];
        size_t  len = 0;
        if (Pose2D_serialize(&pose, buf, &len) == 0 && len > 0) {
            transport_publish(g.transport, "sensor/pose", buf, (uint32_t)len);
            g.poses_published++;
        }

        /* 周期性日志（每 100 帧一次，避免刷屏） */
        if (g.poses_published % 100 == 1) {
            LOG_INFO("slam", "pose #%lu x=%.2f y=%.2f hdg=%.3f spd=%.2f cov=[%.3f,%.3f,%.3f] %s",
                     (unsigned long)g.poses_published,
                     pose.x, pose.y, pose.heading, g.dry_run ? 0.0f : g.speed,
                     pose.cov_xx, pose.cov_yy, pose.cov_hh,
                     g.dry_run ? "[DRY-RUN]" : (g.have_imu ? "[LIVE]" : "[NO-IMU]"));
        }

        /* 按周期定频：睡剩余时间 */
        long elapsed = (long)(clock_now_realtime_us()/1000) - t0;
        long remain  = period_ms - elapsed;
        if (remain > 0) usleep((unsigned long)remain * 1000UL);
    }
    return NULL;
}

/* ── NodePlugin 实现 ─────────────────────────────────────── */

static const char* s_inputs[]  = { "sensor/lidar", "sensor/imu", NULL };
static const char* s_outputs[] = { "sensor/pose", NULL };

static NodePlugin s_plugin;

static int slam_init(MessageBus* bus, Transport* transport,
                     DiscoveryManager* discovery, Scheduler* scheduler,
                     const char* params_json) {
    (void)bus; (void)scheduler;
    memset(&g, 0, sizeof(g));
    g.transport = transport;
    g.discovery = discovery;

    pthread_mutex_init(&g.lock, NULL);

    /* 默认参数 */
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
        g.enabled         = parse_int(params_json, "enable", 1);
        g.publish_hz      = parse_int(params_json, "publish_hz", 20);
        g.dry_run         = parse_int(params_json, "dry_run", 0);
        g.use_lidar       = parse_int(params_json, "use_lidar", 1);
        g.use_imu         = parse_int(params_json, "use_imu", 1);
        g.initial_x       = (float)parse_double(params_json, "initial_x", 0.0);
        g.initial_y       = (float)parse_double(params_json, "initial_y", 0.0);
        g.initial_heading = (float)parse_double(params_json, "initial_heading", 0.0);
        parse_string(params_json, "algo", g.algo, sizeof(g.algo), "dead_reckon");
    }

    /* algo 参数处理：未编译的算法优雅降级到 dead_reckon */
    if (strcmp(g.algo, "dead_reckon") == 0) {
        /* 默认实现，无需特殊处理 */
    } else if (strcmp(g.algo, "fast_lio2") == 0) {
#ifdef HAVE_FAST_LIO2
        /* FAST-LIO2 已编译：初始化后端 */
        if (fast_lio2_init() != 0) {
            LOG_WARN("slam", "FAST-LIO2 init 失败，降级到 dead_reckon");
            snprintf(g.algo, sizeof(g.algo), "dead_reckon");
        } else {
            LOG_INFO("slam", "FAST-LIO2 后端已启用（占位实现，需填充真实 ESKF 调用）");
        }
#else
        LOG_WARN("slam", "algo='fast_lio2' 未编译：需 cmake -DENABLE_FAST_LIO2=ON "
                 "(并预装 PCL/Eigen/Sophus + libfast_lio2.a)。降级到 dead_reckon。"
                 "集成步骤见 docs/HARDWARE_DEPLOYMENT.md#fast-lio2-集成");
        snprintf(g.algo, sizeof(g.algo), "dead_reckon");
#endif
    } else if (strcmp(g.algo, "lio_sam") == 0) {
        LOG_WARN("slam", "algo='lio_sam' 暂未提供编译开关，降级到 dead_reckon。"
                 "集成方式见 slam_node.c 文件头注释");
        snprintf(g.algo, sizeof(g.algo), "dead_reckon");
    } else {
        LOG_WARN("slam", "未知 algo='%s'，使用 dead_reckon", g.algo);
        snprintf(g.algo, sizeof(g.algo), "dead_reckon");
    }

    /* 初始位姿 */
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

    /* 订阅输入话题（transport_subscribe 注册回调 + discovery_advertise 通告订阅意图） */
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

    /* 广播 sensor/pose 为发布者（供 fusion_node 等订阅方发现） */
    discovery_advertise(discovery, "sensor/pose",
                        POSE2D_TYPE_ID, CAP_PUBLISHER, (double)g.publish_hz);

    LOG_INFO("slam", "initialized: algo=%s hz=%d lidar=%d imu=%d "
             "init=[%.1f,%.1f,%.3f] %s",
             g.algo, g.publish_hz, g.use_lidar, g.use_imu,
             g.initial_x, g.initial_y, g.initial_heading,
             g.dry_run ? "[DRY-RUN]" : "[LIVE]");

    if (g.dry_run) {
        LOG_INFO("slam", "DRY-RUN 模式：生成圆形轨迹模拟位姿(R=10m, ω=1rad/s)发布到 "
                 "sensor/pose，供 fusion/planning 算法链测通。"
                 "真车部署时关闭 dry_run，连接 LiDAR+IMU 并替换 slam_update() "
                 "为 FAST-LIO2/LIO-SAM（集成方式见文件头注释）。");
    } else {
        LOG_INFO("slam", "默认 dead reckoning 占位实现，精度有限（只适合短时间/低速）。"
                 "真车部署应替换 slam_update() 为 FAST-LIO2（LiDAR+IMU 紧耦合）。"
                 "集成方式见文件头注释。");
    }
    return 0;
}

static int slam_start(void) {
    if (!g.enabled) return 0;
    g.thread_running = 1;
    g.should_stop    = 0;
    if (pthread_create(&g.thread, NULL, slam_thread, NULL) != 0) {
        LOG_WARN("slam", "worker thread create failed");
        g.thread_running = 0;
        return -1;
    }
    LOG_INFO("slam", "started (publish %dHz, algo=%s, %s)",
             g.publish_hz, g.algo, g.dry_run ? "dry-run" : "live");
    node_announce_self(g.transport, &s_plugin);
    return 0;
}

static void slam_stop(void) {
    g.should_stop    = 1;
    g.thread_running = 0;
}

static void slam_cleanup(void) {
    g.thread_running = 0;
    g.should_stop    = 1;
    if (g.thread) { pthread_join(g.thread, NULL); g.thread = 0; }
    pthread_mutex_destroy(&g.lock);
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
    if (g.dry_run) return 0;     /* dry-run 不依赖真实传感器 */
    if (!g.use_imu) return 0;    /* 配置关闭 IMU 输入时不检查 */
    /* IMU 是 SLAM 必需输入：从未收到 IMU 数据且超过 5 秒 → 异常 */
    time_t now = time(NULL);
    if (g.imu_samples_received == 0 && (now - g.last_imu_time) > 5) return -1;
    return 0;
}

static NodePlugin s_plugin = {
    .api_version   = NODE_PLUGIN_API_VERSION,
    .name          = "slam",
    .version       = "1.0.0",
    .description   = "SLAM adapter (LiDAR + IMU → Pose2D on sensor/pose, dead-reckon default, FAST-LIO2/LIO-SAM hook)",
    .input_topics  = s_inputs,
    .output_topics = s_outputs,
    .init          = slam_init,
    .start         = slam_start,
    .stop          = slam_stop,
    .cleanup       = slam_cleanup,
    .health        = slam_health,
};

NodePlugin* node_get_plugin(void) { return &s_plugin; }
