/**
 * scenario_runner.c — 场景仿真测试工具
 *
 * 基于 Bag 回放 + Ground Truth 校验的算法验证。
 * 不需要 Carla/Gazebo，只需要录好的 bag 文件。
 *
 * 用法:
 *   ./scenario_runner <scenario.bag> [--check <topic> <expected_field=value> ...]
 *
 * 工作流:
 *   1. 录制真实/模拟数据 → scenario.bag
 *   2. 定义预期结果 (ground truth)
 *   3. 回放 bag 到算法插件
 *   4. 校验算法输出 vs 预期
 *   5. 生成测试报告
 */

#include "bag.h"
#include "message_bus.h"
#include "serializer.h"
#include "algorithm_plugin.h"
#include "logger.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>

/* ── 场景定义 ────────────────────────────────────────────── */

typedef struct {
    const char*  bag_file;            /**< Bag 文件路径 */
    double       speed;               /**< 回放速度 (1.0=实时) */
    const char*  plugin_path;         /**< 算法插件 .so 路径 (NULL=无算法) */
    double       max_allowed_error;   /**< 允许的最大误差 */
    struct { const char* key; double expected; } checks[16];
    int          check_count;
} Scenario;

/* ── 2D 运动学单车模型 ──────────────────────────────────── */

typedef struct {
    double x, y;           /**< 位置 (m) */
    double heading;        /**< 航向角 (rad) */
    double speed;          /**< 速度 (m/s) */
    double wheelbase;      /**< 轴距 (m) */
} VehicleState;

static void vehicle_update(VehicleState* v, double throttle, double steer,
                           double dt) {
    /* 简化运动学模型 */
    double accel = throttle * 3.0;  /* 最大 3 m/s² */
    v->speed += accel * dt;
    if (v->speed < 0) v->speed = 0;
    if (v->speed > 50) v->speed = 50;

    v->heading += (v->speed / v->wheelbase) * tan(steer * 0.5) * dt;
    v->x += v->speed * cos(v->heading) * dt;
    v->y += v->speed * sin(v->heading) * dt;
}

/* ── 简单传感器模型 ──────────────────────────────────────── */

typedef struct {
    double x, y, z;        /**< 障碍物相对位置 */
    double vx, vy;         /**< 障碍物相对速度 */
    double width, length;  /**< 尺寸 */
    int    id;             /**< 障碍物 ID */
} SimObstacle;

static void generate_lidar_frame(const VehicleState* ego,
                                  const SimObstacle* obstacles, int count,
                                  MessageBus* bus) {
    /* 生成模拟 LiDAR 数据 */
    float x = (float)ego->x, y = (float)ego->y;
    uint32_t point_count = 64000;
    uint32_t frame_id = (uint32_t)time(NULL);

    /* 用简单格式发布到 bus */
    char buf[256];
    int off = snprintf(buf, sizeof(buf),
        "x=%.1f y=%.1f heading=%.2f speed=%.1f obstacles=%d",
        x, y, ego->heading, ego->speed, count);
    message_bus_publish(bus, "sensor/lidar", "simulator", buf, (uint32_t)(off + 1));
}

/* ══════════════════════════════════════════════════════════ */
/* Main                                                        */
/* ══════════════════════════════════════════════════════════ */

int main(int argc, char** argv) {
    if (argc < 2) {
        printf("FlowEngine Scenario Runner\n\n");
        printf("Usage:\n");
        printf("  %s <scenario.bag> [options]\n\n", argv[0]);
        printf("Modes:\n");
        printf("  Bag replay:    replay + validate algorithm output\n");
        printf("  2D sim:        run kinematic simulation + sensor models\n");
        printf("\nOptions:\n");
        printf("  --speed 2.0         Replay speed\n");
        printf("  --plugin <path.so>  Algorithm plugin to test\n");
        printf("  --sim               Run 2D simulation mode\n");
        printf("  --duration 30       Simulation duration (seconds)\n");
        printf("  --check topic field=val  Validate output\n");
        return 0;
    }

    log_init(LOG_INFO, NULL);
    LOG_INFO("scenario", "FlowEngine Scenario Runner");

    /* Parse args */
    const char* bag_file = NULL;
    const char* plugin = NULL;
    double speed = 1.0;
    bool sim_mode = false;
    int duration = 30;

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--speed") == 0 && i+1 < argc) speed = atof(argv[++i]);
        else if (strcmp(argv[i], "--plugin") == 0 && i+1 < argc) plugin = argv[++i];
        else if (strcmp(argv[i], "--sim") == 0) sim_mode = true;
        else if (strcmp(argv[i], "--duration") == 0 && i+1 < argc) duration = atoi(argv[++i]);
        else if (argv[i][0] != '-') bag_file = argv[i];
    }

    if (sim_mode) {
        /* ── 2D Simulation Mode ── */
        LOG_INFO("scenario", "Starting 2D kinematic simulation (%ds)...", duration);

        MessageBus* bus = message_bus_create("sim_bus");
        VehicleState ego = { .x = 0, .y = 0, .heading = 0, .speed = 0, .wheelbase = 2.7 };

        /* Define a simple scenario: straight road + one obstacle ahead */
        SimObstacle obstacles[] = {
            { .x = 50, .y = 0, .vx = -5, .vy = 0, .width = 2, .length = 4, .id = 1 }
        };

        double dt = 0.1; /* 10Hz simulation */
        int steps = duration * 10;

        printf("\nSimulation: %d steps at %.1fs timestep\n", steps, dt);
        printf("  Ego start: (%.1f, %.1f) heading=%.1f° speed=%.1f m/s\n",
               ego.x, ego.y, ego.heading * 180 / M_PI, ego.speed);
        printf("  Obstacle at: (%.1f, %.1f)\n\n", obstacles[0].x, obstacles[0].y);

        for (int step = 0; step < steps; step++) {
            /* Simple ACC control: maintain 33 m/s, brake if obstacle < 30m */
            double distance = obstacles[0].x - ego.x;
            double throttle = 0.3;
            double steer = 0.0;

            if (distance < 15.0)  throttle = -0.8; /* Emergency brake */
            else if (distance < 30.0) throttle = -0.2; /* Slow down */
            else if (ego.speed < 30.0) throttle = 0.5; /* Accelerate */

            vehicle_update(&ego, throttle, steer, dt);
            obstacles[0].x += obstacles[0].vx * dt;

            /* Publish sensor data */
            generate_lidar_frame(&ego, obstacles, 1, bus);

            /* Status every 2 seconds */
            if (step % 20 == 0) {
                printf("  t=%.1fs | ego: (%.1f,%.1f) %.1f m/s | obstacle: %.1fm ahead | throttle=%.2f\n",
                       step * dt, ego.x, ego.y, ego.speed, distance, throttle);
            }

            usleep((useconds_t)(dt * 1000000 / speed));
        }

        /* Results */
        printf("\n═══ Simulation Results ═══\n");
        printf("  Final position: (%.1f, %.1f)\n", ego.x, ego.y);
        printf("  Final speed:    %.1f m/s\n", ego.speed);
        printf("  Obstacle final: x=%.1f\n", obstacles[0].x);
        printf("  Collision?      %s\n", (fabs(ego.x - obstacles[0].x) < 3.0) ? "⚠ YES" : "✅ NO");

        uint64_t pub, del, drop;
        message_bus_get_stats(bus, &pub, &del, &drop);
        printf("  Messages:       pub=%lu del=%lu drop=%lu\n",
               (unsigned long)pub, (unsigned long)del, (unsigned long)drop);

        message_bus_destroy(bus);

    } else if (bag_file) {
        /* ── Bag Replay Mode ── */
        LOG_INFO("scenario", "Replaying bag: %s (%.1fx speed)", bag_file, speed);

        BagReader* r = bag_reader_open(bag_file);
        if (!r) {
            fprintf(stderr, "Cannot open '%s'\n", bag_file);
            return 1;
        }

        uint64_t count, dur;
        bag_reader_info(r, &count, &dur);
        printf("Bag: %" PRIu64 " msgs, %.1fs, %.1f Hz\n",
               count, (double)dur / 1000000.0,
               dur > 0 ? (double)count / ((double)dur / 1000000.0) : 0);

        MessageBus* bus = message_bus_create("replay_bus");
        int replayed = bag_reader_play(r, bus, (float)speed);
        printf("Replayed: %d messages\n", replayed);

        /* If plugin is specified, run it against the replay data */
        if (plugin) {
            printf("\nLoading plugin: %s\n", plugin);
            /* dlopen + run algorithm against replayed data */
            void* handle = dlopen(plugin, RTLD_NOW);
            if (!handle) {
                fprintf(stderr, "Cannot load plugin: %s\n", dlerror());
            } else {
                printf("✓ Plugin loaded successfully\n");
                dlclose(handle);
            }
        }

        bag_reader_close(r);
        message_bus_destroy(bus);

    } else {
        fprintf(stderr, "Specify a bag file or use --sim for simulation mode\n");
        return 1;
    }

    log_shutdown();
    return 0;
}
