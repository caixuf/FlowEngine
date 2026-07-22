/**
 * test_adas_nodes_logic.c — 真车部署 driver/节点 纯逻辑单测
 *
 * 覆盖 modules/adas_nodes/ 下的 driver 节点（imu / slam / actuator_pwm）
 * 的纯逻辑函数，dry-run 路径不依赖硬件、不依赖 transport/discovery/scheduler
 * 等运行时基础设施。这是真车部署 roadmap 的第一步：先把"两头"的占位实现
 * 的逻辑正确性钉死，再换真实算法/驱动时不会因为回归把已经被验证过的行为
 * 跑丢。
 *
 * ── 测试策略：副本 + 标注来源 ──
 * 节点 .c 文件的纯逻辑函数都是 `static`（不导出符号），且依赖全局 `g` 状态，
 * 无法直接 #include 进测试。本文件采取"副本 + 行号标注"方式：
 *   1. 把源文件里要测的纯逻辑函数原样复制到这里（保留可读性，不改逻辑）
 *   2. 在每个副本顶部用 `// adapted from <file>:<lines>` 标注来源
 *   3. 测试副本本身就能验证逻辑正确性
 *   4. 漂移风险：源码改了纯逻辑但没同步改副本时，CI 不报错——
 *      这是初始可接受的代价；后续若要把这套测试升级为"防漂移"，
 *      可把节点纯逻辑抽离到独立 .c/.h（imu_protocol / slam_dead_reckon / pwm_map）
 *      让节点和测试都链接同一份实现。
 *
 * 编译: cmake --build build --target test_adas_nodes_logic
 * 运行: ./build/bin/test_adas_nodes_logic
 */

#include "adas_msgs_gen.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static int g_passed = 0;
static int g_failed = 0;

#define TEST(name)  printf("  %-50s ", name)
#define PASS()      do { printf("✅ PASS\n"); g_passed++; } while(0)
#define FAIL(fmt, ...) do { printf("❌ FAIL: " fmt "\n", ##__VA_ARGS__); g_failed++; } while(0)
#define ASSERT(cond, fmt, ...) if (!(cond)) { FAIL(fmt, ##__VA_ARGS__); return; }
#define ASSERT_EQ(a, b, fmt, ...) if ((a) != (b)) { FAIL(fmt " (got %d, expected %d)", ##__VA_ARGS__, (int)(a), (int)(b)); return; }
#define ASSERT_NEAR(a, b, eps, fmt, ...) \
    if (fabs((double)(a) - (double)(b)) > (eps)) { \
        FAIL(fmt " (got %.6f, expected %.6f, eps=%.6f)", ##__VA_ARGS__, (double)(a), (double)(b), (double)(eps)); \
        return; \
    }

/* ══════════════════════════════════════════════════════════ */
/* 副本：imu_driver_node.c 的纯逻辑（parse_imu_line + make_synthetic_imu） */
/* ══════════════════════════════════════════════════════════ */

/* adapted from modules/adas_nodes/imu_driver_node.c:135-163
 * 原文件依赖 g.gravity，这里改为参数传入（解耦全局状态）。
 * 行为必须与源文件保持一致——任何修改都要同步两边。 */
static int parse_imu_line_for_test(const char* line, ImuData* out) {
    if (!line || !out) return -1;

    const char* p = line;
    while (*p == ' ' || *p == '\t') p++;                 /* 跳过前导空白 */
    if (*p == '\0' || *p == '\r' || *p == '\n') return -1;

    float v[7];
    int   cnt = 0;
    char* end = NULL;
    for (cnt = 0; cnt < 7; cnt++) {
        v[cnt] = strtof(p, &end);
        if (end == p) return -1;
        p = end;
        while (*p == ',' || *p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') p++;
    }

    out->accel_x = v[0];
    out->accel_y = v[1];
    out->accel_z = v[2];
    out->gyro_x  = v[3];
    out->gyro_y  = v[4];
    out->gyro_z  = v[5];
    out->temperature = v[6];
    return 0;
}

/* adapted from modules/adas_nodes/imu_driver_node.c:170-182
 * 原文件读 g.gravity，这里改为参数。噪声种子用固定值以便测试可重现。 */
static void make_synthetic_imu_for_test(ImuData* out, double gravity) {
    if (!out) return;
    /* 固定噪声序列（不调 rand，保证可重现）；幅度与原实现对齐：±0.01 / ±0.001 */
    static const float na_table[4] = { 0.005f, -0.003f, 0.008f, -0.006f };
    static const float ng_table[4] = { 0.0003f, -0.0002f, 0.0005f, -0.0001f };
    static int idx = 0;
    float na = na_table[idx & 3];
    float ng = ng_table[idx & 3];
    idx++;
    out->accel_x = na;
    out->accel_y = na * 0.5f;
    out->accel_z = (float)gravity + na;
    out->gyro_x  = ng;
    out->gyro_y  = ng * 0.5f;
    out->gyro_z  = ng;
    out->temperature = 25.0f + na * 10.0f;
}

/* ══════════════════════════════════════════════════════════ */
/* 副本：slam_node.c 的纯逻辑（dry-run 圆轨迹生成 + heading 归一化） */
/* ══════════════════════════════════════════════════════════ */

/* adapted from modules/adas_nodes/slam_node.c:385-402 (dry_run 圆轨迹块)
 * 把 g.poses_published / g.publish_hz / R=10 改成参数。
 * 注意：原代码角速度 ω=1 rad/s（t 单位是秒，t = poses_published / publish_hz）。 */
static void slam_dry_run_circle_pose(uint64_t poses_published, int publish_hz,
                                     float R, Pose2D* pose) {
    double t = (double)poses_published / (double)(publish_hz > 0 ? publish_hz : 20);
    pose->x         = R * (float)cos(t);
    pose->y         = R * (float)sin(t);
    pose->heading    = (float)t + (float)(M_PI / 2.0);
    pose->cov_xx    = 0.1f;
    pose->cov_yy    = 0.1f;
    pose->cov_hh    = 0.05f;
    pose->converged = true;
    /* source 在 slam_node.c 里硬编码为 POSE_SOURCE_SLAM=2，这里也保持 */
    pose->source    = 2u;
}

/* adapted from modules/adas_nodes/slam_node.c:198-199 (on_imu heading 归一化)
 * 把 [-pi, pi] 归一化逻辑提取为独立函数。原代码在循环里减/加 2π。 */
static float normalize_heading_pi(float h) {
    while (h >   (float)M_PI) h -= 2.0f * (float)M_PI;
    while (h < -(float)M_PI) h += 2.0f * (float)M_PI;
    return h;
}

/* ══════════════════════════════════════════════════════════ */
/* 副本：actuator_pwm_node.c 的纯逻辑（ControlCmd → PWM 映射） */
/* ══════════════════════════════════════════════════════════ */

/* adapted from modules/adas_nodes/actuator_pwm_node.c:117-118 + 272-291
 * PWM_CENTER_US=1500, PWM_RANGE_US=500（±500μs 对应 ±1.0 控制）
 * MAX_STEER_RAD=0.22 是源文件硬编码的 RC 小车值。
 * 这里把所有"配置常量"参数化，便于测不同配置下的映射。 */
#define PWM_CENTER_US   1500
#define PWM_RANGE_US    500
#define MAX_STEER_RAD   0.22   /* 与源文件 actuator_pwm_node.c:287 一致 */

static void map_control_cmd_to_pwm(double throttle, double brake, double steering_rad,
                                   int e_stop,
                                   double throttle_scale, double steering_scale,
                                   int* esc_us, int* steer_us) {
    int estop = e_stop ? 1 : 0;

    int esc;
    if (estop) {
        esc = PWM_CENTER_US;
    } else if (brake > 0.01) {
        esc = (int)(PWM_CENTER_US - brake * throttle_scale);
    } else {
        esc = (int)(PWM_CENTER_US + throttle * throttle_scale);
    }

    double steer_norm = steering_rad / MAX_STEER_RAD;
    if (steer_norm > 1.0) steer_norm = 1.0;
    if (steer_norm < -1.0) steer_norm = -1.0;
    int steer = (int)(PWM_CENTER_US + steer_norm * steering_scale);

    /* 源文件在 pwm_set_pulse 里做最终钳位 [1000, 2000]，这里同步做。 */
    if (esc < 1000) esc = 1000;
    if (esc > 2000) esc = 2000;
    if (steer < 1000) steer = 1000;
    if (steer > 2000) steer = 2000;

    if (esc_us)   *esc_us   = esc;
    if (steer_us) *steer_us = steer;
}

/* ══════════════════════════════════════════════════════════ */
/* IMU Driver Tests                                           */
/* ══════════════════════════════════════════════════════════ */

static void test_imu_parse_basic(void) {
    TEST("imu parse 'ax,ay,az,gx,gy,gz,temp'");
    ImuData imu;
    int rc = parse_imu_line_for_test("0.1,0.2,9.8,0.01,0.02,0.03,25.5", &imu);
    ASSERT(rc == 0, "should succeed on valid line");
    ASSERT_NEAR(imu.accel_x, 0.1, 1e-6, "ax mismatch");
    ASSERT_NEAR(imu.accel_y, 0.2, 1e-6, "ay mismatch");
    ASSERT_NEAR(imu.accel_z, 9.8, 1e-6, "az mismatch");
    ASSERT_NEAR(imu.gyro_x, 0.01, 1e-6, "gx mismatch");
    ASSERT_NEAR(imu.gyro_y, 0.02, 1e-6, "gy mismatch");
    ASSERT_NEAR(imu.gyro_z, 0.03, 1e-6, "gz mismatch");
    ASSERT_NEAR(imu.temperature, 25.5, 1e-6, "temp mismatch");
    PASS();
}

static void test_imu_parse_negative_values(void) {
    TEST("imu parse negative floats (heavy braking / reverse rotation)");
    ImuData imu;
    int rc = parse_imu_line_for_test("-3.5,-1.2,-9.8,-0.5,-0.3,-2.1,-10.0", &imu);
    ASSERT(rc == 0, "should succeed");
    ASSERT_NEAR(imu.accel_x, -3.5, 1e-6, "ax should be negative");
    ASSERT_NEAR(imu.accel_z, -9.8, 1e-6, "az should be -9.8 (inverted)");
    ASSERT_NEAR(imu.gyro_z, -2.1, 1e-6, "gz should be -2.1 (CCW rotation)");
    ASSERT_NEAR(imu.temperature, -10.0, 1e-6, "temp should be -10");
    PASS();
}

static void test_imu_parse_whitespace_and_crlf(void) {
    TEST("imu parse tolerates spaces / CR / LF");
    ImuData imu;
    int rc = parse_imu_line_for_test("  1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0\r\n", &imu);
    ASSERT(rc == 0, "should tolerate leading space + CR/LF");
    ASSERT_NEAR(imu.accel_x, 1.0, 1e-6, "ax mismatch");
    ASSERT_NEAR(imu.temperature, 7.0, 1e-6, "temp mismatch");
    PASS();
}

static void test_imu_parse_too_few_fields(void) {
    TEST("imu parse rejects too few fields");
    ImuData imu;
    int rc = parse_imu_line_for_test("1.0,2.0,3.0", &imu);
    ASSERT(rc == -1, "should reject 3-field line (need 7)");
    PASS();
}

static void test_imu_parse_empty_line(void) {
    TEST("imu parse rejects empty / whitespace-only line");
    ImuData imu;
    ASSERT(parse_imu_line_for_test("", &imu) == -1, "empty line should fail");
    ASSERT(parse_imu_line_for_test("   \t  ", &imu) == -1, "whitespace-only should fail");
    ASSERT(parse_imu_line_for_test(NULL, &imu) == -1, "NULL line should fail");
    ASSERT(parse_imu_line_for_test("1,2,3,4,5,6,7", NULL) == -1, "NULL out should fail");
    PASS();
}

static void test_imu_parse_non_numeric(void) {
    TEST("imu parse rejects non-numeric field");
    ImuData imu;
    int rc = parse_imu_line_for_test("abc,2.0,3.0,4.0,5.0,6.0,7.0", &imu);
    ASSERT(rc == -1, "non-numeric first field should fail");
    PASS();
}

static void test_imu_synthetic_static_gravity(void) {
    TEST("imu synthetic: accel_z ≈ gravity at rest");
    ImuData imu;
    make_synthetic_imu_for_test(&imu, 9.80665);
    /* accel_z 必须接近重力（±0.02 噪声范围） */
    ASSERT(fabs(imu.accel_z - 9.80665) < 0.02, "accel_z should be ≈ gravity (got %.4f)", imu.accel_z);
    /* 水平轴加速度远小于重力 */
    ASSERT(fabs(imu.accel_x) < 0.02, "accel_x should be near 0 (got %.4f)", imu.accel_x);
    ASSERT(fabs(imu.accel_y) < 0.02, "accel_y should be near 0 (got %.4f)", imu.accel_y);
    /* 角速度应接近 0（±0.001 范围） */
    ASSERT(fabs(imu.gyro_x) < 0.002, "gyro_x should be near 0 (got %.5f)", imu.gyro_x);
    ASSERT(fabs(imu.gyro_z) < 0.002, "gyro_z should be near 0 (got %.5f)", imu.gyro_z);
    /* 温度应在 25°C 附近（±0.2） */
    ASSERT(fabs(imu.temperature - 25.0) < 0.3, "temp should be ≈ 25°C (got %.2f)", imu.temperature);
    PASS();
}

static void test_imu_synthetic_custom_gravity(void) {
    TEST("imu synthetic: respects custom gravity (moon = 1.62)");
    ImuData imu;
    make_synthetic_imu_for_test(&imu, 1.62);
    ASSERT(fabs(imu.accel_z - 1.62) < 0.02, "accel_z should track custom gravity (got %.4f)", imu.accel_z);
    PASS();
}

/* ══════════════════════════════════════════════════════════ */
/* SLAM (dead reckoning dry-run circle) Tests                  */
/* ══════════════════════════════════════════════════════════ */

static void test_slam_circle_pose_t0(void) {
    TEST("slam dry-run circle at t=0: (R, 0), heading=π/2");
    Pose2D pose;
    slam_dry_run_circle_pose(0, 20, 10.0f, &pose);
    ASSERT_NEAR(pose.x, 10.0, 1e-4, "x should be R at t=0");
    ASSERT_NEAR(pose.y, 0.0, 1e-4, "y should be 0 at t=0");
    ASSERT_NEAR(pose.heading, M_PI / 2.0, 1e-4, "heading should be π/2 at t=0");
    ASSERT(pose.converged, "converged should be true");
    ASSERT(pose.source == 2u, "source should be POSE_SOURCE_SLAM=2");
    PASS();
}

static void test_slam_circle_pose_quarter(void) {
    TEST("slam dry-run circle at t≈π/2s: near top of circle, on-circle");
    /* 20Hz 离散采样无法精确命中 t=π/2≈1.5708s（需 poses_published=31.4），
     * 取最近的整数样本 poses_published=31 → t=1.55s。
     * 此时 x=10*cos(1.55)≈0.208, y=10*sin(1.55)≈9.998 —— 量化误差是
     * 采样的固有性质，不是 bug。改用「在圆上」不变量 + 位置方向断言更稳健。 */
    Pose2D pose;
    slam_dry_run_circle_pose(31, 20, 10.0f, &pose);
    /* 不变量：x² + y² = R²，对任意 t 恒成立（验证圆参数化正确） */
    float r2 = pose.x * pose.x + pose.y * pose.y;
    ASSERT_NEAR(r2, 100.0f, 0.1f, "should be on circle of radius 10 (|pos|²=%.4f)", r2);
    /* 方向断言：四分之一圈处 y 接近峰值 R，x 接近 0 */
    ASSERT(pose.y > 9.9f, "y should be near R=10 at quarter circle (got %.4f)", pose.y);
    ASSERT(fabs(pose.x) < 0.3f, "x should be near 0 at quarter circle (got %.4f)", pose.x);
    PASS();
}

static void test_slam_circle_pose_full_loop(void) {
    TEST("slam dry-run circle at t≈2πs: returns near start");
    /* 2π≈6.2832s，20Hz 下最近整数样本 poses_published=126 → t=6.3s。
     * sin(6.3)=0.0168 → y=0.168（量化误差，非 bug）。用 on-circle 不变量
     * +「回到起点附近」断言。 */
    Pose2D pose_start, pose_end;
    slam_dry_run_circle_pose(0, 20, 10.0f, &pose_start);
    slam_dry_run_circle_pose(126, 20, 10.0f, &pose_end);
    /* 不变量：绕一圈后仍在圆上 */
    float r2 = pose_end.x * pose_end.x + pose_end.y * pose_end.y;
    ASSERT_NEAR(r2, 100.0f, 0.1f, "should still be on circle after full loop (|pos|²=%.4f)", r2);
    /* 回到起点附近（x≈R, y≈0），容差容纳 20Hz 量化误差（≤0.25m） */
    ASSERT_NEAR(pose_end.x, 10.0f, 0.3f, "x should return near R after full loop");
    ASSERT(fabs(pose_end.y) < 0.3f, "y should return near 0 after full loop (got %.4f)", pose_end.y);
    PASS();
}

static void test_slam_circle_pose_convergence_flag(void) {
    TEST("slam dry-run: cov_xx/yy/hh are constant 0.1/0.1/0.05");
    Pose2D pose;
    for (int i = 0; i < 5; i++) {
        slam_dry_run_circle_pose((uint64_t)i * 20, 20, 10.0f, &pose);
        ASSERT_NEAR(pose.cov_xx, 0.1, 1e-6, "cov_xx should be constant 0.1");
        ASSERT_NEAR(pose.cov_yy, 0.1, 1e-6, "cov_yy should be constant 0.1");
        ASSERT_NEAR(pose.cov_hh, 0.05, 1e-6, "cov_hh should be constant 0.05");
    }
    PASS();
}

static void test_slam_heading_normalize_basic(void) {
    TEST("slam heading normalize wraps to [-π, π]");
    /* 4π → 0 */
    ASSERT_NEAR(normalize_heading_pi(4.0f * (float)M_PI), 0.0f, 1e-5, "4π → 0");
    /* 3π/2 → -π/2 */
    ASSERT_NEAR(normalize_heading_pi(1.5f * (float)M_PI), -0.5f * (float)M_PI, 1e-5, "3π/2 → -π/2");
    /* -3π/2 → π/2 */
    ASSERT_NEAR(normalize_heading_pi(-1.5f * (float)M_PI), 0.5f * (float)M_PI, 1e-5, "-3π/2 → π/2");
    PASS();
}

static void test_slam_heading_normalize_in_range(void) {
    TEST("slam heading normalize: in-range values unchanged");
    float test_vals[] = { 0.0f, 0.5f, -0.5f, (float)M_PI, -(float)M_PI, 1.0f, -1.0f };
    for (size_t i = 0; i < sizeof(test_vals)/sizeof(test_vals[0]); i++) {
        float v = test_vals[i];
        /* M_PI 本身和 -M_PI 本身按定义是合法范围（≤π / ≥-π），不变 */
        ASSERT_NEAR(normalize_heading_pi(v), v, 1e-6, "in-range value should be unchanged");
    }
    PASS();
}

/* ══════════════════════════════════════════════════════════ */
/* Actuator PWM (ControlCmd → PWM mapping) Tests               */
/* ══════════════════════════════════════════════════════════ */

static void test_pwm_throttle_full_forward(void) {
    TEST("pwm: throttle=+1.0 → esc=2000μs (full forward)");
    int esc, steer;
    map_control_cmd_to_pwm(1.0, 0.0, 0.0, 0, 500.0, 500.0, &esc, &steer);
    ASSERT_EQ(esc, 2000, "throttle +1 should map to 2000μs");
    ASSERT_EQ(steer, 1500, "steering 0 should map to 1500μs");
    PASS();
}

static void test_pwm_throttle_full_reverse(void) {
    TEST("pwm: throttle=-1.0 → esc=1000μs (full reverse / brake)");
    int esc, steer;
    map_control_cmd_to_pwm(-1.0, 0.0, 0.0, 0, 500.0, 500.0, &esc, &steer);
    ASSERT_EQ(esc, 1000, "throttle -1 should map to 1000μs");
    PASS();
}

static void test_pwm_brake_full(void) {
    TEST("pwm: brake=1.0 → esc=1000μs (full brake)");
    int esc, steer;
    map_control_cmd_to_pwm(0.0, 1.0, 0.0, 0, 500.0, 500.0, &esc, &steer);
    ASSERT_EQ(esc, 1000, "brake=1 should map to 1000μs (reverse of throttle)");
    PASS();
}

static void test_pwm_brake_overrides_throttle(void) {
    TEST("pwm: brake>0.01 overrides throttle (priority safety)");
    /* 源文件 actuator_pwm_node.c:280 用 `else if (brake > 0.01)` 走刹车路径，
     * 即 throttle 路径被忽略。这是安全设计：刹车时油门信号被丢弃。 */
    int esc, steer;
    map_control_cmd_to_pwm(1.0, 0.5, 0.0, 0, 500.0, 500.0, &esc, &steer);
    /* brake=0.5 → esc = 1500 - 0.5*500 = 1250, 不是 2000 */
    ASSERT_EQ(esc, 1250, "brake should override throttle (esc=1250, not 2000)");
    PASS();
}

static void test_pwm_e_stop_overrides_all(void) {
    TEST("pwm: emergency_stop forces esc=1500μs (neutral)");
    int esc, steer;
    /* e_stop 即使有 throttle / brake 也应该强制中位 */
    map_control_cmd_to_pwm(1.0, 1.0, 0.5, 1, 500.0, 500.0, &esc, &steer);
    ASSERT_EQ(esc, 1500, "e_stop should force esc=1500 (neutral)");
    PASS();
}

static void test_pwm_steering_max_left(void) {
    TEST("pwm: steering=+0.22rad → steer=2000μs (full right)");
    /* 注意：源文件 actuator_pwm_node.c:286-291 的实现是
     *   steer_norm = steering_rad / MAX_STEER_RAD
     *   steer_us = 1500 + steer_norm * steering_scale
     * steering_rad=+0.22 → steer_norm=+1.0 → steer_us=2000
     * 按舵机约定 2000μs 是"全右"。这里只验证映射不验证物理方向。 */
    int esc, steer;
    map_control_cmd_to_pwm(0.0, 0.0, 0.22, 0, 500.0, 500.0, &esc, &steer);
    ASSERT_EQ(steer, 2000, "steering=+0.22rad should map to 2000μs");
    PASS();
}

static void test_pwm_steering_max_right(void) {
    TEST("pwm: steering=-0.22rad → steer=1000μs (full left)");
    int esc, steer;
    map_control_cmd_to_pwm(0.0, 0.0, -0.22, 0, 500.0, 500.0, &esc, &steer);
    ASSERT_EQ(steer, 1000, "steering=-0.22rad should map to 1000μs");
    PASS();
}

static void test_pwm_steering_clamp(void) {
    TEST("pwm: steering beyond max is clamped to ±0.22rad equivalent");
    int esc, steer_over, steer_under;
    map_control_cmd_to_pwm(0.0, 0.0, 0.5, 0, 500.0, 500.0, &esc, &steer_over);
    map_control_cmd_to_pwm(0.0, 0.0, -0.5, 0, 500.0, 500.0, &esc, &steer_under);
    ASSERT_EQ(steer_over, 2000, "steering=+0.5rad should clamp to 2000μs");
    ASSERT_EQ(steer_under, 1000, "steering=-0.5rad should clamp to 1000μs");
    PASS();
}

static void test_pwm_zero_cmd_is_neutral(void) {
    TEST("pwm: zero throttle/brake/steer → both 1500μs (neutral)");
    int esc, steer;
    map_control_cmd_to_pwm(0.0, 0.0, 0.0, 0, 500.0, 500.0, &esc, &steer);
    ASSERT_EQ(esc, 1500, "zero cmd should produce esc=1500");
    ASSERT_EQ(steer, 1500, "zero cmd should produce steer=1500");
    PASS();
}

static void test_pwm_custom_scale(void) {
    TEST("pwm: custom throttle_scale=300 changes range to 1500±300");
    /* 真车场景：throttle_scale 可能不是默认的 500，验证参数确实生效 */
    int esc, steer;
    map_control_cmd_to_pwm(1.0, 0.0, 0.0, 0, 300.0, 300.0, &esc, &steer);
    ASSERT_EQ(esc, 1800, "throttle=1 with scale=300 should map to 1800μs");
    PASS();
}

/* ══════════════════════════════════════════════════════════ */
/* Main                                                        */
/* ══════════════════════════════════════════════════════════ */

int main(void) {
    printf("\n╔══════════════════════════════════════════╗\n");
    printf("║  FlowEngine ADAS Nodes Logic Tests        ║\n");
    printf("║  (imu / slam / actuator_pwm dry-run)     ║\n");
    printf("╚══════════════════════════════════════════╝\n\n");

    printf("═══ IMU Driver (parse_imu_line / synthetic) ═══\n");
    test_imu_parse_basic();
    test_imu_parse_negative_values();
    test_imu_parse_whitespace_and_crlf();
    test_imu_parse_too_few_fields();
    test_imu_parse_empty_line();
    test_imu_parse_non_numeric();
    test_imu_synthetic_static_gravity();
    test_imu_synthetic_custom_gravity();

    printf("\n═══ SLAM dry-run circle + heading normalize ═══\n");
    test_slam_circle_pose_t0();
    test_slam_circle_pose_quarter();
    test_slam_circle_pose_full_loop();
    test_slam_circle_pose_convergence_flag();
    test_slam_heading_normalize_basic();
    test_slam_heading_normalize_in_range();

    printf("\n═══ Actuator PWM (ControlCmd → PWM mapping) ═══\n");
    test_pwm_throttle_full_forward();
    test_pwm_throttle_full_reverse();
    test_pwm_brake_full();
    test_pwm_brake_overrides_throttle();
    test_pwm_e_stop_overrides_all();
    test_pwm_steering_max_left();
    test_pwm_steering_max_right();
    test_pwm_steering_clamp();
    test_pwm_zero_cmd_is_neutral();
    test_pwm_custom_scale();

    printf("\n═══════════════════════════════════\n");
    printf("  Total: %d  ✅ Passed: %d  ❌ Failed: %d\n",
           g_passed + g_failed, g_passed, g_failed);
    printf("═══════════════════════════════════\n\n");

    return g_failed > 0 ? 1 : 0;
}
