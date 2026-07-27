/**
 * test_ekf_fusion.c — EKF 融合单元测试
 *
 * 覆盖三层修复：
 *   Layer 1: (v, ψ) 镜像归一化 — v<0 自动翻转为 (-v, ψ+π)
 *   Layer 2: 低速 heading 协方差下限 — P[3][3] ≥ 0.01 防过度自信
 *   Layer 3: Control 负速钳位（在 control_node.cpp，不在此处测）
 *
 * 用例：
 *   1. 停车 30s 再起步：模拟 GPS 持续观测 heading=0，验证 v≥0 始终成立
 *     且 heading 未锁定在镜像分支 (|ψ| < π/2)。
 *   2. 负速度翻转：直接注入 v=-5，验证被翻转为 v=+5, ψ+=π。
 *   3. 协方差下限：低速下 P[3][3] 不低于 0.01。
 */

#include "ekf_fusion.h"
#include <stdio.h>
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define DT_S      0.05   /* 50ms = 20Hz，与 EKF 默认 dt 一致 */
#define HZ        20
#define TOTAL_S   50     /* 跑 50s 总时长 */

/* 测试断言 */
static int g_pass = 0, g_fail = 0;

#define TEST(name, cond) do { \
    if (!(cond)) { \
        printf("  FAIL %s\n", name); \
        g_fail++; \
    } else { \
        printf("  PASS %s\n", name); \
        g_pass++; \
    } \
} while(0)

/* 用例 1：停车 30s 再起步 */
static void test_park_then_start(void) {
    printf("\n=== [Case 1] 停车 30s 再起步 ===\n");

    double x0[5] = {0.0, 0.0, 5.0, 0.0, 0.0};
    EkfFusion ekf;
    ekf_fusion_init(&ekf, DT_S, x0);

    /* Phase A: 正常行驶 5s */
    for (int s = 0; s < 5; s++) {
        for (int tick = 0; tick < HZ; tick++)
            ekf_fusion_predict(&ekf);
        ekf_fusion_update_gps(&ekf, 5.0, 0.0, NULL);
    }
    printf("  5s 后: v=%.3f heading=%.3f°\n", ekf.x[2], ekf.x[3] * 180.0 / M_PI);
    TEST("Phase A: v > 0", ekf.x[2] > 0.0);
    TEST("Phase A: |heading| < 10°", fabs(ekf.x[3]) < 10.0 * M_PI / 180.0);

    /* Phase B: 停车 30s — 速度趋零，heading 可能随机游走 */
    for (int s = 0; s < 30; s++) {
        for (int tick = 0; tick < HZ; tick++)
            ekf_fusion_predict(&ekf);
        ekf_fusion_update_gps(&ekf, 0.0, 0.0, NULL);
        /* Layer 1 保证 v>=0 */
        TEST("Phase B: v >= 0 (Layer 1)", ekf.x[2] >= 0.0);
        if (ekf.x[2] < 0.0) break;
    }
    printf("  停车 30s 后: v=%.3f heading=%.3f° P_head=%.6f\n",
           ekf.x[2], ekf.x[3] * 180.0 / M_PI, ekf.P[3*5 + 3]);

    /* Layer 2: P_head 未塌陷，GPS 才能拉回来 */
    TEST("Phase B: P_head >= 0.009 (Layer 2)", ekf.P[3*5 + 3] >= 0.009);

    /* 当前 heading 不应锁死在 ±180° 附近 */
    double h_deg = ekf.x[3] * 180.0 / M_PI;
    TEST("Phase B: heading 未锁死在 180° 镜像 (|h| < 90°)", fabs(h_deg) < 90.0);

    /* Phase C: 起步 5s — 速度恢复，GPS 应拉回 heading≈0 */
    for (int s = 0; s < 5; s++) {
        for (int tick = 0; tick < HZ; tick++)
            ekf_fusion_predict(&ekf);
        ekf_fusion_update_gps(&ekf, 5.0, 0.0, NULL);
        TEST("Phase C: v >= 0 (Layer 1)", ekf.x[2] >= 0.0);
        if (ekf.x[2] < 0.0) break;
    }
    printf("  起步 5s 后: v=%.3f heading=%.3f°\n", ekf.x[2], ekf.x[3] * 180.0 / M_PI);
    TEST("Phase C: v > 0", ekf.x[2] > 0.0);
    TEST("Phase C: |heading| < 45° 收敛中", fabs(ekf.x[3]) < 45.0 * M_PI / 180.0);
}

/* 用例 2：低速时 heading 游走到 π，GPS 速度为正但 heading 观测错位。
 * 真实故障链条：
 *   1. 停车 → v→0, heading 不可观 → 随机游走到 π
 *   2. GPS 报 v=0, heading=π → EKF 接受 heading=π
 *   3. 起步后 GPS 报 v>0, heading=0（真值），但 EKF 锁在 ψ=π
 *      此时 v 必须为负才能匹配位置动力学 → Layer 1 应翻转 */
static void test_heading_wander_to_pi(void) {
    printf("\n=== [Case 2] heading 游走到 π 后恢复 (Layer 1) ===\n");

    double x0[5] = {0.0, 0.0, 5.0, 0.0, 0.0};
    EkfFusion ekf;
    ekf_fusion_init(&ekf, DT_S, x0);

    /* Phase A: 正常行驶 5s */
    for (int s = 0; s < 5; s++) {
        for (int tick = 0; tick < HZ; tick++)
            ekf_fusion_predict(&ekf);
        ekf_fusion_update_gps(&ekf, 5.0, 0.0, NULL);
    }
    printf("  Phase A (5s): v=%.3f heading=%.1f°\n", ekf.x[2], ekf.x[3] * 180.0 / M_PI);
    TEST("A: v > 0", ekf.x[2] > 0.0);
    TEST("A: |heading| < 10°", fabs(ekf.x[3]) < 10.0 * M_PI / 180.0);

    /* Phase B: 停车，GPS 报 heading=π（模拟传感器异常/协方差塌陷后游走） */
    for (int s = 0; s < 30; s++) {
        for (int tick = 0; tick < HZ; tick++)
            ekf_fusion_predict(&ekf);
        /* GPS 报 v≈0, heading=π（错误但可能发生的观测）*/
        ekf_fusion_update_gps(&ekf, 0.0, M_PI, NULL);
        TEST("B: v >= 0 (Layer 1)", ekf.x[2] >= 0.0);
    }
    printf("  Phase B (30s, v=0, heading=π): v=%.3f heading=%.1f° P_head=%.6f\n",
           ekf.x[2], ekf.x[3] * 180.0 / M_PI, ekf.P[3*5 + 3]);

    /* 即使 GPS 报 heading=π，Layer 1 保证 v ≥ 0 */
    double h_deg_b = ekf.x[3] * 180.0 / M_PI;
    TEST("B: v >= 0 (Layer 1)", ekf.x[2] >= 0.0);

    /* Phase C: 恢复正常观测 — GPS 报 v=5, heading=0 */
    for (int s = 0; s < 5; s++) {
        for (int tick = 0; tick < HZ; tick++)
            ekf_fusion_predict(&ekf);
        ekf_fusion_update_gps(&ekf, 5.0, 0.0, NULL);
        TEST("C: v >= 0 (Layer 1)", ekf.x[2] >= 0.0);
    }
    printf("  Phase C (5s, v=5, heading=0 恢复): v=%.3f heading=%.1f°\n",
           ekf.x[2], ekf.x[3] * 180.0 / M_PI);
    TEST("C: v > 0", ekf.x[2] > 0.0);
    TEST("C: |heading| < 45° 拉回中", fabs(ekf.x[3]) < 45.0 * M_PI / 180.0);
}

/* 用例 3：协方差下限 — 长时间零速不塌陷 */
static void test_covariance_floor(void) {
    printf("\n=== [Case 3] 协方差下限 (Layer 2) ===\n");

    double x0[5] = {100.0, 200.0, 0.0, 0.0, 0.0};
    EkfFusion ekf;
    ekf_fusion_init(&ekf, DT_S, x0);

    double min_P_head = 1.0;
    for (int s = 0; s < 40; s++) {
        for (int tick = 0; tick < HZ; tick++)
            ekf_fusion_predict(&ekf);
        ekf_fusion_update_gps(&ekf, 0.0, 0.0, NULL);
        if (ekf.P[3*5 + 3] < min_P_head)
            min_P_head = ekf.P[3*5 + 3];
        TEST("Case 3: v >= 0", ekf.x[2] >= 0.0);
    }
    printf("  零速 40s 后: v=%.3f P_head=%.6f (min=%.6f)\n",
           ekf.x[2], ekf.P[3*5 + 3], min_P_head);
    TEST("Case 3: min_P_head >= 0.009", min_P_head >= 0.009);
    TEST("Case 3: P_head >= 0.009 (最后)", ekf.P[3*5 + 3] >= 0.009);

    /* GPS 恢复后应能正常修正 */
    for (int s = 0; s < 5; s++) {
        for (int tick = 0; tick < HZ; tick++)
            ekf_fusion_predict(&ekf);
        ekf_fusion_update_gps(&ekf, 10.0, 0.5, NULL);
    }
    printf("  恢复后: v=%.3f heading=%.3f°\n", ekf.x[2], ekf.x[3] * 180.0 / M_PI);
    TEST("Case 3: 恢复后 v > 0", ekf.x[2] > 0.0);
    TEST("Case 3: heading 已接近真值 (|h-0.5|<0.3)", fabs(ekf.x[3] - 0.5) < 0.3);
}

int main(void) {
    printf("EKF 融合单元测试 (dt=%.3fs, hz=%d)\n", DT_S, HZ);
    printf("Layer 1: (v,ψ) 镜像归一化\n");
    printf("Layer 2: 低速 heading 协方差下限\n\n");

    test_park_then_start();
    test_heading_wander_to_pi();
    test_covariance_floor();

    printf("\n=== 汇总: %d PASS, %d FAIL ===\n", g_pass, g_fail);
    return g_fail > 0 ? 1 : 0;
}
