/**
 * test_traversability.c — 单元测试 traversability_node 的核心算法
 *
 * 通过单 TU 包含节点 .c 以访问其 static 函数（fit_ground_ransac /
 * build_grid / find_corridor / depth_to_points_3d）与全局 g，用 synthetic
 * 点云 / StereoFrame 验证算法正确性。注意：sensor/stereo 数据源是 OAK-D
 * 真实摄像头，仿真无此源，故端到端无法验证，仅测算法单元。
 */
#include "../modules/adas_nodes/traversability_node.c"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

static int g_fail = 0;
#define CHECK(c, msg) do {                                                  \
    if (!(c)) { printf("FAIL: %s (%s:%d)\n", msg, __FILE__, __LINE__); ++g_fail; } \
    else      { printf("ok:   %s\n", msg); }                                \
} while (0)

/* 镜像 traversability_node.c init 的默认参数 */
static void init_defaults(void) {
    memset(&g, 0, sizeof g);
    g.enabled              = 1;
    g.min_range            = 0.5;
    g.max_range            = 6.0;
    g.stride               = 2;
    g.camera_height_m      = 0.30;
    g.camera_tilt_deg      = 0.0;
    g.ground_tol_m         = 0.08;
    g.obstacle_height_m    = 0.10;
    g.cell_size_m          = 0.20;
    g.x_range_m            = 6.0;
    g.y_range_m            = 3.0;
    g.v_fov_deg            = 50.0;
    g.min_corridor_width_m = 0.6;
    g.publish_hz           = 10;
    g.ransac_iters         = 50;
    g.ransac_inlier_m      = 0.05;
    g.grid_decay           = 0.9;
    g.occ_log_step         = 8;
    g.free_log_step        = 4;
    g.occ_log_thr          = 6;
    g.free_log_thr         = -6;
}

int main(void) {
    init_defaults();
    srand(12345);  /* 固定种子，结果可复现 */

    const int GW = 30, GH = 30;
    const double CELL = 0.20, YR = 3.0;

    /* ── T1: RANSAC 拟合水平地面 ── */
    {
        Point3D pts[200]; int n = 0;
        /* 90 个确定性地面点：z = -camera_height（世界 Z=0） */
        for (int i = 0; i < 10; i++)
            for (int j = 0; j < 9; j++) {
                pts[n].x = -3.0f + i * 0.6f;
                pts[n].y = -3.0f + j * 0.6f;
                pts[n].z = -0.30f;
                pts[n].intensity = 1.0f; n++;
            }
        /* 10 个障碍点：z 抬高 0.6（远高于地面） */
        for (int i = 0; i < 10; i++) {
            pts[n].x = 1.0f + i * 0.1f;
            pts[n].y = 0.0f;
            pts[n].z = 0.30f;
            pts[n].intensity = 1.0f; n++;
        }
        Plane plane;
        int rc = fit_ground_ransac(pts, n, &plane, 200, 0.05f);
        CHECK(rc == 0, "T1 ransac fit succeeds on ground+obstacle cloud");
        CHECK(fabsf(plane.c) > 0.7f, "T1 plane normal c near vertical");
        CHECK(fabsf(plane.a) < 0.5f && fabsf(plane.b) < 0.5f,
              "T1 plane a,b small (near-horizontal ground)");
        /* 水平地面 z=-0.3 → 平面 0*X+0*Y+1*Z+0.3=0。法向符号随机，故用残差验证
         * （取一个地面点代入平面方程应≈0，与法向符号无关）。 */
        float resid = plane.a*pts[0].x + plane.b*pts[0].y + plane.c*pts[0].z + plane.d;
        CHECK(fabsf(resid) < 0.1f, "T1 plane passes through ground points");
    }

    /* ── T2: build_grid 用平面距离正确标障碍 ── */
    {
        memset(g.grid_log, 0, sizeof g.grid_log);
        Plane pl; pl.a = 0; pl.b = 0; pl.c = 1; pl.d = 0.30f;  /* z=-0.3 地面 */
        Point3D obs[1];
        obs[0].x = 2.0f; obs[0].y = 0.0f; obs[0].z = 0.30f;  /* 世界Z=0.6>障碍阈值 */
        int fc, oc, uc;
        build_grid(obs, 1, GW, GH, CELL, g.x_range_m, YR, &pl, &fc, &oc, &uc);
        int gx = (int)(2.0 / CELL), gy = (int)((0.0 + YR) / CELL);
        int idx = gx * GH + gy;
        CHECK(g.grid[idx] == TV_CELL_OCCUPIED, "T2 obstacle cell marked OCCUPIED");
    }

    /* ── T3: 时域融合 — 同一障碍多帧 log-odds 累积 ── */
    {
        memset(g.grid_log, 0, sizeof g.grid_log);
        Plane pl; pl.a = 0; pl.b = 0; pl.c = 1; pl.d = 0.30f;
        Point3D obs[1];
        obs[0].x = 2.0f; obs[0].y = 0.0f; obs[0].z = 0.30f;
        int fc, oc, uc;
        build_grid(obs, 1, GW, GH, CELL, g.x_range_m, YR, &pl, &fc, &oc, &uc);
        int gx = (int)(2.0 / CELL), gy = (int)((0.0 + YR) / CELL);
        int idx = gx * GH + gy;
        int log1 = g.grid_log[idx];
        build_grid(obs, 1, GW, GH, CELL, g.x_range_m, YR, &pl, &fc, &oc, &uc);
        int log2 = g.grid_log[idx];
        CHECK(log1 == 8, "T3 first frame obstacle log = occ_log_step");
        CHECK(log2 > log1, "T3 temporal: obstacle log accumulates across frames");
    }

    /* ── T4: find_corridor BFS 连通性 ── */
    {
        /* 全 FREE */
        for (int i = 0; i < GW * GH; i++) g.grid[i] = TV_CELL_FREE;
        double l, r, w; int blk;
        find_corridor(GW, GH, CELL, YR, &l, &r, &w, &blk);
        CHECK(blk == 0, "T4 all-free => not blocked");
        CHECK(fabs(w - GH * CELL) < 1e-6, "T4 all-free corridor spans full width");

        /* 全 OCCUPIED → 无 FREE，回退失败 */
        for (int i = 0; i < GW * GH; i++) g.grid[i] = TV_CELL_OCCUPIED;
        find_corridor(GW, GH, CELL, YR, &l, &r, &w, &blk);
        CHECK(blk == 1, "T4 all-occupied => blocked");

        /* 障碍列贯通 center+3，左侧应仍可通行 */
        for (int i = 0; i < GW * GH; i++) g.grid[i] = TV_CELL_FREE;
        int center = GH / 2;
        for (int gx = 0; gx < GW; gx++) g.grid[gx * GH + (center + 3)] = TV_CELL_OCCUPIED;
        find_corridor(GW, GH, CELL, YR, &l, &r, &w, &blk);
        CHECK(blk == 0, "T4 obstacle column => still passable on left side");
        /* 连通域在障碍左侧：right_y 应小于障碍列右侧 */
        CHECK(r < (double)(center + 3) * CELL - YR + 0.01,
              "T4 corridor lies left of the obstacle column");
        CHECK(w > g.min_corridor_width_m, "T4 left corridor width passes min");
    }

    /* ── T5: depth_to_points_3d 反投影 synthetic StereoFrame ── */
    {
        StereoFrame f; memset(&f, 0, sizeof f);
        for (int j = 0; j < 60; j++)
            for (int i = 0; i < 80; i++) f.depth_data[j * 80 + i] = 2.0f;
        f.depth_count = 4800;
        f.fov_deg = 65.0f;
        uint8_t buf[44832]; size_t len = 0;
        CHECK(StereoFrame_serialize(&f, buf, &len) == 0, "T5 serialize synthetic frame");
        StereoFrame out;
        CHECK(StereoFrame_deserialize(&out, buf, len) == 0, "T5 deserialize synthetic frame");
        Point3D pts[4000];
        int n = depth_to_points_3d(&out, pts, 4000);
        CHECK(n > 0, "T5 depth_to_points_3d produces points");
        /* 存在近中线点，其 X≈depth=2.0（cos 因子≈1） */
        int near_mid = 0;
        for (int k = 0; k < n; k++)
            if (fabsf(pts[k].x - 2.0f) < 0.3f) { near_mid = 1; break; }
        CHECK(near_mid, "T5 reprojected point X near depth (2.0m)");
    }

    if (g_fail) { printf("\n%d CHECK(s) FAILED\n", g_fail); return 1; }
    printf("\nall traversability checks passed\n");
    return 0;
}
