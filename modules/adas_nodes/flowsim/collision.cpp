/**
 * collision.cpp — 碰撞检测实现
 *
 * 两阶段：
 *   1. AABB broad-phase：用 |Δx| < (len1+len2)/2 && |Δy| < (wid1+wid2)/2 快速排除
 *   2. OBB SAT narrow-phase：对 AABB 通过的对，做 4 轴分离投影判定
 *
 * SAT 2D 矩形：
 *   - 每个盒子的局部 x 轴方向 (cos h, sin h) 和 y 轴方向 (-sin h, cos h)
 *   - 投影两盒子 4 角点到轴上，检查区间是否重叠
 *   - 任一轴分离 → 不相交
 */

#include "collision.h"

#include <cmath>
#include <algorithm>

namespace flowsim {

/* ── OBB 4 角点（中心 + 尺寸 + 航向）── */
static void obb_corners(double cx, double cy, double len, double wid, double h,
                        double out[4][2]) {
    double hl = len * 0.5, hw = wid * 0.5;
    double ch = std::cos(h), sh = std::sin(h);
    // 局部坐标 4 角点：(hl,hw)(-hl,hw)(-hl,-hw)(hl,-hw)
    double local[4][2] = {{hl, hw}, {-hl, hw}, {-hl, -hw}, {hl, -hw}};
    for (int i = 0; i < 4; ++i) {
        out[i][0] = cx + local[i][0] * ch - local[i][1] * sh;
        out[i][1] = cy + local[i][0] * sh + local[i][1] * ch;
    }
}

/* ── 投影点到轴，返回 [min,max] ── */
static void project(double corners[4][2], double ax, double ay,
                    double& out_min, double& out_max) {
    out_min = out_max = corners[0][0] * ax + corners[0][1] * ay;
    for (int i = 1; i < 4; ++i) {
        double p = corners[i][0] * ax + corners[i][1] * ay;
        if (p < out_min) out_min = p;
        if (p > out_max) out_max = p;
    }
}

bool obb_intersect(double cx1, double cy1, double len1, double wid1, double h1,
                   double cx2, double cy2, double len2, double wid2, double h2) {
    double c1[4][2], c2[4][2];
    obb_corners(cx1, cy1, len1, wid1, h1, c1);
    obb_corners(cx2, cy2, len2, wid2, h2, c2);

    // 4 个分离轴：盒1 的 x/y 方向 + 盒2 的 x/y 方向
    double axes[4][2] = {
        {std::cos(h1),  std::sin(h1)},  // 盒1 局部 x
        {-std::sin(h1), std::cos(h1)},  // 盒1 局部 y
        {std::cos(h2),  std::sin(h2)},  // 盒2 局部 x
        {-std::sin(h2), std::cos(h2)},  // 盒2 局部 y
    };

    for (int i = 0; i < 4; ++i) {
        double min1, max1, min2, max2;
        project(c1, axes[i][0], axes[i][1], min1, max1);
        project(c2, axes[i][0], axes[i][1], min2, max2);
        // 任一轴上区间不重叠 → 分离 → 不相交
        if (max1 < min2 || max2 < min1) return false;
    }
    return true;  // 4 轴都重叠 → 相交
}

int detect_collisions(const EntityPool& pool, std::vector<CollisionPair>& pairs) {
    pairs.clear();
    int n = pool.size();
    for (int i = 0; i < n; ++i) {
        const Entity& a = pool[i];
        if (!a.active || !a.is_vehicle()) continue;
        for (int j = i + 1; j < n; ++j) {
            const Entity& b = pool[j];
            if (!b.active || !b.is_vehicle()) continue;

            // Broad-phase: AABB 快速排除
            double dx = std::fabs(a.x - b.x);
            double dy = std::fabs(a.y - b.y);
            double aabb_x = (a.length + b.length) * 0.5;
            double aabb_y = (a.width + b.width) * 0.5;
            if (dx > aabb_x || dy > aabb_y) continue;

            // Narrow-phase: OBB SAT
            if (obb_intersect(a.x, a.y, a.length, a.width, a.heading,
                              b.x, b.y, b.length, b.width, b.heading)) {
                pairs.push_back({i, j});
            }
        }
    }
    return (int)pairs.size();
}

void apply_collision_response(EntityPool& pool, const std::vector<CollisionPair>& pairs) {
    for (const auto& p : pairs) {
        Entity& a = pool[p.a];
        Entity& b = pool[p.b];
        // 双方速度归零（模拟非弹性碰撞后停下）
        a.speed = 0; a.vx = 0; a.vy = 0;
        b.speed = 0; b.vx = 0; b.vy = 0;
        // 刹车踩下，防止下一 tick 又加速
        a.brake = 1.0; a.throttle = 0.0;
        b.brake = 1.0; b.throttle = 0.0;
        // 碰撞冷却 2.0s：之前直接把双方速度永久置 0，任何一次接触
        // 都会让两辆车冻结在原地变路障，长场景越积越多把整条路堵死。
        // 现在用 crash_cooldown 计时器，>0 期间 step_npc_vehicle 跳过 AI
        // （保持 speed=0），到 0 后释放 NPC 重新走 AI（ego 也一样，
        // 但 ego 通常由 control/cmd 驱动，下一帧的油门会立即覆盖 speed）。
        if (a.is_vehicle()) a.crash_cooldown = 2.0;
        if (b.is_vehicle()) b.crash_cooldown = 2.0;
    }
}

}  // namespace flowsim
