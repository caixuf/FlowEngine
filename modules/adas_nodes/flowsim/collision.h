/**
 * collision.h — 碰撞检测（AABB broad-phase + OBB SAT narrow-phase）
 *
 * 设计文档 §3 Step 3：每个 tick 检测所有车辆实体对碰撞。
 *
 * 算法：
 *   1. Broad-phase: AABB（轴对齐包围盒）快速排除明显分离的对
 *   2. Narrow-phase: OBB（有向包围盒）SAT（分离轴定理）精确判定
 *
 * OBB SAT 适用于有航向的车辆盒子（length × width），比纯 AABB 准确，
 * 尤其是变道/转弯时车辆航向 ≠ 道路方向的情况。SAT 2D 矩形只需测 4 个轴
 * （每个盒子的局部 x/y 方向）。
 *
 * 行人不参与碰撞检测（被 perception 单独处理）；事件触发器（红绿灯/ETC）
 * 也不参与。
 */

#ifndef FLOWSIM_COLLISION_H
#define FLOWSIM_COLLISION_H

#include "entity.h"
#include <vector>
#include <utility>

namespace flowsim {

/** 一对发生碰撞的实体索引 */
struct CollisionPair {
    EntityId a;
    EntityId b;
};

/**
 * 检测池中所有车辆实体的碰撞。
 * @param pool  实体池
 * @param pairs 输出碰撞对列表（清空后填充）
 * @return 碰撞对数量
 *
 * 只检测 is_vehicle() 的实体对。每对只报一次（a < b）。
 */
int detect_collisions(const EntityPool& pool, std::vector<CollisionPair>& pairs);

/**
 * 应用碰撞响应：让每对碰撞车辆的双方速度归零（模拟撞击后停下）。
 * @param pool   实体池
 * @param pairs  detect_collisions 的输出
 */
void apply_collision_response(EntityPool& pool, const std::vector<CollisionPair>& pairs);

/**
 * OBB-OBB 相交判定（SAT）。
 * @param cx1,cy1 盒1中心，len1/wid1 尺寸，h1 航向
 * @param cx2,cy2 盒2中心，len2/wid2 尺寸，h2 航向
 * @return true 相交
 */
bool obb_intersect(double cx1, double cy1, double len1, double wid1, double h1,
                   double cx2, double cy2, double len2, double wid2, double h2);

}  // namespace flowsim

#endif  // FLOWSIM_COLLISION_H
