/**
 * road_position.h — 每车持久的 esmini position handle
 *
 * 替代 Route（单链贪心路由）+ FlowRoadNetwork 的单全局 position handle 无状态查询。
 * 每辆车（ego + 每个 NPC）各持有一个 RoadPosition 实例，内部封装独立的
 * esmini position handle，用 RM_PositionMoveForward 沿真实 OpenDRIVE 拓扑推进，
 * 路口处按 junctionSelectorAngle 选支路——从根上消除"单链丢支路"和
 * "无状态查询猜错切线"两个病灶。
 *
 * 生命周期：
 *   - init() 创建 handle 并 RM_SetLanePosition 定位到起点
 *   - 每 tick advance(speed*dt, junction_angle) 沿路推进
 *   - set_lane() 真实切车道（不再用 offset 插值伪换道）
 *   - world() 取当前世界坐标 + 航向（一次 RM_GetPositionData）
 *   - 析构自动 RM_DeletePosition
 *
 * 单线程使用 —— esmini RM_* 操作进程全局路网状态，与 FlowRoadNetwork 一致。
 */

#ifndef FLOWSIM_ROAD_POSITION_H
#define FLOWSIM_ROAD_POSITION_H

#include "road_network.h"
#include "route.h"  // RefPathPoint

#include <cstdint>
#include <vector>

namespace flowsim {

/**
 * 每车持久 esmini position handle 封装。
 *
 * 一个 RoadPosition 实例对应一个 esmini position handle（RM_CreatePosition），
 * 析构时 RM_DeletePosition 释放。handle 在 init() 成功后有效，init 失败或
 * 未调用时 handle_ 为 -1，所有查询返回 false/默认值。
 *
 * junctionSelectorAngle 语义（来自 esmini 文档）：
 *   - 0.0       = 沿 incoming road 方向（通常等于"不转弯"）
 *   - M_PI_2    = 右转
 *   - M_PI      = 直行
 *   - 3*M_PI_2  = 左转
 *   - -1.0      = 随机选支路
 */
class RoadPosition {
public:
    RoadPosition() = default;
    ~RoadPosition();

    // 不可拷贝（持有 esmini handle，需独占）
    RoadPosition(const RoadPosition&) = delete;
    RoadPosition& operator=(const RoadPosition&) = delete;

    // 可移动（转移 handle 所有权）
    RoadPosition(RoadPosition&& o) noexcept;
    RoadPosition& operator=(RoadPosition&& o) noexcept;

    /**
     * 初始化：创建 position handle 并定位到指定 road/lane/s/offset。
     * 必须在 FlowRoadNetwork::load() 成功后调用。
     * @param roads  已加载的路网引用（仅用于校验 loaded 状态，不持有）
     * @param road_id  起始道路 id
     * @param lane_id  起始车道 id（OpenDRIVE：正=左，负=右，0=参考线）
     * @param s        起始纵向距离 (m)
     * @param offset   起始横向偏移 (m)
     * @return true 成功；false 表示 handle 创建失败或 SetLanePosition 失败
     */
    bool init(FlowRoadNetwork& roads, int road_id, int lane_id,
              double s, double offset);

    /**
     * 是否已初始化（handle 有效）。
     */
    bool ok() const { return handle_ >= 0; }

    /**
     * 沿道路推进 dist 米。
     * @param dist              前进距离 (m)，通常 = speed * dt
     * @param junction_angle    路口选支路方向（见类注释），非路口时忽略
     * @return true 成功；false 表示推进失败（如到达路网边界）
     */
    bool advance(double dist, double junction_angle);

    /**
     * 切换到同 road 的另一条车道。
     * @param lane_id  目标车道 id
     * @return true 成功；false 表示 lane_id 不存在
     */
    bool set_lane(int lane_id);

    /**
     * 设置车道内横向偏移（车道微调，不换道）。
     * @param offset  相对车道中心线的横向偏移 (m)
     * @return true 成功
     */
    bool set_offset(double offset);

    /**
     * 获取当前世界坐标 + 航向。
     * @param out  输出 WorldPos（x/y/z/h）
     * @return true 成功；false 表示 handle 无效
     */
    bool world(WorldPos& out) const;

    /**
     * 获取当前 Frenet 坐标。
     * @param out  输出 FrenetPos（road_id/lane_id/s/offset）
     * @return true 成功；false 表示 handle 无效
     */
    bool frenet(FrenetPos& out) const;

    /** 当前道路 id。失败返回 -1。 */
    int road_id() const;

    /** 当前车道 id。失败返回 0。 */
    int lane_id() const;

    /** 当前沿道路纵向距离 s (m)。失败返回 -1。 */
    double s() const;

    /** 当前横向偏移 (m)。失败返回 0。 */
    double offset() const;

    /**
     * 从当前 position 沿路网向前采样 N 个参考点（供 Stanley 横向控制消费）。
     * 用 RM_PositionMoveForward 推进，过路口按 junction_angle 选支路。
     *
     * 注意：此函数会改变 position handle 的状态（推进后不回退），
     * 仅用于临时查询的 position 应使用 copy 后再采样，或采样完重新 init。
     * ego 的 ref_path 采样建议用独立的临时 RoadPosition（copy from ego）。
     *
     * @param roads           路网引用（未使用，保留接口兼容）
     * @param lookahead       前瞻总长 m
     * @param step_m          采样间距 m
     * @param junction_angle  路口选支路方向
     * @param out             输出采样点（清空后追加）
     * @return 实际采样点数
     */
    int sample_ahead(FlowRoadNetwork& roads, double lookahead, double step_m,
                     double junction_angle, std::vector<RefPathPoint>& out);

    /**
     * 创建一个副本（独立 handle，相同位置）。
     * 用于 ego ref_path 采样时不污染主 position。
     * @return 新 RoadPosition（失败返回 ok()==false 的实例）
     */
    RoadPosition clone() const;

private:
    int handle_{-1};  /**< esmini position handle，-1 表示未初始化 */
};

}  // namespace flowsim

#endif  // FLOWSIM_ROAD_POSITION_H
