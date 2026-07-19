/**
 * route.h — 中央有序道路 route（NPC 车道跟随的骨架）
 *
 * 背景：NPC 原本用 step_bicycle(steer=0) 在世界系直线积分，道路一拐弯车就
 * 全飞出路网。本类把 esmini 各 road 按端点连续性 + 航向连续性链成 **一条有序
 * 主 route**，NPC 沿 route 的累计 s 推进；过 road 边界即进入 route 的下一段
 * （中央调度，不依赖 esmini 的过路口 API —— 封装里也没有 RM_PositionMoveForward）。
 *
 * 用法：
 *   Route route;
 *   route.build(roads);                       // 场景加载后构建一次
 *   int rid; double sl; int idx;
 *   route.locate(route_s, rid, sl, idx);      // 累计 s → (road_id, 段内 s, 段号)
 *   roads.frenet_to_world(rid, 0, sl, offset, wp);   // 再反算世界坐标
 */

#ifndef FLOWSIM_ROUTE_H
#define FLOWSIM_ROUTE_H

#include <utility>
#include <vector>

namespace flowsim {

class FlowRoadNetwork;

/** route 的一段（= 一条 esmini road），带累计 s 起点。 */
struct RouteSeg {
    int    road_id{0};
    double length{0.0};
    double s_start{0.0};   /**< 该段在 route 坐标系里的累计 s 起点 */
};

/**
 * 中央有序 route。单线程构建/查询（与 FlowRoadNetwork 一致）。
 */
class Route {
public:
    /**
     * 从路网构建主 route。
     * 算法：取「无前驱」road 作起点（其 start 端点不与任何 road 的 end 重合），
     * 之后每步在未用 road 里挑 start 端点最接近当前 end、且航向最连续者为后继，
     * 长度更长者优先做 tiebreak（避免误选短存根/急弯匝道）。
     * @param tol 端点重合判定容差 (m)
     * @return 至少链出一段返回 true
     */
    bool build(FlowRoadNetwork& roads, double tol = 4.0);

    bool   ok() const { return !segs_.empty(); }
    int    count() const { return static_cast<int>(segs_.size()); }
    double total_length() const { return total_; }
    const RouteSeg& seg(int idx) const { return segs_[idx]; }

    /** road_id → route 段号；不在 route 上返回 -1。 */
    int index_of(int road_id) const;

    /** 任意 road 的长度（含不在 route 上的），查不到返回 0。 */
    double road_length(int road_id) const;

    /** route 累计 s → (road_id, 段内 s_local, 段号)。route_s 夹到 [0,total]。 */
    void locate(double route_s, int& road_id, double& s_local, int& route_idx) const;

    /** (段号, 段内 s) → route 累计 s。参数非法返回 0。 */
    double to_route_s(int route_idx, double s_local) const;

private:
    std::vector<RouteSeg>                segs_;
    double                              total_{0.0};
    std::vector<std::pair<int, double>> all_lengths_;  /**< road_id → length（全部 road） */
};

}  // namespace flowsim

#endif  // FLOWSIM_ROUTE_H
