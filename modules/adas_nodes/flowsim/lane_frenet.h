/**
 * @file lane_frenet.h
 * @brief Frenet 车道中心横向偏移公式（共享 helper）
 *
 * 收敛"车道中心相对道路参考线的横向偏移"公式：
 *   lane_center_t = sign(lane_id) · (|lane_id| − 0.5) · lane_width
 *
 * OpenDRIVE 车道 id 约定：0 = 参考线，正 id = 左侧车道，负 id = 右侧车道。
 * 车道中心相对参考线的横向偏移 = sign(lane_id) · (|lane_id| − 0.5) · lane_width
 * （|lane_id|−0.5 让中心落在车道宽度的一半处，而非车道边界）。
 *
 * 两套偏移量语义（易混，务必区分）：
 *   - ref_offset    ：相对道路参考线（lane_id=0）的横向偏移，Entity::offset 用此语义
 *   - lane_internal ：相对车道中心的横向偏移，esmini RM_SetLanePosition 的 offset 参数用此语义
 *
 * 本 header 提供三个 inline 函数在两种语义间转换，替代各处手写公式。
 * 车道宽度优先从 road network 查询（roads.lane_width），查询失败回退
 * LANE_WIDTH_FALLBACK=3.5m（与历史默认值一致，保证向后兼容）。
 *
 * @note 与 include/road_geometry.h 是两套并存的几何实现：
 *       - road_geometry.h 是 **Cartesian**（世界系 x/y），用 smoothstep 参数化弯道中心线，
 *         供 control_node/planning_node 等不依赖 esmini 的节点使用；
 *       - 本文件是 **Frenet**（路网系 road_id/lane_id/s/offset），依赖 esmini RoadManager，
 *         供 flowsim 内部 ego/NPC 车道定位使用。
 *       两套实现不要混用（同一帧不要把 road_center_y 的结果当 offset 喂给 lane_frenet）。
 */

#ifndef FLOWSIM_LANE_FRENET_H
#define FLOWSIM_LANE_FRENET_H

#include "road_network.h"  // FlowRoadNetwork

#include <cmath>

namespace flowsim {

/** 车道宽度查询失败时的回退值（m），与历史默认值一致。 */
constexpr double LANE_WIDTH_FALLBACK = 3.5;

/**
 * 车道中心相对道路参考线的横向偏移。
 *
 *   lane_center_t = sign(lane_id) · (|lane_id| − 0.5) · lane_width
 *
 * lane_id=0（参考线）时返回 0。车道宽度从 road network 查询，失败回退
 * LANE_WIDTH_FALLBACK。
 *
 * @param roads    esmini 路网封装
 * @param road_id  道路 id
 * @param lane_id  车道 id（OpenDRIVE：0=参考线，正=左，负=右）
 * @param s        沿道路纵向距离 (m)
 * @return 车道中心相对参考线的横向偏移 (m)
 */
inline double lane_center_t(const FlowRoadNetwork& roads, int road_id,
                            int lane_id, double s) {
    if (lane_id == 0) return 0.0;
    double lw = roads.lane_width(road_id, lane_id, s);
    if (lw <= 0.0) lw = LANE_WIDTH_FALLBACK;
    return (lane_id > 0 ? 1.0 : -1.0) * (std::abs(lane_id) - 0.5) * lw;
}

/**
 * 相对参考线偏移 → 相对车道中心偏移（subtractive）。
 *
 *   lane_internal = ref_offset − lane_center_t
 *
 * 用于把 Entity::offset（ref 语义）转成 esmini RM_SetLanePosition 的 offset
 * 参数（lane-internal 语义）。lane_id=0 时二者相等，直接返回 ref_offset。
 *
 * @param roads      esmini 路网封装
 * @param road_id    道路 id
 * @param lane_id    车道 id
 * @param s          沿道路纵向距离 (m)
 * @param ref_offset 相对参考线的横向偏移 (m)
 * @return 相对车道中心的横向偏移 (m)
 */
inline double lane_internal_from_offset(const FlowRoadNetwork& roads, int road_id,
                                        int lane_id, double s, double ref_offset) {
    if (lane_id == 0) return ref_offset;
    return ref_offset - lane_center_t(roads, road_id, lane_id, s);
}

/**
 * 相对车道中心偏移 → 相对参考线偏移（additive）。
 *
 *   ref_offset = lane_center_t + lane_internal
 *
 * 用于把 esmini world_to_frenet 返回的 fp.offset（lane-internal 语义）转成
 * Entity::offset（ref 语义）。lane_id=0 时二者相等，直接返回 lane_internal。
 *
 * @param roads         esmini 路网封装
 * @param road_id       道路 id
 * @param lane_id       车道 id
 * @param s             沿道路纵向距离 (m)
 * @param lane_internal 相对车道中心的横向偏移 (m)
 * @return 相对参考线的横向偏移 (m)
 */
inline double offset_from_lane_internal(const FlowRoadNetwork& roads, int road_id,
                                        int lane_id, double s, double lane_internal) {
    if (lane_id == 0) return lane_internal;
    return lane_center_t(roads, road_id, lane_id, s) + lane_internal;
}

}  // namespace flowsim

#endif  // FLOWSIM_LANE_FRENET_H
