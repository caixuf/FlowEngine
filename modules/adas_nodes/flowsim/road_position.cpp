/**
 * road_position.cpp — RoadPosition 实现
 *
 * 封装 esmini RM_CreatePosition/RM_PositionMoveForward/RM_SetLanePosition/
 * RM_GetPositionData/RM_DeletePosition，每实例独占一个 position handle。
 *
 * 实现 notes：
 *   - RM_PositionMoveForward 返回值 >= 0 表示成功（具体码见
 *     roadmanager::Position::ReturnCode），< 0 表示失败（如到达路网边界）。
 *   - RM_SetLanePosition 的 align 参数：true 时重置航向到车道行驶方向，
 *     仅 init 时用 true；运行时 set_lane/set_offset 用 false 保持当前航向。
 *   - RM_GetPositionData 一次调用取全部字段（x/y/z/h/roadId/laneId/s/offset）。
 *   - sample_ahead 会推进 position 状态，调用后 position 停在最后一个采样点。
 *     ego ref_path 采样应先 clone() 再 sample_ahead，避免污染主 position。
 */

#include "road_position.h"

#include "esminiRMLib.hpp"

#include <cmath>
#include <cstdio>
#include <cstring>

namespace flowsim {

RoadPosition::~RoadPosition() {
    if (handle_ >= 0) {
        RM_DeletePosition(handle_);
        handle_ = -1;
    }
}

RoadPosition::RoadPosition(RoadPosition&& o) noexcept : handle_(o.handle_) {
    o.handle_ = -1;
}

RoadPosition& RoadPosition::operator=(RoadPosition&& o) noexcept {
    if (this != &o) {
        if (handle_ >= 0) {
            RM_DeletePosition(handle_);
        }
        handle_ = o.handle_;
        o.handle_ = -1;
    }
    return *this;
}

bool RoadPosition::init(FlowRoadNetwork& roads, int road_id, int lane_id,
                        double s, double offset) {
    /* 释放旧 handle（如有） */
    if (handle_ >= 0) {
        RM_DeletePosition(handle_);
        handle_ = -1;
    }

    if (!roads.loaded()) {
        std::fprintf(stderr, "RoadPosition::init: road network not loaded\n");
        return false;
    }

    handle_ = RM_CreatePosition();
    if (handle_ < 0) {
        std::fprintf(stderr, "RoadPosition::init: RM_CreatePosition failed\n");
        return false;
    }

    /* RM_SetLanePosition(handle, roadId, laneId, laneOffset, s, align)
     * align=true 让航向对齐到车道行驶方向（仅 init 时用） */
    int rc = RM_SetLanePosition(handle_, (id_t)road_id, lane_id, offset, s, true);
    if (rc < 0) {
        std::fprintf(stderr, "RoadPosition::init: SetLanePosition failed (rc=%d) "
                     "road=%d lane=%d s=%.1f off=%.1f\n",
                     rc, road_id, lane_id, s, offset);
        RM_DeletePosition(handle_);
        handle_ = -1;
        return false;
    }

    return true;
}

bool RoadPosition::advance(double dist, double junction_angle) {
    if (handle_ < 0) return false;
    if (dist < 0) {
        /* 不支持后退（esmini 限制）；静止时直接返回 true */
        return true;
    }
    if (dist < 1e-6) return true;  // 静止，无需推进

    int rc = RM_PositionMoveForward(handle_, dist, junction_angle);
    if (rc < 0) {
        /* 到达路网边界或路网错误 —— 不立即销毁 handle，让调用方决定 recycle */
        return false;
    }
    return true;
}

bool RoadPosition::set_lane(int lane_id) {
    if (handle_ < 0) return false;

    /* 取当前 s 和 offset，在同一 road 内切车道 */
    RM_PositionData data;
    if (RM_GetPositionData(handle_, &data) < 0) return false;

    /* align=false 保持当前航向，仅切车道 */
    int rc = RM_SetLanePosition(handle_, data.roadId, lane_id,
                                data.laneOffset, data.s, false);
    return rc >= 0;
}

bool RoadPosition::set_offset(double offset) {
    if (handle_ < 0) return false;

    RM_PositionData data;
    if (RM_GetPositionData(handle_, &data) < 0) return false;

    int rc = RM_SetLanePosition(handle_, data.roadId, data.laneId,
                                offset, data.s, false);
    return rc >= 0;
}

bool RoadPosition::world(WorldPos& out) const {
    if (handle_ < 0) return false;

    RM_PositionData data;
    if (RM_GetPositionData(handle_, &data) < 0) return false;

    out.x = data.x;
    out.y = data.y;
    out.z = data.z;
    out.h = data.h;
    return true;
}

bool RoadPosition::frenet(FrenetPos& out) const {
    if (handle_ < 0) return false;

    RM_PositionData data;
    if (RM_GetPositionData(handle_, &data) < 0) return false;

    out.road_id = (int)data.roadId;
    out.lane_id = data.laneId;
    out.s = data.s;
    out.offset = data.laneOffset;
    return true;
}

int RoadPosition::road_id() const {
    if (handle_ < 0) return -1;
    RM_PositionData data;
    if (RM_GetPositionData(handle_, &data) < 0) return -1;
    return (int)data.roadId;
}

int RoadPosition::lane_id() const {
    if (handle_ < 0) return 0;
    RM_PositionData data;
    if (RM_GetPositionData(handle_, &data) < 0) return 0;
    return data.laneId;
}

double RoadPosition::s() const {
    if (handle_ < 0) return -1.0;
    RM_PositionData data;
    if (RM_GetPositionData(handle_, &data) < 0) return -1.0;
    return data.s;
}

double RoadPosition::offset() const {
    if (handle_ < 0) return 0.0;
    RM_PositionData data;
    if (RM_GetPositionData(handle_, &data) < 0) return 0.0;
    return data.laneOffset;
}

int RoadPosition::sample_ahead(FlowRoadNetwork& /*roads*/, double lookahead,
                               double step_m, double junction_angle,
                               std::vector<RefPathPoint>& out) {
    out.clear();
    if (handle_ < 0 || lookahead <= 0 || step_m <= 0) return 0;

    /* 先取当前点作为起点 */
    RM_PositionData prev;
    if (RM_GetPositionData(handle_, &prev) < 0) return 0;

    RefPathPoint p0;
    p0.x = prev.x;
    p0.y = prev.y;
    p0.h = prev.h;
    p0.kappa = 0.0;
    p0.route_s = 0.0;
    out.push_back(p0);

    /* 沿路网推进采样 */
    double accumulated = 0.0;
    while (accumulated + step_m < lookahead) {
        int rc = RM_PositionMoveForward(handle_, step_m, junction_angle);
        if (rc < 0) break;  // 到达路网边界

        accumulated += step_m;
        RM_PositionData cur;
        if (RM_GetPositionData(handle_, &cur) < 0) break;

        RefPathPoint p;
        p.x = cur.x;
        p.y = cur.y;
        p.h = cur.h;
        p.route_s = accumulated;

        /* 曲率：相邻点弦角差 / 弦长（数值微分） */
        double dx = cur.x - prev.x;
        double dy = cur.y - prev.y;
        double chord = std::sqrt(dx * dx + dy * dy);
        if (chord > 1e-6) {
            double dh = cur.h - prev.h;
            /* 归一化到 [-pi, pi] */
            while (dh > M_PI) dh -= 2.0 * M_PI;
            while (dh < -M_PI) dh += 2.0 * M_PI;
            p.kappa = dh / chord;
        } else {
            p.kappa = 0.0;
        }

        out.push_back(p);
        prev = cur;
    }

    return static_cast<int>(out.size());
}

RoadPosition RoadPosition::clone() const {
    RoadPosition copy;
    if (handle_ < 0) return copy;  // 返回无效实例

    /* 用 RM_CopyPosition 复制 handle（esmini 原生支持） */
    int new_handle = RM_CopyPosition(handle_);
    if (new_handle >= 0) {
        copy.handle_ = new_handle;
    }
    return copy;
}

}  // namespace flowsim
