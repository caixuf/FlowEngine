/**
 * road_network.cpp — esmini RoadManager C API 封装实现
 *
 * 实现 notes：
 *   - RM_Init/RM_InitWithString 操作进程全局状态，故 loaded_ 是进程级的。
 *     本类析构时调 RM_Close() 释放，避免泄漏。
 *   - RM_CreatePosition 返回一个 handle，本实例独占，所有查询都通过它做。
 *   - frenet_to_world：RM_SetLanePosition(handle, roadId, laneId, laneOffset, s, align)
 *     注意参数顺序：laneOffset 在 s 前。RM_GetPositionData 取 x/y/h。
 *   - world_to_frenet：RM_SetWorldXYHPosition(handle, x, y, nan) 让 RM 反算 road
 *     坐标；RM_GetPositionData 取 roadId/laneId/s/laneOffset。
 *   - speed_limit：需要把 position 移到目标位置，再调 RM_GetSpeedLimit(handle)。
 *   - RM_SetLanePosition 返回值 >= 0 表示成功（具体码见 roadmanager::Position::ReturnCode）。
 */

#include "road_network.h"

#include "esminiRMLib.hpp"

#include <cmath>
#include <cstdio>
#include <cstring>

namespace flowsim {

FlowRoadNetwork::FlowRoadNetwork() {
    // Position handle 在 load() 后创建 —— RM_CreatePosition 要求路网已加载，
    // 否则返回 -1。构造时只初始化成员。
}

FlowRoadNetwork::~FlowRoadNetwork() {
    close();
}

FlowRoadNetwork::FlowRoadNetwork(FlowRoadNetwork&& o) noexcept
    : loaded_(o.loaded_), pos_handle_(o.pos_handle_) {
    o.loaded_ = false;
    o.pos_handle_ = -1;
}

FlowRoadNetwork& FlowRoadNetwork::operator=(FlowRoadNetwork&& o) noexcept {
    if (this != &o) {
        close();
        loaded_ = o.loaded_;
        pos_handle_ = o.pos_handle_;
        o.loaded_ = false;
        o.pos_handle_ = -1;
    }
    return *this;
}

bool FlowRoadNetwork::load(const std::string& xodr_path) {
    close();
    if (RM_Init(xodr_path.c_str()) != 0) {
        std::fprintf(stderr, "FlowRoadNetwork::load failed: %s\n", xodr_path.c_str());
        return false;
    }
    loaded_ = true;
    pos_handle_ = RM_CreatePosition();
    if (pos_handle_ < 0) {
        std::fprintf(stderr, "FlowRoadNetwork::load: RM_CreatePosition failed\n");
        RM_Close();
        loaded_ = false;
        return false;
    }
    return true;
}

bool FlowRoadNetwork::load_string(const std::string& xml) {
    close();
    if (RM_InitWithString(xml.c_str()) != 0) {
        std::fprintf(stderr, "FlowRoadNetwork::load_string failed (xml %zu bytes)\n", xml.size());
        return false;
    }
    loaded_ = true;
    pos_handle_ = RM_CreatePosition();
    if (pos_handle_ < 0) {
        std::fprintf(stderr, "FlowRoadNetwork::load_string: RM_CreatePosition failed\n");
        RM_Close();
        loaded_ = false;
        return false;
    }
    return true;
}

void FlowRoadNetwork::close() {
    if (pos_handle_ >= 0) {
        RM_DeletePosition(pos_handle_);
        pos_handle_ = -1;
    }
    if (loaded_) {
        RM_Close();
        loaded_ = false;
    }
}

int FlowRoadNetwork::road_count() const {
    if (!loaded_) return 0;
    return RM_GetNumberOfRoads();
}

bool FlowRoadNetwork::road_info(int index, RoadInfo& out) const {
    if (!loaded_) return false;
    if (index < 0 || index >= RM_GetNumberOfRoads()) return false;
    id_t rid = RM_GetIdOfRoadFromIndex((unsigned)index);
    out.id = (uint32_t)rid;
    out.str_id = RM_GetRoadIdString(rid) ? RM_GetRoadIdString(rid) : "";
    out.length = RM_GetRoadLength(rid);
    out.drivable_lanes = RM_GetRoadNumberOfDrivableLanes(rid, 0.0);
    return true;
}

bool FlowRoadNetwork::frenet_to_world(int road_id, int lane_id, double s, double offset,
                                       WorldPos& out) {
    if (!loaded_ || pos_handle_ < 0) return false;
    // RM_SetLanePosition(handle, roadId, laneId, laneOffset, s, align)
    int rc = RM_SetLanePosition(pos_handle_, (id_t)road_id, lane_id, offset, s, false);
    if (rc < 0) return false;
    RM_PositionData pd;
    if (RM_GetPositionData(pos_handle_, &pd) < 0) return false;
    out.x = pd.x;
    out.y = pd.y;
    out.z = pd.z;  // 高架 elevation（OpenDRIVE <elevationProfile>，平地场景恒为 0）
    out.h = pd.h;
    return true;
}

bool FlowRoadNetwork::world_to_frenet(double x, double y, FrenetPos& out) {
    if (!loaded_ || pos_handle_ < 0) return false;
    // RM_SetWorldXYHPosition：z/pitch 由路网反算，h 传 NaN 表示由路网决定
    int rc = RM_SetWorldXYHPosition(pos_handle_, x, y, std::nanf(""));
    if (rc < 0) return false;
    RM_PositionData pd;
    if (RM_GetPositionData(pos_handle_, &pd) < 0) return false;
    out.road_id = (int)pd.roadId;
    out.lane_id = pd.laneId;
    out.s = pd.s;
    out.offset = pd.laneOffset;
    return out.road_id >= 0;
}

double FlowRoadNetwork::speed_limit(int road_id, int lane_id, double s,
                                     double default_value) {
    if (!loaded_ || pos_handle_ < 0) return default_value;
    // 把 position 移到目标位置，再查 RM_GetSpeedLimit（它基于当前 position）
    if (RM_SetLanePosition(pos_handle_, (id_t)road_id, lane_id, 0.0, s, false) < 0) {
        return default_value;
    }
    double v = RM_GetSpeedLimit(pos_handle_);
    // RM_GetSpeedLimit 返回 0 表示无数据，回退默认值
    return (v > 0.0) ? v : default_value;
}

double FlowRoadNetwork::lane_width(int road_id, int lane_id, double s) {
    if (!loaded_) return 0.0;
    double w = 0.0;
    if (RM_GetLaneWidthByRoadId((id_t)road_id, lane_id, s, &w) < 0) return 0.0;
    return w;
}

int FlowRoadNetwork::drivable_lane_count(int road_id, double s) {
    /* RM_GetRoadNumberOfDrivableLanes 在任意 s 处统计 drivable 车道数（双向合计）。
     * 与 road_info() 不同：road_info 在 s=0 查，drivable_lane_count 可在任意 s 查
     * （因为 OpenDRIVE 道路车道数可随 s 变化，例如从 3 车道收窄到 2 车道）。
     * 失败返回 0，调用方需用 fallback 默认值。 */
    if (!loaded_) return 0;
    int n = RM_GetRoadNumberOfDrivableLanes((id_t)road_id, s);
    return (n > 0) ? n : 0;
}

}  // namespace flowsim
