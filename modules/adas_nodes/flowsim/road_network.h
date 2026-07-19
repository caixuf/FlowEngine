/**
 * road_network.h — esmini RoadManager C API 封装
 *
 * FlowSim v2 道路网络抽象。封装 esminiRMLib 的 C API（RM_Init/RM_SetLanePosition/
 * RM_GetPositionData 等），提供 Frenet↔World 坐标转换、车道查询、限速查询。
 *
 * 设计要点：
 *   - esminiRMLib 是 **C API**（extern "C"），ABI 稳定，不是文档里写的
 *     roadmanager::OpenDrive C++ 类。C API 隔离了 esmini 内部 C++ 实现的 ABI 变化。
 *   - esmini 的 RoadManager 是 **进程全局单例**：RM_Init 会替换上一次加载的路网。
 *     因此 FlowRoadNetwork 实例不要并发使用；仿真主循环单线程持有即可。
 *   - Position handle 是 per-instance 的，构造时创建、析构时销毁（RAII）。
 *   - 坐标系：esmini 用 OpenDRIVE 约定 —— lane id 0 = 参考线，正 id = 左侧，
 *     负 id = 右侧。我们的场景 JSON 里 y<0 = 左车道、y>0 = 右车道，由
 *     json_to_xodr.py 在生成 xodr 时统一映射，本层只认 OpenDRIVE 语义。
 *
 * 用法：
 *   flowsim::FlowRoadNetwork net;
 *   if (!net.load("/tmp/scene.xodr")) { ... }
 *   flowsim::WorldPos w;
 *   net.frenet_to_world(0, -1, 50.0, 0.0, w);  // road 0, lane -1, s=50, offset=0
 */

#ifndef FLOWSIM_ROAD_NETWORK_H
#define FLOWSIM_ROAD_NETWORK_H

#include <cstdint>
#include <string>

namespace flowsim {

/** 世界坐标（OpenDRIVE 全局系，x 纵向 / y 横向 / z 高度 / h 航向角 rad）。
 *  z 是道路 elevation（高架高度），由 esmini 解析 <elevationProfile> 后给出。
 *  平地场景 z 恒为 0；匝道/高架段 z 随 s 渐变。 */
struct WorldPos {
    double x{0.0};
    double y{0.0};
    double z{0.0};
    double h{0.0};
};

/** Frenet 坐标（沿道路的纵向 s + 横向 laneOffset + 车道 id） */
struct FrenetPos {
    int    road_id{0};
    int    lane_id{0};
    double s{0.0};
    double offset{0.0};
};

/** 道路元信息 */
struct RoadInfo {
    uint32_t id{0};          /**< 数值 id（RM 内部用） */
    std::string str_id;      /**< 字符串 id（xodr 里写的 id 属性） */
    double length{0.0};      /**< 道路总长 (m) */
    int    drivable_lanes{0};/**< 可行驶车道数（在 s=0 处统计） */
};

/**
 * esmini RoadManager 封装。
 *
 * 单线程使用 —— esmini 的 RM_* 函数操作进程全局路网状态，不要在多个线程
 * 同时调用本类方法。仿真主循环（flowcoro 宿主线程）独占即可。
 */
class FlowRoadNetwork {
public:
    FlowRoadNetwork();
    ~FlowRoadNetwork();

    FlowRoadNetwork(const FlowRoadNetwork&) = delete;
    FlowRoadNetwork& operator=(const FlowRoadNetwork&) = delete;
    FlowRoadNetwork(FlowRoadNetwork&&) noexcept;
    FlowRoadNetwork& operator=(FlowRoadNetwork&&) noexcept;

    /** 从 xodr 文件加载路网。返回 false 表示加载失败（RM_Init 返回 -1）。 */
    bool load(const std::string& xodr_path);

    /** 从 OpenDRIVE XML 字符串加载路网。 */
    bool load_string(const std::string& xml);

    /** 释放路网（析构会自动调用，也可显式释放）。 */
    void close();

    /** 是否已加载路网。 */
    bool loaded() const { return loaded_; }

    /** 道路数量。 */
    int road_count() const;

    /**
     * 按索引查询道路信息（index ∈ [0, road_count())）。
     * 返回 false 表示索引越界。
     */
    bool road_info(int index, RoadInfo& out) const;

    /**
     * Frenet → 世界坐标。
     * @param road_id  道路 id
     * @param lane_id  车道 id（OpenDRIVE 约定：0=参考线，正=左，负=右）
     * @param s        沿道路纵向距离 (m)
     * @param offset   相对车道中心线的横向偏移 (m)
     * @param out      输出世界坐标
     * @return true 成功；false 表示 road_id/lane_id 不存在
     */
    bool frenet_to_world(int road_id, int lane_id, double s, double offset,
                         WorldPos& out);

    /**
     * 世界坐标 → Frenet。
     * @param x,y  世界坐标
     * @param out  输出 Frenet 坐标（含 road_id/lane_id/s/offset）
     * @return true 成功；false 表示坐标不在任何道路上
     */
    bool world_to_frenet(double x, double y, FrenetPos& out);

    /**
     * 查询指定道路/车道在 s 处的限速 (m/s)。
     * 失败返回 default_value。
     */
    double speed_limit(int road_id, int lane_id, double s,
                       double default_value = 13.89);

    /**
     * 查询指定道路/车道在 s 处的车道宽度 (m)。
     * 失败返回 0。
     */
    double lane_width(int road_id, int lane_id, double s);

private:
    bool loaded_{false};
    int  pos_handle_{-1};  /**< RM position handle，构造时创建 */
};

}  // namespace flowsim

#endif  // FLOWSIM_ROAD_NETWORK_H
