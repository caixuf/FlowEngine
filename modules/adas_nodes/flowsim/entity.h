/**
 * entity.h — FlowSim v2 Entity System
 *
 * 固定池 64 实体，AOS（Array of Structs）布局。池小（64），AOS 简单且
 * 缓存友好（tick 时遍历全部实体）。设计文档提到 SOA 是远期优化方向，
 * 当前 AOS 足够。
 *
 * 实体类型：
 *   - Ego: 自车，由外部 control/cmd 驱动
 *   - Car/SUV/Truck: NPC 车辆，由 IDM + 状态机驱动
 *   - Pedestrian: 行人，横穿或路边行走
 *   - TrafficLight/ETCGate/StopLine: 场景事件触发器（无动力学）
 *
 * EntityId 是 pool 索引（0..63），-1 表示无效。Ego 固定占用 index 0。
 */

#ifndef FLOWSIM_ENTITY_H
#define FLOWSIM_ENTITY_H

#include <cstdint>
#include <cstring>

#include "road_position.h"

namespace flowsim {

using EntityId = int;
constexpr EntityId INVALID_ENTITY = -1;
constexpr int MAX_ENTITIES = 64;

enum class EntityType : uint8_t {
    None,
    Ego,
    Car,
    SUV,
    Truck,
    Pedestrian,
    TrafficLight,
    ETCGate,
    StopLine,
};

enum class AIState : uint8_t {
    Cruise,      // 巡航（按 target_vx 匀速）
    Follow,      // 跟车（IDM 调速）
    Stop,        // 停止（target_vx=0 且已停）
    StopForTL,   // 红灯停车
    ETCApproach, // ETC 减速通过
    BranchSel,   // 选路（路口分支选择）
    Merge,       // 汇入主路
    Yield,       // 让行
    CutIn,       // 加塞：横向 PID 跨实线变道（收费站排队场景，target_offset 给目标通道）
};

/**
 * 单个仿真实体。包含 Transform + Vehicle 参数 + AI 状态。
 * 行人/事件触发器只填 Transform 部分，Vehicle/AI 字段忽略。
 */
struct Entity {
    bool       active{false};
    EntityType type{EntityType::None};
    int        id{0};              /**< 场景里的 actor id（来自 JSON） */

    /* ── Transform ── */
    double x{0}, y{0}, heading{0}; /**< 世界坐标 + 航向 (rad) */
    double vx{0}, vy{0};           /**< 世界系速度 (m/s) */
    double speed{0};               /**< 标量速度 = √(vx²+vy²)，车辆用 */
    double target_vx{0};           /**< AI 目标纵向速度 */
    double steer{0};               /**< 当前方向盘转角 (rad) */
    double throttle{0}, brake{0};  /**< 油门/刹车 [0,1] */

    /* ── Vehicle 参数（Car/SUV/Truck）── */
    double length{4.6}, width{2.0};
    double wheelbase{2.7};
    double mass{1500.0};           /**< kg */
    double drag_coeff{0.4};        /**< 空气阻力系数 */
    double max_accel{2.0};         /**< m/s² */
    double max_brake{4.0};         /**< m/s² */

    /* ── AI 状态（NPC 车辆）── */
    AIState    ai_state{AIState::Cruise};
    EntityId   lead_id{INVALID_ENTITY};  /**< 跟车目标 */
    double     follow_gap{1e9};          /**< 当前前车间距 (m) */
    double     crash_cooldown{0.0};      /**< 碰撞冷却计时器(s)：>0 期间速度归零不参与 AI；=0 后释放 */

    /* ── 道路坐标（Frenet）── */
    int    road_id{0};
    int    lane_id{0};
    double s{0};                   /**< 沿当前 road 的纵向距离 */
    double offset{0};              /**< 相对参考线横向偏移（= 车道横向位置） */

    /* ── 中央 route 跟随（NPC 车道行驶，见 npc_ai.cpp / route.h）── */
    double route_s{0};             /**< 沿中央 route 的累计纵向距离 */
    int    route_dir{0};           /**< route 行驶方向：+1 顺行，-1 对向，0=未上route(走旧逻辑) */
    int    route_fail_count{0};    /**< frenet_to_world 连续失败计数（≥5 强制 recycle 防飞出） */

    /* ── NPC 避障换道（让 NPC 不再"堵成一坨"）── */
    double lane_change_timer{0.0};  /**< 换道冷却计时器 (s)：>0 期间不评估换道，避免频繁抖动 */
    double target_offset{0.0};      /**< E2: 换道目标横向偏移，offset 每帧向此值平滑插值 */

    /* ── CutIn 加塞状态机（横向 PID 跨实线变道）── */
    double cutin_pid_integral{0.0};  /**< CutIn 横向 PID 积分项（target_offset - offset 的累计误差） */
    double cutin_pid_prev{0.0};      /**< CutIn 横向 PID 上次误差（用于微分项） */
    bool   cutin_active{false};      /**< CutIn 是否激活（ai_state==CutIn 时为 true，结束变道后置 false） */

    /* ── 行人专用 ── */
    double ped_wait_timer{0};      /**< 行人等待计时器 (s) */
    int    ped_parked{0};          /**< 行人是否停在路边 */

    /* ── 场景事件触发器专用 ── */
    int    phase_state{0};         /**< 红绿灯当前相位：0=绿 1=黄 2=红 */
    double phase_timer{0};         /**< 当前相位剩余时间 (s) */

    /* ── RoadPosition（Task: 地图路由重构）──
     * 每车持久的 esmini position handle，替代 route_s/route_dir 的单链路由。
     * ego + 每个 NPC 各自持有独立 handle，用 RM_PositionMoveForward 沿真实
     * OpenDRIVE 拓扑推进，路口按 junction_angle 选支路。
     * 非 vehicle 类型（行人/红绿灯/ETC/停止线）不初始化，handle_ 保持 -1。 */
    RoadPosition road_pos;

    /* 因 RoadPosition 不可拷贝，Entity 显式声明移动语义、删除拷贝 */
    Entity() = default;
    Entity(const Entity&) = delete;
    Entity& operator=(const Entity&) = delete;
    Entity(Entity&&) noexcept = default;
    Entity& operator=(Entity&&) noexcept = default;

    bool is_vehicle() const {
        return type == EntityType::Car || type == EntityType::SUV ||
               type == EntityType::Truck || type == EntityType::Ego;
    }
    bool is_npc_vehicle() const {
        return type == EntityType::Car || type == EntityType::SUV ||
               type == EntityType::Truck;
    }
};

/**
 * 固定容量实体池。alloc() 找第一个 active==false 的槽位。
 * Ego 约定固定在 index 0，由 spawn_ego() 显式分配。
 */
class EntityPool {
public:
    EntityPool() {
        for (int i = 0; i < MAX_ENTITIES; ++i) {
            entities_[i].active = false;
        }
    }

    /** 分配一个新实体，返回索引；池满返回 INVALID_ENTITY。 */
    EntityId alloc(EntityType type) {
        for (int i = 0; i < MAX_ENTITIES; ++i) {
            if (!entities_[i].active) {
                /* Entity 不可拷贝但可移动；Entity{} 创建临时对象后移动赋值。
                 * 旧 road_pos handle（如有）在移动赋值时被 RoadPosition::operator= 自动释放。 */
                entities_[i] = Entity{};
                entities_[i].active = true;
                entities_[i].type = type;
                entities_[i].id = i;
                if (i >= count_) count_ = i + 1;
                return i;
            }
        }
        return INVALID_ENTITY;
    }

    /** 释放实体。 */
    void free(EntityId id) {
        if (id < 0 || id >= MAX_ENTITIES) return;
        entities_[id].active = false;
    }

    /** 访问实体（无边界检查，调用方保证 id 有效）。 */
    Entity& operator[](EntityId id) { return entities_[id]; }
    const Entity& operator[](EntityId id) const { return entities_[id]; }

    /** 已使用的最高索引+1（遍历上界）。 */
    int size() const { return count_; }

    /** 实际活跃实体数。 */
    int active_count() const {
        int n = 0;
        for (int i = 0; i < count_; ++i) if (entities_[i].active) ++n;
        return n;
    }

    /** 清空所有实体（保留容量）。 */
    void clear() {
        for (int i = 0; i < MAX_ENTITIES; ++i) entities_[i].active = false;
        count_ = 0;
    }

    /** 迭代支持 */
    Entity* begin() { return entities_; }
    Entity* end() { return entities_ + count_; }
    const Entity* begin() const { return entities_; }
    const Entity* end() const { return entities_ + count_; }

private:
    Entity entities_[MAX_ENTITIES];
    int    count_{0};
};

}  // namespace flowsim

#endif  // FLOWSIM_ENTITY_H
