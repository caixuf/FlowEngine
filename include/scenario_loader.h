#ifndef SCENARIO_LOADER_H
#define SCENARIO_LOADER_H

/**
 * @file scenario_loader.h
 * @brief 场景描述文件加载器（JSON → C 结构体）
 *
 * 场景文件格式（JSON）：
 * {
 *   "name": "infinite_straight",
 *   "description": "...",
 *   "random_seed": 42,
 *   "duration_s": 60.0,
 *   "ego": {
 *     "x": 0.0, "y": -1.75,
 *     "init_speed": 5.0, "target_speed": 12.0, "heading": 0.0
 *   },
 *   "actors": [
 *     { "id": 0, "type": "car", "x": 35.0, "y": -1.75,
 *       "vx": 7.0, "vy": 0.0, "len": 4.6, "wid": 2.0 },
 *     ...
 *   ],
 *   "pass_criteria": {
 *     "no_collision": true,
 *     "max_duration_s": 60.0,
 *     "min_avg_speed_mps": 5.0
 *   },
 *   "route": [
 *     { "trigger_x": 200.0, "target_lane": 1,  "label": "prepare_exit" },
 *     { "trigger_x": 400.0, "target_lane": -1, "label": "return_main_lane" }
 *   ],
 *   "road": {
 *     "curve_start_x": 300.0,
 *     "curve_length_m": 200.0,
 *     "curve_offset_m": 15.0
 *   }
 * }
 *
 * route（可选）: 导航路线的主动变道指令序列，是 NOA（领航辅助）区别于
 * 单车道居中（LCC）的关键能力——不依赖障碍物触发，而是由导航路线本身
 * 驱动车辆提前变道（如为出口/匝道做准备）。
 *   trigger_x    ego 世界坐标 x 越过此值时触发该指令（m）
 *   target_lane  目标车道方向: -1 = ego 初始车道一侧 (世界坐标 y<0),
 *                +1 = 对侧相邻车道 (世界坐标 y>0)
 *   label        可读描述（可选，用于日志/可视化）
 *
 * road（可选）: 道路中心线弯道定义（见 road_geometry.h）。curve_length_m
 * 缺省或 <=0 时视为直道（禁用），与不含 "road" 字段的既有场景完全等价。
 *   curve_start_x   弯道起点（ego x 坐标, m）
 *   curve_length_m  弯道纵向长度（m），0 = 禁用
 *   curve_offset_m  弯道终点相对起点的横向偏移（m，可正可负）
 *
 * traffic_lights（可选）: 红绿灯定义数组（见 traffic_light.h）。不填 = 无红绿灯，
 * 与既有场景完全兼容。每个灯含停止线位置 + 三色时长 + 初始相位偏移。
 *   [
 *     {"id":0,"x":100.0,"y_lane":-1.75,"red_s":20.0,"yellow_s":3.0,
 *      "green_s":15.0,"phase_offset_s":0.0}
 *   ]
 *   id               灯 ID（用于 topic/可视化标识）
 *   x                停止线位置（ego 前向坐标, m）— ego 到此 x 时应停车
 *   y_lane           灯所在车道横向坐标（m，默认 -1.75 兼容单车道）
 *   red_s/yellow_s/green_s  三色时长（s），任一 <=0 视为无灯（恒绿灯）
 *   phase_offset_s   初始相位偏移（s，默认 0，用于多灯错峰）
 */

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SCENARIO_MAX_ACTORS      32   /* NOA 24-NPC 场景需要 >16 */
#define SCENARIO_NAME_LEN        64
#define SCENARIO_DESC_LEN       128
#define SCENARIO_MAX_ROUTE_STEPS  8
#define SCENARIO_ROUTE_LABEL_LEN 32
#define SCENARIO_MAX_TRAFFIC_LIGHTS 4
#define SCENARIO_MAX_ETC_GATES      4
#define SCENARIO_MAX_STOP_LINES     4
#define SCENARIO_MAX_SCRIPTS        8   /* 顶层 scenarios[] 工况脚本数组上限 */
#define SCENARIO_MAX_OVERRIDES      8   /* 单个工况脚本的 actor_overrides 上限 */
#define SCENARIO_SCRIPT_LABEL_LEN   48

/* ── Actor（NPC 车辆 / 行人） ─────────────────────────────── */

typedef struct {
    int    id;
    char   type[16];   /**< "car" | "pedestrian" | "truck" */
    double x, y;       /**< 初始位置（m，世界坐标系）— 旧格式 */
    double vx, vy;     /**< 初始速度（m/s） */
    double len, wid;   /**< 碰撞包围盒尺寸（m） */
    /* ── road_network 新格式：按 segment 放置 ── */
    int    segment_id; /**< 所属 road edge id（新格式，-1=旧格式用 x/y） */
    double s;          /**< 沿道路纵向位置 m（Frenet s，新格式） */
    double l;          /**< 车道横向偏移 m（新格式，0=参考线） */
} ScenarioActor;

/* ── Ego 初始状态 ─────────────────────────────────────────── */

typedef struct {
    double x, y;           /**< 初始位置（m） */
    double heading;        /**< 初始航向角（rad） */
    double init_speed;     /**< 初始速度（m/s） */
    double target_speed;   /**< 期望巡航速度（m/s） */
} ScenarioEgo;

/* ── 导航路线变道指令（NOA 主动变道） ───────────────────────── */

/**
 * NOA 路线步骤类型。type 字段决定 planning_node 在 trigger_x 处如何处理：
 *   ROUTE_LANE_CHANGE   : 普通变道（同一路段内换车道），用 target_lane 指定方向
 *   ROUTE_BRANCH_SELECT : 路口分叉选路（fork junction），用 branch_id 指定走哪条
 *                         connecting_road（json_to_xodr 生成的 junction 内 connecting
 *                         road 的 id）。planning_node 据此切换 Frenet 参考路径。
 *   ROUTE_MERGE         : 加速车道汇入主路（merge junction），用 target_lane 指定汇入
 *                         主路后的目标车道方向，target_speed 指定加速目标。
 *
 * 缺省（无 "type" 字段）= ROUTE_LANE_CHANGE，与既有场景文件完全向后兼容。
 */
typedef enum {
    ROUTE_LANE_CHANGE    = 0,
    ROUTE_BRANCH_SELECT  = 1,
    ROUTE_MERGE          = 2,
} RouteStepType;

typedef struct {
    double        trigger_x;     /**< ego x 越过此值触发（m） */
    RouteStepType type;          /**< 步骤类型（默认 ROUTE_LANE_CHANGE） */
    /* 目标车道。
     * 新语义（N 车道模型）: 0..N-1=目标车道索引（0=最左, N-1=最右），-1=无目标。
     * 旧语义（2 车道模型）: -1=左车道(y<0), 0=无目标, +1=右车道(y>0)。
     * control_node 接收时按当前 lane_count 做兼容映射：
     *   - 旧值 +1（若 lane_count==2）→ idx 1（右车道）
     *   - 旧值 -1（若 lane_count==2）→ idx 0（左车道）
     *   - 旧值 0 → idx -1（无目标）
     * 新场景应直接写 0..N-1 索引。 */
    int           target_lane;
    int           branch_id;     /**< ROUTE_BRANCH_SELECT: connecting_road 的 id（fork 选路） */
    double        target_speed;  /**< 目标速度（m/s），0 = 不改变当前目标速度（可选项，用于出口匝道减速等） */
    char          label[SCENARIO_ROUTE_LABEL_LEN]; /**< 可读描述（可选） */
} ScenarioRouteStep;

/* ── 道路几何（可选弯道定义, 见 road_geometry.h） ───────────── */

typedef struct {
    double curve_start_x;   /**< 弯道起点 x（m），默认 0 */
    double curve_length_m;  /**< 弯道长度（m），<=0 = 禁用（直道） */
    double curve_offset_m;  /**< 弯道终点横向偏移（m），默认 0 */
    char   type[32];        /**< 道路类型（road_network 新格式：viaduct_highway/urban/ramp_curve 等） */
} ScenarioRoad;

/* ── 红绿灯（可选, 见 traffic_light.h） ────────────────────── */

typedef struct {
    int    id;              /**< 灯 ID（topic/可视化标识） */
    double x;               /**< 停止线位置（ego 前向坐标, m） */
    double y_lane;          /**< 灯所在车道横向坐标（m，默认 -1.75） */
    double heading;         /**< 灯朝向（rad，默认 0 = 朝 +X；未配置时 flowsim 可按道路切线估算） */
    double red_s;           /**< 红灯时长（s），<=0 视为无灯 */
    double yellow_s;        /**< 黄灯时长（s） */
    double green_s;         /**< 绿灯时长（s） */
    double phase_offset_s;  /**< 初始相位偏移（s，默认 0） */
} ScenarioTrafficLight;

/* ── ETC 门架（高速收费站抬杆，FlowSim v2 新增） ─────────── */

typedef struct {
    int    id;              /**< 门架 ID（可视化标识） */
    double x;               /**< 门架位置（ego 前向坐标, m） */
    double y;               /**< 横向坐标（m，默认 0 = 跨路面中心） */
    double heading;         /**< 门架朝向（rad，默认 0 = 朝 +X；未配置时 flowsim 可按道路切线估算） */
    double approach_speed;  /**< 通过时目标速度（m/s，默认 5.0 = ETC 减速） */
    double open_range_m;    /**< ego 进入此距离时抬杆（m，默认 50） */
} ScenarioETCGate;

/* ── 停止线（路口/ETC 停车位置标记，FlowSim v2 新增） ─────── */

typedef struct {
    int    id;              /**< 停止线 ID */
    double x;               /**< 停止线位置（ego 前向坐标, m） */
    double y;               /**< 横向坐标（m，默认 0） */
    double heading;         /**< 停止线法向（rad，默认 0 = 垂直于 +X；未配置时 flowsim 可按道路切线估算） */
} ScenarioStopLine;

/* ── 光照模式（FlowSim v2 中凯路场景新增） ─────────────────── */

/**
 * 场景全局光照模式。场景 JSON 顶层 "lighting" 字段映射到此枚举。
 * 缺省 = SCENARIO_LIGHT_DAY（与既有场景完全向后兼容）。
 * SCENARIO_LIGHT_NIGHT 用于夜间场景：scene3d.js 降低 AmbientLight + DirectionalLight，
 * 提升车辆 emissive（大灯/尾灯），Bloom 阈值降低凸显发光体。
 */
typedef enum {
    SCENARIO_LIGHT_DAY   = 0,  /**< 白天（默认）：环境光 0.20 + 平行光 1.20 */
    SCENARIO_LIGHT_NIGHT = 1,  /**< 夜间：环境光 0.04 + 平行光 0.15 + 大灯亮起 */
    SCENARIO_LIGHT_DUSK  = 2,  /**< 黄昏：环境光 0.12 + 平行光 0.50 + 暖色调 */
} ScenarioLighting;

/* ── 工况脚本（顶层 scenarios[] 数组，FlowSim v2 新增） ─────── */

/**
 * 工况触发类型。
 *   SCRIPT_TRIGGER_EGO_X_GTE : ego 世界 x ≥ value 时触发（顺行经过某点）
 *   SCRIPT_TRIGGER_EGO_X_LTE : ego 世界 x ≤ value 时触发（对向/回程场景）
 *   SCRIPT_TRIGGER_TIME_GTE  : 仿真累计时间 ≥ value（s）时触发
 *   SCRIPT_TRIGGER_ROUTE_S_GTE : ego route 累计 s ≥ value 时触发（含弯道/匝道）
 */
typedef enum {
    SCRIPT_TRIGGER_EGO_X_GTE    = 0,
    SCRIPT_TRIGGER_EGO_X_LTE    = 1,
    SCRIPT_TRIGGER_TIME_GTE     = 2,
    SCRIPT_TRIGGER_ROUTE_S_GTE  = 3,
} ScriptTriggerType;

typedef struct {
    ScriptTriggerType type;   /**< 触发类型 */
    double            value;  /**< 触发阈值（m 或 s） */
} ScenarioTrigger;

/**
 * 工况触发时对单个 actor 的属性覆盖。仅列出的字段被覆盖，未列出的保持不变。
 * 字段语义对应 npc_ai.cpp 的 Entity 字段：
 *   ai_state       → Entity.ai_state（如 "cutin" 对应 AIState::CutIn）
 *   target_offset  → Entity.target_offset（CutIn 目标横向位置）
 *   target_vx      → Entity.target_vx（巡航速度覆盖）
 *   vx/vy          → 直接设置速度（用于行人横穿）
 *   active         → 是否激活动画（行人 parked 翻转等，预留）
 *
 * ai_state 字符串映射（小写）：cruise/follow/stop/stop_for_tl/etc_approach/
 * branch_sel/merge/yield/cutin。flowsim_node.cpp 用 flowsim_ai_state_from_str() 解析。
 */
typedef struct {
    int    actor_id;            /**< 被覆盖的 actor id（场景 JSON actors[].id） */
    char   ai_state[16];        /**< 覆盖 AI 状态（"cutin"/"stop"/"yield"/...），空串 = 不覆盖 */
    double target_offset;       /**< 覆盖目标横向偏移（m），NaN = 不覆盖 */
    double target_vx;           /**< 覆盖目标速度（m/s），NaN = 不覆盖 */
    double vx;                  /**< 直接设置 vx（m/s），NaN = 不覆盖 */
    double vy;                  /**< 直接设置 vy（m/s），NaN = 不覆盖 */
} ScenarioActorOverride;

/**
 * 单个工况脚本：触发器 + 触发时对 actors 的属性覆盖。
 * fired 字段运行时使用：触发一次后置 true，避免重复触发。
 */
typedef struct {
    char                   name[SCENARIO_NAME_LEN];       /**< 工况名（"toll_cutin" 等） */
    char                   label[SCENARIO_SCRIPT_LABEL_LEN]; /**< 可读描述 */
    ScenarioTrigger        trigger;                       /**< 触发条件 */
    ScenarioActorOverride  overrides[SCENARIO_MAX_OVERRIDES]; /**< 触发时应用的属性覆盖 */
    int                    override_count;
    bool                   fired;                         /**< 运行时：是否已触发过（防重复） */
} ScenarioScript;

/* ── 通过 / 失败判据 ──────────────────────────────────────── */

typedef struct {
    bool   no_collision;       /**< true = 碰撞即 FAIL */
    double max_duration_s;     /**< 超时即 FAIL（0 = 不设超时） */
    double min_avg_speed_mps;  /**< 平均速度低于此值即 FAIL（0 = 不检查） */
    double min_distance_m;     /**< ego 行驶距离低于此值即 FAIL（0 = 不检查） */
} ScenarioCriteria;

/* ── 场景配置主体 ─────────────────────────────────────────── */

typedef struct {
    char             name[SCENARIO_NAME_LEN];
    char             description[SCENARIO_DESC_LEN];
    uint32_t         random_seed;      /**< 固定随机种子（确定性仿真） */
    double           duration_s;       /**< 场景最大运行时长（s，0 = 不限制） */
    ScenarioEgo      ego;
    ScenarioActor    actors[SCENARIO_MAX_ACTORS];
    int              actor_count;
    ScenarioCriteria criteria;
    ScenarioRouteStep route[SCENARIO_MAX_ROUTE_STEPS]; /**< 导航路线变道指令（可选，NOA 用） */
    int               route_count;
    ScenarioRoad      road;    /**< 道路几何（可选弯道，默认全零 = 直道） */
    ScenarioTrafficLight traffic_lights[SCENARIO_MAX_TRAFFIC_LIGHTS]; /**< 红绿灯（可选，默认全零 = 无灯） */
    int               traffic_light_count;
    ScenarioETCGate   etc_gates[SCENARIO_MAX_ETC_GATES]; /**< ETC 门架（FlowSim v2 新增） */
    int               etc_gate_count;
    ScenarioStopLine  stop_lines[SCENARIO_MAX_STOP_LINES]; /**< 停止线（FlowSim v2 新增） */
    int               stop_line_count;
    /* ── 中凯路场景新增（Task 3 + Task 4）── */
    ScenarioLighting  lighting;        /**< 全局光照模式（day/night/dusk，默认 day） */
    ScenarioScript    scripts[SCENARIO_MAX_SCRIPTS]; /**< 顶层 scenarios[] 工况脚本数组 */
    int               script_count;
} ScenarioConfig;

/**
 * 从 JSON 文件加载场景配置。
 *
 * @param path  JSON 文件路径（绝对或相对路径）
 * @return 分配的场景配置指针，失败返回 NULL。
 *         调用者负责用 scenario_free() 释放。
 */
ScenarioConfig* scenario_load(const char* path);

/**
 * 释放 scenario_load() 返回的配置。
 */
void scenario_free(ScenarioConfig* scenario);

/**
 * 将场景配置序列化为 JSON 字符串（用于调试输出）。
 * 返回动态分配的字符串，调用者负责 free()。
 */
char* scenario_to_json(const ScenarioConfig* scenario);

#ifdef __cplusplus
}
#endif

#endif /* SCENARIO_LOADER_H */
