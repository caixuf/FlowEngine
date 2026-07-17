#ifndef SCENARIO_LOADER_H
#define SCENARIO_LOADER_H

/**
 * @file scenario_loader.h
 * @brief 场景描述文件加载器（JSON → C 结构体）
 *
 * 场景文件格式（JSON）：
 * {
 *   "name": "pedestrian_crossing",
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

#define SCENARIO_MAX_ACTORS      16
#define SCENARIO_NAME_LEN        64
#define SCENARIO_DESC_LEN       128
#define SCENARIO_MAX_ROUTE_STEPS  8
#define SCENARIO_ROUTE_LABEL_LEN 32
#define SCENARIO_MAX_TRAFFIC_LIGHTS 4
#define SCENARIO_MAX_ETC_GATES      4
#define SCENARIO_MAX_STOP_LINES     4

/* ── Actor（NPC 车辆 / 行人） ─────────────────────────────── */

typedef struct {
    int    id;
    char   type[16];   /**< "car" | "pedestrian" | "truck" */
    double x, y;       /**< 初始位置（m，世界坐标系） */
    double vx, vy;     /**< 初始速度（m/s） */
    double len, wid;   /**< 碰撞包围盒尺寸（m） */
} ScenarioActor;

/* ── Ego 初始状态 ─────────────────────────────────────────── */

typedef struct {
    double x, y;           /**< 初始位置（m） */
    double heading;        /**< 初始航向角（rad） */
    double init_speed;     /**< 初始速度（m/s） */
    double target_speed;   /**< 期望巡航速度（m/s） */
} ScenarioEgo;

/* ── 导航路线变道指令（NOA 主动变道） ───────────────────────── */

typedef struct {
    double trigger_x;     /**< ego x 越过此值触发（m） */
    int    target_lane;   /**< 目标车道方向: -1=初始车道一侧(y<0), +1=对侧相邻车道(y>0) */
    double target_speed;  /**< 目标速度（m/s），0 = 不改变当前目标速度（可选项，用于出口匝道减速等） */
    char   label[SCENARIO_ROUTE_LABEL_LEN]; /**< 可读描述（可选） */
} ScenarioRouteStep;

/* ── 道路几何（可选弯道定义, 见 road_geometry.h） ───────────── */

typedef struct {
    double curve_start_x;   /**< 弯道起点 x（m），默认 0 */
    double curve_length_m;  /**< 弯道长度（m），<=0 = 禁用（直道） */
    double curve_offset_m;  /**< 弯道终点横向偏移（m），默认 0 */
} ScenarioRoad;

/* ── 红绿灯（可选, 见 traffic_light.h） ────────────────────── */

typedef struct {
    int    id;              /**< 灯 ID（topic/可视化标识） */
    double x;               /**< 停止线位置（ego 前向坐标, m） */
    double y_lane;          /**< 灯所在车道横向坐标（m，默认 -1.75） */
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
    double approach_speed;  /**< 通过时目标速度（m/s，默认 5.0 = ETC 减速） */
    double open_range_m;    /**< ego 进入此距离时抬杆（m，默认 50） */
} ScenarioETCGate;

/* ── 停止线（路口/ETC 停车位置标记，FlowSim v2 新增） ─────── */

typedef struct {
    int    id;              /**< 停止线 ID */
    double x;               /**< 停止线位置（ego 前向坐标, m） */
    double y;               /**< 横向坐标（m，默认 0） */
} ScenarioStopLine;

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
