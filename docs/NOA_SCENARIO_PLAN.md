# NOA 全链路回归测试场景 — 实施计划

> **状态：设计阶段。** 本文件是对原始"综合场景设计方案"的更新版本，
> 已根据 FlowSim v2 的实际完成状态重新评估每条实施路径。
>
> 上一版本写于 FlowSim v2 动工之前，假设了大量需要从零构建的组件。
> 现在 flowsim 的核心（Entity System / NPC AI / 碰撞 / esmini 集成 /
> scene/frame 发布）已全部落地并通过 `demo_evaluator.py` 验证。

---

## 零、现状盘点：什么已经做好了

### FlowSim v2 已完成的能力（对应原始设计的各阶段需求）

| 原始设计要求 | 当前状态 | 实现位置 |
|-------------|---------|---------|
| 实体系统（Ego/Car/Truck/Pedestrian/TL/ETC/StopLine） | ✅ 完成 | `flowsim/entity.h` |
| 自行车模型 ego 动力学 | ✅ 完成 | `flowsim/physics.cpp` |
| IDM 跟车 + 8 状态 NPC AI | ✅ 完成 | `flowsim/npc_ai.cpp` |
| OBB SAT 碰撞检测 | ✅ 完成 | `flowsim/collision.cpp` |
| 红绿灯相位循环 | ✅ 完成 | `flowsim/scene_events.cpp` |
| ETC 抬杆状态机（closed→opening→open） | ✅ 完成 | `flowsim/scene_events.cpp` |
| esmini RoadManager 集成 + Frenet↔World | ✅ 完成 | `flowsim/road_network.cpp` |
| scene/frame topic（road_network + entities JSON） | ✅ 完成 | `flowsim/scene_pub.cpp` |
| JSON→xodr 转换器（旧格式 + road_network 新格式） | ✅ 完成 | `tools/json_to_xodr.py` |
| curvature_profile（分段圆弧弯道） | ✅ 完成 | `json_to_xodr.py:105-123` |
| pipeline.json 默认使用 flowsim | ✅ 完成 | `config/pipeline.json` |
| flowcoro 协程主循环（20Hz） | ✅ 完成 | `flowsim_node.cpp:498-646` |

### 仍然缺失的能力

| 缺口 | 影响 | 优先级 |
|------|------|--------|
| **分叉/汇入路网**（OpenDRIVE `<junction>`） | 无法表达匝道分叉、加速车道汇入 | 🔴 P0 |
| **planning branch_select** | route 只有 lane_change，不能选路 | 🔴 P0 |
| **planning 汇入逻辑** | 加速车道找间隙并入主线 | 🔴 P0 |
| **perception 绕过 sensor_model** | 感知节点从 vehicle/state 读真值，不读 sensor/lidar | 🔴 P0 |
| **3D 前端不消费 scene/frame entities** | 障碍物渲染仍用 vehicle/state 的 ox/oy 扁平数组 | 🟡 P1 |
| **3D LiDAR 点云是假随机数** | 从未接入真实 sensor/lidar 数据 | 🟡 P1 |
| **障碍物池上限 8 个** | 24 NPC 场景中大量障碍物不显示 | 🟡 P1 |
| **monitor_node 截断 scene/frame 数据** | 只透传 road_network + etc_gate/stop_line，丢弃 NPC entities | 🟡 P1 |

### 近期已修复

| 问题 | 修复 | 日期 |
|------|------|------|
| **车道线不可见**（THREE.Line 线宽 1px） | `_addLaneMarkRibbon()` 替换为 ribbon mesh，线宽 0.12–0.15m | 2026-07-17 |
| **沥青路面无质感**（color=0x3a3f4a 过亮） | 路面颜色改为 0x2a2d30（深灰近黑），roughness 0.95 | 2026-07-17 |
| **路肩颜色与路面接近** | 路肩改为 0x4a4d50（比沥青略亮），有区分度 | 2026-07-17 |
| **车道线无 emissive** | 标线材质添加 `emissiveIntensity: 0.25`，夜间/暗光下仍可见 | 2026-07-17 |

---

## 一、场景流程总览（保持不变）

整个场景是一条连续的驾驶旅程，按 x 坐标分为 6 个阶段（总长约 2.2km）：

```
  x=0        x=200     x=350       x=500     x=650        x=900             x=1400            x=2200
  ┌──────────┬─────────┬───────────┬─────────┬────────────┬────────────────┬──────────────────┐
  │ 城区道路  │ 信号灯  │ ETC 收费站 │ 匝道分叉 │ 急弯匝道    │ 加速车道 + 汇入 │ 高速公路主路      │
  │ 3车道    │ 停止线  │ ETC 闸门   │ 左/右选路 │ R=45m 弯道 │ 加速至 28m/s   │ 变道超车          │
  │ 40km/h   │ 红15+黄 │ 限速20km/h│ 走右匝道  │ 限速40km/h│ 找间隙汇入     │ 100km/h           │
  │          │ 3+绿12  │           │          │            │                │                   │
  └──────────┴─────────┴───────────┴─────────┴────────────┴────────────────┴──────────────────┘
```

| 阶段 | x 范围 | 里程 | 关键行为 |
|------|--------|------|----------|
| **城区路** | 0–200m | 200m | 3 车道城市道路，有静止/慢行 NPC |
| **信号灯路口** | 200–350m | 150m | 红灯停车等待 15s → 绿灯通过 → 横向来车 NPC |
| **ETC 收费站** | 350–500m | 150m | 减速至 20km/h，模拟 ETC 抬杆通过 |
| **匝道分叉** | 500–650m | 150m | 路牌提示，导航指示走右侧匝道 |
| **急弯匝道** | 650–900m | 250m | S 形匝道 + 半径 45m 回头弯，限速 40km/h |
| **加速+汇入** | 900–1400m | 500m | 加速车道 900→1200m 加速至 100km/h，找间隙汇入 |
| **高速超车** | 1400–2200m | 800m | 主路 3 车道，前方慢车 → 左变道超车 → 回到右道 |

---

## 二、核心架构差距及方案

### 2.1 分叉/汇入路网 → 扩展 json_to_xodr.py 生成 `<junction>`

**不需要自己写 RoadGraph。** esmini 就是 RoadGraph。OpenDRIVE 原生支持 `<junction>` 元素
来表示道路分叉和汇入。

当前 `json_to_xodr.py` 已支持的格式：

```json
"road_network": {
  "edges": [
    { "id": 0, "type": "urban", "length_m": 200, "lanes": 3 },
    { "id": 1, "type": "ramp_curve", "length_m": 250, "lanes": 1,
      "curvature_profile": [{"radius": 45, "arc": 130}] }
  ]
}
```

缺口：edges 之间是**顺序拼接**的，不支持"从一条 road 分叉出两条 road"。

**扩展方案**：在 `road_network` 中新增 `junctions` 数组，描述道路连接关系：

```json
"road_network": {
  "edges": [
    { "id": 0, "type": "urban",       "length_m": 200, "lanes": 3 },
    { "id": 1, "type": "intersection", "length_m": 150, "lanes": 3,
      "traffic_lights": [{"s": 200, "red_s": 15, "yellow_s": 3, "green_s": 12}] },
    { "id": 2, "type": "toll",         "length_m": 150, "lanes": 4, "speed_limit": 5.56,
      "etc_gates": [{"s": 100, "l": 0, "open_delay_s": 1.5}] },
    { "id": 3, "type": "ramp_curve",   "length_m": 250, "lanes": 1, "speed_limit": 11.11,
      "curvature_profile": [
        {"radius": 200, "arc": 30}, {"radius": 45, "arc": 130},
        {"radius": -200, "arc": 40}, {"radius": 1000, "arc": 50}
      ]},
    { "id": 4, "type": "merge_lane",   "length_m": 200, "lanes": 1},
    { "id": 5, "type": "highway",      "length_m": 800, "lanes": 3, "speed_limit": 27.78}
  ],
  "junctions": [
    {
      "id": 100,
      "type": "fork",
      "incoming_road": 2,
      "connecting_roads": [
        {"id": 10, "name": "left_ramp",  "length_m": 200, "lanes": 1,
         "curvature_profile": [{"radius": -60, "arc": 100}]},
        {"id": 3, "name": "right_ramp", "length_m": 250, "lanes": 1, "speed_limit": 11.11,
         "curvature_profile": [
           {"radius": 200, "arc": 30}, {"radius": 45, "arc": 130},
           {"radius": -200, "arc": 40}, {"radius": 1000, "arc": 50}
         ]}
      ]
    },
    {
      "id": 101,
      "type": "merge",
      "incoming_road": 4,
      "target_road": 5,
      "target_s": 0
    }
  ]
}
```

`json_to_xodr.py` 新增逻辑（~150 行）：
- 解析 `junctions` 数组
- 对于 `type=fork`：生成 OpenDRIVE `<junction>` 元素，每个 `connecting_road` 生成一条 `<road>` + 对应的 `<link>`（predecessor/successor）
- 对于 `type=merge`：生成 `<junction>` 元素，connecting_road 的 successor 指向 target_road
- 连接道路的 `planView` 初始位置由 incoming_road 的终点状态自动计算（RoadState 已有此能力）

### 2.2 planning branch_select → 扩展 Route Step

当前 `ScenarioRouteStep` 只支持 `lane_change`（见 `scenario_loader.h`）：

```c
// 现有结构
typedef struct {
    double trigger_x;
    int    target_lane;
    double target_speed;
    char   label[32];
} ScenarioRouteStep;
```

**扩展方案**：新增 route step type 字段（向后兼容：type 缺省 = lane_change）：

```c
typedef enum {
    ROUTE_LANE_CHANGE = 0,   // 现有行为
    ROUTE_BRANCH_SELECT,     // 选路（分叉点）
    ROUTE_MERGE,             // 汇入
} RouteStepType;

typedef struct {
    RouteStepType type;       // 新增
    double trigger_x;         // 触发 x 坐标（全局）
    union {
        struct { int target_lane; double target_speed; } lane_change;
        struct { int branch_id; } branch_select;  // junction connecting_road id
        struct { int target_lane; double target_speed; } merge;  // 汇入参数
    };
    char label[32];
} ScenarioRouteStep;
```

场景 JSON 中的表达：
```json
"route": [
  { "trigger_x": 0,    "type": "lane_change",   "target_lane": 0,  "label": "city_cruise" },
  { "trigger_x": 200,  "type": "lane_change",   "target_lane": 0,  "target_speed": 0,   "label": "stop_at_red" },
  { "trigger_x": 500,  "type": "branch_select", "branch_id": 3,    "label": "take_right_ramp" },
  { "trigger_x": 900,  "type": "merge",         "target_lane": 1,  "target_speed": 28,  "label": "merge_highway" },
  { "trigger_x": 1600, "type": "lane_change",   "target_lane": -1, "target_speed": 28,  "label": "overtake_left" },
  { "trigger_x": 1850, "type": "lane_change",   "target_lane": 1,  "label": "return_right" }
]
```

planning_node 的 branch_select 处理逻辑（~80 行）：
1. ego 到达 trigger_x 时检查当前 road_id，确认在分叉点 incoming_road 上
2. 从 road_network 配置中查找目标 `connecting_road_id`
3. 更新 Frenet 参考线为目标 connecting_road → 后续规划自动沿新路走

planning_node 的 merge 处理逻辑（~60 行）：
1. 进入 merge_lane 段后，target_speed 逐步提升到 merge.target_speed
2. 从 `sensor/lidar` 或 `fusion/localization` 获取主线来车信息
3. 找到 ≥ min_gap 的间隙后执行并入
4. 并入后恢复目标车道巡航

### 2.3 perception ↔ sensor_model 数据流对接

当前链路（错误）：
```
flowsim → vehicle/state → perception_node (读真值 ox/oy 撒点)
         → vehicle/state → sensor_model → sensor/lidar → fusion (仅用于定位)
```

修正后的链路：
```
flowsim → vehicle/state → sensor_model → sensor/lidar → perception_node (读模拟 LiDAR)
                                                    → fusion (EKF 定位，不变)
```

改动范围（~50 行，`perception_node.cpp`）：
- 改为订阅 `sensor/lidar`（或同时订阅两个，通过 params 切换模式）
- 从 `sensor/lidar` 的 `LidarFrame` 二进制消息中提取点云数据
- 旧行为保留为 `"mode": "ground_truth"`（向后兼容），新行为用 `"mode": "sensor"`

pipeline.json 对应改动：
```json
{
  "name": "perception",
  "subscribe": ["sensor/lidar"],
  "params": "{\"frequency_hz\":10.0,\"mode\":\"sensor\"}"
}
```

### 2.4 monitor_node 透传完整 entities

当前 `monitor_node.c` 的 `on_scene_frame()` 只提取 `road_network` 和 `etc_gate`/`stop_line` 实体。
需要改为**透传完整的 entities 数组**（~30 行改动）。

这会让 `metrics.scene.entities` 包含所有 NPC 车辆/行人/红绿灯/ETC 的结构化数据，
前端可以直接消费，不再依赖 `vehicle/state` 的 ox/oy 扁平数组。

### 2.5 3D 前端消费 scene/frame entities

当前 `scene3d.js` 的 `update3D()` 障碍物渲染走 `scn.obstacles`（来自 vehicle/state 的 ox/oy），
不读 `scn.entities`。

改动范围（~100 行，`scene3d.js`）：
- `update3D()` 新增 `scn.entities` 消费路径：按 type 分派到 ego/obstacle/tl/etc_gate 池
- 障碍物从 entities 中提取 NPC 车辆/行人（type=car/suv/truck/pedestrian）
- 用 entity 的精确字段（`len/wid/h/vx/vy/ai`）替代 ox/oy 数组的元组
- 障碍物池从 8 扩展到 24（匹配场景需求）
- 旧 `scn.obstacles` 路径保留为 fallback

---

## 三、NPC 阵容（24 个，按段分布）

24 个 NPC 的设计保持与原方案一致，但放在新场景 JSON 的 `actors` 数组中。
每个 actor 分配到一个 `segment_id`（对应 road_network 中的 road id）。
flowsim 的 EntityPool 已支持 64 实体，24 个 NPC + ego + 基础设施实体绰绰有余。

| 编号 | 段 | 类型 | target_vx | 作用 |
|------|----|------|-----------|------|
| **城区道路（Segment 0）** | | | | |
| 0 | 0 | car | 3.0 | 同车道前方慢车，ego 跟车 |
| 1 | 0 | car | 6.0 | 左车道并行，阻塞变道 |
| 2 | 0 | car | 4.0 | 右车道慢车 |
| 3 | 0 | car | 0 | 路边停车 |
| 4 | 0 | pedestrian | 0.8 | 人行道行走 |
| 5 | 0 | pedestrian | -0.6 | 反向行走 |
| **信号灯路口（Segment 1）** | | | | |
| 6 | 1 | car | -6.0 | 横向来车（红灯期间不触发） |
| 7 | 1 | car | -8.0 | 对向直行 |
| 8 | 1 | car | 0 | 同车道等红灯 |
| 9 | 1 | car | 0 | 左道等红灯 |
| 10 | 1 | pedestrian | -1.2 | 绿灯时横穿马路 |
| 11 | 1 | pedestrian | 0.4 | 路边看手机 |
| **ETC 收费站（Segment 2）** | | | | |
| 12 | 2 | car | 1.5 | ETC 排队 |
| 13 | 2 | car | 2.0 | 邻道 ETC |
| 14 | 2 | car | 1.0 | 左道排队 |
| 15 | 2 | car | 5.0 | 已过闸门加速 |
| **左匝道（Junction 100, connecting_road 10）** | | | | |
| 16 | 10 | car | 7.0 | 走左匝道的车（与 ego 分道扬镳） |
| **右匝道（Segment 3）** | | | | |
| 17 | 3 | car | 5.0 | 匝道前方慢车 |
| 18 | 3 | car | 9.0 | 后方快车接近 |
| **加速+汇入（Segment 4+5）** | | | | |
| 19 | 5 | truck | 14.0 | 高速主路卡车（慢） |
| 20 | 5 | car | 26.0 | 左道快车，ego 等它过去 |
| **高速主路（Segment 5）** | | | | |
| 21 | 5 | car | 0 | 应急车道双闪车 |
| 22 | 5 | car | 22.0 | 左后方来车 |
| 23 | 5 | car | 20.0 | 远处车流 |

24 个 NPC，flowsim 的 IDM 跟车模型自动处理排队/减速/保持间距，
不会出现"后车撞上前方静止车辆"的问题。

---

## 四、实施路径（修正版）

### Phase 1 — 路网分叉支持（~2 天）✅ 已完成

| 步骤 | 文件 | 状态 |
|------|------|------|
| 1.1 `json_to_xodr.py` junction 生成 | `tools/json_to_xodr.py` | ✅ 已实现 fork(分叉) + merge(汇入)；fix: duplicate road id |
| 1.2 场景 JSON | `scenarios/city_to_highway_full.json` | ✅ 7 段 road + 2 junction + 24 NPC + branch_select/merge route |
| 1.3 xodr 输出验证 | `json_to_xodr.py --validate` | ✅ 7 unique roads, 2 junctions, 3 connections, 9 arcs |
| 1.4 esmini 加载 | flowsim 启动日志 | 🟡 待 esmini 实际加载测试 |

### Phase 2 — 核心数据流修复（~2 天）

| 步骤 | 文件 | 状态 |
|------|------|------|
| 2.1 perception 支持 sensor/lidar 输入 | `modules/adas_nodes/perception_node.cpp` | 🔴 待其他团队 |
| 2.2 monitor 透传完整 entities | `modules/adas_nodes/monitor_node.c` | ✅ 已有 entities/road_network 透传 (PR #61) |
| 2.3 3D 前端消费 entities | `tools/flowboard/js/scene3d.js` | ✅ 池扩展到 24 + NPC AI 状态标签 (PR #61) |
| 2.4 验证回归 | `demo_evaluator.py` | 🔴 待端到端预跑 |

### Phase 3 — planning 选路与汇入（~3 天）✅ 已完成

| 步骤 | 文件 | 状态 |
|------|------|------|
| 3.1 scenario_loader 新增 branch_select / merge | `src/core/scenario_loader.c` | ✅ 解析 route type + branch_id + target_speed |
| 3.2 planning branch_select 处理 | `planning_node.cpp` | ✅ 分叉选路 + 参考路径切换 + branch_id 日志 |
| 3.3 planning merge 处理 | `planning_node.cpp` | ✅ 加速目标 + 目标车道 + trajectory 下发 |
| 3.4 弯道控制增强 | `control_node.cpp` | 🟡 可优化项，非阻塞 |

### Phase 4 — 场景编排与验证（~2 天）🟡 部分完成

| 步骤 | 状态 |
|------|------|
| 4.1 编写 `city_to_highway_full.json` | ✅ 7 段 road + 2 junction + 24 NPC + branch_select/merge route |
| 4.2 预跑测试 | 🔴 待 esmini 加载 + full pipeline 端到端 |
| 4.3 回归验证 | 🔴 待 Phase 4.2 |
| 4.4 调优参数 | 🔴 待 Phase 4.2 |

### Phase 5 — 可视化增强（~1 天）🟡 部分完成

| 步骤 | 文件 | 状态 |
|------|------|------|
| 5.1 glTF 车辆模型 | `tools/flowboard/gen_models.py`, `models.js` | ✅ 4 类车型 PBR 材质 + 异步加载 + 程序化降级 |
| 5.2 NPC AI 状态标签 | `scene3d.js` | ✅ 障碍物上方 AI 状态文字 (PR #61) |
| 5.3 分叉道路 3D 渲染 | `scene3d.js` | 🔴 待路网分叉实现 |
| 5.4 规划轨迹渲染 | `scene3d.js` | 🔴 待实现 |

**进度：已完成 Phase 1 + 3，Phase 2/4/5 部分完成。剩余约 3 天（预跑验证 + Phase 4 剩余）。**

---

## 五、向后兼容

| 旧特性 | 兼容方式 |
|--------|----------|
| 旧场景 JSON（`road` 字段） | `json_to_xodr.py` 已有 `roads_from_legacy_road()` 路径 |
| 无 `junctions` 的 `road_network` | 按现有顺序拼接逻辑处理（不变） |
| 旧 route（无 `type` 字段） | `type` 缺省 = `lane_change`（不变） |
| perception 旧行为 | `mode: ground_truth`（默认值，不变） |
| 旧 pipeline.json | 所有现有 pipeline 配置零改动 |
| 14 个旧场景 | 全部通过 `demo_evaluator.py` 回归 |

---

## 六、关键设计决策总结

| 决策 | 结论 | 理由 |
|------|------|------|
| 是否自己实现 RoadGraph | **否** | esmini OpenDRIVE 就是 RoadGraph；扩展 json_to_xodr.py 生成 `<junction>` 即可 |
| 是否修改 sim_world_node.c | **否** | flowsim_node.cpp 已完全替代，核心逻辑不动 |
| 是否手写 .xodr | **否** | json_to_xodr.py 已有 curvature_profile 支持，只需加 junction |
| NPC 是否需要独立跟车模型 | **已实现** | flowsim 的 `npc_ai.cpp` 已有 IDM + 8 状态机 |
| 3D 前端是否需要 glTF 模型 | **后期** | 当前 BoxGeometry + 类型着色可先用，glTF 升级是 Phase 6 |
| 感知是否必须走 sensor/lidar | **是** | 当前真值作弊绕过了传感器噪声/FOV/遮挡的全部测试价值 |
