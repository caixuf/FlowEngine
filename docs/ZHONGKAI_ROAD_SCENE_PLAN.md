# 中凯路复合互通自动驾驶仿真场景 — 实施计划

> 目标：在上海松江中凯路（沪昆高速互通 + 主线收费站 + 十字信号平交路口复合型道路）的真实地理拓扑上，
> 复现「高架匝道汇入 / 收费站加塞 / 信号路口抢行」三大高频危险工况，为感知-预测-规划-控制全链路算法验证提供可复现场景。
>
> 对标：用户提供的「中凯路复合互通自动驾驶仿真场景完整设计方案」（豆包生成的原始草案）。
>
> 约束：在 FlowEngine v2 现有能力（flowsim + sensor_model + perception + fusion + planning + control + safety_control + monitor）上增量扩展，
> 不重写已有真实算法节点；前端 Three.js 渲染管线不引入新依赖；xodr 保持在 OpenDRIVE 1.4 子集以维持 esmini 兼容性。
>
> 原则：能用已有节点能触发就别新写算法；豆包草案里有但项目当前完全缺失的能力（毫米波、render-to-texture 相机、raycast LiDAR）单独列「远期扩展」不在初版范围。
>
> 状态：设计阶段（2026-07-19，等用户评审修改后再进入实施）。

---

## 零、现状盘点

> 本节基于对 `/workspace/modules/adas_nodes` 全 29 个节点、4 份 pipeline 配置、`tools/flowboard` 渲染管线、场景 JSON schema 的完整调研。
> 调研产物见本文末「附录 A：调研依据」。

### 0.1 已落地能力（豆包草案里假设"已有"且确实存在的）

| 能力 | 实现位置 | 备注 |
|------|---------|------|
| Bicycle model 自车动力学 | `flowsim_node.cpp` + `flowsim/physics.cpp` | 50 Hz |
| esmini RoadManager 路网查询（Frenet↔World） | `flowsim/road_network.{h,cpp}` | 缺失时降级为旧 curve 几何 |
| IDM 跟车 + NPC 状态机（Cruise/Follow/Stop/StopForTL/ETCApproach/BranchSel/Merge/Yield） | `flowsim/npc_ai.cpp` | 已支持双向 route、recycle 重置 |
| OBB SAT 碰撞 + crash_cooldown | `flowsim/collision.cpp` | 不再永久冻结 |
| 红绿灯/ETC 事件调度 | `flowsim/scene_events.cpp` | 单路口单相位 |
| 信号十字路口（视觉铺装） | `tools/json_to_xodr.py` `cross_roads[]` + `scene3d.js` `_ensureCrossRoadAt` | 3 条 cross_road 已落地于 `city_to_highway_full.json` |
| 高架 elevation | `<elevationProfile>` 透传 → `scene_pub` 三元组 `[x,y,z]` → `scene3d.js` 用 z | 0→8m 已验证 |
| 弯道 curvature_profile | 5 段 S 形（R=60m，转角 90°） | `city_to_highway_full.json` 已修复 |
| 多 edge 串联路网 + fork/merge junction | `json_to_xodr.py` `roads_from_road_network` | 已支持 2 种 junction type |
| DBSCAN 感知 | `perception_node.cpp` | 真实算法，FOV/量程/遮挡过滤 |
| EKF 5D 融合 | `fusion_node.cpp` | 真实算法，LiDAR+GPS+SLAM 三源 |
| Frenet 规划 + NOA 状态机 | `planning_node.cpp` | 真实算法（依赖 Eigen），含 lane_change/branch_select/merge 路线步骤 + merge 闭环 + 红绿灯虚拟停止线墙 |
| PID + Stanley 控制 + 曲率前馈 + ACC + 变道状态机 + LDW + 死锁恢复 | `control_node.cpp` | 真实算法 |
| 安全包络（限幅 + TTC） | `safety_control_node.cpp` | 真实兜底 |
| Kalman + Hungarian 多目标追踪 | `object_tracker_node.cpp` | 真实算法，仅 `pipeline_scene.json` 启用 |
| 3D 渲染：车辆 / 路网 / 红绿灯 / 行人 / 静态障碍物 / ETC 门架 / 雨 / 水洼 / 天空盒 / 太阳光晕 | `tools/flowboard/js/scene3d.js` | 全套 |
| 阴影：DirectionalLight + PCFSoftShadowMap 4096 | `scene3d.js` | 跟随 ego 移动 |
| 后处理：EffectComposer + UnrealBloom + SMAA + 自适应档位 | `scene3d.js` | high/medium/low 三档 |
| 6 种相机模式（chase/top/orbit/driver/front/map） | `CameraController.js` | map 模式已支持拖拽缩放 |
| LiDAR 点云可视化（THREE.Points 900 点缓冲） | `scene3d.js` `_lidarCloud` | 仅消费 `sensor/lidar` 数组 |
| Ego 完整状态（position/velocity/steer/throttle/brake） | `flowsim/entity.h` | 缺 yaw_rate/accel/pitch/roll/z（外部计算） |

### 0.2 部分落地能力（草案假设"已有"但只覆盖一半）

| 能力 | 现状 | 缺口 |
|------|------|------|
| 传感器仿真（LiDAR/GPS/IMU/相机） | `sensor_model_node.c` 仅生成 LidarFrame（中心点+点数+intensity，无真实点云）+ GPS（NMEA 噪声）+ camera 心跳 | 无点云生成、无 IMU 仿真模型、相机无图像 |
| 信号十字路口 | 3 条 `cross_road` 视觉铺装 + 主路 StopForTL 状态 | 东西向无车流、无行人斑马线、信号无联动 |
| 收费站 | 单 `etc_gate` 触发器 + ETCApproach 状态 + 抬杆动画 | 单通道、无多通道并行、无加塞 NPC 行为 |
| U 形绕行匝道 | curvature_profile 已支持任意转角 | 当前转角累计 ≈0（S 形非 U 形）、未跨地面主线、未在双层独立拓扑 |
| 预测 | planning 内部用 `obs_vx/vy` 做 2s 位置外推 | 无独立 prediction_node；`pipeline_scene` 里有但是 sandbox（固定概率 CV/LK/LC） |
| 障碍物语义 | scene/frame 发 `type/id/x/y/h/spd/len/wid/ai/vx/vy` | 无 `height`、无 3D bbox、无 lane_id |
| 红绿灯识别 | `traffic_light_recognition_node.c` 直接读 sim_world 真值 | sandbox，无 CV 模型 |

### 0.3 完全缺失能力（草案假设"已有"但项目当前没有的）

| 缺失 | 影响 | 优先级 |
|------|------|--------|
| 双向道路（同 edge 正反向双 road） | 无法表达中凯路南北向双向 4 车道 | 🔴 P0 |
| 多层独立路网（ground / elevated 独立拓扑） | 无法表达沪昆高速高架层与地面层互不干扰 | 🔴 P0 |
| U 形 180° 跨线匝道（起终点对齐主路两侧） | 无法表达高速车流绕行至对向车道 | 🔴 P0 |
| 收费站多通道并行（N 条 connecting_road） | 无法表达 4 闸口并行排队 | 🔴 P0 |
| 东西向支路车流（cross_road 升级为 junction） | 工况3 缺核心冲突源 | 🟡 P1 |
| 加塞 NPC AI（横向 PID 跨实线变道） | 工况2 缺核心行为 | 🟡 P1 |
| 河道 / 园区建筑 / 绿化带 / 高架桥墩等静态资产 | 感知素材缺失 | 🟡 P1 |
| 障碍物 height / 3D bbox | 感知算法 3D 检测无法验证 | 🟡 P1 |
| PointLight / SpotLight | 夜间路灯/车灯照明无法表达（架构注释明示避免动态光源爆炸） | 🟡 P1 |
| WebGLRenderTarget / CubeCamera | 虚拟相机图像无法生成（render-to-texture 缺失） | 🔵 P2 |
| 基于 raycast 的 LiDAR 点云仿真 | 真实点云分布无法生成 | 🔵 P2 |
| 毫米波雷达（节点 + 仿真模型） | 4D 毫米波工况无法验证 | 🔵 P2（远期扩展） |
| IMU 仿真模型（Allen 方差 / bias drift） | IMU 误差模型简陋 | 🔵 P2 |
| GNSS 多星座 / RTK | 高精定位无法验证 | 🔵 P2 |
| 真实 CV 车道线 / 红绿灯识别 | 感知算法用真值，无算法迭代价值 | 🔵 P2 |
| OpenDRIVE 1.6 完整字段（`<signals>` / `<objects>` / `<t_road_objects>`） | VTD/CARLA/Prescan 严格兼容性 | 🔵 P2 |
| 工况脚本化（多场景包 + 触发条件） | 批量测试无法自动化 | 🟡 P1 |
| 指标采集与导出（ROS bag / OpenLABEL） | 算法迭代无闭环数据 | 🔵 P2 |

---

## 一、目标架构

### 1.1 三段式道路拓扑（按设计方案模块1/2/3 切分）

```
[南段] 沪昆高速高架互通区（高架层 Layer1 + 地面层 Layer0）
  ├─ 沪昆高速东西向 (elevated, 双向 4 车道, 限速 22.22 m/s)
  ├─ U 型绕行匝道 (elevated → ground, 单车道, 限速 11.11 m/s, R=60m, 180°)
  └─ 中凯路南北向主干道 (ground, 双向 4 车道, 限速 13.89 m/s)

[中段] 收费站区域（地面层）
  ├─ 南向 4 通道 toll_plaza (junction type 扩展)
  │   ├─ ETC_1 / ETC_2 (open_delay_s=1.5)
  │   └─ Manual_1 / Manual_2 (open_delay_s=8.0)
  └─ 北向出口拓宽 (3 → 4 车道渐变)

[北段] 十字信号平交路口（地面层）
  ├─ 中凯路南北向直行
  ├─ 园区支路东西向 (cross_road 升级为 at_grade_intersection junction)
  ├─ 红绿灯 ×4 + 行人斑马线 ×4
  └─ 河道禁行边界（路口东侧 30m，蓝色 PlaneGeometry）
```

### 1.2 数据流（沿用主 pipeline.json，不新增节点）

```
flowsim (50Hz, bicycle + esmini + IDM + OBB)
   │
   ├──▶ sensor_model (20Hz) — 仿真 LiDAR/GPS/camera（camera 仅心跳）
   ├──▶ perception (20Hz, DBSCAN) — 真实算法
   ├──▶ fusion (20Hz, EKF 5D) — 真实算法
   ├──▶ planning (20Hz, Frenet + NOA 状态机)
   │       └── route 步骤触发 lane_change / branch_select / merge
   │       └── 红绿灯虚拟停止线墙注入（已有）
   │       └── merge 闭环（已有，从 scene/frame 算前后 gap）
   ├──▶ control (20Hz, PID + Stanley + 曲率前馈 + ACC + 变道状态机)
   ├──▶ safety_control (限幅 + TTC)
   └──▶ monitor (仪表盘)
```

> **关键决策**：不引入 `prediction_node` / `scene_assembler_node` 到主 pipeline。
> 主 pipeline 的 planning 内部已用 `obs_vx/vy` 做 2s 位置外推替代预测功能，
> 引入 sandbox prediction 反而破坏稳定性。预测误差指标改在 `pipeline_scene` 切换时验证。

### 1.3 工况触发机制（不引入新事件类型）

沿用 `route[]` 已有 3 种 type：`lane_change` / `branch_select` / `merge`。

| 工况 | 触发方式 | 复用已有路径 |
|------|---------|------------|
| 工况1 匝道汇入 | `{"type":"merge","target_lane":1,"target_speed":28}` | planning NOA `ROUTE_MERGE` + control 变道状态机 |
| 工况2 收费站通过 | `{"type":"branch_select","branch_id":<etc_channel_id>}` | planning `ROUTE_BRANCH_SELECT` + flowsim `ETCApproach` |
| 工况3 信号路口停车 | `{"type":"lane_change","target_lane":0,"target_speed":0,"label":"stop_at_red"}` | planning 红绿灯虚拟停止线墙 + control 减速 |

---

## 二、核心方案

### 2.1 双向道路数据模型

```json
{ "id": 5, "type": "zhongkai_main",
  "direction": "both",
  "length_m": 400, "lanes": 2, "lane_width": 3.5, "speed_limit": 13.89 }
```

`direction="both"` 时 `json_to_xodr.py` 生成两条 road：
- 正向 road：原 hdg，lane section 正方向
- 反向 road：hdg + π，lane section 负方向，id = base_id + 10000（避免冲突）

ego route 默认走正向；NPC 按 `actor.direction` 字段（+1/-1）分配。

### 2.2 多层独立路网

```json
"road_layers": {
  "ground":  { "z_base": 0.0,   "edges": [...], "junctions": [...], "cross_roads": [...] },
  "elevated": { "z_base": 4.5,   "edges": [...], "junctions": [...] }
}
```

每层独立 `RoadState.z` 偏移 `z_base`，层间无 `<link>` / `<junction>` 连接（仅共享全局坐标系）。
前端 `_buildRoadNetwork` 按 `edge.layer` 分组渲染，高架层 `renderOrder` 高于地面层。

### 2.3 U 形跨线绕行匝道

```json
{ "id": 11, "name": "u_turn_ramp", "lanes": 1, "speed_limit": 11.11,
  "curvature_profile": [
    {"radius": 0,   "arc": 20},
    {"radius": 60,  "arc": 30},
    {"radius": 60,  "arc": 130},   // 主体 U 形 ≈ 124°
    {"radius": 60,  "arc": 30},
    {"radius": 0,   "arc": 80}    // 落地段
  ],
  "elevation_profile": [
    {"s": 0,   "h": 4.5},
    {"s": 200, "h": 4.5},
    {"s": 290, "h": 0.0}          // 落地段线性下降
  ] }
```

起点接沪昆高速高架层终点 `RoadState`，终点对齐中凯路北向 road 某 s 位置（误差 < 1m，否则 route 断链）。

### 2.4 收费站多通道并行（junction type 扩展）

```json
{ "id": 200, "type": "toll_plaza",
  "incoming_road": 5,        // 中凯路南向主线
  "outgoing_road": 8,        // 沪昆高速入口
  "length_m": 60,
  "channels": [
    { "id": 201, "name": "etc_1",     "lane_offset": -5.25, "gate_type": "etc",    "open_delay_s": 1.5 },
    { "id": 202, "name": "etc_2",     "lane_offset": -1.75, "gate_type": "etc",    "open_delay_s": 1.5 },
    { "id": 203, "name": "manual_1",  "lane_offset":  1.75, "gate_type": "manual", "open_delay_s": 8.0 },
    { "id": 204, "name": "manual_2",  "lane_offset":  5.25, "gate_type": "manual", "open_delay_s": 8.0 }
  ] }
```

`json_to_xodr.py` 扩展 `Junction.type` 枚举新增 `toll_plaza`，每个 channel 生成一条 connecting_road（短直线 60m，起点终点偏移 `lane_offset`）。

### 2.5 十字信号路口（cross_road 升级为 junction）

```json
{ "id": 300, "type": "at_grade_intersection",
  "main_road": 5,            // 中凯路北向
  "cross_road_at_s": 275,    // 在主路上的交叉位置
  "cross_road_id": 1002,     // 升级哪条 cross_road
  "connections": [
    { "from": "north", "to": "south", "type": "straight" },
    { "from": "north", "to": "east",  "type": "right" },
    { "from": "north", "to": "west",  "type": "left" },
    { "from": "east",  "to": "west",  "type": "straight" }
  ],
  "signals": [
    { "id": 0, "direction": "north_south", "red_s": 15, "yellow_s": 3, "green_s": 12, "phase_offset_s": 0 },
    { "id": 1, "direction": "east_west",   "red_s": 15, "yellow_s": 3, "green_s": 12, "phase_offset_s": 15 }
  ],
  "crosswalks": [
    { "id": 0, "side": "north" }, { "id": 1, "side": "south" },
    { "id": 2, "side": "east"  }, { "id": 3, "side": "west"  }
  ] }
```

`signals` 数组扩展 `traffic_light_controller`（在 `flowsim/scene_events.cpp` 中），实现南北绿 ↔ 东西红的相位联动。

### 2.6 静态资产清单

| 资产 | 实现方式 | 位置 |
|------|---------|------|
| 河道 | PlaneGeometry 150×40m，蓝色半透明，标记 `collision_forbidden` | 路口东侧 30m |
| 园区建筑 ×4 | BoxGeometry + 顶面贴图 | 道路两侧 20m 外 |
| 绿化带 | PlaneGeometry 5m 宽 + 散布 Cylinder（树） | 主线与匝道之间 |
| 高架桥墩 | CylinderGeometry 每 20m 一根 | 高架层 road 下方 |
| 高架桥面 | BoxGeometry 沿 road | 高架层 road 下方 |
| 收费站顶棚 | 大 BoxGeometry 覆盖 4 通道 | 收费站上方 |
| 收费岛 | 4 个 Box 沿 channel 两侧 | 每个 channel |
| 路灯（夜间光源） | 灯杆（已有）+ emissive 灯罩（已有）+ 路面光斑（PlaneGeometry 半透明黄） | 每 30m |

### 2.7 加塞 NPC AI（工况2 核心）

新增 `AIState::CutIn`，状态机：

```
Cruise → 接近收费站 100m → 选择最短队列通道
                             → 当前通道 != 目标 → CutIn
                                                  → 横向 PID 跨实线变道
                                                  → vx 减速 2 m/s
                                                  → 到达目标通道 → Cruise
                             → 当前通道 == 目标 → Cruise
```

横向 PID 控制器复用 `control_node.cpp` 的 PID 类（参数重调）；
跨实线时记录 `violation_event` 到 `monitor` 仪表盘。

### 2.8 夜间光照（不引入 PointLight/SpotLight，沿用 emissive + Bloom）

> **关键决策**：项目 `scene3d.js` L598 注释明示「不添加 PointLight，避免动态光源数量爆炸」。
> 夜间光照分两档：
> - **5a emissive 档**（初版）：全局环境光强度 0.6 → 0.15，太阳光 1.20 → 0.20（月光），路灯 emissive 强度 ×3，车灯 emissive 开启，红绿灯 emissive 加强；Bloom threshold 0.85 → 0.6 让灯泡更突出。
> - **5b PointLight 档**（性能允许时）：每根路灯 PointLight（强度 50，距离 25m），最多 8 盏动态光源（ego 周围 200m 内）；ego 前照灯 2 个 SpotLight。

---

## 三、实施路径

### Phase 1 — 道路拓扑三维重构（~5 天）🔴 P0

| 步骤 | 文件 | 状态 |
|------|------|------|
| 1.1 双向 edge 数据模型（`direction: both/forward/reverse`） | `tools/json_to_xodr.py` | 🔴 待实施 |
| 1.2 多层 `road_layers` 字段（每层独立 edges/junctions/cross_roads + z_base） | `tools/json_to_xodr.py` + `scene_pub.cpp` + `scene3d.js` | 🔴 待实施 |
| 1.3 U 形 180° 跨线匝道（起点接高架终点，终点对齐地面主路） | `scenarios/zhongkai_road_full.json` | 🔴 待实施 |
| 1.4 `junction.type="toll_plaza"` 扩展（多 channel 并行） | `tools/json_to_xodr.py` | 🔴 待实施 |
| 1.5 `junction.type="at_grade_intersection"` + 信号联动 | `tools/json_to_xodr.py` + `flowsim/scene_events.cpp` | 🔴 待实施 |
| 1.6 新建 `scenarios/zhongkai_road_full.json` 完整场景（南/中/北三段） | `scenarios/zhongkai_road_full.json` | 🔴 待实施 |
| 1.7 xodr 生成验证 + `test_route` 通过 | `tools/json_to_xodr.py` + `flowsim/test_route.cpp` | 🔴 待实施 |

> **M1 里程碑**：ego 能从沪昆高速下匝道 → U 形绕行 → 落地汇入中凯路北向 → 过收费 → 过十字路口。

### Phase 2 — 静态环境资产（~3 天）🟡 P1

| 步骤 | 文件 | 状态 |
|------|------|------|
| 2.1 河道禁行边界（PlaneGeometry + 边界护栏） | `tools/flowboard/js/scene3d.js` | 🔴 待实施 |
| 2.2 园区建筑 POI ×4（Box + 贴图） | `tools/flowboard/js/scene3d.js` | 🔴 待实施 |
| 2.3 绿化带 + 行道树 + 防撞护栏 | `tools/flowboard/js/scene3d.js` | 🔴 待实施 |
| 2.4 高架桥墩 + 桥面（遮挡源） | `tools/flowboard/js/scene3d.js` | 🔴 待实施 |
| 2.5 收费站顶棚 + 岗亭 + 道闸 + 标牌 | `tools/flowboard/js/scene3d.js` | 🔴 待实施 |

### Phase 3 — 动态交通流（~4 天）🔴 P0

| 步骤 | 文件 | 状态 |
|------|------|------|
| 3.1 双向 NPC AI（route_dir / route_layer 三元组匹配前车） | `flowsim/npc_ai.cpp` + `route.cpp` | 🔴 待实施 |
| 3.2 工况1：U 形匝道汇入 NPC AI（`AIState::MergeYield`） | `flowsim/npc_ai.cpp` | 🔴 待实施 |
| 3.3 工况2：收费站加塞 NPC AI（`AIState::CutIn` + 横向 PID） | `flowsim/npc_ai.cpp` | 🔴 待实施 |
| 3.4 工况3：十字路口多向车流 + 行人斑马线 | `scenarios/zhongkai_road_full.json` + `flowsim/scene_events.cpp` | 🔴 待实施 |
| 3.5 障碍物 `height` 字段透传（scene/frame 加 height） | `flowsim/scene_pub.cpp` + `entity.h` | 🔴 待实施 |
| 3.6 基础交通流参数配置（vehicle_mix / speed_limits / behavior_noise） | `scenarios/zhongkai_road_full.json` + `flowsim_node.cpp` | 🔴 待实施 |

> **M2 里程碑**：三大工况可手动触发演示。

### Phase 4 — 夜间光照渲染（~2 天）🟡 P1

| 步骤 | 文件 | 状态 |
|------|------|------|
| 4.1 全局光照下调（环境光 0.6→0.15，太阳光 1.20→0.20） | `tools/flowboard/js/scene3d.js` | 🔴 待实施 |
| 4.2 路灯 emissive 强化 + 路面光斑 PlaneGeometry | `tools/flowboard/js/scene3d.js` | 🔴 待实施 |
| 4.3 车灯 emissive + 对向远光眩光（Bloom threshold 0.85→0.6） | `tools/flowboard/js/scene3d.js` | 🔴 待实施 |
| 4.4 夜间场景配置开关（`scenario.lighting: "night"`） | `include/scenario_loader.h` + `flowsim_node.cpp` | 🔴 待实施 |

### Phase 5 — 工况脚本化（~2 天）🟡 P1

| 步骤 | 文件 | 状态 |
|------|------|------|
| 5.1 工况切换机制（场景 JSON 顶层 `scenarios[]` 数组） | `include/scenario_loader.h` + `flowsim_node.cpp` | 🔴 待实施 |
| 5.2 工况1 脚本：高架匝道汇入 | `scenarios/zhongkai_road_full.json` | 🔴 待实施 |
| 5.3 工况2 脚本：收费站加塞 | `scenarios/zhongkai_road_full.json` | 🔴 待实施 |
| 5.4 工况3 脚本：十字路口抢行 | `scenarios/zhongkai_road_full.json` | 🔴 待实施 |

> **M3 里程碑**：完整闭环可演示（地图 + 资产 + NPC + 夜间 + 工况切换）。

### Phase 6 — 指标采集与导出（~2 天）🔵 P2

| 步骤 | 文件 | 状态 |
|------|------|------|
| 6.1 感知指标（分层识别率 / 漏检率 / 信号灯识别率） | `tools/demo_evaluator.py` | 🔴 待实施 |
| 6.2 预测规划指标（轨迹误差 / 预警提前量 / 制动距离） | `tools/demo_evaluator.py` | 🔴 待实施 |
| 6.3 数据导出（JSONL 已有 + ROS bag 兼容格式 + OpenLABEL） | `tools/data_recorder_node.c` + 新工具 | 🔴 待实施 |
| 6.4 报告生成（Markdown + 关键指标图表） | `tools/demo_evaluator.py` | 🔴 待实施 |

> **M4 里程碑**：可批量测试与指标导出。

---

## 四、远期扩展（不在初版范围）

> 以下能力豆包草案要求，但项目当前完全缺失，需要新增节点或重写渲染管线，初版不做。

| 扩展 | 工作量 | 依赖 |
|------|--------|------|
| 虚拟相机图像（WebGLRenderTarget + render-to-texture → topic） | 大 | 前端无 render-to-texture 管线，需重写 scene3d.js 渲染循环 |
| 基于 raycast 的 LiDAR 点云仿真（128 线 × 1800 点） | 大 | 前端 raycaster 仅用于鼠标拾取；后端无 mesh 几何；需 GPU 加速 |
| 4D 毫米波雷达（5 路节点 + 仿真模型） | 大 | `RadarTarget` 结构体仅占位，无节点；需新建 `radar_model_node` |
| IMU 仿真模型（Allen 方差 / bias drift / 温度漂移） | 中 | 当前 dry-run 仅静止 + 小噪声 |
| GNSS 多星座 / RTK | 中 | 仅单频 NMEA GPS |
| 真实 CV 车道线检测（Canny + Hough） | 中 | `lane_detection_node` sandbox，`HAVE_CV` 宏未启用 |
| 真实红绿灯识别 CV 模型 | 中 | `traffic_light_recognition_node` sandbox，直接读真值 |
| 真实 DL 运动预测（LSTM/CVRNN） | 大 | `prediction_node` sandbox，固定概率三模式 |
| 真实 SLAM（FAST-LIO2 / LIO-SAM） | 大 | `slam_node` 占位，dead_reckon |
| OpenDRIVE 1.6 完整字段（`<signals>` / `<objects>` / `<t_road_objects>`） | 中 | 当前 1.4 子集，1.6 兼容性收益有限 |
| RAG 知识库自动归档 | 大 | 全新基础设施 |

---

## 五、向后兼容

| 旧特性 | 兼容方式 |
|--------|----------|
| 旧 `road{curve_start_x/length_m/offset_m}` 场景 | 不动，`json_to_xodr.py` 旧分支保留 |
| `city_to_highway_full.json`（当前雏形） | 保留作回归基线，新场景另起 `zhongkai_road_full.json` |
| 单向 edge（无 `direction` 字段） | 默认 `direction="forward"`，行为不变 |
| 单层 `road_network`（无 `road_layers`） | 默认单层 ground，z_base=0，行为不变 |
| 旧 `junction.type=fork/merge` | 不动，新增 `toll_plaza` / `at_grade_intersection` 是枚举扩展 |
| 旧 `cross_roads[]`（独立横向铺装） | 保留；如升级为 junction 则用新字段 `at_grade_intersection` |
| 旧 `route[]` 三种 type | 不动，工况触发全复用 |
| 旧 scene/frame JSON 字段 | 不动，仅追加 `height` / `layer` / `direction` 等可选字段 |
| 旧 `pipeline.json`（12 节点） | 不动，不引入 prediction/scene_assembler 到主 pipeline |
| 旧 `pipeline_scene.json` / `pipeline_highway.json` / `pipeline_car.json` | 不动 |

---

## 六、关键设计决策总结

| 决策 | 结论 | 理由 |
|------|------|------|
| 是否升级 OpenDRIVE 到 1.6 | **否** | 1.4 子集兼容性最好，esmini 已验证；1.6 字段（`<signals>`/`<objects>`）对当前 ADAS 节点消费方收益有限，工作量与收益不匹配 |
| 是否引入 prediction_node 到主 pipeline | **否** | sandbox 算法（固定概率 CV/LK/LC）不如 planning 内部 obs 速度外推稳定；预测误差指标改在 `pipeline_scene` 切换时验证 |
| 是否引入 PointLight/SpotLight | **初版否** | 项目 `scene3d.js` 注释明示避免动态光源爆炸；先用 emissive + Bloom 模拟夜间光照，性能允许时再上 PointLight（5b 档） |
| 是否实现虚拟相机图像（render-to-texture） | **否** | 前端无 WebGLRenderTarget 管线，需重写渲染循环；初版用 driver 相机模式（POV）+ sensor/camera 心跳即可 |
| 是否实现基于 raycast 的 LiDAR 点云 | **否** | 后端无 mesh 几何，前端 raycaster 仅用于鼠标拾取；初版沿用 sensor_model 的 FOV 估点数 |
| 是否新建 `radar_model_node` | **否** | `RadarTarget` 结构体仅占位，需全新节点；4D 毫米波工况列入远期扩展 |
| 是否扩展 junction.type 枚举（toll_plaza / at_grade_intersection） | **是** | 现有 fork/merge 两种不足以表达复合互通；扩展是兼容性增量 |
| 是否新建 `zhongkai_road_full.json` | **是** | 保留 `city_to_highway_full.json` 作回归基线；新场景另起避免破坏 |
| 双向 road 用两条独立 road 还是 OpenDRIVE 双向车道 | **两条独立 road** | 实现简单，ego/NPC route 互不干扰；OpenDRIVE 双向车道对 lane section 处理复杂 |
| 多层路网用同一 xodr 还是分文件 | **同一 xodr** | esmini 支持多层 road 共存；分文件需要多 RoadManager 实例，复杂度上升 |
| 工况切换用新事件类型还是复用 route | **复用 route** | route 已有 lane_change/branch_select/merge 三种 type，覆盖三大工况触发；新增事件类型破坏 control 状态机 |

---

## 七、风险与依赖

| 风险 | 概率 | 缓解 |
|------|------|------|
| esmini 对多层 road 共存支持不稳 | 中 | 提前用 minimal xodr 验证；不行则降级为单层 + elevation 高度差模拟 |
| U 形匝道终点对齐误差导致 route 断链 | 中 | `test_route` 必须验证；断链时在 json_to_xodr 加自动 snap 到最近主路 s |
| 双向 NPC 在路口对向冲突 | 中 | `find_lead` 严格按 route_dir 区分；对向车辆只用于决策不让行 |
| 加塞 NPC 跨实线时与 ego 碰撞 | 高 | 工况2 脚本里强制 ego 在 manual 通道（与 ETC 通道分隔）；横向 PID 限速 2 m/s |
| PointLight 5b 档性能不达标 | 高 | 限制动态光源 ≤8 盏（ego 200m 内）；medium/low 档强制走 emissive |
| 三段路网总长 >2km，Three.js 渲染压力 | 中 | 已有 InstancedMesh 复用车辆；路网 mesh 按 chunk 分组，远距离 chunk 降级 |
| scenario_loader.c 不识别新字段（road_layers / scenarios） | 高 | 新字段在 flowsim_node.cpp 解析（已支持 JSON 直读）；不依赖 C 加载器 |

---

## 八、总结

豆包草案对中凯路复合互通场景的描述完整，但假设的传感器栈（3 摄像头 + 128 线 LiDAR + 5 毫米波 + IMU/GNSS）和 OpenDRIVE 1.6 兼容性要求超出 FlowEngine v2 当前能力。本计划把草案拆为「初版可落地」与「远期扩展」两档：初版聚焦道路拓扑三维重构、静态资产、动态交通流、夜间光照、工况脚本、指标采集 6 个 Phase，全部复用已有真实算法节点（DBSCAN/EKF/Frenet/PID+Stanley），仅在场景 JSON schema 和 NPC AI 状态机上增量扩展；远期扩展项（虚拟相机、raycast LiDAR、毫米波、CV/DL 算法）单独列出，不阻塞初版。

**一句话：把豆包草案里"假设已有但项目实际缺失"的部分（毫米波、render-to-texture、raycast LiDAR）砍到远期，初版聚焦场景 JSON 扩展 + NPC AI 状态机扩展 + Three.js 资产，6 个 Phase / 28 个工作包全部基于已有真实算法节点增量演进，不动主 pipeline 12 节点拓扑。**

---

## 附录 A：调研依据

| 调研维度 | 关键文件 | 结论 |
|---------|---------|------|
| ADAS 节点全貌 | `modules/adas_nodes/` 29 节点 + `config/pipeline*.json` ×4 | 主 pipeline 12 节点完整链路，5 节点 sandbox，12 节点未启用 |
| 传感器与渲染 | `sensor_model_node.c` + `tools/flowboard/js/scene3d.js` | sensor_model 仅 FOV 估点数；前端无 render-to-texture / PointLight / SpotLight |
| 场景 JSON schema | `include/scenario_loader.h` + `tools/json_to_xodr.py` | 三套消费者字段不对称；edge.type 自由字符串无枚举；junction 仅 fork/merge |
| docs 规范 | `docs/FLOWSIM_ARCHITECTURE.md` + `NOA_SCENARIO_PLAN.md` + `EVOLUTION_ROADMAP.md` + `E2E_LEARNING_V3_PLAN.md` | 文件名 UPPER_SNAKE_CASE + 后缀；H1 em-dash；中文序数章节；表格密度高；emoji 状态标记 |

---

## 附录 B：相关文档

- [`NOA_SCENARIO_PLAN.md`](NOA_SCENARIO_PLAN.md) — 上一版场景实施计划，本计划是其后继
- [`FLOWSIM_ARCHITECTURE.md`](FLOWSIM_ARCHITECTURE.md) — FlowSim v2 架构设计，本计划依赖其 esmini 集成
- [`EVOLUTION_ROADMAP.md`](EVOLUTION_ROADMAP.md) — 项目进化路线图，本计划属于 Phase 7+ 范畴
- [`FLOWBOARD_SCENE_CONTRACT.md`](FLOWBOARD_SCENE_CONTRACT.md) — scene/frame JSON 字段契约，本计划扩展 `height`/`layer`/`direction` 字段需同步更新
