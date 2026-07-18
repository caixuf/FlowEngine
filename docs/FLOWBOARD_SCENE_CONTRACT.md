# FlowBoard 3D Scene 数据契约

> 本文件是 `metrics.scene`（由 `monitor_node.c` 拼装，供 `tools/flowboard/js/scene3d.js` 消费）的**唯一事实来源**。任何修改 scene schema 的行为都应同步更新本文件，并提升版本号。

## 1. 版本

- **版本**: 2.0.0
- **生效日期**: 2026-07-18
- **变更**:
  - v1.1.0 新增 `entities` 中 `tl`/`etc_gate`/`stop_line` 的 `h`（heading）字段
  - v1.2.0 `trajectory_path` 每点新增可选第 4 元素 `edge_id`；前端实现跨 edge 链式投影，解决弯道投影不准
  - v2.0.0 **破坏性变更**：移除 `scene.traffic_lights`（ego-relative fallback）字段，红绿灯统一由 `scene.entities` 中的 `tl`（world 坐标）提供。monitor 不再订阅 `road/traffic_lights` 透传到 scene（该 topic 仍由 sim_world 发布，供 planning/inference/recognition 消费）
- **维护者**: sim-world / perception / flowboard 三端共同维护

## 2. 数据链路

```text
flowsim_node (sim-world)
  └── topic: scene/frame ────────┐
                                 │
planning_node                    │
  └── topic: planning/trajectory ┼──► monitor_node ──► /tmp/flow_topology.json ──► flowboard/scene3d.js
                                 │      (融合 + 归一化)
sim_world_node                   │
  └── topic: road/geometry ──────┘
```

- **sim-world 侧（flowsim_node）**：发布 `scene/frame`，包含 `road_network` + `entities`（红绿灯以 `tl` 实体提供，world 坐标）
- **感知/规划侧**：发布 `vehicle/state`、`planning/trajectory` 等
- **monitor_node**：将多个 topic 融合为 `metrics.scene`，写入状态文件
- **flowboard/scene3d.js**：仅消费 `metrics.scene`，不反向影响上游
- **注**：`road/traffic_lights` topic 仍由 sim_world 发布，供 planning/inference/recognition 消费，但不再流入 `metrics.scene`（v2.0.0 移除）

## 3. 坐标系约定

所有**世界坐标**字段遵循以下右手坐标系：

- **X 轴**: 纵向（forward / east），沿道路参考线切线方向
- **Y 轴**: 高度（up），地面通常为 0
- **Z 轴**: 横向（right / south），垂直于道路参考线

前端 Three.js 中：

- 场景坐标：`Vector3(x, 0, y)`，即把消息的 `(x, y)` 映射到 Three.js 的 `(x, z)`
- 车辆/模型前向为 **+X**
- 航向角 `heading` 为与 +X 轴的夹角，逆时针为正
- 前端旋转：`rotation.y = -heading`

## 4. `metrics.scene` JSON Schema

```jsonc
{
  "ego": {
    "x": 102.5,          // double, 世界坐标 X
    "y": -1.75,          // double, 世界坐标 Z（映射到 Three.js z）
    "heading": 0.05,     // double, 航向角（rad）
    "speed": 8.0,        // double, m/s
    "steer": 0.02        // double, 方向盘转角（rad）
  },

  "lane": {
    "width": 3.5,        // double, 单车道宽度（m）
    "count": 2,          // int,    可行驶车道数
    "center": 0.0        // double, 当前车道中心横向偏移（m），保留字段
  },

  // 旧场景道路几何（无 road_network 时使用）
  "road": {
    "curve_start_x": 0.0,
    "curve_length_m": 200.0,
    "curve_offset_m": -10.0
  },

  // 新场景道路网络（优先使用）
  "road_network": {
    "edges": [
      {
        "id": 0,
        "name": "road_0",
        "nodes": [[0,0], [50,0], [100,0]],  // [[x,y], ...], 参考线中心点，世界坐标
        "lanes": 2,                          // int, 可行驶车道总数
        "lane_width": 3.5,                   // double, 单车道宽（m）
        "length": 100.0                      // double, 参考线长度（m）
      }
    ]
  },

  // 完整实体数组（flowsim_node 发布，世界坐标）
  "entities": [
    { "type": "ego", "id": 0, "x": 102.5, "y": -1.75, "h": 0.05, "spd": 8.0, "steer": 0.02, "throttle": 0.3, "brake": 0.0, "len": 4.6, "wid": 2.0, "vx": 8.0, "vy": 0.0, "tgt": 10.0 },
    { "type": "car",  "id": 1, "x": 120.0, "y": -1.75, "h": 0.0, "spd": 3.0, "len": 4.6, "wid": 2.0, "ai": "follow", "vx": 3.0, "vy": 0.0 },
    { "type": "suv",  "id": 2, "x": 130.0, "y":  1.75, "h": 0.0, "spd": 5.0, "len": 4.8, "wid": 2.0, "ai": "cruise", "vx": 5.0, "vy": 0.0 },
    { "type": "truck","id": 3, "x": 140.0, "y": -1.75, "h": 0.0, "spd": 4.0, "len": 8.0, "wid": 2.5, "ai": "follow", "vx": 4.0, "vy": 0.0 },
    { "type": "pedestrian", "id": 4, "x": 110.0, "y": 3.5, "spd": 1.0, "vx": 0.0, "vy": 1.0, "parked": false },
    { "type": "tl",   "id": 5, "x": 200.0, "y": -5.0, "h": 0.0, "state": "red", "remain_s": 12.3 },
    { "type": "etc_gate", "id": 6, "x": 450.0, "y": 0.0, "h": 0.0, "state": "closed", "progress": 0.0 },
    { "type": "stop_line", "id": 7, "x": 190.0, "y": -1.75, "h": 0.0 }
  ],

  // 障碍物 fallback（vehicle/state，ego-relative，最多 16 个）
  "obstacles": [
    { "id": 0, "type": "car", "x": 10.0, "y": -1.75, "vx": 0.0, "vy": 0.0, "len": 4.6, "wid": 2.0 }
  ],

  // 规划轨迹（Frenet 坐标）
  "trajectory_path": [
    [0.0,  0.0, 8.0],   // [s, d, spd]
    [5.0,  0.1, 8.5],
    [10.0, 0.2, 9.0]
  ],

  // LiDAR 点云（ego-relative）
  "lidar": [
    [10.0, 1.75, 0.2],  // [x, y, z]
    [15.0, -1.75, 0.1]
  ]
}
```

## 5. 字段详细语义

### 5.1 `ego`

| 字段 | 类型 | 单位 | 说明 |
|------|------|------|------|
| `x` | double | m | 世界坐标 X |
| `y` | double | m | 世界坐标 Z（映射到 Three.js z） |
| `heading` | double | rad | 航向角，与 +X 夹角，逆时针为正 |
| `speed` | double | m/s | 纵向速度 |
| `steer` | double | rad | 方向盘转角 |

### 5.2 `road_network`

| 字段 | 类型 | 说明 |
|------|------|------|
| `edges[].id` | int | 道路段 ID |
| `edges[].name` | string | 道路名称 |
| `edges[].nodes` | `[[x,y],...]` | 参考线中心点序列，世界坐标 |
| `edges[].lanes` | int | 可行驶车道总数 |
| `edges[].lane_width` | double | 单车道宽度（m） |
| `edges[].length` | double | 参考线长度（m） |

**渲染约定**：

- `nodes` 是道路参考线（lane 0），前端用 CatmullRomCurve3 平滑插值
- 道路总宽度 = `lanes * lane_width`
- 道路关于参考线对称：左边缘 = +halfWidth，右边缘 = -halfWidth
- 中心双黄线位于参考线两侧 ±0.15m
- 车道分隔虚线位于 ±(i * lane_width)，i = 1, 2, ...
- 道路边缘白实线位于 ±(halfWidth - 0.06m)

### 5.3 `entities`

#### 5.3.1 车辆类（car / suv / truck / ego）

| 字段 | 类型 | 单位 | 说明 |
|------|------|------|------|
| `type` | string | - | `"car"`, `"suv"`, `"truck"`, `"ego"` |
| `id` | int | - | 实体唯一 ID |
| `x` | double | m | 世界坐标 X |
| `y` | double | m | 世界坐标 Z |
| `h` | double | rad | 航向角 |
| `spd` | double | m/s | 速度 |
| `len` | double | m | 车长 |
| `wid` | double | m | 车宽 |
| `vx` | double | m/s | X 方向速度分量 |
| `vy` | double | m/s | Z 方向速度分量 |
| `ai` | string | - | AI 状态：`cruise`, `follow`, `stop`, `stop_for_tl`, `etc_approach`, `branch_sel`, `merge`, `yield` |

**渲染约定**：

- 车辆模型前向为 +X
- 前端旋转：`rotation.y = -h`
- 行人类型无 `h` 字段时，用 `atan2(vy, vx)` 估算朝向

#### 5.3.2 行人（pedestrian）

| 字段 | 类型 | 说明 |
|------|------|------|
| `type` | string | `"pedestrian"` |
| `id` | int | 实体 ID |
| `x` | double | 世界坐标 X |
| `y` | double | 世界坐标 Z |
| `spd` | double | 速度 |
| `vx` | double | X 方向速度 |
| `vy` | double | Z 方向速度 |
| `parked` | bool | 是否静止 |

#### 5.3.3 红绿灯（tl）

| 字段 | 类型 | 说明 |
|------|------|------|
| `type` | string | `"tl"` |
| `id` | int | 实体 ID |
| `x` | double | 世界坐标 X |
| `y` | double | 世界坐标 Z |
| `h` | double | 航向角（rad），灯 arm 垂直于该方向；未配置时前端按道路切线估算 |
| `state` | string | `"green"`, `"yellow"`, `"red"` |
| `remain_s` | double | 剩余时间（s） |

**注意**：v2.0.0 起，红绿灯统一由 `scene.entities` 中的 `tl`（world 坐标）提供，`scene.traffic_lights`（ego-relative）字段已移除。

#### 5.3.4 ETC 门架（etc_gate）

| 字段 | 类型 | 说明 |
|------|------|------|
| `type` | string | `"etc_gate"` |
| `id` | int | 实体 ID |
| `x` | double | 世界坐标 X |
| `y` | double | 世界坐标 Z |
| `h` | double | 航向角（rad），门架 crossbar 垂直于该方向；未配置时前端按道路切线估算 |
| `state` | string | `"closed"`, `"opening"`, `"open"` |
| `progress` | double | 抬杆进度 [0, 1] |

#### 5.3.5 停止线（stop_line）

| 字段 | 类型 | 说明 |
|------|------|------|
| `type` | string | `"stop_line"` |
| `id` | int | 实体 ID |
| `x` | double | 世界坐标 X |
| `y` | double | 世界坐标 Z |

### 5.4 `trajectory_path`

Frenet 坐标数组：`[[s, d, spd], ...]` 或 `[[s, d, spd, edge_id], ...]`（v1.2.0 起第 4 元素可选）

| 元素 | 类型 | 单位 | 说明 |
|------|------|------|------|
| `s` | double | m | 沿参考线的纵向距离（从 ego 当前位置起算） |
| `d` | double | m | 横向偏移，d > 0 在参考线左侧，d < 0 在右侧 |
| `spd` | double | m/s | 该点目标速度 |
| `edge_id` | int? | - | v1.2.0 可选。所在 edge 在 `road_network.edges` 数组中的下标。planning 升级后填充 |

**渲染约定**（v1.2.0 跨 edge 链式投影）：

- 有 `road_network` 时，前端将 ego 投影到最近 edge，沿 curve 前进 s 米，再横向偏移 d
- **s 超出当前 edge 剩余长度时**，按邻接表（端点重合 < 1.5m 判定连接）跳到下一条 edge 继续投影，避免弯道末端 clamp 堆积
- 若某点含 `edge_id`，直接定位到指定 edge 投影（未来 planning 填充后更精确）
- 无 `road_network` 时，退化为沿 ego heading 直线外推

> 注：planning 当前用单段弯道参考线（`road/geometry`），尚未消费 `road_network`，因此暂不输出 `edge_id`。前端跨 edge 链式投影已能消除弯道堆积；待 planning 重构参考线为多 edge 拼接后，可填充 `edge_id` 进一步对齐。

## 6. 兼容性规则

- **向后兼容**：生产者可以增加字段（`additionalProperties: true`），消费者必须忽略未知字段
- **破坏性变更**：删除字段、修改坐标系、修改字段语义 = 必须提升主版本号（如 1.x → 2.0）
- **非破坏性变更**：新增字段 = 提升次版本号（如 1.0 → 1.1）
- **纯渲染改动**：仅修改 `scene3d.js` 的颜色、材质、相机、模型，不触碰 schema = 不提升版本号

## 7. 当前已知的待改进项

以下改进需要修改 schema，因此需要 sim-world / planning 配合：

1. **轨迹参考线**：~~`trajectory_path` 未附带所属 `edge_id` 或参考线，弯道处前端投影可能不准~~ v1.2.0 已修复：前端实现跨 edge 链式投影；planning 输出 `edge_id` 待后续重构参考线后启用
2. **红绿灯来源冗余**：~~`scene.traffic_lights`（ego-relative）与 `scene.entities` 中的 `tl`（world）并存，建议统一为 world 坐标并移除 `scene.traffic_lights`~~ v2.0.0 已修复：移除 `scene.traffic_lights`，红绿灯统一由 `scene.entities` 中的 `tl` 提供
3. **版本号字段**：建议在 `metrics.scene` 顶层增加 `schema_version` 字段，便于前端做兼容性处理

## 8. 责任边界

| 改动类型 | 需要修改的模块 | 是否需要更新本契约 |
|----------|---------------|-------------------|
| 改颜色/材质/相机/模型 | `scene3d.js` | 否 |
| 改字段解释方式（不改 schema） | `scene3d.js` | 否 |
| 新增可选字段 | flowsim / planning / monitor + `scene3d.js` | 是（次版本号） |
| 修改已有字段语义 | flowsim / planning / monitor + `scene3d.js` | 是（主版本号） |
| 删除字段 | flowsim / planning / monitor + `scene3d.js` | 是（主版本号） |
