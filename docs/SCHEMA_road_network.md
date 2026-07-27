# road_network JSON Schema

> 本文件是 `scene/frame` topic 中 `road_network` 字段的**唯一事实来源**。
> 任何修改 `road_network` 序列化格式的行为都应同步更新本文件，并提升版本号。
>
> 模块: `modules/adas_nodes/flowsim/scene_pub.cpp`
> 调用方: `flowsim_node.cpp:publish_scene_frame` → 20Hz 发布到 `scene/frame`
> 消费方: flowboard/vis 模块树（经 `monitor_node` 融合后渲染）

## 1. 版本

- **版本**: 1.1.0
- **生效日期**: 2026-07-27
- **变更**:
  - v1.0.0 初版：`edges[]` 包含 `id/name/nodes/lanes/lane_width/length`（仅 esmini 模式有 `type`）
  - v1.1.0 **统一 schema**（C-5）：
    - legacy 模式 nodes 由 `[x,y]` 升级为 `[x,y,z]`（z 恒为 0），与 esmini 模式对齐
    - legacy 模式 edge 显式标注 `type="road"`，前端无需对缺失字段做特判
- **维护者**: flowsim / flowboard 共同维护

## 2. 两种构建模式

`build_road_network_json` 根据是否加载 esmini RoadManager 分两种模式，**输出 schema 完全一致**：

| 模式 | 触发条件 | nodes 来源 | 备注 |
|------|----------|------------|------|
| esmini | `cfg.roads->loaded() && road_count() > 0` | `frenet_to_world(road_id, lane_id=0, s, offset=0)` 采样 N 点 | 每条 road 一个 edge，z 为真实 elevation |
| legacy | esmini 未加载 | `road_geometry.h:road_center_y(x)` 计算 | 单条 edge（`id=0, name="legacy_road"`），z 恒为 0 |

**性能**：`build_road_network_json` 在 `cfg.cached_road_network_json` 为空时构建一次，
后续帧用 `cJSON_AddRawToObject` 复用字符串（道路网络在仿真过程中不变）。

## 3. 完整 Schema

```jsonc
{
  "edges": [
    {
      "id": 0,                          // int, 道路段 ID（esmini=OpenDRIVE road id；legacy=0）
      "name": "road_0",                 // string, 道路名称（esmini=str_id；legacy="legacy_road"）
      "nodes": [                        // array of [x,y,z]，参考线中心点序列
        [0.0, 0.0, 0.0],
        [25.0, 0.0, 0.0],
        [50.0, 0.0, 0.0]
      ],
      "lanes": 2,                       // int, 可行驶车道总数（双向合计）
      "lane_width": 3.5,                // double, 单车道宽度（m）
      "oneway": false,                  // bool, 是否单行道（ramp 类道路为 true）
      "length": 200.0,                  // double, 参考线长度（m）
      "type": "road"                    // string, 道路类型（见 §4）
    }
  ]
}
```

### 3.1 nodes 字段

每个 node 是一个 3 元素数组 `[x, y, z]`，**世界坐标（ENU）**：

| 元素 | 类型 | 单位 | 说明 |
|------|------|------|------|
| `x` | double | m | 东向坐标（道路参考线切线方向） |
| `y` | double | m | 北向坐标 |
| `z` | double | m | 高程（elevation），平地场景恒为 0 |

**采样约定**：
- esmini 模式：每条 road 沿参考线（lane_id=0, offset=0）等距采样 `ROAD_NODES_PER_EDGE=8` 个点
- legacy 模式：直道采 2 点（起点+终点），弯道采 `LEGACY_ROAD_NODES=8` 点
- 前端用 `CatmullRomCurve3` 平滑插值，8 点对直道/弯道都足够

**向后兼容**：前端读取 `nodes[ni][2]` 时若需兼容旧版 `[x,y]` 二元组，可用
`nodes[ni][2] || 0` 兜底。v1.1.0 起所有 node 都有 z 元素，兜底逻辑可逐步移除。

### 3.2 lanes / lane_width

| 字段 | 类型 | 说明 |
|------|------|------|
| `lanes` | int | 可行驶车道总数（双向合计，esmini/OpenDRIVE 真实车道数） |
| `lane_width` | double | 单车道宽度（m），esmini 模式查第一条行驶车道实际宽度，失败时退回 `cfg.lane_width`（默认 3.5） |

**渲染约定**：
- 道路总宽度 = `lanes * lane_width`
- 道路关于参考线对称：左边缘 = +halfWidth，右边缘 = -halfWidth
- `halfWidth = lanes * lane_width / 2`

### 3.3 oneway

| 字段 | 类型 | 说明 |
|------|------|------|
| `oneway` | bool | 单行道标记，ramp 类道路为 true，其余默认 false（双向对称） |

### 3.4 length

| 字段 | 类型 | 单位 | 说明 |
|------|------|------|------|
| `length` | double | m | 参考线长度（esmini 模式来自 `RoadInfo.length`；legacy 模式 = `max(curve_start_x + curve_length_m, 200.0)`） |

## 4. type 字段（道路类型）

`type` 字段标识道路几何/功能类型，用于前端选择渲染样式（颜色、车道线密度等）。
**所有 edge（esmini + legacy）必填**（C-5 起统一）。

### 4.1 type 取值

| 取值 | 来源 | 含义 |
|------|------|------|
| `"road"` | 默认 / legacy 兜底 | 普通道路（无特殊标识） |
| `"ramp_curve"` | esmini name 含 `"ramp"` | 匝道（曲线，单向） |
| `"viaduct_highway"` | esmini name 含 `"viaduct"` | 高架桥高速公路 |
| `"urban"` | esmini name 含 `"urban"` | 城市道路 |
| `"cross_road"` | esmini name 含 `"cross"` | 十字路口段 |

### 4.2 推断规则（esmini 模式）

- 第一条 edge（`i==0`）若 `cfg.road_type` 非空，直接用 `cfg.road_type`（来自场景配置）
- 其余 edge 根据 `info.str_id` 子串匹配推断（见上表）
- 都不匹配时默认 `"road"`

### 4.3 legacy 模式

固定 `"road"`（legacy 道路无类型信息）。

## 5. 渲染约定

前端 `flowboard/vis/director/SceneDirector.js` 用 `edges[]` 构建 `CatmullRomCurve3`：

1. 每个 edge 用 `nodes` 数组构建一条样条曲线（参考线）
2. 沿曲线左右各外扩 `halfWidth` 构建道路边缘 mesh
3. 根据 `lanes` 数画车道分隔线（同向虚线，对向双黄）
4. 根据 `type` 选择材质/颜色（如 `ramp_curve` 用浅灰，`urban` 用深灰等）
5. `oneway=true` 时只画单侧车道线

## 6. 坐标系

`nodes` 中的 `x/y/z` 是 **世界坐标（ENU）**：

- **X 轴**: 东向（道路参考线切线方向）
- **Y 轴**: 北向
- **Z 轴**: 上（高程，平地为 0）

> ⚠️ 注意：`nodes` 是 `[x, y, z]` 而非 entities 中常见的 `[x, y]`。前端渲染时
> 把 `(x, y)` 映射到 Three.js 的 `(x, z)`，`z` 元素映射到 Three.js 的 `y`（高度）。
> 平地场景 `z` 恒为 0，Three.js 中所有点都在 `y=0` 平面。

## 7. 与 FlowBoard Scene Contract 的关系

- `docs/FLOWBOARD_SCENE_CONTRACT.md` 描述 `metrics.scene` 整体 schema（含 `road_network`）
- 本文件是 `road_network` 字段的**详细规范**，包括字段语义、采样约定、类型枚举
- 任何修改 `road_network` 序列化的 PR 都应同步更新本文件，并在 §1 提升版本号
