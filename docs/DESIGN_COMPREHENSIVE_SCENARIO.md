# 综合场景设计方案 — 真 NOA 回归测试

> 这不是视觉演示。这是一个 **NOA（Navigate on Autopilot）全链路回归测试场景**：
> ego 的 planning/control/safety_control 要真实处理每个阶段的驾驶决策。
> 24 个 NPC 是**对手车辆**，不是背景板——每辆车的存在都为了测试 NOA 的某项能力边界。
> 3D 可视化是辅助调试手段，不是产出目标。产出的核心是通过 `demo_evaluator.py` 的回归验证。

## 城区道路 → 信号灯路口 → ETC 收费站 → 匝道选路 → 急弯匝道 → 汇入高速 → 变道超车

---

## 一、场景流程总览

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
| **城区路** | 0–200m | 200m | 3 车道城市道路，有静止/慢行 NPC 车 |
| **信号灯路口** | 200–350m | 150m | 红灯停车等待 15s → 绿灯通过 → 横向来车 NPC |
| **ETC 收费站** | 350–500m | 150m | 减速至 20km/h，模拟 ETC 抬杆通过 |
| **匝道分叉** | 500–650m | 150m | 路牌提示，导航指示走右侧匝道（C 方向） |
| **急弯匝道** | 650–900m | 250m | S 形匝道 + 半径 45m 回头弯，限速 40km/h |
| **加速+汇入** | 900–1400m | 500m | 加速车道 900→1200m 加速至 100km/h，找间隙汇入 |
| **高速超车** | 1400–2200m | 800m | 主路 3 车道，前方慢车 → 左变道超车 → 回到右道 |

---

## 二、当前架构限制分析

| 限制 | 具体问题 | 影响 |
|------|----------|------|
| **单一线形道路** | road_geometry.h 只有单个 smoothstep 弯道 | 无法表示匝道分叉、多段不同曲率的路段 |
| **无分叉/汇入** | 道路是一条线，不支持分支 | 无法模拟两个匝道选路 |
| **无收费站概念** | 无 toll/ETC 相关代码 | 需要新增收费站区域类型 |
| **NPC 全局坐标** | NPC 位置 `(x, y)` 是全局坐标，相对单一直道路中心线 | 分叉后 NPC 无法独立于 ego 所在路段 |
| **Route 只支持变道** | `target_lane: ±1` 只能变道，不能选路 | 无法表达"走右匝道"的导航决策 |
| **红绿灯固定车道** | 红绿灯 `y_lane` 只有一个值 | 无法表示收费站多车道闸门 |
| **可视化单线** | 前端拓扑/D3 只画一条道路中心线 | 分叉后的另一条匝道不会显示 |

---

## 三、核心架构扩展方案

### 3.1 核心思路：RoadGraph（多段道路图）

将当前的"单一直道路"替换为**分段道路图（RoadGraph）**—— 按链接关系排列的等宽路段序列，支持分叉和汇入。

```
RoadGraph
├── Segment[0]: 城区路 (type=urban, 3 lanes, 200m)
├── Segment[1]: 路口段 (type=intersection, 3 lanes, 150m)
│   └── traffic_light[0]: 停止线 x=200 (从 segment 起点算)
├── Segment[2]: ETC 段 (type=toll, 4 lanes, 150m)
│   └── etc_gate[0]: 闸门 x=100 (从起点算)
├── Segment[3]: 分叉段 (type=fork, 2→1 lane, 0m)
│   ├── Branch A: → Segment[4a] 左匝道 (本场景不走)
│   └── Branch B: → Segment[4b] 右匝道 ← 导航选择
├── Segment[4b]: 急弯匝道 (type=ramp_curve, 1 lane, 250m)
│   └── 内含三段曲率：缓和→R45→缓和
├── Segment[5]: 加速车道 (type=merge, 1→3 lanes, 200m)
├── Segment[6]: 高速主路 (type=highway, 3 lanes, 800m)
└── ...
```

#### 坐标系统

每个 `Segment` 定义一条**局部参考线**（arc-length 参数化）：

```
s = 沿参考线的里程 (0 ~ segment.length)
l = 相对参考线的横向偏移 (左负右正, 单位米)

(x, y, heading) = segment_local_to_global(segment_id, s, l)
```

对于 fork/merge 节点，父子段在端点处的 (x, y, heading) 自然连续。**Ego 的位置由 `(current_segment, s, l)` 全局唯一确定**。

#### 关键实现细节

```c
typedef struct {
    int    id;
    char   type[16];     // "urban" | "intersection" | "toll" | "fork" | "ramp_curve" | "merge" | "highway"
    double length_m;     // 段长度（米）
    int    lane_count;   // 车道数
    double lane_width_m; // 车道宽度（默认 3.5）
    double speed_limit;  // 限速（m/s）

    // 几何参数（type-specific）
    struct {
        int    curve_points;     // 曲线分段数（>= 2）
        double radii[16];        // 每段曲率半径（正=右弯, 负=左弯, 0=直）
        double arc_lengths[16];  // 每段弧长（总和 = length_m）
    } geometry;                  // 用于 ramp_curve / merge / highway

    // 分叉/汇入
    struct {
        int    child_count;      // 子段数（fork≥2, merge=1）
        int    child_ids[4];     // 子段 ID
        char   child_labels[4][32]; // 标签: "北向/南向"
    } fork;
    struct {
        int    parent_id;       // 汇入到哪个 segment
        int    target_lane;     // 汇入到第几车道 (0-indexed)
    } merge_info;

    // 区域要素
    struct {
        int    traffic_light_count;
        ScenarioTrafficLight traffic_lights[4];
        int    etc_gate_count;
        ETCGate etc_gates[2];
        int    stop_sign_count;
        StopSign stop_signs[2];
    } zones;

    // 参考线起点/终点的全局坐标（计算缓存, 运行时由 RoadGraph 自动计算）
    double start_x, start_y, start_heading;
    double end_x, end_y, end_heading;
} RoadSegment;
```

### 3.2 ETC 收费站

新增 `ETCGate` 结构：

```c
typedef struct {
    int    id;               // 闸门 ID
    double s;                // 距 segment 起点的里程 (m)
    double l;                // 所在车道的横向偏移
    double gate_width_m;     // 闸门宽度（默认 3.0）
    double pass_speed_mps;   // 通过限速（默认 5.56 = 20km/h）
    double open_delay_s;     // 抬杆延时（默认 1.5s, 模拟 ETC 识别）
} ETCGate;
```

**行为逻辑：**
1. ego 接近闸门 50m 时，sim_world 发布 `road/etc_gates` topic（JSON: `{"gates": [{"id":0,"s":...,"state":"closed/opening/open","open_progress":0.0}]}`）
2. planning_node 订阅，减速至 20km/h
3. ego 到闸门前 10m → 触发抬杆动画（3s 过渡）
4. 杆全开后通过，通过后杆自动放下
5. 闸门前后各有一组 NPC 排队的车

### 3.3 分叉选路

扩展 `ScenarioRouteStep`，新增 `type = "branch"`：

```c
// 新字段（扩展 route 能力）
typedef struct {
    double trigger_x;       // 触发 x （全局坐标）
    int    type;            // 0 = lane_change (旧), 1 = branch_select (新)
    union {
        struct { int target_lane; double target_speed; } lane_change;
        struct { int branch_id; } branch_select;  // 选哪个分支
    };
    char   label[32];
} ScenarioRouteStep;
```

场景 JSON 中的表达：
```json
"route": [
  { "trigger_x": 0, "type": "lane_change", "target_lane": 0, "label": "stay_lane" },
  { "trigger_x": 500, "type": "branch_select", "branch_id": 1, "label": "take_right_ramp" },
  { "trigger_x": 900, "type": "merge", "target_lane": 1, "target_speed": 28, "label": "merge_highway" },
  { "trigger_x": 1600, "type": "lane_change", "target_lane": -1, "label": "overtake_left" },
  { "trigger_x": 1850, "type": "lane_change", "target_lane": 1, "label": "return_right" }
]
```

### 3.4 NPC 多段放置

NPC 的坐标改为 `(segment_id, s, l)` 三元组，支持最大 24 个 NPC（当前 16 上限 → 提升至 24）。

**向后兼容：** 未指定 `segment` 时默认为 0（等价于现有场景，所有 NPC 在第一个段）。

### 3.5 车道扩展

当前 lane_count 固定为 2。场景 JSON 的 `road` 段扩展为每个 segment 指定：

```json
"road_segments": [
  { "id": 0, "type": "urban", "length_m": 200, "lane_count": 3, "speed_limit": 11.11 },
  { "id": 1, "type": "intersection", "length_m": 150, "lane_count": 3, "speed_limit": 11.11,
    "traffic_lights": [{"id": 0, "s": 200, "red_s": 15, "yellow_s": 3, "green_s": 12}] },
  { "id": 2, "type": "toll", "length_m": 150, "lane_count": 4, "speed_limit": 5.56,
    "etc_gates": [{"id": 0, "s": 100, "l": 0, "open_delay_s": 1.5}] },
  { "id": 3, "type": "fork", "length_m": 0, "lane_count": 2,
    "fork": { "children": [{"branch": 0, "segment_id": 10, "label": "北向"},
                            {"branch": 1, "segment_id": 11, "label": "南向"}] } },
  { "id": 4, "type": "ramp_curve", "length_m": 250, "lane_count": 1, "speed_limit": 11.11,
    "curvature_profile": [
      {"radius":  200, "arc": 30},    // 缓和曲线进入
      {"radius":   45, "arc": 130},   // 回头弯（半径 45m，典型高速匝道）
      {"radius": -200, "arc": 40},    // 缓和曲线退出
      {"radius": 1000, "arc": 50}     // 接近直线进入加速车道
    ] },
  { "id": 5, "type": "merge", "length_m": 200, "lane_count": 3,
    "merge": { "parent_id": 6, "target_lane": 0 } },
  { "id": 6, "type": "highway", "length_m": 800, "lane_count": 3, "speed_limit": 27.78 }
]
```

---

## 四、各节点改动范围

### 4.1 scenario_loader.c /.h — 较大改动

- 新增 `SegmentConfig` / `RoadGraphConfig` 结构体
- 解析 `"road_segments": [...]` 数组（保持 `"road": {...}` 旧格式向后兼容：解析为单段 road_segments[0]）
- 新增 `etc_gates`, `fork`, `curvature_profile` 子段解析
- NPC 解析新增 `segment_id`, `s`, `l` 字段（保持旧 `x`,`y` 向后兼容：自动 `segment=0, s=x, l=y`）

### 4.2 sim_world_node.c — 核心改动

| 改动 | 规模 | 说明 |
|------|------|------|
| **RoadGraph 初始化** | ~150 行 | 从 ScenarioConfig 构建 segment 数组，预计算各段参考线 (x, y, heading) 映射表 |
| **全局坐标计算** | ~100 行 | `segment_local_to_global()` + `global_to_segment_local()`，供所有子系统调用 |
| **Ego 里程跟踪** | ~60 行 | 跟踪 ego 的 `current_segment_id` 和 `s`，跨段时自动迁移 |
| **NPC 多段更新** | ~100 行 | NPC 改按 `(segment, s, l)` 更新位置，分叉后另一支 NPC 独立运动 |
| **ETC 闸门逻辑** | ~80 行 | 发布闸门状态，处理抬杆/放杆动画和通过逻辑 |
| **分叉可视化数据** | ~50 行 | 在 `road/geometry` topic 中加入分叉信息，供前端渲染 |
| **vehicle/state 扩展** | ~30 行 | 加入 `segment_id`, `s`, `l` 字段，通知下游当前路段 |
| **向后兼容** | ~80 行 | `segment_count==0` 时退化为现有单段逻辑，旧场景全部照常运行 |

### 4.3 planning_node.cpp — 中等改动

- **路线预处理**：将 route steps 转为基于 segment 的决策序列（branch_select → 选段）
- **Frenet 参考线切换**：进入新 segment 时，Frenet 规划器的参考线自动切换
- **ETC 减速逻辑**：检测前方 ETC 闸门 → 设 `target_speed = 5.56 m/s` → 通过后恢复
- **汇入逻辑**：合并段中，Frenet 虚拟终点设到目标车道中心线，找间隙并入
- **匝道限速自适应**：从 road_segments 读取 `speed_limit` 作为最高速度约束

### 4.4 control_node.cpp — 小改动

- **路段自适应 PID**：不同 segment type 切换 PID 参数（城市路段 kp 低，高速 kp 高）
- **匝道弯道前馈增强**：短半径弯道（R ≤ 60m）时加大 heading 前馈权重，防止冲出车道
- **急弯限速强制覆盖**：急弯匝道段强制 `target_speed ≤ speed_limit`，不让规划器超速

### 4.5 road_geometry.h — 中等改动

- 保留现有 `road_center_y()` / `road_center_heading()` 做向后兼容的退化实现
- 新增多段参考线 API：
  ```c
  // 多段道路坐标映射（segment_id 从 0 开始）
  int road_segment_count(void);
  void road_segment_local_to_global(int seg_id, double s, double l,
                                     double* out_x, double* out_y, double* out_heading);
  void road_global_to_segment_local(double gx, double gy,
                                     int* out_seg_id, double* out_s, double* out_l);
  ```

### 4.6 前端仪表盘 / flowboard 可视化 — 中等改动

- **D3 拓扑图**：RoadGraph 的树形拓扑显示，分叉节点画分支
- **2D 俯视图**：渲染所有 segment 的道路边界和车道线，分叉的两条路都画
- **Ego 位置指示**：在 2D/3D 视图上用 segment 坐标映射到全局坐标

### 4.7 新增：RoadGraph 测试/验证工具

- `tools/roadgraph_visualize.py`：读取场景 JSON 的 `road_segments`，输出 SVG 俯瞰图
- 在 CI 中验证每个 segment 的端点连续性（自动检查端点到子段起点的距离 < 0.1m）

---

## 五、急弯匝道建模（土木工程部分）

> 用户说"建模好点"、"参考现实的高速入口"。这里**确实涉及道路工程**。

### 5.1 匝道几何

典型的城市高架/高速立交匝道设计（参考《公路路线设计规范 JTG D20》）：

| 参数 | 值 | 说明 |
|------|-----|------|
| 设计速度 | 40 km/h (11.11 m/s) | 城市立交匝道典型值 |
| 最小圆曲线半径 | 45 m | 40km/h 对应最小半径 |
| 缓和曲线长度 | 30 m | Ls ≥ V/1.2 ≈ 33m，取 30m |
| 超高 | 4% | 弯道横向坡度（向内侧倾斜） |
| 车道宽度 | 3.0 m | 匝道单车道比主路窄 |
| 停车视距 | 40 m | 保证看到前方障碍物 |

### 5.2 curvature_profile 实现

使用 **回旋曲线（clothoid）** 实现缓和段，用分段等曲率的折线近似：

```
curvature at s:
  s=0:    κ = 0         (直线, 与 toll 段衔接)
  s=0~30: κ 线性从 0→1/45  (缓和曲线, clothoid 近似)
  s=30~160: κ = 1/45    (圆曲线, R=45m, 弧长 130m → 转角 ≈ 165°)
  s=160~200: κ 线性从 1/45→-1/200 (缓和退出, 并预扭向反方向)
  s=200~250: κ ≈ -1/200 → 0 (接近直线)
```

总转角 ≈ 165°（接近回头弯，车辆近乎掉头），这正是高速互通立交中连接两条交叉高速的典型匝道形态。

### 5.3 实景参考

以上海 **G2 京沪高速 → G15 沈海高速 立交** 的北向西匝道为例：
- 半径约 60m
- 经过一座跨线桥（上跨被交路）
- 2 车道的匝道在桥上收缩为 1 车道
- 出口端有加速车道

本场景简化为此类立交的半支（一个方向的匝道），聚焦于从城区上高速的完整流程。

---

## 六、NPC 全景阵容 & 视觉编排

### 6.1 NPC 总表（24 个）

按段分布，确保每帧画面都有看头：

| 编号 | 段 | 模拟场景 | `type` | `(s, l)` | `vx` | 作用 |
|------|-----|----------|--------|-----------|------|------|
| **城区道路（Segment 0, 3车道, 200m）** | | | | | |
| 0 | 0 | 同车道前方慢车 | car | (80, -1.75) | 3.0 | 跟车场景，ego 从后方接近 |
| 1 | 0 | 左车道并行 | car | (85, 0) | 6.0 | 左道有车，不能随意变道 |
| 2 | 0 | 右车道慢车 | car | (40, -3.5) | 4.0 | 右道也有车，展示城市密集交通 |
| 3 | 0 | 路边停车 | car | (170, -5.25) | 0 | 靠边停着的车，增加真实感 |
| 4 | 0 | 人行道行人(a) | pedestrian | (100, -8.0) | 0.8 | 沿人行道行走，vy 方向横穿画面 |
| 5 | 0 | 人行道行人(b) | pedestrian | (40, -8.5) | -0.6 | 反向行走，与行人4交错 |
| **信号灯路口（Segment 1, 3车道, 150m）** | | | | | |
| 6 | 1 | 横向来车（红灯期间） | car | (60, 5.5) | 0→-6.0 | **关键**: 绿灯放行后横向冲出，验证 ego 没闯黄灯 |
| 7 | 1 | 对向直行 | car | (80, 8.0) | -8.0 | 对面车道来车 |
| 8 | 1 | 路口排队车1 | car | (10, -1.75) | 0 | 同车道等红灯的车（ego 后面） |
| 9 | 1 | 路口排队车2 | car | (25, 0) | 0 | 左道等红灯 |
| 10 | 1 | 横穿行人 | pedestrian | (40, 6.0) | -1.2 | 绿灯时行人横穿马路 |
| 11 | 1 | 路边行人 | pedestrian | (20, -8.0) | 0.4 | 站在路边看手机 |
| **ETC 收费站（Segment 2, 4车道, 150m）** | | | | | |
| 12 | 2 | ETC 排队车1（同车道） | car | (30, 0) | 1.5→0 | 前面有辆车也在过 ETC |
| 13 | 2 | ETC 排队车2（邻道） | car | (50, 1.75) | 2.0 | 隔壁 ETC 车道同时过车 |
| 14 | 2 | ETC 排队车3 | car | (20, -1.75) | 1.0 | 左边车道排队 |
| 15 | 2 | ETC 刚通过的车 | car | (110, 0) | 5.0 | 已经过了闸门在加速离开 |
| **匝道分叉（Segment 3, 2→1车道, 10m）** | | | | | |
| 16 | 10 | **左匝道的车** | car | (30, 0) | 7.0 | **视觉重点**: 走左匝道的 NPC，在分叉点与 ego 分道扬镳 |
| **急弯匝道（Segment 11, 1车道, 250m）** | | | | | |
| 17 | 11 | 匝道前方慢车 | car | (80, 0) | 5.0 | 匝道上也有车，ego 跟车过弯 |
| 18 | 11 | 匝道后面来车 | car | (-30, 0) | 9.0 | 后方有更快的车接近（后视镜效应） |
| **加速+汇入（Segment 5+6, 200m）** | | | | | |
| 19 | 6 | 高速主路卡车 | truck | (200, -1.75) | 14.0 | 前方慢车，ego 汇入后要超它 |
| 20 | 6 | 高速左道快车 | car | (250, 0) | 26.0 | 左道有快车，ego 要等它过去才能变道 |
| **高速主路（Segment 6, 3车道, 800m）** | | | | | |
| 21 | 6 | 应急车道停靠 | car | (100, 1.75) | 0 | 应急车道停着一辆双闪车 |
| 22 | 6 | 后方来车 | car | (-50, 0) | 22.0 | 左后方有车接近，限制变道时机 |
| 23 | 6 | 远处车流 | car | (400, -1.75) | 20.0 | 远处车流，增加纵深感 |

**共 24 个 NPC**，保证从场景启动到结束，**画面里永远有东西在动**。

### 6.2 场景电影感编排（按时间轴）

`demo.sh` 跑起来后观众看到的画面节奏：

```
t=0s   ─── 场景启动
        城区道路，ego 前方有一辆慢车，左右道都有车并行。
        路边有人在走，路边停了一辆车。
        → 观众第一印象：「哇，车好多，像真的城市道路。」

t=10s  ─── 接近路口
        信号灯红了（15s 红灯刚开始）。
        ego 前方排了两辆车等红灯。
        横向有车在等，对面有车过来。
        路边有人在看手机。
        → 观众感受到：「它在等红灯，不是傻等——前面有车排队，旁边有人。」

t=25s  ─── 绿灯亮
        红灯转绿灯，排队车启动。
        横向来车在黄灯结束时停在停止线前——ego 没闯。
        行人横穿马路。
        → 关键验证：「系统正确处理了红绿灯 + 行人。」

t=35s  ─── 进入 ETC 区域
        车道变宽（4车道），前方 ETC 闸门。
        前面有车在排队过闸，隔壁车道也在过。
        抬杆动画，ego 减速到 20km/h 通过。
        → 视觉亮点：「ETC 抬杆！排队！细节拉满。」

t=50s  ─── 匝道分叉
        路牌出现，道路分叉。
        左边一辆车走了左匝道（与 ego 分道扬镳）。
        观众看到两条路都有车在走——「分叉是真的，不是贴图。」
        → 最惊艳的瞬间：「两个方向都有车在走！」

t=60s  ─── 急弯匝道
        R=45m 回头弯，匝道仅一车道。
        前面有慢车在过弯，后视镜里有快车接近。
        Ego 稳稳过弯，限速 40km/h 自动执行。
        → 展示：「弯道控制能力 + 限速自适应。」

t=75s  ─── 加速车道汇入
        匝道汇入高速，加速车道 200m。
        Ego 从 40km/h 加速到 100km/h。
        高速主路有卡车（慢）、左道有快车。
        Ego 找间隙并入主路。
        → 展示：「汇入逻辑 + 速度大幅度变化。」

t=90s  ─── 变道超车
        高速上前方有卡车（14m/s），ego 跟车一段。
        左道快车超过后，ego 左变道。
        加速超车，再右变道回原车道。
        应急车道还有一辆停着的双闪车。
        → 收尾：「完整的 NOA 变道超车流程。」

t=110s ─── 场景收尾
        Ego 在高速最右道稳定巡航。
        远处还有车流。
        → 余韵。
```

### 6.3 场景画面密度分析

| 阶段 | 可见 NPC 数 | 画面亮点 |
|------|------------|----------|
| 城区路 | 6 | 3车道都有车 + 行人交错 |
| 路口 | 6+3（来自城区段仍在视野内） | 红绿灯 + 排队 + 行人横穿 |
| ETC | 4+ | 多车道排队 + 抬杆 |
| 分叉 | 2+（左匝道的车是关键） | **双路分流** |
| 匝道 | 2~3 | 前后车 + 弯道 |
| 加速 | 3+高速NPC | 速度骤变 + 汇入 |
| 高速 | 5+ | 多车道高速车流 + 超车 |

**全程平均可见 NPC 数 ≥ 4**，画面从不会冷场。

### 6.4 NPC 防碰撞：不能撞车

> **当前 NPC 是"刚体运动"**：`obstacles_tick()` 里每个 NPC 只是 `x += vx·dt`，对同车道的前后车完全无感知。6 个 NPC 在 3 车道城区路段上跑还勉强可控，24 个 NPC 在 ETC 排队/路口等候/匝道跟车等密集场景下，**必撞**。

解决方案：为 NPC 引入**简化 IDM（Intelligent Driver Model）** 跟车模型 + 红绿灯感知。

#### 6.4.1 SimObstacle 新增字段

```c
typedef struct {
    // ... 现有字段 ...
    double  target_vx;         // 目标速度（来自场景配置，即旧 vx）
    double  max_accel;         // 最大加速度（m/s²，默认 2.0）
    double  max_brake;         // 最大减速度（m/s²，默认 4.0）
    int     obey_traffic;      // 是否响应红绿灯/ETC（默认 1=是）
} SimObstacle;
```

#### 6.4.2 跟车模型（IDM 简化版）

每个 NPC tick 的计算流程：

```
npc_tick(i):
  1. 找最近的前车（同车道，±2m 横向容差，在前方 0~200m 内）
  2. 找最近的红灯停止线（同车道 ±2m，在前方 0~200m 内）
  3. 计算期望速度 v_desired：
     最低优先级的限制胜出
     a. 限速（segment speed_limit）→ v_limit
     b. ETC 闸门减速（距闸门 < 50m 时，目标速度 pass_speed_mps）→ v_gate
     c. 跟车间距 → v_gap (IDM 公式)
     d. 红绿灯（距红灯 < 50m 时，目标速度 0）→ v_tl
     v_desired = min(v_limit, v_gate, v_gap, v_tl)
  4. 调整实际速度：
     dv = v_desired - current_vx
     if dv > 0: vx += min(dv, max_accel * dt)     // 加速
     if dv < 0: vx -= min(-dv, max_brake * dt)     // 减速
  5. 位置更新：x += vx * dt
```

**IDM 间距公式**（简化版，去掉加速度项）：
```
safe_gap = min_gap + vx * reaction_time
gap_error = actual_gap - safe_gap
if gap_error > 0:
    v_gap = min(vx + max_accel * dt, target_vx)   // 巡航
else:
    // 减速系数随 gap 缩小指数增加
    brake_factor = exp(-gap_error / 2.0)           // gap_error 越负，brake_factor 越大
    v_gap = max(0, vx - max_brake * brake_factor * dt)
```

参数默认值：
| 参数 | 值 | 说明 |
|------|-----|------|
| `min_gap` | 3.0 m | 停车时的最小间距 |
| `reaction_time` | 1.0 s | 跟车时距（速度相关的安全间距） |
| `max_accel` | 2.0 m/s² | 普通加速（约 0.2g） |
| `max_brake` | 4.0 m/s² | 正常刹车（约 0.4g） |
| `emergency_brake` | 8.0 m/s² | 紧急刹车（约 0.8g，距前车 < 2m 触发） |

#### 6.4.3 NPC 与 ego 的交互

NPC 对 ego 也是"同车道前车"——NPC 感知障碍物列表里包含 ego。反过来，ego 的 `lead_gap()` 也正常感知 NPC。

这意味着：
- **NPC 0（前方慢车 vx=3.0）**：ego 从后面接近时，NPC 0 不会因为 ego 贴上来就加速——它保持自己的 `target_vx=3.0`。但 NPC 0 **不会撞上更前面的车**（比如红灯前排队的 NPC 8/9），会自动刹车排队。
- **汇入场景**：高速上的 NPC 19/20/22 保持自己的巡航速度，不会避让正在汇入的 ego——这正好是 realistic 的"汇入找间隙"场景，而不是 NPC 主动让行。

行为模式矩阵：

| NPC 类型 | 对前车 | 对红灯 | 对 ETC 闸门 | 对行人 |
|----------|--------|--------|------------|--------|
| `car` / `truck` | ✅ 跟车减速 | ✅ 停车 | ✅ 减速排队 | ❌（可后续） |
| `pedestrian` | ❌（走人行道） | ❌ | ❌ | ❌ |
| 静止 NPC（vx=0） | ❌（已跳过更新） | ❌ | ❌ | ❌ |

#### 6.4.4 NPC 表修正

vx 列的含义从"硬速度"变为"目标速度 target_vx"。在密集场景中，NPC 的实际速度由跟车模型实时决定：

| 编号 | 段 | target_vx | 预期实际行为 |
|------|----|-----------|-------------|
| 0 | 0 | 3.0 | 城区路巡航 ~3m/s；接近路口红灯时减速到 0 排队 |
| 1 | 0 | 6.0 | 左车道自由行驶 ~6m/s（前方无车） |
| 2 | 0 | 4.0 | 右车道巡航 ~4m/s |
| 3 | 0 | 0 | 静止（跳过更新） |
| 4,5 | 0 | 0.8/-0.6 | 行人，不在车道内 |
| 6 | 1 | 0→-6.0 | 横向来车，y 方向运动，不在同车道 |
| 7 | 1 | -8.0 | 对向，不在同车道 |
| 8 | 1 | 0 | 路口等红灯，停在停止线前 |
| 9 | 1 | 0 | 路口等红灯 |
| 10 | 1 | -1.2 | 行人横穿 |
| 11 | 1 | 0.4 | 路边行人 |
| 12 | 2 | 1.5 | ETC 排队，受前车和闸门双重影响 → 自然停在闸门前 |
| 13,14 | 2 | 2.0/1.0 | 邻道 ETC 排队 |
| 15 | 2 | 5.0 | 刚过闸门，正在加速离开 |
| 16 | 10 | 7.0 | **左匝道**，完全独立于 ego 所在段，不受影响 |
| 17 | 11 | 5.0 | 匝道跟车，前面可能有更慢的车 |
| 18 | 11 | 9.0 | 匝道后方来车，接近时自动减速跟车 |
| 19 | 6 | 14.0 | 高速巡航 14m/s，前方无车不减速 |
| 20 | 6 | 26.0 | 左道快车 |
| 21 | 6 | 0 | 静止双闪车（跳过更新） |
| 22 | 6 | 22.0 | 后方来车 |
| 23 | 6 | 20.0 | 远处车流 |

#### 6.4.5 对 sim_world 的扩展要求

| 能力 | 规模 | 说明 |
|------|------|------|
| **SimObstacle 新字段** | ~10 行 | `target_vx`, `max_accel`, `max_brake`, `obey_traffic` |
| **IDM 跟车函数** | ~60 行 | `npc_car_following(i)`: 找前车 + 算间距 + 调速度 |
| **红绿灯感知** | ~30 行 | NPC 检测同车道红灯停止线，自动减速停车 |
| **ETC 闸门感知** | ~20 行 | NPC 检测前方闸门（closed/opening），排队等待 |
| **紧急刹车** | ~15 行 | `gap < 2m` 时触发 max_brake×2，防止碰撞残留 |
| **行人避让** | ~30 行 | 可选 Phase 2：同车道前方有行人横穿时减速 |
| **向后兼容** | ~5 行 | 旧场景 NPC 没有 target_vx→`vx = target_vx`，`obey_traffic=0` |

**新增代码总量约 120~150 行**，全部在 `sim_world_node.c` 内。旧场景 NPC（`obey_traffic=0`，无 target_vx）走原路径，零行为变化。

---

## 八、实施路径

> 完整场景 JSON 示例（road_segments + 24 NPC）已在三、五、六节中覆盖，此处不再重复。

### Phase 1 — 基础：RoadGraph 核心（~3 天）

| 步骤 | 文件 | 产出 |
|------|------|------|
| 1.1 定义 RoadSegment 结构体 | `include/road_geometry.h` | 数据结构 + API 声明 |
| 1.2 实现坐标映射 | `src/core/road_graph.c` | `local_to_global()`, `global_to_local()` |
| 1.3 扩展 scenario_loader | `src/core/scenario_loader.c` | 解析 `road_segments` 数组（旧格式兼容） |
| 1.4 测试工具 | `tools/roadgraph_visualize.py` | SVG 俯瞰图输出 |

### Phase 2 — 核心：sim_world 多段支持（~4 天）

| 步骤 | 文件 | 产出 |
|------|------|------|
| 2.1 Ego 多段跟踪 | `sim_world_node.c` | `current_segment_id`, `s` 里程更新，跨段迁移 |
| 2.2 NPC 多段更新 | `sim_world_node.c` | NPC 按 `(segment, s, l)` 更新、回收 |
| 2.3 Road/geometry topic 扩展 | `sim_world_node.c` | 发布当前段 + 前方段的信息 |
| 2.4 ETC 闸门逻辑 | `sim_world_node.c` | 抬杆/放杆状态机 |

### Phase 3 — 规划与控制适配（~3 天）

| 步骤 | 文件 | 产出 |
|------|------|------|
| 3.1 Route 扩展 branch_select | `planning_node.cpp` | `route type=branch` 解析 + 段选择 |
| 3.2 Frenet 段切换 | `planning_node.cpp` | 跨段时参考线自动切换 |
| 3.3 ETC 减速 + 汇入逻辑 | `planning_node.cpp` | 减速至限速 → 加速找间隙汇入 |
| 3.4 弯道自适应控制 | `control_node.cpp` | R≤60m 前馈增强、限速强制覆盖 |

### Phase 4 — 场景与验证（~2 天）

| 步骤 | 产出 |
|------|------|
| 4.1 编写 `city_to_highway_full.json` | 完整场景文件 |
| 4.2 编写 `pipeline_full_scenario.json` | 专用 pipeline（Frenet ON, lane_detection 等全开） |
| 4.3 预跑测试 | 逐个阶段手动验证 |
| 4.4 回归验证 `demo_evaluator.py` | 确保旧场景全部通过 |

### Phase 5 — 可视化增强（~2 天）

| 步骤 | 文件 | 产出 |
|------|------|------|
| 5.1 前端 RoadGraph 渲染 | `tools/flowboard/index.html` | 分叉道路 + 当前段高亮 |
| 5.2 ETC 动画 | `tools/flowboard/index.html` | 闸门抬杆 CSS 动画 |
| 5.3 交通灯 3D 显示 | `foxglove_bridge.py` | 红绿灯状态映射到 3D 场景 |

**总计：约 14 天（一个人全力投入）**

---

## 九、向后兼容方案

| 旧特性 | 兼容方式 |
|--------|----------|
| `"road": { curve_* }` 单段弯道 | 解析为 road_segments[0] `{type: "curve", length_m: curve_length, geometry: ...}` |
| `"actors"[].x/y` 全局坐标 | 解析为 `segment=0, s=x, l=y` |
| `"route"[].target_lane` 旧格式 | `type` 缺省时默认 `lane_change` |
| 无 `traffic_lights` 的场景 | road_segments[0] 不设 traffic_lights，planning 照旧 |
| 旧 pipeline.json 不传 scenario_file | sim_world 内默认场景退化到与现有完全一致 |

**所有旧场景文件零改动、零回归。**
