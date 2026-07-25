# Sim Digest — 仿真基础层 invariant 与调试可视化

> 模块: `modules/adas_nodes/flowsim/sim_digest.{h,cpp}`
> 调用方: `flowsim_node.cpp`（init 阶段 + 每帧主循环）
> 目的: 把"眼睛会检查的位置关系"编码为数值断言，错一个符号即 FAIL；
>       附带调试可视化（ASCII 俯视）与回归基线（golden 快照）。

## 1. 数据模型

### 1.1 静态场景 digest — `StaticDigest`

几何变更时构建一次（`flowsim_node.cpp:1410` 在 esmini 网络加载后调用）。

| 字段 | 类型 | 说明 |
|------|------|------|
| `lanes` | `vector<LaneDigest>` | 每条车道的中线采样 + 宽度 + 边界类型 |
| `markings` | `vector<RoadMarkingDigest>` | 路面标线（虚线/实线/双黄） |
| `traffic_lights` | `vector<TrafficLightDigest>` | 红绿灯位置与受控车道 |
| `drivable_poly_x/y` | `vector<double>` | 可行驶区多边形外扩（外沿 +20m 留白） |
| `road_half_width` | `double` | 全局路面半宽（取最大车道数 × 3.5 × 0.5） |

**LaneDigest**:
```cpp
struct LaneDigest {
    int    id, road_id, lane_id;
    std::vector<double> centerline_x, centerline_y;  // 沿 s 每 10m 采样
    double width;                    // 车道宽（标准 3.5m）
    int    left_boundary_type;       // 0=虚线 1=实线 2=双黄
    int    direction;                // +1 正向 / -1 反向
};
```

**TrafficLightDigest**:
```cpp
struct TrafficLightDigest {
    int    id;
    double x, y;          // 灯杆世界坐标（ENU）— ASCII 渲染定位
    double heading;       // 灯杆朝向（rad）— ASCII + invariant 检查
    int    controlled_road_id, controlled_lane_id;
};
```

> **P2 清理 + 闭环调试恢复**：`x / y / heading` 曾在 P2 清理时被移除（认为"无
> 读取者"），但 ASCII 俯视渲染需要位置才能在网格上画灯，导致闭环调试"灯杆落在
> 路面外/不与车辆重叠"问题无从下手。现已恢复 `x / y / heading`（位置静态，
> build_static_digest 一次性填）；动态相位走 `DynamicDigest::traffic_light_states`
> 每帧刷新。仍移除 `z`（灯杆高度，对俯视渲染无意义）和把 `phase` 改名
> `phase_state` 放到 `TrafficLightStateDigest`，避免静态/动态字段混在一个 struct。

### 1.2 红绿灯动态相位 — `TrafficLightStateDigest`

每帧由 `build_dynamic_digest` 从 `Entity::phase_state` 提取，与
`StaticDigest::traffic_lights` 按 id 关联供 `render_ascii_overhead` 画 G/Y/R 字符。

```cpp
struct TrafficLightStateDigest {
    int    id;
    int    phase_state;   // 0=绿 1=黄 2=红（与 Entity::phase_state 一致）
    double phase_timer;   // 当前相位剩余时间 (s)
};
```

### 1.3 动态演员 digest — `DynamicDigest`

每帧构建（digest 块内，每 20 帧 ≈ 1s）。

```cpp
struct ActorDigest {
    int    id, type;              // type: 0=ego 1=car 2=suv 3=truck 4=pedestrian
    double pos[3];                // 世界坐标 [x,y,z]
    double bbox[3];               // 尺寸 [L,W,H]
    double heading;               // 朝向 (rad)
    double vel[2];                // [vx, vy]
    double speed;
    int    road_id, lane_id;
    double lateral_offset;        // 横向偏移
    double s;                     // 沿路里程
    double rotation_y;            // 给 THREE 的 rotation.y
    int    route_dir;             // +1 顺行 / -1 对向 / 0 未上 route
    uint32_t last_teleport_cycle; // 显式传送标记（choreography/recycle_npc）
};

struct DynamicDigest {
    double sim_time;
    int    frame;
    bool   ego_centered;
    double origin[2];
    std::vector<ActorDigest> actors;
    std::vector<TrafficLightStateDigest> traffic_light_states;  // 红绿灯相位（每帧）
};
```

> **P2 清理**：移除了 `ActorDigest::yaw_rate / accel / ai_state` —
> yaw_rate/accel 仅 `build_dynamic_digest` 写入（恒为 0），invariant 检查用
> `(ca.heading - pa->heading) / dt` 本地计算；ai_state 由 `scene_pub.cpp::npc_state_str()`
> 独立发布到前端，与 digest 无关。

## 2. Invariant 检查

`flowsim_node.cpp:1243` 每 20 帧跑一次完整 invariant，避免每帧序列化开销过大。
失败详情写入 stderr 供 `demo_evaluator.py` 捕获。

### 2.1 静态 invariant — `check_static_invariants(sd)`

| # | 检查 | 容差 |
|---|------|------|
| 1 | 车道宽 ∈ [2.5, 4.0]m | 硬约束 |
| 2 | 边界类型自洽（同向分隔=虚线、外沿=实线、对向=双黄） | 枚举匹配 |
| 3 | 虚线段长 ~3m，间距 ~6–9m | ±10% |
| 4 | 可行驶区多边形闭合、不自交 | 几何检查 |
| 5 | 红绿灯朝向 · 车道方向 < 0（面向来车） | 点积符号 |
| 6 | 没有物体堆在 (0,0,0) | 坐标非零 |
| 7 | 每条 lane 中线落在可行驶多边形内 | 几何包含 |

> P2 清理：原检查 5"路面高程连续"已删除 — 其依赖的 `height_samples_z` 从未被填充，
> 该 if 块恒不执行属死分支。

### 2.2 单帧空间 invariant — `check_spatial_invariants(dd, sd, roads)`

| # | 检查 | 容差 |
|---|------|------|
| 1 | `|z − roadHeight(x,y)| < ε` | 浮空/埋地 | ε=0.5m |
| 2 | `|lateral_offset| ≤ 半路宽 + 裕量` | 飞出路面 | ε=1.0m |
| 3 | `rotationY == headingToRotationY(heading)` | ENU→THREE 符号翻错 | 精确 |
| 4 | `0 ≤ speed ≤ 1.5×限速` | 超速/呆滞 | 限速=22 m/s |
| 5 | `bbox ≈ 标准尺寸` | 尺度错 | Car=[4.5,1.8,1.5] Ped=[0.5,0.5,1.7] |
| 6 | 两 actor bbox 不重叠 | 穿模/重叠 | OBB SAT |

### 2.3 运动方向 invariant — `check_motion_direction(dd, sd, roads)`

| # | 检查 | 阈值 |
|---|------|------|
| 1 | `dot(forward(heading), vel/|vel|) > cos(30°)` | 车头≈前进方向 |
| 2 | `dot(forward(heading), lane_dir) > cos(45°)` | 与车道方向一致 |
| 3 | `sign(lane 允许方向) == sign(沿 s 前进)` | 不逆行 |

### 2.4 时序 invariant — `check_temporal_invariants(prev, curr, dt)`

需要连续两帧，从第 2 帧开始（`flowsim_node.cpp:1282` 在 cycle=1 初始化 prev）。

| # | 检查 | 容差 |
|---|------|------|
| 1 | `Δpos ≈ vel × dt` | 不瞬移 | ε=1.0m |
| 2 | `|Δpos| ≤ v_max × dt` | 不超速瞬移 | v_max=50 m/s |
| 3 | `|Δheading| ≤ yaw_max × dt` | 朝向不瞬变 | yaw_max=π rad/s |
| 4 | `accel ∈ [−8, +4] m/s²` | 运动学可行 | 硬约束 |

**瞬移豁免**：`ActorDigest::last_teleport_cycle` 标记 choreography beat 或
`recycle_npc` 传送的 cycle，时序检查跳过本次采样间隔的 Δpos 检查（设计内瞬移，非 bug）。

## 3. 调试可视化

### 3.1 ASCII 俯视渲染 — `render_ascii_overhead(sd, dd, w=80, h=40)`

终端调试用，无需 GUI。3D pipeline 运行时 `flowsim_node.cpp:1284` 自动调用，
每秒覆盖写入 `/tmp/flow_ascii_overhead.txt`。

**渲染规则**：
- 等比缩放保持纵横比，沿车道中心线 + 红绿灯位置扫描极值 + 20m 留白
  （灯杆落在路面外侧 7m+，不纳入扫描会被裁出画面）
- 车道线：`-` 普通车道，`#` 双黄线
- 红绿灯：`G` 绿 `Y` 黄 `R` 红（位置取 `sd.traffic_lights[i].(x,y)`，
  相位取 `dd.traffic_light_states` 按 id 关联）
- 演员标记：
  - `E` = ego
  - `*` = pedestrian
  - `> < ^ v` = 车辆朝向（cos/sin > 0.7 主方向）
  - `7 L J \` = 对角朝向
- 绘制顺序：车道线 → 红绿灯 → 演员（车辆字符会覆盖同格灯字符，看到 `E`/`C`
  覆盖 `G/Y/R` 即说明灯杆落在路面内与车重叠，是闭环调试要抓的 bug）
- Unicode 边框 `┌─┐│└─┘` + 图例 + frame/time 信息

**示例输出**（`straight_road.json` frame 80, time 3.95s）：
```
┌──────────────────────────────────────────────────────────────────────────────┐
│ E>>*-<------------------------------------------------------------------------ │
│                                                                                │
│ ...                                                                             │
└──────────────────────────────────────────────────────────────────────────────┘
E=ego C=car *=pedestrian ><^v=朝向 -=车道线 #=双黄 G/Y/R=灯(绿黄红)
frame:80 time:3.950000
```

> **闭环调试恢复**：原 P2 清理移除了红绿灯 RYG 标记（连带 `TrafficLightDigest::x/y/phase`
> 字段），导致"灯杆落在路面内与车重叠"问题在 ASCII 视图里看不到。现已恢复
> （`x/y/heading` 回到 `TrafficLightDigest`，相位走 `TrafficLightStateDigest`），
> 可直接 `cat /tmp/flow_ascii_overhead.txt` 看灯杆与车辆相对位置。

### 3.2 查看方式

```bash
# 3D pipeline 运行时
cat /tmp/flow_ascii_overhead.txt
watch -n 1 cat /tmp/flow_ascii_overhead.txt   # 实时刷新
```

## 4. Golden 快照（回归基线）

### 4.1 `golden_snapshot(dd)`

按 actor id 排序后生成 JSON：
```json
{
  "frame":80,
  "sim_time":3.95,
  "actors":[
    {"name":"actor_0","pos":[102.5,-1.75,0.0],"rotY":0.05,"scale":[4.60,2.00,1.50]},
    {"name":"actor_1","pos":[120.0,-1.75,0.0],"rotY":0.00,"scale":[4.60,2.00,1.50]}
  ]
}
```

可 commit 到仓库 `tests/golden/` 作为基准参考。

### 4.2 `golden_diff(golden, current, tolerance=0.01)`

简易逐行 diff（不引入完整 JSON 解析器）：
- 完全相同 → 返回空字符串（PASS）
- 行不同 → 提取行内所有数值，按容差比较
  - 数值差异在容差内 → 跳过
  - 超出容差或结构不同 → 记录 `L{n} golden: ... / current: ...`

**用途**：任何 actor 的 pos/rotY/scale 偏差超过 tolerance 即 FAIL。
只有几何合法变更时才需更新 golden（PR 附截图）。

## 5. 调用流程

```cpp
// flowsim_node.cpp init 阶段（line 1410）
g.static_digest = flowsim::build_static_digest(g.roads, g.route, g.pool);
auto static_inv = flowsim::check_static_invariants(g.static_digest);
if (static_inv.failed > 0) { /* LOG_WARN + stderr */ }

// flowsim_node.cpp 主循环（line 1243，每 20 帧）
if (g.cycle % 20 == 0 && g.digest_initialized) {
    auto dd = flowsim::build_dynamic_digest(g.pool, sim_time_s, g.cycle, true);
    // 单帧 invariant
    flowsim::check_spatial_invariants(dd, g.static_digest, &g.roads);
    flowsim::check_motion_direction(dd, g.static_digest, &g.roads);
    // 时序 invariant（需 prev）
    if (g.prev_dynamic_digest.actors.size() > 0) {
        flowsim::check_temporal_invariants(g.prev_dynamic_digest, dd, DT * 20);
    }
    // ASCII 俯视自动写入 /tmp/flow_ascii_overhead.txt
    auto ascii = flowsim::render_ascii_overhead(g.static_digest, dd, 80, 40);
    /* fopen + fputs + fclose */
    g.prev_dynamic_digest = std::move(dd);
} else if (!g.digest_initialized && g.cycle == 1) {
    g.prev_dynamic_digest = flowsim::build_dynamic_digest(g.pool, sim_time_s, 0, true);
    g.digest_initialized = true;
}
```

## 6. 相关文件

| 文件 | 角色 |
|------|------|
| `modules/adas_nodes/flowsim/sim_digest.h` | 公开 API：结构体 + 函数声明 |
| `modules/adas_nodes/flowsim/sim_digest.cpp` | 实现：digest 构建 + invariant + ASCII + golden |
| `modules/adas_nodes/flowsim_node.cpp` | 调用方：init 构建 static_digest，主循环每 20 帧跑 invariant + 写 ASCII |
| `tools/demo_evaluator.py` | 解析 stderr 的 invariant 失败详情 |
