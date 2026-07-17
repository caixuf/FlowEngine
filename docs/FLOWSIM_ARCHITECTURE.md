# FlowSim Engine v2 — sim_world 重构设计方案

> 目标：让 sim_world 从"一个 1100 行的 C 文件"变成**真正可用的仿真引擎**。
> 对标：不是 3A 大作，但至少不能比 GTA Vice City（2002）的道路和车辆效果差。
> 约束：C++20 + flowcoro，link esmini RoadManager 处理道路网络，Three.js 做 3D 渲染。
> 原则：**能用开源轮子就不自己写**，架构钱花在刀刃上（NPC AI + 场景驱动 + 确定性）。

---

## 零、核心理念：只写开源没有的东西

现有开源项目已经覆盖的大块：

| 模块 | 开源方案 | 我们省掉的代码 |
|------|----------|---------------|
| 道路网络解析 | esmini RoadManager | ~1500 行 |
| Frenet↔World 坐标 | esmini RoadManager | ~300 行 |
| 车道查询/邻道/限速 | esmini RoadManager | ~400 行 |
| 3D 道路渲染 | Three.js 开源 OpenDRIVE viewer | ~600 行 |
| 车辆 glTF 模型 | Blender 导出 + Three.js GLTF loader | ~0 行（建模） |
| 车身油漆/光照 | Three.js 标准材质 | ~0 行（配置） |
| 车辆动力学 | TORCS / 自行车模型（已有） | ~200 行 |
| 碰撞检测 | 自己写（无合适开源） | ~300 行 |

**我们真正要写的东西：**

| 模块 | 行数 | 理由 |
|------|------|------|
| Entity System（C++ flowcoro） | ~300 | 简单池管理，不需要 ECS 库 |
| NPC AI 状态机（IDM + 行为树） | ~500 | esmini 不提供 NPC 驾驶行为 |
| 场景驱动（红灯/ETC/事件序列） | ~300 | 场景文件解析 + 事件调度 |
| scene/frame topic 发布 | ~200 | 将仿真状态转成 topic 消息 |
| Topic 适配层（控制/cmd → 仿真） | ~150 | ego 从 control/cmd 读输入 |
| monitor 适配 + flow_topology 写入 | ~100 | 复用现有逻辑 |
| Three.js 场景整合 | ~400 | 拼装现有开源的渲染片段 |

**总计要写的代码：约 2000 行 C++ + 400 行 JavaScript**
**开源省掉的代码：约 3000 行**
**比纯自研（FLOWSIM v1 的 5000+ 行）少了一半还多**

---

## 一、架构总览

```
┌─────────────────────────────────────────────────────────────────────┐
│  FlowSim Node (C++20, flowcoro)                                     │
│                                                                     │
│  ╔══════════════════════════════════════════════════════════════════╗ │
│  ║  esmini RoadManager (esminiRMLib)  ──── 开源轮子，静态链接      ║ │
│  ║  - RoadNetwork: load OpenDRIVE .xodr / custom JSON              ║ │
│  ║  - road_frenet_to_xyz(road_id, lane_id, s, offset) → (x,y,h)   ║ │
│  ║  - road_xyz_to_frenet(x, y) → (road_id, lane_id, s, offset)    ║ │
│  ║  - GetLaneInfo(): speed_limit, lane_width, neighbor_lanes       ║ │
│  ╚══════════════════════════════════════════════════════════════════╝ │
│                                                                     │
│  ┌──────────────────────────────────────────────────────────────┐  │
│  │  Entity System (自己写)                                       │  │
│  │  固定池 64 实体, SOA 布局, 组件: Transform/Vehicle/ AI/NPC   │  │
│  └──────────────────────────────────────────────────────────────┘  │
│                                                                     │
│  ┌──────────────────────────────────────────────────────────────┐  │
│  │  Simulation Loop (flowcoro 协程, 20Hz)                      │  │
│  │                                                              │  │
│  │  tick():                                                     │  │
│  │    1. 读 control/cmd → 更新 ego 动力学                        │  │
│  │    2. NPC AI: IDM 跟车 + 状态机 → 更新 NPC 位置              │  │
│  │    3. 碰撞检测 → 碰撞响应                                      │  │
│  │    4. 场景事件: 红绿灯相位/ETC闸门/actor触发                     │  │
│  │    5. 发布 vehicle/state + scene/frame                        │  │
│  └──────────────────────────────────────────────────────────────┘  │
│                                                                     │
│  ┌──────────────────────────────────────────────────────────────┐  │
│  │  Topic 接口                                                  │  │
│  │  subscribe: control/cmd (来自 planning→control→safety 链路)  │  │
│  │  publish: vehicle/state (ego 状态, 兼容现有 monitor)          │  │
│  │  publish: scene/frame (完整场景帧, 20Hz, 给 3D 前端)         │  │
│  │  publish: road/geometry (兼容现有 planning/control 节点)      │  │
│  │  publish: road/traffic_lights                                │  │
│  └──────────────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────────┘
          │
          │  20Hz JSON via IPC/TCP
          ▼
┌─────────────────────────────────────────────────────────────────────┐
│  flowboard_server.py + Browser (Three.js)                           │
│                                                                     │
│  ┌──────────────────────────────────────────────────────────────┐  │
│  │  3D 场景 (scene3d.js, 重写)                                  │  │
│  │  - 道路: CatmullRomCurve3 + TubeGeometry (借开源 OpenDRIVE   │  │
│  │          viewer 的渲染代码)                                   │  │
│  │  - 车辆: glTF 模型 (Blender 导出 Sedan/Truck/SUV, GLTFLoader)│  │
│  │  - 建筑: 来自 open-source Three.js city generator            │  │
│  │  - 树木/路灯: Three.js 原生几何 + 实例化                     │  │
│  │  - 红绿灯: 现有逻辑(升级)+ emissive bloom                    │  │
│  │  - ETC 门架: BoxGeometry 组合+ 抬杆动画                      │  │
│  │  - Bloom/Shadow: 现有 EffectComposer 升级                     │  │
│  └──────────────────────────────────────────────────────────────┘  │
│                                                                     │
│  ┌──────────────────────────────────────────────────────────────┐  │
│  │  2D 俯视图 (scene2d.js, 保持)                                │  │
│  │  - 保持现有 Canvas 2D 渲染（一直比 3D 好用）                 │  │
│  │  - 升级支持多段道路显示                                        │  │
│  └──────────────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────────┘
```

---

## 二、esmini RoadManager 集成

### 2.1 集成方式

esmini 的 RoadManager（`esminiRMLib`）是一个独立的 C++ 静态库，在 esmini 仓库的 `EnvironmentSimulator/RoadManager` 目录下。

```
依赖关系:
flowsim_node.cpp
    └── #include "RoadManager.hpp"    ← esmini 头文件
    └── link esminiRMLib.a            ← 编译好的静态库
```

esmini 的 CMakeLists.txt 已经支持作为子项目引入：

```cmake
# 在 FlowEngine CMakeLists.txt 中
add_subdirectory(third_party/esmini)
target_link_libraries(flowsim_node PRIVATE esminiRMLib)
```

### 2.2 使用 RoadManager 做道路查询

```cpp
#include "RoadManager.hpp"

class FlowRoadNetwork {
    roadmanager::OpenDrive* m_odr;

public:
    bool load(const char* xodr_path) {
        m_odr = new roadmanager::OpenDrive(xodr_path);
        return m_odr->GetNumOfRoads() > 0;
    }

    // Frenet → 世界坐标
    bool frenet_to_world(int road_id, int lane_id, double s, double offset,
                         double& out_x, double& out_y, double& out_h) {
        auto* road = m_odr->GetRoadById(road_id);
        if (!road) return false;
        auto* lane = road->GetLaneById(lane_id);
        if (!lane) return false;

        roadmanager::Position pos;
        pos.SetLanePos(road_id, lane_id, s, offset);
        out_x = pos.GetX();
        out_y = pos.GetY();
        out_h = pos.GetH();
        return true;
    }

    // 世界坐标 → Frenet
    bool world_to_frenet(double x, double y,
                         int& out_road, int& out_lane, double& out_s, double& out_offset) {
        roadmanager::Position pos;
        pos.SetInertiaPos(x, y, 0);
        out_road   = pos.GetTrackId();
        out_lane   = pos.GetLaneId();
        out_s      = pos.GetS();
        out_offset = pos.GetOffset();
        return out_road >= 0;
    }

    double speed_limit(int road_id, int lane_id) {
        auto* road = m_odr->GetRoadById(road_id);
        if (!road) return 30.0;
        auto* lane = road->GetLaneById(lane_id);
        return lane ? lane->GetSpeed() : 30.0;
    }
};
```

### 2.3 自定义 JSON → OpenDRIVE 转换

esmini 原生吃 `.xodr`（OpenDRIVE XML）文件。但我们不想手写 xodr，所以写一个**极简转换器**，把我们的场景 JSON `road_network` 转成合法的 xodr：

```
flow_sim_scene.json
    │
    ▼
json_to_xodr.py (或内置 C++ 转换)      ← 约 300 行
    │
    ▼
temp_scene.xodr
    │
    ▼
esmini RoadManager::OpenDrive("temp_scene.xodr")
```

xodr 格式其实很简单（对直路/弯道来说），核心要素：

```xml
<OpenDrive>
  <road name="urban" length="200" id="0" junction="-1">
    <planView>
      <geometry s="0" x="0" y="0" hdg="0" length="200">
        <line/>
      </geometry>
    </planView>
    <lanes>
      <laneSection s="0">
        <center><lane id="0" type="none"><width a="0" b="0" c="0" d="0"/></lane></center>
        <right>
          <lane id="-1" type="driving"><width a="3.5" b="0" c="0" d="0"/></lane>
          <lane id="-2" type="driving"><width a="3.5" b="0" c="0" d="0"/></lane>
          <lane id="-3" type="driving"><width a="3.5" b="0" c="0" d="0"/></lane>
        </right>
        <left>
          <lane id="1" type="driving"><width a="3.5" b="0" c="0" d="0"/></lane>
        </left>
      </laneSection>
    </lanes>
  </road>
</OpenDrive>
```

弯曲路段用 `<spiral>`（clothoid）或 `<arc>` 几何类型：

```xml
<geometry s="0" x="200" y="0" hdg="0" length="250">
  <arc curvature="0.0222"/>    <!-- R=45m -->
</geometry>
```

**这项工作只是一次性写一个 JSON→xodr 转换器，后续所有场景自动受益。**

---

## 三、Entity System（C++ flowcoro）

### 3.1 实体类型

```cpp
enum class EntityType : uint8_t {
    None,
    Ego,          // 自车（外部控制）
    Car,          // NPC 轿车
    SUV,          // NPC SUV
    Truck,        // NPC 卡车
    Pedestrian,   // 行人
    TrafficLight, // 红绿灯
    ETCGate,      // ETC 闸门
    StopLine,     // 虚拟停止线
};

struct Entity {
    bool     active = false;
    EntityType type = EntityType::None;

    // Transform
    double x = 0, y = 0, heading = 0;
    double vx = 0, vy = 0;       // 当前速度
    double target_vx = 0;        // AI 目标速度

    // Vehicle (car/truck/suv only)
    double length = 4.6, width = 2.0;
    double wheelbase = 2.7;      // 自行车模型
    double max_accel = 2.0;      // m/s²
    double max_brake = 4.0;      // m/s²

    // AI state (NPC only, Ego doesn't use this)
    enum class AIState {
        Cruise,     // 巡航
        Follow,     // 跟车
        Stop,       // 停止
        StopForTL,  // 红灯停车
        ETCApproach,// ETC 减速
        BranchSel,  // 选路
        Merge,      // 汇入
        Yield,      // 让行
    };
    AIState ai_state = AIState::Cruise;
    EntityId lead_id = -1;       // 跟车目标
    int edge_id = -1;            // 所在道路 edge
    double s = 0, l = 0;        // Frenet 坐标
    int lane_id = 0;             // 车道 ID
};
```

### 3.2 flowcoro 协程主循环

```cpp
#include "flowcoro/flowcoro.h"

class FlowSimNode : public flowcoro::Task {
    Entity m_entities[64];
    int m_entity_count = 0;
    FlowRoadNetwork m_roads;
    uint64_t m_tick_count = 0;

    // topic handlers
    flowcoro::Subscriber<ControlCmd> m_cmd_sub;
    flowcoro::Publisher<VehicleState> m_state_pub;
    flowcoro::Publisher<SceneFrame> m_scene_pub;

    Task<> run() override {
        // 等待第一个 control/cmd 或超时
        auto cmd = co_await m_cmd_sub.next();
        // ... 初始化 ...

        while (!should_stop()) {
            uint64_t tick_start = clock_now_us();

            // 1. 读控制指令 → 更新 ego
            if (auto cmd = m_cmd_sub.try_next()) {
                apply_control(m_entities[0], *cmd);
            }
            step_ego_physics(&m_entities[0]);

            // 2. NPC AI
            for (int i = 1; i < m_entity_count; i++) {
                if (m_entities[i].active) {
                    step_npc_ai(&m_entities[i]);
                }
            }

            // 3. 碰撞检测
            detect_collisions();

            // 4. 场景事件
            tick_scene_events();

            // 5. 发布
            publish_vehicle_state();
            publish_scene_frame();

            // 等待下一个 tick（20Hz）
            uint64_t elapsed = clock_now_us() - tick_start;
            if (elapsed < 50000) {  // 50ms tick
                co_await sleep_us(50000 - elapsed);
            }
            m_tick_count++;
        }
    }
};
```

---

## 四、NPC AI 状态机

### 4.1 状态定义

```
每个 NPC tick:
  1. 更新 Frenet 坐标 (s, l) 从世界坐标反算
  2. 找同车道最近的前车
  3. 检测前方是否为红灯/ETC/分叉/汇入点
  4. 计算 v_desired = min(各限制)
  5. 加减速到 v_desired
  6. 更新世界坐标
```

### 4.2 IDM 跟车（复用 FLOWSIM v1 设计）

```cpp
double idm_desired_speed(double v, double gap, double target_v) {
    double safe_gap = 3.0 + v * 1.0;
    double gap_error = gap - safe_gap;
    if (gap_error > 0) {
        return std::min(v + 2.0 * 0.05, target_v);
    } else {
        double brake = 4.0 * std::exp(-gap_error / 2.0);
        return std::max(0.0, v - brake * 0.05);
    }
}
```

### 4.3 红灯感知

```cpp
void check_traffic_light(Entity* npc) {
    // 用 RoadManager 找前方最近的 traffic light
    double ahead_s = npc->s + 1.0;  // 前看 1m 起步
    for (int i = 0; i < m_entity_count; i++) {
        auto* tl = &m_entities[i];
        if (tl->type != EntityType::TrafficLight) continue;
        if (tl->edge_id != npc->edge_id) continue;
        if (tl->s <= npc->s || tl->s > npc->s + 80.0) continue;
        if (std::abs(tl->l - npc->l) > 2.0) continue;  // 同车道

        if (tl->ai_state == AIState::Stop) {
            // 红灯：目标速度设为 0
            double dist_to_stop = tl->s - npc->s;
            // 减速到刚好在停止线前停住
            double brake_dist = npc->vx * npc->vx / (2 * 4.0);  // v²/2a
            if (dist_to_stop < brake_dist + 3.0) {
                npc->target_vx = std::min(npc->target_vx, 0.0);
            }
        }
        break;  // 只处理最近的灯
    }
}
```

---

## 五、场景事件调度

### 5.1 事件类型

```cpp
struct SceneEvent {
    enum Type {
        TrafficLightCycle,   // 红绿灯周期
        ETCGateTrigger,      // ETC 抬杆
        NPCWaypoint,         // NPC 路径点
        RoadChange,          // 道路切换
    };
    Type type;
    double trigger_s;        // 触发里程（ego 的 s）
    union {
        struct { int tl_id; double red_s, yellow_s, green_s; } tl;
        struct { int gate_id; } etc;
        struct { int npc_id; double target_vx; } npc;
    };
};
```

### 5.2 红绿灯

```cpp
void tick_traffic_lights() {
    double t = m_tick_count * 0.05;  // 仿真时间（秒）
    for (auto& e : m_entities) {
        if (e.type != EntityType::TrafficLight) continue;
        // 从场景配置中取的 green_s/yellow_s/red_s
        double T = e.green_s + e.yellow_s + e.red_s;
        double tp = std::fmod(t + e.phase_offset, T);
        if (tp < e.green_s) {
            e.ai_state = AIState::Cruise;  // 绿灯
        } else if (tp < e.green_s + e.yellow_s) {
            e.ai_state = AIState::Yield;    // 黄灯
        } else {
            e.ai_state = AIState::Stop;     // 红灯
        }
    }
}
```

### 5.3 ETC 闸门

```cpp
void tick_etc_gates() {
    Entity* ego = &m_entities[0];
    for (auto& gate : m_entities) {
        if (gate.type != EntityType::ETCGate) continue;
        double dist = gate.s - ego->s;
        if (dist < 50.0 && dist > 10.0) {
            gate.ai_state = AIState::Stop;  // gate closed
            // ego 的 target_speed 会被 planning_node 限制
        } else if (dist <= 10.0 && dist > 0) {
            gate.ai_state = AIState::Yield; // opening...
            gate.vy += 1.0 * 0.05;          // 抬杆动画进度
        } else if (dist <= 0) {
            gate.ai_state = AIState::Cruise;// passed
        }
    }
}
```

---

## 六、Topic 接口

### 6.1 输入：control/cmd

直接复用现有的 `ControlCmd` 二进制类型（`adas_msgs.h`），订阅 `control/cmd` topic：
- `throttle` [0,1]
- `brake` [0,1]
- `steering` rad

### 6.2 输出：vehicle/state

保持现有 JSON 格式不变（兼容 monitor_node + flowboard）：

```json
{
  "x": 102.5, "y": -1.75, "hdg": 0.05,
  "spd": 8.0, "thr": 0.3, "brk": 0.0, "st": 0.02,
  "tgt": 10.0,
  "n_obs": 6,
  "ox0": 120, "oy0": -1.75, "ov0": 3.0, "ot0": "car",
  ...
}
```

### 6.3 新增输出：scene/frame

新 topic，20Hz，给 3D 前端用的完整场景帧：

```json
{
  "t_us": 12345678,
  "road_network": {
    "edges": [
      { "id": 0, "nodes": [[0,0], [200,0]], "lanes": 3, "lane_width": 3.5 },
      { "id": 1, "nodes": [[200,0], [350,0]], "lanes": 3, "lane_width": 3.5 },
      { "id": 4, "nodes": [[510,0], [650,100], [900,200]], "lanes": 1 },
      { "id": 5, "nodes": [[900,200], [1400,200]], "lanes": 3 }
    ]
  },
  "entities": [
    { "id": 0, "type": "ego", "x": 102.5, "y": -1.75, "h": 0.05,
      "spd": 8.0, "steer": 0.02, "len": 4.6, "wid": 2.0,
      "brake": false, "turn_signal": 0 },
    { "id": 1, "type": "car_red", "x": 120, "y": -1.75,
      "h": 0, "spd": 3.0, "len": 4.6, "wid": 2.0 },
    { "id": 6, "type": "tl", "x": 200, "y": -1.75,
      "state": "red", "remain_s": 12.3 },
    { "id": 12, "type": "etc_gate", "x": 450, "y": 0,
      "state": "closed", "progress": 0.0 }
  ]
}
```

**频率：20Hz**（与仿真 tick 一致，前端不再需要死区推算）

---

## 七、3D 渲染：从开源拼方案

### 7.1 道路渲染（借 Three.js OpenDRIVE viewer）

核心思路：不用自己写道路渲染代码。已经有 `odrviewer.io` 这样的在线工具证明了 Three.js 可以渲染 OpenDRIVE 道路。我们用同样的技术：

```javascript
// 从 scene/frame 的 road_network 构建 Three.js 道路
function buildRoadFromScene(sceneData) {
    for (const edge of sceneData.road_network.edges) {
        // nodes 是控制点数组 [[x,y], [x,y], ...]
        const points = edge.nodes.map(n => new THREE.Vector3(n[0], -0.3, n[1]));
        const curve = new THREE.CatmullRomCurve3(points);

        // 道路宽度 = lanes * lane_width
        const roadWidth = edge.lanes * edge.lane_width;

        // TubeGeometry 沿着曲线生成道路网格
        const tubeGeo = new THREE.TubeGeometry(curve, 64, roadWidth / 2, 8, false);
        const roadMat = new THREE.MeshStandardMaterial({
            color: 0x333333,
            roughness: 0.8,
            metalness: 0.1,
        });
        const roadMesh = new THREE.Mesh(tubeGeo, roadMat);
        roadMesh.rotation.x = -Math.PI / 2;  // 平躺
        scene.add(roadMesh);

        // 车道线：沿曲线画虚线
        // 借用 Three.js 官方 Line 示例 + dash 材质
    }
}
```

**开源参考**：
- Three.js 官方示例 `webgl_geometry_extrude_splines.html` — 沿样条线挤出几何体
- Three.js `CatmullRomCurve3` — 平滑曲线
- 车道线可以用 `LineDashedMaterial` 沿曲线画虚线
- 开源项目 `three-city` 有建筑生成

### 7.2 车辆渲染（glTF 模型）

```
车辆模型: 从开源 3D 模型站下载或 Blender 导出
  ├── sedan.gltf        → 轿车（白色/红色/蓝色/黑色 4 种材质变体）
  ├── truck.gltf        → 卡车
  ├── suv.gltf          → SUV
  └── pedestrian.gltf   → 行人（胶囊或低面人形）

Three.js 加载:
  const loader = new THREE.GLTFLoader();
  loader.load('/models/sedan.gltf', (gltf) => {
      const car = gltf.scene.clone();
      car.position.set(x, 0, y);
      car.rotation.y = -heading;
      scene.add(car);
  });

车辆数量: 
  - 64 辆车 = 64 个 glTF 实例
  - 每辆车 ~1000 三角形，64 辆 = 64K 三角形
  - Three.js 轻松处理（手机都能跑 100K 三角形）
  - 优化：用 InstancedMesh 复用同一模型，64 辆轿车 = 1 次 draw call
```

### 7.3 场景装饰（开源资产）

| 元素 | 来源 | 方案 |
|------|------|------|
| 建筑 | `three.js` 示例或 `three-city` | 随机 BoxGeometry + 窗户纹理 |
| 树木 | Three.js 原生组合几何 | 树干 Cynlinder + 树冠 Sphere |
| 路灯 | 简单定制 | 圆柱 + 球体，沿道路边缘放置 |
| 天空 | Three.js `Sky` 示例 | 渐变色或 HDR 环境贴图 |
| 地面 | Three.js 网格地面 | PlaneGeometry + GridHelper |
| 阴影 | Three.js 阴影 | 方向光 + ShadowMap |

### 7.4 罪恶都市 vs 我们

| 维度 | GTA Vice City (2002) | 我们的 Three.js |
|------|---------------------|----------------|
| 车辆面数 | ~800-1500 三角/辆 | glTF ~1000 三角/辆（可调整） |
| 车辆数量 | ~50 辆同屏 | 64 辆（InstancedMesh = 1 draw call） |
| 道路 | 纹理映射 + 车道线 | TubeGeometry + 程序化车道线 |
| 建筑 | 纹理盒子 | BoxGeometry + 窗户纹理 |
| 光照 | DirectX 8.1 固定管线 | Three.js PBR (Metalness/Roughness) |
| 阴影 | 无 | Three.js ShadowMap |
| Bloom | 无 | EffectComposer + UnrealBloomPass（已有） |
| 分辨率 | 640×480 | 1920×1080+ |

**关键差距不在几何复杂度，在美术资产**（纹理、模型细节、环境设计）。但场景是从上帝视角俯视的 ADAS 仿真，对美术要求远低于开放世界游戏。

---

## 八、文件结构

```
src/
├── flowsim_node.cpp              # C++ flowcoro 节点（主入口）
├── flowsim_node.h
├── flowsim/
│   ├── entity.h / entity.cpp     # Entity 系统
│   ├── road_network.h / .cpp     # esmini RoadManager 封装
│   ├── npc_ai.h / .cpp           # NPC AI 状态机 + IDM
│   ├── scene_events.h / .cpp     # 场景事件调度
│   ├── physics.h / .cpp          # 车辆动力学
│   ├── collision.h / .cpp        # 碰撞检测
│   └── scene_pub.h / .cpp        # scene/frame 发布
│
tools/
├── flowboard/
│   ├── index.html
│   ├── app.js                    # 主逻辑
│   ├── scene3d.js                # 3D 场景（重写）
│   ├── scene2d.js                # 2D 俯视图（升级）
│   ├── three/                    # Three.js 库 + GLTFLoader
│   └── models/                   # glTF 车辆模型
│
scripts/
├── json_to_xodr.py               # 场景 JSON → OpenDRIVE xodr
├── demo_flowsim.sh               # 启动脚本
│
third_party/
└── esmini/                       # git submodule
```

---

## 九、与现有系统的互操作

### 9.1 替换 sim_world_node

新 `flowsim_node` 直接替换 `sim_world_node` 在 pipeline 中的位置：

```json
{
  "library_path": "libflowsim_node.so",
  "name": "flowsim",
  "params": {
    "scenario_file": "scenarios/city_to_highway.json",
    "enable_3d": true
  }
}
```

### 9.2 向后兼容

| 旧组件 | 新方案 |
|--------|--------|
| `sim_world_node` | `flowsim_node`（C++ flowcoro） |
| `road/geometry` topic | 不变，继续发布 |
| `vehicle/state` topic | 不变，兼容 monitor/flowboard |
| `road/traffic_lights` topic | 不变 |
| 场景 JSON 格式 | 扩展 `road_network` 字段，旧 `road` 字段自动转换 |
| `scene3d.js` | 重写，保持 select 切换 2D/3D 不变 |
| `foxglove_bridge.py` | 不变，继续从 vehicle/state 读数据 |

**旧场景兼容：** 不含 `road_network` 字段的场景自动构建为单条直道，NPC 按 `(x, y)` 方式读取，`obey_traffic=0` 退化为原样运动。

---

## 十、实施路径

### Phase 0 — 环境准备（1天）

```bash
git submodule add https://github.com/esmini/esmini third_party/esmini
cd third_party/esmini && mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build . --target esminiRMLib
```

### Phase 1 — 核心仿真（5天）

| 步骤 | 产出 | 依赖 |
|------|------|------|
| 1.1 JSON→xodr 转换器 | `scripts/json_to_xodr.py` | 无 |
| 1.2 esmini RoadManager 封装 | `flowsim/road_network.h/.cpp` | 1.1 |
| 1.3 Entity System + 车辆动力学 | `entity.h/.cpp`, `physics.h/.cpp` | 无 |
| 1.4 NPC AI（IDM + 状态机） | `npc_ai.h/.cpp` | 1.2, 1.3 |
| 1.5 碰撞检测 | `collision.h/.cpp` | 1.3 |
| 1.6 场景事件调度（红灯/ETC） | `scene_events.h/.cpp` | 1.2 |

### Phase 2 — Topic 集成（3天）

| 步骤 | 产出 | 依赖 |
|------|------|------|
| 2.1 flowcoro 主循环 | `flowsim_node.cpp` | 1.1–1.6 |
| 2.2 scene/frame 发布 | `scene_pub.h/.cpp` | 1.3 |
| 2.3 monitor 适配 | `monitor_node.c` 改动 | 2.2 |
| 2.4 旧场景回归测试 | 全部既有场景通过 | 2.1 |

### Phase 3 — 3D 渲染（4天）

| 步骤 | 产出 | 依赖 |
|------|------|------|
| 3.1 道路 TubeGeometry 渲染 | `scene3d.js` 重写 | 2.2 |
| 3.2 glTF 车辆模型 + InstancedMesh | 模型加载 + 实例化 | 3.1 |
| 3.3 场景装饰（建筑/树/路灯） | 城市生成 | 3.1 |
| 3.4 特效升级（阴影/Bloom/车灯） | 后期处理 | 3.2 |
| 3.5 2D 俯视图多段道路适配 | `scene2d.js` 升级 | 2.2 |

### Phase 4 — 场景落地（2天）

| 步骤 | 产出 |
|------|------|
| 4.1 编写 `city_to_highway.json` | 完整场景 |
| 4.2 验证 NOA 全链路行为 | 红灯/ETC/选路/汇入/超车 |
| 4.3 demo_evaluator.py 回归 | pass_criteria 全绿 |
| 4.4 美化参数调优 | 光照/颜色/摄像机角度 |

**总计：约 15 天（一个人）**

---

## 十一、总结：比 FLOWSIM v1 好在哪

| 维度 | FLOWSIM v1（纯 C 自研） | FLOWSIM v2（C++ + esmini） |
|------|------------------------|---------------------------|
| 道路网络 | 自己写 Node/Edge/Junction + Hermite | esmini RoadManager，行业标准 OpenDRIVE |
| Frenet 坐标 | 自己实现 + 采样表 + 二分查找 | RoadManager 内置，精确到厘米级 |
| 车道查询 | 自己算相邻关系 | RoadManager `GetLaneById()`, `GetNeighbors()` |
| 3D 道路渲染 | 自己写 TubeGeometry | 参考开源 OpenDRIVE Three.js viewer |
| 车辆模型 | BoxGeometry 方块 | glTF 模型（Blender 导出的轿车/卡车/SUV） |
| 引入外部依赖 | 0 | esminiRMLib（C++ 静态库，~2MB） |
| 代码量 | ~5000 行 C | ~2000 行 C++ + ~400 行 JS |
| 道路格式兼容性 | 只有自定义 JSON | 兼容 OpenDRIVE 行业标准 |
| 重构风险 | 高（全自研） | 低（核心道路模块用开源成熟方案） |

**一句话：把最难的部分（道路网络）交给 esmini，我们只写开源没有的东西（NPC AI + 场景驱动 + 确定性仿真）。这样 2000 行代码就能拿到原本 5000 行的效果，而且 3D 效果能追上 20 年前的游戏。**
