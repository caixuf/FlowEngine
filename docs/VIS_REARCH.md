# FlowEngine 可视化与仿真分层重构方案

> **状态**：草案，待评审
> **范围**：C++ 仿真侧补车灯信号 + JS 可视化侧彻底重构
> **原则**：分层架构、一物体一文件、数据单向流、纯 PBR 材质、旧代码保留备份验证后删

---

## 1. 背景与问题

### 1.1 现状问题

3D 可视化代码三套并存，互相覆盖，无法正常迭代：

| 路径 | 状态 | 问题 |
|---|---|---|
| `tools/flowboard/js/scene3d.js` | 主入口，~3000 行 | God Object，51 个模块级 `let` 全局状态耦合，路面材质修复反复失败 |
| `tools/flowboard/js/scene3d/` | 10 个子文件 | 部分分层，但 scene3d.js 仍直连 THREE，子模块未被真正使用 |
| `tools/flowboard/js/scene3d_v2/` | 6 个文件，未启用 | 阉割版（`EntityView` 不做车灯，`RoadBuilder` 砍掉护栏/路灯/ETC） |

### 1.2 仿真侧缺口

仿真层设计良好（`entity.h` / `physics.h` / `scene_pub.cpp`），但缺车灯信号链路：

- `Entity` 结构有 `throttle/brake/steer`，**无车灯字段**
- `scene_pub` 发布 type/ai_state/tl_phase，**不发布灯信号**
- 3D 层永远画不出转向灯、刹车灯，控制层信号传不到可视化

### 1.3 场景文件膨胀

`scenarios/` 下 18 个 JSON，多数废弃或重复，阻碍迭代。只保留 2 个核心场景。

---

## 2. 目标分层架构

### 2.1 仿真侧（C++，增量改造）

```
┌──────────────────────────────────────────────────────────┐
│  Layer 4: ADAS Nodes                                      │
│  perception → planning → control → actuator              │
│  ★ control 输出 lights cmd，actuator 转发给 flowsim       │
└────────────────────────┬─────────────────────────────────┘
                         │ cmd (throttle/brake/steer/lights)
                         ▼
┌──────────────────────────────────────────────────────────┐
│  Layer 3: Actor Modules (物体行为) ★新增                  │
│  VehicleActor / PedestrianActor / TrafficLightActor      │
│  每个 Actor 持 Entity&，tick() 更新 transform + lights    │
└────────────────────────┬─────────────────────────────────┘
                         │ EntityPool.tick(dt)
                         ▼
┌──────────────────────────────────────────────────────────┐
│  Layer 2: World Engine (仿真世界引擎)                     │
│  flowsim_node: 主循环 / 时间步 / 调度 / 事件分发          │
│  physics: 自行车模型积分                                  │
│  collision: AABB 碰撞                                    │
│  scene_events: TL 相位 / ETC 抬杆 / StopLine 触发         │
└────────────────────────┬─────────────────────────────────┘
                         │
                         ▼
┌──────────────────────────────────────────────────────────┐
│  Layer 1: Coordinate & Map (坐标系统 + 地图)              │
│  road_position: World ↔ Frenet (s, offset)              │
│  road_network: edges/vertices/junctions 拓扑             │
│  road_geometry: 路面高程 / heading / 曲率                │
│  CoordinateSystem: ENU 统一世界系（x=East, y=North, z=Up）│
└──────────────────────────────────────────────────────────┘
```

### 2.2 可视化侧（JS，彻底重构）

```
┌──────────────────────────────────────────────────────────┐
│  Layer 4: App Shell (app.js)                              │
│  连接 / 视角切换 / 性能档位 / UI 面板                      │
└────────────────────────┬─────────────────────────────────┘
                         │ SceneState (只读快照)
                         ▼
┌──────────────────────────────────────────────────────────┐
│  Layer 3: Scene Director (场景导演)                       │
│  SceneDirector.js: 接 scene/frame JSON → 写 SceneStore   │
│  - diff 检测：road_network hash 变了才重建路              │
│  - 实体生命周期：spawn/update/despawn                    │
│  - 灯信号分发：ego/npc lights → VehicleView              │
└────────────────────────┬─────────────────────────────────┘
                         │
            ┌────────────┴────────────┐
            ▼                         ▼
┌──────────────────────┐  ┌──────────────────────────────┐
│  Layer 2: SceneStore │  │  Layer 2: View Modules       │
│  (单一数据源)         │  │  (物体渲染，互不依赖)         │
│  - roadNetwork       │  │  RoadView      路面/车道线    │
│  - ego + lights      │  │  VehicleView   ego+npc+车灯   │
│  - entities[] + lights│  │  StreetlightView 路灯        │
│  - trafficLights[]   │  │  BarrierView   护栏/桥栏      │
│  - env (fog/lighting)│  │  ETCGateView   ETC 门架       │
│  - cameraState       │  │  TrafficLightView 红绿灯     │
└──────────────────────┘  │  GroundView    草地/地面      │
                          │  SkyView        天空/雾       │
                          │  JunctionView   路口导流岛    │
                          └──────────────┬───────────────┘
                                         │ THREE.Mesh
                                         ▼
┌──────────────────────────────────────────────────────────┐
│  Layer 1: Render Engine (渲染引擎)                        │
│  Renderer.js: WebGLRenderer + Composer + 性能档位        │
│  CameraRig.js: Chase/Map/Orbit/Top/Driver/Front 6 模式   │
│  AssetFactory.js: 几何体/材质工厂（纯 PBR，无 canvas）   │
│  Lighting.js: 环境光/方向光/半球光                       │
│  SkyEnv.js: 天空色 + fog                                 │
└──────────────────────────────────────────────────────────┘
```

### 2.3 数据流（端到端）

```
control_node → actuator → flowsim Entity.lights
                              ↓ tick(20Hz)
                          scene_pub (JSON frame, lights 位掩码 1 字节)
                              ↓
                          flowmond /api/stream (SSE)
                              ↓
                          app.js → SceneDirector.update(frame)
                              ↓
                          SceneStore.ego.lights = {headlight, turn, brake, reverse}
                              ↓
                          VehicleView.update(ego)
                              ↓ 车灯 mesh emissive 切换
                          Renderer 60fps 渲染
```

---

## 3. 目录结构

### 3.1 可视化新代码（`tools/flowboard/js/vis/`）

```
tools/flowboard/js/vis/
├── core/                       # Layer 1 Render Engine
│   ├── Renderer.js             # WebGLRenderer + Composer + 性能档位
│   ├── CameraRig.js            # 6 视角模式（Chase/Map/Orbit/Top/Driver/Front）
│   ├── AssetFactory.js         # 几何体/材质工厂，实例化缓存
│   ├── Lighting.js             # 环境光/方向光/半球光/白天夜间切换
│   └── SkyEnv.js               # 天空色 + fog
│
├── store/                      # Layer 2 Data
│   └── SceneStore.js           # 单一数据源（roadNetwork/ego/entities/env/cam）
│
├── view/                       # Layer 2 View Modules（一物体一文件）
│   ├── RoadView.js             # 路面 ribbon + 车道线（纯色 #2a2a2a，无 canvas）
│   ├── GroundView.js           # 草地/地面（纯色 #3e6b34）
│   ├── VehicleView.js          # ego + npc + 车灯 mesh（emissive）
│   ├── StreetlightView.js      # 路灯
│   ├── BarrierView.js          # 护栏 + 桥栏矮墙 + 桥墩
│   ├── ETCGateView.js          # ETC 门架（4 通道共享 1 mesh）
│   ├── TrafficLightView.js     # 红绿灯
│   ├── SignView.js             # 标志牌
│   └── JunctionView.js         # 路口导流岛/拓宽段
│
├── director/                   # Layer 3
│   └── SceneDirector.js        # JSON → SceneStore + 驱动各 View 更新
│
└── math/                       # 共享数学工具
    ├── Coord.js                # ENU ↔ THREE 世界系映射（x→x, y→z, elevation→y）
    ├── Curve.js                # CatmullRom / Bezier 采样
    └── GeometryMerge.js        # BufferGeometry 合并
```

### 3.2 可视化旧代码（保留备份，验证后删）

以下文件/目录在新代码验证通过后删除：

```
tools/flowboard/js/scene3d.js          # 3000 行 God Object
tools/flowboard/js/scene3d/            # 10 个子文件，部分分层
tools/flowboard/js/scene3d_v2/         # 6 个文件，未启用的阉割版
tools/flowboard/js/deadreckon.js       # dead reckoning 已在 SceneDirector 里做
```

**切换方式**：`tools/flowboard/index.html` 直接改 `<script>` 引用为 `vis/` 下的入口模块，一次切换，不保留 feature flag。

### 3.3 仿真侧改动（`modules/adas_nodes/`）

```
modules/adas_nodes/flowsim/
├── entity.h                # ★ 改：加 VehicleLights 结构（位掩码）
├── entity.cpp              # ★ 新增：VehicleLights 默认值
├── physics.{h,cpp}         # 不变
├── road_position.{h,cpp}   # 不变
├── road_network.{h,cpp}    # 不变
├── route.{h,cpp}           # 不变
├── npc_ai.{h,cpp}          # ★ 改：CutIn/Yield/Merge 状态输出 turn_signal
├── scene_events.{h,cpp}    # 不变
├── collision.{h,cpp}       # 不变
├── scene_pub.{h,cpp}       # ★ 改：发布 lights 字段（位掩码 1 字节）
└── actor/                  # ★ 新增 Layer 3 Actor 层
    ├── VehicleActor.{h,cpp}    # 从 control/cmd 读 lights 写 Entity
    ├── PedestrianActor.{h,cpp}
    └── TrafficLightActor.{h,cpp}

modules/adas_nodes/
├── control_node.cpp        # ★ 改：输出 lights cmd
├── actuator_node.c         # ★ 改：转发 lights 到 flowsim
└── flowsim_node.cpp        # ★ 改：tick 末尾调 actor.update_lights()
```

### 3.4 场景文件清理

`scenarios/` 下 18 个文件，只保留 2 个：

| 保留 | 用途 |
|---|---|
| `scenarios/zhongkai_road_full.json` | 中凯路综合场景（城市+高速+匝道+ETC） |
| `scenarios/infinite_straight.json` | 直路场景（基础验证） |

删除以下 16 个：

```
scenarios/city_to_highway_full.json
scenarios/intersection_signalized.json
scenarios/congestion_follow.json
scenarios/suite.json
scenarios/ghost_pedestrian.json
scenarios/city_to_highway.json
scenarios/obstacle_avoid.json
scenarios/curve_road.json
scenarios/roadwork_zone.json
scenarios/multi_pedestrian.json
scenarios/highway_overtake.json
scenarios/pedestrian_crossing.json
scenarios/cutin.json
scenarios/intersection_unprotected.json
scenarios/highway_exit.json
scenarios/highway_noa_route.json
```

---

## 4. 模块职责清单

### 4.1 可视化侧（`vis/`）

| 模块 | 职责 | 输入 | 输出 |
|---|---|---|---|
| `Renderer` | WebGLRenderer 初始化、渲染循环、Composer 后处理、性能档位 | canvas DOM | 渲染帧 |
| `CameraRig` | 6 视角模式切换、相机位置/朝向计算 | SceneStore.cameraState + ego pose | THREE.Camera |
| `AssetFactory` | 几何体/材质创建与缓存，所有 View 共享 | type + params | THREE.Mesh/Material |
| `Lighting` | 环境光/方向光/半球光、白天/夜间/雾天切换 | SceneStore.env | THREE.Light[] |
| `SkyEnv` | 天空背景色、fog 设置 | SceneStore.env | scene.background + scene.fog |
| `SceneStore` | 单一数据源，存所有场景状态，getter 只读 | SceneDirector 写入 | 各 View 读取 |
| `SceneDirector` | 接 scene/frame JSON，diff 检测，分发到 View | JSON frame | SceneStore 更新 |
| `RoadView` | 路面 ribbon mesh + 车道线，纯色无 canvas | SceneStore.roadNetwork | THREE.Group |
| `GroundView` | 草地/地面平面 | SceneStore.env | THREE.Mesh |
| `VehicleView` | ego + npc 车辆 mesh + 车灯 mesh，转向/刹车/倒车灯 | SceneStore.ego + entities | THREE.Group |
| `StreetlightView` | 路灯杆 + 灯头 | SceneStore.roadNetwork | THREE.Group |
| `BarrierView` | 护栏 + 桥栏矮墙 + 桥墩 | SceneStore.roadNetwork | THREE.Group |
| `ETCGateView` | ETC 门架（4 通道共享 1 mesh） | SceneStore.entities | THREE.Group |
| `TrafficLightView` | 红绿灯 mesh + 相位切换 | SceneStore.trafficLights | THREE.Group |
| `SignView` | 标志牌 | SceneStore.entities | THREE.Group |
| `JunctionView` | 路口导流岛/拓宽段 | SceneStore.roadNetwork | THREE.Group |
| `Coord` | ENU ↔ THREE 世界系映射 | (x, y, z) world | (x, z, y) three |
| `Curve` | CatmullRom/Bezier 采样 | 控制点 | 采样点序列 |
| `GeometryMerge` | BufferGeometry 合并 | geo[] | 合并 geo |

### 4.2 仿真侧新增/改动

| 模块 | 职责 | 改动类型 |
|---|---|---|
| `VehicleLights` (entity.h) | 车灯状态位掩码结构 | 新增 |
| `VehicleActor` | 从 control/cmd 读 lights 写 Entity.lights | 新增 |
| `PedestrianActor` | 行人行为 tick | 新增 |
| `TrafficLightActor` | 红绿灯相位 tick | 新增 |
| `control_node` | 输出 lights cmd（headlight/turn/brake/reverse） | 改 |
| `actuator_node` | 转发 lights 到 flowsim | 改 |
| `npc_ai` | CutIn/Yield/Merge 状态输出 turn_signal | 改 |
| `scene_pub` | 序列化 lights 位掩码到 JSON | 改 |
| `flowsim_node` | tick 末尾调 actor.update_lights() | 改 |

---

## 5. 数据流契约

### 5.1 scene/frame JSON Schema（含 lights）

```json
{
  "timestamp": 1234567890.0,
  "scene": {
    "road_network": {
      "edges": [
        { "id": "e0", "start": [0,0,0], "end": [1000,0,0], "lanes": 4, "length": 1000 }
      ],
      "nodes": [[x,y,z], ...]
    },
    "ego": {
      "id": 0, "type": "ego",
      "x": 123.4, "y": 0, "heading": 0.05,
      "speed": 18.5, "steer": 0.03, "brake": 0.0, "throttle": 0.4,
      "lights": 0b01100010
    },
    "entities": [
      {
        "id": 5, "type": "car",
        "x": 200.0, "y": 3.5, "heading": 0.0,
        "speed": 15.0, "steer": 0.0, "brake": 0.8,
        "length": 4.6, "width": 2.0,
        "ai_state": "follow",
        "lights": 0b00000100
      }
    ]
  }
}
```

### 5.2 VehicleLights 位掩码定义（1 字节）

```
bit 7  bit 6  bit 5  bit 4  bit 3  bit 2  bit 1  bit 0
 雾灯   倒车   高位   近光   远光   双闪   右转   左转
                       beam   beam          转向   转向
```

| 场景 | 掩码值 | 说明 |
|---|---|---|
| 关灯 | `0b00000000` = 0 | 白天默认 |
| 近光灯 | `0b00010000` = 16 | 夜间/隧道 |
| 左转向 | `0b00000001` = 1 | steer < -0.1 |
| 右转向 | `0b00000010` = 2 | steer > 0.1 |
| 双闪 | `0b00000100` = 4 | 紧急停车 |
| 刹车灯 | `0b00001000` | brake > 0.1（独立位，不与转向冲突） |

> **注**：刹车灯实际由 `brake` 字段直接驱动（brake > 0.1 即亮），不占 lights 位。lights 位只管前灯和转向灯，减少传输。

### 5.3 灯信号生成规则（control_node）

| 条件 | 输出 |
|---|---|
| 时间 18:00-06:00 或隧道内 | 近光灯 bit4=1 |
| `steer > 0.1` 持续 0.5s | 右转向 bit1=1 |
| `steer < -0.1` 持续 0.5s | 左转向 bit0=1 |
| 回正后 3s | 转向灯自动灭 |
| 紧急停车状态 | 双闪 bit2=1 |
| `brake > 0.1` | 刹车灯（由 VehicleView 直接读 brake 字段亮） |
| 速度 < 5km/h 且挡位 R | 倒车灯 bit6=1 |

---

## 6. 车灯联动设计

### 6.1 VehicleView mesh 结构

```
VehicleGroup
├── body                # BoxGeometry，PBR 车漆
├── wheelFL / wheelFR   # 前轮，rotation.y = steer * 0.5（转向联动）
├── wheelRL / wheelRR   # 后轮
├── headlightL / headlightR   # emissive 白色，intensity = headlight ? 2 : 0
├── taillightL / taillightR   # emissive 红色，intensity = brake>0.1 ? 3 : 0.5
├── turnSignalFL / turnSignalFR  # emissive 橙色，闪烁周期 500ms
├── turnSignalRL / turnSignalRR  # emissive 橙色，闪烁周期 500ms
└── reverseLightL / reverseLightR  # emissive 白色，reverse 时亮
```

### 6.2 闪烁实现

```js
// VehicleView.update(entity, now_ms)
const blink = Math.floor(now_ms / 500) % 2;  // 500ms 周期
const leftTurn = (entity.lights & 0x01) || (entity.lights & 0x04);  // 左转或双闪
const rightTurn = (entity.lights & 0x02) || (entity.lights & 0x04);
turnSignalFL.visible = leftTurn && blink;
turnSignalFR.visible = rightTurn && blink;
```

### 6.3 性能策略

- **车灯用 emissive mesh，不用 THREE.Light**：64 实体 × 8 灯 = 512 mesh，但 0 个 Light，GPU 无压力
- **mesh 池复用**：despawn 的车不销毁 mesh，隐藏归池
- **ego 夜间 spotlight（可选）**：仅 ego 在夜间模式时挂 1 个 SpotLight 投光，NPC 永远不投光

---

## 7. 性能预算

### 7.1 面数预算（总计 < 50000）

| 物体 | 单个面数 | 数量 | 总面数 |
|---|---|---|---|
| 路面 ribbon | 2 | 1（合并） | 2 |
| 草地 | 2 | 1 | 2 |
| ego 车 | 12 | 1 | 12 |
| npc 车 | 12 | 32 | 384 |
| 车灯 mesh | 4 | 64×8=512 | 2048 |
| 路灯 | 8 | 50 | 400 |
| 护栏 | 4 | 200 | 800 |
| ETC 门架 | 20 | 4 | 80 |
| 红绿灯 | 10 | 10 | 100 |
| 桥墩/桥栏 | 6 | 100 | 600 |
| 车道线 | 2 | 1（合并） | 2 |
| **合计** | | | **~4400** |

> 远低于 50000 上限，留足余量。

### 7.2 draw call 预算（< 30）

| 项 | draw call |
|---|---|
| 路面 + 车道线（合并） | 2 |
| 草地 | 1 |
| ego 车（含车灯） | 1（合并为 1 group） |
| npc 车（实例化） | 1 |
| 路灯（合并） | 1 |
| 护栏（合并） | 1 |
| ETC | 1 |
| 红绿灯 | 1 |
| 天空 | 1 |
| **合计** | **~10** |

---

## 8. 迁移步骤

### Phase 1：仿真侧补灯信号（C++）

1. `entity.h` 加 `VehicleLights` 位掩码结构
2. 新建 `actor/VehicleActor.{h,cpp}`，从 control/cmd 读 lights 写 Entity
3. `control_node.cpp` 输出 lights cmd（按 §5.3 规则）
4. `actuator_node.c` 转发 lights
5. `scene_pub.cpp` 序列化 lights 位掩码到 JSON
6. `npc_ai.cpp` 在 CutIn/Yield/Merge 状态输出 turn_signal
7. `flowsim_node.cpp` tick 末尾调 `VehicleActor::update_lights()`
8. 编译 + 跑 `test_entity_physics` / `test_scene_pub` 回归

### Phase 2：可视化骨架（JS）

1. 新建 `vis/core/Renderer.js` + `CameraRig.js` + `AssetFactory.js` + `Lighting.js` + `SkyEnv.js`
2. 新建 `vis/store/SceneStore.js`
3. 新建 `vis/director/SceneDirector.js`（最小版：接 ego + road_network）
4. 新建 `vis/view/RoadView.js`（纯色路面 #2a2a2a）+ `GroundView.js`（纯色草地 #3e6b34）
5. 新建 `vis/math/Coord.js` + `Curve.js` + `GeometryMerge.js`
6. 重写 `app.js` 调用新骨架
7. `index.html` 改 `<script type="module" src="js/vis/main.js">`
8. **验证**：infinite_straight 跑通，ego + 路面 + 草地可见，6 视角可切

### Phase 3：补全 View 模块（JS）

1. `vis/view/VehicleView.js`（含车灯 mesh + steer 转向）
2. `vis/view/StreetlightView.js`
3. `vis/view/BarrierView.js`
4. `vis/view/ETCGateView.js`（4 通道共享 1 mesh）
5. `vis/view/TrafficLightView.js`
6. `vis/view/JunctionView.js`
7. `vis/view/SignView.js`
8. **验证**：中凯路全场景跑通，所有物体可见，车灯联动正确

### Phase 4：清理

1. 删除 `scenarios/` 下 16 个废弃场景文件
2. 删除 `tools/flowboard/js/scene3d.js`
3. 删除 `tools/flowboard/js/scene3d/`
4. 删除 `tools/flowboard/js/scene3d_v2/`
5. 删除 `tools/flowboard/js/deadreckon.js`
6. **验证**：全场景跑通，无残留引用，build 无 warning

---

## 9. 验证标准

### 9.1 功能验证

| 场景 | 验证点 |
|---|---|
| infinite_straight | ego 在直路上行驶，路面深灰、草地深绿、6 视角可切 |
| zhongkai_road_full | 城市段+高速段+匝道段全可见，ETC 4 通道不堆叠 |
| 车灯联动 | ego 转向时转向灯闪烁、刹车时尾灯变亮、夜间近光灯亮 |
| NPC 车灯 | NPC CutIn 时打转向灯，跟车刹车时尾灯亮 |

### 9.2 性能验证

| 指标 | 目标 |
|---|---|
| 场景总面数 | < 50000（实际 ~4400） |
| draw call | < 30（实际 ~10） |
| 帧率 | 60fps（Chase 视角，32 NPC） |
| 启动时间 | < 2s（无 canvas 纹理生成） |

### 9.3 架构验证

| 检查项 | 标准 |
|---|---|
| 模块耦合 | View 之间无 import，只通过 SceneStore 通信 |
| 数据流 | 单向：Director → Store → View，View 不回写 |
| 纹理依赖 | 0 个 canvas 纹理（纯 PBR 纯色） |
| 全局状态 | 0 个模块级 `let`（全收敛到 SceneStore） |

---

## 10. 风险与对策

| 风险 | 对策 |
|---|---|
| C++ 改 entity.h 触发全量重编 | VehicleLights 用独立头文件 `vehicle_lights.h`，entity.h include 它 |
| 删 scene3d.js 后 app.js 引用全断 | Phase 2 先让新骨架跑通，Phase 4 才删旧代码 |
| 车灯闪烁与帧率不同步 | 闪烁用 `performance.now()`，不用 sim tick |
| scene_pub JSON 带宽增加 | lights 用 1 字节位掩码，每实体 +1 字节 |
| 旧场景文件被 CI 引用 | Phase 4 删除前先全局 grep 引用，确认无残留 |

---

## 附录 A：保留的旧文档参考

- `docs/FLOWSIM_ARCHITECTURE.md` — 仿真架构（不变）
- `docs/FLOWBOARD_SCENE_CONTRACT.md` — scene/frame JSON 契约（本方案扩展 lights 字段）
