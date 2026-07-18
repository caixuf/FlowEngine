# 3D 场景视觉/行为问题修复计划

> 状态：计划已定稿，待执行
> 范围：FlowBoard 3D 场景（`tools/flowboard/js/`）+ 必要的事件透传后端
> 前置：黑屏根因（glTF `sheen` 裸数字材质 bug）已修复，用户首次能真正看到 3D 场景内容

---

## 1. 背景

黑屏修复后，在一次连续 live walkthrough 中暴露出一批横跨两个子系统的问题：
- **JS/Three.js 场景图**：几何/缩放 bug、美术调优诉求
- **C++ control/planning 决策逻辑**：红绿灯前死锁

性质差异大：有的可精确定位，有的需现场复现，有的是开放式美术重做。本计划按**确认度分组**处理，不逐条盲改。

## 2. 问题清单（原话，按到达顺序）

| # | 问题 | 子系统 |
|---|------|--------|
| 1 | 路面效果不行，沥青材质加车道线都不行 | JS 场景 |
| 2 | 路灯看着很奇怪 | JS 场景 |
| 3 | 前视图时候看着前车车轮怎么像是横着装的 | JS 场景 |
| 4 | 一直卡在红绿灯前 | C++ 决策 |
| 5 | 这个人的建模，一言难尽啊，看着比车都大，而且就是几个方块 | JS 场景 |
| 6 | 这个街景看着跟…，根据上海某个高速入口重新设计下吧 | JS 场景 |
| 追加 | 物理引擎缺失（车身侧倾/俯仰/悬挂） | JS 场景 |

## 3. 处理策略

```
┌─ A. 已确认根因（可直接修）─────────────────────┐
│  P1 前轮转向轴心错误                            │
│  P2 障碍物默认缩放不分类型（行人比车大）        │
└─────────────────────────────────────────────────┘
┌─ B. 新增前端视觉物理效果（已确认方向）─────────┐
│  P3 车身侧倾/俯仰/悬挂起伏（纯前端近似）        │
│  P4 碰撞视觉反馈（可选，需小后端插件透传事件）  │
└─────────────────────────────────────────────────┘
┌─ C. 需要现场诊断（先查再改，不盲猜）───────────┐
│  P5 沥青路面/车道线/路灯"看着不对"             │
│  P6 红绿灯前一直卡住                            │
└─────────────────────────────────────────────────┘
┌─ D. 单独立项，不纳入本轮───────────────────────┐
│  P7 参考上海某高速入口重新设计街景              │
│  P8 仿真后端动力学升级（侧滑/悬挂/冲量碰撞）    │
└─────────────────────────────────────────────────┘
```

执行顺序：**A → B → C**，D 单独排期。

---

## A. 已确认根因（可直接修）

### P1 前轮转向轴心错误 → 前视图看到"车轮像横着装"

**根因**
- `gen_models.py` 生成 glTF 时，每个 part 的绝对坐标直接烘进顶点数据（`box_vertices`/`cylinder_vertices` 用 `cx,cy,cz` 写入每个顶点），而 `build_gltf()`（gen_models.py:561-566）给每个节点写的是 `{"mesh": i, "name": part_names[i]}`——**没有 translation/rotation**。`wheel_FL`/`wheel_FR` 节点的本地变换是单位矩阵。
- `models.js` 的 `_buildVehicleFromGltf`（约第 42-101 行）建立前轴转向 pivot 时：
  ```js
  var center = new THREE.Vector3();
  fl.getWorldPosition(center);   // 单位变换节点 → 恒返回 (0,0,0)
  var frPos = new THREE.Vector3();
  fr.getWorldPosition(frPos);    // 同样恒为 (0,0,0)
  center.add(frPos).multiplyScalar(0.5);   // fwGroup 被放在车身原点
  ```
  `Object3D.getWorldPosition()` 读的是节点变换矩阵，不是几何体顶点数据——两者对不上。
- 于是 `frontWheels`（fwGroup）的转向 pivot 被放在车身原点，而不是真实前轴位置（按 gen_models.py 约 1.35~1.5m）。`_renderFrame` 用 `frontWheels.rotation.y` 打方向时，两个前轮绕错误支点画弧线，而非原地转向——正面/前视图角度下最明显。
- `_relinkWheelUserData`（clone 路径，约 201-238 行）是同一段逻辑的复制，同样有此问题。

**修复方案**
- 不要用 `getWorldPosition()`（对单位变换节点无意义），改用**几何体包围盒中心**算 pivot：
  ```js
  var box = new THREE.Box3().setFromObject(fl);
  box.expandByObject(fr);
  var center = new THREE.Vector3();
  box.getCenter(center);
  ```
  或直接读 `fl.geometry.boundingBox`。
- `_buildVehicleFromGltf` 和 `_relinkWheelUserData` 两处都要改。

**涉及文件**
- `tools/flowboard/js/models.js`（`_buildVehicleFromGltf`、`_relinkWheelUserData`）

**验证**
- 前视图（`setCameraMode('front')`）下打方向，前轮原地转向，不画弧线。
- `gen_models.py` 重新生成模型后，包围盒中心应落在前轴位置（约 x≈1.4m）。

**状态**：已完成（2026-07-18）— `models.js:75-90` 与 `models.js:215-230` 已改用 `Box3().setFromObject()` 取左右前轮几何中心作为 frontWheels Group pivot，并随分层架构重构一同推送到 main。

---

### P2 障碍物默认缩放不分类型 → 行人"比车都大，就是几个方块"

**根因**
- `_buildObstacle(type, color)`（utils.js:398）文档明确：返回 unit-normalized（约 1×1×1 包围盒）模型，调用方需 `.scale.set(L,H,W)` 映射到真实米制尺寸。
- 调用方在 `scene3d.js`（约 2307-2351 行）：
  ```js
  var L = ow.len || DEFAULT_OBS_LEN, W = ow.wid || 2;   // DEFAULT_OBS_LEN = 4（车辆尺寸）
  var H = _OBS_H[ow.type] || 1.5;                        // H 已 per-type
  ```
  `H` 已按类型区分（`_OBS_H = {truck:2.8, pedestrian:1.8, cyclist:1.7, cone:0.8}`，scene3d.js:133），但 `L` 和 `W` **没有 per-type 表**，缺失时统一 fallback 到车辆尺寸（4m 长 × 2m 宽）。
- 若感知/融合数据里行人 track 没有 `len`/`wid` 字段（很可能——行人一般不描述"车长"），胶囊体就被拉伸成 4m×2m 的粗大方块——比实际车身（约 4~4.5m）还显眼。

**修复方案**
- 仿照 `_OBS_H` 新增 `_OBS_L` / `_OBS_W` 两张 per-type 表：
  ```js
  const _OBS_L = { truck: 8, pedestrian: 0.5, cyclist: 1.8, cone: 0.5 };
  const _OBS_W = { truck: 2.5, pedestrian: 0.5, cyclist: 0.6, cone: 0.5 };
  ```
- 调用处改成：
  ```js
  var L = ow.len || _OBS_L[ow.type] || DEFAULT_OBS_LEN;
  var W = ow.wid || _OBS_W[ow.type] || 2;
  ```

**涉及文件**
- `tools/flowboard/js/scene3d.js`（`_OBS_H` 附近新增表 + 调用处）

**验证**
- 行人显示为约 0.5×0.5×1.8m，明显小于车辆。
- 锥桶显示为约 0.5×0.5×0.8m。

**状态**：已完成（2026-07-18）— `scene3d.js:155-156` 已新增 `_OBS_L` / `_OBS_W` per-type 表，`scene3d.js:2241` 已改用 `ow.len || _OBS_L[ow.type] || DEFAULT_OBS_LEN`，并随分层架构重构一同推送到 main。

---

## B. 新增前端视觉物理效果（已确认方向）

### P3 车身侧倾/俯仰 + 悬挂起伏（roll/pitch/bounce）

**背景**
- `modules/adas_nodes/flowsim/physics.cpp` 确实有一个有文档的 2D 自行车模型（纵向力→加速度，转向角→航向，physics.h:8-12 明确写"无侧滑""低速近似"），但只有 x/y/heading，没有 z/roll/pitch，没有悬挂状态。
- 碰撞响应（collision.h:44-48 `apply_collision_response`）只是把双方速度归零，不是基于冲量的真实响应。
- 前端 `scene3d.js` 目前只有一个纯装饰性效果：`ego.position.y = 0.05 + sin(_animT*6.5)*...`（固定频率车身整体浮动，与车速/转向/刹车都无关）。这就是"看着没有物理感"的真实原因——不是哪里错了，是这类效果本来就没做。

**方案：纯前端近似，不改后端**
用现有遥测（`vehicle.speed`/`vehicle.steer`/`vehicle.brake`，已在 `_renderFrame()` 读取，scene3d.js:2198-2222）：

- **横向侧倾（roll）**：转弯时车身向外侧倾斜。用航向帧间差分估算横摆角速度（`yaw_rate = Δheading/dt`），乘车速近似横向加速度，乘小增益并 clamp 到 ±3°，作为车身 group 的 `rotation.z` 目标值，每帧 lerp 平滑（代码已有先例：ETC 抬杆 `boom.rotation.y += (target - current) * 0.15`，scene3d.js:2518）。
- **纵向俯仰（pitch）**：刹车点头/加速下蹲。用车速帧间差分估算纵向加速度，clamp+lerp，驱动车身 group 的 `rotation.x`。
- **悬挂起伏（bounce）**：复用已有 `ego.position.y` 正弦效果，不做成独立四轮悬挂——过度设计，车身整体近似先够用。
- **NPC 同步处理**：现在只有 ego 有 bounce，NPC 完全刚体。用同一个"航向/速度帧间差分" helper（按 obstacle id 缓存上一帧 heading/speed）复用到障碍物更新循环（scene3d.js ~2380-2410 的 `ow` 循环），否则只有 ego 会动、NPC 僵硬，视觉不一致。

**不改** `modules/adas_nodes/flowsim/physics.cpp`——所有计算只读已有遥测字段，纯视觉近似，零后端风险。

**涉及文件**
- `tools/flowboard/js/scene3d.js`（`_renderFrame` ego 段 + obstacle 循环段）

**验证**
- 转弯时车身向外侧倾斜 2~3°。
- 急刹时车头下沉。
- NPC 转弯/刹车时也有相同效果。

**状态**：待执行

---

### P4 碰撞视觉反馈（可选，本轮内）

**背景**
- `flowsim_node.cpp` 已在检测碰撞并发布 `sim/collision`（flowsim_node.cpp:529-539，`detect_collisions`+`apply_collision_response` 调用见 677-688），但只在 ego 涉及碰撞时发布。
- `modules/adas_nodes/monitor_node.c` **没有把这个 topic 的内容转发进前端读的拓扑 JSON**——`demo_evaluator.py:491` 只读到 `sim/collision` 的 pub 计数，读不到事件本身（时间/位置/对象）。

**方案**
- 需要一个很小的后端插件：`monitor_node.c` 订阅 `sim/collision`，把最近一次事件（时间戳+位置）透传进拓扑 JSON / `dashboard_bridge`，走 CLAUDE.md 里已文档化的 `stats_bridge`/`dashboard_bridge` 同一套转发模式——是事件透传，不是"动力学升级"。
- 前端收到事件后，在碰撞位置触发一个短暂的闪光/震动 sprite。

**涉及文件**
- `modules/adas_nodes/monitor_node.c`（新增订阅 + 透传）
- `tools/flowboard/js/scene3d.js`（碰撞特效 sprite）

**降级**
- 如果这轮想保持零后端文件改动，可跳过此项，留到 P8 一起做。

**状态**：可选，待定

---

## C. 需要现场诊断（先查再改，不盲猜）

### P5 沥青路面 / 车道线 / 路灯"看着不对"

**静态读代码结论**
- 通读了 `_buildRoad` / `_buildRoadNetwork` / `_makeAsphaltTexture`（scene3d.js:204-750）：构造逻辑本身相当完整——程序化 canvas 纹理（色块+颗粒+裂缝）、沿曲线的 ribbon mesh 路面、合并后的顶点色车道线（实线/虚线区分）、交错路灯。
- 静态读代码**没有发现**类似 sheen 那种明确的类型 bug。
- 考虑到黑屏 bug 刚修好，用户很可能是第一次真正看到这部分几何——所以"不行"可能是：
  - (a) 确实有还没暴露的渲染缺陷
  - (b) 合理的美术/调优诉求（贴图对比度偏低、路灯只是发光方块没有真实光晕，因为项目为了性能特意不加 PointLight，见 scene3d.js:627 注释）

**处理方式**
1. 先在浏览器跑一次本 session 刚建好的 `flowboard._auditMaterials()`（免费、秒级），排除材质类型 bug。
2. 若结果干净，截图对比后按"调优"处理：
   - 贴图对比度
   - 路灯自发光强度/颜色
   - 必要时加一个 sprite 模拟灯晕
3. 作为比 A 组两个确认 bug 更低优先级的后续小项，**不在本轮和前两个 bug 一起动手术式修改**。

**涉及文件**
- `tools/flowboard/js/scene3d.js`（`_makeAsphaltTexture`、`_buildRoadNetwork` 路灯段）

**验证**
- `_auditMaterials()` 输出干净（无 MeshPhysicalMaterial 未知属性）。
- 截图对比调优前后。

**状态**：待诊断

---

### P6 红绿灯前一直卡住

**静态追链路结论**
- `include/traffic_light.h` 的 `traffic_light_phase_at()` 是无状态纯函数相位循环（绿→黄→红→绿），单独看逻辑没问题。
- `planning_node.cpp`（约 658-692 行）只在红/黄灯且刹停距离内才向 Frenet 规划器注入虚拟停止墙，**绿灯时明确跳过注入**（第 668 行）——规划层理论上灯变绿就不再挡车。
- `control_node.cpp` 里有一套通用的、不感知红绿灯的"停太久就给油门"死锁恢复（`SPEED_ZERO_RECOVERY` 第 843-854 行，`ROAD_GUARD` 第 859-873 行）——这正是 CLAUDE.md 里之前记录的"车速降到 0 永久卡死"问题的修复，从第 613-615 行注释看历史 bug 已修过，**不是当前卡死的原因**。
- 静态读代码**没找到确凿的 bug 点**。

**最可能的解释**
- (a) 场景配置的 `red_s`/`green_s` 时长在 demo 时间尺度内实际上很难/不会循环到绿灯。
- (b) 通用死锁恢复的油门与红灯虚拟墙互相拉扯，车一直"够不到"真正通过停止线的状态。

**处理方式**
- **先现场复现**：跑 demo，盯着一辆车完整经历一次红→绿周期，观察 control 节点已有的 mode 字段日志（如 `ROAD_GUARD`/`SPEED_ZERO_RECOVERY`）在卡住前后的变化。
- 定位到真实机制后再动手改，**不盲猜代码**。

**涉及文件**
- `scenarios/city_to_highway_full.json`（红绿灯时长配置，待查）
- `modules/adas_nodes/planning_node.cpp`（虚拟停止墙注入逻辑，待查）
- `modules/adas_nodes/control_node.cpp`（死锁恢复逻辑，待查）

**验证**
- 完整经历 红→绿 周期后，ego 能通过停止线。

**状态**：待现场复现

---

## D. 单独立项，不纳入本轮

### P7 参考上海某高速入口重新设计街景

**性质**：开放式美术/场景设计诉求，可能涉及参考资料收集、标识/护栏/车道比例、灯光氛围等更大范围的重做，和上面几个具体 bug 的性质、工作量都不是一个量级。

**建议**
- 本轮 bug 修完、验证通过后单独立项。
- 立项前需明确：
  - 参考什么（具体路口的照片/描述）
  - 重做范围（只调材质/灯光，还是要改路网几何）

**状态**：单独立项

---

### P8 仿真后端动力学升级（轮胎侧滑/悬挂状态/冲量碰撞响应）

**性质**：用户已确认这是独立后续项目，不在本轮做。

**范围供后续立项参考**
- `flowsim/entity.h` 的 `Entity` 加 z/roll/pitch/悬挂行程等字段
- `flowsim/physics.cpp` 的 `step_bicycle` 引入侧滑模型（当前 physics.h:11 明确写"无侧滑，低速近似"）
- `flowsim/collision.cpp` 的 `apply_collision_response` 从"双方速度归零"改成基于质量/速度的冲量响应
- 这套改动是 planning/control 算法验证用的仿真核心，改完必须跑 `demo_evaluator.py` 全套回归，且可能让已调好的控制参数（如 Stanley 增益）表现变化，需要重新调参——工作量和风险都明显大于本轮前端项，独立排期。

**状态**：单独立项

---

## 5. 验证方式

| 修复项 | 验证方式 |
|--------|----------|
| P1 前轮转向 | 前视图打方向，前轮原地转向 |
| P2 行人缩放 | 行人明显小于车辆，尺寸约 0.5×0.5×1.8m |
| P3 车身物理 | 转弯侧倾、刹车点头、NPC 同步 |
| P4 碰撞反馈 | 碰撞瞬间闪光/震动（若做） |
| P5 路面/路灯 | `_auditMaterials()` 干净 + 截图对比 |
| P6 红绿灯 | 完整红→绿周期后 ego 通过停止线 |

每个修复项完成后跑 `python3 tools/demo_evaluator.py --scenario scenarios/city_to_highway_full.json --duration 20` 确保 CI 回归不退化。

---

## 6. 与分层架构重构的关系

本计划中的 P1/P2/P3 修复会落在 `scene3d/` 分层架构的对应模块：

| 修复项 | 落点模块 |
|--------|----------|
| P1 前轮转向 | `view/builders/` 或 `models.js`（glTF 加载层） |
| P2 障碍物缩放 | `model/SceneModel.js`（per-type 尺寸表）+ `view/updaters/ObstacleUpdater.js` |
| P3 车身物理 | `view/updaters/EgoUpdater.js` + `view/updaters/ObstacleUpdater.js` |

分层重构已完成第一批（`utils/` + `model/` 层迁移），后续 builders/updaters 迁移时，上述修复直接落在新模块里，避免改了又搬。
