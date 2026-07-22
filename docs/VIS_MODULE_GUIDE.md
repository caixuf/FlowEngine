# vis/ 模块设计规范 — 给设计 AI 的接入指南

> 本文档定义 FlowEngine 可视化层 `tools/flowboard/js/vis/` 的模块接口契约。
> 任何 AI 设计师按此规范输出的 View 模块，可以零修改集成进 SceneDirector。

## 1. 架构层次（必须遵守）

```
App Shell (app.js)
    ↓
Scene Director (vis/director/SceneDirector.js)
    ↓
Layer 树 (vis/core/Layer.js — Qt 对象树 + 错误隔离)
  root
  ├── env     (ground, viaduct)
  ├── road    (road, streetlight, barrier, connector)
  ├── agent   (vehicle)
  └── infra   (trafficLight, etcGate)
    ↓
View Modules (vis/view/*.js)         ← 你要写的模块在这里
  ↓
Core (vis/core/) + Math (vis/math/)  ← 你只能 import，不能修改
  ↓
Scene Store (vis/store/SceneStore.js) ← 单一数据源，View 只读
```

**铁律**：
- View 模块**只能 import** `vis/core/`、`vis/math/`、`vis/store/` 的导出
- View 模块**不能**直接读 `topoData` / `window.*` / DOM
- View 模块**不能**创建自己的全局变量；状态必须闭包在 factory 函数内
- View 模块**不能**反向调 SceneDirector 或 Layer 方法（单向依赖，对应 Qt 信号槽默认子→父方向）

**View 注册到 SceneDirector 的方式**（不是直接 import + 实例化）：
1. 在 SceneDirector.js 顶部调 `ViewRegistry.register('xxx', createXxxView)`
2. `instantiateAll(scene)` 批量建实例
3. 挂到对应 Layer（表驱动：`['agent', ['vehicle', 'pedestrian']]`）
4. 动态 View 由 `tickAnimation → rootLayer.update` 每帧递归调用
5. 静态 View 由 `update()` 内部走 `ViewRegistry.safeCall('xxx', 'build', rn)`

**错误隔离**：每个 View 的 build/update/clear/dispose 调用都包 try/catch，
单个 View 抛错只 log + 跳过，不传染兄弟。这是"一个模块坏了整个 3D 就坏了"的解药。

## 2. View 模块接口契约

每个 View 模块必须导出一个 factory 函数，签名固定：

```javascript
// vis/view/XxxView.js
import { getBox, getCylinder, getStdMaterial, createEmissiveMaterial } from '../core/AssetFactory.js';
import { worldToThree, headingToRotationY } from '../math/Coord.js';

/**
 * XxxView — 一句话描述
 * @param {THREE.Scene} scene  场景对象，由 SceneDirector 注入
 * @returns {{
 *   build?:  (ctx) => void,            // 静态 View 用：roadNetwork 变了才调
 *   update?: (store, simTime?) => void, // 动态 View 用：每帧由 Layer 树递归调
 *   clear:   () => void,                // 必选：清空所有实例（场景重建时调）
 *   dispose?: () => void,               // 可选：销毁（Layer.dispose 递归调；无则退化为 clear）
 * }}
 */
export function createXxxView(scene) {
  // 闭包状态：实例池、共享几何体、共享材质
  const pool = new Map();  // id → { group, ... }

  // 可选：静态布局 build（静态 View 用，如路灯/护栏沿路布置）
  // roadNetwork hash 变了才被 SceneDirector 调用，不走每帧路径
  function build(rn) {
    // 用 InstancedMesh 沿 edge 节点布置，sampleEdgeNodes 内部已做 ENU→THREE 交换
    // 直接用 p.x/p.z，不要再调 worldToThree（否则双交换）
  }

  // 必选：每帧 update（动态 View 用，由 Layer 树递归调）
  function update(store, simTime) {
    // 1. 从 store 读数据（entities/ego/roadNetwork）
    const all = (store.entities || []).filter(e => e.type === 'xxx');
    // 2. diff：删除消失的实例
    const aliveIds = new Set(all.map(e => e.id));
    for (const [id, entry] of pool.entries()) {
      if (!aliveIds.has(id)) { scene.remove(entry.group); pool.delete(id); }
    }
    // 3. 创建/更新
    for (const ent of all) {
      let entry = pool.get(ent.id);
      if (!entry) { entry = _createOne(ent); pool.set(ent.id, entry); }
      _updateOne(entry, ent, simTime);
    }
  }

  // 必选：清空所有实例（场景重建 / Layer.clear 时调）
  function clear() {
    for (const [, entry] of pool) scene.remove(entry.group);
    pool.clear();
  }

  // 可选：销毁（Layer.dispose 递归调；若无此方法，Layer 会退化为调 clear）
  // 释放 geometry/material 资源（geometry.dispose() / material.dispose()）
  function dispose() {
    clear();
    // 释放共享 geometry/material
  }

  function _createOne(ent) { /* 构建 THREE.Group 并 scene.add */ }
  function _updateOne(entry, ent, simTime) { /* 位姿 + 状态 */ }
  function _setLight(entry, state) { /* 灯/动作切换 */ }

  return { build, update, clear, dispose };  // 按需导出
}
```

**View 分两类**：
- **静态布局 View**（road/ground/connector/streetlight/barrier/viaduct）— 只 `build`，
  roadNetwork hash 变了才重建。不挂 Layer 树的 update 路径（无 update 方法被跳过）。
- **动态更新 View**（vehicle/trafficLight/etcGate）— 每帧 `update`，
  挂到 Layer 树由 `tickAnimation → rootLayer.update` 递归调用。

## 3. SceneStore 数据契约（View 只读）

```javascript
store = {
  roadNetwork: {                  // 道路网络（hash 变化才重建）
    edges: [{
      id, type, length_m, lanes, lane_width, speed_limit,
      nodes: [[x,y], ...],        // 路段节点
      curvature_profile, traffic_lights, etc_gates, junctions, ...
    }],
    junctions: [...]
  },
  roadHash: '...',                // diff 检测用

  ego: {                          // ego 车（单车）
    x, y, heading, speed, steer, brake, throttle,
    lights,                       // 车灯位掩码（见第 6 节）
    vx, vy, length, width
  },

  entities: [{                    // 所有动态实体（车辆/行人/红绿灯/ETC）
    id, type,                     // type: 'car'|'suv'|'truck'|'pedestrian'|'tl'|'etc_gate'|...
    x, y, heading, speed,
    lights, ai_state,             // ai_state: 'cruise'|'cutin'|'yield'|'stop'|...
    length, width, vx, vy,
    state,                        // 红绿灯/ETC 专用：'red'/'green'/'open'/'closed'
    progress, remain_s, parked
  }],

  env: { isNight, lighting },
  perfTier: 'high'|'medium'|'low',
}
```

## 4. 坐标系映射（极易踩坑）

FlowEngine 仿真用 **ENU 世界坐标**（x=East, y=North, z=Up）。
THREE.js 默认 **y=up**，所以映射：

| 仿真 | THREE | 说明 |
|------|-------|------|
| `ent.x` | `group.position.x` | 前向（East） |
| `ent.y` | `group.position.z` | 横向（North） |
| `0`（地面） | `group.position.y` | 高度（Up） |
| `ent.heading` (rad) | `group.rotation.y = -heading - π/2` | 0=朝 +x |

**使用 `headingToRotationY(heading)` 转换**，不要手写公式。

示例：
```javascript
group.position.set(ent.x, 0, ent.y);  // y=0 是地面
group.rotation.y = headingToRotationY(ent.heading || 0);
```

## 5. 资源约束（性能红线）

| 项 | 上限 | 推荐做法 |
|----|------|---------|
| 总面数 | 50,000 | 用 `getBox/getCylinder` 共享几何体 |
| Draw call | < 500 | 同质 mesh 用 `mergeGeometries`（`vis/math/GeometryMerge.js`）|
| 灯光数 | < 16 | 车灯用 `emissiveIntensity`，**不要**用 SpotLight |
| 材质实例 | 谨慎 | 静态材质用 `getStdMaterial`（缓存），动态发光用 `createEmissiveMaterial`（独立） |

**材质规范**：
- 一律用 `MeshStandardMaterial`（PBR）
- 颜色用 `0xRRGGBB` 数字格式
- 发光物体（车灯/信号灯/屏幕）用 `createEmissiveMaterial(color, intensity)`
- **不要**用 `MeshBasicMaterial`（不参与光照，违和）

**几何体工厂**（`vis/core/AssetFactory.js`）：
```javascript
import { getBox, getCylinder, getPlane, getStdMaterial, createEmissiveMaterial } from '../core/AssetFactory.js';

const bodyGeo = getBox(4.6, 1.4, 2.0);              // 缓存
const wheelGeo = getCylinder(0.32, 0.32, 0.25, 12); // 缓存
const bodyMat = getStdMaterial(0x1A528C, 0.6, 0.1); // 缓存
const headMat = createEmissiveMaterial(0xfff4d6, 1.0); // 独立（每帧改 intensity）
```

## 6. 车灯位掩码（车辆模块必读）

`ent.lights` 是 1 字节位掩码：

| bit | 值 | 含义 |
|-----|----|----|
| 0 | 0x01 | 左转向灯 |
| 1 | 0x02 | 右转向灯 |
| 2 | 0x04 | 双闪 |
| 3 | 0x08 | 远光灯 |
| 4 | 0x10 | 近光灯 |
| 6 | 0x40 | 倒车灯 |
| 7 | 0x80 | 雾灯 |

刹车灯由 `ent.brake > 0.05` 触发（不在位掩码里）。

**闪烁频率**：1.5 Hz（周期 ~666ms），用 `simTime / 1000` 作为相位传入。

## 7. 集成步骤（写完模块后必须做）

新架构下集成是插件式的 —— 改 SceneDirector.js 三处即可：

1. **创建文件**：`tools/flowboard/js/vis/view/XxxView.js`

2. **注册到 ViewRegistry**：编辑 `vis/director/SceneDirector.js` 顶部
   ```javascript
   import { createXxxView } from '../view/XxxView.js';
   ViewRegistry.register('xxx', createXxxView);
   ```
   （`instantiateAll(scene)` 会自动实例化）

3. **挂到 Layer 树**：在 `createSceneDirector` 内部的挂载表加一行
   ```javascript
   // 动态 View（每帧 update）→ agent/infra 层
   ['agent', ['vehicle', 'pedestrian']],    // 加到对应层
   
   // 静态 View（roadNetwork 变了才 build）→ env/road 层
   ['road', ['road', 'streetlight', 'barrier', 'connector', 'signpost']],
   ```

4. **静态 View 还需在 update() 的分支调 build**：
   ```javascript
   // 普通道路分支
   ViewRegistry.safeCall('xxx', 'build', rn);
   // 高架分支（如需跳过）
   ViewRegistry.safeCall('xxx', 'build', { edges: [] });
   ```

5. **缓存刷新**：编辑 `index.html`，把 `app.js?v=XXX` 改一个新版本号

6. **语法检查**：`node --check tools/flowboard/js/vis/view/XxxView.js`

**动态 View 不需要第 4 步** —— 挂到 Layer 树后，`tickAnimation → rootLayer.update`
会自动每帧调用，抛错只 log + 跳过，不传染兄弟。

**错误隔离保证**：即使新 View 抛错，整个 3D 场景不受影响 —— 其他 View 继续
渲染。这是 Layer 对象树 + ViewRegistry 的核心价值。

## 8. 设计 AI 提示词模板

给设计 AI 时，复制以下模板，替换 `<占位符>`：

---

```
你是 FlowEngine 可视化层的设计 AI。请按以下规范输出一个 View 模块。

## 任务
设计一个 <模块名，如：StreetlightView / PedestrianView / BarrierView> 模块：
<功能描述，如：路灯杆 6m + 灯臂 1.5m + 灯泡 emissive，根据 edge.type='urban' 自动沿路侧布置，间距 30m>

## 输入数据
从 store.entities 扫描 type='<entity type>' 的实体（或从 store.roadNetwork.edges 扫描 edge.<字段>）。
字段：<列出可读字段，如 x, y, state, ...>

## 输出格式
- 单文件 ES Module：`tools/flowboard/js/vis/view/<模块名>.js`
- 导出 `create<模块名>(scene)` factory，返回 `{ build?, update?, clear, dispose? }`
  - 静态 View（沿路布置）：返回 `{ build(rn), clear() }`
  - 动态 View（每帧更新）：返回 `{ update(store, simTime?), clear() }`
- 闭包内维护 `const pool = new Map()`，按 entity.id diff 管理 mesh 生命周期

## 集成方式（SceneDirector.js 改三处）
1. 顶部 `import { create<模块名> } from '../view/<模块名>.js';`
2. `ViewRegistry.register('<name>', create<模块名>);`
3. Layer 挂载表加一行：`['<layer>', [..., '<name>']]`
   - 动态 View 挂 agent/infra 层 → 每帧自动 update
   - 静态 View 挂 env/road 层 + update() 里调 `ViewRegistry.safeCall('<name>', 'build', rn)`
4. 错误隔离已内置：View 抛错只 log，不传染兄弟

## 强制约束
1. 只 import：`../core/AssetFactory.js`、`../math/Coord.js`、`../math/Curve.js`、`../math/GeometryMerge.js`
2. 坐标映射：`group.position.set(ent.x, 0, ent.y)` + `group.rotation.y = headingToRotationY(ent.heading||0)`
   - 或用 `worldToThree(ent.x, ent.y, ent.z||0)` 解构
   - sampleEdgeNodes 返回的节点已做 ENU→THREE 交换，不要再调 worldToThree
3. 材质：`getStdMaterial(color, rough, metal)` 缓存版；发光体用 `createEmissiveMaterial(color, intensity)` 独立版
4. 几何体：用 `getBox/getCylinder/getPlane` 共享缓存，不要 `new THREE.BoxGeometry` 重复创建
5. 总面数预算：<N，如 2000>；用低分段数（cylinder 12 段、box 1 段）
6. 不读 DOM / window / topoData；不创建全局变量
7. 不反向调 SceneDirector 或 Layer 方法（单向依赖）

## 参考实现
见 `tools/flowboard/js/vis/view/TrafficLightView.js`（124 行，标准模板）
见 `tools/flowboard/js/vis/view/ETCGateView.js`（207 行，复杂组装配式）
见 `tools/flowboard/js/vis/view/StreetlightView.js`（190 行，InstancedMesh 静态布局）
```

---

## 9. 已有模块清单（设计 AI 可参考）

| 模块 | 路径 | 复杂度 | 学习点 |
|------|------|-------|-------|
| GroundView | `vis/view/GroundView.js` | 简单 | 单 mesh，无 pool |
| TrafficLightView | `vis/view/TrafficLightView.js` | 中等 | 标准 pool + emissive 状态切换 |
| VehicleView | `vis/view/VehicleView.js` | 复杂 | gltf 优先 + 程序化 fallback + 车灯位掩码 |
| ConnectorView | `vis/view/ConnectorView.js` | 复杂 | 自动派生（无 entity，扫 edge 元数据） |
| ETCGateView | `vis/view/ETCGateView.js` | 复杂 | 多部件组装 + 动态 boom 抬起 |

## 10. 待设计模块清单（候选）

| 模块 | 数据源 | 功能 |
|------|-------|------|
| StreetlightView | edge.type='urban' | 沿城市路段两侧布置路灯（5m 杆 + 臂 + 灯泡） |
| BarrierView | edge.type='highway' | 高速护栏 + 防眩板 |
| PedestrianView | entity.type='pedestrian' | 行人模型（gltf 或程序化胶囊） |
| SignView | edge.signs[] | 交通标志牌（限速/匝道预警/ETC 预告） |
| JunctionView | rn.junctions[] | 路口区域（斑马线 + 停止线 + 导流线） |
| SkylineView | env | 远景天际线建筑剪影 |
| WeatherView | env.weather | 雨/雪/雾粒子 |
