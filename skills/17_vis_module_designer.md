# vis 模块设计 — 让 AI 按规范输出可无缝集成的 View 模块

> 设计并生成 `tools/flowboard/js/vis/view/` 下的新 View 模块（路灯/护栏/行人/标志/ETC 等）。
> 按本规范输出的模块可以零修改集成进 SceneDirector 的 Layer 对象树。

## 架构概览（Qt 对象树 + 插件化）

```
SceneDirector
  ├─ ViewRegistry.register('xxx', createXxxView)   # 注册工厂
  ├─ instantiateAll(scene)                          # 批量实例化
  └─ Layer 树（Qt 对象树 + 错误隔离）
      root
      ├── env     (ground, viaduct)              — 环境层
      ├── road    (road, streetlight, barrier, connector) — 道路层
      ├── agent   (vehicle)                       — 智能体层
      └── infra   (trafficLight, etcGate)         — 路侧设施层
```

**核心价值**：
- **错误隔离** — 一个 View 抛错只 log + 跳过，兄弟继续渲染（解药："一个模块坏了整个 3D 就坏了"）
- **插件化** — 新增 View 只需 `register + addView`，不动 SceneDirector 主体
- **递归 dispose** — Layer 树自动递归清理所有 View 资源

完整架构见 [docs/VISUALIZATION_ARCHITECTURE.md](../docs/VISUALIZATION_ARCHITECTURE.md)。

## 何时使用

- 用户说："加个路灯模块"、"新增行人 View"、"设计个护栏模块"、"vis/ 里加个 XX"
- 用户说："让设计 agent 给我设计个 XX"、"用提示词模板生成 XX 模块"
- 用户说："我想加个 XX 但不知道怎么接进 vis"

**不触发**：
- 修改已有 View 模块（直接编辑文件即可）
- 后端仿真模块（不归 vis 管）
- 3D 场景调试（使用浏览器开发者工具 + window.__vis 接口）

## 执行流程（必按此顺序）

### Step 1：澄清需求（用 AskUserQuestion）

收到"加个 XX 模块"后，先问清楚 3 件事：

1. **数据源**：模块从哪读数据？
   - `store.entities` 里 `type='xxx'` 的实体（动态对象，如车辆/行人）
   - `store.roadNetwork.edges` 元数据（静态沿路布置，如路灯/护栏按 edge 类型）
   - `store.roadNetwork.junctions`（路口元素，如斑马线/停止线）
   - `store.env`（环境元素，如天气/天际线）

2. **触发条件**：模块何时出现？
   - 全场景常驻（路灯/护栏）
   - 按 edge 类型（`edge.type='urban'` 才有路灯）
   - 按 entity 字段（`ent.state='red'` 切换红灯）

3. **面数预算**：模块总面数上限（默认 2000，复杂模块可放宽到 5000）

### Step 2：读取接口契约

**必读文件**（理解架构后再动手）：
- `tools/flowboard/js/vis/view/TrafficLightView.js` — 标准模板（124 行，emissive 状态切换）
- `tools/flowboard/js/vis/view/ETCGateView.js` — 复杂组装配式（207 行，多部件+动态 boom）
- `tools/flowboard/js/vis/core/AssetFactory.js` — 几何体/材质工厂
- `tools/flowboard/js/vis/math/Coord.js` — 坐标映射
- `tools/flowboard/js/vis/store/SceneStore.js` — 数据契约
- `docs/VIS_MODULE_GUIDE.md` — 完整规范文档

### Step 3：生成模块文件

按以下模板输出 `tools/flowboard/js/vis/view/XxxView.js`：

```javascript
/**
 * XxxView.js — 一句话描述
 *
 * 数据源：<store.entities 里 type='xxx' / store.roadNetwork.edges 元数据>
 * 触发条件：<何时出现/切换>
 */

import { getBox, getCylinder, getStdMaterial, createEmissiveMaterial } from '../core/AssetFactory.js';
import { headingToRotationY } from '../math/Coord.js';

export function createXxxView(scene) {
  const pool = new Map();  // id → { group, ... }

  function _createOne(ent) {
    const group = new THREE.Group();
    // ── 构建 mesh（用 getBox/getCylinder 共享几何体）──
    // ── 材质：静态用 getStdMaterial，发光体用 createEmissiveMaterial ──
    scene.add(group);
    return { group, /* 其他状态 */ };
  }

  function _updateOne(entry, ent, simTime) {
    entry.group.position.set(ent.x, 0, ent.y);
    entry.group.rotation.y = headingToRotationY(ent.heading || 0);
    // ── 状态切换（如 emissiveIntensity）──
  }

  /** 主更新入口：SceneDirector 每帧调用 */
  function update(store, simTime) {
    // 1. 从 store 读数据
    const all = (store.entities || []).filter(e => e.type === 'xxx');
    // 2. diff：删除消失的
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

  function clear() {
    for (const [, entry] of pool) scene.remove(entry.group);
    pool.clear();
  }

  return { update, clear };
}
```

### Step 4：接入 SceneDirector（插件式 — 改三处）

编辑 `tools/flowboard/js/vis/director/SceneDirector.js`：

1. **import + 注册**（顶部，其他 register 旁边）：
   ```javascript
   import { createXxxView } from '../view/XxxView.js';
   ViewRegistry.register('xxx', createXxxView);
   ```
   （`ViewRegistry.instantiateAll(scene)` 会自动建实例，不用手动 `createXxxView(scene)`）

2. **挂到 Layer 树**（`createSceneDirector` 内部的挂载表，表驱动）：
   ```javascript
   // 动态 View（每帧 update）→ agent/infra 层
   ['agent', ['vehicle', 'pedestrian']],    // 加 'xxx'
   
   // 静态 View（roadNetwork 变了才 build）→ env/road 层
   ['road', ['road', 'streetlight', 'barrier', 'connector', 'signpost']],
   ```

3. **静态 View 还需在 update() 分支调 build**（动态 View 不需要）：
   ```javascript
   // 普通道路分支
   ViewRegistry.safeCall('xxx', 'build', rn);
   // 高架分支（如需跳过）
   ViewRegistry.safeCall('xxx', 'build', { edges: [] });
   ```

**错误隔离已内置**：新 View 抛错只 log + 跳过，整个 3D 场景不受影响 ——
其他 View 继续渲染。这是 Layer 对象树 + ViewRegistry 的核心价值。
不再需要手动写 `getXxxView()` getter，ViewRegistry.get('xxx') 是单一来源。

### Step 5：刷新缓存 + 验证

1. **语法检查**：
   ```bash
   node --check tools/flowboard/js/vis/view/XxxView.js
   ```

2. **刷新版本号**：编辑 `tools/flowboard/index.html`，把 `app.js?v=XXX` 改新版本号（如 `20260722xxx`）

3. **告知用户验证**：让用户硬刷新 http://localhost:8800 查看效果

## 铁律（违反即拒绝合并）

### 1. 只能 import 这三个目录
```javascript
import { ... } from '../core/AssetFactory.js';   // 几何体/材质工厂
import { ... } from '../math/Coord.js';           // 坐标映射
import { ... } from '../math/Curve.js';           // 路曲线采样（沿路布置用）
```
**禁止**：
- `import ... from '../../models.js'`（gltf 模型，仅 VehicleView 可用）
- `import ... from '../../utils.js'`（旧代码，正在清退）
- 直接读 `window.*` / DOM / `topoData`

### 2. 坐标系映射（极易踩坑）

FlowEngine 仿真用 ENU 世界坐标（x=East, y=North, z=Up）。
THREE.js 默认 y=up，映射表：

| 仿真 | THREE | 说明 |
|------|-------|------|
| `ent.x` | `group.position.x` | 前向（East） |
| `ent.y` | `group.position.z` | 横向（North） |
| `0`（地面） | `group.position.y` | 高度（Up） |
| `ent.heading` (rad) | `group.rotation.y = headingToRotationY(heading)` | 0=朝 +x |

**正确写法**：
```javascript
group.position.set(ent.x, 0, ent.y);
group.rotation.y = headingToRotationY(ent.heading || 0);
```
**错误写法**（会位置/朝向错乱）：
```javascript
group.position.set(ent.x, ent.y, 0);    // ❌ y/z 颠倒
group.rotation.y = ent.heading;          // ❌ 没用 headingToRotationY
```

### 3. 资源约束（性能红线）

| 项 | 上限 | 推荐做法 |
|----|------|---------|
| 单模块面数 | 2000（默认）/ 5000（复杂） | 用 `getBox/getCylinder` 共享几何体 |
| Draw call | < 500 全场景 | 同质 mesh 用 `mergeGeometries`（`vis/math/GeometryMerge.js`）|
| 灯光数 | < 16 全场景 | 车灯/信号灯用 `emissiveIntensity`，**禁止** SpotLight |
| 材质实例 | 谨慎 | 静态用 `getStdMaterial`（缓存），动态发光用 `createEmissiveMaterial`（独立）|

### 4. 材质规范

- 一律用 `MeshStandardMaterial`（PBR）
- 颜色用 `0xRRGGBB` 数字格式（如 `0x1A528C`）
- 发光体（车灯/信号灯/屏幕）用 `createEmissiveMaterial(color, intensity)`
- **禁止** `MeshBasicMaterial`（不参与光照，违和）
- **禁止** `new THREE.BoxGeometry` 重复创建（用 `getBox` 共享缓存）

### 5. SceneStore 只读

View 模块**不能修改** store 字段，只能读。所有状态变更必须通过闭包内的 pool 管理。

## SceneStore 数据契约（View 只读）

```javascript
store = {
  roadNetwork: {
    edges: [{
      id, type, length_m, lanes, lane_width, speed_limit,
      nodes: [[x,y], ...],
      curvature_profile, traffic_lights, etc_gates, junctions
    }],
    junctions: [...]
  },
  roadHash: '...',
  ego: { x, y, heading, speed, steer, brake, lights, length, width },
  entities: [{
    id, type, x, y, heading, speed,
    lights, ai_state, length, width, vx, vy,
    state,  // 'red'/'green'/'open'/'closed'
    progress, remain_s, parked
  }],
  env: { isNight, lighting },
  perfTier: 'high'|'medium'|'low',
}
```

## 车灯位掩码（车辆模块必读）

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
闪烁频率 1.5 Hz，用 `simTime / 1000` 作为相位。

## 沿路布置模块（路灯/护栏专用）

如果模块需要沿 edge 等距布置（路灯每隔 30m 一个），用 `vis/math/Curve.js` 的 `sampleEdgeNodes`：

```javascript
import { sampleEdgeNodes } from '../math/Curve.js';

function build(roadNetwork) {
  for (const edge of roadNetwork.edges) {
    if (edge.type !== 'urban') continue;
    const samples = sampleEdgeNodes(edge, /*spacing_m=*/30);
    for (const p of samples) {
      // p = { x, y, heading }
      const lamp = _createLamp();
      lamp.position.set(p.x, 0, p.y);
      lamp.rotation.y = headingToRotationY(p.heading);
      scene.add(lamp);
    }
  }
}
```

## 已有模块清单（设计参考）

| 模块 | 路径 | 复杂度 | 学习点 |
|------|------|-------|-------|
| GroundView | `vis/view/GroundView.js` | 简单 | 单 mesh，无 pool |
| TrafficLightView | `vis/view/TrafficLightView.js` | 中等 | 标准 pool + emissive 状态切换 |
| VehicleView | `vis/view/VehicleView.js` | 复杂 | gltf 优先 + 程序化 fallback + 车灯位掩码 |
| ConnectorView | `vis/view/ConnectorView.js` | 复杂 | 自动派生（无 entity，扫 edge 元数据）|
| ETCGateView | `vis/view/ETCGateView.js` | 复杂 | 多部件组装 + 动态 boom 抬起 |

## 待设计模块候选

| 模块 | 数据源 | 功能 |
|------|-------|------|
| StreetlightView | edge.type='urban' | 沿城市路段两侧布置路灯（5m 杆 + 臂 + 灯泡）|
| BarrierView | edge.type='highway' | 高速护栏 + 防眩板 |
| PedestrianView | entity.type='pedestrian' | 行人模型（gltf 或程序化胶囊）|
| SignView | edge.signs[] | 交通标志牌（限速/匝道预警/ETC 预告）|
| JunctionView | rn.junctions[] | 路口区域（斑马线 + 停止线 + 导流线）|
| SkylineView | env | 远景天际线建筑剪影 |
| WeatherView | env.weather | 雨/雪/雾粒子 |

## 验证清单（生成后必查）

- [ ] `node --check tools/flowboard/js/vis/view/XxxView.js` 通过
- [ ] 只 import `../core/`、`../math/`（无 `../../` 跨目录）
- [ ] 坐标用 `headingToRotationY`，position 是 `(x, 0, y)`
- [ ] 材质用 `getStdMaterial` / `createEmissiveMaterial`（无 `new THREE.MeshStandardMaterial`）
- [ ] 几何体用 `getBox/getCylinder`（无 `new THREE.BoxGeometry`）
- [ ] SceneDirector.js 已 import + 创建实例 + 调 update + 导出 getter
- [ ] index.html 版本号已刷新
- [ ] 单模块面数 < 5000

## 提示词模板（给外部设计 AI）

如果用户想用外部设计 AI（如 Midjourney 出贴图、Blender 出 gltf），复制以下模板填空：

```
你是 FlowEngine 可视化层的设计 AI。请按以下规范输出一个 View 模块。

## 任务
设计一个 <模块名> 模块：<功能描述>

## 输入数据
从 store.<entities/roadNetwork> 扫描 <type/字段>。
字段：<列出可读字段>

## 输出格式
- 单文件 ES Module：`tools/flowboard/js/vis/view/<模块名>.js`
- 导出 `create<模块名>(scene)` factory，返回 `{ update(store, simTime?), clear() }`
- 闭包内维护 `const pool = new Map()`，按 entity.id diff 管理 mesh 生命周期

## 强制约束
1. 只 import：`../core/AssetFactory.js`、`../math/Coord.js`、`../math/Curve.js`
2. 坐标映射：`group.position.set(ent.x, 0, ent.y)` + `group.rotation.y = headingToRotationY(ent.heading||0)`
3. 材质：`getStdMaterial(color, rough, metal)` 缓存版；发光体用 `createEmissiveMaterial(color, intensity)` 独立版
4. 几何体：用 `getBox/getCylinder/getPlane` 共享缓存
5. 总面数预算：<N>
6. 不读 DOM / window / topoData；不创建全局变量

## 参考实现
见 `tools/flowboard/js/vis/view/TrafficLightView.js`（标准模板）
```
