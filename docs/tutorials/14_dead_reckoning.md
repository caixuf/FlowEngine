# Skill 14：前端航位推算（Dead Reckoning）统一设计

## 场景

当后端以低频（如 10Hz SSE）推送离散的车辆位置/姿态数据，而前端需要以 60fps（rAF）渲染平滑、无卡顿的连续运动时，必须使用航位推算（Dead Reckoning）。

**典型场景：** FlowBoard 3D/2D 场景渲染、任何稀疏传感器数据的可视化、遥测仪表盘。

## 核心原则：单一引擎，多消费者

**反模式（Phase 3 之前的状态）：**
- `scene3d.js` 自己维护一套 lerp 逻辑
- `scene2d.js` 自己维护另一套 lerp 逻辑（参数不同）
- 两个模块都直接修改 `_dr.lastX/Z/Heading`，无单一 truth source
- `scene2d.js` 用 monkey-patch 污染 `updateAll()`

**正确模式（Phase 3 之后）：**

```
数据到达 (SSE ~10Hz)
    │
    ▼
app.js sync2DTarget()
    │
    ├─ updateDeadReckon(x, z, speed, heading)  ← 唯一 ground-truth 写入点
    │   (内部做：dedup + first-sample snap + init 标志)
    │
    └─ 每帧 rAF
        │
        ▼
    tickDeadReckon()
        ├─ 速度外推: target = last + speed * elapsed_since_last_data
        ├─ 帧率无关指数平滑: smooth += (target - smooth) * (1 - exp(-λ·dt))
        └─ 角度环绕: dh 归一化到 [-π, π]
        │
        ▼
    scene3d.js / scene2d.js / charts.js
        统一读取 getDeadReckonState() 返回的 smoothX / smoothZ / smoothHeading
```

**铁律：**
1. **只有一个模块**拥有航位推算状态（`_dr`）。
2. **只有一个函数**写入 ground-truth（`updateDeadReckon()`）。
3. **每帧调用一次** `tickDeadReckon()`，然后所有渲染器从同一个状态对象读取。
4. **绝不允许**渲染模块直接修改 `_dr.lastX/Z/Heading`。

## 帧率无关平滑公式

不要用固定 lerp 因子（如 `0.15`），它在 30fps 和 144fps 下表现完全不同：

```javascript
// ❌ 错误 — 帧率相关
var lerpF = 0.15;
smoothX += (targetX - smoothX) * lerpF;   // 30fps vs 144fps 差异巨大

// ✅ 正确 — 帧率无关
var alphaPos = 1 - Math.exp(-LAMBDA_POS * dt);      // λ = 8.0
var alphaHeading = 1 - Math.exp(-LAMBDA_HEADING * dt); // λ = 6.0
smoothX += (targetX - smoothX) * alphaPos;
```

- `dt` 是相邻两帧的真实时间差（秒），从 `performance.now()` 计算。
- **dt 必须钳位**（如 `dt > 0.1 ? 0.1 : dt`），防止浏览器 tab 后台挂起 10s 后切回导致车辆弹射。
- λ 越大 = 响应越快。建议位置 λ=8，heading λ=6（heading 需要更稳重，避免噪声引起的抖动）。

## 角度环绕（Angle Wrapping）

**P0 Bug — 航向角跨 ±π 时整圈旋转：**

```javascript
// ❌ 错误 — heading 从 3.13 跳到 -3.13 时，差值 = -6.26，车会整圈旋转
smoothHeading += (targetHeading - smoothHeading) * alpha;

// ✅ 正确 — 最短路径差值
var dh = targetHeading - smoothHeading;
while (dh > Math.PI)  dh -= 2 * Math.PI;
while (dh < -Math.PI) dh += 2 * Math.PI;
smoothHeading += dh * alphaHeading;
```

## 速度外推（True Dead Reckoning）

数据到达时记录 `lastX/Z/Speed/Heading/Time`。每帧：

```javascript
var elapsed = now - lastTime;     // 距上次数据的时间
if (elapsed > 2.0) elapsed = 2.0; // 超过 2s 认为 stale，停止外推

var targetX = lastX + lastSpeed * elapsed;   // 假设匀速直线
var targetZ = lastZ;                          // 本系统横向速度=0
```

外推目标 + 平滑收敛 = 视觉上的"预测-修正"效果。

## 首帧 Snap

首次收到数据时，不要把 `_dr.smoothX` 从 `(0,0)` lerp 到目标位置（车辆会从原点飞过来）。**直接 snap：**

```javascript
if (!_dr.init) {
    _dr.smoothX = x;          // 直接设为 ground-truth
    _dr.smoothZ = z;
    _dr.smoothHeading = heading;
    _dr.init = true;
}
```

## 代码模板

```javascript
// deadreckon.js — 单一引擎
export var LAMBDA_POS = 8.0;
export var LAMBDA_HEADING = 6.0;

export var _dr = {
  lastX: 0, lastZ: 0, lastSpeed: 0, lastHeading: 0, lastTime: 0,
  targetX: 0, targetZ: 0, targetHeading: 0,
  smoothX: 0, smoothZ: 0, smoothHeading: 0, smoothSpeed: 0,
  lastFrameTime: 0, init: false
};

export function updateDeadReckon(x, z, speed, heading) {
  var now = performance.now() / 1000;
  if (!_dr.init ||
      Math.abs(x - _dr.lastX) > 0.01 ||
      Math.abs(z - _dr.lastZ) > 0.01 ||
      Math.abs(speed - _dr.lastSpeed) > 0.1) {
    _dr.lastX = x; _dr.lastZ = z; _dr.lastSpeed = speed; _dr.lastHeading = heading;
    _dr.lastTime = now;
    if (!_dr.init) {
      _dr.smoothX = x; _dr.smoothZ = z; _dr.smoothHeading = heading; _dr.smoothSpeed = speed;
      _dr.init = true;
    }
  }
}

export function tickDeadReckon() {
  if (!_dr.init) return;
  var now = performance.now() / 1000;
  if (_dr.lastFrameTime === 0) _dr.lastFrameTime = now;
  var dt = now - _dr.lastFrameTime;
  if (dt > 0.1) dt = 0.1;    // 钳位！
  if (dt <= 0) { _dr.lastFrameTime = now; return; }
  _dr.lastFrameTime = now;

  // 速度外推
  var elapsed = now - _dr.lastTime;
  if (elapsed > 2.0) elapsed = 2.0;
  _dr.targetX = _dr.lastX + _dr.lastSpeed * elapsed;
  _dr.targetZ = _dr.lastZ;
  _dr.targetHeading = _dr.lastHeading;

  // 帧率无关平滑
  var aPos = 1 - Math.exp(-LAMBDA_POS * dt);
  var aHdg = 1 - Math.exp(-LAMBDA_HEADING * dt);
  _dr.smoothX += (_dr.targetX - _dr.smoothX) * aPos;
  _dr.smoothZ += (_dr.targetZ - _dr.smoothZ) * aPos;
  _dr.smoothSpeed += (_dr.lastSpeed - _dr.smoothSpeed) * aPos;

  // 角度环绕
  var dh = _dr.targetHeading - _dr.smoothHeading;
  while (dh > Math.PI)  dh -= 2 * Math.PI;
  while (dh < -Math.PI) dh += 2 * Math.PI;
  _dr.smoothHeading += dh * aHdg;
}

export function getDeadReckonState() { return _dr; }
```

```javascript
// scene3d.js — 纯消费者（绝不写入 _dr）
import { tickDeadReckon, getDeadReckonState } from './deadreckon.js';

function _renderFrame() {
  tickDeadReckon();
  var st = getDeadReckonState();
  ego.position.set(st.smoothX, 0, st.smoothZ);
  ego.rotation.y = st.smoothHeading;
  // 障碍物用 st.lastX/Z 做世界锚定（非 smooth，因为障碍物位置是离散的）
}

function update3D() {
  // 不再直接改 _dr！app.js 的 sync2DTarget 已经统一 feed 了
  // 只负责障碍物/LiDAR 世界坐标转换
}
```

## 参考

- 实现：[deadreckon.js](file:///workspace/tools/flowboard/js/deadreckon.js)
- 使用方：[scene3d.js](file:///workspace/tools/flowboard/js/scene3d.js)、[scene2d.js](file:///workspace/tools/flowboard/js/scene2d.js)
- 馈入点：[app.js sync2DTarget()](file:///workspace/tools/flowboard/js/app.js)
- 架构文档：[VISUALIZATION_ARCHITECTURE.md](file:///workspace/docs/VISUALIZATION_ARCHITECTURE.md)
