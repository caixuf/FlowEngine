# VIS 重构进度日志

> **状态**：Step 1–6 全部落地并 push 到 main；Phase 4 清理待启动
> **范围**：JS 可视化侧（`tools/flowboard/js/vis/`）增量重构
> **配套文档**：[VIS_REARCH.md](./VIS_REARCH.md)（重构方案 spec，Phase 1–4 总览）
>
> **核心原则**：增量、可回滚、commit 即进度记录。
> AI 会话记忆有限，**重构进度以 git log 为准**，本文档是 git log 的人类可读索引。

---

## 0. 为什么 3D 反复重构都失败

历史重构 3–4 次，每次都重复同一循环：
1. 写一份"完美"新方案
2. 实现 60%
3. 删旧代码
4. 上线发现新 bug
5. 再起一次重构

根因不在代码质量，而在**迭代策略**：
- 一次性大爆炸式重构 → 没有"中间可工作"状态 → 出 bug 后回退无处可去
- 进度只在 AI 会话记忆里 → 会话截断后丢失 → 下次又从头开始
- 没有"小步可测"的回归保护 → 改一行 Coord.js / RoadView 就悄无声息破坏朝向

本次重构明确改变策略：
- **小步快走**：拆 6 个 Step，每个 Step 独立可 commit、可回滚、可回归测
- **commit 是进度记录**：commit message 写明本步做了什么、下一步做什么
- **每步都跑测试**：测试通过才进下一步
- **每步做完立即 push**：不依赖本地 commit（本地 sandbox 可能被重置）
- **本文档是 git log 索引**：会话截断后从 git log + 本文档继续

---

## 1. 总览：6 个 Step

| Step | 标题 | Commit | 文件改动 | 测试 |
|---|---|---|---|---|
| 1 | SceneDirector 补数据校验与 warn 日志 | `da0d554` | 1 改 | — |
| 2 | 死推算下沉 SceneDirector，deadreckon.js 改 shim | `5a316e3` | 3 改 1 新 | — |
| 3 | 启用 Coord.js 统一坐标系映射 | `85ca96b` | 4 改 | `vis_geometry.test.mjs`（已存在） |
| 4 | 补 StreetlightView + BarrierView（数据驱动） | `7c4fdf7` | 1 改 + 2 新 | — |
| 5 | 抽取纯函数 + 补冒烟测试 | `f1807a9` | 2 改 + 2 新模块 + 2 新测试 | 3 套 69 case 全过 |
| 6 | 文档化整个重构进度（本文档） | 本次 | 1 新 | — |

---

## 2. 详细记录

### Step 1 — SceneDirector 补数据校验与 warn 日志（`da0d554`）

**问题**：后端发坏 JSON 时前端静默继续渲染（用 `|| 0` 兜底），用户看不到原因。20Hz tick 下坏字段每帧报错会刷屏。

**改动**：
- `SceneDirector.js` 新增 `_warnOnce(key, msg)` —— 同一 key 只 warn 一次
- 8 项 schema 校验：topoData / frame / road_network / edges / ego / ego.x / entities / entity.type
- 校验失败不抛错（保持向后兼容），只 warn + 设 `skipXxx` flag
- 新增 `resetWarnings()`（测试 / 切场景用）

**关键决策**：
- 不用 try/catch —— 不是异常，是数据问题，warn 比抛错更友好
- warn key 用字段路径（如 `'entities[3].type'`），不是序号 —— 切场景时不会被旧 key 干扰
- `_warned` Set 在 `createSceneDirector` 闭包里 —— 跨 update() 调用持久，但跨场景隔离

---

### Step 2 — 死推算下沉 SceneDirector，deadreckon.js 改 shim（`5a316e3`）

**问题**：`main.js` 行 104-112 直接 `import deadreckon` + 覆盖 `store.ego.x/y/heading/speed`，违反"数据单向流：Director → Store → View"。数据流断裂导致：
- View 不知道 ego 是 sim 值还是 smooth 值
- 2D / 3D 各自调死推算，可能不同步

**改动**：
- `deadreckon.js` → `vis/core/DeadReckon.js`（路径变化 + 算法零改动）
- `SceneDirector.js` 新增 `tickAnimation(now)`：调 `tickDeadReckon()` + 把 `_dr.smooth*` 写回 `store.ego`
- `main.js` 渲染循环只调 `director.tickAnimation(now)`，不再直接碰 deadreckon / store.ego
- 旧 `deadreckon.js` 改为 re-export shim（`app.js` / `scene2d.js` 仍从 `./deadreckon.js` import，Phase 4 一起清理）
- 2D（`scene2d.js`）仍直接调 `tickDeadReckon` + 读 `_dr.smooth*`，与 3D 共享同一 `_dr` 全局单例

**关键决策**：
- `_dr` 仍是模块级单例（不是 SceneDirector 实例字段）—— 2D/3D 共享需要全局可见
- 算法不动（LAMBDA_POS=8, LAMBDA_HEADING=6，20Hz → 60fps 平滑参数已调好）
- 不抽接口 —— 只有一个实现，YAGNI
- 不直接删 deadreckon.js —— app.js / scene2d.js 还在 import，Phase 4 一起清理

**数据流图**（落地后）：
```
app.js sync2DTarget (SSE tick)
  → updateDeadReckon(x,z,speed,heading) → _dr
main.js 渲染帧
  → director.tickAnimation(now)
    → tickDeadReckon() 推进 _dr.smooth*
    → store.ego ← smooth*（覆盖 x/y/heading/speed，保留 _simX 等原始字段）
  → 各 View 读 store.ego 渲染
scene2d.js（2D）
  → tickDeadReckon() + 读 _dr.smooth*（与 3D 同一 _dr，保证同步）
```

---

### Step 3 — 启用 Coord.js 统一坐标系映射（`85ca96b`）

**问题**：ENU（仿真坐标）↔ THREE（右手系）的 y/z 交换散落 4 处，每处各写一遍 `position.set(ent.x, ent.z||0, ent.y)`。一旦某个 View 漏改或写反，车就钻地底或飞天上。

**改动**：4 处 ENU→THREE 转换统一用 `Coord.worldToThree(x, y, z)`：
- `TrafficLightView.js`：`entry.group.position.set(...worldToThree(ent.x, ent.y, 0))`
- `ETCGateView.js`：`entry.group.position.set(...worldToThree(ent.x || 0, ent.y || 0, 0))`
- `VehicleView.js`（gltf 路径）：`group.position.set(...worldToThree(ent.x, ent.y, ent.z || 0))`
- `VehicleView.js`（程序化路径）：同上

**两处未改（已核实不需要）**：
- `ConnectorView.js`：用 `start.x` / `start.z`，但 `start` 来自 `edge.nodes`，scene_pub 已经在 JSON 里输出 THREE 顺序（不是 ENU），所以 ConnectorView 是直接消费 THREE 坐标，不需要再交换。
- `Curve.js`（`sampleEdgeNodes`）：内部已做 ENU→THREE 交换（`out.push(a[0]+..., a[2]||0+..., a[1]+...)`）。RoadView 注释明确写"轴交换交给 sampleEdgeNodes"。

**Coord.js 清理**：
- 删除 `@deprecated` 标记（函数仍在用，标记容易误导）
- 补 usage 文档 + sampleEdgeNodes 例外说明

**回归保护**：`vis_geometry.test.mjs`（5 case：`Coord.headingToRotationY` + RoadView 顶点法线朝向）跑通无回归。

**关键决策**：不强行统一 ConnectorView / Curve.js —— 它们的输入约定不同（THREE 顺序 vs ENU），强行统一反而引入隐式交换 bug。

---

### Step 4 — 补 StreetlightView + BarrierView（`7c4fdf7`）

**问题**：Phase 3 View 模块清单 9 个，其中路灯 + 护栏 2 个未实现。仿真侧 `scene_pub.cpp` 只发道路几何（edges/nodes），不发路灯 / 护栏数据 —— 必须 3D 层从 `roadNetwork.edges` 自动布局。

**新增模块**：

`StreetlightView.js`（~190 行）：
- 4 个 InstancedMesh：pole / arm / head / glow
- `LAMP_SPACING = 40m`，道路两侧交替放置
- offset = `halfWidth + 1.5m`（路灯在路肩外）
- 内部跳过 `type === 'viaduct_highway'` 的 edge（高架场景路灯由 ViaductView 内置）

`BarrierView.js`（~180 行）：
- 3 个 InstancedMesh：1 个 post + 2 个 beam 层（护栏上下横梁）
- `POST_SPACING = 3m`，道路两侧对称
- offset = `halfWidth + 0.5m`（紧贴路肩）
- 同样跳过 viaduct edge

**SceneDirector 接线**：
- 导入 2 个新 View 工厂
- 普通道路分支：`streetlightView.build(rn)` + `barrierView.build(rn)`
- 高架分支：`streetlightView.build({edges:[]})` + `barrierView.build({edges:[]})` 显式 clear（避免切场景时残留）
- 新增 `getStreetlightView()` / `getBarrierView()` getter

**坐标约定**（重要，避免双交换 bug）：
- `edge.nodes` 经 `sampleEdgeNodes` 采样后输出已是 THREE 顺序
- （ENU→THREE 交换由 `sampleEdgeNodes` 内部完成，详见 Curve.js）
- 所以 StreetlightView / BarrierView 直接用 `p.x` / `p.z`，**不要再调 `worldToThree`**（否则会双交换让灯钻地底）

**关键决策**：
- 用 InstancedMesh 不用 mergeGeometries —— InstancedMesh 增删实例更灵活，frustum culling 友好
- 不发数据驱动模式（auto-layout from roadNetwork）—— 让 sim 侧保持简单，3D 层负责美化
- viaduct 跳过 —— ViaductView 已经有内置路灯 + 护栏，StreetlightView/BarrierView 在高架场景不画

**性能预算达成**：7 个 InstancedMesh 共增加 ~7 draw call，远低于 30 draw call 预算。

---

### Step 5 — 抽取纯函数 + 补冒烟测试（`f1807a9`）

**问题**：`VehicleView.deriveLightState` 和 `SceneDirector.validateFrame` 是纯逻辑（位掩码 → state、frame schema 校验），但藏在 THREE 依赖模块里，纯 Node 跑测试时 import 链拉 `window.THREE` 直接报错。

**改动**：抽出 2 个零 THREE 依赖的纯模块：

`vis/view/VehicleLights.js`（新）：
- `LIGHT_*` 位掩码常量（与 `flowsim/vehicle_lights.h` 一致）
- `deriveLightState(mask, brake)` 纯函数 → `{brake, turnL, turnR, head}`

`vis/director/FrameValidator.js`（新）：
- `validateFrame(topoData)` 纯函数 → `{ok, frame, rn, skipRoad, skipEgo, skipEntities, warnings}`
- 无副作用（不 console.warn），warnings 由调用方 emit

源模块 re-export 保持向后兼容（外部 import 路径不变）：
- `VehicleView.js` 从 `VehicleLights.js` re-export `LIGHT_*` + `deriveLightState`；
  `_updateGltfVehicle` 改调 `deriveLightState`（删 inline state 字面量）
- `SceneDirector.js` 从 `FrameValidator.js` re-export `validateFrame`，`update()` 改为
  调纯函数 + 遍历 warnings 调 `_warnOnce`；删除已不用的 `_typeOf` helper

**新增测试**：

`tests/vis_vehicle_lights.test.mjs`（18 case）：
- 6 个常量值（与 vehicle_lights.h 对齐）
- brake 阈值边界（0.05 不触发，0.06 触发）
- 6 个单独位（TURN_LEFT/RIGHT, HAZARD, LOW/HIGH_BEAM, REVERSE）
- 2 个组合位
- 全位 OR（验证 REVERSE 不污染 state.head）

`tests/vis_director_validation.test.mjs`（46 case）：
- topoData 顶层：null / undefined / 字符串 / 数字
- frame 解包：`{scene: null}` 等价未指定 wrapper、`{scene: 'oops'}` / `{scene: 42}` 报错、`metrics.scene` 解包路径
- road_network：非 object / edges 非数组 / edges 空数组（允许）
- ego：非 object / x 非数字 / x=NaN / x=Infinity / heading 非数字 / lights 非数字 / 缺省字段
- entities：非数组 / 元素缺 type / ego 元素跳过 / index 顺位
- 组合多 warning 一次返回

**已发现的契约 quirk（已记入测试注释）**：
- `{scene: null}` 和 `{metrics: {scene: null}}` 都不触发 `frame.type` warning —— wrapper 检测用 truthy 判断，`null` 等价"未指定 wrapper"，frame 回退到 `topoData` 本身。这是 Step 1 落地时的现有行为，不收紧。
- `ego.x` 非数字时仅 warn，不设 `skipEgo=true` —— 与原始 `|| 0` 兜底逻辑保持一致。

**3 套 69 case 全过**：
```
node tests/vis_vehicle_lights.test.mjs       → 18 pass, 0 fail
node tests/vis_director_validation.test.mjs  → 46 pass, 0 fail
node tests/vis_geometry.test.mjs            → 5 pass, 0 fail（回归）
```

**关键决策**：
- re-export 保持向后兼容 —— 调用方无需改 import 路径，避免触发不相关的破坏性改动
- 测试用 `node` 直接跑，不引入 vitest/jest —— 依赖最小化，CI 友好
- 纯函数不抛错，返回 `{ok, warnings}` —— 与原 `_warnOnce` 行为对齐，避免调用方 try/catch

---

### Step 6 — 文档化整个重构进度（本次提交）

**目的**：AI 会话记忆有限，git log 是机器可读进度，本文档是人类可读索引。新会话只需读本文档 + git log 即可继续。

---

## 3. 后续工作（未完成）

### Phase 4：清理（VIS_REARCH.md §8 Phase 4，本次未做）

Step 2 已经把 `deadreckon.js` 改成 re-export shim（指向 `vis/core/DeadReckon.js`），其余清理待启动：

- [ ] 删除 `scenarios/` 下废弃场景文件（保留 2 个核心）
- [ ] 删除 `tools/flowboard/js/scene3d.js`（~3000 行 God Object）
- [ ] 删除 `tools/flowboard/js/scene3d/`（10 个未真正使用的子文件）
- [ ] 删除 `tools/flowboard/js/scene3d_v2/`（未启用的阉割版）
- [ ] 删除 `tools/flowboard/js/deadreckon.js` shim，让 `app.js` / `scene2d.js` 直接从 `vis/core/DeadReckon.js` import
- [ ] 验证：全场景跑通，无残留引用，build 无 warning

### 测试覆盖扩展（可选，建议下个 Step）

- [ ] `tests/vis_deadreckon.test.mjs`：死推算 LAMBDA 参数 + 20Hz → 60fps 收敛测试
- [ ] `tests/vis_roadnetwork_hash.test.mjs`：`roadNetworkHash` diff 检测（同 edges 同 hash，不同 edges 不同 hash）
- [ ] `tests/vis_coord.test.mjs`：`worldToThree` + `headingToRotationY` 完整覆盖（现 vis_geometry 只测了 heading）

### 已知技术债

- `VehicleView._updateProceduralVehicle` 仍内联解析 mask（没复用 `deriveLightState`）—— 优先级低，程序化 fallback 路径只在 gltf 加载失败时走
- `validateFrame` wrapper 检测用 truthy，`{scene: null}` 不报错 —— 行为与 Step 1 一致，未收紧（注释里标记了）

---

## 4. 重构策略总结（供下次参考）

**有效的做法**：
1. **小步快走**：6 个 Step，每个独立可 commit、可回滚
2. **commit message 即进度记录**：写明本步做了什么、下一步做什么
3. **每步跑测试**：测试通过才进下一步，绿才 commit
4. **每步做完立即 push**：不依赖本地 commit（本地 sandbox 可能被重置）
5. **纯函数抽离**：把可测逻辑从 THREE 耦合模块抽出，纯 Node 可测
6. **re-export 保兼容**：内部重构不动外部 import 路径，避免破坏性改动
7. **本文档是 git log 索引**：会话截断后从 git log + 本文档继续

**避免的坑**：
1. 一次性大爆炸式重构 —— 没有"中间可工作"状态
2. 进度只在 AI 会话记忆里 —— 会话截断就丢
3. 没有"小步可测"的回归保护 —— 改一行就破坏朝向还看不出来
4. 抽象过度 —— 给纯逻辑套接口、用 vitest/jest，反而引入新依赖
5. 在没有测试保护下做"顺手清理" —— 容易破坏行为
6. 把进度只留在本地 commit —— sandbox 重置就全丢，必须 push 到远程

---

## 5. 参考

- 重构方案 spec：[VIS_REARCH.md](./VIS_REARCH.md)
- 可视化模块清单：[VIS_MODULE_GUIDE.md](./VIS_MODULE_GUIDE.md)
- 3D 调试 troubleshooting：[TROUBLESHOOTING_3D_DASHBOARD.md](./TROUBLESHOOTING_3D_DASHBOARD.md)
- 场景数据契约：[FLOWBOARD_SCENE_CONTRACT.md](./FLOWBOARD_SCENE_CONTRACT.md)

**Git 查看进度**：
```bash
git log --oneline | grep '\[重构'
```
