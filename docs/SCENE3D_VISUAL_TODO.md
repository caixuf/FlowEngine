# 3D 场景视觉 — 待修问题交接（2026-07-19）

> Claude 已定位根因并做了部分试改（见"工作树状态"）。用户接手继续调。
> 参考目标：用户提供的低多边形卡通街景（浅灰沥青、白虚线+黄双实线、黑护栏、
> 锥形树、低多边形远山、蓝渐变天空+扁平白云）。

---

## 第二轮：车辆动画/灯光/状态修复（已改完，语法通过）

| 修复 | 文件 | 说明 |
|------|------|------|
| NPC+ego 轮滚动角速度半径错误 | scene3d.js / utils.js / models.js | 旧代码统一用 `0.17*L`（unit 模型时代遗留）；realScale 模型 scale.x=1，轮半径与车长无关，车轮慢 2~3 倍。现建模时写入 `userData.wheelRadius`（sedan 0.33 / 卡车 0.42 / glTF 按前轮包围盒量取），滚动用 `v·dt / wheelRadius` |
| 类型切换重建后车轮不转/车灯灭/车身拉伸 | scene3d.js `~2504` | 重建只搬 children 没迁 userData，`wheels` 指向已 dispose 旧轮、`realScale` 不更新。现迁移 8 个键：wheels/frontAxle/rearAxle/brakeLights/turnSignals/headlights/realScale/wheelRadius |
| NPC 面板 id 不稳定 | scene3d.js update3D | `_obsWorld` 补 `id: ent.id`（面板本就 `ow.id \|\| idx`，之前恒 fallback 槽位号） |
| 卡车灯光接入 `_setVehicleLights` | utils.js `_buildObstacle` | 灯 mesh 命名 headlight_/brakelight_/turnsignal_ + userData 引用；补四角琥珀转向灯 |
| NPC 挂 48 个 SpotLight 性能灾难 | utils.js `_buildSedan(addSpots)` | 第三参 false 时不挂 SpotLight；ego 默认 true |
| 车道中心双黄线弯道消失 | scene3d/view/RoadView.js | `userData.baseZ` 未存 → NaN；已补 |
| lat→plat ReferenceError | scene3d.js `~478` | 水坑生成崩溃导致道路网络构建中断 |
| context 恢复后场景不重建 | scene3d.js init | 新 scene 前清空 roadNetwork/water/rain/sky 等模块级引用与缓存键 |
| 天空球/太阳光晕不随 ego | scene3d.js `_renderFrame` | `_skyMesh`/`_sunSprite` 每帧跟随 ego 位置 |

- 4 个改动文件 `node --check --input-type=module` 全部通过。

---


## 工作树状态（未提交）

`tools/flowboard/js/scene3d.js` 有以下**未提交**改动（黑屏修复、NPC Frenet 已各自单独 commit）：

| 改动 | 位置 | 状态 |
|------|------|------|
| NPC 车身晃动修复（tilt 喂平滑 yaw） | `~2237` | 建议**保留** |
| 沥青基色 `#7a7d80→#3f4247` | `_makeAsphaltTexture` `~209` | 部分改善，主观，可继续调/回退 |
| 沥青骨料颗粒加深加密 | `~215–240` | 同上 |
| roadMat `bumpScale 0.02→0.035`、`roughness 0.85` | `~600` | 同上 |
| 胎痕底色匹配新沥青 | `~1184` | 同上 |
| 方向光 `1.75→1.45` | `~1642` | 部分改善，可继续降 |

- 看当前效果：`bash scripts/demo.sh --no-browser 60`，浏览器 `localhost:8800` **硬刷新**（scene3d.js import 无 `?v=`，会死缓存）。
- 只回退视觉试改、保留 NPC 晃动修复：先 `git stash`，挑出晃动那处再 `git stash pop`；或手动改回上表各行。

---

## 问题 1：NPC 车身晃来晃去（roll 尖峰）

- **现象**：NPC 行驶/过弯时车身左右猛晃（"好搞笑"）。原来 NPC 直线跑不晃，Frenet 让它真的会转向后才暴露。
- **根因**：`_renderFrame` 障碍物 tilt 原为
  `ti.updateTargets(ow.t0, ow.heading, ...)`。其中 `ow.heading` 是后端原始航向，
  只在 10–20Hz **数据帧阶跃**变化；`ow.t0` 又被设成每帧 `now`。于是
  `yawRate = 阶跃航向差 / 一帧 dt` 被放大数倍 → VisualPhysics 的
  `roll = -yawRate·speed·gain` 尖峰 → 回弹 → 晃。ego 不晃是因为它喂
  `_dr.smoothHeading`（每帧平滑）。
- **已试改**（`~2237`）：改喂平滑后的 yaw：`ti.updateTargets(now, -om.rotation.y, ...)`，
  与 ego 一致。**待你肉眼确认是否彻底不晃**。若 pitch 仍点头，同理把 speed 也做平滑再喂。

---

## 问题 2：路面看不出沥青（被强光冲白）

- **现象**：路面苍白、和草地糊在一起，不像沥青。
- **排查结论**：**不是 UV/纹理没贴**——路面 UV 是 world-space `(x/4, z/4)`、
  `_makeAsphaltTexture` 正确应用（`curl localhost:8800/js/scene3d.js` 已确认服务端发的是新代码）。
  真因是**光照 ≈2×**：`DirectionalLight 1.75 + AmbientLight 0.32 + HemisphereLight 0.4`，
  把浅灰沥青（原 `#7a7d80`≈0.48 反照率）冲到 ~0.9 → 发白发平。
- **可调旋钮**（都在 `scene3d.js`）：
  - 沥青基色：`_makeAsphaltTexture()` 的 `ctx.fillStyle`（`~209`）。已试改 `#3f4247`。
  - 方向光强度：`new THREE.DirectionalLight(0xfff8ee, X)`（`~1642`）。已试改 `1.45`。
  - 环境光/半球光：`AmbientLight(0xaabbdd, 0.32)`（`~1641`）、`HemisphereLight(..., 0.4)`（`~1661`）。
  - 路面材质：`roadMat` 的 `color/roughness/bumpScale`（`~600`）。
- **建议方向**：想要参考图那种"**亮但不曝**的浅灰"，核心是**降光**，而不是一味加深路面
  （加深会偏离参考图的浅色调）。可先把 `Directional` 降到 1.1–1.3、`Ambient` ~0.2，
  再把沥青基色调回中灰（`#55585e` 一带），对着参考图微调。
  远山/草地/天空目前也偏曝，同一降光会顺带改善整体观感。

---

## 顺带记录（相关但非本次，单独立项）

- **chase 相机**用固定 +x 朝向、不跟 ego 航向/弯道 → 取景对不准参考图那种"跟车顺弯"。
  位置：`_renderFrame` chase 分支 `_camLookTarget.set(sx + 8, 0.5, sz)`（应改为按 heading 投影）。
- 远处 **skyline 深色方块（建筑）** 缩放偏大偏近，`_buildSkyline` 可调。
- **ego 偏匝道**（planning/control 横向没跟道路几何）——`#6`，动 C++，单独一轮。
