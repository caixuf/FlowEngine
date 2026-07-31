# FlowEngine — 项目概览

轻量级自动驾驶中间件，核心是一个 Pub/Sub 消息总线 + 调度器 + 传输层。

> **开发流程：** 设计 → 执行 → 测试 → 迭代 → 清理 → 文档。详见 `.claude/skills/SKILL.md`（入口路由）。
> 改完代码后必跑：`/verify` → `/code-review` → `/simplify` → commit → 更新文档。
>
> **重构类改动** commit body 必含 `Removed:` 段（详见 `.claude/skills/workflow.md` 第七节），纯新增/fix/docs 可省略。

## 架构

```
sim_world → sensor_model → perception → fusion → planning → control → safety_control → monitor
     ↓            ↓             ↓           ↓          ↓          ↓            ↓            ↓
 vehicle/state  sensor/lidar perception/  fusion/  planning/  control/raw  control/cmd  dashboard
                sensor/gps  obstacles   localization trajectory  _cmd                     JSON
                     ↓             ↓           ↓          ↓          ↓            ↓
              ════════════════ Message Bus ════════════════════
                                    ↓
                             Transport (IPC/TCP) → Discovery → FlowRegistry
                                    ↓
                             flowmond (IPC stats bridge + HTTP/SSE) → DashBoard
```

## 关键文件

| 文件 | 作用 |
|------|------|
| `src/core/message_bus.c` | 进程内 Pub/Sub 总线 |
| `src/core/transport.c` | 统一传输抽象（local/IPC/TCP） |
| `src/core/scheduler.c` | 任务调度器（classic/choreo 模式） |
| `src/flowmond.c` | 监控守护进程（HTTP 仪表盘 + IPC 统计/仪表盘桥接 + 自动重连） |
| `src/core/monitor_server.c` | 内嵌 HTTP 服务器（多线程连接、/tools 静态资源、JSON 安全转义、过期缓存自动 fallback） |
| `src/core/stats_bridge.c` | 跨进程 topic 统计 IPC 桥接 |
| `src/core/dashboard_bridge.c` | 跨进程仪表盘 JSON IPC 桥接（分块传输协议） |
| `modules/adas_nodes/sensor_model_node.c` | 传感器模型（LiDAR/GPS/Camera，FOV/遮挡/噪声） |
| `modules/adas_nodes/safety_control_node.cpp` | FlowCoro 协程安全控制（TTC/横向交叉/行人防护） |
| `tools/flowboard/index.html` | 前端仪表盘（3D+2D+图表+D3 拓扑，ES modules） |
| `tools/foxglove_bridge.py` | Foxglove Studio WebSocket 桥接 |
| `ci/evaluators/demo_evaluator.py` | 回归评估器：采样 JSON 并自动评分（碰撞/偏航/停滞/频率） |
| `tools/pipeline_check.py` | 离线管道完整性检查：不启动 demo，秒级验证 9 类 32 项指标 |
| `tools/quick_verify.py` | 交互式调参验证工具：实时仪表盘 + 即时 eval 评分 |
| `ci/evaluators/test_param_regression.py` | 参数回归对比：保存 baseline，改参后自动检测退化 |
| `scripts/demo.sh` | 一键启动脚本 |
| `src/flow_launcher.c` | 配置驱动启动器（读取 pipeline.json，dlopen 加载插件节点） |
| `src/flowctl.c` | CLI 工具（list/inspect/dashboard/param/bag 等子命令） |
| `src/core/flow_registry.c` | 统一元信息注册中心（Task/Topic/Type/Plugin/Schema） |
| `src/core/param_registry.c` | 参数系统（int/float/bool/string，范围校验，hot-reload） |
| `src/core/param_bridge.c` | 参数跨进程通道（AF_UNIX，`flowctl param set` 边跑边调） |
| `src/core/scenario_loader.c` | 场景 JSON 加载器（actor 定义 + ego 配置） |
| `include/platform_compat.h` | 跨平台兼容层（macOS⇄Linux，CMake force-include，仅 APPLE 生效）：pthread 命名签名、robust mutex、condvar 时钟降级 |
| `modules/adas_nodes/flowsim/physics.cpp` | 运动学自行车模型；dynamics 桩（未实现，降级到运动学） |
| `modules/adas_nodes/flowsim/entity.h` | 仿真实体（含 v_x_body/v_y_body/yaw_rate/F_yf/F_yr 动力学桩字段，运行时未使用） |
| `scenarios/straight_road.json` | 直道场景（默认，唯一） |
| `modules/adas_nodes/data_recorder_node.c` | 训练样本采集（Learning Loop Stage 0） |
| `modules/adas_nodes/inference_node.cpp` | tiny-MLP 影子推理（Learning Loop Stage 2） |
| `modules/adas_nodes/tiny_mlp.h` | 纯 C 单隐层 MLP 推理内核 |
| `tools/train_demo_model.py` | 一站式训练入口（调度 `tools/train_e2e/` 各 backend） |
| `tools/train_e2e/{train,torch_train,temporal_train}.py` | tiny-MLP / PyTorch / 时序 训练实现 |
| `tools/modelctl.py` | artifact 管理（list / inspect / diff / promote / ota） |
| `docs/LEARNING_LOOP.md` | 车端学习闭环架构 |

> 深入教程见 `skills/` 目录（16 篇，覆盖 OOP in C、插件系统、消息总线、IPC、Bag、Clock、
> Serializer、State Machine、Discovery、Fusion、Coroutine、Demo Evaluator、E2E Learning Loop、
> Dead Reckoning、SocketCAN Actuator、FlowSim 场景设计）。

## 运行

```bash
bash scripts/demo.sh [duration]          # 启动演示
bash scripts/demo.sh --no-browser 15     # 不打开浏览器
```

> **平台：** Linux（主力/CI）与 macOS 原生均可 `bash scripts/demo.sh` 跑通。
> 平台差异由兼容层 `include/platform_compat.h`（force-include，仅 APPLE 生效）
> + CMake `if(APPLE)` 分支收口，**所有改动都在 `#ifdef __APPLE__` / `if(APPLE)`
> 内，Linux 行为零变化**。macOS 弱化项（仅影响非默认路径）：`--multi` robust-mutex
> 崩溃自愈降级、benchmark 不构建、CAN/I2C dry-run、无线程 CPU 亲和 —— 详见 README。

仪表盘: `http://localhost:8800`
3D 桥接: `ws://localhost:8765`

## 验证

```bash
# 每次改动 pipeline 链路上的节点后，跑评估器
python3 ci/evaluators/demo_evaluator.py --duration 45 --interval 0.5

# 秒级离线检查（不启动 demo）
python3 tools/pipeline_check.py

# 仅分析当前数据，不重新启动 demo
python3 ci/evaluators/demo_evaluator.py --no-run
```

评估器采样 `/tmp/flow_topology.json`，自动检查：拓扑完整性、topic 频率、碰撞、路沿偏离、停滞、变道次数、偏航抖动、NPC 瞬移。WARN 是已知问题可忽略，FAIL 必须修复。

## 编码规范（统一 API — 2026-07 重构后强制执行）

> 以下 API 是本项目的**唯一合法入口**。禁止绕过它们直接调用底层函数。

### JSON 序列化/反序列化 → `cjson/cJSON.h`

```c
#include <cjson/cJSON.h>

// 序列化（替代手拼 snprintf）
cJSON* root = cJSON_CreateObject();
cJSON_AddNumberToObject(root, "speed", v);
cJSON_AddStringToObject(root, "mode", "highway");
char* s = cJSON_PrintUnformatted(root);   // topic publish 用 compact
// char* s = cJSON_Print(root);           // 文件写入用 formatted
transport_publish(t, "topic", (const uint8_t*)s, (uint32_t)strlen(s) + 1);
free(s);
cJSON_Delete(root);

// 反序列化（替代 strstr + sscanf）
cJSON* root = cJSON_Parse((const char*)msg->data);
if (root) {
    cJSON* j = cJSON_GetObjectItemCaseSensitive(root, "speed");
    if (cJSON_IsNumber(j)) double v = j->valuedouble;
    cJSON_Delete(root);
}
```

- ❌ **禁止** `snprintf("{\"key\":%d}", ...)` 手拼 JSON
- ❌ **禁止** `strstr(msg, "\"key\":")` + `sscanf` 手写解析
- ✅ **必须** 用 cJSON 序列化/反序列化所有 topic 消息和文件
- 参考：`src/core/scenario_loader.c`（已有成熟 pattern）

### 时间 → `clock_service.h`

```c
#include "clock_service.h"

uint64_t now = clock_now_us();               // CLOCK_MONOTONIC，仿真/回放模式下可注入
uint64_t real = clock_now_realtime_us();     // CLOCK_REALTIME，始终真实墙钟（训练样本、仪表盘）
```

- ❌ **禁止** `static uint64_t monotonic_us(void) { clock_gettime(...); }` 重复造轮子
- ❌ **禁止** `clock_gettime(CLOCK_MONOTONIC, &ts)` 裸调用
- ✅ **必须** 用 `clock_now_us()`；需绝对时间用 `clock_now_realtime_us()`

### 参数解析 → cJSON_Parse

```c
// 替代 strstr(params_json, "\"target_speed\":")
cJSON* p = cJSON_Parse(params_json);
cJSON* j = cJSON_GetObjectItemCaseSensitive(p, "target_speed");
if (cJSON_IsNumber(j)) cfg.target_speed = j->valuedouble;
cJSON_Delete(p);
```

- ❌ **禁止** `strstr(params_json, "\"param_name\":")` + `sscanf` 手写参数解析
- ✅ **必须** 用 `cJSON_Parse(params_json)` 解析所有节点参数

### 调参 → `flowctl param set`（不要改常量重编译）

```bash
flowctl param list                              # 运行中进程的实时参数
flowctl param get control.mpc_r_ddelta
flowctl param set control.mpc_r_ddelta 2.0      # 下一帧生效，不用重启
```

新增一个可调参数，**三处都要通，只做注册等于没做**：

1. `params_json` 里加 `cJSON_GetObjectItemCaseSensitive` 解析分支
   （漏了这步，pipeline.json 里那行就是死字符串）
2. `param_register_*` 的默认值用 `g.<字段>` 而非硬编码字面量
   （用字面量会把上一步解析到的值盖掉）
3. 逐帧 tick 里 `param_get_float` 重读
   （漏了这步，注册了也改不动，只能重启）

- ❌ **禁止** 为了试一个值去改代码常量重新编译
- ✅ 整段 run 的聚合指标 A/B（avg_speed / flip_rate）用 `tools/auto_tune_mpc.py` 或 `ci/evaluators/scenario_regression.py`，
  它每个取值重启一次是必需的 —— 要可比就得从 x=0 起跑干净的 run

### 节点线程 → `node_pump()`

```c
flowcoro::rt::RtExecutor ex{{ .pin_cpu=-1 }};
ex.spawn(ct.run(), "node_name");
node_pump(ex, [] { return (bool)g.should_stop; });
ex.shutdown();
```

- ❌ **禁止** 裸 `while (!g.should_stop) ex.run();` —— `run()` 是非阻塞 tick，
  这样写 100% 忙等自旋占满一个核
- ❌ **禁止** 靠 `RtExecutor::Config` 的 `idle_sleep_us` 限速 —— 它只被
  `run_blocking()` 读取，而 `run_blocking()` 零调用者

### 重构/替代 → 同一 commit 删旧

- 任何「重写 / v2 / 换实现」类改动，新实现与被替代的旧实现必须在**同一个 commit** 内，旧的物理删除
- ❌ 禁止 `_v2` / `_v3` / `_new` / `_old` / `_bak` 后缀文件并存超过 1 个 commit
- ❌ 禁止新增与现有入口同名不同实现的第二份脚本（如已有 `tools/train_e2e/train.py`，禁止再添加第二个 `tools/train/train.py` 做相同事情——必须删旧）
- ✅ 确实要暂留旧路径 → 文件头第一行加注释：
  ```python
  # @deprecated superseded-by=tools/train_e2e/train.py remove-by=2026-08-21
  ```
  ```c
  // @deprecated superseded-by=modules/adas_nodes/inference_node.cpp remove-by=2026-08-21
  ```
  无 `remove-by` 日期视为违规。

可 grep 性：
- `git diff HEAD~1 --name-only | grep -E '_v[0-9]+\.|_new\.|_old\.|_bak\.'` 出现多个同名前缀 → 违规
- `git grep -l "@deprecated" -- ':!*.md' | xargs -I{} grep -L "remove-by" {}` → 缺日期违规
- 任何重构类 commit 的 `git show --stat HEAD` 必须显示 `D <旧文件>` 至少一行

### 错误码 → `error_codes.h`（modules 中待推广）

```c
#include <error_codes.h>
return ERR_INVALID_PARAM;  // 替代 return -1
```

### 违反以上规范的代码不会被合并。

## 3D 渲染门禁（与 C 侧对称 — 2026-07 重建）

> C 链路有 `demo_evaluator.py` 运行时门禁（碰撞/频率/拓扑回归），3D 渲染
> 此前零等价物。以下门禁补上这条缺口，防止"undefined ref / 漏括号 / 漏调用"
> 类回归反复复发。

### 可执行门禁：`npm run vis:check`

```
npm run vis:check
```

等价于顺序执行：

1. **全量模块加载** `node --import ./tests/support/three-preload.mjs tests/vis_module_load.test.mjs`
   — 逐一 import 每个 `js/vis/**` 模块，抓语法错、顶层 ReferenceError、import 路径错误
2. **ESLint no-undef** `npx eslint -c tools/flowboard/eslint.config.mjs tools/flowboard/js/vis/`
   — no-undef 为 error（不通过阻断合并），no-unused-vars 为 warn（标出"定义了但未调用的函数"）
3. **单帧 tick 冒烟** `node --import ./tests/support/three-preload.mjs tests/vis_render_tick.test.mjs`
   — 用 THREE shim 构 SceneDirector，喂 3 帧（平路/高架/多车道），各调一次 tickAnimation()，任何抛错即 FAIL

- ❌ 任何 `tools/flowboard/**` 改动，`npm run vis:check` 红了**不合并**
- ❌ 禁止在无渲染门禁覆盖时对 flowboard 做结构性重构（拆层/换架构）；结构重构与覆盖它的 gate 必须**同 commit**
- ✅ 故障模式表里的 3D 条目，凡能转成 tick 冒烟用例的**必须转成测试**，禁止让文档替代测试

### 门禁覆盖范围

| 门禁 | 覆盖率 | 抓什么 |
|------|--------|--------|
| `vis_module_load.test.mjs` | 26/27 模块 (main.js 除外) | 语法错、顶层 ReferenceError、import 路径 |
| `eslint no-undef` | 全部 27 模块 | 未定义变量引用（如 `VIADUCT_VIS_LENGTH` 未导入） |
| `eslint no-unused-vars` | 全部 27 模块 | 定义了但未调用的函数（如 `followEgo` 漏调） |
| `vis_render_tick.test.mjs` | director + 9 view | tickAnimation 运行时抛错、store 数据完整性 |

### 与 C 侧门禁的对称性

```
C 侧: 改 pipeline 节点 → demo_evaluator.py (45s 真跑) → FAIL 阻断
3D 侧: 改 flowboard/** → npm run vis:check (~3s) → FAIL 阻断
```

### 坐标约定门禁（有牙，可 grep — 2026-07 收敛）

> 所有位置/朝向/高度/尺度转换的**唯一合法入口**是 `js/vis/math/Coord.js` 的纯函数。
> 禁止在任何 view 里手写裸 -y 翻转、裸 atan2 求朝向、裸 position.set 配魔法数。

#### 唯一合法纯函数

| 函数 | 替代 | 用途 |
|------|------|------|
| `worldToThree(x,y,z)` | 手写 `[x, z, -y]` | ENU→THREE 坐标映射 |
| `headingToRotationY(h)` | 裸赋值 `rotation.y = h` | heading→THREE rotation.y |
| `directionToRotationY(dx,dz)` | 裸 `Math.atan2(dx,dz)` | 2D 方向→rotation.y |
| `forwardENU(heading)` | 裸 `Math.cos(h), Math.sin(h)` | heading→ENU 单位前向向量 |
| `offsetAlongNormal(px,pz,nx,nz,d)` | 裸 `px + nx*d` | 中心线法线偏移 |
| `tangentToNormal(tx,tz)` | 裸 `l=sqrt(...); -tz/l, tx/l` | 切线→单位法线 |
| `placeOnRoad(spine,s,lateralOffset)` | 手算插值+法线偏移 | 沿路参数→{pos,rotY,height} |

#### grep 强制违规检测

```
npm run vis:check:grep
```

等价于 `node tests/vis_grep_enforce.mjs`，扫描 `js/vis/view/` 目录：

- 裸 `z: -(n[2])` → 应走 `worldToThree`
- 裸 `Math.atan2(...)` → 应走 `directionToRotationY`
- 裸 `Math.sin/Math.cos`（坐标计算用）→ 应走 `forwardENU` / `offsetAlongNormal`
- 裸 `.position.set(...)` 配魔法数 → 应走 `placeOnRoad`

命中即 FAIL（注释豁免：行内含 `Coord.` 或 `// Coord` 注释）。

#### 纯函数 property-test

```
npm run vis:check:invariant
```

等价于 `node --import ./tests/support/three-preload.mjs tests/vis_coord_property.test.mjs`，验证：

- `worldToThree`: ENU +y(北) → THREE -z 轴映射 golden 表
- `headingToRotationY`: heading=0(东/+x)→车头 forward 指 +x
- `forwardENU`: 单位向量长度=1，方向正确
- `directionToRotationY` 与 `headingToRotationY` 一致性
- `tangentToNormal`: 法线与切线正交，单位长度
- `placeOnRoad`: 任意 s，|height−groundHeightAt(s)|<ε（永不浮空/埋地）；|lateral|≤半路宽⇒点在路面内

#### 全量门禁

```
npm run vis:check:all
```

等价于 `vis:check` + `vis:check:invariant` + `vis:check:grep`，三者全绿才可合并。

- ❌ 坐标/朝向/高度/尺度只准走 `Coord.*` / `placeOnRoad` 纯函数；view 里裸 `-y`/`atan2`/`position.set`+魔法数 = 违规
- ❌ 每个坐标纯函数必须有 property-test
- ❌ flowboard 改动跑 `npm run vis:check:all` 必须绿；几何变更同 commit 更新 golden + PR 附截图
- ✅ 残余诚实说："低/丑/材质光照"这种纯观感 invariant 测不了，只能人 diff（puppeteer 无头跑一帧存图，人肉审 golden 图）。但"位置关系"——功能性几何错——100% 可数值化。收敛约定 + invariant，是把"黑屏后人肉手修"变成"提交前红灯拦下"。

### 违反以上规范的代码不会被合并。

## NPC 智能 — IDM + 边界权限门（2026-07）

> NPC 行为使用「IDM 纵向跟车 + 边界权限门」。

### 架构

```
每帧 NPC tick:
  1. 找同车道前车 (find_lead) → IDM 期望速度
  2. 纵向积分 + 位置更新（Frenet 车道跟随 + road_pos 推进）
  3. 红绿灯/让行期间停车（StopForTL/Yield 状态保持当前车道）
```

### MOBIL 变道

MOBIL 变道已禁用（`#if 0`）。完整代码保留在 `npc_ai.cpp` 中作为参考，可通过将 `#if 0` 改为 `#if 1` 重新启用。当前演示场景中 NPC 各守其道不变道。

| 参数 | 默认值 | 含义 |
|------|--------|------|
| IDM 跟车 | base=5.0, time=1.5 | 安全间距 = base + v × time |
| 加速/减速 | 1.5 / 3.5 m/s² | 平稳加减速 |

### 关键文件

| 文件 | 作用 |
|------|------|
| `modules/adas_nodes/flowsim/npc_ai.h` | NPC AI 配置（NpcAiConfig）+ API |
| `modules/adas_nodes/flowsim/npc_ai.cpp` | IDM + MOBIL + 边界权限门实现 |
| `modules/adas_nodes/flowsim/scene_events.cpp` | 红绿灯/ETC 事件调度 + NPC 响应 |
| `modules/adas_nodes/flowsim/route.cpp` | 中央有序 route（NPC 车道跟随骨架） |

## 仿真基础层 — Digest + Invariant（2026-07）

> 静态/动态 digest 编码空间关系为数值，invariant 断言在提交前红灯拦下功能性几何错。

### 帧契约

```
frame: THREE  | up: +Y | 单位: m | ENU→THREE: [x, z, -y] | ego_centered: true/false
```

### 静态 digest

几何变更时（road network 加载后）建一次，包含：
- 车道（id、中线采样点、宽度、边界类型、行驶方向、限速、s 范围）
- 车道线（虚线段/实线/双黄，位置+段长+间距）
- 可行驶区（路面多边形/路沿 polyline）
- 红绿灯（位置、朝向、管辖车道、相位）

### 动态 digest

每帧 dump ego + 每个 NPC：
- pos [x,y,z]、bbox [L,W,H]、heading、vel [vx,vy]、speed
- yaw_rate、accel、lane_id、lateral_offset、s、rotationY

### Invariant 检查

| 类型 | 检查项 | 抓的 bug |
|------|--------|----------|
| 静态 | 车道宽 ∈ [2.5,4.0]m | 车道宽度配置错 |
| 静态 | 边界类型自洽（同向分隔=虚线、对向=双黄/实线、外沿=实线） | 边界类型配置错 |
| 静态 | 虚线段长 ~3m、间距 ~6–9m | 标线参数错 |
| 静态 | 可行驶区闭合 | 多边形不闭合 |
| 静态 | 路面高程连续 | 高度阶跃跳变 |
| 静态 | 红绿灯朝向 · 车道方向 < 0 | 红绿灯背对来车 |
| 静态 | 没有物体堆在 (0,0,0) | 路网未初始化 |
| 静态 | 每条 lane 中线落在可行驶多边形内 | 可行驶区与车道不一致 |
| 空间 | \|z − roadHeight(x,y)\| < ε | 浮空/埋地 |
| 空间 | \|lateral_offset\| ≤ 半路宽 | 飞出路面 |
| 空间 | rotationY == headingToRotationY(heading) | ENU→THREE 符号翻错 |
| 空间 | 0 ≤ speed ≤ 1.5×限速 | 超速/呆滞 |
| 空间 | bbox ≈ 标准尺寸 | 尺度错 |
| 空间 | 两 actor bbox 不重叠 | 穿模/重叠 |
| 运动方向 | dot(forward, vel) > cos(30°) | 车头≠前进方向（横着/倒着开） |
| 运动方向 | dot(forward, lane_dir) > cos(45°) | 与车道方向不一致 |
| 时序 | Δpos ≈ vel × dt | 瞬移/teleport |
| 时序 | \|Δpos\| ≤ v_max × dt | 超速瞬移 |
| 时序 | \|Δheading\| ≤ yaw_max × dt | 朝向瞬变 |
| 时序 | accel ∈ [−8, +4] m/s² | 运动学不可行 |

### 关键文件

| 文件 | 作用 |
|------|------|
| `modules/adas_nodes/flowsim/sim_digest.h` | Digest 数据结构 + invariant API |
| `modules/adas_nodes/flowsim/sim_digest.cpp` | 实现：digest 生成 + invariant 检查 |
| `modules/adas_nodes/flowsim_node.cpp` | 集成点：静态 digest 建一次，动态 digest + invariant 每 20 帧 |

### 违反以上规范的代码不会被合并。

## 常见故障模式

| 现象 | 根因 | 位置 |
|------|------|------|
| 仪表盘/3D 一直 "Waiting for data"，curl 却有数据 | 仪表盘 JSON 是 cJSON_Print 多行格式，SSE 单 `data:` 帧发送被 EventSource 按行丢弃，浏览器只收到 45 字节前缀。已在发送前压平为单行，详见 [排查文档](docs/TROUBLESHOOTING_3D_DASHBOARD.md) | `monitor_server.c` handle_sse |
| 3D 场景整屏黑（curl 有数据、console 报 `Unexpected token 'export'`） | MVC 重构（c5e4ba9）拆 Controller 层时 `_renderFrame` 相机块漏闭合一个 `}`，scene3d.js 顶层 `export` 被当块内语句、整模块编译失败不执行 → `init3DScene` 未导出。已补回 | `scene3d.js:2159` 附近 |
| 仪表盘所有请求挂死（端口在监听） | 终端对前台 demo.sh 按了 Ctrl+Z，整个进程组 `T (stopped)`。Ctrl+C 结束或后台运行 | `scripts/demo.sh` |
| 车速降到 0 后永久卡死 | ROAD_GUARD 低速恢复条件要求 `|y|>=road_center_limit`，但车可在任意 `2.1<|y|<2.5` 停下。改为只要 `speed<2.5` 就给小油门 | `control_node.cpp:534` |
| 红灯停稳后灯转绿也不走（评估器双峰：要么跑完 x≈306，要么卡死 x≈180） | planning 用 `spd_out[0]`（≈当前车速）覆盖 `command_speed`，停稳后 `v=0 → target=0 → 油门=0 → v=0` 自维持闭锁；TL override 只置 0 从不恢复，解不开。改为取 max | `planning_node.cpp:787` |
| MPC 输出每帧翻符号（bang-bang），调 `r_ddelta` 无效 | 求解器内部 `max_steer=0.35` 而外部限幅 ≈0.027，差 12.9 倍。解从不触及自身约束边界，平滑项全部失效。改为 solve 前 `mpc_set_max_steer()` 注入真实限幅 | `control_node.cpp:1372` |
| 8 个节点线程各占满一个核 | 裸 `while(!stop) ex.run();` 忙等；`idle_sleep_us` 只被零调用者的 `run_blocking()` 读取。改用 `node_pump()` | `coroutine_task.h` node_pump |
| 变道冲出车道 | Stanley heading 阻尼硬编码 0.5，pipeline.json 的 `lat_kd_heading` 未生效 | `control_node.cpp:548` |
| NPC 瞬移 | 障碍物回收逻辑放入 100m 外（设计如此，非 bug） | `flowsim/npc_ai.cpp:204` |
| NPC/车飞出路面、不在车道上、坐标飞到几千米外 | flowsim NPC 用 `step_bicycle(steer=0)` 世界系直线积分、不跟道路几何，路一拐弯就直线冲出路网。已改为中央 `Route`（把各 road 连成有序主路）+ Frenet 沿车道推进 + 到头回收 | `npc_ai.cpp` step_npc_vehicle / `flowsim/route.cpp` |
| 感知降频 | DBSCAN 点云过多时聚类耗时超过 deadline | `perception_node.cpp` |
| 车身左右晃动（1-2Hz 极限环） | 历史根因：`road_pos.world()` 每帧把 ego.heading 重置为道路切线，control 的 `v_lat_damp` 失效（heading_err≈0），退化为纯 P。**96447a9 起已改为两模式都自由积分 heading**：运动学模式由 step_bicycle 积分，靠 `sin(dh)` 负反馈闭环 + cte/heading 项 + 低通 + 死区稳住（曾尝试自由积分导致斜行后回滚，后加 sin(dh) 反馈再启用）；动力学模式由轮胎侧偏力积分。故 `is_dynamic` 分支对两模式一视同仁，只做 heading 归一化 | `flowsim_node.cpp` 主循环 ego 段 |
| steer 打到 0.25 硬限幅导致抖动 | 运动学自行车模型下 heading 漂移可达 0.8 rad，steer 限幅过紧导致控制器累积误差撞 clamp。修复：`lc_lat_accel_max` 从 2.4→4.5，`steer_min_clamp` 从 0.016→0.030 | `control_node.cpp:1245-1253` `pipeline.json:198` |
| 内部巡航 fallback 输出大 steer | `internal_cruise_control` 用 `road_h - heading` 全量前馈，运动学模型下 heading 漂移可达 0.8 rad，公式输出 0.8 被 clamp 到 0.25。修复：改用 `heading_err*0.3 + yaw_damp + lat_err*0.03`，cap 降到 0.15 | `flowsim_node.cpp:1007-1027` |
| 控制遇到慢车不减速、油门全开撞前车 | 积分饱和 anti-windup 逻辑错误：刹车分支 `g.integral < 0` 应为 `g.integral > 0`，`-= error*dt` 应为 `+=`。加速阶段积分累积到 +500，进入减速后 P term 不足以抵消 I term，油门全开撞车。修复：error 翻负时直接清零正积分 | `control_node.cpp:550-554` |
| behavior planner 一直不进 FOLLOW | `worthwhile = blocked && (best_gap < min_gap)` 是反逻辑——等 gap 小于 15.6m 才觉得"值得超车"。高速 8m/s 接近速度下只剩 ~2s，变道来不及。修复：改为 `best_gap > min_gap`，阈值提至 base=25m，mult=2.0 | `behavior_planner_node.cpp:516` |
| 管道检查 topics 列表缺 perception/obstacles | `monitor_node.c` 的 `TopicStats tstats[16]` 只能装 16 个 topic，第 17 个静默丢弃。扩到 64 | `monitor_node.c:647` |
| HTTP 返回 JSON 在 64KB 被截断 | `monitor_server.c` 的 `MONITOR_HTTP_BUF_SIZE 65536` 不够装含 samples 的完整拓扑 JSON。扩到 131072 (128KB) | `monitor_server.c:46` |
| 转向灯左右颠倒（变道打右灯亮左边/打左灯亮右边） | 车辆模型所有 `*_L` 件放在 z=+0.82（THREE 右手系车头朝 +X 时 +z=几何右），`*_R` 在 z=-0.82；前端 `_setVehicleLights` 按名字点 FL/RL → 左灯请求点亮几何右灯。修复：gen_models.py 全部 L/R 件 z 互换 + 重生成 gltf + `--validate` 对称性门禁（生成物中心 z 符号必须与名字一致，防复发） | `tools/flowboard/gen_models.py`（L/R 部件 z 号） |
| 变道减速甚至全刹、超车没意义 | behavior 变道中 P5 分支 `else if (blocked) target_speed = lead_speed`：变道转移首帧 blocked=1 且前车停着（等红灯）→ target=0，后续 blocked=0 帧无分支重置 → 锁死 → planning command_speed=0 → 全刹（实测 spd=10.7→0.0 brk=1.00）。修复：删除该分支，防追尾改由 planning TTC + safety_control 近场 TTC 兜底 | `behavior_planner_node.cpp` 变道分支 |
| 超车/归位后立刻在红灯前刹停（无效变道） | 归位/超车决策不看目标车道前方红绿灯，切回内侧道（灯只管辖 y_lane=-1.75）即刹停。修复：behavior 订阅 road/traffic_lights，`lane_ahead_stop_light()` 检查目标车道前方 60m 内非绿灯则不归位/不变入 | `behavior_planner_node.cpp` `lane_ahead_stop_light` |

## 最新 tag

`v0.1.0` — 创始版本，8 节点全链路稳定运行

---

# 可视化架构

详见 [可视化架构](docs/VISUALIZATION_ARCHITECTURE.md)。

前端 3D 渲染使用 `vis/` 架构：

```
tools/flowboard/js/vis/
├── main.js           — 入口
├── core/             — 核心渲染框架（SceneDirector, CameraRig, Lighting, Constants, Layer, Renderer, SkyEnv, ViewRegistry, AssetFactory, DeadReckon）
├── director/         — 场景导演（SceneDirector, FrameValidator）
├── view/             — 3D 视图（VehicleView, RoadView, GroundView, ViaductView, BarrierView, TreeView, TrafficLightView, ETCGateView, StreetlightView, ConnectorView, VehicleLights）
├── math/             — 坐标/几何工具（Coord.js — 唯一事实源, Curve, GeometryMerge, RoadHeight）
└── store/            — 场景状态（SceneStore）
```

数据链路由 C 监控守护进程 **flowmond**（`src/flowmond.c` → `build/bin/flowmond`）提供：

```
monitor_node → 10Hz 写 /tmp/flow_topology.json → flowmond :8800 → 浏览器 (文件桥接回退)
monitor_node → stats_bridge / dashboard_bridge → IPC SHM → flowmond :8800 → 浏览器 (IPC 桥接)
```

- 前端 `tools/flowboard/index.html` 由 flowmond 通过 `--html-path` 加载并托管。
- `modules/adas_nodes/flowmond_node.cpp` 是 flowmond 的 `NodePlugin` 包装版，可作节点插件在 pipeline 内运行。
- 启动：`./build/bin/flowmond --html-path tools/flowboard/index.html`（或通过 `scripts/demo.sh`，已改为调用 flowmond）。

| 组件 | 端口 | 说明 |
|------|------|------|
| `flowmond` (C) | 8800 | HTTP 仪表盘：IPC 桥接（首选）+ 文件桥接（`/tmp/flow_topology.json` 回退）+ 自动重连 |
| `foxglove_bridge.py` | 8765 | Foxglove Studio 3D 桥接 |
