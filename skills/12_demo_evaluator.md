# Demo Evaluator — 回归评估器

`tools/demo_evaluator.py` 是 FlowEngine 的自动化回归测试工具。它启动 demo pipeline，
采样运行时数据，按预定义标准评分，输出 PASS/FAIL 判定。

## 为什么需要它

手动观察日志判断 demo 是否正常容易遗漏问题（2 分钟后才出现的停滞、偶发碰撞等）。
评估器自动检查 16 个维度，每次改动 pipeline 链路节点后都应运行。

## 基本用法

```bash
# 默认 15 秒运行
python3 tools/demo_evaluator.py

# 长运行（捕获慢漂移 — 路边缘 creep 需要 >20s 才出现）
python3 tools/demo_evaluator.py --duration 45 --interval 0.5

# 只分析已存在的数据，不重新启动 demo
python3 tools/demo_evaluator.py --no-run
```

## 工作原理

1. 启动 `scripts/demo.sh --no-browser <duration>`
2. 每隔 `--interval` 秒采样 `/tmp/flow_topology.json`
3. 从 `config/pipeline.json` 读取场景的 `pass_criteria`
4. 在所有采样上运行 16 个检查项
5. 打印摘要表 + WARN/FAIL 列表

## 检查项

| 类别 | 检查内容 | 阈值 |
|------|---------|------|
| 拓扑 | 5 条必须链路存在 | — |
| 拓扑 | 传感器链路可选对存在 | — |
| 频率 | vehicle/state, sensor/lidar 等 | ≥ 最小频率 |
| 碰撞 | sim/collision topic 和日志 | 0 |
| 路沿偏离 | 车体超出路边界 | margin ≥ 0 |
| 前进距离 | x 方向累计位移 | ≥ min_distance_m |
| 低速停滞 | 低速采样占比 + 最长连续区间 | <50%, <5s |
| 变道 | 至少 1 次变道 | ≥1 |
| 偏航抖动 | 偏航角速度 RMS + 最大值 | <0.35, <1.2 rad/s |
| 转向振荡 | 转向角速度 + 翻转频率 | <0.9/s, <1.0 Hz |
| NPC 运动 | NPC 最大速度（捕获瞬移 bug） | <45 m/s |
| 丢帧 | topic 丢弃计数 | 0 |

## 结果解读

### PASS
所有强制检查通过。Pipeline 在当前回归包络内正常。

### WARN（非致命）
- `large lane-center deviation` — 变道期间正常（可达 ±1.75m）
- `npc respawn jump` — 已知障碍物回收瞬移（`sim_world_node.c`）
- `steer saturated` — 可能表示横向控制器欠阻尼，检查 `lat_kp` / `lat_kd_heading`

### FAIL（需修复）
- `vehicle stuck or no progress` → 车永久停止。常见于 ROAD_GUARD 死锁：
  车漂移到路边缘 → 刹车至 0 → 自行车模型无法横向移动。
  修复：`control_node.c` ROAD_GUARD 低俗恢复条件改为 `>=`
- `road departure` → 变道冲出车道。检查 Stanley 控制器 heading 阻尼
- `collision detected` → AEB/ACC 间距太激进
- `low-speed stagnation` → 被慢车阻塞且无法变道

## 深层故障模式（2026-07 新增）

### Pattern A: EKF 收敛期 committed_lane_side 误初始化
**症状：** 仿真前 0.5s ego 从 y=-1.75（左车道）被拉到右车道（y>0），随后 ROAD_GUARD 触发。
**根因：** `fusion_node.cpp` EKF 初值 `x0[5] = {0,0,5,0,0}` — y=0，但 ego 实际在 y=-1.75。
EKF 收敛前（~10 cycle = 0.5s）fusion 报告的 ego_y 接近 0，control_node 的
`committed_lane_side = (ego_y < road_c) ? -1 : 1` 在 ego_y=0 时判定为 +1（右车道），
于是 cruise_lane_y = road_c + half_lane = +1.75，把 ego 拉向右车道。
**修复：** `control_node.cpp` line ~706 — 仅当 `|ego_y - road_c| > 1.0`（EKF 已收敛）
才初始化 committed_lane_side。未收敛时 committed_lane_side=0，cruise_lane_y=ego_y（横向不动）。

### Pattern B: ref_path 在 junction/fork 处的航向腐败
**症状：** ego 接近 fork junction（如 x=500）时突然被反向打满 steer 冲向岔路。
**根因：** esmini 的 `Route::sample_ahead()` 在 junction 附近会采样到 connecting road
（如匝道）上的点，其切线航向可能与 ego 当前航向偏差极大（如 ref_h≈5 rad 而 ego_h≈0）。
Stanley heading_term = `lat_kd_heading * (ego_h - ref_h)` 爆炸，反向打满 steer。
**修复：** `control_node.cpp` line ~1048 — 将 `(ref_h - ego_heading)` 归一化到 [-π,π]，
若 |dh| > 0.5 rad（≈29°）则视为无效参考，用 ego_heading 替代（heading_term=0）。
真正的急弯跟随由 ff_term（曲率前馈）处理，不依赖 heading_term。

### Pattern C: 非 route 路段 NPC 的世界系投影陷阱
**症状：** NPC 放在 connecting road（如 left_ramp segment 10）上，仿真中却出现在
主路上 ego 车道前方，触发幽灵变道或碰撞。
**根因：** NPC 若所在 road 不在 `Route::build()` 链出的主 route 上（如匝道、对向路），
`npc_init_route()` 会置 `route_dir=0`，走 `npc_ai.cpp` line 314 的世界系兜底分支：
`world_to_frenet → frenet_to_world`。esmini 在 junction 附近的最近 road 查询可能
把 NPC 投影到主路上，造成位置漂移。
**修复：** 场景设计时，避免在 connecting road 上放置需要长距离行驶的 NPC。
若需占位，用 `vx=0` 的静止车辆，或移到主 route 上的 segment。

### Pattern D: 场景 JSON 负 s 值导致 NPC 叠在 route 起点
**症状：** 多个 NPC 同时出现在 route 起点（x≈0），互相碰撞或阻塞 ego。
**根因：** `npc_init_route()` line 346 — `if (npc.s < 0.0) { npc.route_dir = 0; return; }`
负 s 的 NPC 走世界系兜底，但若同时有多个，esmini 可能将它们都 locate 到最近的
road 起点。
**修复：** 场景 JSON 中所有 actor 的 `s` 值必须 ≥ 0。负 s 在 OpenDRIVE 语义上
无效（road 起点之前不存在路面）。

## 常见故障快速定位

```bash
# 查看最后一次运行的完整日志
grep -E "COLLISION|ROAD_GUARD|STUCK|INTERVENED" /tmp/flow_launcher_stderr.txt

# 查看速度掉到 0 的时间点
grep "spd=0.0\|v=0.0" /tmp/flow_launcher_stderr.txt | head -5

# 查看 ego 横向位置变化（判断是否漂移到路边缘）
grep "fusion.*EKF" /tmp/flow_launcher_stderr.txt | awk -F'[()]' '{print $2}'

# 查看变道触发原因（cur_gap/adj_gap/lead_v）
grep "LANE CHANGE" /tmp/flow_launcher_stderr.txt

# 查看 ego 位置周期快照（flowsim 每 100 cycle 打印一次）
grep "flowsim.*ego(" /tmp/flow_launcher_stderr.txt

# 查看控制循环详细日志（spd/err/thr/brk/st/target_y/lc）
grep "control       \] #" /tmp/flow_launcher_stderr.txt | tail -50

# 查看当前 topology 中所有 NPC 位置（定位幽灵车辆）
python3 -c "
import json
d = json.load(open('/tmp/flow_topology.json'))
ego = [e for e in d['metrics']['scene']['entities'] if e['type']=='ego'][0]
for e in d['metrics']['scene']['entities']:
    if e.get('type') in ('car','truck'):
        dx = e['x'] - ego['x']
        if -10 < dx < 130 and abs(e['y']-ego['y']) < 3:
            print(f\"id={e['id']:2d} x={e['x']:7.1f} y={e['y']:6.2f} dx={dx:7.1f} spd={e.get('spd',0):.1f}\")
"
```

## CI 集成

```bash
# 在 GitHub Actions 中运行
python3 tools/demo_evaluator.py --duration 30
# 退出码: 0=PASS, 2=FAIL
```
