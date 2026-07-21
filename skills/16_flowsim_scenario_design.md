# FlowSim 场景设计 — 多 edge 路网 + NPC 放置规范

> 编写或修改 `scenarios/*.json` 时必读。基于 `city_to_highway_full.json` 实战调试经验。

## 何时使用

- 新建多 edge + junction 的复杂场景（城区→路口→收费站→匝道→高速）
- 在现有场景中增减 NPC actor
- 修改 NPC 的 `segment_id` / `s` / `l` / `vx` 参数
- 调试 `demo_evaluator.py` 报告的 collision / road departure / 幽灵变道

## 场景 JSON 结构关键点

```json
{
  "ego": { "x": 10.0, "y": -1.75, "heading": 0.0, "init_speed": 8.0, "target_speed": 13.89 },
  "road_network": {
    "edges": [...],          // 主路段序列
    "junctions": [...],      // fork / merge
    "cross_roads": [...]     // 交叉口
  },
  "actors": [
    { "id": 1, "segment_id": 0, "type": "car", "s": 80, "l": -5.25, "vx": 3.0, "len": 4.6, "wid": 2.0 }
  ]
}
```

## 路网设计规则

### 主 route 链
`Route::build()` 会按端点连续性 + 航向连续性把 edges 链成**一条主 route**。
ego 沿这条 route 行驶；control_node 的 Stanley 控制器从 `sample_ahead()` 消费 ref_path。

- **主 route 上的 edges**：urban(0) → intersection(1) → toll(2) → ramp_curve(3) → merge(4) → highway(5)
- **不在主 route 上的 edges**：junctions 中的 connecting_roads（如 left_ramp segment 10）

### Junction connecting road 的陷阱

放在 connecting road（如 `left_ramp` segment 10）上的 NPC：
1. `npc_init_route()` 发现该 road 不在主 route 上 → 置 `route_dir=0`
2. 走 `npc_ai.cpp` 世界系兜底分支：`world_to_frenet → frenet_to_world`
3. esmini 在 junction 附近的最近 road 查询可能把 NPC 投影到主路上
4. NPC 出现在 ego 车道前方，触发幽灵变道或碰撞

**规则：connecting road 上不要放行驶中的 NPC。** 若需占位，用 `vx=0` 静止车辆，
或移到主 route 上的 segment。

## Actor 放置规则

### s 值约束
- 所有 actor 的 `s` 必须 ≥ 0
- `npc_init_route()` line 346 显式拒绝负 s：`if (npc.s < 0.0) { npc.route_dir = 0; return; }`
- 负 s 的 NPC 走世界系兜底，可能被 esmini locate 到 road 起点，互相叠车

### l 值（横向偏移）约定
- `l=0`：道路中心线
- `l=-1.75`：左车道中心（lane_width=3.5 时）
- `l=1.75`：右车道中心
- `l=-5.25`： ego 车道左侧相邻车道（避开 ego）
- `l=-8.75`：再左侧车道或路缘

### ego 车道避让
若 ego 起步在 y=-1.75（左车道），前方 NPC 应放在：
- 同段 `l=-5.25` 或更左（避开 ego 车道）
- 或不同 segment 上（前方足够远）

避免把慢车放在 ego 同车道近距离（如 s 相差 < 30m），否则会触发被动变道。

## vx 速度约定

- `vx > 0`：同向行驶车辆，沿 route_s 增加方向
- `vx = 0`：静止车辆（路障、停车）
- `vx < 0`：对向来车（需放在对向路段，且 `npc_init_route` 加 π 航向）

**注意：** `on_scene_frame()` line 388 过滤条件是 `if (evx < 0.0) continue;`，
即 vx=0 的静止车辆**会**被 control_node 当作障碍物。若静止车辆在 ego 车道前方
近距离（< 90m），会触发被动变道。

## evaluator 路沿判定

`tools/demo_evaluator.py` 的路沿判定公式（line 382）：
```python
road_edge_margin = lane_width * lane_count * 0.5 - abs(y_rel) - 1.0
```

- `lane_width` 和 `lane_count` 取场景顶层 `lane` 对象
- 场景无顶层 `lane` 对象时默认 `lane_width=3.5, lane_count=2`
- 默认下：`margin = 2.5 - |y|`，|y| > 2.5 判定路沿偏离

**注意：** `road_network.edges[].lanes` 和 `lane_width` 是 per-edge 几何参数，
**不影响 evaluator 判定**。若 4 车道 toll 段 ego 偏到 y=3.0，evaluator 仍按
默认 2 车道判定路沿偏离。

## 排查清单（场景导致 evaluator FAIL 时）

### collision detected
1. 用 diagnostic 命令列出 ego 车道内所有 NPC：
   ```bash
   python3 -c "
   import json
   d = json.load(open('/tmp/flow_topology.json'))
   ego = [e for e in d['metrics']['scene']['entities'] if e['type']=='ego'][0]
   for e in d['metrics']['scene']['entities']:
       if e.get('type') in ('car','truck'):
           dx = e['x'] - ego['x']
           if -10 < dx < 130 and abs(e['y']-ego['y']) < 3:
               print(f\"id={e['id']} x={e['x']:.1f} y={e['y']:.2f} dx={dx:.1f} spd={e.get('spd',0):.1f}\")
   "
   ```
2. 检查是否有 NPC 出现在非预期位置（被 esmini 投影到主路）
3. 检查 connecting road 上的 NPC — 移到主 route segment 或改 vx=0

### road departure
1. `grep "LANE CHANGE" /tmp/flow_launcher_stderr.txt` — 看是否有幽灵变道
2. 检查 ego_y 是否在仿真前 0.5s 漂移到右车道（EKF 收敛期问题，已在 control_node 修复）
3. 检查是否在 junction 附近被 ref_path 航向腐败拉向岔路（已在 control_node 修复）

### low-speed stagnation
1. 检查 ego 车道前方是否有慢车（vx < ego target_speed）在近距离（< 50m）
2. 若被动变道未触发，检查 `min_overtake_gap_cap`（默认 90.0）是否被场景配置覆盖

## 已知良好场景参考

- `scenarios/zhongkai_road_full.json` — 多 edge + junction + NPC，已通过 evaluator
  - 关键修复：NPC 17 从 segment 10 移到 segment 5；NPC 19 的 s 从 -30 改为 150
  - NPC 1-6 在 segment 0 的 l=-5.25 / -8.75（避开 ego 车道）
  - NPC 14 在 segment 2 的 l=1.75（右车道，不挡 ego）
