# 算法快速验证工作流

## 动机

"调不好"的根因往往不在算法本身，而在**数据反馈回路断裂**：
- 改参数后要等 45s demo 跑完才看到效果
- 评估器 `--no-run` 只读 1 帧快照，所有时序指标恒为 DEAD SIGNAL
- 没有离线验证手段，不知道改动是否有效

## 工作流

```
写 C/C++ 算法
  → python3 tools/pipeline_check.py           (3s, 不启动 demo)
  → python3 tools/quick_verify.py             (实时交互式调参)
  → python3 tests/test_param_regression.py    (改参前后对比)
  → python3 ci/evaluators/demo_evaluator.py           (45s 全量回归)
  → commit
```

## 单一数据源

所有验证工具都只读 `/tmp/flow_topology.json`，由 `monitor_node.c` 的
`export_dashboard_json()` 以 20Hz 写入。JSON 结构：

```
{
  "self": "flow_launcher",
  "timestamp": <double>,
  "samples": [{t, x, y, heading, speed, steer}, ...],   ← 最近 ~10s 的时序
  "metrics": {
    "behavior": {state, best_gap, lead_speed, blocked, ...},
    "scene": {
      "ego": {x, y, heading, speed, steer, lights, ai_state, ...},
      "lane": {width, count},
      "entities": [{type, id, x, y, heading, speed, ai_state}, ...],
      "obstacles": [{id, type, x, y, vx, vy, len, wid}, ...],
      "trajectory_path": [[s, d, spd], ...]
    },
    "vehicle": {speed, target_speed, throttle, brake},
    "topics": [{topic, pub, del, freq}, ...],
    "registry": {tasks, plugins, types, params}
  }
}
```

## 新增算法的步骤

### Step 1: 写好 C/C++ 算法
确保数据最终能写入拓扑 JSON 的某个字段。

### Step 2: 用 pipeline_check.py 验证数据可见
```bash
# 不启动 demo，直接检查拓扑 JSON
python3 tools/pipeline_check.py --focus perception

# 检查特定字段
python3 tools/pipeline_check.py --verbose
```

如果 pipeline_check 说数据缺失 → 回到 Step 1 修链路。

### Step 3: 在 pipeline_check.py 加检查项
在 `tools/pipeline_check.py` 中找到对应的 `check_*()` 函数，
加一个 `section.ok/fail/warn` 断言。

### Step 4: 用 quick_verify.py 交互式验证
```bash
# 启动 demo + 仪表盘
python3 tools/quick_verify.py --duration 120

# 改参数看效果
> set behavior.acc_time_headway 2.5
> eval 15
```

### Step 5: 用 test_param_regression.py 做回归对比
```bash
# 先保存 baseline
python3 tests/test_param_regression.py --save-baseline

# 改参数后对比
python3 tests/test_param_regression.py
```

### Step 6: 全量回归
```bash
python3 ci/evaluators/demo_evaluator.py --duration 45
python3 tools/pipeline_check.py
```

### Step 7: commit

## 以"行人识别"为例

假设你要新写行人识别 + 让车让行：

1. **写 perception_node.cpp** — 在 `ObstacleList` 中填充 `type=pedestrian`
2. **确认数据链路** — 验证 `scene.obstacles[i].type == "pedestrian"` 出现在拓扑 JSON 中：
   ```bash
   python3 -c "
   import json
   d = json.load(open('/tmp/flow_topology.json'))
   obs = d['metrics']['scene']['obstacles']
   for o in obs:
       if o.get('type') == 'pedestrian':
           print(f'  行人 id={o[\"id\"]} x={o[\"x\"]:.1f} y={o[\"y\"]:.1f}')
   "
   ```
3. **在 pipeline_check.py 加行人断言** — `check_perception()` 里已有
   `pedestrians_truth > 0 and pedestrians_perceived == 0 → FAIL`
4. **写 behavior 让行逻辑** — 在 `behavior_planner_node.cpp` 中识别行人并发出 YIELD 状态
5. **确认 behavior/state 显示 YIELD**：
   ```bash
   python3 tools/pipeline_check.py --focus behavior
   ```
6. **回归检查** — 跑 `test_param_regression.py` 确认让行后 CTE 和车速没有异常
7. **全量回归** — `demo_evaluator.py --duration 60`

## 工具速查

| 工具 | 启动 demo? | 耗时 | 用途 |
|------|-----------|------|------|
| `pipeline_check.py` | 不启动 | ~1s | 静态检查数据完整性 |
| `quick_verify.py` | 启动 | 交互式 | 实时调参 + 即时评估 |
| `test_param_regression.py` | 启动 | ~30s | 参数前后对比 |
| `demo_evaluator.py` | 启动 | ~60s | 全量回归门禁 |
