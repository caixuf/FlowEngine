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

## 常见故障快速定位

```bash
# 查看最后一次运行的完整日志
grep -E "COLLISION|ROAD_GUARD|STUCK|INTERVENED" /tmp/flow_launcher_stderr.txt

# 查看速度掉到 0 的时间点
grep "spd=0.0\|v=0.0" /tmp/flow_launcher_stderr.txt | head -5

# 查看 ego 横向位置变化（判断是否漂移到路边缘）
grep "fusion.*EKF" /tmp/flow_launcher_stderr.txt | awk -F'[()]' '{print $2}'
```

## CI 集成

```bash
# 在 GitHub Actions 中运行
python3 tools/demo_evaluator.py --duration 30
# 退出码: 0=PASS, 2=FAIL
```
