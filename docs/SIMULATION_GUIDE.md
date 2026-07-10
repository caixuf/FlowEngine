# 仿真测试指南

> **注意：** 本文档中 `scenario_runner` 二进制尚未加入构建（源码在 `tools/scenario_runner.c`）。
> 当前可用的仿真入口是 `flow_e2e`（单进程全链路）和 `flow_launcher config/pipeline.json`（配置驱动）。
> Carla 集成桥接 `carla_bridge.py` 尚未实现，此处保留为设计参考。

## 三层仿真体系

```
Layer 1: Bag 回放 (零依赖, 现在就能跑)
  → 录制数据 → 回放给算法 → 校验输出

Layer 2: 2D 运动学模拟 (轻量, 验证控制/规划)
  → 单车模型 + 简单传感器 + 场景定义

Layer 3: Carla/Gazebo (真实传感器, 完整闭环) — 待实现
  → 物理引擎 + 相机/LiDAR/雷达模型 + 3D 场景
```

## Layer 1: Bag 回放测试

```bash
# 1. 录制场景数据
./build/bin/flow_e2e 30 &
# 数据自动写入 /tmp/flow_topology.json + bus stats

# 2. 录制到 bag
# (FlowEngine 自动通过 bag_writer_attach 录制)

# 3. 回放 + 校验
./build/bin/flow_e2e --replay scenario.bag
```

## Layer 2: 2D 模拟器

```bash
# 内置场景: 直道 + 前车 + 行人横穿
bash scripts/demo.sh

# 或使用配置驱动启动器指定场景
./build/bin/flow_launcher config/pipeline.json
```

内置场景定义见 `scenarios/pedestrian_crossing.json` 和 `scenarios/highway_overtake.json`。

## 场景库

| 场景 | 难度 | 测试目标 |
|------|------|---------|
| 直道巡航 | ⭐ | ACC 纵向控制 |
| 弯道保持 | ⭐ | 横向控制 + 车道线检测 |
| 前车切入 | ⭐⭐ | 感知跟踪 + AEB |
| 行人横穿 | ⭐⭐ | 检测 + 紧急制动 |
| 高速变道 | ⭐⭐⭐ | 规划 + 预测 + 控制 |
| 无保护左转 | ⭐⭐⭐⭐ | 完整决策链路 |
| 鬼探头 | ⭐⭐⭐⭐⭐ | 感知极限测试 |

## 测试指标

```bash
# 每个场景跑完，自动收集:
flowctl topic stats control/cmd     # 控制指令统计
flowctl param get control.max_speed # 参数状态
grep "collision" scenario.log       # 碰撞检测
grep "timeout\|miss" scenario.log   # 超时/丢帧
```

## 快速开始

```bash
# 1. 最简单: 一键 demo (含全链路节点)
bash scripts/demo.sh

# 2. Bag 回放 (需要预先录制的 bag)
./build/bin/flow_e2e 10          # 先录制
./build/bin/flow_e2e --replay /tmp/test.bag  # 再回放

# 3. 接入算法测试
./build/bin/flow_launcher config/pipeline.json
```

## e2e 内置 3D 场景仿真

`flow_e2e` 的 monitor 任务会导出真实 3D 场景到 `/tmp/flow_topology.json`
的 `metrics.scene` 字段，FlowBoard 仪表盘据此渲染真实三维场景（自车、
障碍物包围盒、LiDAR 点云），而非随机占位点。

包含的真实仿真要素：

- **横向自行车模型**：自车具备 `heading` / `steer`，转向为向目标车道的
  P 控制，可演示**变道**（行驶到一定里程自动切到左车道再回正）。
- **ACC 纵向控制**：依据与同车道前车的间距动态限制目标速度，间距越近
  目标速度越低，触发真实的减速/刹车行为。
- **动态障碍物**：前车、对向来车、过街行人，具备运动学与循环边界，
  使场景持续有内容。
- **LiDAR 点云**：对障碍物表面 + 地面环带做光线投射后下采样，坐标位于
  自车系。

一键运行（业务节点 + HTTP 桥接 + 浏览器）：

```bash
./scripts/demo_sim.sh
# 或手动:
./build/bin/flow_e2e 3600 &
python3 tools/flowboard_server.py &
open http://localhost:8800
```

`scene` 数据结构与坐标系约定详见
[VISUALIZATION_ARCHITECTURE.md](VISUALIZATION_ARCHITECTURE.md#scene-数据结构真实-3d-仿真)。
完整的离线问题根因分析与鲁棒性设计详见
[E2E_SIMULATION_DESIGN.md](E2E_SIMULATION_DESIGN.md)。
