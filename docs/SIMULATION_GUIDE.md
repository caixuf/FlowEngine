# 仿真测试指南

## 三层仿真体系

```
Layer 1: Bag 回放 (零依赖, 现在就能跑)
  → 录制数据 → 回放给算法 → 校验输出

Layer 2: 2D 运动学模拟 (轻量, 验证控制/规划)
  → 单车模型 + 简单传感器 + 场景定义

Layer 3: Carla/Gazebo (真实传感器, 完整闭环)
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
./build/bin/scenario_runner scenario.bag \
    --plugin lib/libmy_controller.so \
    --check "control/cmd" "throttle<0.5"
```

## Layer 2: 2D 模拟器

```bash
# 内置场景: 直道 + 前车
./build/bin/scenario_runner --sim --duration 30

# 输出:
#   t=0.0s | ego: (0,0) 0.0 m/s | obstacle: 50m ahead
#   t=2.0s | ego: (60,0) 30.0 m/s | obstacle: 40m ahead
#   t=4.0s | ego: (120,0) 30.0 m/s | obstacle: 30m ahead | braking...
```

## Layer 3: Carla 集成（需要安装 Carla）

```
┌──────────────┐         ┌──────────────┐
│    Carla     │  TCP    │  FlowEngine   │
│  (simulator) │◄───────►│  (middleware) │
│              │         │               │
│  camera ─────┼──data──►│ perception ───┼──► planning
│  lidar  ─────┤         │ fusion     ───┤    control
│  radar  ─────┤         │ control ──────┼──► Carla
└──────────────┘         └──────────────┘
```

### Carla 桥接步骤

1. 安装 Carla 0.9.15+
2. 使用 `carla_bridge.py` 将 Carla 传感器数据转发到 FlowEngine topic
3. FlowEngine 算法处理 → control cmd 发回 Carla
4. Carla 执行控制 → 下一帧传感器数据

### 最小 Carla 集成示例

```python
# carla_bridge.py
import carla

client = carla.Client('localhost', 2000)
world = client.get_world()

# 订阅 Carla 传感器 → 发布到 FlowEngine UDP discovery
def on_lidar(data):
    # 转换 Carla LidarMeasurement → FlowEngine Message
    msg = serialize_lidar(data)
    send_to_flowengine(msg)

sensor.listen(on_lidar)
```

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
# 1. 最简单: 2D 模拟器 (零依赖)
./build/bin/scenario_runner --sim --duration 30

# 2. Bag 回放 (需要预先录制的 bag)
./build/bin/flow_e2e 10          # 先录制
./build/bin/scenario_runner /tmp/test.bag  # 再回放

# 3. 接入算法测试
./build/bin/scenario_runner scenario.bag --plugin lib/libmy_algo.so
```
