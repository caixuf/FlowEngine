# 真车硬件部署指南

> 把 FlowEngine 从仿真部署到真实智能小车。配套配置模板：[config/pipeline_car.json](../config/pipeline_car.json)

## 架构对比

### 仿真模式（pipeline.json）

```
sim_world ──▶ vehicle/state ──▶ sensor_model ──▶ sensor/lidar, sensor/gps
    │                                   │
    ├──▶ road/geometry                  └──▶ perception ──▶ perception/obstacles
    │                                                        │
    └──▶ vehicle/state ──▶ control/safety/actuator(无)         ▼
                                                          fusion, planning
```

`sim_world` 同时扮演"物理世界"+"车辆动力学"+"真值源"三个角色，`sensor_model` 从真值加噪声生成传感器数据。

### 真车模式（pipeline_car.json）

```
[真实GPS] ──▶ gps_driver ──▶ sensor/gps ──┐
                                          ├──▶ fusion ──▶ fusion/localization ──┐
[真实LiDAR] ──▶ lidar_driver ──▶ perception/obstacles ─────────────────────────┤
                                                                                ▼
                                                                    planning ──▶ control
                                                                      │            │
                                                                      ▼            ▼
                                                              perception/obstacles  control/raw_cmd
                                                                      │            │
                                                                      ▼            ▼
                                                            safety_control ──▶ control/cmd
                                                                      │
                                                                      ▼
                                                                  actuator ──▶ [CAN总线] ──▶ ESC/舵机
```

**关键变化**：
- `sim_world` / `sensor_model` / `perception` 三个仿真节点**移除**
- 新增 `gps_driver`（真实 GPS 串口 → sensor/gps）
- 新增 `lidar_driver`（真实 LiDAR → DBSCAN → perception/obstacles，替代 perception_node）
- 新增 `actuator`（control/cmd → SocketCAN → ESC/舵机）

## 节点清单

| 节点 | 库 | 输入 | 输出 | 硬件依赖 |
|------|----|------|------|---------|
| gps_driver | libgps_driver_node.so | 串口 NMEA | sensor/gps | GPS 模块（/dev/ttyUSB0） |
| lidar_driver | liblidar_driver_node.so | 串口点云 | perception/obstacles | LiDAR（/dev/ttyUSB1） |
| fusion | libfusion_node.so | sensor/gps | fusion/localization | — |
| planning | libplanning_node.so | fusion/localization, perception/obstacles | planning/trajectory | — |
| control | libcontrol_node.so | fusion/localization, planning/trajectory | control/raw_cmd | — |
| safety_control | libsafety_control_node.so | control/raw_cmd, perception/obstacles | control/cmd | — |
| actuator | libactuator_node.so | control/cmd | CAN 帧 | CAN 接口（can0） |
| inference | libinference_node.so | control/cmd, fusion/localization, ... | inference/trajectory | — |
| data_recorder | libdata_recorder_node.so | fusion, planning, obstacles, cmd | JSONL 文件 | — |
| learner | liblearner_node.so | 同上 | learner/status | — |
| model_ota | libmodel_ota_node.so | model_ota/cmd, fusion | model_ota/* | — |
| monitor | libmonitor_node.so | obstacles, latency, ... | 仪表盘 JSON | — |

## 硬件准备

### 计算平台

| 平台 | CPU | 内存 | 能跑吗 |
|------|-----|------|--------|
| 树莓派 4B | 4×A72 @1.5GHz | 2-8GB | 轻松（算力需求 <10% 单核） |
| 树莓派 5 | 4×A76 @2.4GHz | 4-8GB | 富余 |
| Jetson Nano | 4×A57 @1.4GHz | 4GB | 轻松（GPU 用不上） |
| 香橙派 Zero2 | 4×A55 @1.5GHz | 1GB | 能跑 |

### GPS 模块

| 模块 | 价格 | 接口 | 输出格式 | 配置 |
|------|------|------|---------|------|
| u-blox NEO-M8N | ¥40-80 | UART | NMEA 0183 | serial_port=/dev/ttyUSB0, baud=9600 |
| ATGM336H | ¥15-30 | UART | NMEA 0183 | 同上 |
| VK2828U7G5LF | ¥30 | USB | NMEA 0183 | serial_port=/dev/ttyACM0, baud=9600 |

接 USB-TTL 转换器后插入树莓派 USB 口，识别为 `/dev/ttyUSB0`。nmea_parser 支持 GGA + RMC 语句（经纬度、速度、航向），覆盖定位基本需求。

### LiDAR

| 型号 | 价格 | 接口 | 配置 |
|------|------|------|------|
| RPLIDAR A1/A2 | ¥100-400 | UART/USB | 需改 read_lidar_frame 接 RPLIDAR SDK |
| 速腾浦 M1 | ¥500+ | UDP/串口 | 需改 read_lidar_frame 接 SDK |
| Velodyne VLP-16 | ¥3000+ | UDP | 需改 read_lidar_frame 接 UDP 协议 |

> **适配点**：[lidar_driver_node.c](../modules/adas_nodes/lidar_driver_node.c) 的 `read_lidar_frame()` 函数是硬件适配钩子。默认实现 ASCII 行协议（`x,y,z,intensity\n`）。按你的 LiDAR 协议改这一个函数即可，其余 DBSCAN + publish 逻辑不变。

### CAN 接口

| 方案 | 价格 | 说明 |
|------|------|------|
| MCP2515 SPI-CAN 扩展板 | ¥30 | 树莓派 GPIO SPI 接线，需 dtoverlay 配置 |
| USB-CAN 适配器 | ¥100-300 | 即插即用，无需接线 |

CAN 总线两端必须各接 **120Ω 终端电阻**，否则 BUS-OFF。详见 [skills/15_socketcan_actuator.md](../skills/15_socketcan_actuator.md)。

## 部署步骤

### 1. 编译

```bash
# 主框架（含 serial_port 串口库）
cmake -B build -S . && cmake --build build --target flowengine_core

# 节点插件
cmake -B build/modules/adas_nodes -S modules/adas_nodes
cmake --build build/modules/adas_nodes
```

确认产物：
```bash
ls build/lib/lib{gps_driver,lidar_driver,actuator}_node.so
```

### 2. 配置串口/CAN 接口

```bash
# GPS 串口权限（默认 ttyUSB 只有 root 可读）
sudo usermod -aG dialout $USER
# 重新登录生效，或 sudo chmod 666 /dev/ttyUSB0 临时生效

# CAN 接口（MCP2515 示例）
# /boot/firmware/config.txt 加:
#   dtparam=spi=on
#   dtoverlay=mcp2515-can0,oscillator=16000000,interrupt=25
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up
```

### 3. 改配置

编辑 [config/pipeline_car.json](../config/pipeline_car.json)，改 `params` 里的设备路径和参数：

```json
// gps_driver: 改成你的 GPS 串口
"params": "{\"serial_port\":\"/dev/ttyUSB0\",\"baud_rate\":9600,...}"

// lidar_driver: 改成你的 LiDAR 串口 + DBSCAN 参数
"params": "{\"serial_port\":\"/dev/ttyUSB1\",\"baud_rate\":115200,...}"

// actuator: 改成你的 CAN 接口和 ESC 协议 ID
"params": "{\"can_interface\":\"can0\",\"can_throttle_id\":\"0x100\",...}"
```

### 4. 先 dry-run 测试

无硬件时所有驱动节点自动降级为 dry-run（只日志不发真实数据）。先用 dry-run 验证算法链：

```bash
# 把三个驱动的 dry_run 改成 1（或拔掉硬件自动降级）
./build/bin/flow_launcher config/pipeline_car.json
```

日志应看到：
- `[gps_driver] DRY-RUN 模式：打印假 NMEA 行`
- `[lidar_driver] DRY-RUN：生成模拟点云`
- `[actuator] [dry-run] CAN 0x100 [8] ...`
- `[fusion] localization: x=... y=... v=...`
- `[control] cmd #1 thr=... steer=...`

算法链跑通后，接真硬件把 dry_run 改回 0。

### 5. 真车启动

```bash
./build/bin/flow_launcher config/pipeline_car.json
```

用 `candump can0` 确认 actuator 在发 CAN 帧，用 FlowBoard 确认定位和障碍物数据。

## 适配点（改配置 vs 改代码）

| 硬件差异 | 改什么 | 改配置还是代码 |
|---------|--------|---------------|
| GPS 串口路径/波特率 | pipeline_car.json params | 改配置 |
| LiDAR 串口路径/波特率 | pipeline_car.json params | 改配置 |
| LiDAR 协议格式 | [lidar_driver_node.c](../modules/adas_nodes/lidar_driver_node.c) `read_lidar_frame()` | 改代码（~50 行） |
| CAN 接口名 | pipeline_car.json params | 改配置 |
| ESC 的 CAN ID | pipeline_car.json params | 改配置 |
| ESC 的 CAN 帧编码 | [actuator_node.c](../modules/adas_nodes/actuator_node.c) `encode_*_frame()` | 改代码（~20 行） |
| DBSCAN 聚类参数 | pipeline_car.json params | 改配置 |
| 控制目标速度 | pipeline_car.json params | 改配置 |

**大多数情况只需改配置**。需要改代码的两个适配点（LiDAR 协议、CAN 编码）都有清晰的函数边界和注释，按硬件手册填即可。

## 已知限制（后续改造方向）

1. **fusion 定位输入**：当前 fusion_node 的 EKF 从 `sensor/lidar` 的 x/y 字段取自车位置（仿真语义）。真车上 LiDAR 给的是障碍物点云。真车模式里 fusion 只用 GPS（速度+航向），位置不更新。后续需改造 fusion 从 GPS 经纬度推算局部坐标，或增加 IMU 输入。

2. **IMU 未接入**：`ImuData` 消息类型尚未定义，fusion 的 5D EKF 无 IMU predict 增量。高速场景需加 IMU 提升定位精度。后续在 `msg/adas_msgs.msg` 加 ImuData 结构 + 重新生成 + fusion 扩展 EKF 状态。

3. **safety_control 障碍物输入**：safety_control 原订阅 `vehicle/state`（JSON 含 ego+obstacles），真车模式改成订阅 `perception/obstacles`（ObstacleList 二进制）。safety_control 内部的障碍物解析逻辑需确认是否已适配 ObstacleList 格式。

4. **road/geometry 缺失**：真车上无 sim_world 发布 road/geometry，planning/control 按直道处理。需要地图节点（HD Map）提供道路几何。

5. **实时性**：Linux 非实时内核下 control 的 20Hz 周期可能抖动。建议树莓派用 `PREEMPT_RT` 内核或 `isolcpus` 隔离一个核给 control。

## 调试

### CAN 调试
```bash
sudo apt install can-utils
candump can0              # 监听所有 CAN 帧
cansend can0 100#E803000001000000  # 手动发一帧测试 ESC
```

### GPS 调试
```bash
# 直接读串口确认 GPS 输出
cat /dev/ttyUSB0          # 应看到 $GPRMC / $GPGGA 语句
stty -F /dev/ttyUSB0 9600 raw -echo  # 配波特率
```

### LiDAR 调试
```bash
# 确认串口有数据
cat /dev/ttyUSB1 | xxd | head    # 看原始字节
```

### FlowBoard 监控
```bash
# monitor 节点发布 /tmp/flow_topology.json，FlowBoard 读它
cd tools/flowboard && python3 -m http.server 8000
# 浏览器开 http://localhost:8000
```

## 参考

- [skills/15_socketcan_actuator.md](../skills/15_socketcan_actuator.md) — SocketCAN + actuator_node 深度教程
- [config/pipeline_car.json](../config/pipeline_car.json) — 真车配置模板
- [docs/SIMULATION_GUIDE.md](SIMULATION_GUIDE.md) — 仿真模式文档（对照参考）
