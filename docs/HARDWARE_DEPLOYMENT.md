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

pipeline_car.json 默认走 **L2 自主跟随**（RC 小车：航点跟随 + 局部避障）。把 `waypoint_follower` 换回 `planning`、`actuator_pwm` 换回 `actuator` 即切到真车高速 NOA 路径。

```
[真实GPS]    ──▶ gps_driver     ──▶ sensor/gps     ─┐
[真实IMU]    ──▶ imu_driver     ──▶ sensor/imu     ─┤
[真实LiDAR]  ──▶ lidar_driver   ─┐                  │
[OAK-D 双目] ──▶ stereo_camera  ──▶ sensor/stereo ──┼──▶ stereo_vision ──┐
                                   │                │                    │
                                   │                │   (lidar_driver)   │
                                   │                └──▶ lidar_driver ───┤
                                   │                                     ▼
                              ┌── sensor/lidar ──────┐            perception/obstacles
                              │                      │                    │
                              ▼                      │                    │
                           slam ◀── sensor/imu       │                    │
                              │                      │                    │
                              ▼                      ▼                    │
                        sensor/pose          fusion ──▶ fusion/localization ─┐
                              │                                  │            │
                              └──────────────────────────────────┤            │
                                                                 ▼            ▼
                                          waypoint_follower (L2) / planning (NOA)
                                                                 │
                                                                 ▼
                                                          control ──▶ control/raw_cmd
                                                                 │
                                                                 ▼
                                                         safety_control ──▶ control/cmd
                                                                 │
                                                                 ▼
                                          actuator_pwm (L2/RC) / actuator (NOA/CAN)
                                                                 │
                                          L2:  [PCA9685 I2C / GPIO PWM] ──▶ 舵机+电调
                                          NOA: [CAN 总线]               ──▶ ESC/舵机
```

**关键变化**：
- `sim_world` / `sensor_model` / `perception` 三个仿真节点**移除**
- 新增 `gps_driver`（真实 GPS 串口 → sensor/gps）
- 新增 `imu_driver`（真实 IMU 串口 → sensor/imu，100Hz）
- 新增 `lidar_driver`（真实 LiDAR → DBSCAN → perception/obstacles，替代 perception_node）
- 新增 `stereo_camera`（OAK-D 双目 → StereoFrame → sensor/stereo，10fps）
- 新增 `stereo_vision`（StereoFrame 深度反投影 → DBSCAN → perception/obstacles，与 lidar_driver 互补：双目近距稠密 0.5-8m / LiDAR 远距 8-60m）
- 新增 `slam`（订阅 sensor/lidar + sensor/imu → Pose2D → sensor/pose，20Hz）
- 新增 `waypoint_follower`（**L2**：Pure Pursuit 航点跟随 → planning/trajectory，替代 planning_node）
- 新增 `actuator_pwm`（**L2/RC**：control/cmd → PCA9685 I2C / GPIO PWM → 舵机+电调，替代 actuator_node 的 SocketCAN）
- `fusion` 升级为 **三源融合**（LiDAR + GPS + SLAM Pose2D），GPS 丢失时 SLAM 接管

## 节点清单

| 节点 | 库 | 输入 | 输出 | 硬件依赖 |
|------|----|------|------|---------|
| gps_driver | libgps_driver_node.so | 串口 NMEA | sensor/gps | GPS 模块（/dev/ttyUSB0） |
| imu_driver | libimu_driver_node.so | 串口 IMU 行 | sensor/imu | IMU 模块（/dev/ttyUSB2） |
| lidar_driver | liblidar_driver_node.so | 串口点云 | perception/obstacles, sensor/lidar | LiDAR（/dev/ttyUSB1） |
| stereo_camera | libstereo_camera_node.so | OAK-D USB | sensor/stereo | OAK-D 双目（USB） |
| stereo_vision | libstereo_vision_node.so | sensor/stereo | perception/obstacles | — |
| slam | libslam_node.so | sensor/lidar, sensor/imu | sensor/pose | — |
| fusion | libfusion_node.so | sensor/gps, sensor/pose | fusion/localization | — |
| waypoint_follower | libwaypoint_follower_node.so | fusion/localization, perception/obstacles | planning/trajectory | 航点文件（/tmp/waypoints.json） |
| planning | libplanning_node.so | fusion/localization, perception/obstacles | planning/trajectory | — |
| control | libcontrol_node.so | fusion/localization, planning/trajectory | control/raw_cmd | — |
| safety_control | libsafety_control_node.so | control/raw_cmd, perception/obstacles | control/cmd | — |
| actuator | libactuator_node.so | control/cmd | CAN 帧 | CAN 接口（can0） |
| actuator_pwm | libactuator_pwm_node.so | control/cmd | PWM 信号 | PCA9685 I2C / GPIO（L2/RC） |
| inference | libinference_node.so | control/cmd, fusion/localization, ... | inference/trajectory | — |
| data_recorder | libdata_recorder_node.so | fusion, planning, obstacles, cmd | JSONL 文件 | — |
| learner | liblearner_node.so | 同上 | learner/status | — |
| model_ota | libmodel_ota_node.so | model_ota/cmd, fusion | model_ota/* | — |
| monitor | libmonitor_node.so | obstacles, latency, ... | 仪表盘 JSON | — |

> **L2 vs NOA 二选一**：`waypoint_follower`（L2 RC 小车，Pure Pursuit）和 `planning`（NOA 高速，Frenet+状态机）都发 `planning/trajectory`，pipeline_car.json 里二选一。同理 `actuator_pwm`（L2 PWM）和 `actuator`（NOA CAN）都消费 `control/cmd`，二选一。

## 硬件准备

### 计算平台

| 平台 | CPU | 内存 | 能跑吗 |
|------|-----|------|--------|
| 树莓派 4B | 4×A72 @1.5GHz | 2-8GB | 轻松（算力需求 <10% 单核） |
| 树莓派 5 | 4×A76 @2.4GHz | 4-8GB | 富余（含 SLAM 算法） |
| Jetson Nano | 4×A57 @1.4GHz | 4GB | 轻松（GPU 用不上） |
| 香橙派 Zero2 | 4×A55 @1.5GHz | 1GB | 能跑（SLAM 建议换大内存） |

> 若启用 FAST-LIO2 / LIO-SAM 等真实 SLAM 算法，建议树莓派 5 或 Jetson Orin Nano（≥4GB RAM）。默认 dead-reckoning 占位算法任何平台都能跑。

### GPS 模块

| 模块 | 价格 | 接口 | 输出格式 | 配置 |
|------|------|------|---------|------|
| u-blox NEO-M8N | ¥40-80 | UART | NMEA 0183 | serial_port=/dev/ttyUSB0, baud=9600 |
| ATGM336H | ¥15-30 | UART | NMEA 0183 | 同上 |
| VK2828U7G5LF | ¥30 | USB | NMEA 0183 | serial_port=/dev/ttyACM0, baud=9600 |

接 USB-TTL 转换器后插入树莓派 USB 口，识别为 `/dev/ttyUSB0`。nmea_parser 支持 GGA + RMC 语句（经纬度、速度、航向），覆盖定位基本需求。

### IMU 模块

| 模块 | 价格 | 接口 | 输出 | 配置 |
|------|------|------|------|------|
| MPU6050 | ¥8-15 | I²C/SPI（接 USB-I²C 转 TTL） | 6 轴 accel+gyro | serial_port=/dev/ttyUSB2, baud=115200 |
| ICM-42688-P | ¥25-40 | SPI | 6 轴，低噪 | 同上 |
| Xsens MTi-300 | ¥1500+ | UART/USB | 9 轴 AHRS，工厂标定 | serial_port=/dev/ttyUSB2, baud=115200 |

> **适配点**：[imu_driver_node.c](../modules/adas_nodes/imu_driver_node.c) 的 `parse_imu_line()` 是硬件适配钩子。默认实现 ASCII 行协议（`ax,ay,az,gx,gy,gz,temp\n`）。按你的 IMU 模块协议改这一个函数即可，其余滤波 + publish 逻辑不变。

### LiDAR

| 型号 | 价格 | 接口 | 配置 |
|------|------|------|------|
| RPLIDAR A1/A2 | ¥100-400 | UART/USB | 需改 read_lidar_frame 接 RPLIDAR SDK |
| 速腾浦 M1 | ¥500+ | UDP/串口 | 需改 read_lidar_frame 接 SDK |
| Velodyne VLP-16 | ¥3000+ | UDP | 需改 read_lidar_frame 接 UDP 协议 |

> **适配点**：[lidar_driver_node.c](../modules/adas_nodes/lidar_driver_node.c) 的 `read_lidar_frame()` 函数是硬件适配钩子。默认实现 ASCII 行协议（`x,y,z,intensity\n`）。按你的 LiDAR 协议改这一个函数即可，其余 DBSCAN + publish 逻辑不变。

### 双目摄像头（OAK-D）

| 方案 | 价格 | 接口 | 说明 |
|------|------|------|------|
| OAK-D Lite | ¥800-1200 | USB 3.0 | 板载 Movidius NPU，深度计算零 CPU 负担 |
| OAK-D Pro | ¥1500-2000 | USB 3.0 | 含红外，弱光场景更好 |
| DIY 双目（两个 USB 摄像头） | ¥100-200 | 2×USB | 主机 CPU 跑立体匹配，需自标定 |

> **适配点**：[stereo_camera_node.c](../modules/adas_nodes/stereo_camera_node.c) 的 `read_stereo_frame()` 是硬件适配钩子，用 `#ifdef HAVE_DEPTHAI` 控制真实 OAK-D / dry-run 路径。装 `depthai` SDK 后 `cmake -DHAVE_DEPTHAI=ON` 启用真实采集；不装则自动降级为 dry-run（生成伪 JPEG + 80×60 模拟深度图）。

```bash
# OAK-D USB 权限（无需驱动，加 udev 规则即可）
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | \
  sudo tee /etc/udev/rules.d/80-oakd.rules
sudo udevadm control --reload-rules
```

### CAN 接口

| 方案 | 价格 | 说明 |
|------|------|------|
| MCP2515 SPI-CAN 扩展板 | ¥30 | 树莓派 GPIO SPI 接线，需 dtoverlay 配置 |
| USB-CAN 适配器 | ¥100-300 | 即插即用，无需接线 |

CAN 总线两端必须各接 **120Ω 终端电阻**，否则 BUS-OFF。详见 [skills/15_socketcan_actuator.md](../skills/15_socketcan_actuator.md)。

> CAN 接口对应 `actuator` 节点（真车高速 NOA）。RC 小车 L2 用下面的 PWM 方案，走 `actuator_pwm` 节点，无需 CAN。

### PWM 执行器（RC 小车 L2）

RC 小车用舵机（转向）+ 电调 ESC（油门/刹车），都是 PWM 信号驱动，不接 CAN。`actuator_pwm_node` 支持三种 backend：

| backend | 硬件 | 价格 | 说明 |
|---------|------|------|------|
| `pca9685` | PCA9685 I2C-PWM 扩展板 | ¥10 | 16 路 12-bit PWM，I2C 地址 0x40-0x7F，50Hz 舵机频率。**推荐**——一路 I2C 同时驱动转向+油门，不占 GPIO |
| `gpio` | 树莓派 GPIO 硬件 PWM | ¥0 | sysfs `/sys/class/pwm`，需 dtoverlay 开 PWM 通道。两路 PWM 各占一个 pin |
| `dry_run` | 无 | — | 只日志不发真实 PWM，开发调试用（I2C 打开失败也自动降级到此） |

**PCA9685 接线**（树莓派）：

```
树莓派           PCA9685
GPIO2 (SDA) ──── SDA
GPIO3 (SCL) ──── SCL
3.3V        ──── VCC（逻辑电源）
5V          ──── V+（舵机电源，独立供电！别从树莓派 5V 取，电流不够会重启）
GND         ──── GND
                CH0 ──▶ ESC 信号线（油门）
                CH1 ──▶ 舵机信号线（转向）
```

**PWM 参数约定**（50Hz，周期 20ms）：
- 中位 1500μs，油门范围 1000μs（全刹）~ 2000μs（全油门），转向范围 1000μs（左满）~ 2000μs（右满）
- `actuator_pwm_node` 把 ControlCmd 映射成：`esc_us = 1500 + throttle*500`，`steer_us = 1500 + steering*500`，e_stop → 1500 中位
- 带看门狗：`watchdog_timeout_s`（默认 3s）内无 cmd 强制 ESC 回中位，防 control 崩溃小车失控

**开启 I2C**（树莓派）：
```bash
sudo raspi-config nonint do_i2c 0
# 确认设备识别（addr 0x40 即 PCA9685 默认地址）
sudo i2cdetect -y 1
```

pipeline_car.json 配置：
```json
"params": "{\"backend\":\"pca9685\",\"i2c_bus\":1,\"i2c_addr\":64,
  \"esc_channel\":0,\"steer_channel\":1,\"pwm_freq_hz\":50,
  \"throttle_scale\":500,\"steering_scale\":500,\"watchdog_timeout_s\":3}"
```
> `i2c_addr` 是十进制（64 = 0x40）。无 PCA9685 时改 `"backend":"gpio"` 或 `"dry_run"`。

## 部署步骤

### 1. 编译

```bash
# 主框架（含 serial_port 串口库）
cmake -B build -S . && cmake --build build --target flowengine_core

# 节点插件
cmake -B build/modules/adas_nodes -S modules/adas_nodes
cmake --build build/modules/adas_nodes

# 可选：启用 OAK-D 真实采集（需先 pip install depthai）
cmake -B build/modules/adas_nodes -S modules/adas_nodes -DHAVE_DEPTHAI=ON
cmake --build build/modules/adas_nodes --target stereo_camera_node
```

确认产物：
```bash
ls build/lib/lib{gps_driver,imu_driver,lidar_driver,stereo_camera,slam,actuator}_node.so
```

### 2. 配置串口/CAN/USB 接口

```bash
# 串口权限（默认 ttyUSB 只有 root 可读）
sudo usermod -aG dialout $USER
# 重新登录生效，或 sudo chmod 666 /dev/ttyUSB0 临时生效

# CAN 接口（MCP2515 示例）
# /boot/firmware/config.txt 加:
#   dtparam=spi=on
#   dtoverlay=mcp2515-can0,oscillator=16000000,interrupt=25
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up

# OAK-D USB 权限（见上方"双目摄像头"小节）
```

### 3. 改配置

编辑 [config/pipeline_car.json](../config/pipeline_car.json)，改 `params` 里的设备路径和参数：

```json
// gps_driver: 改成你的 GPS 串口
"params": "{\"serial_port\":\"/dev/ttyUSB0\",\"baud_rate\":9600,...}"

// imu_driver: 改成你的 IMU 串口
"params": "{\"serial_port\":\"/dev/ttyUSB2\",\"baud_rate\":115200,\"sample_hz\":100,...}"

// lidar_driver: 改成你的 LiDAR 串口 + DBSCAN 参数
"params": "{\"serial_port\":\"/dev/ttyUSB1\",\"baud_rate\":115200,...}"

// stereo_camera: OAK-D 用默认参数；DIY 双目需改 baseline/fov
"params": "{\"baseline\":0.075,\"fov_h\":68.7938,\"fps\":10,...}"

// slam: 默认 dead reckoning 占位；接真实算法改 algo
"params": "{\"algo\":\"dead_reckoning\",\"pose_hz\":20,...}"
//   可选: "dead_reckoning" / "fast_lio2" / "lio_sam"
//   真实算法未编译时自动降级 + LOG_WARN

// actuator: 改成你的 CAN 接口和 ESC 协议 ID
"params": "{\"can_interface\":\"can0\",\"can_throttle_id\":\"0x100\",...}"
```

### 4. 先 dry-run 测试

无硬件时所有驱动节点自动降级为 dry-run（只日志不发真实数据）。先用 dry-run 验证算法链：

```bash
# 把驱动的 dry_run 改成 1（或拔掉硬件自动降级）
./build/bin/flow_launcher config/pipeline_car.json
```

日志应看到：
- `[gps_driver] DRY-RUN 模式：打印假 NMEA 行`
- `[imu_driver] DRY-RUN：静止状态模拟 (accel_z=9.8)`
- `[lidar_driver] DRY-RUN：生成模拟点云`
- `[stereo_camera] DRY-RUN：生成伪 JPEG + 模拟深度图`
- `[stereo_vision] obstacles: N clusters`
- `[slam] dead reckoning 模式，生成圆形轨迹`
- `[actuator_pwm] [dry-run] esc=1500us steer=1500us`（L2 RC）/ `[actuator] [dry-run] CAN 0x100 [8] ...`（NOA CAN）
- `[fusion] localization: x=... y=... v=...`
- `[waypoint_follower] plan: N wp_idx=I lap=L target_speed=S`（L2）
- `[control] cmd #1 thr=... steer=...`

算法链跑通后，接真硬件把 dry_run 改回 0。slam 的 `algo` 改成 `fast_lio2` 或 `lio_sam` 启用真实 SLAM（需先编译对应算法库）。

### 5. 真车启动

```bash
./build/bin/flow_launcher config/pipeline_car.json
```

- **L2 RC 小车**：确认 `actuator_pwm` 在发 PWM（看日志 `esc=XXXXus steer=XXXXus`），用 FlowBoard 确认定位和航点跟随进度（`lap` 计数）
- **NOA 真车**：用 `candump can0` 确认 actuator 在发 CAN 帧，用 FlowBoard 确认定位和障碍物数据

## L2 自主跟随快速上手（RC 小车）

L2 模式 = 预录制航点 + Pure Pursuit 跟随 + 前方障碍物减速停车。**不需要地图、不需要全局规划器**，靠 GPS 航点跑固定路线。流程：录制航点 → 跑。

### 1. 录制航点

用 [tools/waypoint_record.py](../tools/waypoint_record.py) 接 GPS 串口，人推/遥控小车走一遍路线，按需打点：

```bash
# 实时录制（接 GPS 串口）
python3 tools/waypoint_record.py \
    --serial /dev/ttyUSB0 --baud 9600 \
    --out /tmp/waypoints.json \
    --min-distance 1.0          # 至少移动 1m 才打新点，避免抖动重复

# 或从已有 NMEA 日志回放录制
python3 tools/waypoint_record.py --replay gps_log.txt --out /tmp/waypoints.json
```

输出 JSON 格式（waypoint_follower_node 直接读）：
```json
{
  "lookahead_m": 1.5,
  "cruise_speed": 2.0,
  "origin_lat": 31.2304,
  "origin_lon": 121.4737,
  "waypoints": [
    {"x": 0.0, "y": 0.0, "lat": ..., "lon": ...},
    {"x": 1.2, "y": 0.1, "lat": ..., "lon": ...}
  ]
}
```
> 坐标转换：以第一个 GPS 点为原点，`x = Δlon·cos(lat)·111320`，`y = Δlat·110540`（平面近似，<1km 误差 <1%）。`x/y` 是 waypoint_follower 用的局部笛卡尔坐标，`lat/lon` 保留供核对。

### 2. 检查/重采样航点

用 [tools/waypoint_player.py](../tools/waypoint_player.py) 看路径统计和 ASCII 图：

```bash
python3 tools/waypoint_player.py /tmp/waypoints.json
# 输出：总长 X.X m，N 个点，间距均值/最大值，ASCII 路径图

# 按固定间距重采样（消除录制时的疏密不均）
python3 tools/waypoint_player.py /tmp/waypoints.json --resample 1.0 --out /tmp/waypoints_resampled.json
```

### 3. 配置 waypoint_follower

pipeline_car.json 里 `waypoint_follower` 节点的 params：

```json
"params": "{\"waypoints_file\":\"/tmp/waypoints.json\",
  \"loop\":1,\"cruise_speed\":2.0,\"lookahead_m\":1.5,\"plan_hz\":10,
  \"obstacle_slow_dist\":3.0,\"obstacle_stop_dist\":0.8}"
```

| 参数 | 含义 | RC 小车典型值 |
|------|------|--------------|
| `waypoints_file` | 航点 JSON 路径 | `/tmp/waypoints.json` |
| `loop` | 1=到终点绕回第 0 点循环跑；0=到终点停车 | 1 |
| `cruise_speed` | 巡航速度 m/s | 2.0（≤5） |
| `lookahead_m` | Pure Pursuit 前瞻距离 | 1.5（速度高可加大） |
| `plan_hz` | 规划频率 | 10 |
| `obstacle_slow_dist` | 障碍物在此距离开始线性减速 | 3.0 |
| `obstacle_stop_dist` | 障碍物在此距离完全停车 | 0.8 |

> **避障策略**：L2 只做**减速停车**，不做绕行。前方 3m 内有障碍物开始减速，0.8m 停车。绕行需 L3 全局规划器（待后续）。`perception/obstacles` 由 `stereo_vision`（近距）或 `lidar_driver`（远距）提供。

### 4. 跑

```bash
./build/bin/flow_launcher config/pipeline_car.json
```

日志关注 `[waypoint_follower]` 的 `wp_idx`（当前跟踪航点序号）和 `lap`（圈数）。小车会沿航点循环跑，遇障减速停车，障碍物移开后自动恢复。

> **RC 小车控制参数**已在 pipeline_car.json 调好：`wheelbase:0.3`（轴距）、`target_speed:2.0`、`pid_kp:300/ki:20/kd:40`、`steer_min_clamp:0.02`、`max_steer:0.35`。真车高速场景把这些改回（`wheelbase:2.7`、`target_speed:15` 等）即可。

## 适配点（改配置 vs 改代码）

| 硬件差异 | 改什么 | 改配置还是代码 |
|---------|--------|---------------|
| GPS 串口路径/波特率 | pipeline_car.json params | 改配置 |
| IMU 串口路径/波特率/采样率 | pipeline_car.json params | 改配置 |
| LiDAR 串口路径/波特率 | pipeline_car.json params | 改配置 |
| LiDAR 协议格式 | [lidar_driver_node.c](../modules/adas_nodes/lidar_driver_node.c) `read_lidar_frame()` | 改代码（~50 行） |
| IMU 协议格式 | [imu_driver_node.c](../modules/adas_nodes/imu_driver_node.c) `parse_imu_line()` | 改代码（~30 行） |
| SLAM 算法选择 | pipeline_car.json params `algo` | 改配置（算法库需预编译） |
| SLAM 算法接入 | [slam_node.c](../modules/adas_nodes/slam_node.c) `slam_update()` | 改代码（按算法 SDK） |
| OAK-D 参数（baseline/fov/fps） | pipeline_car.json params | 改配置 |
| DIY 双目采集 | [stereo_camera_node.c](../modules/adas_nodes/stereo_camera_node.c) `read_stereo_frame()` | 改代码（接 OpenCV stereoBM） |
| CAN 接口名 | pipeline_car.json params | 改配置 |
| ESC 的 CAN ID | pipeline_car.json params | 改配置 |
| ESC 的 CAN 帧编码 | [actuator_node.c](../modules/adas_nodes/actuator_node.c) `encode_*_frame()` | 改代码（~20 行） |
| **PWM backend 选择**（pca9685/gpio/dry_run） | pipeline_car.json params `backend` | 改配置 |
| **PCA9685 I2C 地址/通道** | pipeline_car.json params `i2c_addr`/`esc_channel`/`steer_channel` | 改配置 |
| **航点文件路径** | pipeline_car.json params `waypoints_file` | 改配置 |
| **航点录制**（GPS→JSON） | [tools/waypoint_record.py](../tools/waypoint_record.py) | 用工具 |
| **轴距 wheelbase**（RC 0.3 / 真车 2.7） | pipeline_car.json control params `wheelbase` | 改配置 |
| **Pure Pursuit 前瞻距离** | pipeline_car.json params `lookahead_m` | 改配置 |
| DBSCAN 聚类参数 | pipeline_car.json params | 改配置 |
| 控制目标速度 | pipeline_car.json params | 改配置 |

**大多数情况只需改配置**。需要改代码的适配点都有清晰的函数边界和注释，按硬件手册填即可。

## 已知限制（后续改造方向）

1. ~~**fusion 定位输入**~~ **[已解决]**：fusion_node 现为 **三源融合**（LiDAR 位置 + GPS 速度/航向 + SLAM Pose2D 位置+航向）。当 `sensor/pose`（Pose2D）的 `converged=1` 且协方差 <100 时，用 `ekf_fusion_update_gps_full` 接入 SLAM 位置+航向；否则回退 LiDAR 位置更新。GPS 丢失（室内/隧道）场景 SLAM 自动接管定位。

2. **slam 默认是占位算法**：默认 `dead_reckoning` 只做航位推算（圆形轨迹模拟），精度随时间漂移。`fast_lio2` 已提供编译开关 `ENABLE_FAST_LIO2`（占位实现，编译能过但运行时降级到 dead_reckon + LOG_WARN 提示填充真实 ESKF 调用）；`lio_sam` 选项已预留接口（`slam_update()` 钩子）。完整集成步骤见下文 [FAST-LIO2 集成](#fast-lio2-集成) 章节。

3. **safety_control 障碍物输入**：safety_control 原订阅 `vehicle/state`（JSON 含 ego+obstacles），真车模式改成订阅 `perception/obstacles`（ObstacleList 二进制）。safety_control 内部的障碍物解析逻辑需确认是否已适配 ObstacleList 格式。

4. **road/geometry 缺失**：真车上无 sim_world 发布 road/geometry，planning/control 按直道处理。L2 模式不依赖 road/geometry（航点即路线），NOA 模式需要地图节点（HD Map）提供道路几何。

5. ~~**stereo_camera 下游消费方**~~ **[已解决]**：新增 `stereo_vision_node` 订阅 `sensor/stereo`，把 80×60 深度图反投影成 3D 点云（针孔模型：`X=depth·cos(θ)`, `Y=depth·sin(θ)`），DBSCAN 聚类后输出 `perception/obstacles`。与 lidar_driver 互补：双目测近距稠密（0.5-8m），LiDAR 测远距（8-60m）。pipeline_car.json 默认 lidar_driver 关、stereo_vision 开。

6. **L2 不做绕行**：waypoint_follower 只做障碍物减速停车（3m 减速、0.8m 停车），不做路径绕行。绕行需 L3 全局规划器 + 占据栅格（traversability_node + 全局重规划），待后续。

7. **实时性**：Linux 非实时内核下 control 的 20Hz 周期可能抖动。建议树莓派用 `PREEMPT_RT` 内核或 `isolcpus` 隔离一个核给 control。启用真实 SLAM 算法时更要注意线程优先级。

## 调试

### CAN 调试
```bash
sudo apt install can-utils
candump can0              # 监听所有 CAN 帧
cansend can0 100#E803000001000000  # 手动发一帧测试 ESC
```

### PWM 调试（L2 RC 小车）
```bash
# 1. 确认 I2C 总线和 PCA9685 地址（0x40 = 十进制 64）
sudo i2cdetect -y 1
#      0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
# 00:                         -- -- -- -- -- -- -- --
# 40: 40 -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
#  ↑ PCA9685 默认地址

# 2. actuator_pwm_node dry-run（不接硬件，看 cmd→PWM 映射对不对）
#    pipeline_car.json 里 actuator params 改 "dry_run":1
./build/bin/flow_launcher config/pipeline_car.json
# 日志应看到: [actuator_pwm] [dry-run] esc=1500us steer=1500us
#   油门时 esc=2000us，刹车时 esc=1000us，左转 steer=1000us

# 3. 看门狗测试：跑起来后 kill -STOP <control_pid> 暂停 control
#    3 秒后 actuator_pwm 日志应出现 watchdog 触发、ESC 回中位 1500us
```

### 航点调试（L2）
```bash
# 看航点路径形状和间距
python3 tools/waypoint_player.py /tmp/waypoints.json

# waypoint_follower 运行时看跟踪进度（日志）
# [waypoint_follower] plan: 8 wp_idx=3 lap=1 target_speed=1.5
#   wp_idx = 当前跟踪的航点序号
#   lap = 圈数（loop=1 时持续累加）
#   target_speed = 当前目标速度（遇障会从 cruise_speed 降到 0）
```

### GPS 调试
```bash
# 直接读串口确认 GPS 输出
cat /dev/ttyUSB0          # 应看到 $GPRMC / $GPGGA 语句
stty -F /dev/ttyUSB0 9600 raw -echo  # 配波特率
```

### IMU 调试
```bash
# 确认 IMU 串口有数据
cat /dev/ttyUSB2          # 应看到 ax,ay,az,gx,gy,gz,temp 行
stty -F /dev/ttyUSB2 115200 raw -echo
```

### LiDAR 调试
```bash
# 确认串口有数据
cat /dev/ttyUSB1 | xxd | head    # 看原始字节
```

### OAK-D 调试
```bash
# 确认 USB 识别
lsusb | grep 03e7              # 应看到 Movidius 设备
# depthai 自带 demo 验证硬件
python3 -c "import depthai as d; print(d.__version__)"
```

### SLAM 调试
```bash
# 看 slam 节点输出位姿
# FlowBoard 仪表盘的"定位"卡片会显示 sensor/pose
# 或订阅 sensor/pose topic 查看 Pose2D.converged 字段
#   converged=0 → dead reckoning（漂移）
#   converged=1 → 真实 SLAM（如 fast_lio2 已启用）
```

### FlowBoard 监控
```bash
# monitor 节点发布 /tmp/flow_topology.json，FlowBoard 读它
cd tools/flowboard && python3 -m http.server 8000
# 浏览器开 http://localhost:8000
```

## FAST-LIO2 集成

默认 `slam_node` 用 dead reckoning 占位，长时间会漂移。真车部署建议接入 [FAST-LIO2](https://github.com/hku-mars/FAST_LIO)（LiDAR+IMU 紧耦合，港大 MARS 实验室），误差不发散、室内/隧道/GPS 丢失场景能稳定位。

代码侧已预留编译开关 `HAVE_FAST_LIO2` 与占位实现：`slam_update()` 按 `algo` 路由到 `slam_update_fast_lio2()`，后者当前为占位（编译能过、运行时 LOG_WARN 降级到 dead reckoning）。社区用户**只需填三个函数 + 链库**，订阅/发布/线程框架无需改动。

### 步骤 1：编译 libfast_lio2.a

```bash
# 依赖：PCL ≥1.8、Eigen ≥3.3、Sophus、yaml-cpp
sudo apt install libpcl-dev libeigen3-dev libsophus-dev libyaml-cpp-dev

git clone https://github.com/hku-mars/FAST_LIO.git
cd FAST_LIO && mkdir build && cd build
cmake .. && make -j
# 产出 libfast_lio2.a（路径如 ~/FAST_LIO/build/libfast_lio2.a）
```

### 步骤 2：把 slam_node.c 改名为 .cpp 并填充占位

FAST-LIO2 是 C++ 模板库（ESKF），不能用 C 编译。把 [modules/adas_nodes/slam_node.c](../modules/adas_nodes/slam_node.c) 重命名为 `slam_node.cpp`，并填三个占位函数（位置见源码 `#ifdef HAVE_FAST_LIO2` 块）：

```cpp
// 文件顶部加 include
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include "IKFoM_toolkit/esekfom/esekfom.hpp"
#include "use-ikfom.hpp"

// 1. fast_lio2_init(): 初始化 ESKF + 加载 LiDAR 内参/外参
static int fast_lio2_init(void) {
    esekfom_ = std::make_shared<esekfom::esekfom<...>>();
    // 读 config/<lidar>.yaml: 内参矩阵、外参 (LiDAR↔IMU)、噪声参数
    return 0;
}

// 2. slam_update_fast_lio2(): 每周期被 slam_thread 调用
static void slam_update_fast_lio2(Pose2D* pose) {
    pthread_mutex_lock(&g.lock);
    ImuData    imu   = g.last_imu;     // 拷贝，避免长时间持锁
    LidarFrame lidar = g.last_lidar;
    pthread_mutex_unlock(&g.lock);

    esekfom_->predict(imu);            // IMU 预测（高频）
    esekfom_->update(lidar);           // LiDAR 配准更新
    auto state = esekfom_->get_state();

    pose->x        = state.pos(0);
    pose->y        = state.pos(1);
    pose->heading   = state.rot.yaw();  // 或 2D 投影 yaw
    pose->cov_xx   = state.cov(0, 0);
    pose->cov_yy   = state.cov(1, 1);
    pose->cov_hh   = state.cov(5, 5);
    pose->converged = true;
    pose->source    = POSE_SOURCE_SLAM;  // 高精度源，fusion 据此给高权重
}

// 3. fast_lio2_cleanup(): 释放 ESKF/点云缓冲
static void fast_lio2_cleanup(void) { esekfom_.reset(); }
```

### 步骤 3：启用编译开关 + 取消注释链接

[modules/adas_nodes/CMakeLists.txt](../modules/adas_nodes/CMakeLists.txt) 中 `ENABLE_FAST_LIO2` 块已预留链接模板，取消注释 `find_package` 与 `target_link_libraries` 行：

```bash
cd /workspace
cmake -B build/modules/adas_nodes -S modules/adas_nodes \
      -DFLOWENGINE_BUILD=$PWD/build \
      -DENABLE_FAST_LIO2=ON
cmake --build build/modules/adas_nodes --target slam_node
```

### 步骤 4：pipeline 切换 algo

```json
{
  "name": "slam",
  "params": "{\"enable\":1,\"algo\":\"fast_lio2\",\"publish_hz\":20,...}"
}
```

启动后日志应看到 `slam: FAST-LIO2 backend initialized` 而非降级警告。`sensor/pose` 的 `converged=1` + `source=2 (POSE_SOURCE_SLAM)` 表示真实 SLAM 已接管，fusion 会按高权重采信。

### 验证

```bash
# 看 source 字段：2 = SLAM（高精度），3 = 里程计（dead reckon，低精度）
./build/bin/flowctl topic echo sensor/pose
# FlowBoard 定位卡片应显示稳定不漂移的位姿轨迹
```

### 备注

- **LIO-SAM** 集成思路一致（同样在 `slam_update_fast_lio2` 调 `factor_graph->optimize()`），但依赖 GTSAM 且未提供 `ENABLE_LIO_SAM` 编译开关，需自行添加。
- **Cartographer** 不需 IMU 也能跑，可仿照本模式加 `ENABLE_CARTOGRAPHER` 开关 + `slam_update_cartographer()` 占位。
- 占位实现编译能过是为了让 `cmake -DENABLE_FAST_LIO2=ON` 在未填充真实代码时也能跑通 CI，但运行时会 LOG_WARN 降级——**真车部署前务必填三个函数**。

## 参考

- [skills/15_socketcan_actuator.md](../skills/15_socketcan_actuator.md) — SocketCAN + actuator_node 深度教程（NOA 真车）
- [config/pipeline_car.json](../config/pipeline_car.json) — 真车配置模板（默认 L2 RC，可切 NOA）
- [tools/waypoint_record.py](../tools/waypoint_record.py) — GPS 航点录制工具（NMEA → JSON）
- [tools/waypoint_player.py](../tools/waypoint_player.py) — 航点查看/重采样工具（ASCII 路径图）
- [modules/adas_nodes/waypoint_follower_node.c](../modules/adas_nodes/waypoint_follower_node.c) — L2 Pure Pursuit 规划器源码
- [modules/adas_nodes/actuator_pwm_node.c](../modules/adas_nodes/actuator_pwm_node.c) — RC PWM 执行器源码（PCA9685/GPIO）
- [modules/adas_nodes/stereo_vision_node.c](../modules/adas_nodes/stereo_vision_node.c) — 双目深度反投影感知源码
- [msg/adas_msgs.msg](../msg/adas_msgs.msg) — 消息类型 IDL（含 ImuData / Pose2D / StereoFrame）
- [docs/SIMULATION_GUIDE.md](SIMULATION_GUIDE.md) — 仿真模式文档（对照参考）
