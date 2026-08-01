# RC 小车硬件连接操作清单

> 面向 L2 航点跟随（GPS + 航点 + Pure Pursuit → 舵机/电调 PWM）。
> 这是从仿真到真车**第一步**，把代码烧到物理小车跑起来。

---

## 一、购物清单

### 必备

| 物品 | 规格 | 参考价 | 用途 |
|------|------|--------|------|
| RC 小车底盘 | 1/10 ~ 1/14 比例，带舵机+电调+电机 | ¥200-500 | 车体 |
| 树莓派 4B / 5 | 4GB+ RAM，带 WiFi | ¥300-600 | 主控 |
| 32GB+ TF 卡 | Class 10，建议 64GB | ¥40-80 | 系统盘 |
| GPS 模块 | NEO-6M / NEO-8M / NEO-M9N，串口输出 NMEA | ¥30-100 | 定位 |
| 舵机 | RC 小车自带（通常 SG90/MG996R 级别） | 随底盘 | 转向 |
| 电调 ESC | RC 小车自带（有刷/无刷） | 随底盘 | 油门/刹车 |
| 锂电池 | 2S~3S Lipo，匹配电调电压 | ¥80-200 | 动力 |
| 移动电源 | 5V 2A+ 输出，给树莓派供电 | ¥50-100 | 树莓派供电 |

### 可选（按需加）

| 物品 | 规格 | 参考价 | 用途 |
|------|------|--------|------|
| PCA9685 I2C-PWM 扩展板 | 16 路 12-bit PWM，I2C 地址 0x40 | ¥10 | 舵机+电调 PWM 输出（推荐） |
| USB-TTL 模块 | CH340 / CP2102 | ¥10 | 串口调试/IMU 接入 |
| MCP2515 SPI-CAN 扩展板 | SPI-CAN 模块 | ¥30 | 后续升级 CAN 总线用 |
| OAK-D 双目摄像头 | 板载 NPU 深度计算 | ¥800-1200 | 视觉避障 |
| IMU 模块 | MPU6050 / ICM-42688 串口模块 | ¥20-60 | 姿态/航向 |
| RPLIDAR A1/A2 | 2D LiDAR，串口 | ¥300-800 | 激光感知 |

### 工具

| 物品 | 用途 |
|------|------|
| 十字螺丝刀套装 | 固定树莓派、PCA9685 |
| 杜邦线（公母各若干） | 接线 |
| 热缩管 + 电烙铁（可选） | 焊 SMA 头/电源线 |
| USB 键盘/鼠标 + HDMI 线 | 首次配置树莓派 |
| 网线 | 首次 SSH 配置 |

---

## 二、接线图

### 最小系统接线（树莓派 + PCA9685 + 舵机 + 电调 + GPS）

```
  ┌─────────────────────────────────────────────────────────┐
  │                    树莓派 4B (GPIO)                      │
  │                                                         │
  │  GPIO2 (SDA)  ═══════════════════════════════ PCA9685   │
  │  GPIO3 (SCL)  ═══════════════════════════════ SDA       │
  │  5V            ═══════════════════════════════ SCL       │
  │  GND           ═══════════════════════════════ VCC       │
  │                                     GND       │         │
  │  ┌─ USB ── GPS 模块 ─── /dev/ttyUSB0          │         │
  │  │  USB-TTL(CH340) ── GPS TX ──→ RX          │         │
  │  │  GPS RX ←─────────── TX                    │         │
  │  │  GPS VCC ── 5V                            │         │
  │  │  GPS GND ── GND                           │         │
  │  │                                           │         │
  │  └───────────────────────────────────────────┘         │
  └─────────────────────────────────────────────────────────┘
                                │
              ┌─────────────────┴──────────────────┐
              │          PCA9685 扩展板              │
              │                                     │
              │  CH0 (PWM0) ──── 电调 ESC 信号线     │
              │  CH1 (PWM1) ──── 舵机 信号线         │
              │  V+ ──────────── 电调 BEC 输出 5V    │
              │  GND ─────────── 电调 GND + 舵机 GND  │
              └─────────────────────────────────────┘
```

> **关键接线约束**：
> 1. 电调 BEC（5V 输出）接 PCA9685 V+ 和舵机 VCC，**不要**接树莓派 5V（大电流舵机会拉崩树莓派）
> 2. 树莓派 GND **必须**和电调 GND 共地，否则 PWM 信号无法形成回路
> 3. PCA9685 逻辑电源（VCC）接树莓派 3.3V 或 5V，GND 共地
> 4. GPS 模块 TX 接树莓派 RX（USB-TTL 自动处理，无需额外接线）

### 无 PCA9685 的最小接线（GPIO 直连 PWM）

```
  树莓派 GPIO12 (PWM0) ──── 电调 ESC 信号线
  树莓派 GPIO13 (PWM1) ──── 舵机 信号线
  树莓派 GND ────────────── 电调 GND + 舵机 GND
```

> 需要启用硬件 PWM：
> ```bash
> # /boot/firmware/config.txt 加
> dtoverlay=pwm-2chan
> # 禁用板载音频（否则 PWM 和音频共用时钟，频率不准）
> dtparam=audio=off
> ```

---

## 三、树莓派系统配置

### 3.1 烧录系统

```bash
# 推荐 Raspberry Pi OS Lite (64-bit, Bookworm)
# 用 Raspberry Pi Imager 烧录到 TF 卡

# 烧录时提前配置（Imager 右下角齿轮图标）：
#   - SSH 开启
#   - 用户名/密码: pi/flowengine
#   - WiFi: 你的 SSID
#   - 地区: CN
```

### 3.2 基础配置

```bash
# SSH 登录
ssh pi@raspberrypi.local

# 更新系统
sudo apt update && sudo apt upgrade -y

# 安装编译依赖
sudo apt install -y build-essential cmake git python3-pip \
  libi2c-dev i2c-tools

# 安装 Python 依赖（航点录制工具用）
pip3 install pyserial

# 启用 I2C（PCA9685 用）
sudo raspi-config nonint do_i2c 0

# 验证 I2C 设备
sudo i2cdetect -y 1
# 应该看到地址 0x40（PCA9685 默认地址）
```

### 3.3 配置串口（GPS 用）

```bash
# 确认 USB 串口设备
ls -l /dev/ttyUSB*

# 将当前用户加入 dialout 组（免 sudo 读写串口）
sudo usermod -aG dialout $USER

# 退出重登录后验证
groups
# 应包含 dialout

# 测试 GPS 数据
stty -F /dev/ttyUSB0 9600 raw -echo
cat /dev/ttyUSB0 | head -20
# 应看到 $GPGGA / $GPRMC 等 NMEA 语句

# 永久配置 /dev/ttyUSB0 权限（可选）
echo 'KERNEL=="ttyUSB[0-9]*", MODE="0666"' | \
  sudo tee /etc/udev/rules.d/99-usb-serial.rules
sudo udevadm control --reload-rules
```

### 3.4 配置 GPIO PWM（无 PCA9685 时）

```bash
# /boot/firmware/config.txt 追加
echo "dtoverlay=pwm-2chan" | sudo tee -a /boot/firmware/config.txt
echo "dtparam=audio=off" | sudo tee -a /boot/firmware/config.txt

# 重启生效
sudo reboot
```

---

## 四、编译 FlowEngine

```bash
# 克隆代码
git clone https://github.com/caixuf/FlowEngine.git
cd FlowEngine

# 编译
mkdir -p build && cd build
cmake ..
cmake --build . -j$(nproc)

# 编译节点插件
cmake --build modules/adas_nodes -j$(nproc)

# 验证关键节点
ls -la lib/libgps_driver_node.so      # GPS 驱动
ls -la lib/libwaypoint_follower_node.so  # 航点跟随
ls -la lib/libcontrol_node.so          # 控制
ls -la lib/libsafety_control_node.so   # 安全控制
ls -la lib/libactuator_pwm_node.so     # PWM 执行器
```

---

## 五、分步测试流程

### 步骤 1：干跑测试（无硬件，验证软件链）

```bash
# 用 dry_run 模式启动 pipeline_car.json
./build/bin/flow_launcher config/pipeline_car.json --duration 10

# 验证输出
cat /tmp/flow_topology.json | python3 -c "
import json,sys; d=json.load(sys.stdin)
print('Entities:', len(d['metrics']['scene']['entities']))
print('Ego:', d['metrics']['scene']['ego'])
"
```

### 步骤 2：GPS 测试

```bash
# 确认 GPS 串口数据
cat /dev/ttyUSB0 | head -5
# 应输出类似: $GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47

# 录制航点（推车走一圈，按 Ctrl-C 停止）
python3 tools/waypoint_record.py /dev/ttyUSB0 --out /tmp/waypoints.json --interval 0.5

# 查看录制的航点
python3 tools/waypoint_player.py /tmp/waypoints.json --plot
```

### 步骤 3：舵机/电调标定

```bash
# 用 actuator_pwm dry_run 测试 PWM 输出范围
# 改 pipeline_car.json actuator 的 dry_run 为 0

# 手动发 PWM 信号测试舵机（用 pwm_test.py 工具）
python3 -c "
import smbus, time
bus = smbus.SMBus(1)
addr = 0x40
# PCA9685 初始化
bus.write_byte_data(addr, 0x00, 0x20)  # MODE1: sleep off
time.sleep(0.01)
bus.write_byte_data(addr, 0xFE, 0x79)  # PRE_SCALE: 50Hz
bus.write_byte_data(addr, 0x00, 0x80)  # MODE1: restart
time.sleep(0.01)
# 通道 0 (电调): 1500us 中位
bus.write_byte_data(addr, 0x06, 0x70)  # ON_L
bus.write_byte_data(addr, 0x07, 0x17)  # ON_H
bus.write_byte_data(addr, 0x08, 0xDC)  # OFF_L (1500us)
bus.write_byte_data(addr, 0x09, 0x05)  # OFF_H
print('PCA9685 CH0 1500us 已输出')
"
```

### 步骤 4：电调行程校准

> **重要**：首次使用电调必须先校准油门行程，否则电调不响应。

```bash
# 标准校准流程（大多数 RC 电调）：
# 1. 电调断电
# 2. 按住电调上的 SET 按键（如有）
# 3. 电调上电，LED 闪烁
# 4. 此时 PCA9685 输出最大油门（2000us）
# 5. 听到提示音后，PCA9685 输出最小油门（1000us）
# 6. 再听到提示音，校准完成
# 7. 电调断电重启

# 手动输出最大油门
python3 -c "
import smbus, time
bus = smbus.SMBus(1)
addr = 0x40
bus.write_byte_data(addr, 0x00, 0x20)
time.sleep(0.01)
bus.write_byte_data(addr, 0xFE, 0x79)
bus.write_byte_data(addr, 0x00, 0x80)
time.sleep(0.01)
# 2000us (最大油门)
bus.write_byte_data(addr, 0x06+0*4, 0xD0); bus.write_byte_data(addr, 0x07+0*4, 0x07)
bus.write_byte_data(addr, 0x08+0*4, 0x40); bus.write_byte_data(addr, 0x09+0*4, 0x1C)
print('ESC max throttle (2000us)')
"
```

### 步骤 5：全流程真车测试

```bash
# 1. 确认舵机/电调已标定，车轮离地（用砖块垫起底盘）

# 2. 修改 pipeline_car.json 参数：
#    - gps_driver: dry_run=0, serial_port=/dev/ttyUSB0
#    - actuator: dry_run=0, backend=pca9685
#    - control: target_speed=1.0 (慢速，首次跑)

# 3. 启动
./build/bin/flow_launcher config/pipeline_car.json

# 4. 观察日志
#    - [gps_driver] 应看到 GPS 数据发布（lat/lon/spd）
#    - [waypoint_follower] 应看到航点号
#    - [control] 应看到 throttle/steer 输出
#    - [actuator_pwm] 应看到 PWM 值

# 5. 车放地上，检查舵机响应（原地打方向确认左右正确）
#    正确：左转舵机向左，右转舵机向右
#    错误：舵机反向 → 改 params.steering_scale 符号

# 6. 车轮离地再测油门
#    - 观察油门响应：正油门车轮正转，负油门反转
#    - 错误：油门反向 → 改 params.throttle_scale 符号

# 7. 全部正常后，放地上，低速（target_speed=0.5）试跑
```

---

## 六、pipeline_car.json 参数速查

### GPS 驱动

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `serial_port` | `/dev/ttyUSB0` | GPS 串口设备 |
| `baud_rate` | 9600 | NMEA 波特率（大多 GPS 模块 9600） |
| `dry_run` | 0 | 1=模拟数据，不读真串口 |

### 航点跟随

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `waypoints_file` | `/tmp/waypoints.json` | 录制的航点文件 |
| `loop` | 1 | 1=到达终点后回到起点循环 |
| `cruise_speed` | 2.0 | 巡航速度 (m/s)，首次跑设 0.5~1.0 |
| `lookahead_m` | 1.5 | Pure Pursuit 前瞻距离 (m)，越大越平滑 |
| `obstacle_stop_dist` | 0.8 | 障碍物停车距离 (m) |

### 控制

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `target_speed` | 2.0 | 目标速度 (m/s)，首次跑设为 0.5 |
| `pid_kp` | 300.0 | 纵向 PID 比例系数 |
| `pid_ki` | 20.0 | 纵向 PID 积分系数 |
| `lat_kp` | 0.8 | 横向 Stanley 比例系数 |
| `wheelbase` | 0.3 | 轴距 (m)，RC 小车通常 0.25~0.30 |

### PWM 执行器

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `backend` | `pca9685` | `pca9685` / `gpio` / `dry_run` |
| `i2c_bus` | 1 | 树莓派 4B 是 1，5 是 1 |
| `i2c_addr` | 64 | PCA9685 地址，0x40=64 |
| `esc_channel` | 0 | PCA9685 通道 0 接电调 |
| `steer_channel` | 1 | PCA9685 通道 1 接舵机 |
| `throttle_scale` | 500 | 油门缩放（1500±500=1000~2000us） |
| `steering_scale` | 500 | 舵机缩放（1500±500=1000~2000us） |
| `watchdog_timeout_s` | 3 | 3 秒无指令 → 强制中位刹停 |
| `dry_run` | 0 | 1=只日志不发 PWM |

### 安全控制

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `max_throttle` | 0.6 | 最大油门限幅 (0~1) |
| `max_steer` | 0.35 | 最大转向限幅 (0~1) |
| `time_headway` | 1.0 | 时距 (s)，前车距离/速度 |
| `low_speed_steer` | 0.35 | 低速时最大转向角（大一点好掉头） |

---

## 七、调试清单

### 跑不起来

| 现象 | 原因 | 检查 |
|------|------|------|
| 舵机不动 | PCA9685 没供电/没接线 | `i2cdetect -y 1` 看到 0x40？ |
| 电调不叫 | 行程未校准 | 看步骤 4 走一遍校准 |
| 车不动 | 电调安全模式 | 电调上电后等提示音，通常需先拉油门到底再回中 |
| 舵机抖动 | GND 不共地 | 树莓派 GND 接电调 GND |
| 舵机反向 | 接到反向通道 | 把 steering_scale 改负号 |
| 不走直线 | 舵机中位偏 | 改舵机拉杆物理微调 |
| GPS 无数据 | 波特率不对 | `stty -F /dev/ttyUSB0 9600` 确认 |
| 航点太少 | 采样间隔太大 | 改 `--interval 0.5` |
| 跑出路沿 | 航点录歪了 | 重录，推车时保持路径在场地中央 |

### 首次启动检查清单

- [ ] 树莓派已通电，SSH 可达
- [ ] `i2cdetect -y 1` 看到 0x40（PCA9685）
- [ ] `cat /dev/ttyUSB0` 看到 NMEA 语句（GPS）
- [ ] 电调上电后有提示音
- [ ] 舵机手动旋转顺畅（无卡死）
- [ ] 车轮离地（砖块垫起）
- [ ] 电池满电
- [ ] `pipeline_car.json` 的 `dry_run=0`
- [ ] `target_speed=0.5`（首次安全速度）
- [ ] 已录好航点文件 `/tmp/waypoints.json`

### 安全注意事项

- **首次跑务必车轮离地**，确认舵机和油门方向正确
- 电调上电后**不要站在车头方向**，油门误触可能冲撞
- 失控时**关电调开关比拔树莓派电源更快**
- 锂电池不要过放，电压低于 3.5V/节时停止
- 树莓派不要用舵机/电调同电源，大电流脉冲会重启

---

## 八、参考

- [真车硬件部署指南](HARDWARE_DEPLOYMENT.md) — 架构对比、节点清单
- [SocketCAN 执行器教程](tutorials/15_socketcan_actuator.md) — CAN 总线升级路线
- [真车部署待办清单](REAL_VEHICLE_ROADMAP.md) — 剩余缺位项
- [SIMULATION_GUIDE.md](SIMULATION_GUIDE.md) — 仿真模式对照
- `tools/waypoint_record.py` — 航点录制工具（源码）
- `tools/waypoint_player.py` — 航点查看/重采样工具
- `config/pipeline_car.json` — 真车配置模板