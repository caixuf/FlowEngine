# 真车部署待办清单

> FlowEngine 当前在仿真闭环验证完成，真车部署尚有占位/缺位。
> 本文按"核心问题一句话"原则列出待办，不展开实现细节。
> 落地路径建议：从 RC 小车最小闭环（actuator_pwm + waypoint_follower，已最完整）起步。

---

## 感知输入层（大半占位）

- [ ] **stereo_camera** — 默认 dry_run，需填 OAK-D/depthai 或 V4L2 真实读取
- [ ] **lidar_driver** — 现仅 ASCII 占位，需填真实二进制协议（RPLIDAR SDK / 3D UDP）
- [ ] **imu_driver** — 主流 IMU 走 I2C，现默认串口 ASCII，需重写解析钩子

## 定位（不可用）

- [ ] **slam** — 现为会发散的 dead-reckoning 占位，需接真 SLAM 后端（FAST-LIO2）
- [ ] **fusion** — 模板级 EKF 需升级为真多源融合，并做精度验证

## 执行 + 安全（缺位）

- [ ] **actuator_node (CAN)** — 补失联看门狗（断流自动刹停），换真实 ESC 协议
- [ ] **分级降级状态机** — 传感器丢失 / 定位发散 / 通信中断的处置
- [ ] **e-stop 软硬件双回路** — 急停冗余

## 可观测性 / 真值（测试预言机的前提）

- [ ] **执行器物理回读**（轮速 / 电流）→ 指令 vs 实际闭环可观测
- [ ] **外部真值源**（RTK / 标记场地）→ 定位精度可量化
- [ ] **系统健康 topic**（心跳 / 新鲜度 / 看门狗状态）+ 告警面板

## 标定 / 参数

- [ ] **control 参数** — 仿真调的，真车需重新标定
- [ ] **传感器外参标定** — 安装位姿 / 基线 / 坐标系对齐

## 测试

- [x] **6 个 driver + slam/fusion 纯逻辑单测**（dry-run，便宜，先做）
      → 已交付 `tests/test_adas_nodes_logic.c`（24 用例：imu parse/synthetic、
      slam 圆轨迹+heading 归一化、actuator_pwm ControlCmd→PWM 映射），
      CMake target `test_adas_nodes_logic`，全部通过。采用「副本 + 行号标注」策略。
- [ ] **HIL 台架冒烟**（车轮离地：跟踪误差 / 看门狗 / e-stop 时序）
- [ ] **真车 bag 回放 + 物理信号打分回归**

## 时间同步（跨设备，当前完全缺位）

> 现状：`clock_service.c` 是单进程逻辑时钟 + 仿真时间注入（`clock_now_us()` 可被
> `g_sim_time` 顶替，给确定性回放用）。对 bag 回放够用，但**不是跨设备时间同步**：
> 无 PTP/gPTP/NTP、无 offset/drift 估计。主机、每个传感器、MCU 各跑各的钟。

- [ ] **时间戳语义改为「采集时刻」** — 现在所有传感器时间戳是「主机到达时刻」
      （imu_driver_node.c:240、lidar:291 都是处理到那一行时打 `clock_now_us()`）。
      GPS 1Hz / LiDAR 整帧扫描 / IMU 100~1000Hz 各自管道延迟不同 → 融合时在对齐
      实际不同时刻采集的数据，定位/融合误差就从这来。
- [ ] **接 GNSS 纪律主机钟** — `gps_driver_node.c` 只 `#include <time.h>`，NMEA 里
      自带的 UTC/GNSS 时间（RMC/GGA 的 utc 字段）被 `nmea_parser` 解析后丢弃，没接进
      `timestamp_us`。一个免费的全局同步时钟，没用。最低成本同步：用 GNSS 1PPS +
      UTC 纪律主机 CLOCK_MONOTONIC（或 chrony/ntpsec 指向 GPS）。
- [ ] **PTP/gPTP**（若多传感器需 <ms 级对齐）— IEEE 1588 软件时间戳够 IMU/LiDAR
      对齐；若要硬件时间戳需网卡支持。
- [ ] **修 IMU 时间戳 32 位回绕** — `ImuData.timestamp_us` / `GpsData.timestamp_us`
      在 `msg/adas_msgs.msg` 里是 `uint32`（微秒级 ~71 分钟回绕一次），而
      `ControlCmd.timestamp_us` 是 `uint64`（不回绕）。schema 不一致。需统一改
      `uint64`（涉及 schema 迁移 + 消费端 + schema_hash 变更，非一行可改）。

## MCU 控制端（整个概念不存在）

> 现状：架构是主机 Linux → SocketCAN/I2C → 直接怼电调/舵机。控制环、看门狗、
> 失联刹停全跑在非实时 Linux 上。无 `firmware/`、无 `mcu/`、无 `.ino/.stm32`。

- [ ] **引入安全 MCU 层** — 确定性内环（高速控制环，如 1kHz 油门/转向伺服）+
      硬看门狗（主机失联超时强制中位/刹停）+ E-stop 硬回路 + 真实执行器协议。
      真车问题：①Pi 一负载控制环可能卡几十 ms；②保命 failsafe/E-stop 应在
      确定性 MCU 而非 Linux；③真实执行器协议 + 快速内环通常也在 MCU。
- [ ] **定义主机↔MCU 协议** — 含心跳、指令、状态回读，并做时间同步（MCU 时钟
      归律到主机/GNSS）。当前 `actuator_pwm_node.c` 的看门狗是软件看门狗（超时
      强制 ESC 中位），不是硬件级 failsafe。

## 构建前置（已核实 ✅）

- [x] **esmini submodule 状态** — 子模块为空，仅影响 `flowsim_node`（仿真器）；
      核心框架 + 真车链路节点（感知/定位/执行）独立编译，不依赖 esmini。
      CMakeLists.txt 有 fallback：找不到 esminiRMLib 时 flowsim_node 警告但继续链接。

## 附：仿真可视化打磨（非阻塞，影响长途观感）

> 不影响「可用」，影响「高架长途看起来顺不顺」。esmini 子模块为空时这些
> 属于 flowsim_node 视图层问题，不阻塞真车链路。

- [ ] **ViaductView.followEgo 的 -100 常数** — 疑似没跟 `VIS_LENGTH=500` 同步，
      高架段相机/视野跟随偏移可能不一致。
- [ ] **wrap 周期(500) 与道路 build 长度(1000) 不一致** — 环境物可能每 500m 跳一下
      （接缝处可见跳变）。需对齐 wrap 周期与 build 长度，或加平滑过渡。

---

## 核心问题（一句话）

**两头（感知输入 + 定位 + 执行安全）是占位或缺失，中间（规划/控制）是
真代码但只在仿真里验证过。** 建议从 RC 小车最小闭环（actuator_pwm +
waypoint_follower，已最完整）起步，逐步外扩感知与定位。
