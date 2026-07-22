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

- [ ] **6 个 driver + slam/fusion 纯逻辑单测**（dry-run，便宜，先做）
- [ ] **HIL 台架冒烟**（车轮离地：跟踪误差 / 看门狗 / e-stop 时序）
- [ ] **真车 bag 回放 + 物理信号打分回归**

## 构建前置（已核实 ✅）

- [x] **esmini submodule 状态** — 子模块为空，仅影响 `flowsim_node`（仿真器）；
      核心框架 + 真车链路节点（感知/定位/执行）独立编译，不依赖 esmini。
      CMakeLists.txt 有 fallback：找不到 esminiRMLib 时 flowsim_node 警告但继续链接。

---

## 核心问题（一句话）

**两头（感知输入 + 定位 + 执行安全）是占位或缺失，中间（规划/控制）是
真代码但只在仿真里验证过。** 建议从 RC 小车最小闭环（actuator_pwm +
waypoint_follower，已最完整）起步，逐步外扩感知与定位。
