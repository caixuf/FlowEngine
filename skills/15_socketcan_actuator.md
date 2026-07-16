# Skill 15 - SocketCAN 执行器节点（真车硬件接入）

## 核心思想

算法栈算出 `throttle / brake / steering` 之后，要把这三个浮点数变成真实硬件动作——电机加速、刹车夹紧、舵机转向。这一步通过 **CAN 总线** 完成，Linux 内核提供的 **SocketCAN** 把 CAN 控制器抽象成网络接口（`can0`），应用程序用标准 socket API（`socket / bind / write`）收发 CAN 帧，和发 UDP 包几乎一样。

本项目 `actuator_node` 节点插件就是这一环：订阅 `control/raw_cmd`（`ControlRaw` 二进制消息），编码成 SocketCAN 帧发到 `can0`，驱动真实 ESC（电子调速器）+ 转向舵机。

```
control_node (PID/Stanley)
  → control/raw_cmd (ControlRaw 二进制)
  → [actuator_node]
  → SocketCAN write()
  → can0 总线
  → ESC + 舵机
```

这是从仿真车走向真车**最关键的一步**——前面整条算法链都在软件里跑，只有 actuator_node 的 `write()` 系统调用真正影响了物理世界。

## 为什么是 SocketCAN，不是别的

| 方案 | 物理层 | Linux 支持 | 延迟 | 适合场景 |
|------|--------|-----------|------|---------|
| **SocketCAN** | CAN 总线（双绞线） | 内核原生 | 1-5ms | 智能小车 / RC / 教育平台 |
| EtherCAT | 工业以太网 | 需 IgH/SOEM 第三方栈 | 10-100μs | 工业机器人 / 伺服阵列 |
| 串口 PWM | UART | 内核原生 | 5-20ms | 玩具级 RC（无 CAN 接口时） |
| ROS serial | UART/USB | 需 ROS | 10-50ms | ROS 生态项目 |

本项目选 SocketCAN 的理由：
1. **树莓派/Jetson 原生支持**——MCP2515 SPI-CAN 扩展板 ¥30 就能加，或直接用带 CAN 控制器的 SoC
2. **CAN 总线本身就是车载标准**——真实车辆 ECU 之间就是 CAN 通信，直接对接不用协议转换
3. **20Hz 控制周期下 1-5ms 延迟完全够用**——不需要 EtherCAT 的微秒级实时性
4. **Linux 内核原生 API**——不需要装额外协议栈，`#include <linux/can.h>` 就能用

## CAN 总线基础

### 帧结构

CAN 标准帧 111 bit，有效载荷 8 字节：

```
┌──────┬────────┬──────┬──────────┬──────────────────┬─────┐
│ SOF  │ CAN ID │ RTR  │ DLC (4b) │ Data (0-8 bytes) │ CRC │
└──────┴────────┴──────┴──────────┴──────────────────┴─────┘
  1b     11b      1b      4b             0-64b          15b
```

- **CAN ID**（11 bit，标准帧）：标识这一帧的"地址"，类似 TCP 端口。`0x100` 可以约定为油门帧，`0x101` 为转向帧
- **DLC**（4 bit）：数据长度，0-8
- **Data**（0-8 字节）：实际 payload

> CAN FD（Flexible Data-rate）扩展到 64 字节，5-8Mbps。本项目用经典 CAN 8 字节帧就够（throttle+brake+gear+e_stop 刚好 8 字节）。

### Arbitration（仲裁）

CAN 是多主总线，多个节点同时发时靠 CAN ID 优先级仲裁：**ID 数值越小优先级越高**（`0x100` 比 `0x200` 先发）。仲裁失败的节点自动重试，无需中心调度。

所以约定 ID 时，安全相关帧（如 e_stop）应该用小 ID，遥测帧用大 ID。本项目默认：
- `0x100` 油门/刹车帧
- `0x101` 转向帧
- `0x102` 心跳/状态帧

## SocketCAN 原理

Linux 内核把 CAN 控制器（硬件）映射成一个**网络接口**，名字通常是 `can0`。你可以像配置网卡一样配置它：

```bash
# 把 can0 设成 500kbps（CAN 经典波特率）
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up

# 看接口状态（和看 eth0 一样）
ip -details link show can0
```

接口起来之后，用户态程序用**标准 socket API** 收发 CAN 帧：

```c
// 创建 CAN socket —— 和创建 UDP socket 几乎一模一样
int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);   // CAN_RAW = 原始 CAN 帧

// 找到 can0 的接口索引（类似 bind 到某个网卡）
struct ifreq ifr;
strcpy(ifr.ifr_name, "can0");
ioctl(s, SIOCGIFINDEX, &ifr);

// bind 到 can0
struct sockaddr_can addr;
addr.can_family  = AF_CAN;
addr.can_ifindex = ifr.ifr_ifindex;
bind(s, (struct sockaddr*)&addr, sizeof(addr));

// 发一帧 —— 和 write() 写文件一样
struct can_frame frame;
frame.can_id  = 0x100;          // CAN ID
frame.can_dlc = 8;              // 数据长度
memcpy(frame.data, payload, 8); // 8 字节数据
write(s, &frame, sizeof(frame)); // 就这么简单
```

对比 UDP socket，只有三点不同：
1. `PF_CAN` 代替 `PF_INET`，`CAN_RAW` 代替 `SOCK_DGRAM`
2. bind 到网络接口而不是 IP:端口
3. 数据单元是 `struct can_frame`（8 字节）而不是任意长 buffer

## 树莓派硬件准备

### 方案 A：MCP2515 SPI-CAN 扩展板（最便宜，¥30）

MCP2515 是 Microchip 的独立 CAN 控制器，通过 SPI 接树莓派 GPIO。

**接线**（MCP2515 模块 → 树莓派 GPIO）：

| MCP2515 | 树莓派 GPIO | 说明 |
|---------|------------|------|
| VCC     | 3.3V (Pin 1) | 供电（必须 3.3V，5V 会烧） |
| GND     | GND (Pin 6) | 地 |
| CS      | GPIO8 (Pin 24, CE0) | SPI 片选 |
| SO      | GPIO9 (Pin 21, MISO) | SPI 主入从出 |
| SI      | GPIO10 (Pin 19, MOSI) | SPI 主出从入 |
| SCK     | GPIO11 (Pin 23, SCLK) | SPI 时钟 |
| INT     | GPIO25 (Pin 22) | 中断（可选，轮询也行） |
| CAN-H   | CAN 总线高线 | 接 ESC 的 CAN-H |
| CAN-L   | CAN 总线低线 | 接 ESC 的 CAN-L |

**配置**（`/boot/firmware/config.txt`，树莓派 4B）：

```
dtparam=spi=on
dtoverlay=mcp2515-can0,oscillator=16000000,interrupt=25
```

`oscillator` 必须和模块晶振一致（常见 8MHz 或 16MHz，模块上有标注），错了波特率会算错。

**启动接口**：

```bash
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up
```

### 方案 B：USB-CAN 适配器（即插即用，¥100-300）

插上就识别成 `can0`（或 `can1`），无需接线、无需 dtoverlay。常见的有 LAWICEL CANUSB、PCAN-USB、各种国产 USB-CAN。大部分走 `slcan` 或原生 `c_CAN` 驱动，dmesg 能看到识别日志。

```bash
# 某些 USB-CAN 需要先转成 SocketCAN 接口
sudo slcan_attach -f -o -s6 /dev/ttyUSB0
sudo ip link set can0 up type can bitrate 500000
```

### 验证接口

```bash
# 看接口是否起来
ip -details link show can0

# 应该看到 state UP
# 如果看到 state ERROR-ACTIVE，说明总线正常
# 如果看到 state BUS-OFF，说明总线没接终端电阻或线接反了
```

> CAN 总线两端必须各接一个 **120Ω 终端电阻**，否则信号反射会导致 BUS-OFF。MCP2515 模块通常有跳线可启用板载终端电阻。

## actuator_node 代码走读

源码：[modules/adas_nodes/actuator_node.c](../modules/adas_nodes/actuator_node.c)

### 1. 打开 CAN socket

[actuator_node.c:117-150](../modules/adas_nodes/actuator_node.c#L117) 的 `can_open()` 函数封装了上面那段 socket 创建流程：

```c
static int can_open(const char* ifname) {
    int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    // ... ioctl 找 ifindex ...
    // ... bind 到 can0 ...
    return s;  // 成功返回 fd，失败返回 -1
}
```

关键点：**失败不致命**。[actuator_node.c:371-380](../modules/adas_nodes/actuator_node.c#L371) 在 init 里检查 `can_open` 返回值，失败就自动切到 `dry_run` 模式：

```c
g.can_sock = can_open(g.can_interface);
if (g.can_sock < 0) {
    LOG_WARN("actuator", "CAN open failed on '%s', falling back to dry-run ...", ...);
    g.dry_run = 1;  // 切到只日志模式
}
```

这样在沙箱、CI、开发机（没有 `can0` 接口）上 actuator_node 也能正常启动，不会因为没硬件就阻塞整个流水线。这是嵌入式节点设计的重要原则：**硬件缺失要降级而不是崩溃**。

### 2. CAN 帧编码

[actuator_node.c:182-219](../modules/adas_nodes/actuator_node.c#L182) 把 `ControlRaw`（浮点数）编码成 CAN 帧（字节）：

**油门/刹车帧**（CAN ID `0x100`，8 字节）：

```
[0-1] throttle 量化值 (uint16 LE, 0..throttle_scale)
[2-3] brake    量化值 (uint16 LE, 0..throttle_scale)
[4]   gear     (1=DRIVE, 0=N, -1=REV)
[5]   e_stop   (0/1)
[6-7] reserved
```

**转向帧**（CAN ID `0x101`，4 字节）：

```
[0-1] steering 量化值 (int16 LE, -steering_scale..+steering_scale)
[2-3] seq (uint16 LE, 用于检测丢帧)
```

`throttle_scale` / `steering_scale` 把 `[0,1]` 或 `[-1,1]` 的浮点数量化成整数。默认 1000，即 0.001 精度。具体编码方案**取决于你的 ESC 协议**——这是社区用户最需要按硬件改的地方。如果你用的是 VESC、ODrive 或某种商用 ESC，查它的 CAN 协议手册改 `encode_*_frame` 两个函数即可，不用动框架。

### 3. 订阅回调

[actuator_node.c:222-256](../modules/adas_nodes/actuator_node.c#L222) 的 `on_control_raw_cmd` 是核心回调：

```c
static void on_control_raw_cmd(const Message* msg, void* user_data) {
    ControlRaw raw;
    ControlRaw_deserialize(&raw, msg->data, msg->data_size);  // 反序列化

    // 紧急停车检测
    int e_stop = (raw.brake > 0.95f && raw.throttle < 0.05f) ? 1 : 0;

    // 编码 + 发送
    uint8_t tbuf[8];
    encode_throttle_frame(raw.throttle, raw.brake, 1, e_stop, tbuf);
    can_send(g.can_throttle_id, tbuf, 8);

    uint8_t sbuf[4];
    encode_steering_frame(raw.steering, raw.seq, sbuf);
    can_send(g.can_steering_id, sbuf, 4);
}
```

每收到一帧 `control/raw_cmd`（control_node 20Hz 发布），就发两帧 CAN（油门 + 转向），延迟约 1-2ms，完全跟得上 20Hz 闭环。

### 4. 心跳线程

[actuator_node.c:257-271](../modules/adas_nodes/actuator_node.c#L257) 有个独立线程定期发 status 帧：

```c
static void* heartbeat_thread(void* arg) {
    long period_us = 1000000L / g.heartbeat_hz;  // 默认 10Hz
    while (g.hb_running) {
        usleep(period_us);
        uint8_t buf[8];
        memcpy(buf,     &g.frames_sent,    4);
        memcpy(buf + 4, &g.cmds_received,  4);
        can_send(g.can_status_id, buf, 8);
    }
}
```

为什么要心跳？很多 ESC 有**看门狗**机制：如果 N 毫秒没收到指令就自动断电保护。心跳帧让 ESC 知道控制器还活着，即使 control_node 暂时没发新指令也不会误触发断电。

### 5. 配置参数

[actuator_node.c:353-364](../modules/adas_nodes/actuator_node.c#L353) 从 `pipeline.json` 的 `params` 读配置：

```json
{
  "name": "actuator",
  "library": "libactuator_node.so",
  "params": {
    "can_interface": "can0",
    "can_throttle_id": "0x100",
    "can_steering_id": "0x101",
    "throttle_scale": 1000.0,
    "steering_scale": 1000.0,
    "dry_run": false,
    "heartbeat_hz": 10
  }
}
```

CAN ID 支持 `0x` 十六进制写法（[actuator_node.c:275-289](../modules/adas_nodes/actuator_node.c#L275) 的 `parse_hex_int`），符合 CAN 协议手册的惯例。

## 动手实验

### 实验 1：dry-run 模式看帧格式（无需硬件）

在没 CAN 硬件的开发机上也能跑 actuator_node，它会自动降级为 dry-run，把要发的 CAN 帧打印到日志：

```bash
# 在 pipeline.json 里加 actuator 节点（dry_run 不用设，无 can0 会自动降级）
./build/bin/flow_launcher config/pipeline.json

# 日志里会看到：
# [actuator] [dry-run] CAN 0x100 [8] e8 03 00 00 01 00 00 00
# [actuator] [dry-run] CAN 0x101 [4] d0 07 2c 00
```

观察 `e8 03` 就是 throttle=1000 的 uint16 LE 编码，`d0 07` 是 steering=2000 的 int16 LE。拿这个对照你 ESC 的协议手册，确认编码方案对不对。

### 实验 2：用 cansend 命令行调试

Linux 自带 `can-utils` 工具包，可以脱离 FlowEngine 直接发 CAN 帧测硬件：

```bash
sudo apt install can-utils

# 手动发一帧油门（CAN ID 0x100，8 字节数据）
cansend can0 100#E803000001000000

# 监听总线上所有帧
candump can0

# 监听并按 ID 过滤
candump can0,100:7FF   # 只看 ID 0x100（掩码 0x7FF）
```

`candump` 是调试 CAN 最常用的工具——actuator_node 发的每一帧都能在这里看到，对照 ESC 手册确认收发一致。

### 实验 3：接真 ESC 跑起来

1. **ESC 上电，CAN-H/CAN-L 接到 can0 总线**（注意终端电阻 120Ω）
2. **查 ESC 协议手册**，确认它的 CAN ID 和数据格式
3. **改 actuator_node 的 `encode_*_frame`** 适配你的 ESC 协议
4. **先用 `cansend` 手动发一帧**，确认 ESC 响应（电机转/舵机转）
5. **再让 actuator_node 接管**，观察 control_node 的 PID 输出是否能驱动真车

```bash
# 树莓派上启动完整流水线
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up
./build/bin/flow_launcher config/pipeline.json
```

## 进阶话题

### CAN FD（Flexible Data-rate）

CAN FD 把单帧 payload 从 8 字节扩到 64 字节，波特率从 1Mbps 提到 5-8Mbps。SocketCAN 支持，只需：

```c
// 启用 FD 模式
int enable_fd = 1;
setsockopt(s, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_fd, sizeof(enable_fd));

// 用 canfd_frame 代替 can_frame（最大 64 字节）
struct canfd_frame fd_frame;
fd_frame.can_id = 0x100;
fd_frame.len = 64;
memcpy(fd_frame.data, big_payload, 64);
write(s, &fd_frame, sizeof(fd_frame));
```

本项目用经典 CAN 8 字节帧就够（油门+刹车+gear+e_stop 刚好 8 字节），不需要 FD。FD 适合需要传大 payload 的场景（如 OBD-II 诊断、地图下发）。

### J1939 协议

J1939 是基于 CAN 的高层协议，主要用于商用车（卡车、工程机械）。它把 8 字节 CAN 帧拼接成多帧消息，定义了 PGN（参数组编号）作为应用层寻址。如果你的目标车辆是商用卡车，ESC 走 J1939 而不是裸 CAN，需要加 J1939 解析层。Linux 内核有 `can-j1939` 模块支持。

### 为什么这个项目不需要 EtherCAT

EtherCAT 是工业实时以太网，延迟微秒级，用于工业机器人伺服阵列。它需要：
- 专用以太网控制器做主站（树莓派 USB 网卡做不了硬实时主站）
- IgH 或 SOEM 第三方协议栈
- PREEMPT_RT 补丁内核
- EtherCAT 从站伺服驱动器（¥1000+）

对于智能小车（ESC + 舵机，20Hz 闭环），SocketCAN 的 1-5ms 延迟完全够用，EtherCAT 是杀鸡用牛刀。只有在做工业 AGV（仓储搬运车、多轴伺服协同）时才需要换 EtherCAT。

## 调试 checklist

真车跑不起来时按这个顺序排查：

1. **`ip link show can0` 能看到接口吗？** → 看不到：dtoverlay 没配对 / USB-CAN 没识别
2. **`candump can0` 能收到 ESC 的心跳吗？** → 收不到：接线错 / 终端电阻没接 / 波特率不对
3. **`cansend` 手动发帧 ESC 响应吗？** → 不响应：CAN ID 错 / 编码格式错 / ESC 没上电
4. **actuator_node 日志有 "CAN write failed" 吗？** → 有：总线 BUS-OFF，检查物理层
5. **FlowBoard 的 control/raw_cmd 有数据吗？** → 没有：control_node 没启动，actuator 收不到指令
6. **dry-run 模式能跑通吗？** → 能：算法链正常，问题在硬件；不能：先修算法链

## 小结

- **SocketCAN 是 Linux 标准 CAN 抽象**，把 CAN 控制器映射成 `can0` 网络接口，用 `socket/write` 收发帧
- **actuator_node 是真车接入的最后一环**，订阅 `control/raw_cmd` 编码成 CAN 帧发到总线
- **硬件缺失自动降级 dry-run**，开发机/CI 也能跑，不阻塞流水线
- **CAN 帧编码按硬件协议改**，`encode_*_frame` 两个函数是适配点
- **树莓派 + MCP2515 + ¥30 扩展板就能跑**，不需要 EtherCAT 那套工业级方案

读完这篇，你应该能：理解 CAN 总线基础、配通树莓派 SocketCAN 接口、看懂 actuator_node 代码、按自己 ESC 协议改编码、把仿真算法链接到真车硬件上跑起来。
