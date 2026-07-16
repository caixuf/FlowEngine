# =============================================================================
# FlowEngine IDL v1 — ADAS 消息类型定义
# =============================================================================
#
# 每个 struct 由 msg_codegen.py 生成:
#   - C struct 定义
#   - TYPE_ID 常量 (FNV-1a hash)
#   - SCHEMA_VERSION 常量
#   - serialize/deserialize 函数
#   - register_type() 初始化函数
#
# 支持的基本类型:
#   bool, int8, uint8, int16, uint16, int32, uint32,
#   int64, uint64, float, float64 (double)
#
# 域格式: <type> <name> [= <default>]
# 数组格式: <type> <name>[N]
# 枚举格式: enum <Name> { <values> }
# 嵌套引用: 直接用 struct 名作为类型
# =============================================================================

# ── 枚举 ─────────────────────────────────────────────────────

enum ObstacleType {
    OBJ_TYPE_UNKNOWN   = 0
    OBJ_TYPE_VEHICLE   = 1
    OBJ_TYPE_PEDESTRIAN = 2
    OBJ_TYPE_CYCLIST   = 3
}

enum Gear {
    GEAR_REVERSE = -1
    GEAR_NEUTRAL = 0
    GEAR_DRIVE   = 1
}

# ── 传感器原始数据 ───────────────────────────────────────────

struct LidarFrame {
    float    x
    float    y
    float    z
    float    intensity
    uint32   point_count
    uint32   frame_id
}

struct GpsData {
    float64  latitude       # 纬度（度）
    float64  longitude      # 经度（度）
    float    speed_mps      # 速度（m/s）
    float    heading_deg    # 航向角（度，北偏东）
    float    accuracy_m     # 定位精度（米，1-sigma）
}

# IMU 原始数据（6 轴：加速度 + 陀螺仪）
# 用于 fusion EKF 的 predict 增量，或 SLAM（FAST-LIO2/LIO-SAM）的输入。
# 常见模块：MPU6050(6轴)、ICM-42688、BMI088、Xsens MTi-300。
struct ImuData {
    float    accel_x        # 加速度 X 轴（m/s²）
    float    accel_y        # 加速度 Y 轴（m/s²）
    float    accel_z        # 加速度 Z 轴（m/s²，含重力）
    float    gyro_x         # 陀螺仪 X 轴（rad/s）
    float    gyro_y         # 陀螺仪 Y 轴（rad/s）
    float    gyro_z         # 陀螺仪 Z 轴（rad/s，即 yaw_rate）
    uint32   timestamp_us   # 采样时间戳（CLOCK_MONOTONIC，微秒）
    float    temperature    # 温度（℃，部分模块提供，用于温漂补偿）
}

# 2D 位姿（SLAM 或里程计输出）
# 用于 fusion 多源融合，或在 GPS 丢失时作为定位来源。
# 坐标系：局部笛卡尔（米），X=forward, Y=left, heading 弧度。
struct Pose2D {
    float    x              # X 坐标（米）
    float    y              # Y 坐标（米）
    float    heading        # 航向角（弧度）
    float    cov_xx         # 位置 X 协方差
    float    cov_yy         # 位置 Y 协方差
    float    cov_hh         # 航向协方差
    bool     converged      # SLAM 是否收敛
    uint32   source         # 来源标志：0=UNKNOWN, 1=GPS, 2=SLAM, 3=ODOMETRY
}

# 双目视觉帧（OAK-D / ZED / DIY 双目）
# OAK-D 在板载 NPU 上算好深度，主机只收结果，零 CPU 负担。
# 设计权衡：左图 JPEG 压缩（~10-30KB），深度图降采样到 80×60 float 数组
# （80*60*4=19.2KB），单帧总计 <50KB，MessageBus 零拷贝下无压力。
# 如果用 DIY 双目在主机上跑 SGBM，depth_data 仍填算好的深度，计算在节点内完成。
struct StereoFrame {
    uint32   width          # 图像宽度（像素）
    uint32   height         # 图像高度（像素）
    uint32   timestamp_us   # 采样时间戳（CLOCK_MONOTONIC，微秒）
    uint32   left_jpeg_size # 左图 JPEG 数据有效字节数
    uint32   depth_count    # 深度数据有效浮点数个数 (width/4 * height/4 降采样)
    float    baseline_m     # 双目基线长度（米），OAK-D≈0.075, ZED2≈0.12
    float    fov_deg        # 水平视场角（度）
    uint8    left_jpeg[25600]   # 左图 JPEG 压缩数据（25600=25KB 上限，320×240 级别）
    float    depth_data[4800]   # 深度图降采样数据（80×60=4800 个 float，单位米）
}

# ── 感知输出 ─────────────────────────────────────────────────

# 嵌套结构体 — 被 ObstacleList 引用
struct Obstacle {
    uint32   id             # 目标 ID（跨帧唯一）
    float    x              # 纵向距离（米）
    float    y              # 横向距离（米）
    float    vx             # 纵向相对速度（m/s）
    float    vy             # 横向相对速度（m/s）
    float    width          # 目标宽度（米）
    float    length         # 目标长度（米）
    ObstacleType type       # 目标类型（引用枚举）
    float    confidence     # 置信度 [0, 1]
}

struct ObstacleList {
    uint32   frame_id
    uint64   timestamp_us   # CLOCK_MONOTONIC 时间戳（微秒）
    uint32   count          # 有效障碍物数量
    Obstacle obstacles[8]   # 固定 8 个障碍物
}

struct EgoState {
    float64  latitude       # 纬度（度）
    float64  longitude      # 经度（度）
    float    heading_deg    # 航向角（度）
    float    speed_mps      # 车速（m/s）
    float    yaw_rate_dps   # 偏航角速度（度/秒）
    float    acceleration_mss # 纵向加速度（m/s²）
}

# ── 融合输出 ─────────────────────────────────────────────────

struct Localization {
    float    x              # 融合后 X 坐标（米）
    float    y              # 融合后 Y 坐标（米）
    float    v              # 融合速度（m/s）
    float    heading        # 航向角（弧度）
    float    yaw_rate       # 偏航角速度（弧度/秒）
    float    cov_xx         # 协方差对角元素 [0]
    float    cov_yy         # 协方差对角元素 [1]
    float    cov_vv         # 协方差对角元素 [2]
    float    cov_hh         # 协方差对角元素 [3]
    float    cov_yyaw       # 协方差对角元素 [4]
    float    innovation     # EKF 新息
    bool     diverged       # EKF 发散标志
    float    raw_pos_x      # 原始 GPS 位置 X
    float    raw_pos_y      # 原始 GPS 位置 Y
    float    raw_speed      # 原始 GPS 速度
}

struct LatencyReport {
    uint32   avg_us         # 平均延迟（微秒）
    uint32   p50_us         # P50 延迟（微秒）
    uint32   p99_us         # P99 延迟（微秒）
}

# ── 控制输出 ─────────────────────────────────────────────────

struct ControlRaw {
    uint32   seq            # 指令序号
    float    throttle       # 油门 [0.0, 1.0]
    float    brake          # 制动 [0.0, 1.0]
    float    steering       # 转向：-1.0=最大左转，+1.0=最大右转
    float    speed          # 当前速度 (m/s)
    float    target         # 目标速度 (m/s)
    float    error          # 速度误差 (m/s)
    char     mode[24]       # 模式字符串: ACCEL/BRAKE/HOLD/ROAD_GUARD等
}

struct ControlCmd {
    uint32   seq            # 指令序号
    float    throttle       # 油门 [0.0, 1.0]
    float    brake          # 制动 [0.0, 1.0]
    float    steering       # 转向：-1.0=最大左转，+1.0=最大右转
    Gear     gear           # 档位
    bool     emergency_stop # 紧急制动标志
}
