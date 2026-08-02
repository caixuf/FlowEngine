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
    OBJ_TYPE_CONSTRUCTION = 4
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
    uint64   timestamp_us   # 采样时间戳（微秒；采集时刻优先，无 GNSS 时用主机钟）
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
    uint64   timestamp_us   # 采样时间戳（微秒，uint64 不回绕）
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
    uint64   timestamp_us   # 采样时间戳（微秒，uint64 不回绕）
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
    float    x              # 纵向距离（米，车体坐标系）
    float    y              # 横向距离（米，车体坐标系）
    float    vx             # 纵向相对速度（m/s）
    float    vy             # 横向相对速度（m/s）
    float    width          # 目标宽度（米）
    float    length         # 目标长度（米）
    int8     lane_id        # 感知计算的车道归属：0=最左, N-1=最右, -1=未分配
    ObstacleType type       # 目标类型（引用枚举）
    float    confidence     # 置信度 [0, 1]
}

struct ObstacleList {
    uint32   frame_id
    uint64   timestamp_us   # CLOCK_MONOTONIC 时间戳（微秒）
    uint32   count          # 有效障碍物数量
    Obstacle obstacles[128] # 固定 128 个障碍物
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
    float    steering       # 转向角（rad）：正值右转，负值左转，范围约 ±0.22 rad，actuator 负责归一化
    float    speed          # 当前速度 (m/s)
    float    target         # 目标速度 (m/s)
    float    error          # 速度误差 (m/s)
    float    cte            # 横向跟踪误差 (m)，正=偏左，负=偏右
    char     mode[24]       # 模式字符串: ACCEL/BRAKE/HOLD/ROAD_GUARD等
    uint8    turn_signal    # 转向灯指令：0=off, 1=left, 2=right（planning/control 决策下发）
    bool     hazard         # 双闪灯指令（紧急停车 / 碰撞时 planning/control 决策下发）
}

struct ControlCmd {
    uint32   seq            # 指令序号
    float    throttle       # 油门 [0.0, 1.0]
    float    brake          # 制动 [0.0, 1.0]
    float    steering       # 转向角（rad）：正值右转，负值左转，范围约 ±0.22 rad，actuator 负责归一化
    Gear     gear           # 档位
    bool     emergency_stop # 紧急制动标志
    uint8    turn_signal    # 转向灯指令：0=off, 1=left, 2=right（planning/control 决策下发，意图先行）
    bool     hazard         # 双闪灯指令（紧急停车 / 碰撞时 planning/control 决策下发）
}

# ── 规划输出 ─────────────────────────────────────────────────

enum BehaviorCommand {
    BEH_CRUISE       = 0     # 巡航（自由行驶）
    BEH_FOLLOW       = 1     # 跟车（保持安全距离）
    BEH_LEFT_CHANGE  = 2     # 向左变道
    BEH_RIGHT_CHANGE = 3     # 向右变道
    BEH_STOP         = 4     # 停车
    BEH_YIELD        = 5     # 让行（减速等待）
    BEH_EMERGENCY    = 6     # 紧急停车
}

struct Behavior {
    uint32   seq            # 指令序号
    uint64   stamp_us       # 发布时刻（微秒，CLOCK_MONOTONIC）
    BehaviorCommand command # 行为指令
    int8     target_lane_idx # 目标车道索引：-1=保持当前, 0..N-1=具体车道
    float    target_speed   # 目标速度（m/s）
    uint32   follow_obs_id  # 跟车目标障碍物 ID（0=无前车）
}

struct TrajectoryPoint {
    uint32   t_rel_us       # 相对轨迹起点的时间偏移（微秒）
    float    x              # X 坐标（米，全局坐标系）
    float    y              # Y 坐标（米，全局坐标系）
    float    s              # Frenet 纵坐标（米）
    float    l              # Frenet 横坐标（米）
    float    heading        # 期望航向角（弧度）
    float    kappa          # 曲率（1/m）
    float    v              # 期望速度（m/s）
    float    a              # 期望加速度（m/s²）
    float    jerk           # 加加速度（m/s³）
}

struct Trajectory {
    uint32   seq            # 轨迹序号
    uint64   stamp_us       # 发布时刻（微秒，CLOCK_MONOTONIC）
    uint32   ref_line_id    # 参考线 ID（变了就禁用拼接）
    uint16   point_count    # 有效轨迹点数
    TrajectoryPoint points[64]  # 最多 64 个轨迹点
    uint8    planner_state  # 规划器状态：0=unknown, 1=sampling, 2=qp, 3=failsafe
    uint8    is_stitched    # 是否拼接轨迹（与上帧连续）
    uint8    valid          # 轨迹是否通过可行性检查
}

struct PredictionHypothesis {
    float    probability    # 该模态概率
    float    sigma_x        # 纵向不确定度增长因子
    float    sigma_y        # 横向不确定度增长因子
    uint8    intent         # 意图：0=LANE_KEEP, 1=LEFT_CHANGE, 2=RIGHT_CHANGE, 3=TURN_LEFT, 4=TURN_RIGHT, 5=YIELD
    uint32   point_count    # 有效轨迹点数
    TrajectoryPoint points[25]  # 最多 25 个预测点（5s @ 0.2s）
}

struct PredictionSet {
    uint32   seq            # 预测序号
    uint64   stamp_us       # 发布时刻（微秒）
    uint32   track_id       # 被预测的障碍物 track_id
    uint8    n_hypotheses   # 有效假设数
    PredictionHypothesis hypotheses[3]  # 最多 3 模态
}

struct ReferenceLinePoint {
    float    s              # Frenet 纵坐标（米）
    float    x              # 全局 X 坐标（米）
    float    y              # 全局 Y 坐标（米）
    float    theta          # 切线航向角（弧度）
    float    kappa          # 曲率（1/m）
    float    dkappa         # 曲率对 s 的导数（1/m²）
    float    left_bound     # 左侧可通行边界（米，相对中心线）
    float    right_bound    # 右侧可通行边界（米，相对中心线）
    float    speed_limit    # 速度限制（m/s）
}

# 帧约定：
#   Obstacle.x/y/vx/vy = 车体系
#   TrajectoryPoint.x/y = 世界系
#   TrajectoryPoint.heading = 世界系航向角（弧度）
#   PredictionHypothesis 输出追踪均与世界系一致
