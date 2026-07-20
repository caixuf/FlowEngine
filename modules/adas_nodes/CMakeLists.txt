cmake_minimum_required(VERSION 3.16)
project(flowengine_nodes C CXX)

# ── 节点插件独立 cmake 子项目 ───────────────────────────────────
#
# 设计原则: 节点插件不进入主项目的 FetchContent/configure 阶段，
# 避免污染主项目编译环境（c89-gcc vs gnu11 等冲突）。
# 只 link 已构建好的 libflowengine_core.a，不重新触发框架编译。
#
# 构建方式:
#   cmake -B build/nodes -S src/nodes \
#         -DFLOWENGINE_BUILD=<项目根目录>/build
#   cmake --build build/nodes
#
# 或直接用 build.sh: ./build/buildnodes.sh

# ── 路径推导 ─────────────────────────────────────────────────
# modules/adas_nodes/ 从项目根向上三级: modules/adas_nodes -> modules -> root
get_filename_component(FLOW_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)

# 允许外部传入 build 目录（主项目 build）
if(NOT DEFINED FLOWENGINE_BUILD)
    set(FLOWENGINE_BUILD "${FLOW_ROOT}/build")
endif()

message(STATUS "FlowEngine root:  ${FLOW_ROOT}")
message(STATUS "FlowEngine build: ${FLOWENGINE_BUILD}")

# ── 头文件目录 ────────────────────────────────────────────────
include_directories(
    ${FLOW_ROOT}/include
    ${FLOW_ROOT}/src/algorithms            # ekf_fusion.h, dbscan_cluster.h
    ${FLOWENGINE_BUILD}/_deps/flowcoro-src/include
    ${FLOWENGINE_BUILD}/gen                # 生成的 adas_msgs_gen.h
)

# ── 链接主框架库 ──────────────────────────────────────────────
find_library(FLOWENGINE_CORE
    NAMES flowengine_core
    HINTS ${FLOWENGINE_BUILD}/lib
    REQUIRED
)
message(STATUS "libflowengine_core: ${FLOWENGINE_CORE}")

# Threads — 节点直接指定 -lpthread，不走 FindThreads（避免 c89-gcc 检测失败）
set(NODE_LINK_LIBS ${FLOWENGINE_CORE} m pthread)

# ── cJSON ─────────────────────────────────────────────────────
# libflowengine_core.a 是静态库，包含 scenario_loader.o，其中引用了 cJSON 符号。
# 节点插件编译为 .so，dlopen 加载时这些符号必须可解析，故也需链接 cJSON。
find_package(PkgConfig QUIET)
if(PkgConfig_FOUND)
    pkg_check_modules(CJSON QUIET libcjson)
endif()
if(NOT CJSON_FOUND)
    # 主项目通过 FetchContent 构建的 cJSON 静态库。FetchContent 目标的产物
    # libcjson.a 落在 ${FLOWENGINE_BUILD}/lib（主项目 ARCHIVE_OUTPUT_DIRECTORY），
    # 而非 _deps/cjson-build（后者只含 .pc/cmake 配置文件）。故 HINTS 必须同时
    # 包含 build/lib，否则无系统 libcjson-dev 时 find_library 失败 → CJSON_FOUND=FALSE
    # → include_directories 被跳过 → <cjson/cJSON.h> 找不到。
    find_library(CJSON_LIB
        NAMES cjson libcjson
        HINTS
            ${FLOWENGINE_BUILD}/lib
            ${FLOWENGINE_BUILD}/_deps/cjson-build
            ${FLOWENGINE_BUILD}/_deps/cjson-build/Debug
            ${FLOWENGINE_BUILD}/_deps/cjson-build/Release
    )
    if(CJSON_LIB)
        set(CJSON_LIBRARIES ${CJSON_LIB})
        set(CJSON_FOUND TRUE)
    endif()
endif()
# 节点插件现在直接使用 cJSON API（JSON 序列化/反序列化迁移），需要头文件路径。
if(CJSON_FOUND)
    list(APPEND NODE_LINK_LIBS ${CJSON_LIBRARIES})
    message(STATUS "cJSON found for node plugins: ${CJSON_LIBRARIES}")
    # 头文件路径（兼容系统安装的 pkg-config 和 FetchContent 构建两种来源）
    if(CJSON_INCLUDE_DIRS)
        include_directories(${CJSON_INCLUDE_DIRS})
    elseif(EXISTS "${FLOWENGINE_BUILD}/_cjson_include/cjson/cJSON.h")
        include_directories("${FLOWENGINE_BUILD}/_cjson_include")
    endif()
else()
    message(WARNING "cJSON not found — node plugins may fail at dlopen due to undefined cJSON symbols")
endif()

# ── 编译选项 ──────────────────────────────────────────────────
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -fPIC -D_GNU_SOURCE")
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -fPIC -D_GNU_SOURCE")
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# 输出目录: 与主项目的 build/lib 保持一致，方便 flow_launcher 查找
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${FLOWENGINE_BUILD}/lib)

# ── 节点插件 ──────────────────────────────────────────────────

# fusion_node: EKF 5D 传感器融合 (FlowCoro 协程版)
# 采用 FlowCoroTask（线程池 resume）：EKF + 序列化 + publish 单次 ~50-100μs，
# 同步 resume 会阻塞消息总线分发线程导致 drops，故改用线程池 resume。
# flowcoro 核心库为 header-only（INTERFACE），子项目已 include 其头文件目录，
# 故只需 FLOWCORO_INTEGRATION 定义 + -fcoroutines，无需额外链接 flowcoro 库。
add_library(fusion_node SHARED fusion_node.cpp)
target_compile_definitions(fusion_node PRIVATE FLOWCORO_INTEGRATION)
target_compile_options(fusion_node PRIVATE $<$<CXX_COMPILER_ID:GNU>:-fcoroutines>)
target_link_libraries(fusion_node ${NODE_LINK_LIBS} stdc++ atomic)

# perception_node: DBSCAN 感知 + 传感器仿真 (FlowCoro 协程版)
# 采用 FlowCoroTask（线程池 resume）：DBSCAN 点云聚类为重计算，同步 resume 会阻塞
# 消息总线分发线程导致 drops，故改用线程池 resume。
add_library(perception_node SHARED perception_node.cpp)
target_compile_definitions(perception_node PRIVATE FLOWCORO_INTEGRATION)
target_compile_options(perception_node PRIVATE $<$<CXX_COMPILER_ID:GNU>:-fcoroutines>)
target_link_libraries(perception_node ${NODE_LINK_LIBS} stdc++ atomic)

# sensor_model_node: 噪声/FOV/遮挡传感器模型
add_library(sensor_model SHARED sensor_model_node.c)
target_link_libraries(sensor_model ${NODE_LINK_LIBS})

# ── 新增节点插件 ──────────────────────────────────────────────

# flowsim_node: FlowSim v2 仿真世界 (C++20 flowcoro + esmini RoadManager)
# 替换 sim_world_node。集成 Phase 1 组件（road_network/entity/physics/npc_ai/
# collision/scene_events）为 20Hz flowcoro 协程主循环节点。
# 采用 FlowCoroTask（线程池 resume）：物理+NPC AI+碰撞+事件+JSON 序列化单次
# ~50-200μs，同步 resume 会阻塞消息总线分发线程，故改用线程池 resume。
# 链接 esminiRMLib（C API，ABI 稳定）做道路网络 Frenet↔World 转换。
# 优先用预构建库（lib/），其次用子模块构建产物（third_party/esmini/build/）。
set(ESMINI_ROOT "${FLOW_ROOT}/third_party/esmini")
# 头文件搜索路径：子模块 → 本地预构建 include（CI 无子模块时走第二条）
set(ESMINI_RMLIB_INCLUDE
    "${ESMINI_ROOT}/EnvironmentSimulator/Libraries/esminiRMLib"
    "${FLOW_ROOT}/include"
)
find_library(ESMINI_RMLIB
    NAMES esminiRMLib
    HINTS "${FLOW_ROOT}/lib"                                    # 预构建（CI 优先）
          "${ESMINI_ROOT}/build/EnvironmentSimulator/Libraries/esminiRMLib"  # 子模块构建
)
add_library(flowsim_node SHARED
    flowsim_node.cpp
    flowsim/road_network.cpp
    flowsim/road_position.cpp
    flowsim/physics.cpp
    flowsim/npc_ai.cpp
    flowsim/collision.cpp
    flowsim/scene_events.cpp
    flowsim/scene_pub.cpp
    flowsim/route.cpp
)
target_compile_definitions(flowsim_node PRIVATE FLOWCORO_INTEGRATION)
target_compile_options(flowsim_node PRIVATE $<$<CXX_COMPILER_ID:GNU>:-fcoroutines>)
target_include_directories(flowsim_node PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${ESMINI_RMLIB_INCLUDE}
)
if(ESMINI_RMLIB)
    target_link_libraries(flowsim_node ${NODE_LINK_LIBS} stdc++ atomic ${ESMINI_RMLIB})
    message(STATUS "flowsim_node: esminiRMLib found at ${ESMINI_RMLIB}")
else()
    target_link_libraries(flowsim_node ${NODE_LINK_LIBS} stdc++ atomic)
    message(WARNING "flowsim_node: esminiRMLib NOT found — road_network.cpp will fail to link")
endif()

# planning_node: Frenet 轨迹规划 (FlowCoro 协程版)
# 采用 FlowCoroTask（线程池 resume）：Frenet 轨迹规划为重计算，同步 resume 会阻塞
# 消息总线分发线程导致 drops，故改用线程池 resume。
find_library(FRENET_BRIDGE
    NAMES frenet_bridge
    HINTS ${FLOWENGINE_BUILD}/lib ${FLOW_ROOT}/build-algo/lib
)
find_library(FRENET_PLANNER
    NAMES frenet_planner
    HINTS ${FLOWENGINE_BUILD}/lib ${FLOW_ROOT}/build-algo/lib
)
add_library(planning_node SHARED planning_node.cpp)
target_compile_definitions(planning_node PRIVATE FLOWCORO_INTEGRATION)
target_compile_options(planning_node PRIVATE $<$<CXX_COMPILER_ID:GNU>:-fcoroutines>)
if(FRENET_BRIDGE AND FRENET_PLANNER)
    target_link_libraries(planning_node ${NODE_LINK_LIBS}
        ${FRENET_BRIDGE} ${FRENET_PLANNER} stdc++ atomic)
    target_include_directories(planning_node PRIVATE
        ${FLOW_ROOT}/third_party/frenet_planner/src
        ${FLOW_ROOT}/third_party/frenet_planner/src/FrenetOptimalTrajectory
    )
    target_compile_definitions(planning_node PRIVATE HAVE_FRENET)
    message(STATUS "planning_node: frenet_bridge found at ${FRENET_BRIDGE}")
else()
    target_link_libraries(planning_node ${NODE_LINK_LIBS} stdc++ atomic)
    message(STATUS "planning_node: building in fallback mode (no frenet/Eigen)")
endif()

# control_node: PID 纵向/横向控制 (FlowCoro 协程版)
# 采用 CoroutineTask（同步 resume）：control 是延迟敏感的闭环控制，FlowCoroTask
# 线程池 resume 的调度抖动会导致 20Hz 周期不一致，prev_steer 低通滤波时间间隔
# 波动，steer 产生小幅振荡。CoroutineTask 周期精确，且 PID+Stanley 计算量小，
# 同步 resume 阻塞可忽略。与 safety_control 一致。
add_library(control_node SHARED control_node.cpp)
target_link_libraries(control_node ${NODE_LINK_LIBS} stdc++)

# safety_control_node: safety envelope for control commands
# Uses CoroutineTask (synchronous resume on bus delivery thread) instead of
# FlowCoroTask — the safety check is lightweight (<10us) and synchronous resume
# avoids thread-pool starvation that caused control/cmd staleness.
add_library(safety_control_node SHARED safety_control_node.cpp)
target_link_libraries(safety_control_node ${NODE_LINK_LIBS} stdc++)

# monitor_node: 系统监控 + 仪表盘 JSON 导出
add_library(monitor_node SHARED monitor_node.c)
target_link_libraries(monitor_node ${NODE_LINK_LIBS})

# ── 车端学习闭环节点 (data → train → inference → learner → ota) ─

# inference_node: 内置 tiny-MLP 车端推理（影子模式，支持 OTA 热重载，FlowCoro 协程版）
# 采用 FlowCoroTask（线程池 resume）：MLP 推理为重计算，同步 resume 会阻塞
# 消息总线分发线程导致 drops，故改用线程池 resume。
add_library(inference_node SHARED inference_node.cpp)
target_compile_definitions(inference_node PRIVATE FLOWCORO_INTEGRATION)
target_compile_options(inference_node PRIVATE $<$<CXX_COMPILER_ID:GNU>:-fcoroutines>)
target_link_libraries(inference_node ${NODE_LINK_LIBS} stdc++ atomic)

# data_recorder_node: 训练样本采集（模仿学习 JSONL）
add_library(data_recorder_node SHARED data_recorder_node.c)
target_link_libraries(data_recorder_node ${NODE_LINK_LIBS})

# learner_node: 车端增量 SGD 微调（Stage 3）
add_library(learner_node SHARED learner_node.c)
target_link_libraries(learner_node ${NODE_LINK_LIBS})

# model_ota_node: 模型 OTA + 版本管理 + A-B 对比（Stage 4）
add_library(model_ota_node SHARED model_ota_node.c)
target_link_libraries(model_ota_node ${NODE_LINK_LIBS})

# flowmond_node: HTTP 监控仪表盘服务（C++ NodePlugin）
add_library(flowmond_node SHARED flowmond_node.cpp)
target_link_libraries(flowmond_node ${NODE_LINK_LIBS})

# actuator_node: SocketCAN 执行器节点（control/raw_cmd → CAN 总线 → ESC/舵机）
# 真车硬件接入的最后一环。订阅 control_node 输出的 ControlRaw，编码成 SocketCAN 帧
# 发到 can0 接口。无 CAN 设备时（沙箱/CI/开发机）自动降级为 dry-run 日志模式。
# 依赖 linux/can.h（Linux 内核头自带，无需额外库）。
add_library(actuator_node SHARED actuator_node.c)
target_link_libraries(actuator_node ${NODE_LINK_LIBS})

# manual_drive_node: 终端 WASD 键盘侧车工具（游戏模式）
# 用键盘直接驱动 ego，不依赖感知-融合-规划-控制 AI 链，用于人工验证场景几何。
# 订阅 stdin（termios raw 模式）→ 发布 control/cmd。配合 config/pipeline_manual.json 使用。
add_library(manual_drive_node SHARED manual_drive_node.c)
target_link_libraries(manual_drive_node ${NODE_LINK_LIBS})

# lidar_driver_node: 真实 LiDAR 驱动节点（点云 → DBSCAN → ObstacleList）
# 真车感知入口，替代仿真用的 perception_node（后者依赖 vehicle/state 真值）。
# 直接读串口点云 → DBSCAN 聚类 → 发布 perception/obstacles。无硬件时自动降级为
# dry-run（生成模拟点云走完整链路）。read_lidar_frame() 是硬件适配点，社区用户
# 按自己 LiDAR 协议（RPLIDAR/速腾浦/Velodyne）改它即可。
# 依赖 serial_port.h (POSIX termios)、dbscan_cluster.h，均随 libflowengine_core 提供。
add_library(lidar_driver_node SHARED lidar_driver_node.c)
target_link_libraries(lidar_driver_node ${NODE_LINK_LIBS})

# gps_driver_node: 真实 GPS 串口驱动节点（NMEA 0183 → GpsData → sensor/gps）
# 真车定位入口，替代仿真用的 sensor_model_node（后者从 vehicle/state 真值加噪声）。
# 从串口读 NMEA → nmea_parser 解析 → GpsData 序列化发布。无硬件时自动降级为 dry-run。
# 依赖 serial_port.h (POSIX termios)、nmea_parser.h，均随 libflowengine_core 提供。
add_library(gps_driver_node SHARED gps_driver_node.c)
target_link_libraries(gps_driver_node ${NODE_LINK_LIBS})

# imu_driver_node: 真实 IMU 串口驱动节点（6-axis IMU → ImuData → sensor/imu）
# 高频惯性输入入口，为 fusion EKF 提供 predict 增量、为 SLAM 提供运动先验。
# 从串口读 ASCII IMU 行 → parse_imu_line 解析 → ImuData 序列化发布。无硬件时自动降级
# 为 dry-run（生成静止模拟 IMU 走完整链路）。parse_imu_line() 是硬件适配点，社区用户
# 按自己 IMU 协议（MPU6050/ICM-42688/Xsens MTi-300/WIT-Motion）改它即可。
# 依赖 serial_port.h (POSIX termios)，随 libflowengine_core 提供。
add_library(imu_driver_node SHARED imu_driver_node.c)
target_link_libraries(imu_driver_node ${NODE_LINK_LIBS})

# slam_node: SLAM 适配层节点（LiDAR + IMU → Pose2D → sensor/pose）
# 为 GPS 丢失场景（室内/隧道）提供定位。默认 dead reckoning 占位，真实算法
# （FAST-LIO2/LIO-SAM）作为外部库通过 slam_update() 钩子接入。无传感器时
# 自动降级为 dry-run（生成圆形轨迹模拟位姿）。slam_update() 是算法替换点，
# 社区用户按自己 SLAM 库替换该函数实现即可，订阅/发布/线程框架无需改动。
# 编译依赖: adas_msgs_gen.h (LidarFrame/ImuData/Pose2D 序列化)，随构建生成。
#
# ENABLE_FAST_LIO2 开关：cmake -DENABLE_FAST_LIO2=ON ...
# 启用后 slam_node 会定义 HAVE_FAST_LIO2 宏，slam_update() 按 algo=fast_lio2
# 路由到 slam_update_fast_lio2() 占位实现（编译能过，运行时降级到 dead_reckon
# 并 LOG_WARN 提示填充真实 ESKF 调用）。用户需：
#   1. 把 slam_node.c 重命名为 slam_node.cpp（FAST-LIO2 是 C++ 模板库）
#   2. 在 slam_update_fast_lio2() / fast_lio2_init() 里填真实 ESKF 调用
#   3. 在下面 ENABLE_FAST_LIO2 块里取消注释 find_package + target_link_libraries
# 完整步骤见 docs/HARDWARE_DEPLOYMENT.md "FAST-LIO2 集成" 章节。
option(ENABLE_FAST_LIO2 "Enable FAST-LIO2 SLAM backend (requires PCL/Eigen/Sophus + libfast_lio2.a, and slam_node.c → .cpp rename)" OFF)
add_library(slam_node SHARED slam_node.c)
target_link_libraries(slam_node ${NODE_LINK_LIBS})
if(ENABLE_FAST_LIO2)
    target_compile_definitions(slam_node PRIVATE HAVE_FAST_LIO2)
    # 用户预装好依赖后取消注释以下行（同时把 slam_node.c 改名为 slam_node.cpp）：
    # find_package(PCL REQUIRED COMPONENTS common io)
    # find_package(Eigen3 REQUIRED)
    # find_package(Sophus REQUIRED)
    # find_library(FAST_LIO2_LIB NAMES fast_lio2 HINTS /usr/local/lib)
    # target_link_libraries(slam_node ${NODE_LINK_LIBS}
    #     ${PCL_LIBRARIES} Eigen3::Eigen Sophus::Sophus ${FAST_LIO2_LIB})
    message(STATUS "slam_node: HAVE_FAST_LIO2 enabled (占位实现，需填充真实 ESKF 调用)")
endif()

# stereo_camera_node: 双目摄像头驱动节点（OAK-D left+depth → StereoFrame → sensor/stereo）
# 真车视觉感知入口，为 perception 提供 RGB+深度输入。默认 dry-run 占位（生成模拟左图
# + 中心近/边缘远模拟深度走完整链路），真实 OAK-D 通过 depthai 库经 HAVE_DEPTHAI 宏
# 启用（板载 NPU 算深度，主机零 CPU 负担）。read_stereo_frame() 是硬件适配点，社区用户
# 按自己方案（OAK-D NPU / OAK-D 原始图+OpenCV SGBM / DIY 双目 V4L2）改它即可。
# 集成 depthai：find_package(depthai) + 链接 + target_compile_definitions(... HAVE_DEPTHAI)。
# 编译依赖: adas_msgs_gen.h (StereoFrame 序列化)，随构建生成。
add_library(stereo_camera_node SHARED stereo_camera_node.c)
target_link_libraries(stereo_camera_node ${NODE_LINK_LIBS})

# waypoint_follower_node: Pure Pursuit 航点跟随规划器（L2 自主跟随，RC 小车用）
# 替代 planning_node 为 RC 小车提供轻量规划能力。读取预录制 GPS 航点文件，用 Pure
# Pursuit 算法跟踪航点，输出 planning/trajectory。遇前方障碍物自动减速（不做绕行，
# 绕行交给 control/safety 限速）。planning_node 是为高速 NOA 设计的（Frenet+状态机），
# 对 RC 小车（<5m/s，校园/公园）过重且参数不匹配，pipeline_car.json 里二选一。
# 编译依赖: adas_msgs_gen.h (Localization/ObstacleList 反序列化)，随构建生成。
add_library(waypoint_follower_node SHARED waypoint_follower_node.c)
target_link_libraries(waypoint_follower_node ${NODE_LINK_LIBS})

# stereo_vision_node: 双目视觉感知节点（StereoFrame 深度反投影 → DBSCAN → ObstacleList）
# 订阅 stereo_camera_node 发布的 sensor/stereo，把 80×60 深度图反投影成 3D 点云，
# DBSCAN 聚类后输出 ObstacleList 到 perception/obstacles。与 lidar_driver 互补：
# LiDAR 测远距（8-60m），双目测近距稠密（0.5-8m）。RC 小车场景二选一即可。
# 依赖 dbscan_cluster.h，随 libflowengine_core 提供。
add_library(stereo_vision_node SHARED stereo_vision_node.c)
target_link_libraries(stereo_vision_node ${NODE_LINK_LIBS})

# actuator_pwm_node: RC 小车 PWM 执行器节点（control/cmd → PCA9685 I2C / GPIO PWM）
# 替代 actuator_node 的 SocketCAN 方案。RC 小车用舵机+电调（PWM 信号），不接 CAN。
# 通过 PCA9685 I2C-PWM 扩展板（¥10）或树莓派 GPIO 硬件 PWM 输出。backend 参数
# 选 "pca9685" / "gpio" / "dry_run"。带看门狗：3s 无 cmd 强制 ESC 中位（防失控）。
# I2C 打开失败自动降级 dry_run，不阻塞流水线。Linux 平台用 linux/i2c-dev.h。
add_library(actuator_pwm_node SHARED actuator_pwm_node.c)
target_link_libraries(actuator_pwm_node ${NODE_LINK_LIBS})

# perception_fusion_node: 感知融合节点（LiDAR + Stereo 两路 ObstacleList 去重合并 + 跨帧追踪）
# 解决 lidar_driver 和 stereo_vision 都发 perception/obstacles 时的"后写者覆盖"问题。
# 两源各发到独立 topic，本节点订阅两路按 (x,y) 距离去重合并 + 最近邻跨帧关联，
# 输出带持久 ID 和差分速度的 perception/obstacles。
# FlowCoro 协程版（参照 fusion_node.cpp）：select_for 等两路输入 + MessageBuffer 时间对齐，
# 替代旧 pthread_create + usleep 轮询。fuse_obstacles/associate_and_track 业务算法原样保留。
add_library(perception_fusion_node SHARED perception_fusion_node.cpp)
target_compile_definitions(perception_fusion_node PRIVATE FLOWCORO_INTEGRATION)
target_compile_options(perception_fusion_node PRIVATE $<$<CXX_COMPILER_ID:GNU>:-fcoroutines>)
target_link_libraries(perception_fusion_node ${NODE_LINK_LIBS} stdc++ atomic)

# traversability_node: 可通行区域检测节点（StereoFrame 深度图 → 3D 反投影 → 地面分割 → 占用栅格）
# 订阅 stereo_camera_node 发布的 sensor/stereo，把 80×60 深度图反投影成带 Z 高度的 3D 点云
# (与 stereo_vision 的 2D z=0 不同)，用"相机高度+高度阈值"做地面分割，构建占用栅格，
# 提取可通行走廊，输出 JSON 摘要到 perception/traversability。回答"前方哪里是路"的问题，
# 是 L2 → L3 的桥梁（obstacles 只告诉"哪有障碍"，本节点告诉"哪能走"）。
# 算法为沙箱简化版（无 PCL 依赖），RANSAC 地面拟合/Octomap 可作为后续替换点。
# 编译依赖: adas_msgs_gen.h (StereoFrame 反序列化)，随构建生成。
add_library(traversability_node SHARED traversability_node.c)
target_link_libraries(traversability_node ${NODE_LINK_LIBS})

# lane_detection_node: Lane boundary detection (sandbox: from road_geometry)
add_library(lane_detection_node SHARED lane_detection_node.c)
target_link_libraries(lane_detection_node ${NODE_LINK_LIBS})

# traffic_light_recognition_node: Traffic light perception (sandbox: from sim_world)
add_library(traffic_light_recognition_node SHARED traffic_light_recognition_node.c)
target_link_libraries(traffic_light_recognition_node ${NODE_LINK_LIBS})

# object_tracker_node: Multi-object Kalman tracker
add_library(object_tracker_node SHARED object_tracker_node.c)
target_link_libraries(object_tracker_node ${NODE_LINK_LIBS})

# prediction_node: Multi-trajectory motion prediction
add_library(prediction_node SHARED prediction_node.c)
target_link_libraries(prediction_node ${NODE_LINK_LIBS} m)

# scene_assembler_node: Aggregate all perception into unified EnvironmentModel
add_library(scene_assembler_node SHARED scene_assembler_node.c)
target_link_libraries(scene_assembler_node ${NODE_LINK_LIBS})

# ── 安装 ──────────────────────────────────────────────────────
install(TARGETS fusion_node perception_node sensor_model flowsim_node planning_node control_node safety_control_node monitor_node inference_node data_recorder_node learner_node model_ota_node flowmond_node actuator_node lidar_driver_node gps_driver_node imu_driver_node slam_node stereo_camera_node waypoint_follower_node stereo_vision_node actuator_pwm_node perception_fusion_node traversability_node lane_detection_node traffic_light_recognition_node object_tracker_node prediction_node scene_assembler_node
        LIBRARY DESTINATION ${FLOWENGINE_BUILD}/lib)

message(STATUS "Node plugins will be installed to: ${FLOWENGINE_BUILD}/lib")
