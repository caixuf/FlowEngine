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
    # 主项目通过 FetchContent 构建的 cJSON 位于 build/_deps/cjson-build/
    find_library(CJSON_LIB
        NAMES cjson libcjson
        HINTS
            ${FLOWENGINE_BUILD}/_deps/cjson-build
            ${FLOWENGINE_BUILD}/_deps/cjson-build/Debug
            ${FLOWENGINE_BUILD}/_deps/cjson-build/Release
    )
    if(CJSON_LIB)
        set(CJSON_LIBRARIES ${CJSON_LIB})
        set(CJSON_FOUND TRUE)
    endif()
endif()
# 节点插件本身不包含 cJSON 头文件，只需链接库以解析 scenario_loader.o 引入的符号
if(CJSON_FOUND)
    list(APPEND NODE_LINK_LIBS ${CJSON_LIBRARIES})
    message(STATUS "cJSON found for node plugins: ${CJSON_LIBRARIES}")
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

# fuse_node: EKF 5D 传感器融合 (FlowCoro 协程版)
# 采用 CoroutineTask 同步 resume，与 safety_control_node 一致；仅需 stdc++，
# 无需 FLOWCORO_INTEGRATION/flowcoro 库（子项目已 include flowcoro 头文件目录）。
add_library(fusion_node SHARED fusion_node.cpp)
target_link_libraries(fusion_node ${NODE_LINK_LIBS} stdc++)

# perception_node: DBSCAN 感知 + 传感器仿真 (FlowCoro 协程版)
add_library(perception_node SHARED perception_node.cpp)
target_link_libraries(perception_node ${NODE_LINK_LIBS} stdc++)

# sensor_model_node: 噪声/FOV/遮挡传感器模型
add_library(sensor_model SHARED sensor_model_node.c)
target_link_libraries(sensor_model ${NODE_LINK_LIBS})

# ── 新增节点插件 ──────────────────────────────────────────────

# sim_world_node: 车辆动力学 + 障碍物仿真
add_library(sim_world SHARED sim_world_node.c)
target_link_libraries(sim_world ${NODE_LINK_LIBS})

# planning_node: Frenet 轨迹规划 (FlowCoro 协程版)
# 采用 CoroutineTask 同步 resume，与 fusion/perception/safety_control 一致；
# 仅需 stdc++（子项目已 include flowcoro 头文件目录）。
find_library(FRENET_BRIDGE
    NAMES frenet_bridge
    HINTS ${FLOWENGINE_BUILD}/lib ${FLOW_ROOT}/build-algo/lib
)
find_library(FRENET_PLANNER
    NAMES frenet_planner
    HINTS ${FLOWENGINE_BUILD}/lib ${FLOW_ROOT}/build-algo/lib
)
add_library(planning_node SHARED planning_node.cpp)
if(FRENET_BRIDGE AND FRENET_PLANNER)
    target_link_libraries(planning_node ${NODE_LINK_LIBS}
        ${FRENET_BRIDGE} ${FRENET_PLANNER} stdc++)
    target_include_directories(planning_node PRIVATE
        ${FLOW_ROOT}/third_party/frenet_planner/src
        ${FLOW_ROOT}/third_party/frenet_planner/src/FrenetOptimalTrajectory
    )
    target_compile_definitions(planning_node PRIVATE HAVE_FRENET)
    message(STATUS "planning_node: frenet_bridge found at ${FRENET_BRIDGE}")
else()
    target_link_libraries(planning_node ${NODE_LINK_LIBS} stdc++)
    message(STATUS "planning_node: building in fallback mode (no frenet/Eigen)")
endif()

# control_node: PID 纵向/横向控制 (FlowCoro 协程版)
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
# 采用 CoroutineTask 同步 resume，与 fusion/perception/planning/control/safety_control 一致；
# 仅需 stdc++（子项目已 include flowcoro 头文件目录）。
add_library(inference_node SHARED inference_node.cpp)
target_link_libraries(inference_node ${NODE_LINK_LIBS} stdc++)

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

# ── 安装 ──────────────────────────────────────────────────────
install(TARGETS fusion_node perception_node sensor_model sim_world planning_node control_node safety_control_node monitor_node inference_node data_recorder_node learner_node model_ota_node flowmond_node
        LIBRARY DESTINATION ${FLOWENGINE_BUILD}/lib)

message(STATUS "Node plugins will be installed to: ${FLOWENGINE_BUILD}/lib")
