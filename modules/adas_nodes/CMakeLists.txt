cmake_minimum_required(VERSION 3.16)
project(flowengine_nodes C)

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

# ── 编译选项 ──────────────────────────────────────────────────
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -fPIC -D_GNU_SOURCE")
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# 输出目录: 与主项目的 build/lib 保持一致，方便 flow_launcher 查找
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${FLOWENGINE_BUILD}/lib)

# ── 节点插件 ──────────────────────────────────────────────────

# fuse_node: EKF 5D 传感器融合
add_library(fusion_node SHARED fusion_node.c)
target_link_libraries(fusion_node ${NODE_LINK_LIBS})

# perception_node: DBSCAN 感知 + 传感器仿真
add_library(perception_node SHARED perception_node.c)
target_link_libraries(perception_node ${NODE_LINK_LIBS})

# ── 新增节点插件 ──────────────────────────────────────────────

# sim_world_node: 车辆动力学 + 障碍物仿真
add_library(sim_world SHARED sim_world_node.c)
target_link_libraries(sim_world ${NODE_LINK_LIBS})

# planning_node: Frenet 轨迹规划（需要 C++ 运行时的 frenet_bridge）
find_library(FRENET_BRIDGE
    NAMES frenet_bridge
    HINTS ${FLOWENGINE_BUILD}/lib ${FLOW_ROOT}/build-algo/lib
)
find_library(FRENET_PLANNER
    NAMES frenet_planner
    HINTS ${FLOWENGINE_BUILD}/lib ${FLOW_ROOT}/build-algo/lib
)
if(FRENET_BRIDGE AND FRENET_PLANNER)
    add_library(planning_node SHARED planning_node.c)
    target_link_libraries(planning_node ${NODE_LINK_LIBS}
        ${FRENET_BRIDGE} ${FRENET_PLANNER} stdc++)
    target_include_directories(planning_node PRIVATE
        ${FLOW_ROOT}/third_party/frenet_planner/src
        ${FLOW_ROOT}/third_party/frenet_planner/src/FrenetOptimalTrajectory
    )
    message(STATUS "planning_node: frenet_bridge found at ${FRENET_BRIDGE}")
    message(STATUS "planning_node: frenet_planner found at ${FRENET_PLANNER}")
    set(PLANNING_NODE_TARGET planning_node)
else()
    message(WARNING "planning_node disabled: missing frenet_bridge/frenet_planner (searched in ${FLOWENGINE_BUILD}/lib and ${FLOW_ROOT}/build-algo/lib)")
endif()

# control_node: PID 纵向/横向控制
add_library(control_node SHARED control_node.c)
target_link_libraries(control_node ${NODE_LINK_LIBS})

# monitor_node: 系统监控 + 仪表盘 JSON 导出
add_library(monitor_node SHARED monitor_node.c)
target_link_libraries(monitor_node ${NODE_LINK_LIBS})

# ── 安装 ──────────────────────────────────────────────────────
install(TARGETS fusion_node perception_node sim_world ${PLANNING_NODE_TARGET} control_node monitor_node
        LIBRARY DESTINATION ${FLOWENGINE_BUILD}/lib)

message(STATUS "Node plugins will be installed to: ${FLOWENGINE_BUILD}/lib")
