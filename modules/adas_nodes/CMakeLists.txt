cmake_minimum_required(VERSION 3.16)
project(flowengine_nodes C CXX)
enable_testing()

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
else()
    # 确保 FLOWENGINE_BUILD 是绝对路径，避免相对路径被 CMake 解析为子项目源目录下的相对路径
    get_filename_component(FLOWENGINE_BUILD "${FLOWENGINE_BUILD}" ABSOLUTE BASE_DIR "${FLOW_ROOT}")
endif()

message(STATUS "FlowEngine root:  ${FLOW_ROOT}")
message(STATUS "FlowEngine build: ${FLOWENGINE_BUILD}")
if(WIN32 AND NOT TARGET m)
    add_library(m INTERFACE)
endif()
if(WIN32)
    set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
endif()

# ── 头文件目录 ────────────────────────────────────────────────
# flowcoro 路径优先级：1. 同级目录本地仓库 2. third_party 3. FetchContent下载
set(_FLOWCORO_INCLUDE_CANDIDATES
    "${FLOW_ROOT}/../flowcoro/include"
    "${FLOW_ROOT}/third_party/flowcoro/include"
    "${FLOWENGINE_BUILD}/_deps/flowcoro-src/include"
)
set(FLOWCORO_INCLUDE_DIR "")
foreach(_cand IN LISTS _FLOWCORO_INCLUDE_CANDIDATES)
    if(EXISTS "${_cand}/flowcoro/rt_executor.h")
        set(FLOWCORO_INCLUDE_DIR "${_cand}")
        message(STATUS "flowcoro include: ${_cand}")
        break()
    endif()
endforeach()

include_directories(
    ${FLOW_ROOT}/include
    ${FLOW_ROOT}/src/algorithms            # ekf_fusion.h, dbscan_cluster.h
    ${FLOWCORO_INCLUDE_DIR}
    ${FLOWENGINE_BUILD}/gen                # 生成的 adas_msgs_gen.h
)

# ── 链接主框架库 ──────────────────────────────────────────────
find_library(FLOWENGINE_CORE
    NAMES flowengine_core
    HINTS
        ${FLOWENGINE_BUILD}/lib
        ${FLOWENGINE_BUILD}/lib/Release
        ${FLOWENGINE_BUILD}/lib/Debug
    REQUIRED
)
message(STATUS "libflowengine_core: ${FLOWENGINE_CORE}")

# scheduler_cpp: 提供 net_transport_start / net_transport_bridge_topic /
# net_transport_unbridge_topic 等 C++ transport 符号。所有节点 .so 在 init
# 阶段调用 transport API，dlopen RTLD_NOW 模式会立即解析这些符号，未链接 →
# "undefined symbol: net_transport_*" → Loaded 0/N plugins。主项目编译产物
# 落在 ${FLOWENGINE_BUILD}/lib/libscheduler_cpp.a（静态库，需 OBJECT 模式整体
# 链接进 .so）。find_library 缺失时降级为空字符串占位，节点链接会失败但
# cmake 仍可配置（错误在 link 阶段更明显）。
find_library(SCHEDULER_CPP
    NAMES scheduler_cpp
    HINTS
        ${FLOWENGINE_BUILD}/lib
        ${FLOWENGINE_BUILD}/lib/Release
        ${FLOWENGINE_BUILD}/lib/Debug
)
if(SCHEDULER_CPP)
    message(STATUS "libscheduler_cpp: ${SCHEDULER_CPP}")
else()
    message(WARNING "libscheduler_cpp NOT found — node plugins will fail at dlopen "
                    "(undefined symbol: net_transport_*)")
endif()

# Threads — 节点直接指定 -lpthread，不走 FindThreads（避免 c89-gcc 检测失败）
# stdc++ + atomic: launcher 是纯 C 程序，主进程符号表不含 C++ ABI（__cxxabiv1::*）
# 与 __atomic_store；C++ 节点 .so 与用 atomic 的节点在 dlopen RTLD_NOW 时会失败。
# 让每个节点 .so 自带这两个库依赖（--as-needed 会自动过滤未实际引用的库，
# 故 C 节点 .so 不会引入额外运行时依赖）。
# macOS: clang 用 libc++ 而非 libstdc++，原子操作为编译器内建、无独立 libatomic，
# 故这两个库名不存在，替换为 c++（libc++）。
if(WIN32)
    set(NODE_LINK_LIBS ${FLOWENGINE_CORE} ${SCHEDULER_CPP} ws2_32)
    set(NODE_CXX_RT "")
elseif(APPLE)
    set(NODE_LINK_LIBS ${FLOWENGINE_CORE} ${SCHEDULER_CPP} m pthread c++)
    # 各节点 target_link_libraries 里额外重申的 C++/atomic 运行时:macOS 用 libc++,
    # 无独立 libatomic(原子为编译器内建),故为 c++;Linux 保持 stdc++ atomic 不变。
    set(NODE_CXX_RT c++)
else()
    set(NODE_LINK_LIBS ${FLOWENGINE_CORE} ${SCHEDULER_CPP} m pthread stdc++ atomic)
    set(NODE_CXX_RT stdc++ atomic)
endif()

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
            ${FLOWENGINE_BUILD}/lib/Release
            ${FLOWENGINE_BUILD}/lib/Debug
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
if(MSVC)
    add_compile_options(/W4 /utf-8 /Zc:__cplusplus /experimental:c11atomics /wd4200 /wd4201 /wd4996 /UERROR)
    add_compile_definitions(_CRT_SECURE_NO_WARNINGS NOMINMAX WIN32_LEAN_AND_MEAN)
else()
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -fPIC -D_GNU_SOURCE")
endif()
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT MSVC)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -fPIC -D_GNU_SOURCE")
endif()
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# flowcoro/logger.h（经 rt_executor.h → coroutine_task.h 引入）在默认参数中用
# PROJECT_SOURCE_DIR "/flowcoro.log" 做字符串拼接，需要该宏是字符串字面量。
# 主项目通过 flowcoro_headers INTERFACE 目标提供此定义；节点子项目不链接该
# 目标，故在此显式补充（值用 FlowEngine 根目录，日志文件落在根目录下）。
add_compile_definitions(PROJECT_SOURCE_DIR="${FLOW_ROOT}")

# macOS 跨平台兼容层：全局 force-include，统一 pthread_setname_np 签名、robust
# mutex 降级等平台差异（详见 include/platform_compat.h）。仅 APPLE 生效，
# Linux 完全透明。第三方（cJSON/Eigen/flowcoro）不调用这些符号，无副作用。
if(WIN32)
    if(MSVC)
        add_compile_options(/FI${FLOW_ROOT}/include/platform_compat.h)
    else()
        add_compile_options(-include ${FLOW_ROOT}/include/platform_compat.h)
    endif()
    include_directories(BEFORE ${FLOW_ROOT}/include/compat_win)
elseif(APPLE)
    add_compile_options(-include ${FLOW_ROOT}/include/platform_compat.h)
    # 节点 .so 是被 flow_launcher / flow_node_host 用 dlopen 加载的插件,宿主进程
    # 已链接 flowengine_core。两处差异需要对齐 Linux 的"扁平命名空间"语义:
    #
    #  (1) 未定义符号(如 monitor_server 的 zlib deflate)—— Linux dlopen 在加载时
    #      从宿主解析;macOS 两级命名空间默认链接期就报 undefined。
    #  (2) 单例全局状态 —— 每个节点 .so 都静态链入了 libflowengine_core.a,于是各自
    #      带了一份 flow_registry 的 g_tasks[]、param_registry 等全局。Linux 扁平命名
    #      空间下所有引用都绑定到宿主可执行文件里的那一份(符号插入/interposition),
    #      故注册表跨"宿主 + 各插件"共享同一实例;macOS 两级命名空间则各用各的副本,
    #      导致 flow_launcher 往宿主副本注册任务、monitor_node 却读自己那份空的 ——
    #      仪表盘/pipeline_check 里 registry.tasks 恒为空。
    #
    # -flat_namespace 让插件的符号解析改用扁平命名空间:同名符号跨所有已加载映像
    # 解析,先加载者(宿主可执行文件)胜出,与 Linux 完全一致;-undefined
    # dynamic_lookup 处理 (1) 的未定义符号推迟到 dlopen 解析。二者仅 APPLE 生效。
    add_link_options(-Wl,-flat_namespace -Wl,-undefined,dynamic_lookup)
endif()

# 输出目录: 与主项目的 build/lib 保持一致，方便 flow_launcher 查找
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${FLOWENGINE_BUILD}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${FLOWENGINE_BUILD}/lib)

# ── 节点插件 ──────────────────────────────────────────────────

# fusion_node: EKF 5D 传感器融合 (FlowCoro 协程版)
# 采用 FlowCoroTask（线程池 resume）：EKF + 序列化 + publish 单次 ~50-100μs，
# 同步 resume 会阻塞消息总线分发线程导致 drops，故改用线程池 resume。
# flowcoro 核心库为 header-only（INTERFACE），子项目已 include 其头文件目录，
# 故只需 FLOWCORO_INTEGRATION 定义 + -fcoroutines，无需额外链接 flowcoro 库。
add_library(fusion_node SHARED fusion_node.cpp)
target_compile_definitions(fusion_node PRIVATE FLOWCORO_INTEGRATION)
target_compile_options(fusion_node PRIVATE $<$<CXX_COMPILER_ID:GNU>:-fcoroutines>)
target_link_libraries(fusion_node ${NODE_LINK_LIBS} ${NODE_CXX_RT})

# perception_node: DBSCAN 感知 + 传感器仿真 (FlowCoro 协程版)
# 采用 FlowCoroTask（线程池 resume）：DBSCAN 点云聚类为重计算，同步 resume 会阻塞
# 消息总线分发线程导致 drops，故改用线程池 resume。
add_library(perception_node SHARED perception_node.cpp)
target_compile_definitions(perception_node PRIVATE FLOWCORO_INTEGRATION)
target_compile_options(perception_node PRIVATE $<$<CXX_COMPILER_ID:GNU>:-fcoroutines>)
target_link_libraries(perception_node ${NODE_LINK_LIBS} ${NODE_CXX_RT})

# behavior_planner_node: 行为规划节点 (FlowCoro 协程版)
# 消费感知语义输出，运行巡航/跟车/变道状态机，发布 planning/behavior。
add_library(behavior_planner_node SHARED behavior_planner_node.cpp)
target_compile_definitions(behavior_planner_node PRIVATE FLOWCORO_INTEGRATION)
target_compile_options(behavior_planner_node PRIVATE $<$<CXX_COMPILER_ID:GNU>:-fcoroutines>)
target_link_libraries(behavior_planner_node ${NODE_LINK_LIBS} ${NODE_CXX_RT})

# navigation_node: 导航路线节点（场景 route[] -> navigation/path）。
# planning_node 只消费 navigation/path，不再自行读取 scenario route。
add_library(navigation_node SHARED navigation_node.c)
target_link_libraries(navigation_node ${NODE_LINK_LIBS})

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
# 预构建库按平台存放：Linux = lib/libesminiRMLib.so（ELF），macOS = lib/libesminiRMLib.dylib（Mach-O）。
# find_library 按平台自动匹配后缀（Linux → .so，macOS → .dylib），两个文件可并存。
set(ESMINI_ROOT "${FLOW_ROOT}/third_party/esmini")
# 头文件搜索路径：子模块 → 本地预构建 include（CI 无子模块时走第二条）
set(ESMINI_RMLIB_INCLUDE
    "${ESMINI_ROOT}/EnvironmentSimulator/Libraries/esminiRMLib"
    "${FLOW_ROOT}/include"
)
if(APPLE)
    # macOS：预构建为 Mach-O .dylib，显式优先 .dylib，避免误命中 Linux 的 .so
    find_library(ESMINI_RMLIB
        NAMES esminiRMLib.dylib esminiRMLib
        HINTS "${FLOW_ROOT}/lib"                                    # 预构建（CI 优先）
              "${ESMINI_ROOT}/build/EnvironmentSimulator/Libraries/esminiRMLib"  # 子模块构建
    )
else()
    find_library(ESMINI_RMLIB
        NAMES esminiRMLib
        HINTS "${FLOW_ROOT}/lib"                                    # 预构建（CI 优先）
              "${ESMINI_ROOT}/build/EnvironmentSimulator/Libraries/esminiRMLib"  # 子模块构建
    )
endif()
# ── 预构建库平台格式门禁 ─────────────────────────────────────────
# 0ca4c09 曾把 macOS 构建的 Mach-O 直接覆盖 lib/libesminiRMLib.so（Linux ELF），
# 导致 Linux 链接全部失败且报错晦涩（"file format not recognized"）。
# 这里 configure 期用 `file` 校验格式：Linux 必须 ELF、macOS 必须 Mach-O，
# 交叉提交二进制立即 FATAL_ERROR 并附恢复指引（对齐 Apollo：构建产物按平台
# 强绑定校验，绝不交叉使用）。
if(ESMINI_RMLIB)
    find_program(_ESMINI_FILE_TOOL file)
    if(_ESMINI_FILE_TOOL)
        execute_process(COMMAND ${_ESMINI_FILE_TOOL} "${ESMINI_RMLIB}"
            OUTPUT_VARIABLE _esmini_file_out ERROR_QUIET)
        if(APPLE)
            if(NOT _esmini_file_out MATCHES "Mach-O")
                message(FATAL_ERROR
                    "esminiRMLib at ${ESMINI_RMLIB} is NOT Mach-O (macOS 需要 Mach-O).\n"
                    "  若被 Linux ELF 误覆盖，请在 macOS 上重新构建 libesminiRMLib.dylib。")
            endif()
        else()
            if(NOT _esmini_file_out MATCHES "ELF")
                message(FATAL_ERROR
                    "esminiRMLib at ${ESMINI_RMLIB} is NOT ELF (Linux 需要 ELF).\n"
                    "  若被 macOS 构建误覆盖：git checkout bd69f4a -- lib/libesminiRMLib.so\n"
                    "  恢复真实 Linux 库后重新 configure。")
            endif()
        endif()
        message(STATUS "flowsim_node: esminiRMLib format OK (${_esmini_file_out})")
    endif()
endif()
add_library(flowsim_node SHARED
    flowsim_node.cpp
    flowsim/road_network.cpp
    flowsim/road_position.cpp
    flowsim/physics.cpp
    flowsim/npc_ai.cpp
    flowsim/collision.cpp
    flowsim/scene_events.cpp
    flowsim/scene_pub.cpp
    flowsim/sim_digest.cpp
    flowsim/actor/VehicleActor.cpp
    flowsim/route.cpp
)
if(NOT ESMINI_RMLIB)
    target_sources(flowsim_node PRIVATE flowsim/esmini_stub.cpp)
endif()
target_compile_definitions(flowsim_node PRIVATE FLOWCORO_INTEGRATION)
target_compile_options(flowsim_node PRIVATE $<$<CXX_COMPILER_ID:GNU>:-fcoroutines>)
target_include_directories(flowsim_node PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${ESMINI_RMLIB_INCLUDE}
)
if(ESMINI_RMLIB)
    target_link_libraries(flowsim_node ${NODE_LINK_LIBS} ${NODE_CXX_RT} ${ESMINI_RMLIB})
    message(STATUS "flowsim_node: esminiRMLib found at ${ESMINI_RMLIB}")
else()
    target_link_libraries(flowsim_node ${NODE_LINK_LIBS} ${NODE_CXX_RT})
    message(WARNING "flowsim_node: esminiRMLib NOT found — using limited RoadManager stub")
endif()

# ── 离线 esmini 链路自检（挂 ctest）───────────────────────────
# test_road_network 是场景无关的 invariant 测试：json_to_xodr.py 把场景 JSON 转成
# xodr，再用真实预构建 esminiRMLib 验证 load/路数/车道宽/frenet↔world 往返。
# 不启动 demo、秒级完成（对齐 pipeline_check.py 的离线哲学）。
# 曾因硬编码已删除场景的断言而无法运行，2026-07 改为通用不变式。
if(ESMINI_RMLIB)
    find_package(Python3 COMPONENTS Interpreter QUIET)
    add_executable(test_road_network
        flowsim/test_road_network.cpp
        flowsim/road_network.cpp
        flowsim/road_position.cpp
    )
    target_include_directories(test_road_network PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${ESMINI_RMLIB_INCLUDE}
    )
    target_link_libraries(test_road_network ${ESMINI_RMLIB})
    if(Python3_Interpreter_FOUND)
        add_test(NAME gen_esmini_xodr
            COMMAND ${Python3_EXECUTABLE} ${FLOW_ROOT}/tools/json_to_xodr.py
                    ${FLOW_ROOT}/scenarios/straight_road.json
                    -o ${CMAKE_CURRENT_BINARY_DIR}/straight_road.xodr
        )
        set_tests_properties(gen_esmini_xodr PROPERTIES FIXTURES_SETUP XODR)
        add_test(NAME flowsim_road_network
            COMMAND test_road_network ${CMAKE_CURRENT_BINARY_DIR}/straight_road.xodr
        )
        set_tests_properties(flowsim_road_network PROPERTIES FIXTURES_REQUIRED XODR)
    endif()
endif()

# planning_node: Frenet 轨迹规划 (FlowCoro 协程版)
# 采用 FlowCoroTask（线程池 resume）：Frenet 轨迹规划为重计算，同步 resume 会阻塞
# 消息总线分发线程导致 drops，故改用线程池 resume。
find_library(FRENET_BRIDGE
    NAMES frenet_bridge
    HINTS ${FLOWENGINE_BUILD}/lib ${FLOWENGINE_BUILD}/lib/Release ${FLOWENGINE_BUILD}/lib/Debug
          ${FLOW_ROOT}/build-algo/lib
)
find_library(FRENET_PLANNER
    NAMES frenet_planner
    HINTS ${FLOWENGINE_BUILD}/lib ${FLOWENGINE_BUILD}/lib/Release ${FLOWENGINE_BUILD}/lib/Debug
          ${FLOW_ROOT}/build-algo/lib
)
add_library(planning_node SHARED planning_node.cpp st_graph.c)
target_compile_definitions(planning_node PRIVATE FLOWCORO_INTEGRATION)
target_compile_options(planning_node PRIVATE $<$<CXX_COMPILER_ID:GNU>:-fcoroutines>)
if(FRENET_BRIDGE AND FRENET_PLANNER)
    target_link_libraries(planning_node ${NODE_LINK_LIBS}
        ${FRENET_BRIDGE} ${FRENET_PLANNER} ${NODE_CXX_RT})
    target_include_directories(planning_node PRIVATE
        ${FLOW_ROOT}/third_party/frenet_planner/src
        ${FLOW_ROOT}/third_party/frenet_planner/src/FrenetOptimalTrajectory
        ${FLOWENGINE_BUILD}/_deps/eigen-src   # Eigen3 headers (downloaded)
    )
    target_compile_definitions(planning_node PRIVATE HAVE_FRENET)
    message(STATUS "planning_node: frenet_bridge found at ${FRENET_BRIDGE}")
else()
    target_link_libraries(planning_node ${NODE_LINK_LIBS} ${NODE_CXX_RT})
    message(STATUS "planning_node: building in fallback mode (no frenet/Eigen)")
endif()

# control_node: PID 纵向/横向控制 (FlowCoro 协程版)
# 采用 CoroutineTask（同步 resume）：control 是延迟敏感的闭环控制，FlowCoroTask
# 线程池 resume 的调度抖动会导致 20Hz 周期不一致，prev_steer 低通滤波时间间隔
# 波动，steer 产生小幅振荡。CoroutineTask 周期精确，且 PID+Stanley 计算量小，
# 同步 resume 阻塞可忽略。与 safety_control 一致。
add_library(control_node SHARED control_node.cpp)
target_link_libraries(control_node ${NODE_LINK_LIBS} ${NODE_CXX_RT})

# safety_control_node: safety envelope for control commands
# Uses CoroutineTask (synchronous resume on bus delivery thread) instead of
# FlowCoroTask — the safety check is lightweight (<10us) and synchronous resume
# avoids thread-pool starvation that caused control/cmd staleness.
add_library(safety_control_node SHARED safety_control_node.cpp)
target_link_libraries(safety_control_node ${NODE_LINK_LIBS} ${NODE_CXX_RT})

# monitor_node: 系统监控 + 仪表盘 JSON 导出
add_library(monitor_node SHARED monitor_node.c)
target_link_libraries(monitor_node ${NODE_LINK_LIBS})

# ── 车端学习闭环节点 (data → train → inference → learner → ota) ─

# inference_node: 内置 tiny-MLP 车端推理（影子模式，支持 OTA 热重载，FlowCoro 协程版）
# 采用 FlowCoroTask（线程池 resume）：MLP 推理为重计算，同步 resume 会阻塞
# 消息总线分发线程导致 drops，故改用线程池 resume。
#
# 前向计算有两个可切换后端：tiny-MLP（纯 C，默认，零依赖）与 ONNX Runtime（可选）。
# ENABLE_ONNX 开关：cmake -DENABLE_ONNX=ON ...。开启且系统装有 onnxruntime 时，
# 定义 HAVE_ONNXRUNTIME 并链接 ORT，inference_node 可加载 .onnx 模型；未开启或未找到
# 时 onnx_backend.cpp 编成降级桩（load→-1），运行时自动回退 tiny-MLP，行为零变化。
# 后端切换只影响「前向计算」一步，特征组装/输出映射/影子逻辑完全复用（见 onnx_backend.h）。
option(ENABLE_ONNX "Enable ONNX Runtime inference backend (requires onnxruntime)" OFF)
add_library(inference_node SHARED inference_node.cpp onnx_backend.cpp)
target_compile_definitions(inference_node PRIVATE FLOWCORO_INTEGRATION)
target_compile_options(inference_node PRIVATE $<$<CXX_COMPILER_ID:GNU>:-fcoroutines>)
target_link_libraries(inference_node ${NODE_LINK_LIBS} ${NODE_CXX_RT})
if(ENABLE_ONNX)
    # 优先 CONFIG 包（onnxruntime 官方 CMake 导出），兜底 find_path + find_library。
    find_package(onnxruntime CONFIG QUIET)
    if(onnxruntime_FOUND)
        target_compile_definitions(inference_node PRIVATE HAVE_ONNXRUNTIME)
        target_link_libraries(inference_node onnxruntime::onnxruntime)
        message(STATUS "inference_node: HAVE_ONNXRUNTIME enabled (onnxruntime CONFIG package)")
    else()
        find_path(ONNXRUNTIME_INCLUDE_DIR onnxruntime_cxx_api.h
                  HINTS $ENV{HOME}/.local/onnxruntime/include
                  PATH_SUFFIXES onnxruntime onnxruntime/core/session)
        find_library(ONNXRUNTIME_LIB NAMES onnxruntime
                     HINTS $ENV{HOME}/.local/onnxruntime/lib)
        if(ONNXRUNTIME_INCLUDE_DIR AND ONNXRUNTIME_LIB)
            target_compile_definitions(inference_node PRIVATE HAVE_ONNXRUNTIME)
            target_include_directories(inference_node PRIVATE ${ONNXRUNTIME_INCLUDE_DIR})
            target_link_libraries(inference_node ${ONNXRUNTIME_LIB})
            message(STATUS "inference_node: HAVE_ONNXRUNTIME enabled (${ONNXRUNTIME_LIB})")
        else()
            message(WARNING "ENABLE_ONNX=ON 但未找到 onnxruntime，onnx_backend 编为降级桩，运行时回退 tiny-MLP")
        endif()
    endif()
endif()

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
# 为 GPS 丢失场景（室内/隧道）提供定位。支持三种算法模式：
#   - dead_reckon: 默认简易里程计（陀螺仪积分 heading，加速度计积分速度）
#   - ekf_slam:   轻量 EKF-SLAM（5维状态估计器，内置实现，无需外部依赖）
#   - fast_lio2:  FAST-LIO2 紧耦合（未实现：当前 LidarFrame 单点非点云，一律拒绝降级）
# 无传感器时自动降级为 dry-run（生成圆形轨迹模拟位姿）。
# 编译依赖: adas_msgs_gen.h (LidarFrame/ImuData/Pose2D 序列化)，随构建生成。
#
# EKF-SLAM 已内置（algo="ekf_slam"），无需额外编译开关，直接启用即可。
# 状态向量: [x, y, heading, v, omega]，完整协方差传播与量测更新。
#
# ENABLE_FAST_LIO2 开关：cmake -DENABLE_FAST_LIO2=ON ...
# 启用后 slam_node 定义 HAVE_FAST_LIO2 宏。但**即便开启也无法真正运行 FAST-LIO2**：
# 当前 sensor/lidar 的 LidarFrame 是单点伪传感器（非点云），fast_lio2_init() 一律
# 拒绝并降级 dead_reckon（见 slam_node.cpp 文件头 + HAVE_FAST_LIO2 块注释）。
# 前置里程碑（不在本轮）：先把 LidarFrame 扩成真实点云 topic，才谈得上接 FAST_LIO。
option(ENABLE_FAST_LIO2 "Enable FAST-LIO2 SLAM backend (requires PCL/Eigen/Sophus + libfast_lio2.a)" OFF)
add_library(slam_node SHARED slam_node.cpp ekf_slam.c)
target_link_libraries(slam_node ${NODE_LINK_LIBS} ${NODE_CXX_RT})
if(ENABLE_FAST_LIO2)
    target_compile_definitions(slam_node PRIVATE HAVE_FAST_LIO2)
    # 用户预装好依赖后取消注释以下行：
    # find_package(PCL REQUIRED COMPONENTS common io)
    # find_package(Eigen3 REQUIRED)
    # find_package(Sophus REQUIRED)
    # find_library(FAST_LIO2_LIB NAMES fast_lio2 HINTS /usr/local/lib)
    # target_link_libraries(slam_node ${NODE_LINK_LIBS} stdc++
    #     ${PCL_LIBRARIES} Eigen3::Eigen Sophus::Sophus ${FAST_LIO2_LIB})
    message(STATUS "slam_node: HAVE_FAST_LIO2 enabled (仍未实现：LidarFrame 单点非点云，运行时拒绝降级 dead_reckon)")
endif()

# test_ekf_slam / test_ekf_fusion 已移至主项目 CMakeLists.txt 注册（enable_testing
# + add_test，纳入标准 ctest 门禁）。此子项目不 enable_testing()，此前的 add_executable
# 只编译不进门禁，形同虚设，故删除。

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
target_link_libraries(perception_fusion_node ${NODE_LINK_LIBS} ${NODE_CXX_RT})

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

# guardian_node: 已删除（C-4）。
# 原为独立安全看门狗（Apollo Guardian 等价），监控碰撞风险/车道偏离/超速/CMD 合法性/
# 传感器超时。但当前 in-process NodePlugin 架构下"独立进程"假设不成立——所有节点在
# 同一进程内，guardian 无法在 control 崩溃时幸存。其安全功能由 safety_control_node
# 覆盖（对 control/cmd 做最终 envelope 检查，是 pipeline 最后一环）。

# scene_assembler_node: Aggregate all perception into unified EnvironmentModel
add_library(scene_assembler_node SHARED scene_assembler_node.c)
target_link_libraries(scene_assembler_node ${NODE_LINK_LIBS})

# ── macOS 插件后缀强制 .so ─────────────────────────────────────
# macOS 上 SHARED 库默认后缀为 .dylib，但 pipeline.json 与 flow_launcher 均按
# build/lib/libXXX_node.so 硬编码查找。dlopen 对后缀无要求（按路径加载即可），
# 故统一把本目录所有 SHARED 目标的后缀强制成 .so，让路径与 Linux 完全一致。
if(APPLE)
    get_property(_flow_node_targets DIRECTORY PROPERTY BUILDSYSTEM_TARGETS)
    foreach(_t ${_flow_node_targets})
        get_target_property(_ty ${_t} TYPE)
        if(_ty STREQUAL "SHARED_LIBRARY")
            set_target_properties(${_t} PROPERTIES SUFFIX ".so")
        endif()
    endforeach()
endif()

# ── 安装 ──────────────────────────────────────────────────────
set(FLOWENGINE_NODE_PLUGIN_TARGETS
    fusion_node
    perception_node
    behavior_planner_node
    sensor_model
    flowsim_node
    planning_node
    control_node
    safety_control_node
    monitor_node
    inference_node
    data_recorder_node
    learner_node
    model_ota_node
    flowmond_node
    actuator_node
    manual_drive_node
    lidar_driver_node
    gps_driver_node
    imu_driver_node
    slam_node
    stereo_camera_node
    waypoint_follower_node
    stereo_vision_node
    actuator_pwm_node
    perception_fusion_node
    traversability_node
    lane_detection_node
    traffic_light_recognition_node
    object_tracker_node
    prediction_node
    scene_assembler_node
)

install(TARGETS ${FLOWENGINE_NODE_PLUGIN_TARGETS}
        LIBRARY DESTINATION lib/flowengine/plugins)

message(STATUS "Node plugins will be installed to: lib/flowengine/plugins")
