cmake_minimum_required(VERSION 3.16)
project(FlowEngine VERSION 1.0.0 LANGUAGES C CXX)

# 设置C和C++标准
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 设置编译选项
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -fPIC -D_GNU_SOURCE")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -fPIC -fcoroutines -D_GNU_SOURCE")
set(CMAKE_C_FLAGS_DEBUG "-g -O0 -DDEBUG")
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0 -DDEBUG")
set(CMAKE_C_FLAGS_RELEASE "-O2 -DNDEBUG")
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -DNDEBUG")

# ── Sanitizer options ──────────────────────────────────────
option(ENABLE_ASAN "Enable Address Sanitizer" OFF)
option(ENABLE_TSAN "Enable Thread Sanitizer" OFF)
option(ENABLE_UBSAN "Enable Undefined Behavior Sanitizer" OFF)
option(ENABLE_COVERAGE "Enable code coverage" OFF)

if(ENABLE_ASAN)
    add_compile_options(-fsanitize=address -fno-omit-frame-pointer)
    add_link_options(-fsanitize=address)
    message(STATUS "ASAN enabled")
endif()
if(ENABLE_TSAN)
    add_compile_options(-fsanitize=thread -fno-omit-frame-pointer)
    add_link_options(-fsanitize=thread)
    message(STATUS "TSAN enabled")
endif()
if(ENABLE_UBSAN)
    add_compile_options(-fsanitize=undefined -fno-omit-frame-pointer)
    add_link_options(-fsanitize=undefined)
    message(STATUS "UBSAN enabled")
endif()
if(ENABLE_COVERAGE)
    add_compile_options(--coverage -O0 -g)
    add_link_options(--coverage)
    message(STATUS "Coverage enabled")
endif()

# 如果没有指定构建类型，默认为Release
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# 设置输出目录
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# 包含目录
include_directories(include)

# 查找依赖库
find_package(Threads REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

# =============================================================================
# Phase 1: IDL 代码生成 (msg/*.msg -> 序列化头文件)
# =============================================================================
set(MSG_CODEGEN "${CMAKE_SOURCE_DIR}/tools/msg_codegen.py")
set(GEN_DIR     "${CMAKE_BINARY_DIR}/gen")

# 主 IDL 文件
set(MSG_FILES
    msg/adas_msgs.msg
)

# 为每个 .msg 生成拆分的 per-struct 头文件 + umbrella 头文件
foreach(MSG_FILE ${MSG_FILES})
    get_filename_component(MSG_NAME ${MSG_FILE} NAME_WE)
    set(GEN_HEADER "${GEN_DIR}/${MSG_NAME}_gen.h")
    add_custom_command(
        OUTPUT ${GEN_HEADER}
        COMMAND ${Python3_EXECUTABLE} ${MSG_CODEGEN} "${CMAKE_SOURCE_DIR}/${MSG_FILE}"
                --split --output-dir ${CMAKE_BINARY_DIR}
        DEPENDS ${MSG_CODEGEN} ${CMAKE_SOURCE_DIR}/${MSG_FILE}
        COMMENT "Generating split serialization headers from ${MSG_FILE}"
    )
    list(APPEND GEN_HEADERS ${GEN_HEADER})
endforeach()

# 便捷目标：只生成代码
add_custom_target(msg_codegen DEPENDS ${GEN_HEADERS})

# =============================================================================
# flowcoro — C++20 高性能协程库 (https://github.com/caixuf/flowcoro)
# =============================================================================
include(FetchContent)

# 查找cjson库 — 优先使用系统版本，否则从源码构建
pkg_check_modules(CJSON libcjson)
if(NOT CJSON_FOUND)
    message(STATUS "libcjson not found via pkg-config — building from source via FetchContent")
    FetchContent_Declare(
        cjson
        GIT_REPOSITORY https://github.com/DaveGamble/cJSON.git
        GIT_TAG        v1.7.17
        GIT_SHALLOW    TRUE
    )
    set(ENABLE_CJSON_TEST   OFF CACHE BOOL "" FORCE)
    set(ENABLE_CJSON_UTILS  OFF CACHE BOOL "" FORCE)
    set(BUILD_SHARED_LIBS   OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(cjson)
    # config_manager.c uses <cjson/cJSON.h> (the system install layout).
    # When bundling, create a cjson/ subdirectory so the same include path works.
    set(_CJSON_WRAPPER_DIR "${CMAKE_BINARY_DIR}/_cjson_include/cjson")
    file(MAKE_DIRECTORY "${_CJSON_WRAPPER_DIR}")
    configure_file("${cjson_SOURCE_DIR}/cJSON.h" "${_CJSON_WRAPPER_DIR}/cJSON.h" COPYONLY)
    # Map FetchContent target to the variables used below
    set(CJSON_LIBRARIES    cjson)
    set(CJSON_INCLUDE_DIRS "${CMAKE_BINARY_DIR}/_cjson_include")
    set(CJSON_CFLAGS_OTHER "")
endif()

# 关闭 flowcoro 自身的测试/示例/文档，只编译核心库
set(FLOWCORO_BUILD_TESTS     OFF CACHE BOOL "" FORCE)
set(FLOWCORO_BUILD_EXAMPLES  OFF CACHE BOOL "" FORCE)
set(FLOWCORO_BUILD_BENCHMARKS OFF CACHE BOOL "" FORCE)
set(FLOWCORO_BUILD_DOCS      OFF CACHE BOOL "" FORCE)
set(FLOWCORO_ENABLE_DATABASE OFF CACHE BOOL "" FORCE)
set(FLOWCORO_BUILD_V3        OFF CACHE BOOL "" FORCE)

FetchContent_Declare(
    flowcoro
    GIT_REPOSITORY https://github.com/caixuf/flowcoro.git
    GIT_TAG        0934b6f22cf75bc12ca2c60caaaf78519bf4921c
    GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(flowcoro)

# ── Eigen3 (header-only linear algebra, for Frenet planner) ──
set(EIGEN_BUILD_DOC OFF CACHE BOOL "" FORCE)
set(EIGEN_BUILD_PKGCONFIG OFF CACHE BOOL "" FORCE)
# Try to get Eigen 3.4.0; degrade gracefully if network is unavailable.
# Eigen is header-only so we only need the source tree, not a full CMake build.
set(_EIGEN_SRC_DIR "${CMAKE_BINARY_DIR}/_deps/eigen-src")
if(NOT EXISTS "${_EIGEN_SRC_DIR}/Eigen/Core")
    message(STATUS "Attempting to download Eigen 3.4.0 (non-interactive)...")
    find_package(Git QUIET)
    if(Git_FOUND OR GIT_FOUND)
        set(_GIT_EXE "${GIT_EXECUTABLE}")
    else()
        find_program(_GIT_EXE git)
    endif()
    execute_process(
        COMMAND ${_GIT_EXE} clone
            --depth 1 --branch 3.4.0 --quiet
            -c credential.helper=   # disable interactive credential prompts so the clone fails fast if auth is needed
            https://github.com/libeigen/eigen.git
            "${_EIGEN_SRC_DIR}"
        TIMEOUT 60
        RESULT_VARIABLE _eigen_clone_result
        OUTPUT_QUIET
        ERROR_QUIET
    )
    if(_eigen_clone_result EQUAL 0)
        message(STATUS "Eigen 3.4.0 downloaded to ${_EIGEN_SRC_DIR}")
    else()
        message(STATUS "Eigen download failed (network unavailable) — frenet planner will be skipped")
    endif()
endif()

set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
# Use system-installed Eigen if available (fastest path, no network/download needed)
find_package(Eigen3 3.3 QUIET NO_MODULE)
if(Eigen3_FOUND)
    set(FLOWENGINE_HAVE_EIGEN TRUE)
    message(STATUS "Eigen found (system) — frenet planner will be built")
elseif(EXISTS "${_EIGEN_SRC_DIR}/Eigen/Core")
    # Add Eigen as interface library without building its tests/docs
    if(NOT TARGET Eigen3::Eigen)
        add_subdirectory("${_EIGEN_SRC_DIR}" "${CMAKE_BINARY_DIR}/_deps/eigen-build" EXCLUDE_FROM_ALL)
    endif()
    set(FLOWENGINE_HAVE_EIGEN TRUE)
    message(STATUS "Eigen found (downloaded) — frenet planner will be built")
else()
    set(FLOWENGINE_HAVE_EIGEN FALSE)
    message(STATUS "Eigen not available — building without frenet planner. "
        "IMPACT: planning_node falls back to lane-keep-only mode — the ego "
        "vehicle will NEVER change lanes or overtake. Install libeigen3-dev "
        "and reconfigure to enable lane changes (core + unit tests unaffected).")
endif()
set(BUILD_TESTING ON CACHE BOOL "" FORCE)

# =============================================================================
# flowcoro_headers — 全局 INTERFACE 目标
#
# 暴露 flowcoro 头文件路径 + C++20 协程支持，供所有协程相关目标统一引用。
# 避免继承 flowcoro INTERFACE 的激进编译参数影响整个项目。
# 使用方式：
#   target_link_libraries(<target> flowcoro_headers)
#   target_compile_definitions(<target> PRIVATE FLOWCORO_INTEGRATION)
# =============================================================================
add_library(flowcoro_headers INTERFACE)
target_include_directories(flowcoro_headers INTERFACE
    ${flowcoro_SOURCE_DIR}/include
)
target_compile_features(flowcoro_headers INTERFACE cxx_std_20)
# GCC 需要显式 -fcoroutines（C++20 模式下通常已隐含，但显式更安全）
target_compile_options(flowcoro_headers INTERFACE
    $<$<AND:$<COMPILE_LANGUAGE:CXX>,$<CXX_COMPILER_ID:GNU>>:-fcoroutines>
)
target_compile_definitions(flowcoro_headers INTERFACE
    PROJECT_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
)
target_link_libraries(flowcoro_headers INTERFACE Threads::Threads)

# 定义源文件
set(CORE_SOURCES
    src/core/config_manager.c
    src/core/logger.c
    src/core/process_manager.c
    src/core/task_interface.c
    src/core/task_manager.c
    src/core/message_bus.c
    src/core/ipc_channel.c
    src/core/clock_service.c
    src/core/bag.c
    src/core/msg_schema.c
    src/core/serializer.c
    src/core/scheduler.c
    src/core/state_machine.c
    src/core/discovery.c
    src/core/fusion.c
    src/core/proto_support.c
    src/core/transport.c
    src/core/flow_registry.c
    src/core/param_registry.c
    src/core/monitor_server.c
    src/core/stats_bridge.c
    src/core/dashboard_bridge.c
    src/core/mcap_writer.c
    src/core/mcap_reader.c
    src/core/sysmonitor.c
    src/core/scenario_loader.c
    src/core/topic_bridge.c
    src/core/json_extract.c
    src/core/json_schema.c
    src/core/node_plugin.c
)

# ── 算法库 (纯 C，无外部依赖) ─────────────────────────────────
set(ALGORITHM_SOURCES
    src/algorithms/ekf_fusion.c
    src/algorithms/dbscan_cluster.c
    src/algorithms/kalman_tracker.c
    src/algorithms/nuscenes_loader.c
    src/algorithms/nmea_parser.c
    src/algorithms/serial_port.c
)

# ── Optional: Protobuf support ────────────────────────────
find_package(ProtobufC QUIET)
if(ProtobufC_FOUND)
    message(STATUS "Protobuf-C found: enabling FLOWENGINE_USE_PROTOBUF")
    target_compile_definitions(flowengine_core PUBLIC FLOWENGINE_USE_PROTOBUF)
    target_link_libraries(flowengine_core PUBLIC ${ProtobufC_LIBRARIES})
    target_include_directories(flowengine_core PUBLIC ${ProtobufC_INCLUDE_DIRS})
else()
    message(STATUS "Protobuf-C not found — protobuf support disabled (install libprotobuf-c-dev)")
endif()

set(PLUGIN_SOURCES
    src/plugins/example_process.c
    src/plugins/example_task.c
)

# 创建核心静态库 (核心 + 算法)
add_library(flowengine_core STATIC ${CORE_SOURCES} ${ALGORITHM_SOURCES})
target_include_directories(flowengine_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/algorithms
)
# 依赖代码生成（msg_codegen 先完成再编译 core）
add_dependencies(flowengine_core msg_codegen)
target_link_libraries(flowengine_core
    Threads::Threads
    ${CMAKE_DL_LIBS}
    ${CJSON_LIBRARIES}
)
target_include_directories(flowengine_core PUBLIC
    ${CJSON_INCLUDE_DIRS}
    ${GEN_DIR}              # 代码生成的头文件
)
target_compile_options(flowengine_core PUBLIC ${CJSON_CFLAGS_OTHER})

# 创建插件动态库
add_library(example_process SHARED src/plugins/example_process.c)
target_link_libraries(example_process flowengine_core)

add_library(example_task SHARED src/plugins/example_task.c)
target_link_libraries(example_task flowengine_core)

# 简化的C++任务插件 (替代复杂版本)
add_library(cpp_example_task SHARED src/plugins/simple_cpp_task.cpp)
target_link_libraries(cpp_example_task flowengine_core)

# 监控守护进程 (独立进程, cyber_monitor 等价物)
add_executable(flowmond src/flowmond.c)
target_link_libraries(flowmond flowengine_core)

# 命令行工具
add_executable(flowctl src/flowctl.c)
target_link_libraries(flowctl flowengine_core scheduler_cpp)
add_dependencies(flowctl msg_codegen)

# 创建主启动器可执行文件
add_executable(launcher src/launcher.c)
target_link_libraries(launcher flowengine_core)

# 性能基准测试程序
add_executable(benchmark src/benchmark.c)
target_link_libraries(benchmark flowengine_core m)

# 消息总线可执行程序
add_executable(flow_bus src/bus_demo.c)
target_link_libraries(flow_bus flowengine_core m)

# ── Step 1: IPC 跨进程通道 ───────────────────────────────
add_executable(flow_ipc src/ipc_demo.c)
target_link_libraries(flow_ipc flowengine_core rt)

# ── Step 3: Bag 录制回放（含 Step 5 统一时钟）────────────
add_executable(flow_bag src/bag_demo.c)
target_link_libraries(flow_bag flowengine_core m)
target_link_options(flow_bag PRIVATE -Wl,--unresolved-symbols=ignore-in-object-files)

# ── Step 3b: Bag 检查工具 ────────────────────────────────
add_executable(bag_check tools/bag_check.c)
target_link_libraries(bag_check flowengine_core m)

# ── Step 4: 消息驱动任务插件 ─────────────────────────────
add_library(reactive_task SHARED src/plugins/reactive_task.c)
target_link_libraries(reactive_task flowengine_core)

# ── Algorithm plugins (real implementations) ──

# ── Frenet Optimal Trajectory Planner (from open-source, Apache-2.0) ──
# Source: https://github.com/fangedward/frenet-optimal-trajectory-planner
# Eigen is fetched via CMake FetchContent (no local dependency)
if(FLOWENGINE_HAVE_EIGEN)
set(FRENET_PLANNER_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/frenet_planner/src")
set(FRENET_PLANNER_SOURCES
    ${FRENET_PLANNER_DIR}/CubicSpline/CubicSpline1D.cpp
    ${FRENET_PLANNER_DIR}/CubicSpline/CubicSpline2D.cpp
    ${FRENET_PLANNER_DIR}/FrenetOptimalTrajectory/FrenetPath.cpp
    ${FRENET_PLANNER_DIR}/FrenetOptimalTrajectory/FrenetOptimalTrajectory.cpp
    ${FRENET_PLANNER_DIR}/FrenetOptimalTrajectory/fot_wrapper.cpp
    ${FRENET_PLANNER_DIR}/Polynomials/QuinticPolynomial.cpp
    ${FRENET_PLANNER_DIR}/Polynomials/QuarticPolynomial.cpp
    ${FRENET_PLANNER_DIR}/Obstacle/Obstacle.cpp
    ${FRENET_PLANNER_DIR}/Car/Car.cpp
)

add_library(frenet_planner STATIC ${FRENET_PLANNER_SOURCES})
target_include_directories(frenet_planner PUBLIC
    ${FRENET_PLANNER_DIR}
    ${FRENET_PLANNER_DIR}/CubicSpline
    ${FRENET_PLANNER_DIR}/FrenetOptimalTrajectory
    ${FRENET_PLANNER_DIR}/Polynomials
    ${FRENET_PLANNER_DIR}/Obstacle
    ${FRENET_PLANNER_DIR}/Car
)
target_compile_options(frenet_planner PRIVATE -Wno-unused-variable -Wno-sign-compare)
target_link_libraries(frenet_planner PUBLIC Eigen3::Eigen m)

# ── Frenet C Bridge (C++ → C wrapper for e2e demo) ──
add_library(frenet_bridge STATIC src/algorithms/frenet_bridge.cpp)
target_include_directories(frenet_bridge PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/algorithms
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/frenet_planner/src
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/frenet_planner/src/FrenetOptimalTrajectory
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/frenet_planner/src/CubicSpline
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/frenet_planner/src/Polynomials
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/frenet_planner/src/Obstacle
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/frenet_planner/src/Car
)
target_link_libraries(frenet_bridge PUBLIC frenet_planner Eigen3::Eigen m)
target_compile_options(frenet_bridge PRIVATE -Wno-unused-variable -Wno-sign-compare)
endif() # FLOWENGINE_HAVE_EIGEN

# =============================================================================
# flowcoro 协程集成目标（统一管理）
#
# 所有协程相关目标均链接 flowcoro_headers 并定义 FLOWCORO_INTEGRATION，
# 确保编译开关与头文件路径一致。
# =============================================================================

# ── Phase 2: C++ 调度器 (M:N coroutine scheduling) ────────
add_library(scheduler_cpp STATIC src/cpp/scheduler_cpp.cpp src/cpp/fusion_cpp.cpp src/cpp/network_transport.cpp)
target_link_libraries(scheduler_cpp flowengine_core flowcoro_headers)
target_include_directories(scheduler_cpp PUBLIC ${GEN_DIR})
target_compile_options(scheduler_cpp PRIVATE
    $<$<AND:$<COMPILE_LANGUAGE:CXX>,$<CXX_COMPILER_ID:GNU>>:-fcoroutines>
)

# ── 协程任务插件（动态库，供 launcher 动态加载）────────────
add_library(flowcoro_task SHARED src/plugins/flowcoro_task.cpp)
target_link_libraries(flowcoro_task flowengine_core flowcoro_headers flowcoro_net scheduler_cpp)
target_compile_definitions(flowcoro_task PRIVATE FLOWCORO_INTEGRATION)

# ── 协程 + 消息总线综合演示程序 ───────────────────────────
add_executable(flow_coro src/coro_bus_demo.cpp)
target_link_libraries(flow_coro flowengine_core flowcoro_headers flowcoro_net scheduler_cpp)
target_compile_definitions(flow_coro PRIVATE FLOWCORO_INTEGRATION)

# ── 协程路径性能基准测试程序 ──────────────────────────────
add_executable(benchmark_coro src/benchmark_coro.cpp)
target_link_libraries(benchmark_coro flowengine_core flowcoro_headers flowcoro_net m)
target_compile_definitions(benchmark_coro PRIVATE FLOWCORO_INTEGRATION)

# ── 协程集成层正确性测试程序 ──────────────────────────────
add_executable(coro_correctness_test test/coro_correctness_test.cpp)
target_link_libraries(coro_correctness_test flowengine_core flowcoro_headers flowcoro_net)
target_compile_definitions(coro_correctness_test PRIVATE FLOWCORO_INTEGRATION)

# ── Config-driven Launcher ──
# 读 config/pipeline.json → 按顺序 fork+exec 各节点进程
# 是 e2e_demo --multi 的解耦升级版: 编排逻辑完全在配置文件里
add_executable(flow_launcher src/flow_launcher.c)
# --whole-archive: 强制拉入 cJSON 全部符号（否则链接器丢弃未直接引用的 .o）
# scheduler_cpp: 提供 net_transport_unbridge_topic 等 C++ 符号
target_link_libraries(flow_launcher
    flowengine_core scheduler_cpp m dl
    -Wl,--whole-archive ${CJSON_LIBRARIES} -Wl,--no-whole-archive)
# --export-dynamic: 让主程序导出 cJSON 等符号，使 dlopen 加载的 .so 能解析它们
target_link_options(flow_launcher PRIVATE
    -Wl,--export-dynamic
    -Wl,--unresolved-symbols=ignore-in-object-files)

# ── Node Host: 单节点插件宿主进程 ──
# 多进程模式的正确载体: 每个进程 dlopen 一个 NodePlugin .so, 走 IPC 桥接。
# 让「进程模式」与「线程/dlopen 模式」复用同一份节点 .so。
add_executable(flow_node_host src/flow_node_host.c)
target_link_libraries(flow_node_host flowengine_core m dl)
target_link_options(flow_node_host PRIVATE -Wl,--unresolved-symbols=ignore-in-object-files)

# ── Node Plugins 有独立 cmake 子项目: modules/adas_nodes/ ──
# 节点不依赖主项目的 FetchContent 依赖，只 link 已编译的 libflowengine_core.a。
# 构建方式: cmake -B build/modules/adas_nodes -S modules/adas_nodes && cmake --build build/modules/adas_nodes
# 参见 modules/adas_nodes/CMakeLists.txt

# ── MCAP Replay Tool ──
add_executable(flow_mcap_replay src/mcap_replay.c)
if(FLOWENGINE_HAVE_EIGEN)
target_link_libraries(flow_mcap_replay flowengine_core scheduler_cpp frenet_bridge frenet_planner m)
else()
target_link_libraries(flow_mcap_replay flowengine_core scheduler_cpp m)
target_compile_definitions(flow_mcap_replay PRIVATE NO_FRENET)
endif()
set_target_properties(flow_mcap_replay PROPERTIES LINKER_LANGUAGE CXX)

# ── nuScenes Demo ──
add_executable(flow_nuscenes_demo src/nuscenes_demo.c)
target_link_libraries(flow_nuscenes_demo flowengine_core m)
# 安装规则
install(TARGETS launcher flowmond flowctl flow_bus flow_ipc flow_bag bag_check flow_coro flow_mcap_replay flow_nuscenes_demo benchmark benchmark_coro coro_correctness_test
    RUNTIME DESTINATION bin
)

install(TARGETS flowengine_core
    ARCHIVE DESTINATION lib
)

install(TARGETS example_process example_task cpp_example_task reactive_task flowcoro_task
    LIBRARY DESTINATION lib/flowengine/plugins
)

install(DIRECTORY include/
    DESTINATION include/flowengine
    FILES_MATCHING PATTERN "*.h"
)

# ── pkg-config ──────────────────────────────────────────────
configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/flowengine.pc.in"
    "${CMAKE_CURRENT_BINARY_DIR}/flowengine.pc"
    @ONLY
)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/flowengine.pc"
    DESTINATION lib/pkgconfig
)

# ── Scripts ─────────────────────────────────────────────────
install(PROGRAMS
    scripts/demo.sh
    scripts/stress_test.sh
    tools/flowboard_server.py
    DESTINATION share/flowengine/scripts
)

# 创建配置文件
configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/config.json.in"
    "${CMAKE_BINARY_DIR}/config/example.json"
    @ONLY
)

# 自定义目标：创建配置目录和文件
add_custom_target(config
    COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_BINARY_DIR}/config
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${CMAKE_CURRENT_SOURCE_DIR}/cmake/config.json.in
        ${CMAKE_BINARY_DIR}/config/example.json
    COMMENT "Creating configuration files"
)

# 自定义目标：运行启动器
add_custom_target(run_launcher
    COMMAND ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/launcher config/example.json
    DEPENDS launcher config
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    COMMENT "Running launcher with example config"
)

# 测试支持
enable_testing()

# 简单的测试
add_test(NAME flow_bus_test
    COMMAND ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/flow_bus
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)
set_tests_properties(flow_bus_test PROPERTIES TIMEOUT 10)

add_test(NAME flow_coro_test
    COMMAND ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/flow_coro
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)
set_tests_properties(flow_coro_test PROPERTIES TIMEOUT 10)

add_test(NAME flow_coro_bench
    COMMAND ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/benchmark_coro
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)
set_tests_properties(flow_coro_bench PROPERTIES TIMEOUT 300 LABELS "benchmark;manual")

add_test(NAME coro_correctness_test
    COMMAND ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/coro_correctness_test
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)
set_tests_properties(coro_correctness_test PROPERTIES TIMEOUT 60)

# ── Unit tests ────────────────────────────────────────────
# Rename to .cpp for C++ compilation (uses lambdas in tests)
add_executable(test_modules tests/test_modules.c)
set_source_files_properties(tests/test_modules.c PROPERTIES LANGUAGE CXX)
target_link_libraries(test_modules flowengine_core scheduler_cpp)
add_dependencies(test_modules msg_codegen)
target_include_directories(test_modules PRIVATE ${GEN_DIR})

add_test(NAME unit_tests COMMAND test_modules)
set_tests_properties(unit_tests PROPERTIES TIMEOUT 10)

# ── 新增模块单元测试（Task Manager / IPC / Discovery / Bag / FlowRegistry）──
add_executable(test_new_modules tests/test_new_modules.c)
target_link_libraries(test_new_modules flowengine_core)
add_dependencies(test_new_modules msg_codegen)
target_include_directories(test_new_modules PRIVATE ${GEN_DIR})

add_test(NAME new_module_tests COMMAND test_new_modules)
set_tests_properties(new_module_tests PROPERTIES TIMEOUT 10)

# ── 传输/可视化桥接栈单元测试（transport / stats_bridge / dashboard_bridge）──
add_executable(test_bridges tests/test_bridges.c)
target_link_libraries(test_bridges flowengine_core scheduler_cpp)
add_dependencies(test_bridges msg_codegen)
target_include_directories(test_bridges PRIVATE ${GEN_DIR})

add_test(NAME bridge_tests COMMAND test_bridges)
set_tests_properties(bridge_tests PROPERTIES TIMEOUT 30)

# ── JSON Schema 严格校验单元测试 ──
add_executable(test_json_schema tests/test_json_schema.c)
target_link_libraries(test_json_schema flowengine_core)

add_test(NAME json_schema_tests COMMAND test_json_schema)
set_tests_properties(json_schema_tests PROPERTIES TIMEOUT 10)

add_test(NAME e2e_training_tools
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_e2e_training_tools.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)
set_tests_properties(e2e_training_tools PROPERTIES TIMEOUT 30)

add_test(NAME demo_evaluator_tools
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_demo_evaluator.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)
set_tests_properties(demo_evaluator_tools PROPERTIES TIMEOUT 10)

# ── Smoke test: flow_launcher short integration run ──
add_test(NAME e2e_smoke
    COMMAND ${CMAKE_COMMAND} -E env BUILD_DIR=${CMAKE_BINARY_DIR}
            bash ${CMAKE_CURRENT_SOURCE_DIR}/scripts/launcher_smoke.sh 3
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)
set_tests_properties(e2e_smoke PROPERTIES TIMEOUT 150 LABELS "integration")

# ── Stability test: long-running pipeline (10 min, optional/manual) ──
add_test(NAME e2e_stability
    COMMAND ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/flow_launcher
            ${CMAKE_CURRENT_SOURCE_DIR}/config/pipeline.json --duration 600
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)
set_tests_properties(e2e_stability PROPERTIES TIMEOUT 660 LABELS "stability;manual;integration")

# ── Integration smoke: flow_launcher loads all node plugins & runs pipeline ──
# Labeled "integration" so the standard build jobs (which don't build the
# separate adas_nodes .so subproject) skip it; a dedicated CI job runs it.
add_test(NAME flow_launcher_smoke
    COMMAND ${CMAKE_COMMAND} -E env BUILD_DIR=${CMAKE_BINARY_DIR}
            bash ${CMAKE_CURRENT_SOURCE_DIR}/scripts/launcher_smoke.sh 3
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)
set_tests_properties(flow_launcher_smoke PROPERTIES TIMEOUT 150 LABELS "integration")

# 打印构建信息
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "C flags: ${CMAKE_C_FLAGS}")
message(STATUS "Install prefix: ${CMAKE_INSTALL_PREFIX}")
message(STATUS "flowcoro source dir: ${flowcoro_SOURCE_DIR}")

# 创建开发者友好的别名
add_custom_target(dev DEPENDS launcher flow_bus flow_coro benchmark benchmark_coro coro_correctness_test example_process example_task flowcoro_task config)

# CPack配置（用于创建安装包）
set(CPACK_PACKAGE_NAME "FlowEngine")
set(CPACK_PACKAGE_VERSION_MAJOR ${PROJECT_VERSION_MAJOR})
set(CPACK_PACKAGE_VERSION_MINOR ${PROJECT_VERSION_MINOR})
set(CPACK_PACKAGE_VERSION_PATCH ${PROJECT_VERSION_PATCH})
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Lightweight middleware for autonomous driving & robotics — C kernel + C++20 coroutine shell")
set(CPACK_PACKAGE_VENDOR "FlowEngine")
set(CPACK_PACKAGE_CONTACT "https://github.com/caixuf/FlowEngine")

include(CPack)
