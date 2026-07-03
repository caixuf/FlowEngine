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

# 查找cjson库
pkg_check_modules(CJSON REQUIRED libcjson)

# =============================================================================
# Phase 1: IDL 代码生成 (msg/*.msg -> 序列化头文件)
# =============================================================================
set(MSG_CODEGEN "${CMAKE_SOURCE_DIR}/tools/msg_codegen.py")
set(GEN_DIR     "${CMAKE_BINARY_DIR}/gen")

# 主 IDL 文件
set(MSG_FILES
    msg/adas_msgs.msg
)

# 为每个 .msg 生成对应的 _gen.h
foreach(MSG_FILE ${MSG_FILES})
    get_filename_component(MSG_NAME ${MSG_FILE} NAME_WE)
    set(GEN_HEADER "${GEN_DIR}/${MSG_NAME}_gen.h")
    add_custom_command(
        OUTPUT ${GEN_HEADER}
        COMMAND ${Python3_EXECUTABLE} ${MSG_CODEGEN} "${CMAKE_SOURCE_DIR}/${MSG_FILE}" ${GEN_HEADER}
        DEPENDS ${MSG_CODEGEN} ${CMAKE_SOURCE_DIR}/${MSG_FILE}
        COMMENT "Generating serialization code from ${MSG_FILE}"
    )
    list(APPEND GEN_HEADERS ${GEN_HEADER})
endforeach()

# 便捷目标：只生成代码
add_custom_target(msg_codegen DEPENDS ${GEN_HEADERS})

# =============================================================================
# flowcoro — C++20 高性能协程库 (https://github.com/caixuf/flowcoro)
# =============================================================================
include(FetchContent)

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
    GIT_TAG        main
    GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(flowcoro)

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
    $<$<CXX_COMPILER_ID:GNU>:-fcoroutines>
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
)

# ── Optional: Protobuf support ────────────────────────────
find_package(ProtobufC QUIET)
if(ProtobufC_FOUND)
    message(STATUS "Protobuf-C found: enabling FLOWENGINE_USE_PROTOBUF")
    target_compile_definitions(starttool_core PUBLIC FLOWENGINE_USE_PROTOBUF)
    target_link_libraries(starttool_core PUBLIC ${ProtobufC_LIBRARIES})
    target_include_directories(starttool_core PUBLIC ${ProtobufC_INCLUDE_DIRS})
else()
    message(STATUS "Protobuf-C not found — protobuf support disabled (install libprotobuf-c-dev)")
endif()

set(PLUGIN_SOURCES
    src/plugins/example_process.c
    src/plugins/example_task.c
)

# 创建核心静态库
add_library(starttool_core STATIC ${CORE_SOURCES})
# 依赖代码生成（msg_codegen 先完成再编译 core）
add_dependencies(starttool_core msg_codegen)
target_link_libraries(starttool_core
    Threads::Threads
    ${CMAKE_DL_LIBS}
    ${CJSON_LIBRARIES}
)
target_include_directories(starttool_core PUBLIC
    ${CJSON_INCLUDE_DIRS}
    ${GEN_DIR}              # 代码生成的头文件
)
target_compile_options(starttool_core PUBLIC ${CJSON_CFLAGS_OTHER})

# 创建插件动态库
add_library(example_process SHARED src/plugins/example_process.c)
target_link_libraries(example_process starttool_core)

add_library(example_task SHARED src/plugins/example_task.c)
target_link_libraries(example_task starttool_core)

# 简化的C++任务插件 (替代复杂版本)
add_library(cpp_example_task SHARED src/plugins/simple_cpp_task.cpp)
target_link_libraries(cpp_example_task starttool_core)

# 网络服务任务 (C++)
add_library(network_service_task SHARED src/plugins/network_service_task.cpp)
target_link_libraries(network_service_task starttool_core)

# 数据处理任务 (C++)
add_library(data_processor_task SHARED src/plugins/data_processor_task.cpp)
target_link_libraries(data_processor_task starttool_core)

# 创建主启动器可执行文件
add_executable(launcher src/launcher.c)
target_link_libraries(launcher starttool_core)

# 性能基准测试程序
add_executable(benchmark src/benchmark.c)
target_link_libraries(benchmark starttool_core m)

# 任务系统可执行程序
add_executable(flow_task src/task_demo.c)
target_link_libraries(flow_task starttool_core example_task)

# 消息总线可执行程序
add_executable(flow_bus src/bus_demo.c)
target_link_libraries(flow_bus starttool_core m)

# C++ 任务可执行程序
add_executable(flow_cpp_task src/cpp_task_demo.cpp)
target_link_libraries(flow_cpp_task starttool_core cpp_example_task)

# 简化 C++ 可执行程序
add_executable(flow_cpp src/simple_cpp_demo.cpp)
target_link_libraries(flow_cpp starttool_core cpp_example_task)

# ── Step 1: IPC 跨进程通道 ───────────────────────────────
add_executable(flow_ipc src/ipc_demo.c)
target_link_libraries(flow_ipc starttool_core rt)

# ── Step 3: Bag 录制回放（含 Step 5 统一时钟）────────────
add_executable(flow_bag src/bag_demo.c)
target_link_libraries(flow_bag starttool_core m)

# ── Step 4: 消息驱动任务插件 ─────────────────────────────
add_library(reactive_task SHARED src/plugins/reactive_task.c)
target_link_libraries(reactive_task starttool_core)

# =============================================================================
# flowcoro 协程集成目标（统一管理）
#
# 所有协程相关目标均链接 flowcoro_headers 并定义 FLOWCORO_INTEGRATION，
# 确保编译开关与头文件路径一致。
# =============================================================================

# ── Phase 2: C++ 调度器 (M:N coroutine scheduling) ────────
add_library(scheduler_cpp STATIC src/cpp/scheduler_cpp.cpp src/cpp/fusion_cpp.cpp src/cpp/network_transport.cpp)
target_link_libraries(scheduler_cpp starttool_core flowcoro_headers)
target_include_directories(scheduler_cpp PUBLIC ${GEN_DIR})
target_compile_options(scheduler_cpp PRIVATE
    $<$<CXX_COMPILER_ID:GNU>:-fcoroutines>
)

# ── 协程任务插件（动态库，供 launcher 动态加载）────────────
add_library(flowcoro_task SHARED src/plugins/flowcoro_task.cpp)
target_link_libraries(flowcoro_task starttool_core flowcoro_headers flowcoro_net scheduler_cpp)
target_compile_definitions(flowcoro_task PRIVATE FLOWCORO_INTEGRATION)

# ── 协程 + 消息总线综合演示程序 ───────────────────────────
add_executable(flow_coro src/coro_bus_demo.cpp)
target_link_libraries(flow_coro starttool_core flowcoro_headers flowcoro_net scheduler_cpp)
target_compile_definitions(flow_coro PRIVATE FLOWCORO_INTEGRATION)

# ── 协程路径性能基准测试程序 ──────────────────────────────
add_executable(benchmark_coro src/benchmark_coro.cpp)
target_link_libraries(benchmark_coro starttool_core flowcoro_headers flowcoro_net m)
target_compile_definitions(benchmark_coro PRIVATE FLOWCORO_INTEGRATION)

# ── 协程集成层正确性测试程序 ──────────────────────────────
add_executable(coro_correctness_test test/coro_correctness_test.cpp)
target_link_libraries(coro_correctness_test starttool_core flowcoro_headers flowcoro_net)
target_compile_definitions(coro_correctness_test PRIVATE FLOWCORO_INTEGRATION)

# ── ADAS: 感知节点 + 控制节点全链路仿真 ──────────────────
add_library(fake_perception_task SHARED src/plugins/fake_perception_task.c)
target_link_libraries(fake_perception_task starttool_core)

add_library(fake_control_task SHARED src/plugins/fake_control_task.c)
target_link_libraries(fake_control_task starttool_core m)

add_executable(flow_adas src/adas_demo.c)
target_link_libraries(flow_adas starttool_core fake_perception_task fake_control_task)

# ── Full-stack multi-process demo (Phase 3+4 integration) ──
add_executable(flow_fullstack src/fullstack_demo.c)
target_link_libraries(flow_fullstack starttool_core scheduler_cpp)
# Depends on codegen for ADAS type registration
add_dependencies(flow_fullstack msg_codegen)

# 安装规则
install(TARGETS launcher flow_task flow_cpp flow_bus flow_ipc flow_bag flow_cpp_task flow_coro flow_adas flow_fullstack benchmark benchmark_coro coro_correctness_test
    RUNTIME DESTINATION bin
)

install(TARGETS starttool_core
    ARCHIVE DESTINATION lib
)

install(TARGETS example_process example_task cpp_example_task reactive_task flowcoro_task network_service_task data_processor_task fake_perception_task fake_control_task
    LIBRARY DESTINATION lib/starttool/plugins
)

install(DIRECTORY include/
    DESTINATION include/starttool
    FILES_MATCHING PATTERN "*.h"
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

# 自定义目标：运行演示
add_custom_target(run_demo
    COMMAND ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/flow_task
    DEPENDS flow_task config
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    COMMENT "Running flow_task"
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
add_test(NAME flow_task_test
    COMMAND ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/flow_task
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)

add_test(NAME flow_bus_test
    COMMAND ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/flow_bus
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)

add_test(NAME flow_coro_test
    COMMAND ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/flow_coro
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)

add_test(NAME flow_coro_bench
    COMMAND ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/benchmark_coro
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)
set_tests_properties(flow_coro_bench PROPERTIES TIMEOUT 120)

add_test(NAME coro_correctness_test
    COMMAND ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/coro_correctness_test
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)
set_tests_properties(coro_correctness_test PROPERTIES TIMEOUT 60)

# ── Unit tests ────────────────────────────────────────────
# Rename to .cpp for C++ compilation (uses lambdas in tests)
add_executable(test_modules tests/test_modules.c)
set_source_files_properties(tests/test_modules.c PROPERTIES LANGUAGE CXX)
target_link_libraries(test_modules starttool_core scheduler_cpp)
add_dependencies(test_modules msg_codegen)
target_include_directories(test_modules PRIVATE ${GEN_DIR})

add_test(NAME unit_tests COMMAND test_modules)

add_test(NAME flow_adas_test
    COMMAND ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/flow_adas
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)

# 打印构建信息
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "C flags: ${CMAKE_C_FLAGS}")
message(STATUS "Install prefix: ${CMAKE_INSTALL_PREFIX}")
message(STATUS "flowcoro source dir: ${flowcoro_SOURCE_DIR}")

# 创建开发者友好的别名
add_custom_target(dev DEPENDS launcher flow_task flow_bus flow_coro flow_adas benchmark benchmark_coro coro_correctness_test example_process example_task flowcoro_task fake_perception_task fake_control_task config)

# CPack配置（用于创建安装包）
set(CPACK_PACKAGE_NAME "FlowEngine")
set(CPACK_PACKAGE_VERSION_MAJOR ${PROJECT_VERSION_MAJOR})
set(CPACK_PACKAGE_VERSION_MINOR ${PROJECT_VERSION_MINOR})
set(CPACK_PACKAGE_VERSION_PATCH ${PROJECT_VERSION_PATCH})
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "进程启动器和任务管理系统")
set(CPACK_PACKAGE_VENDOR "Your Company")
set(CPACK_PACKAGE_CONTACT "your.email@company.com")

include(CPack)
