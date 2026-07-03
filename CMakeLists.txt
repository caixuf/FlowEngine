cmake_minimum_required(VERSION 3.10)
project(StartTool VERSION 1.0.0 LANGUAGES C CXX)

# 设置C和C++标准
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 设置编译选项
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -fPIC")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -fPIC")
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

# 查找cjson库
pkg_check_modules(CJSON REQUIRED libcjson)

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
)

set(PLUGIN_SOURCES
    src/plugins/example_process.c
    src/plugins/example_task.c
)

# 创建核心静态库
add_library(starttool_core STATIC ${CORE_SOURCES})
target_link_libraries(starttool_core 
    Threads::Threads
    ${CMAKE_DL_LIBS}
    ${CJSON_LIBRARIES}
)
target_include_directories(starttool_core PUBLIC ${CJSON_INCLUDE_DIRS})
target_compile_options(starttool_core PUBLIC ${CJSON_CFLAGS_OTHER})

# 创建插件动态库
add_library(example_process SHARED src/plugins/example_process.c)
target_link_libraries(example_process starttool_core)

add_library(example_task SHARED src/plugins/example_task.c)
target_link_libraries(example_task starttool_core)

# 简化的C++任务插件 (替代复杂版本)
add_library(cpp_example_task SHARED src/plugins/simple_cpp_task.cpp)
target_link_libraries(cpp_example_task starttool_core)

# 网络服务任务 (C++) - 暂时注释掉
#add_library(network_service_task SHARED src/plugins/network_service_task.cpp)
#target_link_libraries(network_service_task starttool_core)

# 数据处理任务 (C++) - 暂时注释掉
#add_library(data_processor_task SHARED src/plugins/data_processor_task.cpp)
#target_link_libraries(data_processor_task starttool_core)

# 创建主启动器可执行文件
add_executable(launcher src/launcher.c)
target_link_libraries(launcher starttool_core)

# 创建任务演示程序
add_executable(task_demo src/task_demo.c)
target_link_libraries(task_demo starttool_core example_task)

# 创建通信总线演示程序
add_executable(bus_demo src/bus_demo.c)
target_link_libraries(bus_demo starttool_core m)

# C++任务演示程序 (有编译问题，暂时注释)
#add_executable(cpp_task_demo src/cpp_task_demo.cpp)
#target_link_libraries(cpp_task_demo starttool_core)

# 简化的C++任务演示程序
add_executable(simple_cpp_demo src/simple_cpp_demo.cpp)
target_link_libraries(simple_cpp_demo starttool_core cpp_example_task)

# ── Step 1: IPC 跨进程通道演示 ───────────────────────────
add_executable(ipc_demo src/ipc_demo.c)
target_link_libraries(ipc_demo starttool_core rt)

# ── Step 3: Bag 录制回放演示（含 Step 5 统一时钟）────────
add_executable(bag_demo src/bag_demo.c)
target_link_libraries(bag_demo starttool_core m)

# ── Step 4: 消息驱动任务插件 ─────────────────────────────
add_library(reactive_task SHARED src/plugins/reactive_task.c)
target_link_libraries(reactive_task starttool_core)

# 安装规则
install(TARGETS launcher task_demo simple_cpp_demo bus_demo ipc_demo bag_demo
    RUNTIME DESTINATION bin
)

install(TARGETS starttool_core
    ARCHIVE DESTINATION lib
)

install(TARGETS example_process example_task cpp_example_task reactive_task
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
    COMMAND ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/task_demo
    DEPENDS task_demo config
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    COMMENT "Running task demo"
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
add_test(NAME task_demo_test
    COMMAND ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/task_demo
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)

# 打印构建信息
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "C flags: ${CMAKE_C_FLAGS}")
message(STATUS "Install prefix: ${CMAKE_INSTALL_PREFIX}")

# 创建开发者友好的别名
add_custom_target(dev DEPENDS launcher task_demo bus_demo example_process example_task config)

# CPack配置（用于创建安装包）
set(CPACK_PACKAGE_NAME "StartTool")
set(CPACK_PACKAGE_VERSION_MAJOR ${PROJECT_VERSION_MAJOR})
set(CPACK_PACKAGE_VERSION_MINOR ${PROJECT_VERSION_MINOR})
set(CPACK_PACKAGE_VERSION_PATCH ${PROJECT_VERSION_PATCH})
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "进程启动器和任务管理系统")
set(CPACK_PACKAGE_VENDOR "Your Company")
set(CPACK_PACKAGE_CONTACT "your.email@company.com")

include(CPack)
