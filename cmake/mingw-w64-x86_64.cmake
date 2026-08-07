# cmake/mingw-w64-x86_64.cmake
# CMake toolchain file for cross-compiling to Windows x86_64 using mingw-w64.
# Used by the CI job "build-windows-mingw" on ubuntu-24.04.

set(CMAKE_SYSTEM_NAME Windows)
set(CMAKE_SYSTEM_PROCESSOR x86_64)

# 线程模型选 win32 变体：项目自带 compat_win/pthread.h（SRWLock/cond-var 的
# pthread 兼容层），只能与 win32 线程模型配对。若用 posix 变体 (g++-posix)，
# libstdc++ 的 gthr-posix.h 会 #include <pthread.h>，而 compat_win/pthread.h 遮蔽
# 了 winpthreads 头 → pthread_key_t/pthread_once_t/mutex_timedlock 未定义 → 编译崩。
# win32 变体的 libstdc++ 直接用 Win32 原语实现 std::thread/mutex/condvar，
# 不 include pthread.h，与 compat shim 互补。winpthreads 未参与构建。
find_program(MINGW_CC
    NAMES x86_64-w64-mingw32-gcc-win32 x86_64-w64-mingw32-gcc)
find_program(MINGW_CXX
    NAMES x86_64-w64-mingw32-g++-win32 x86_64-w64-mingw32-g++)
find_program(MINGW_RC  x86_64-w64-mingw32-windres)

if(NOT MINGW_CC)
    message(FATAL_ERROR
        "mingw-w64 C compiler not found. "
        "Install it with: sudo apt-get install gcc-mingw-w64-x86-64")
endif()
if(NOT MINGW_CXX)
    message(FATAL_ERROR
        "mingw-w64 C++ compiler not found. "
        "Install it with: sudo apt-get install g++-mingw-w64-x86-64")
endif()

set(CMAKE_C_COMPILER   "${MINGW_CC}")
set(CMAKE_CXX_COMPILER "${MINGW_CXX}")
if(MINGW_RC)
    set(CMAKE_RC_COMPILER "${MINGW_RC}")
endif()

# Sysroot / search paths
set(CMAKE_FIND_ROOT_PATH /usr/x86_64-w64-mingw32)
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

# Windows target — disable Linux-only features at configure time
set(FLOWENGINE_WINDOWS_MINGW ON CACHE BOOL "Cross-compiling for Windows with mingw-w64" FORCE)
