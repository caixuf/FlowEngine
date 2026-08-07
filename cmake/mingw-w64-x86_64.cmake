# cmake/mingw-w64-x86_64.cmake
# CMake toolchain file for cross-compiling to Windows x86_64 using mingw-w64.
# Used by the CI job "build-windows-mingw" on ubuntu-24.04.

set(CMAKE_SYSTEM_NAME Windows)
set(CMAKE_SYSTEM_PROCESSOR x86_64)

# Prefer posix-thread variant (supports C++11 threads, std::mutex, etc.)
find_program(MINGW_CC
    NAMES x86_64-w64-mingw32-gcc-posix x86_64-w64-mingw32-gcc)
find_program(MINGW_CXX
    NAMES x86_64-w64-mingw32-g++-posix x86_64-w64-mingw32-g++)
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
