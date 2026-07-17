#!/bin/bash

# CMake构建脚本
# 用法: ./build.sh [clean|debug|release|install|test|run]

BUILD_DIR="build"
INSTALL_PREFIX="/usr/local"

# 颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 打印彩色消息
print_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# 清理构建目录
clean_build() {
    print_info "清理构建目录..."
    rm -rf $BUILD_DIR
    print_success "构建目录已清理"
}

# 创建构建目录
create_build_dir() {
    if [ ! -d "$BUILD_DIR" ]; then
        print_info "创建构建目录: $BUILD_DIR"
        mkdir -p $BUILD_DIR
    fi
}

# 配置和构建项目
build_project() {
    local build_type=$1
    
    print_info "配置CMake项目 (构建类型: $build_type)..."
    
    create_build_dir
    cd $BUILD_DIR
    
    cmake .. \
        -DCMAKE_BUILD_TYPE=$build_type \
        -DCMAKE_INSTALL_PREFIX=$INSTALL_PREFIX \
        -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
        -DCMAKE_C_COMPILER=gcc
    
    if [ $? -ne 0 ]; then
        print_error "CMake配置失败"
        exit 1
    fi
    
    print_info "构建项目..."
    local warn_log
    warn_log="$(mktemp)"
    make -j$(nproc) 2> >(grep -i "warning:" > "$warn_log" || true)
    local make_rc=$?

    if [ $make_rc -ne 0 ]; then
        print_error "构建失败"
        rm -f "$warn_log"
        exit 1
    fi

    local warn_count
    warn_count=$(wc -l < "$warn_log" 2>/dev/null || echo 0)
    if [ "$warn_count" -gt 0 ]; then
        print_warning "发现 $warn_count 个编译警告:"
        while IFS= read -r line; do
            echo "  ${YELLOW}⚠${NC} $line"
        done < "$warn_log"
    fi
    rm -f "$warn_log"

    cd ..
    print_success "构建完成 ($warn_count 个警告)"
}

# 安装项目
install_project() {
    if [ ! -d "$BUILD_DIR" ]; then
        print_error "构建目录不存在，请先构建项目"
        exit 1
    fi
    
    print_info "安装项目到 $INSTALL_PREFIX..."
    cd $BUILD_DIR
    sudo make install
    cd ..
    print_success "安装完成"
}

# 运行测试
run_tests() {
    if [ ! -d "$BUILD_DIR" ]; then
        print_error "构建目录不存在，请先构建项目"
        exit 1
    fi
    
    print_info "运行测试..."
    cd $BUILD_DIR
    ctest --verbose
    cd ..
}

# 运行性能基准测试
run_benchmark() {
    if [ ! -d "$BUILD_DIR" ]; then
        print_error "构建目录不存在，请先构建项目"
        exit 1
    fi

    print_info "运行性能基准测试..."
    ${BUILD_DIR}/bin/benchmark
}

# 运行演示程序
run_demo() {
    if [ ! -d "$BUILD_DIR" ]; then
        print_error "构建目录不存在，请先构建项目"
        exit 1
    fi
    
    print_info "运行任务演示程序..."
    cd $BUILD_DIR
    make run_demo
    cd ..
}

# 运行启动器
run_launcher() {
    if [ ! -d "$BUILD_DIR" ]; then
        print_error "构建目录不存在，请先构建项目"
        exit 1
    fi
    
    print_info "运行启动器..."
    cd $BUILD_DIR
    make run_launcher
    cd ..
}

# 显示帮助信息
show_help() {
    echo "FlowEngine CMake构建脚本"
    echo ""
    echo "用法: $0 [选项]"
    echo ""
    echo "选项:"
    echo "  clean      - 清理构建目录"
    echo "  debug      - 构建Debug版本"
    echo "  release    - 构建Release版本 (默认)"
    echo "  install    - 安装到系统"
    echo "  test       - 运行测试"
    echo "  bench      - 运行性能基准测试"
  echo "  demo       - 运行任务演示"
    echo "  launcher   - 运行启动器"
    echo "  help       - 显示此帮助信息"
    echo ""
    echo "示例:"
    echo "  $0 debug      # 构建Debug版本"
    echo "  $0 release    # 构建Release版本"
    echo "  $0 clean debug # 清理后构建Debug版本"
}

# 检查依赖
check_dependencies() {
    print_info "检查依赖..."
    
    # 检查CMake
    if ! command -v cmake &> /dev/null; then
        print_error "CMake未安装，请先安装CMake"
        exit 1
    fi
    
    # 检查Make
    if ! command -v make &> /dev/null; then
        print_error "Make未安装，请先安装Make"
        exit 1
    fi
    
    # 检查pkg-config
    if ! command -v pkg-config &> /dev/null; then
        print_error "pkg-config未安装，请先安装pkg-config"
        exit 1
    fi
    
    # 检查libcjson
    if ! pkg-config --exists libcjson; then
        print_error "libcjson未安装，请先安装libcjson-dev"
        exit 1
    fi

    # 检查Eigen3（可选，但缺失时 planning 节点会静默退化为"只保持车道、永不变道/
    # 超车"的兜底模式。不作为硬性失败，只提醒用户。）
    # 注：这里仅探测常见安装路径（pkg-config / apt 默认头文件位置），无法感知
    # 通过 CMAKE_PREFIX_PATH / EIGEN3_ROOT 等自定义路径安装的 Eigen3，因此在非
    # 标准安装场景下可能出现误报警告；实际是否启用 Frenet 以 CMake 配置期的
    # find_package(Eigen3) 结果（见 CMakeLists.txt）为准。
    if ! pkg-config --exists eigen3 2>/dev/null && [ ! -f /usr/include/eigen3/Eigen/Core ] \
        && [ ! -f /usr/local/include/eigen3/Eigen/Core ]; then
        print_warning "libeigen3未安装：Frenet 规划器将不会被编译，车辆将无法变道/超车（仅保持车道）。"
        print_warning "建议安装: sudo apt install libeigen3-dev（若已通过自定义路径安装 Eigen3，可忽略此提示）"
    fi

    print_success "所有依赖都已满足"
}

# 主程序
main() {
    case $1 in
        clean)
            clean_build
            ;;
        debug)
            check_dependencies
            build_project "Debug"
            ;;
        release)
            check_dependencies
            build_project "Release"
            ;;
        install)
            install_project
            ;;
        test)
            run_tests
            ;;
        bench)
            run_benchmark
            ;;
        demo)
            run_demo
            ;;
        launcher)
            run_launcher
            ;;
        help|--help|-h)
            show_help
            ;;
        "")
            # 默认构建Release版本
            check_dependencies
            build_project "Release"
            ;;
        *)
            print_error "未知选项: $1"
            show_help
            exit 1
            ;;
    esac
}

# 处理多个参数
for arg in "$@"; do
    main $arg
done

if [ $# -eq 0 ]; then
    main
fi
