#!/bin/bash
# FlowEngine Algorithm Stack — Dependency Setup
# Installs: OpenCV, Eigen, OSQP (optional, user can skip any)

set -e
echo "╔══════════════════════════════════════════╗"
echo "║  Algorithm Stack Dependencies            ║"
echo "╚══════════════════════════════════════════╝"
echo ""

check_pkg() {
    if pkg-config --exists "$1" 2>/dev/null; then
        echo "  ✅ $1: $(pkg-config --modversion "$1" 2>/dev/null || echo 'installed')"
        return 0
    else
        echo "  ❌ $1: not found"
        return 1
    fi
}

install_if_missing() {
    local pkg="$1" apt_name="$2"
    if ! check_pkg "$pkg"; then
        echo "     Installing $apt_name..."
        sudo apt-get install -y -qq "$apt_name" 2>/dev/null && echo "     ✅ installed" || echo "     ⚠ failed (try manually: sudo apt install $apt_name)"
    fi
}

echo "Detection:"
echo ""

# OpenCV
echo "  OpenCV (Perception):"
install_if_missing "opencv4" "libopencv-dev"

# Eigen
echo "  Eigen (Fusion):"
if [ -d /usr/include/eigen3 ] || [ -d /usr/local/include/eigen3 ]; then
    VER=$(grep "EIGEN_.*_VERSION" /usr/include/eigen3/Eigen/src/Core/util/Macros.h 2>/dev/null | head -3 | tr '\n' ' ')
    echo "  ✅ eigen3: header-only (no linking needed)"
else
    echo "  ❌ eigen3: not found"
    echo "     Installing libeigen3-dev..."
    sudo apt-get install -y -qq libeigen3-dev 2>/dev/null
fi

# OSQP
echo "  OSQP (MPC Control):"
install_if_missing "osqp" "libosqp-dev"

echo ""
echo "Summary:"
echo "  Perception:  OpenCV DNN (YOLO/SSD) → camera → objects"
echo "  Fusion:      Eigen Kalman Filter → LiDAR+GPS → state estimate"
echo "  Control:     OSQP MPC Solver → trajectory → throttle/brake/steer"
echo ""
echo "To build algorithm plugins:"
echo "  mkdir -p build/plugins && cd build/plugins"
echo "  g++ -shared -fPIC -O2 ../../src/plugins/perception_opencv.cpp -o libperception_opencv.so \$(pkg-config --cflags --libs opencv4)"
echo "  g++ -shared -fPIC -O2 ../../src/plugins/fusion_eigen.cpp -o libfusion_eigen.so \$(pkg-config --cflags eigen3)"
echo "  g++ -shared -fPIC -O2 ../../src/plugins/control_osqp.cpp -o libcontrol_osqp.so \$(pkg-config --cflags --libs osqp)"
echo ""
echo "See docs/ALGORITHM_STACK.md for the full integration guide."
