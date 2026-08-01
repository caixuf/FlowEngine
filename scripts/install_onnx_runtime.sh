#!/bin/bash
# =============================================================================
# install_onnx_runtime.sh — 下载并安装 ONNX Runtime C++ 库
#
# 用于启用 inference_node 的 HAVE_ONNXRUNTIME 编译路径。
# 安装后，以 -DENABLE_ONNX=ON 构建 modules/adas_nodes 即可启用 ONNX 推理后端。
# 未安装时，onnx_backend.cpp 编译为降级桩，运行时自动回退 tiny-MLP，行为零变化。
#
# 用法:
#   bash scripts/install_onnx_runtime.sh              # 安装到 /usr/local
#   bash scripts/install_onnx_runtime.sh --prefix ~/local  # 安装到指定目录
# =============================================================================
set -e

# ── 配置 ──────────────────────────────────────────────────────
# 默认安装到 /usr/local（需要 sudo），可用 --prefix 覆盖
PREFIX="/usr/local"
# 默认架构：x86_64（Linux），可用 --arch 覆盖
ARCH="x86_64"
# ONNX Runtime 版本
ONNX_VERSION="1.18.0"

# 解析参数
while [ $# -gt 0 ]; do
    case "$1" in
        --prefix) PREFIX="$2"; shift 2 ;;
        --arch)   ARCH="$2";   shift 2 ;;
        --help|-h)
            echo "用法: $0 [--prefix <路径>] [--arch <架构>]"
            echo "  架构: x86_64 (默认), aarch64"
            exit 0 ;;
        *) echo "未知参数: $1"; exit 1 ;;
    esac
done

# ── 检测架构 ──────────────────────────────────────────────────
DETECTED_ARCH="$(uname -m)"
case "$DETECTED_ARCH" in
    x86_64|amd64) ARCH="x64" ;;
    aarch64|arm64) ARCH="aarch64" ;;
    *) echo "[WARN] 未识别架构 $DETECTED_ARCH，用 --arch 指定"; ARCH="x64" ;;
esac

# ── 下载 ──────────────────────────────────────────────────────
URL="https://github.com/microsoft/onnxruntime/releases/download/v${ONNX_VERSION}/onnxruntime-linux-${ARCH}-${ONNX_VERSION}.tgz"
TMP_DIR="$(mktemp -d)"
trap "rm -rf '$TMP_DIR'" EXIT

echo "[INFO] 下载 ONNX Runtime v${ONNX_VERSION} (${ARCH})..."
echo "       ${URL}"

if command -v curl >/dev/null 2>&1; then
    curl -L -# "$URL" -o "$TMP_DIR/onnxruntime.tgz"
elif command -v wget >/dev/null 2>&1; then
    wget -q --show-progress "$URL" -O "$TMP_DIR/onnxruntime.tgz"
else
    echo "[ERROR] 需要 curl 或 wget"
    exit 1
fi

# ── 解压 ──────────────────────────────────────────────────────
echo "[INFO] 解压..."
tar xzf "$TMP_DIR/onnxruntime.tgz" -C "$TMP_DIR"
EXTRACTED_DIR="$TMP_DIR/onnxruntime-linux-${ARCH}-${ONNX_VERSION}"

if [ ! -d "$EXTRACTED_DIR" ]; then
    echo "[ERROR] 解压失败：未找到 $EXTRACTED_DIR"
    exit 1
fi

# ── 安装 ──────────────────────────────────────────────────────
echo "[INFO] 安装到 ${PREFIX}..."

# 复制头文件
mkdir -p "$PREFIX/include/onnxruntime"
cp -r "$EXTRACTED_DIR/include/"* "$PREFIX/include/onnxruntime/"

# 复制库文件
mkdir -p "$PREFIX/lib"
cp -r "$EXTRACTED_DIR/lib/"* "$PREFIX/lib/"

# 更新动态链接器缓存（仅 Linux + 系统目录需要）
if [ "$PREFIX" = "/usr/local" ] && [ "$(uname)" = "Linux" ]; then
    if command -v ldconfig >/dev/null 2>&1; then
        echo "[INFO] 更新 ldconfig..."
        ldconfig 2>/dev/null || true
    fi
fi

echo "[SUCCESS] ONNX Runtime v${ONNX_VERSION} 已安装到 ${PREFIX}"
echo ""
echo "构建方式:"
echo "  cmake -B build/modules/adas_nodes -S modules/adas_nodes \\"
echo "        -DFLOWENGINE_BUILD=build \\"
echo "        -DENABLE_ONNX=ON \\"
echo "        -DCMAKE_PREFIX_PATH=${PREFIX}"
echo "  cmake --build build/modules/adas_nodes -j\$(nproc)"
echo ""
echo "验证:"
echo "  python3 -c 'import onnxruntime; print(onnxruntime.__version__)'"