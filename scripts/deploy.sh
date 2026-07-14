#!/bin/bash
# =============================================================================
# FlowEngine Deploy — 一键部署到目标目录
#
# 用法:
#   bash scripts/deploy.sh /mnt/m              # 部署到嵌入式设备挂载点
#   bash scripts/deploy.sh /opt/flowengine      # 部署到本地目录
#   bash scripts/deploy.sh --release /mnt/m     # Release 构建 + 部署
#   bash scripts/deploy.sh --strip /mnt/m       # 部署并 strip 符号
#
# 部署后目标目录结构:
#   /mnt/m/
#   ├── bin/           # flowctl, flow_launcher, launcher, ...
#   ├── lib/           # libflowengine_core.a, plugins/
#   ├── include/       # 头文件
#   ├── share/         # scripts, tools
#   └── flowengine.env # 环境变量（source 即可用）
# =============================================================================
set -e

BUILD_TYPE=Release
STRIP=false
PREFIX=""

for arg in "$@"; do
    case "$arg" in
        --release) BUILD_TYPE=Release ;;
        --debug)   BUILD_TYPE=Debug ;;
        --strip)   STRIP=true ;;
        --*)       echo "Unknown option: $arg"; exit 1 ;;
        *)         PREFIX="$arg" ;;
    esac
done

if [ -z "$PREFIX" ]; then
    echo "Usage: bash scripts/deploy.sh [--release|--debug] [--strip] <target_dir>"
    echo ""
    echo "Examples:"
    echo "  bash scripts/deploy.sh /mnt/m              # Deploy to mounted device"
    echo "  bash scripts/deploy.sh /opt/flowengine      # Deploy to local dir"
    echo "  bash scripts/deploy.sh --strip /mnt/m       # Strip + deploy"
    exit 1
fi

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BUILD_DIR="$ROOT/build_deploy"

echo "╔══════════════════════════════════════════╗"
echo "║  FlowEngine Deploy                       ║"
echo "║  Target: $PREFIX"
echo "║  Build:  $BUILD_TYPE"
echo "╚══════════════════════════════════════════╝"
echo ""

# ── 1. Build ─────────────────────────────────────────────────
echo "[1/4] Building ($BUILD_TYPE)..."
cmake -S "$ROOT" -B "$BUILD_DIR" \
    -DCMAKE_BUILD_TYPE="$BUILD_TYPE" \
    -DCMAKE_INSTALL_PREFIX="$PREFIX" \
    > /dev/null 2>&1
cmake --build "$BUILD_DIR" -j$(nproc) 2>/dev/null
echo "  ✓ Build complete"

# ── 2. Install ───────────────────────────────────────────────
echo "[2/4] Installing to $PREFIX..."
rm -rf "$PREFIX/bin" "$PREFIX/lib" "$PREFIX/include/flowengine" "$PREFIX/share/flowengine" 2>/dev/null || true
cmake --install "$BUILD_DIR" 2>/dev/null
echo "  ✓ Installed"

# ── 3. Strip (optional) ──────────────────────────────────────
if $STRIP; then
    echo "[3/4] Stripping symbols..."
    find "$PREFIX/bin" -type f -executable 2>/dev/null | while read f; do
        strip "$f" 2>/dev/null && echo "  stripped: $(basename $f)" || true
    done
    find "$PREFIX/lib" -name "*.so" 2>/dev/null | while read f; do
        strip --strip-unneeded "$f" 2>/dev/null && echo "  stripped: $(basename $f)" || true
    done
else
    echo "[3/4] Skipping strip (use --strip to reduce size)"
fi

# ── 4. Environment setup ─────────────────────────────────────
echo "[4/4] Generating environment..."
cat > "$PREFIX/flowengine.env" << EOF
# FlowEngine environment — source this file to use
export PATH="$PREFIX/bin:\$PATH"
export LD_LIBRARY_PATH="$PREFIX/lib:\$LD_LIBRARY_PATH"
export PKG_CONFIG_PATH="$PREFIX/lib/pkgconfig:\$PKG_CONFIG_PATH"
export FLOWENGINE_HOME="$PREFIX"
EOF

# ── Summary ──────────────────────────────────────────────────
SIZE=$(du -sh "$PREFIX" 2>/dev/null | cut -f1)
echo ""
echo "╔══════════════════════════════════════════╗"
echo "║  Deploy Complete                          ║"
echo "╠══════════════════════════════════════════╣"
echo "║  Path:  $PREFIX"
echo "║  Size:  $SIZE"
echo "║  Files: $(find "$PREFIX" -type f 2>/dev/null | wc -l)"
echo "╠══════════════════════════════════════════╣"
echo "║  To use on target:                       ║"
echo "║    source $PREFIX/flowengine.env          ║"
echo "║    flowctl version                       ║"
echo "╚══════════════════════════════════════════╝"
