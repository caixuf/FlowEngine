#!/bin/bash
set -e
BUILD_TYPE=Release
STRIP=false
PREFIX=""
PACKAGE_MODE=false

for arg in "$@"; do
    case "$arg" in
        --release) BUILD_TYPE=Release ;;
        --debug)   BUILD_TYPE=Debug ;;
        --strip)   STRIP=true ;;
        --package) PACKAGE_MODE=true ;;
        --*)       echo "Unknown option: $arg"; exit 1 ;;
        *)         PREFIX="$arg" ;;
    esac
done

if [ "$PACKAGE_MODE" = false ] && [ -z "$PREFIX" ]; then
    echo "Usage: bash scripts/deploy.sh [--release|--debug] [--strip] <target_dir>"
    echo "   or: bash scripts/deploy.sh --package [--release|--debug|--strip]"
    exit 1
fi

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BUILD_DIR="$ROOT/build_deploy"
NODES_BUILD_DIR="$BUILD_DIR/modules/adas_nodes"

if [ "$PACKAGE_MODE" = true ]; then
    VERSION=$(git -C "$ROOT" describe --tags --always 2>/dev/null || echo "dev")
    TARNAME="flowengine-${VERSION}-linux-x86_64.tar.gz"
    DIST_DIR="$ROOT/dist"
    mkdir -p "$DIST_DIR"
    PREFIX="$(mktemp -d /tmp/flowengine_pkg_XXXXXX)"
fi

echo "[1/4] Building..."
cmake -S "$ROOT" -B "$BUILD_DIR" -DCMAKE_BUILD_TYPE="$BUILD_TYPE" -DCMAKE_INSTALL_PREFIX="$PREFIX" > /dev/null 2>&1
cmake --build "$BUILD_DIR" -j$(nproc) 2>/dev/null
cmake -S "$ROOT/modules/adas_nodes" -B "$NODES_BUILD_DIR" \
    -DFLOWENGINE_BUILD="$BUILD_DIR" \
    -DCMAKE_BUILD_TYPE="$BUILD_TYPE" \
    -DCMAKE_INSTALL_PREFIX="$PREFIX" > /dev/null 2>&1
cmake --build "$NODES_BUILD_DIR" -j$(nproc) 2>/dev/null
echo "[2/4] Installing..."
cmake --install "$BUILD_DIR" 2>/dev/null
cmake --install "$NODES_BUILD_DIR" 2>/dev/null
if $STRIP; then echo "[3/4] Stripping..."; find "$PREFIX/bin" "$PREFIX/lib" -type f -executable -o -name "*.so" | xargs strip 2>/dev/null || true; else echo "[3/4] Skipping strip"; fi
echo "[4/4] Environment..."
cat > "$PREFIX/flowengine.env" << EOF
export FLOWENGINE_HOME="$PREFIX"
export PATH="\$FLOWENGINE_HOME/bin:\$PATH"
export LD_LIBRARY_PATH="\$FLOWENGINE_HOME/lib:\$FLOWENGINE_HOME/lib/flowengine/plugins:\${LD_LIBRARY_PATH:-}"
EOF

if [ "$PACKAGE_MODE" = true ]; then
    cd "$PREFIX/.."
    tar czf "$DIST_DIR/$TARNAME" "$(basename "$PREFIX")/"
    rm -rf "$PREFIX"
    echo "✓ $DIST_DIR/$TARNAME"
else
    echo "✓ Deploy to $PREFIX"
fi
