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
echo "[2/4] Installing..."
cmake --install "$BUILD_DIR" 2>/dev/null
if $STRIP; then echo "[3/4] Stripping..."; find "$PREFIX/bin" "$PREFIX/lib" -type f -executable -o -name "*.so" | xargs strip 2>/dev/null || true; else echo "[3/4] Skipping strip"; fi
echo "[4/4] Environment..."
cat > "$PREFIX/flowengine.env" << EOF
export PATH="$PREFIX/bin:\\/home/caixuf/.npm-global/bin:/home/caixuf/.kimi-code/bin:/home/caixuf/.trae-cn-server/bin/stable-428b3440427a64b51632df94fd7d6611443d246b-debian10/bin/remote-cli:/home/caixuf/.local/bin:/home/caixuf/.npm-global/bin:/home/caixuf/.kimi-code/bin:/home/caixuf/.nvm/versions/node/v24.18.1/bin:/home/caixuf/.local/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/games:/usr/local/games:/usr/lib/wsl/lib:/mnt/c/Users/20247/.trae-cn/tools/trae-gopls/current:/mnt/c/Users/20247/.trae-cn/sdks/workspaces/77977199/versions/node/current:/mnt/c/Users/20247/.trae-cn/sdks/versions/node/current:/mnt/c/WINDOWS/system32:/mnt/c/WINDOWS:/mnt/c/WINDOWS/System32/Wbem:/mnt/c/WINDOWS/System32/WindowsPowerShell/v1.0/:/mnt/c/WINDOWS/System32/OpenSSH/:/mnt/c/Qt/Qt5.12.12/5.12.12/mingw73_64/bin:/mnt/c/mingw64/mingw64/bin:/mnt/c/mingw64/mingw64/bin/g++.exe:/mnt/c/Program Files/Microsoft SQL Server/150/Tools/Binn/:/mnt/d/MyCode/MyLib/qpdf 11.6.3/bin:/mnt/c/Qt/Qt6.5.3/6.5.3/msvc2019_64:/mnt/c/Qt/Qt6.5.3/6.5.3/msvc2019_64/bin:/mnt/c/Qt/Qt6.5.3/Tools/CMake_64/bin:/mnt/c/Qt/Qt6.5.3/Tools/QtCreator/bin:/mnt/c/Program Files/Java/jdk-19/bin:/mnt/c/Program Files/Git/cmd:/mnt/d/MyCode/CodeTools:/mnt/c/Program Files/Graphviz/bin:/mnt/c/Program Files/TortoiseGit/bin:/mnt/c/Program Files/Sunshine:/mnt/c/Program Files/Sunshine/tools:/mnt/c/Users/20247/AppData/Local/Microsoft/WindowsApps:/mnt/c/Users/20247/AppData/Local/Programs/Microsoft VS Code/bin:/mnt/c/Qt/6.5.3/mingw_64/bin:/mnt/c/Users/20247/AppData/Local/Programs/Trae CN/resources/app/bin/lib:/snap/bin:/home/caixuf/.trae-cn-server/extensions/ms-python.debugpy-2026.6.0-linux-x64/bundled/scripts/noConfigScripts"
export LD_LIBRARY_PATH="$PREFIX/lib:\"
export FLOWENGINE_HOME="$PREFIX"
EOF

if [ "$PACKAGE_MODE" = true ]; then
    cd "$PREFIX/.."
    tar czf "$DIST_DIR/$TARNAME" "$(basename "$PREFIX")/"
    rm -rf "$PREFIX"
    echo "✓ $DIST_DIR/$TARNAME"
else
    echo "✓ Deploy to $PREFIX"
fi

