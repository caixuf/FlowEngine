#!/usr/bin/env bash
# ───────────────────────────────────────────────────────────────
# install-skills.sh — 安装 FlowEngine skills 到用户 Claude 目录
# ───────────────────────────────────────────────────────────────
# 用法:
#   bash scripts/install-skills.sh                # 安装项目 skill
#   bash scripts/install-skills.sh --all          # 同时安装教程
#
# 安装后, 所有 Claude Code 项目中均可使用这些 skill。
# ───────────────────────────────────────────────────────────────

set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SKILLS_SRC="$SCRIPT_DIR/.claude/skills"
TUTORIALS_SRC="$SCRIPT_DIR/skills"
SKILLS_DST="${HOME}/.claude/skills"

INSTALL_TUTORIALS=false
for arg in "$@"; do
    case "$arg" in
        --tutorials|--all) INSTALL_TUTORIALS=true ;;
    esac
done

mkdir -p "$SKILLS_DST"

echo "✦ FlowEngine Skills Installer"
echo "  源:   $SKILLS_SRC"
echo "  目标: $SKILLS_DST"
echo ""

# 安装项目 skills
count=0
while IFS= read -r -d '' f; do
    name=$(basename "$f")
    cp "$f" "$SKILLS_DST/$name"
    echo "  ✓ $name"
    count=$((count + 1))
done < <(find "$SKILLS_SRC" -maxdepth 1 -name "*.md" -print0)

# 可选：安装教程
tcount=0
if $INSTALL_TUTORIALS; then
    echo ""
    echo "  教程:"
    while IFS= read -r -d '' f; do
        name=$(basename "$f")
        cp "$f" "$SKILLS_DST/$name"
        echo "  ✓ $name"
        tcount=$((tcount + 1))
    done < <(find "$TUTORIALS_SRC" -maxdepth 1 -name "*.md" -print0)
fi

# 统计
total=$((count + tcount))
echo ""
echo "✦ 完成: 安装 $count 个项目 skill + $tcount 个教程 = $total 个"
echo "   路径: $SKILLS_DST"

if command -v claude &>/dev/null; then
    echo "   重启 Claude Code 后即可使用 /<skill-name>"
fi
