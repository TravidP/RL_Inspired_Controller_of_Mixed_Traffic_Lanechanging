#!/bin/bash
# 自动修复 SUMO XSD schema 缺失问题

TARGET="/home/spei/sumo_binaries/data/xsd"

# 常见 SUMO 安装路径
CANDIDATES=(
  "/usr/share/sumo/data/xsd"
  "/usr/local/share/sumo/data/xsd"
  "$HOME/sumo/data/xsd"
  "$HOME/sumo_binaries/sumo/data/xsd"
)

if [ -d "$TARGET" ]; then
    echo "✅ 已存在: $TARGET, 无需修复"
    exit 0
fi

echo "⚠️ 未找到 $TARGET, 开始尝试修复..."

for SRC in "${CANDIDATES[@]}"; do
    if [ -d "$SRC" ]; then
        echo "🔍 找到 schema 目录: $SRC"
        mkdir -p "$(dirname "$TARGET")"
        cp -r "$SRC" "$TARGET"
        echo "✅ 已复制 schema 到: $TARGET"
        exit 0
    fi
done

echo "❌ 没有找到 SUMO schema，请检查 SUMO_HOME 是否设置正确"
exit 1

