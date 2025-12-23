#!/bin/bash

###############################################################################
# Apollo 数据录制简化脚本
# 自动处理文件名格式,确保 DreamView+ 可以识别
###############################################################################

set -e

# 自动检测 Apollo 根目录
APOLLO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

RECORD_DIR="$HOME/.apollo/resources/records"

show_usage() {
    echo "用法: $0 <录制名称> [录制时长(秒)]"
    echo ""
    echo "示例:"
    echo "  $0 test_drive          # 开始录制,手动停止(Ctrl+C)"
    echo "  $0 test_drive 60       # 录制 60 秒后自动停止"
    echo ""
    echo "录制文件会保存到: $RECORD_DIR/<录制名称>.record"
    exit 1
}

if [ -z "$1" ]; then
    show_usage
fi

RECORD_NAME=$1
DURATION=${2:-0}
TEMP_NAME="temp_${RECORD_NAME}_$$"  # 使用临时名称避免冲突

# 定义清理和保存函数
cleanup_and_save() {
    echo ""
    echo -e "${GREEN}停止录制...${NC}"
    "${APOLLO_ROOT}/scripts/record_bag.py" --stop 2>/dev/null || true
    sleep 2

    # 保存录制
    echo -e "${GREEN}保存录制...${NC}"
    "${APOLLO_ROOT}/scripts/record_bag.py" --default_name "$TEMP_NAME" --rename "$RECORD_NAME" 2>/dev/null || true

    # 修正文件名格式
    echo -e "${GREEN}修正文件名格式...${NC}"
    cd "$RECORD_DIR"

    # 处理所有相关文件 (格式: name.00000.timestamp.record 或 name_s.00000.timestamp.record)
    for f in "${RECORD_NAME}"*.record "${RECORD_NAME}_s"*.record; do
        if [ -f "$f" ] && [[ "$f" != *.record ]]; then
            # 如果文件名不是以 .record 结尾(说明有额外后缀)
            # 提取基础名称(去掉 .00000.timestamp.record 部分)
            if [[ "$f" =~ (.*)\.([0-9]{5})\.([0-9]{14})\.record$ ]]; then
                base_name="${BASH_REMATCH[1]}"
                new_name="${base_name}.record"
                mv "$f" "$new_name" 2>/dev/null && echo "  重命名: $f -> $new_name"
            fi
        elif [ -f "$f" ] && [[ "$f" == *.record ]]; then
            # 文件名已经是正确格式,跳过
            continue
        fi
    done

    # 显示结果
    echo ""
    echo -e "${GREEN}=== 录制完成! ===${NC}"
    echo "录制文件:"
    ls -lh "${RECORD_NAME}"*.record 2>/dev/null || echo "  (没有找到文件)"
    echo ""
    echo -e "${GREEN}现在可以在 DreamView+ 中查看录制文件了!${NC}"
    
    exit 0
}

# 捕获 Ctrl+C 信号
trap cleanup_and_save INT TERM

echo -e "${GREEN}=== Apollo 数据录制 ===${NC}"
echo "录制名称: $RECORD_NAME"
if [ $DURATION -gt 0 ]; then
    echo "录制时长: $DURATION 秒"
else
    echo "录制时长: 手动停止(Ctrl+C)"
fi
echo ""

# 停止现有录制
echo -e "${YELLOW}停止现有录制进程...${NC}"
"${APOLLO_ROOT}/scripts/record_bag.py" --stop 2>/dev/null || true
sleep 2

# 开始录制
echo -e "${GREEN}开始录制...${NC}"
"${APOLLO_ROOT}/scripts/record_bag.py" --start --all --dreamview --default_name "$TEMP_NAME"
sleep 3

# 等待指定时长或用户中断
if [ $DURATION -gt 0 ]; then
    echo -e "${YELLOW}录制中... ($DURATION 秒)${NC}"
    sleep $DURATION
    # 时间到了,执行清理和保存
    cleanup_and_save
else
    echo -e "${YELLOW}录制中... (按 Ctrl+C 停止)${NC}"
    while true; do
        sleep 1
    done
fi
