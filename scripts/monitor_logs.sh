#!/bin/bash

# Apollo日志实时监控脚本

echo "=== Apollo关键日志监控 ==="
echo ""
echo "选择要查看的日志:"
echo "1) 主进程 (mainboard)"
echo "2) CAN总线 (canbus)"
echo "3) DreamView+"
echo "4) 定位 (localization)"
echo "5) 所有ERROR"
echo "6) 所有WARNING"
echo "7) 自定义grep"
echo ""
read -p "请选择 [1-7]: " choice

case $choice in
    1)
        echo "=== 监控 mainboard.INFO ==="
        tail -f data/log/mainboard.INFO
        ;;
    2)
        echo "=== 监控 canbus.INFO ==="
        tail -f data/log/canbus.INFO
        ;;
    3)
        echo "=== 监控 dreamview_plus.INFO ==="
        tail -f data/log/dreamview_plus.INFO
        ;;
    4)
        echo "=== 监控 localization.INFO ==="
        tail -f data/log/localization.INFO 2>/dev/null || echo "定位模块未启动"
        ;;
    5)
        echo "=== 监控所有ERROR ==="
        tail -f data/log/*.INFO | grep --line-buffered "^E"
        ;;
    6)
        echo "=== 监控所有WARNING ==="
        tail -f data/log/*.INFO | grep --line-buffered "^W"
        ;;
    7)
        read -p "输入grep关键词: " keyword
        echo "=== 监控包含 '$keyword' 的日志 ==="
        tail -f data/log/*.INFO | grep --line-buffered "$keyword"
        ;;
    *)
        echo "无效选择"
        ;;
esac
