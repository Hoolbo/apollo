#!/bin/bash

# CAN接口自动启动脚本（供systemd调用）
# 此脚本假设udev规则已配置，/dev/ttyUSB_CAN已存在

CAN_DEV="can0"
TTY_DEV="/dev/ttyUSB_CAN"
BAUDRATE_SLCAN="-s6"  # 500k
SERIAL_BAUD="3000000"

# 等待设备就绪
sleep 2

# 检查设备是否存在
if [ ! -e "$TTY_DEV" ]; then
    echo "错误: 设备 $TTY_DEV 不存在"
    exit 1
fi

# 清理旧的进程和接口
pkill slcand 2>/dev/null
if ip link show $CAN_DEV > /dev/null 2>&1; then
    ip link set $CAN_DEV down 2>/dev/null
fi
sleep 0.5

# 启动 slcand
slcand -o -c $BAUDRATE_SLCAN -t hw -S $SERIAL_BAUD $TTY_DEV $CAN_DEV
if [ $? -ne 0 ]; then
    echo "slcand 启动失败"
    exit 1
fi

sleep 0.5

# 启用接口
ip link set $CAN_DEV up
if [ $? -ne 0 ]; then
    echo "无法启用 $CAN_DEV"
    exit 1
fi

# 验证
if ip link show $CAN_DEV | grep "UP" > /dev/null; then
    echo "CAN接口 $CAN_DEV 已成功启动"
    exit 0
else
    echo "CAN接口启动异常"
    exit 1
fi
