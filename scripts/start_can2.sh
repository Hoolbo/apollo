#!/bin/bash
# 启动工控机 can2 接口 (500kbps)

sudo ip link set can2 down 2>/dev/null
sudo ip link set can2 type can bitrate 500000
sudo ip link set can2 up

echo "✅ can2 已启动 (500kbps)"
ip -details link show can2 | grep -E "can|state"
