#!/bin/bash

# 检查是否以 root 运行
if [ "$EUID" -ne 0 ]; then
  echo "请使用 sudo 运行此脚本: sudo ./init_can.sh"
  exit
fi

CAN_DEV="can0"
TTY_DEV="/dev/ttyUSB_CAN"
BAUDRATE_SLCAN="-s6" # 500k
SERIAL_BAUD="3000000"

echo "========================================================"
echo "          Mycar CAN 一键配置与启动工具"
echo "========================================================"

# ========================================================
# 第一阶段：检查与配置设备名称
# ========================================================

if [ -e "$TTY_DEV" ]; then
    echo "✅ 检测到固定的设备名称 [$TTY_DEV] 已存在。"
else
    echo "⚠️ 未检测到 [$TTY_DEV]。正在进入首次配置模式..."
    echo "--------------------------------------------------------"
    
    # 列出 ttyACM 设备
    devices=(/dev/ttyACM*)
    
    if [ ! -e "${devices[0]}" ]; then
        echo "❌ 错误：未检测到任何 /dev/ttyACM* 设备。" 
        echo "   请确认 USB-CAN 模块已插入电脑。"
        exit 1
    fi
    
    # 只有一个设备时自动选择，多个时询问
    if [ "${#devices[@]}" -eq 1 ]; then
        SELECTED_DEV=${devices[0]}
        echo "自动检测到唯一的 USB 设备: $SELECTED_DEV"
    else
        echo "检测到多个设备："
        for i in "${!devices[@]}"; do
            echo "[$i] ${devices[$i]}"
        done
        read -p "请输入 CAN 卡设备的序号 [0]: " dev_index
        dev_index=${dev_index:-0}
        SELECTED_DEV=${devices[$dev_index]}
    fi

    if [ ! -e "$SELECTED_DEV" ]; then
        echo "❌ 无效的设备选择。"
        exit 1
    fi

    # 获取 ID 并写入 udev 规则
    VENDOR_ID=$(udevadm info -a -n $SELECTED_DEV | grep "ATTRS{idVendor}" | head -n1 | cut -d '"' -f 2)
    PRODUCT_ID=$(udevadm info -a -n $SELECTED_DEV | grep "ATTRS{idProduct}" | head -n1 | cut -d '"' -f 2)
    SERIAL=$(udevadm info -a -n $SELECTED_DEV | grep "ATTRS{serial}" | head -n1 | cut -d '"' -f 2)

    if [ -z "$VENDOR_ID" ] || [ -z "$PRODUCT_ID" ]; then
        echo "❌ 无法读取设备的 Vendor/Product ID，配置中止。"
        exit 1
    fi

    echo "正在绑定设备 (VID:$VENDOR_ID PID:$PRODUCT_ID)..."
    RULE_FILE="/etc/udev/rules.d/99-usb-can.rules"
    
    if [ ! -z "$SERIAL" ]; then
        RULE_CONTENT="SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"$VENDOR_ID\", ATTRS{idProduct}==\"$PRODUCT_ID\", ATTRS{serial}==\"$SERIAL\", SYMLINK+=\"ttyUSB_CAN\", MODE=\"0666\""
    else
        RULE_CONTENT="SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"$VENDOR_ID\", ATTRS{idProduct}==\"$PRODUCT_ID\", SYMLINK+=\"ttyUSB_CAN\", MODE=\"0666\""
    fi

    echo "$RULE_CONTENT" > $RULE_FILE
    udevadm control --reload-rules
    udevadm trigger
    
    # 等待 udev 生效
    echo "等待设备挂载..."
    sleep 2
    
    if [ -e "$TTY_DEV" ]; then
         echo "✅配置成功！设备已绑定到 $TTY_DEV"
    else
         echo "❌ 配置写入似乎尚未生效，请尝试重新插拔设备后再次运行本脚本。"
         exit 1
    fi
    echo "--------------------------------------------------------"
fi

# ========================================================
# 第二阶段：启动 CAN 接口
# ========================================================

echo "正在准备启动 CAN 接口..."

# 1. 清理
pkill slcand 2>/dev/null
if ip link show $CAN_DEV > /dev/null 2>&1; then
    ip link set $CAN_DEV down
fi
sleep 0.5

# 2. 启动 slcand
echo "启动 slcand (500k baud)..."
slcand -o -c $BAUDRATE_SLCAN -t hw -S $SERIAL_BAUD $TTY_DEV $CAN_DEV
if [ $? -ne 0 ]; then
    echo "❌ slcand 启动失败。"
    exit 1
fi

sleep 0.5

# 3. 启用接口
ip link set $CAN_DEV up
if [ $? -ne 0 ]; then
    echo "❌ 无法 UP 接口 $CAN_DEV。"
    exit 1
fi

# 4. 验证
echo "--------------------------------------------------------"
if ip link show $CAN_DEV | grep "UP" > /dev/null; then
    echo "🎉 成功！Mycar CAN 接口已就绪。"
    echo "   状态: UP"
    echo "   设备: $TTY_DEV -> $CAN_DEV"
else
    echo "⚠️ 未知状况：命令执行完毕但接口未处于 UP 状态。"
fi
echo "========================================================"
