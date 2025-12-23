#!/bin/bash

# CAN服务安装脚本（支持USB热插拔）

if [ "$EUID" -ne 0 ]; then
  echo "请使用 sudo 运行此脚本: sudo ./install_can_service.sh"
  exit 1
fi

# 脚本在 scripts/ 目录下，Apollo根目录是上一级
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
APOLLO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
SERVICE_FILE="mycar-can@.service"
SCRIPT_FILE="start_can_interface.sh"
UDEV_RULE_FILE="/etc/udev/rules.d/99-usb-can-auto.rules"

echo "========================================================"
echo "     安装 Mycar CAN 热插拔自动启动服务"
echo "========================================================"

# 1. 检测USB-CAN设备
echo ""
echo "正在检测USB-CAN设备..."
devices=(/dev/ttyACM*)

if [ ! -e "${devices[0]}" ]; then
    echo "⚠️  未检测到 /dev/ttyACM* 设备"
    echo "   请先插入USB-CAN设备，然后重新运行此脚本"
    exit 1
fi

# 选择设备
if [ "${#devices[@]}" -eq 1 ]; then
    SELECTED_DEV=${devices[0]}
    echo "✅ 自动检测到设备: $SELECTED_DEV"
else
    echo "检测到多个设备："
    for i in "${!devices[@]}"; do
        echo "[$i] ${devices[$i]}"
    done
    read -p "请输入 CAN 设备的序号 [0]: " dev_index
    dev_index=${dev_index:-0}
    SELECTED_DEV=${devices[$dev_index]}
fi

# 2. 获取设备ID
echo "正在读取设备信息..."
VENDOR_ID=$(udevadm info -a -n $SELECTED_DEV | grep "ATTRS{idVendor}" | head -n1 | cut -d '"' -f 2)
PRODUCT_ID=$(udevadm info -a -n $SELECTED_DEV | grep "ATTRS{idProduct}" | head -n1 | cut -d '"' -f 2)
SERIAL=$(udevadm info -a -n $SELECTED_DEV | grep "ATTRS{serial}" | head -n1 | cut -d '"' -f 2)

if [ -z "$VENDOR_ID" ] || [ -z "$PRODUCT_ID" ]; then
    echo "❌ 无法读取设备的 Vendor/Product ID"
    exit 1
fi

echo "  VID: $VENDOR_ID"
echo "  PID: $PRODUCT_ID"
[ ! -z "$SERIAL" ] && echo "  Serial: $SERIAL"

# 3. 创建udev规则（支持热插拔触发）
echo ""
echo "创建udev规则..."
if [ ! -z "$SERIAL" ]; then
    RULE_CONTENT="SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"$VENDOR_ID\", ATTRS{idProduct}==\"$PRODUCT_ID\", ATTRS{serial}==\"$SERIAL\", SYMLINK+=\"ttyUSB_CAN\", MODE=\"0666\", TAG+=\"systemd\", ENV{SYSTEMD_WANTS}=\"mycar-can@%k.service\""
else
    RULE_CONTENT="SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"$VENDOR_ID\", ATTRS{idProduct}==\"$PRODUCT_ID\", SYMLINK+=\"ttyUSB_CAN\", MODE=\"0666\", TAG+=\"systemd\", ENV{SYSTEMD_WANTS}=\"mycar-can@%k.service\""
fi

echo "$RULE_CONTENT" > $UDEV_RULE_FILE
echo "✅ udev规则已写入: $UDEV_RULE_FILE"

# 4. 给启动脚本添加执行权限
echo ""
echo "设置脚本权限..."
chmod +x "$SCRIPT_DIR/$SCRIPT_FILE"

# 5. 复制服务文件到systemd目录
echo "安装systemd服务..."
# 更新服务文件中的脚本路径为绝对路径
sed "s|/home/hoolbo/code/apollo/start_can_interface.sh|${SCRIPT_DIR}/start_can_interface.sh|g" \
    "$SCRIPT_DIR/$SERVICE_FILE" > /tmp/mycar-can@.service
cp /tmp/mycar-can@.service /etc/systemd/system/
rm /tmp/mycar-can@.service

# 6. 重载配置
echo "重载配置..."
udevadm control --reload-rules
systemctl daemon-reload

# 7. 触发udev规则（如果设备已插入）
echo "触发udev规则..."
udevadm trigger

sleep 2

# 8. 检查状态
echo ""
echo "========================================================"
if [ -e "/dev/ttyUSB_CAN" ]; then
    echo "✅ 设备符号链接已创建: /dev/ttyUSB_CAN"
    
    # 检查服务是否自动启动
    if systemctl is-active --quiet "mycar-can@$(basename $SELECTED_DEV).service" 2>/dev/null; then
        echo "✅ CAN服务已自动启动"
        ip link show can0 2>/dev/null && echo "✅ can0 接口已就绪"
    else
        echo "⚠️  服务未自动启动，尝试手动启动..."
        systemctl start "mycar-can@$(basename $SELECTED_DEV).service"
    fi
else
    echo "⚠️  符号链接未创建，请重新插拔USB设备"
fi

echo ""
echo "========================================================"
echo "✅ 安装完成！"
echo ""
echo "📌 工作模式："
echo "  - USB插入时：自动启动CAN接口"
echo "  - USB拔出时：自动停止CAN接口"
echo ""
echo "常用命令："
echo "  查看服务: sudo systemctl status 'mycar-can@*'"
echo "  查看日志: sudo journalctl -u 'mycar-can@*' -f"
echo "  查看can0: ip link show can0"
echo "  卸载服务: sudo rm $UDEV_RULE_FILE /etc/systemd/system/mycar-can@.service"
echo "========================================================"
