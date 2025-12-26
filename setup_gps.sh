#!/bin/bash

# setup_gps.sh - 系统级配置
# 创建 udev 规则，让设备自动出现在 /dev/gps_serial
# 自动设置权限 0666（无需每次 sudo）
# 自动配置波特率
# 持久化：写入 /etc/udev/rules.d/，重启后依然有效

# Define the rule content
RULE_FILE="/etc/udev/rules.d/99-gps-serial.rules"
VENDOR_ID="067b"
PRODUCT_ID="23c3"
SYMLINK="gps_serial"
BAUD_RATE="115200"

echo "Setting up udev rules for GPS device..."
echo "Vendor: $VENDOR_ID, Product: $PRODUCT_ID, Symlink: /dev/$SYMLINK"

# Create the rule file
# We use sudo tee to write to the protected directory
echo "Creating $RULE_FILE..."
# Use stty raw mode and -echo to prevent device from receiving its own echoed data
echo "SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"$VENDOR_ID\", ATTRS{idProduct}==\"$PRODUCT_ID\", ENV{ID_MM_DEVICE_IGNORE}=\"1\", MODE=\"0666\", SYMLINK+=\"$SYMLINK\", RUN+=\"/bin/stty -F /dev/%k $BAUD_RATE raw cs8 -cstopb -parenb -echo -echoe -echok -echoctl -echoke\"" | sudo tee $RULE_FILE > /dev/null

if [ $? -eq 0 ]; then
    echo "Rule file created successfully."
else
    echo "Error creating rule file. Please check sudo permissions."
    exit 1
fi

# Reload udev rules
echo "Reloading udev rules..."
sudo udevadm control --reload-rules
sudo udevadm trigger

echo "Done! The device should now be accessible at /dev/$SYMLINK with permissions automatically set."
echo "If it doesn't appear immediately, try unplugging and replugging the device."
