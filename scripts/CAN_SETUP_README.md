# CAN接口自动启动配置指南

## 📋 概述

本目录包含用于自动配置USB-CAN设备的脚本，支持热插拔自动启动CAN接口。

## 📦 文件说明

### 1. `install_can_service.sh` - 一键安装脚本
- **功能**: 自动检测USB-CAN设备并配置systemd服务
- **运行次数**: 只需运行一次
- **位置**: 可在任何目录运行（不依赖工作目录）

### 2. `start_can_interface.sh` - CAN接口启动脚本
- **功能**: 启动slcand守护进程并启用can0接口
- **调用方式**: 由systemd服务自动调用
- **无需手动运行**

### 3. `mycar-can@.service` - systemd服务模板
- **功能**: 定义CAN接口服务的启动方式
- **特性**: 支持设备热插拔，设备拔出时自动停止

## 🚀 使用步骤

### 首次安装（只需一次）

1. **插入USB-CAN设备**

2. **运行安装脚本**
   ```bash
   sudo /home/hoolbo/code/apollo/scripts/install_can_service.sh
   ```

3. **完成！** 脚本会自动：
   - 检测USB-CAN设备的VID/PID
   - 创建udev规则（`/etc/udev/rules.d/99-usb-can-auto.rules`）
   - 安装systemd服务（`/etc/systemd/system/mycar-can@.service`）
   - 配置设备符号链接（`/dev/ttyUSB_CAN`）

### 日常使用

**完全自动，无需任何操作！**

- ✅ **插入USB** → 自动启动CAN接口（can0）
- ✅ **拔出USB** → 自动停止服务
- ✅ **系统重启** → 插入USB后自动启动

## 🔍 验证和管理

### 检查CAN接口状态
```bash
# 查看can0接口
ip link show can0

# 查看服务状态
sudo systemctl status 'mycar-can@*'

# 实时查看日志
sudo journalctl -u 'mycar-can@*' -f
```

### 测试CAN通信
```bash
# 发送测试帧
cansend can0 123#DEADBEEF

# 监听CAN消息
candump can0
```

### 手动控制（如需要）
```bash
# 手动启动服务
sudo systemctl start mycar-can@ttyACM0.service

# 手动停止服务
sudo systemctl stop mycar-can@ttyACM0.service

# 重启服务
sudo systemctl restart mycar-can@ttyACM0.service
```

## 🗑️ 卸载

如需卸载自动启动服务：

```bash
# 删除udev规则
sudo rm /etc/udev/rules.d/99-usb-can-auto.rules

# 删除systemd服务
sudo rm /etc/systemd/system/mycar-can@.service

# 重载配置
sudo udevadm control --reload-rules
sudo systemctl daemon-reload
```

## ⚙️ 工作原理

### 热插拔流程

1. **USB插入** → udev检测到设备
2. **udev规则触发** → 创建 `/dev/ttyUSB_CAN` 符号链接
3. **启动systemd服务** → `mycar-can@ttyACM0.service`
4. **运行启动脚本** → `start_can_interface.sh`
5. **slcand映射** → `/dev/ttyUSB_CAN` → `can0`
6. **can0接口UP** → 准备就绪

### USB拔出流程

1. **USB拔出** → systemd检测到设备消失
2. **服务自动停止** → 执行 `ExecStop`
3. **清理资源** → 停止slcand，关闭can0

## 🔧 技术细节

### CAN参数配置

- **波特率**: 500kbps (`-s6`)
- **串口速率**: 3000000
- **设备映射**: `/dev/ttyUSB_CAN` → `can0`

### 修改参数

如需修改CAN波特率，编辑 `start_can_interface.sh`：

```bash
BAUDRATE_SLCAN="-s6"  # 500k
# 可选值:
# -s4 = 125kbps
# -s5 = 250kbps
# -s6 = 500kbps
# -s8 = 1000kbps
```

修改后需重新运行安装脚本。

## 📝 故障排查

### 问题：设备插入后can0未启动

**检查步骤**：
1. 确认设备已识别：`ls -l /dev/ttyUSB_CAN`
2. 查看服务日志：`sudo journalctl -u 'mycar-can@*' -n 50`
3. 检查udev规则：`cat /etc/udev/rules.d/99-usb-can-auto.rules`

### 问题：权限错误

**解决方案**：
```bash
# 确保当前用户在dialout组
sudo usermod -a -G dialout $USER
# 重新登录生效
```

### 问题：slcand启动失败

**可能原因**：
- USB-CAN设备不支持SLCAN协议
- 串口已被其他程序占用

**检查**：
```bash
# 查看是否有其他进程占用
sudo lsof /dev/ttyUSB_CAN
```

## 📞 相关脚本

- GNSS配置: `init_gnss.sh`
- 数据录制: `record.sh`
- Apollo启动: `apollo.sh`
