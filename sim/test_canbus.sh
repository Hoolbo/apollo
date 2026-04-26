#!/bin/bash
# ═══════════════════════════════════════════════════════════════
#  铰接车 CAN 报文解析验证脚本
#  在容器内运行，canbus 模块需已启动
#  用法: bash sim/test_canbus.sh
# ═══════════════════════════════════════════════════════════════

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

echo -e "${CYAN}═══════════════════════════════════════════════════════════${NC}"
echo -e "${CYAN}  铰接车 CAN 报文逐条验证${NC}"
echo -e "${CYAN}═══════════════════════════════════════════════════════════${NC}"
echo ""
echo -e "${YELLOW}请确保 canbus 模块已启动: cyber_launch start modules/canbus/launch/canbus.launch${NC}"
echo -e "${YELLOW}请确保 cyber_monitor 已打开查看 /apollo/canbus/chassis${NC}"
echo ""
read -p "按 Enter 开始测试..."

# ──────────────────────────────────────────────────────────────
# 测试 1: 前车驱动电机反馈 (0x23C = 572)
# ──────────────────────────────────────────────────────────────
echo ""
echo -e "${GREEN}═══ 测试 1: 前车驱动电机反馈 (CAN ID 0x23C) ═══${NC}"
echo ""

# drive_motor_speed: bytes[5](low) + bytes[6](high), factor=0.1, unit=km/h
# 目标: 3.6 km/h → raw = 36 = 0x0024 → bytes[5]=0x24, bytes[6]=0x00
# drive_motor_torque: bytes[3](low) + bytes[4](high), factor=1.0
# 目标: torque = 50 → raw = 50 = 0x0032 → bytes[3]=0x32, bytes[4]=0x00
# drive_motor_shift: bytes[7] bit[5:4], 1=SHIFT_D (前进), 2=SHIFT_R (倒退)
# 目标: 前进挡 → value=1 → bit[5:4]=01 → bytes[7] |= 0x10
# drive_motor_enable: bytes[7] bit[0], 1=enabled
# 目标: 使能 → bytes[7] |= 0x01
# bytes[7] = 0x10 | 0x01 = 0x11
# drive_motor_stop: bytes[0] bit[0], 0=not stopped
#
# 合成: Byte0=00, 1=00, 2=00, 3=32, 4=00, 5=24, 6=00, 7=11
FRAME_23C="00000032002400011"
# 修正为8字节hex: 0000003200240011
echo -e "  发送: cansend can0 23C#0000003200240011"
echo -e "  预期:"
echo -e "    speed_mps      = 3.6 km/h ÷ 3.6 = ${CYAN}1.0 m/s${NC}"
echo -e "    torque         = ${CYAN}50${NC}"
echo -e "    shift          = ${CYAN}SHIFT_D (前进)${NC}"
echo -e "    enable         = ${CYAN}true${NC}"
echo ""

# 持续发送 2 秒
for i in $(seq 1 100); do
    cansend can0 23C#0000003200240011
    sleep 0.02
done
echo -e "${GREEN}  ✓ 已发送 100 帧，请检查 cyber_monitor${NC}"
read -p "  按 Enter 继续下一个测试..."

# ──────────────────────────────────────────────────────────────
# 测试 2: 前车 EPS 转向反馈 (0x22C = 556)
# ──────────────────────────────────────────────────────────────
echo ""
echo -e "${GREEN}═══ 测试 2: 前车 EPS 转向反馈 (CAN ID 0x22C) ═══${NC}"
echo ""

# eps_angle: bytes[5](low) + bytes[6](high), factor=1.0, offset=-1024
# 目标: eps_angle = 30° → raw = 30 + 1024 = 1054 = 0x041E
#   → bytes[5]=0x1E, bytes[6]=0x04
# eps_angle_speed: bytes[4], factor=2.0
# 目标: 角速度 = 20°/s → raw = 10 = 0x0A → bytes[4]=0x0A
# eps_enable: bytes[7] bit[0], 1=enabled
#   → bytes[7] = 0x01
# eps_error: bytes[0], 0=无故障
#
# 合成: Byte0=00, 1=00, 2=00, 3=00, 4=0A, 5=1E, 6=04, 7=01
echo -e "  发送: cansend can0 22C#000000000A1E0401"
echo -e "  预期:"
echo -e "    eps_angle      = raw(1054) - 1024 = ${CYAN}30°${NC}"
echo -e "    eps_angle_speed = raw(10) × 2.0 = ${CYAN}20°/s${NC}"
echo -e "    eps_enable     = ${CYAN}true${NC}"
echo -e "    eps_error      = ${CYAN}0${NC}"
echo -e "    chassis.steering_percentage = 30 ÷ 120 × 100 = ${CYAN}25.0%${NC}"
echo ""

for i in $(seq 1 100); do
    cansend can0 22C#000000000A1E0401
    sleep 0.02
done
echo -e "${GREEN}  ✓ 已发送 100 帧，请检查 cyber_monitor${NC}"
read -p "  按 Enter 继续下一个测试..."

# ──────────────────────────────────────────────────────────────
# 测试 3: 前车 VCU 状态 (0x20C = 524)
# ──────────────────────────────────────────────────────────────
echo ""
echo -e "${GREEN}═══ 测试 3: 前车 VCU 状态反馈 (CAN ID 0x20C) ═══${NC}"
echo ""

# acu_error: bytes[0] bit[0], 0=无故障
# acu_remote_control: bytes[7] bit[6], 1=遥控模式
# acu_control_mode: bytes[7] bit[7], 1=自动模式
# acu_receive_info: bytes[7] bit[3], 1=接收正常
# 目标: 自动模式 + 接收正常 → bytes[7] = 0x80 | 0x08 = 0x88
#
# 合成: Byte0=00, 1-6=00, 7=88
echo -e "  发送: cansend can0 20C#0000000000000088"
echo -e "  预期:"
echo -e "    acu_error       = ${CYAN}false${NC}"
echo -e "    acu_control_mode = ${CYAN}true (自动)${NC}"
echo -e "    acu_receive_info = ${CYAN}true${NC}"
echo ""

for i in $(seq 1 100); do
    cansend can0 20C#0000000000000088
    sleep 0.02
done
echo -e "${GREEN}  ✓ 已发送 100 帧，请检查 cyber_monitor${NC}"
read -p "  按 Enter 继续下一个测试..."

# ──────────────────────────────────────────────────────────────
# 测试 4: 后车运动反馈 (0x221 = 545)
# ──────────────────────────────────────────────────────────────
echo ""
echo -e "${GREEN}═══ 测试 4: 后车运动反馈 (CAN ID 0x221) ═══${NC}"
echo ""

# linear_speed: bytes[0](high) + bytes[1](low), Motorola, signed, factor=0.001, unit=m/s
# 目标: 0.5 m/s → raw = 500 = 0x01F4 → bytes[0]=0x01, bytes[1]=0xF4
# steering_angle: bytes[6](high) + bytes[7](low), Motorola, signed, factor=0.001, unit=rad
# 目标: 0.1 rad → raw = 100 = 0x0064 → bytes[6]=0x00, bytes[7]=0x64
#
# 合成: Byte0=01, 1=F4, 2-5=00, 6=00, 7=64
echo -e "  发送: cansend can0 221#01F4000000000064"
echo -e "  预期:"
echo -e "    linear_speed    = raw(500) × 0.001 = ${CYAN}0.5 m/s${NC}"
echo -e "    steering_angle  = raw(100) × 0.001 = ${CYAN}0.1 rad${NC}"
echo ""

for i in $(seq 1 100); do
    cansend can0 221#01F4000000000064
    sleep 0.02
done
echo -e "${GREEN}  ✓ 已发送 100 帧，请检查 cyber_monitor${NC}"
read -p "  按 Enter 继续下一个测试..."

# ──────────────────────────────────────────────────────────────
# 测试 5: 后车底盘状态 (0x211 = 529)
# ──────────────────────────────────────────────────────────────
echo ""
echo -e "${GREEN}═══ 测试 5: 后车底盘状态 (CAN ID 0x211) ═══${NC}"
echo ""

# vehicle_state: bytes[0], 0=NORMAL
# control_mode: bytes[1], 1=CAN_CONTROL
# battery_voltage: bytes[2](high) + bytes[3](low), Motorola, factor=0.1, unit=V
# 目标: 24.0V → raw = 240 = 0x00F0 → bytes[2]=0x00, bytes[3]=0xF0
# fault_high: bytes[4], 0=无故障
# fault_low: bytes[5], 0=无故障
# message_count: bytes[7]
# 目标: count = 42 = 0x2A → bytes[7]=0x2A
#
# 合成: Byte0=00, 1=01, 2=00, 3=F0, 4=00, 5=00, 6=00, 7=2A
echo -e "  发送: cansend can0 211#000100F00000002A"
echo -e "  预期:"
echo -e "    vehicle_state   = ${CYAN}NORMAL (0)${NC}"
echo -e "    control_mode    = ${CYAN}CAN_CONTROL (1)${NC}"
echo -e "    battery_voltage = raw(240) × 0.1 = ${CYAN}24.0V${NC}"
echo -e "    fault           = ${CYAN}0${NC}"
echo -e "    message_count   = ${CYAN}42${NC}"
echo ""

for i in $(seq 1 100); do
    cansend can0 211#000100F00000002A
    sleep 0.02
done
echo -e "${GREEN}  ✓ 已发送 100 帧，请检查 cyber_monitor${NC}"

echo ""
echo -e "${CYAN}═══════════════════════════════════════════════════════════${NC}"
echo -e "${CYAN}  所有 5 个接收报文测试完毕！${NC}"
echo -e "${CYAN}═══════════════════════════════════════════════════════════${NC}"
echo ""
echo -e "  验证要点（在 cyber_monitor 中确认）:"
echo -e "  1. speed_mps      → 应为 1.0"
echo -e "  2. steering_%     → 应为 25.0"
echo -e "  3. gear_location  → 应切换到 GEAR_DRIVE"
echo -e "  4. 后车 linear_speed → 0.5"
echo -e "  5. 后车 battery_voltage → 24.0"
echo ""
echo -e "  下发帧验证（candump can0 中确认）:"
echo -e "  0x111 = 前车EPS指令 / 后车运动指令"
echo -e "  0x223 = 前车驱动电机指令"
echo -e "  0x233 = 前车VCU状态指令"
echo -e "  0x421 = 后车控制模式指令"
