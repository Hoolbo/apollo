#!/usr/bin/env python3
"""
铰接车 CAN 下发帧验证脚本
通过 Cyber RT 发送 ControlCommand，然后用 candump 验证 CAN 帧编码是否正确。

用法（在容器内）：
  终端1 (已启动): cyber_launch start modules/canbus/launch/canbus.launch
  终端2 (监听):   candump can0
  终端3 (运行):   python3 sim/test_canbus_send.py
"""

import time
import math

from cyber.python.cyber_py3 import cyber
from cyber.python.cyber_py3 import cyber_time
from modules.common_msgs.control_msgs.control_cmd_pb2 import ControlCommand
from modules.common_msgs.chassis_msgs.chassis_pb2 import Chassis

def main():
    print("=" * 60)
    print("  铰接车 CAN 下发帧验证")
    print("=" * 60)
    print()
    print("请确保:")
    print("  1. canbus 已启动")
    print("  2. 另一终端运行 candump can0 监听")
    print()

    cyber.init()
    node = cyber.Node("test_canbus_send")

    ctrl_writer = node.create_writer('/apollo/control', ControlCommand)
    chassis_writer = node.create_writer('/apollo/canbus/chassis', Chassis)

    time.sleep(0.5)

    # ── 先切到自动模式 ──
    print("[0] 发送自动模式 Chassis (模拟 VCU 反馈 acu_control_mode=1)")
    print("    同时持续发送 ControlCommand")
    print()

    # ── 测试 1: 停车状态（默认值）──
    print("═══ 测试 1: 默认停车指令 ═══")
    print("  发送: speed=0, steering_target=0")
    print("  candump 预期:")
    print("    0x233 (前车驱动): bytes[5:6]=0000 (speed=0 km/h)")
    print("    0x223 (前车 EPS): bytes[5:6]= raw=1024→0x0400 → bytes[5]=00, bytes[6]=04")
    print("                     (angle=0, raw=0+1024=1024)")
    print("    0x111 (后车运动): bytes[0:1]=0000 (speed=0)")
    print("    0x421 (后车模式): bytes[0]=00 (standby)")
    print()

    for i in range(100):
        cmd = ControlCommand()
        cmd.header.timestamp_sec = cyber_time.Time.now().to_sec()
        cmd.header.module_name = 'test_canbus_send'
        cmd.speed = 0.0
        cmd.steering_target = 0.0
        cmd.gear_location = Chassis.GEAR_PARKING
        ctrl_writer.write(cmd)
        time.sleep(0.02)

    input("  请检查 candump，按 Enter 继续...")
    print()

    # ── 测试 2: 前进 + 转向 ──
    print("═══ 测试 2: 前进 1.0 m/s + 右转 10° ═══")
    print("  发送: speed=1.0, steering_target=10.0 (度)")
    print("  candump 预期:")
    print("    0x233 (前车驱动):")
    print("      speed = 1.0*3.6 = 3.6 km/h, raw = 36 = 0x0024")
    print("      → bytes[5]=0x24, bytes[6]=0x00")
    print("    0x223 (前车 EPS):")
    print("      wheel_angle=10°, eps = -10 * 6.0 = -60°")
    print("      raw = -60 + 1024 = 964 = 0x03C4")
    print("      → bytes[5]=0xC4, bytes[6]=0x03")
    print("    0x111 (后车运动):")
    print("      speed 和 steer 由 MPC 控制，此处应为 0")
    print()

    for i in range(100):
        cmd = ControlCommand()
        cmd.header.timestamp_sec = cyber_time.Time.now().to_sec()
        cmd.header.module_name = 'test_canbus_send'
        cmd.speed = 1.0
        cmd.steering_target = 10.0  # 度
        cmd.gear_location = Chassis.GEAR_DRIVE
        ctrl_writer.write(cmd)
        time.sleep(0.02)

    input("  请检查 candump，按 Enter 继续...")
    print()

    # ── 测试 3: 更大转角 ──
    print("═══ 测试 3: 前进 2.0 m/s + 左转 -15° ═══")
    print("  发送: speed=2.0, steering_target=-15.0")
    print("  candump 预期:")
    print("    0x233 (前车驱动):")
    print("      speed = 2.0*3.6 = 7.2 km/h, raw = 72 = 0x0048")
    print("      → bytes[5]=0x48, bytes[6]=0x00")
    print("    0x223 (前车 EPS):")
    print("      wheel_angle=-15°, eps = -(-15) * 6.0 = 90°")
    print("      raw = 90 + 1024 = 1114 = 0x045A")
    print("      → bytes[5]=0x5A, bytes[6]=0x04")
    print()

    for i in range(100):
        cmd = ControlCommand()
        cmd.header.timestamp_sec = cyber_time.Time.now().to_sec()
        cmd.header.module_name = 'test_canbus_send'
        cmd.speed = 2.0
        cmd.steering_target = -15.0  # 度，左转
        cmd.gear_location = Chassis.GEAR_DRIVE
        ctrl_writer.write(cmd)
        time.sleep(0.02)

    input("  请检查 candump，按 Enter 继续...")
    print()

    # ── 测试 4: 紧急停车 ──
    print("═══ 测试 4: 停止发送（回归默认 0） ═══")
    print("  停止 ControlCommand，观察 canbus 是否发送 0 值")
    print()
    time.sleep(2)

    print()
    print("═══════════════════════════════════════════════════════════")
    print("  所有下发帧测试完毕！")
    print("═══════════════════════════════════════════════════════════")
    print()
    print("  candump 验证对照表:")
    print("  ┌──────────┬──────────────────────────────────────────┐")
    print("  │ CAN ID   │ 含义                                    │")
    print("  ├──────────┼──────────────────────────────────────────┤")
    print("  │ 0x233    │ 前车驱动: bytes[5:6]=speed, [7]=档位    │")
    print("  │ 0x223    │ 前车 EPS: bytes[5:6]=angle+1024, [7]=en │")
    print("  │ 0x111    │ 后车运动: bytes[0:1]=speed, [6:7]=steer │")
    print("  │ 0x421    │ 后车模式: bytes[0]=mode                 │")
    print("  └──────────┴──────────────────────────────────────────┘")

    cyber.shutdown()


if __name__ == '__main__':
    main()
