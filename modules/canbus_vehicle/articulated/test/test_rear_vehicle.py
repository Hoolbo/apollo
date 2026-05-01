#!/usr/bin/env python3
"""
铰接车后车 (Hunter SE) CAN 通信测试脚本

测试内容：
  1. 监听 chassis_detail，打印后车反馈数据（速度、转角、电池、电机状态等）
  2. 发送 ControlCommand，测试后车使能和运动控制

使用方法：
  # 先确保 canbus 模块已启动
  cyber_launch start modules/canbus/launch/canbus.launch

  # 然后运行本脚本
  # 模式1: 只监听后车反馈
  python3 test_rear_vehicle.py --mode monitor

  # 模式2: 使能后车 CAN 控制模式（不发运动指令）
  python3 test_rear_vehicle.py --mode enable

  # 模式3: 使能后车 + 发送低速测试指令（小心！车会动！）
  python3 test_rear_vehicle.py --mode drive --speed 0.3 --steer 0.0

  # 模式4: 停止后车（发零速并切回 Standby）
  python3 test_rear_vehicle.py --mode stop
"""

import argparse
import time
import sys

from cyber.python.cyber_py3 import cyber
from modules.canbus_vehicle.articulated.proto.articulated_pb2 import Articulated
from modules.common_msgs.chassis_msgs.chassis_pb2 import Chassis
from modules.common_msgs.control_msgs.control_cmd_pb2 import ControlCommand
from modules.common_msgs.control_msgs.pad_msg_pb2 import PadMessage, START, RESET


# ============================================================================
# 监听后车反馈
# ============================================================================
def monitor_rear_vehicle(duration=30):
    """监听 chassis_detail 和 chassis，打印后车反馈数据"""
    print("=" * 60)
    print("  后车 (Hunter SE) 反馈数据监听")
    print(f"  持续 {duration} 秒，Ctrl+C 提前退出")
    print("=" * 60)

    node = cyber.Node("rear_vehicle_monitor")

    last_print_time = [0]

    def chassis_detail_cb(msg):
        now = time.time()
        if now - last_print_time[0] < 1.0:  # 每秒打印一次
            return
        last_print_time[0] = now

        print(f"\n--- [{time.strftime('%H:%M:%S')}] Chassis Detail ---")

        # 后车运动反馈 (0x221)
        if msg.HasField("rear_motion_feedback_545"):
            fb = msg.rear_motion_feedback_545
            print(f"  [运动反馈] 线速度: {fb.linear_speed:.3f} m/s, "
                  f"转向角: {fb.steering_angle:.3f} rad")
        else:
            print("  [运动反馈] 未收到 (0x221)")

        # 后车底盘状态 (0x211)
        if msg.HasField("rear_chassis_status_529"):
            cs = msg.rear_chassis_status_529
            state_names = {0: "正常", 1: "急停", 2: "系统故障"}
            mode_names = {0: "待机", 1: "CAN控制", 2: "遥控"}
            print(f"  [底盘状态] 状态: {state_names.get(cs.vehicle_state, '?')}, "
                  f"模式: {mode_names.get(cs.control_mode, '?')}, "
                  f"电池: {cs.battery_voltage:.1f}V, "
                  f"故障H:{cs.fault_high} L:{cs.fault_low}")
        else:
            print("  [底盘状态] 未收到 (0x211)")

        # 后车电机反馈 High (0x251-253)
        for i, field in enumerate([
            "rear_motor_feedback_high_1_593",
            "rear_motor_feedback_high_2_594",
            "rear_motor_feedback_high_3_595"
        ], 1):
            if msg.HasField(field):
                mf = getattr(msg, field)
                speed = getattr(mf, f"motor{i}_speed")
                current = getattr(mf, f"motor{i}_current")
                print(f"  [电机{i} High] 转速: {speed} RPM, 电流: {current:.1f} A")

        # 后车电机反馈 Low (0x261-263)
        for i, field in enumerate([
            "rear_motor_feedback_low_1_609",
            "rear_motor_feedback_low_2_610",
            "rear_motor_feedback_low_3_611"
        ], 1):
            if msg.HasField(field):
                mf = getattr(msg, field)
                voltage = getattr(mf, f"driver_voltage_{i}")
                d_temp = getattr(mf, f"driver_temp_{i}")
                m_temp = getattr(mf, f"motor_temp_{i}")
                status = getattr(mf, f"driver_status_{i}")
                print(f"  [电机{i} Low ] 电压: {voltage:.1f}V, "
                      f"驱动温度: {d_temp}°C, 电机温度: {m_temp}°C, "
                      f"状态: 0x{status:02X}")

    node.create_reader("/apollo/canbus/chassis_detail",
                       Articulated, chassis_detail_cb)

    # 也监听标准 Chassis
    def chassis_cb(msg):
        pass  # chassis_detail 已包含所有信息

    node.create_reader("/apollo/canbus/chassis", Chassis, chassis_cb)

    print("\n等待数据...")
    start = time.time()
    try:
        while not cyber.is_shutdown() and (time.time() - start < duration):
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    print("\n监听结束。")


# ============================================================================
# 辅助: 构建带有 pad_msg 的 ControlCommand
# ============================================================================
def _make_cmd(v_rear=0.0, delta_rear=0.0, pad_action=None):
    """构建 ControlCommand，可选嵌入 pad_msg（时间戳实时）"""
    cmd = ControlCommand()
    cmd.header.timestamp_sec = time.time()
    cmd.header.module_name = "test_rear_vehicle"
    cmd.header.status.msg = (
        f"v_front=0.0,delta_front=0.0,"
        f"v_rear={v_rear},delta_rear={delta_rear}"
    )
    cmd.speed = 0.0
    cmd.steering_target = 0.0
    cmd.gear_location = Chassis.GEAR_PARKING
    if pad_action is not None:
        cmd.pad_msg.header.timestamp_sec = time.time()
        cmd.pad_msg.header.module_name = "test_rear_vehicle"
        cmd.pad_msg.action = pad_action
    return cmd


# ============================================================================
# 发送控制指令
# ============================================================================
def send_enable():
    """发送带 pad_msg(START) 的 ControlCommand 使能自动驾驶模式"""
    print("=" * 60)
    print("  发送 EnableAutoMode (后车进入 CAN 控制模式)")
    print("  将持续发送 3 秒确保 canbus 收到...")
    print("=" * 60)

    node = cyber.Node("rear_vehicle_enable")
    cmd_writer = node.create_writer("/apollo/control", ControlCommand)
    time.sleep(0.5)

    # 持续发送 3 秒 (pad_msg 时间戳每次都刷新)
    start = time.time()
    count = 0
    while time.time() - start < 3.0:
        cmd = _make_cmd(pad_action=START)
        cmd_writer.write(cmd)
        count += 1
        time.sleep(0.02)  # 50Hz

    print(f"✅ 已发送 {count} 条带 pad_msg(START) 的 ControlCommand")
    print("   等待 canbus 切换模式...")

    # 监听反馈确认模式是否切换
    time.sleep(0.5)
    monitor_rear_vehicle(duration=5)


def send_drive(speed, steer):
    """先 enable 再发运动指令（全部通过 ControlCommand + 嵌入 pad_msg）"""
    print("=" * 60)
    print(f"  发送后车运动指令: speed={speed} m/s, steer={steer} rad")
    print("  ⚠️  车辆将会移动！确保安全！")
    print("  5 秒后开始发送，Ctrl+C 取消...")
    print("=" * 60)

    try:
        for i in range(5, 0, -1):
            print(f"  {i}...")
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n已取消。")
        return

    node = cyber.Node("rear_vehicle_driver")
    cmd_writer = node.create_writer("/apollo/control", ControlCommand)

    # 同时监听反馈
    def chassis_detail_cb(msg):
        if msg.HasField("rear_motion_feedback_545"):
            fb = msg.rear_motion_feedback_545
            if msg.HasField("rear_chassis_status_529"):
                cs = msg.rear_chassis_status_529
                mode_names = {0: "待机", 1: "CAN", 2: "遥控"}
                mode_str = mode_names.get(cs.control_mode, "?")
            else:
                mode_str = "?"
            print(f"  [反馈] v={fb.linear_speed:.3f} m/s, "
                  f"δ={fb.steering_angle:.3f} rad, "
                  f"模式={mode_str}", end="\r")

    node.create_reader("/apollo/canbus/chassis_detail",
                       Articulated, chassis_detail_cb)
    time.sleep(0.5)

    # Phase 1: 先发 2 秒 enable (带 pad_msg START)
    print("\n[Phase 1] 发送 Enable (2秒)...")
    start = time.time()
    while time.time() - start < 2.0:
        cmd = _make_cmd(pad_action=START)
        cmd_writer.write(cmd)
        time.sleep(0.02)

    # Phase 2: 发运动指令 (不再带 pad_msg，保持 auto 模式)
    print(f"\n[Phase 2] 发送运动指令 (10秒)...")
    start = time.time()
    try:
        while not cyber.is_shutdown() and (time.time() - start < 10.0):
            cmd = _make_cmd(v_rear=speed, delta_rear=steer)
            cmd_writer.write(cmd)
            time.sleep(0.02)
    except KeyboardInterrupt:
        pass

    # Phase 3: 零速停车
    print("\n\n[Phase 3] 发送零速停车...")
    for _ in range(50):
        cmd = _make_cmd()
        cmd_writer.write(cmd)
        time.sleep(0.02)

    print("✅ 已停止。")


def send_stop():
    """发送零速 + 切回 Standby（通过 ControlCommand 嵌入 pad_msg RESET）"""
    print("=" * 60)
    print("  停止后车 (零速 + 切回 Standby)")
    print("=" * 60)

    node = cyber.Node("rear_vehicle_stop")
    cmd_writer = node.create_writer("/apollo/control", ControlCommand)
    time.sleep(0.5)

    # 发零速 + RESET
    for _ in range(100):
        cmd = _make_cmd(pad_action=RESET)
        cmd_writer.write(cmd)
        time.sleep(0.02)

    print("✅ 零速已发送，Standby 模式已请求。")


# ============================================================================
# Main
# ============================================================================
def main():
    parser = argparse.ArgumentParser(description="铰接车后车 (Hunter SE) CAN 测试")
    parser.add_argument("--mode", choices=["monitor", "enable", "drive", "stop"],
                        default="monitor",
                        help="测试模式: monitor=只监听, enable=使能CAN控制, "
                             "drive=发送运动指令, stop=停车")
    parser.add_argument("--speed", type=float, default=0.3,
                        help="drive 模式的速度 (m/s), 默认 0.3")
    parser.add_argument("--steer", type=float, default=0.0,
                        help="drive 模式的转向 (rad), 默认 0.0")
    parser.add_argument("--duration", type=int, default=30,
                        help="monitor 模式的持续时间 (秒), 默认 30")
    args = parser.parse_args()

    # 安全限制
    if abs(args.speed) > 1.0:
        print("⚠️  测试速度限制在 ±1.0 m/s 以内！")
        args.speed = max(-1.0, min(1.0, args.speed))
    if abs(args.steer) > 0.3:
        print("⚠️  测试转向限制在 ±0.3 rad 以内！")
        args.steer = max(-0.3, min(0.3, args.steer))

    cyber.init()
    try:
        if args.mode == "monitor":
            monitor_rear_vehicle(duration=args.duration)
        elif args.mode == "enable":
            send_enable()
        elif args.mode == "drive":
            send_drive(args.speed, args.steer)
        elif args.mode == "stop":
            send_stop()
    finally:
        cyber.shutdown()


if __name__ == "__main__":
    main()
