#!/usr/bin/env python3
"""
铰接车前车 CAN 通信测试脚本

测试内容：
  1. 监听 chassis_detail，打印前车反馈数据（驱动电机、EPS转向、VCU状态）
  2. 发送 ControlCommand，测试前车使能和运动控制

前车控制接口：
  - 驱动电机 (FrontAcuDrivemotor563): 速度/力矩/档位/使能
  - EPS 转向 (FrontAcuEps547): 转角/使能

前车反馈接口：
  - 驱动电机反馈 (FrontDrivemotorAcu572): 速度/力矩/档位/使能状态
  - EPS 反馈 (FrontEpsAcu556): 转角/使能/故障码
  - VCU 通用 (FrontVcuAcuGeneral524): 控制模式/遥控/故障

使用方法：
  # 先确保 canbus 模块已启动
  cyber_launch start modules/canbus/launch/canbus.launch

  # 然后运行本脚本
  # 模式1: 只监听前车反馈
  python3 test_front_vehicle.py --mode monitor

  # 模式2: 使能前车 CAN 控制模式（不发运动指令）
  python3 test_front_vehicle.py --mode enable

  # 模式3: 使能前车 + 发送低速测试指令（小心！车会动！）
  python3 test_front_vehicle.py --mode drive --speed 0.3 --steer 0.0

  # 模式4: 停止前车（发零速并切回 Standby）
  python3 test_front_vehicle.py --mode stop
"""

import argparse
import time
import sys

from cyber.python.cyber_py3 import cyber
from modules.canbus_vehicle.articulated.proto.articulated_pb2 import Articulated
from modules.common_msgs.chassis_msgs.chassis_pb2 import Chassis
from modules.common_msgs.control_msgs.control_cmd_pb2 import ControlCommand
from modules.common_msgs.control_msgs.pad_msg_pb2 import PadMessage, START, RESET

# 前车参数 (与 articulated_controller.cc 保持一致)
FRONT_STEER_RATIO = 5.074     # EPS 转向比 (标定值: 137°EPS / 27°车轮)
FRONT_MAX_EPS_DEG = 137.0     # EPS 电机最大角度 (度, 标定值)


# ============================================================================
# 监听前车反馈
# ============================================================================
def monitor_front_vehicle(duration=30):
    """监听 chassis_detail，打印前车反馈数据"""
    print("=" * 60)
    print("  前车反馈数据监听")
    print(f"  持续 {duration} 秒，Ctrl+C 提前退出")
    print("=" * 60)

    node = cyber.Node("front_vehicle_monitor")

    last_print_time = [0]

    def chassis_detail_cb(msg):
        now = time.time()
        if now - last_print_time[0] < 1.0:  # 每秒打印一次
            return
        last_print_time[0] = now

        print(f"\n--- [{time.strftime('%H:%M:%S')}] 前车 Chassis Detail ---")

        # 驱动电机反馈 (FrontDrivemotorAcu572, 0x23C)
        if msg.HasField("front_drivemotor_acu_572"):
            dm = msg.front_drivemotor_acu_572
            shift_names = {0: "P", 1: "R", 2: "N", 3: "D"}
            shift_str = shift_names.get(dm.drive_motor_shift, "?")
            speed_mps = dm.drive_motor_speed / 3.6
            print(f"  [驱动电机] 速度: {dm.drive_motor_speed:.1f} km/h "
                  f"({speed_mps:.2f} m/s), "
                  f"力矩: {dm.drive_motor_torque:.1f} Nm, "
                  f"档位: {shift_str}")
            print(f"             使能: {dm.drive_motor_enable}, "
                  f"回复: {dm.drive_motor_reply}, "
                  f"模式: {'速度' if dm.drive_motor_mode else '力矩'}, "
                  f"锁止: {dm.drive_motor_lock}, "
                  f"停车: {dm.drive_motor_stop}")
        else:
            print("  [驱动电机] 未收到 (0x23C)")

        # EPS 转向反馈 (FrontEpsAcu556, 0x22C)
        if msg.HasField("front_eps_acu_556"):
            eps = msg.front_eps_acu_556
            # EPS 角度/转向比 = 前轮转角
            wheel_angle = eps.eps_angle / FRONT_STEER_RATIO
            print(f"  [EPS 转向] EPS角度: {eps.eps_angle:.1f}°, "
                  f"前轮转角: {wheel_angle:.2f}°, "
                  f"使能: {eps.eps_enable}, "
                  f"故障码: 0x{eps.eps_error:02X}, "
                  f"角速度: {eps.eps_angle_speed:.1f}°/s")
        else:
            print("  [EPS 转向] 未收到 (0x22C)")

        # VCU 通用状态 (FrontVcuAcuGeneral524, 0x20C)
        if msg.HasField("front_vcu_acu_general_524"):
            vcu = msg.front_vcu_acu_general_524
            if vcu.acu_remote_control:
                mode_str = "遥控"
            elif vcu.acu_control_mode:
                mode_str = "自动(CAN)"
            else:
                mode_str = "手动"
            print(f"  [VCU 状态] 模式: {mode_str}, "
                  f"遥控: {vcu.acu_remote_control}, "
                  f"自动: {vcu.acu_control_mode}, "
                  f"故障: {vcu.acu_error}, "
                  f"收到数据: {vcu.acu_receive_info}")
        else:
            print("  [VCU 状态] 未收到 (0x20C)")

    node.create_reader("/apollo/canbus/chassis_detail",
                       Articulated, chassis_detail_cb)

    # 也监听标准 Chassis
    def chassis_cb(msg):
        pass

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
# 辅助: 构建 ControlCommand
# ============================================================================
def _make_cmd(speed_mps=0.0, steer_deg=0.0, gear=Chassis.GEAR_DRIVE,
              pad_action=None):
    """构建 ControlCommand
    
    Args:
        speed_mps: 前车速度 (m/s), controller 会转换为 km/h
        steer_deg: 前轮转角 (度), controller 通过 Steer() 乘以转向比
        gear: 档位
        pad_action: START 或 RESET
    """
    cmd = ControlCommand()
    cmd.header.timestamp_sec = time.time()
    cmd.header.module_name = "test_front_vehicle"
    # 后车命令设为零 (controller 会解析此字段)
    cmd.header.status.msg = (
        f"v_front=0.0,delta_front=0.0,"
        f"v_rear=0.0,delta_rear=0.0"
    )
    cmd.speed = speed_mps
    cmd.steering_target = steer_deg
    cmd.gear_location = gear
    if pad_action is not None:
        cmd.pad_msg.header.timestamp_sec = time.time()
        cmd.pad_msg.header.module_name = "test_front_vehicle"
        cmd.pad_msg.action = pad_action
    return cmd


# ============================================================================
# 发送控制指令
# ============================================================================
def send_enable():
    """发送带 pad_msg(START) 的 ControlCommand 使能自动驾驶模式"""
    print("=" * 60)
    print("  发送 EnableAutoMode (前车进入自动控制模式)")
    print("  将持续发送 3 秒确保 canbus 收到...")
    print("=" * 60)

    node = cyber.Node("front_vehicle_enable")
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
    monitor_front_vehicle(duration=5)


def send_drive(speed, steer):
    """先 enable 再发运动指令

    Args:
        speed: 前车速度 (m/s)
        steer: 前轮转角 (度)
    """
    print("=" * 60)
    print(f"  发送前车运动指令: speed={speed} m/s, steer={steer}°")
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

    node = cyber.Node("front_vehicle_driver")
    cmd_writer = node.create_writer("/apollo/control", ControlCommand)

    # 同时监听反馈
    def chassis_detail_cb(msg):
        parts = []
        if msg.HasField("front_drivemotor_acu_572"):
            dm = msg.front_drivemotor_acu_572
            speed_mps = dm.drive_motor_speed / 3.6
            parts.append(f"v={speed_mps:.2f}m/s")
            parts.append(f"en={dm.drive_motor_enable}")
        if msg.HasField("front_eps_acu_556"):
            eps = msg.front_eps_acu_556
            wheel_angle = eps.eps_angle / FRONT_STEER_RATIO
            parts.append(f"eps={eps.eps_angle:.1f}°")
            parts.append(f"wheel={wheel_angle:.1f}°")
        if msg.HasField("front_vcu_acu_general_524"):
            vcu = msg.front_vcu_acu_general_524
            if vcu.acu_control_mode:
                parts.append("模式=自动")
            elif vcu.acu_remote_control:
                parts.append("模式=遥控")
            else:
                parts.append("模式=手动")
        if parts:
            print(f"  [反馈] {', '.join(parts)}", end="\r")

    node.create_reader("/apollo/canbus/chassis_detail",
                       Articulated, chassis_detail_cb)
    time.sleep(0.5)

    # Phase 1: 先发 2 秒 enable (带 pad_msg START)
    print("\n[Phase 1] 发送 Enable (2秒)...")
    gear = Chassis.GEAR_DRIVE if speed >= 0 else Chassis.GEAR_REVERSE
    start = time.time()
    while time.time() - start < 2.0:
        cmd = _make_cmd(pad_action=START, gear=gear)
        cmd_writer.write(cmd)
        time.sleep(0.02)

    # Phase 2: 发运动指令 (不再带 pad_msg，保持 auto 模式)
    print(f"\n[Phase 2] 发送运动指令 (10秒)...")
    start = time.time()
    try:
        while not cyber.is_shutdown() and (time.time() - start < 10.0):
            cmd = _make_cmd(speed_mps=speed, steer_deg=steer, gear=gear)
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
    print("  停止前车 (零速 + 切回手动模式)")
    print("=" * 60)

    node = cyber.Node("front_vehicle_stop")
    cmd_writer = node.create_writer("/apollo/control", ControlCommand)
    time.sleep(0.5)

    # 发零速 + RESET
    for _ in range(100):
        cmd = _make_cmd(pad_action=RESET)
        cmd_writer.write(cmd)
        time.sleep(0.02)

    print("✅ 零速已发送，手动模式已请求。")


# ============================================================================
# Main
# ============================================================================
def main():
    parser = argparse.ArgumentParser(description="铰接车前车 CAN 测试")
    parser.add_argument("--mode", choices=["monitor", "enable", "drive", "stop"],
                        default="monitor",
                        help="测试模式: monitor=只监听, enable=使能CAN控制, "
                             "drive=发送运动指令, stop=停车")
    parser.add_argument("--speed", type=float, default=0.3,
                        help="drive 模式的速度 (m/s), 默认 0.3")
    parser.add_argument("--steer", type=float, default=0.0,
                        help="drive 模式的前轮转角 (度), 默认 0.0")
    parser.add_argument("--duration", type=int, default=30,
                        help="monitor 模式的持续时间 (秒), 默认 30")
    args = parser.parse_args()

    # 安全限制
    if abs(args.speed) > 1.0:
        print("⚠️  测试速度限制在 ±1.0 m/s 以内！")
        args.speed = max(-1.0, min(1.0, args.speed))
    if abs(args.steer) > 24.0:
        print("⚠️  测试前轮转角限制在 ±24.0° 以内！")
        args.steer = max(-24.0, min(24.0, args.steer))

    cyber.init()
    try:
        if args.mode == "monitor":
            monitor_front_vehicle(duration=args.duration)
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
