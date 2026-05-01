#!/usr/bin/env python3
"""
EPS 转向标定脚本 — 递增发送 EPS 角度，找到机械极限

用法：
  # 先确保 canbus 已启动，前车物理按键切到自动模式
  python3 modules/canbus_vehicle/articulated/test/calibrate_eps.py

  # 指定方向 (positive=右转, negative=左转)
  python3 modules/canbus_vehicle/articulated/test/calibrate_eps.py --direction positive
  python3 modules/canbus_vehicle/articulated/test/calibrate_eps.py --direction negative
"""

import argparse
import time
import sys

from cyber.python.cyber_py3 import cyber
from modules.canbus_vehicle.articulated.proto.articulated_pb2 import Articulated
from modules.common_msgs.chassis_msgs.chassis_pb2 import Chassis
from modules.common_msgs.control_msgs.control_cmd_pb2 import ControlCommand
from modules.common_msgs.control_msgs.pad_msg_pb2 import PadMessage, START


def make_cmd(steer_deg, pad_action=None):
    cmd = ControlCommand()
    cmd.header.timestamp_sec = time.time()
    cmd.header.module_name = "calibrate_eps"
    cmd.header.status.msg = "v_front=0.0,delta_front=0.0,v_rear=0.0,delta_rear=0.0"
    cmd.speed = 0.0
    cmd.steering_target = steer_deg
    cmd.gear_location = Chassis.GEAR_DRIVE
    if pad_action is not None:
        cmd.pad_msg.header.timestamp_sec = time.time()
        cmd.pad_msg.header.module_name = "calibrate_eps"
        cmd.pad_msg.action = pad_action
    return cmd


def main():
    parser = argparse.ArgumentParser(description="EPS 转向标定")
    parser.add_argument("--direction", choices=["positive", "negative"],
                        default="positive",
                        help="positive=右转(EPS正值), negative=左转(EPS负值)")
    parser.add_argument("--max-angle", type=float, default=200.0,
                        help="最大测试角度 (度), 默认 200")
    parser.add_argument("--step", type=float, default=10.0,
                        help="每步递增角度 (度), 默认 10")
    parser.add_argument("--hold-time", type=float, default=3.0,
                        help="每步保持时间 (秒), 默认 3")
    args = parser.parse_args()

    sign = 1.0 if args.direction == "positive" else -1.0
    dir_name = "右转(+)" if sign > 0 else "左转(-)"

    print("=" * 60)
    print(f"  EPS 转向标定 — {dir_name}")
    print(f"  范围: 0° → {args.max_angle}°, 步长: {args.step}°")
    print(f"  每步保持 {args.hold_time} 秒")
    print(f"  5 秒后开始，Ctrl+C 取消...")
    print("=" * 60)

    try:
        for i in range(5, 0, -1):
            print(f"  {i}...")
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n已取消。")
        return

    cyber.init()

    try:
        node = cyber.Node("calibrate_eps")
        cmd_writer = node.create_writer("/apollo/control", ControlCommand)

        # 实时 EPS 反馈
        eps_feedback = [0.0]

        def chassis_detail_cb(msg):
            if msg.HasField("front_eps_acu_556"):
                eps_feedback[0] = msg.front_eps_acu_556.eps_angle

        node.create_reader("/apollo/canbus/chassis_detail",
                           Articulated, chassis_detail_cb)
        time.sleep(0.5)

        # Phase 1: Enable (2秒)
        print("\n[Enable] 发送 pad_msg(START) 2秒...")
        start = time.time()
        while time.time() - start < 2.0:
            cmd = make_cmd(0.0, pad_action=START)
            cmd_writer.write(cmd)
            time.sleep(0.02)

        print(f"  当前 EPS 反馈: {eps_feedback[0]:.1f}°\n")

        # Phase 2: 递增角度
        print(f"{'命令角度':>10} | {'EPS反馈':>10} | {'状态'}")
        print("-" * 40)

        prev_feedback = None
        stable_count = 0
        max_eps_reached = 0.0
        target = args.step

        while target <= args.max_angle:
            cmd_angle = sign * target

            # 保持发送 hold_time 秒
            start = time.time()
            while time.time() - start < args.hold_time:
                cmd = make_cmd(cmd_angle)
                cmd_writer.write(cmd)
                time.sleep(0.02)

            fb = eps_feedback[0]
            abs_fb = abs(fb)

            # 判断是否到达极限
            if prev_feedback is not None and abs(abs_fb - abs(prev_feedback)) < 2.0:
                stable_count += 1
                status = f"⚠️  不再增加 ({stable_count}/3)"
            else:
                stable_count = 0
                status = "✓ 正常"

            if abs_fb > abs(max_eps_reached):
                max_eps_reached = fb

            print(f"  {cmd_angle:>+8.1f}° | {fb:>+8.1f}° | {status}")

            prev_feedback = fb

            if stable_count >= 3:
                print(f"\n🔴 EPS 已到达机械极限！最大 EPS = {max_eps_reached:+.1f}°")
                break

            target += args.step

        # Phase 3: 保持在极限位置，等用户量完
        max_cmd = sign * target  # 保持最后一个命令角度
        print(f"\n⏳ 保持发送 {max_cmd:+.1f}°，请量前轮实际转角...")
        print(f"   量好后按 Enter 回零，或 Ctrl+C 紧急停车")

        import threading
        user_done = [False]

        def wait_input():
            input()
            user_done[0] = True

        t = threading.Thread(target=wait_input, daemon=True)
        t.start()

        while not user_done[0] and not cyber.is_shutdown():
            cmd = make_cmd(max_cmd)
            cmd_writer.write(cmd)
            time.sleep(0.02)

        # Phase 4: 回零
        print(f"\n[回零] 发送 0° ...")
        start = time.time()
        while time.time() - start < 2.0:
            cmd = make_cmd(0.0)
            cmd_writer.write(cmd)
            time.sleep(0.02)

        print(f"\n{'='*60}")
        print(f"  结果: {dir_name} 最大 EPS 角度 = {max_eps_reached:+.1f}°")
        print(f"  steer_ratio = {abs(max_eps_reached):.1f} / 实际前轮转角")
        print(f"{'='*60}")

    except KeyboardInterrupt:
        # 紧急停车
        print("\n\n[紧急停车] 发送 0°...")
        for _ in range(50):
            cmd = make_cmd(0.0)
            cmd_writer.write(cmd)
            time.sleep(0.02)
        print("已停止。")
    finally:
        cyber.shutdown()


if __name__ == "__main__":
    main()
