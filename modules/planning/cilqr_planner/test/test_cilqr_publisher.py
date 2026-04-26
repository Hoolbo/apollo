#!/usr/bin/env python3
"""
CILQR Planner 测试脚本
发布假的 Localization 和 PlanningCommand，用于离线验证 CILQR planner component。

使用方法（在 Apollo 容器内）：
  1. 先启动 planner:
     cyber_launch start modules/planning/cilqr_planner/launch/cilqr_planner.launch
  2. 再运行本脚本:
     python3 modules/planning/cilqr_planner/test/test_cilqr_publisher.py
  3. 另一个终端用 cyber_monitor 查看输出:
     cyber_monitor
"""

import math
import time
import sys

from cyber.python.cyber_py3 import cyber
from cyber.python.cyber_py3 import cyber_time

# ── Protobuf 消息 ──
from modules.common_msgs.localization_msgs.localization_pb2 import LocalizationEstimate
from modules.common_msgs.planning_msgs.planning_command_pb2 import PlanningCommand

# ── 配置参数 ──
# 起始位置 — 相对坐标系（原点=圆环圆心, X∈[-419, 253], Y∈[-372, 414]）
START_X = 0.0
START_Y = 0.0
START_THETA = 0.0    # 朝向 (rad), 0=朝东
START_GAMMA = 0.0    # 铰接角

# 目标位置 — 相对坐标系（圆环圆心为原点）
GOAL_X = 60.0
GOAL_Y = 0.0
GOAL_THETA = 0.0

# Channel 名（和 cilqr_planner_config.pb.txt 一致）
LOC_TOPIC = "/apollo/localization/pose"
CMD_TOPIC = "/apollo/planning/command"
PLANNING_TOPIC = "/apollo/planning"

# 发布频率
LOC_RATE_HZ = 50  # 定位频率
CMD_SEND_ONCE = True  # 目标点只发一次


def make_localization(x, y, theta, vx=0.0, vy=0.0):
    """构造 LocalizationEstimate 消息"""
    msg = LocalizationEstimate()
    msg.header.timestamp_sec = cyber_time.Time.now().to_sec()
    msg.header.module_name = "test_publisher"

    # 位置
    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.position.z = 0.0

    # 航向
    msg.pose.heading = theta

    # 四元数 (yaw only)
    msg.pose.orientation.qw = math.cos(theta / 2.0)
    msg.pose.orientation.qx = 0.0
    msg.pose.orientation.qy = 0.0
    msg.pose.orientation.qz = math.sin(theta / 2.0)

    # 速度
    msg.pose.linear_velocity.x = vx
    msg.pose.linear_velocity.y = vy
    msg.pose.linear_velocity.z = 0.0

    return msg


def make_planning_command(goal_x, goal_y, goal_theta):
    """构造 PlanningCommand 消息，包含目标点"""
    msg = PlanningCommand()
    msg.header.timestamp_sec = cyber_time.Time.now().to_sec()
    msg.header.module_name = "test_publisher"

    # 构造 LaneFollowCommand → RoutingRequest → Waypoint
    lane_cmd = msg.lane_follow_command
    wp = lane_cmd.routing_request.waypoint.add()
    wp.pose.x = goal_x
    wp.pose.y = goal_y
    wp.heading = goal_theta

    return msg


def main():
    print("=" * 60)
    print("  CILQR Planner 测试脚本")
    print("=" * 60)
    print(f"  起始: ({START_X}, {START_Y}, {math.degrees(START_THETA):.1f}°)")
    print(f"  目标: ({GOAL_X}, {GOAL_Y}, {math.degrees(GOAL_THETA):.1f}°)")
    print(f"  定位频率: {LOC_RATE_HZ} Hz")
    print("=" * 60)

    cyber.init()
    node = cyber.Node("cilqr_test_publisher")

    # 创建 writer
    loc_writer = node.create_writer(LOC_TOPIC, LocalizationEstimate)
    cmd_writer = node.create_writer(CMD_TOPIC, PlanningCommand)

    # 创建 reader 监听 planner 输出
    trajectory_count = [0]

    def on_trajectory(msg):
        n = len(msg.trajectory_point)
        if n > 0:
            tp0 = msg.trajectory_point[0]
            trajectory_count[0] += 1
            if trajectory_count[0] % 10 == 1:
                print(f"  [收到轨迹 #{trajectory_count[0]}] "
                      f"{n} 个点, 首点=({tp0.path_point.x:.2f}, "
                      f"{tp0.path_point.y:.2f}), v={tp0.v:.2f} m/s")

    from modules.common_msgs.planning_msgs.planning_pb2 import ADCTrajectory
    node.create_reader(PLANNING_TOPIC, ADCTrajectory, on_trajectory)

    time.sleep(0.5)  # 等待连接建立

    # ── 发送目标点 ──
    print("\n[1/2] 发送目标点...")
    cmd_msg = make_planning_command(GOAL_X, GOAL_Y, GOAL_THETA)
    cmd_writer.write(cmd_msg)
    print(f"  ✓ 目标点已发送: ({GOAL_X}, {GOAL_Y})")

    # ── 循环发送定位 ──
    print(f"\n[2/2] 开始发布定位 ({LOC_RATE_HZ} Hz)...")
    print("  按 Ctrl+C 停止\n")

    dt = 1.0 / LOC_RATE_HZ
    x, y, theta = START_X, START_Y, START_THETA
    count = 0

    try:
        while not cyber.is_shutdown():
            loc_msg = make_localization(x, y, theta)
            loc_writer.write(loc_msg)

            count += 1
            if count % (LOC_RATE_HZ * 2) == 0:  # 每2秒打印一次
                print(f"  [定位 #{count}] pos=({x:.2f}, {y:.2f}), "
                      f"θ={math.degrees(theta):.1f}°, "
                      f"已收到 {trajectory_count[0]} 条轨迹")

            # 如果收到了轨迹输出，说明 planner 在工作
            if trajectory_count[0] >= 3 and count > LOC_RATE_HZ * 5:
                print(f"\n  ✓ 测试成功! 已收到 {trajectory_count[0]} 条轨迹输出")
                break

            time.sleep(dt)

    except KeyboardInterrupt:
        print(f"\n  停止. 共收到 {trajectory_count[0]} 条轨迹.")

    cyber.shutdown()


if __name__ == "__main__":
    main()
