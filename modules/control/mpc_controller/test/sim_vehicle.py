#!/usr/bin/env python3
"""
铰接车运动学仿真器 — 闭环测试用

形成闭环：
  CILQR Planner → MPC Controller → 本仿真器 → Localization → Planner...

使用方法（在容器内，按顺序启动 3 个终端）：
  1. cyber_launch start modules/planning/cilqr_planner/launch/cilqr_planner.launch
  2. python3 modules/control/mpc_controller/mpc_controller_node.py
  3. python3 modules/control/mpc_controller/test/sim_vehicle.py
"""

import math
import time
import re
import numpy as np

from cyber.python.cyber_py3 import cyber
from cyber.python.cyber_py3 import cyber_time

from modules.common_msgs.localization_msgs.localization_pb2 import LocalizationEstimate
from modules.common_msgs.control_msgs.control_cmd_pb2 import ControlCommand
from modules.common_msgs.planning_msgs.planning_command_pb2 import PlanningCommand
from modules.common_msgs.chassis_msgs.chassis_pb2 import Chassis


# ── 仿真参数 ──
SIM_DT = 0.02        # 仿真步长 50Hz
LOC_PUBLISH_HZ = 50  # 定位发布频率

# ── 初始状态 ──
INIT_X = 0.0
INIT_Y = 0.0
INIT_THETA_FRONT = 0.0
INIT_THETA_REAR = 0.0

# ── 目标点（圆环圆心为原点）──
GOAL_X = 60.0
GOAL_Y = 0.0
GOAL_THETA = -1.57

# ── 车辆参数（和 vehicle.json 一致）──
L_WB_FRONT = 0.90    # 前车轴距
L_WB_REAR = 0.90     # 后车轴距
LF = 0.45            # 前车中心到铰接点
LR = 0.45            # 后车中心到铰接点


class ArticulatedVehicleSim:
    """双 Ackermann 铰接车运动学仿真"""

    def __init__(self):
        # 状态: [x_front, y_front, theta_front, theta_rear]
        self.x = INIT_X
        self.y = INIT_Y
        self.theta_front = INIT_THETA_FRONT
        self.theta_rear = INIT_THETA_REAR

        # 控制输入（从 ControlCommand 读取）
        self.v_front = 0.0
        self.delta_front = 0.0
        self.v_rear = 0.0
        self.delta_rear = 0.0

    @property
    def gamma(self):
        """铰接角 = 前车航向 - 后车航向"""
        return self.theta_front - self.theta_rear

    def step(self, dt):
        """单步运动学更新（Ackermann 模型）"""
        # 前车 Ackermann
        if abs(self.v_front) > 1e-6:
            omega_front = self.v_front * math.tan(self.delta_front) / L_WB_FRONT
        else:
            omega_front = 0.0

        self.x += self.v_front * math.cos(self.theta_front) * dt
        self.y += self.v_front * math.sin(self.theta_front) * dt
        self.theta_front += omega_front * dt

        # 后车 Ackermann
        if abs(self.v_rear) > 1e-6:
            omega_rear = self.v_rear * math.tan(self.delta_rear) / L_WB_REAR
        else:
            omega_rear = 0.0

        self.theta_rear += omega_rear * dt

        # 角度归一化
        self.theta_front = self._wrap(self.theta_front)
        self.theta_rear = self._wrap(self.theta_rear)

    @staticmethod
    def _wrap(a):
        while a > math.pi:
            a -= 2 * math.pi
        while a < -math.pi:
            a += 2 * math.pi
        return a

    def set_control(self, v_front, delta_front, v_rear, delta_rear):
        self.v_front = v_front
        self.delta_front = delta_front
        self.v_rear = v_rear
        self.delta_rear = delta_rear


def parse_debug_string(msg_str):
    """从 ControlCommand.header.status.msg 解析 4 路控制量"""
    result = {'v_front': 0.0, 'delta_front': 0.0,
              'v_rear': 0.0, 'delta_rear': 0.0}
    for item in msg_str.split(','):
        m = re.match(r'(\w+)=([-\d.eE+]+)', item.strip())
        if m:
            key, val = m.group(1), float(m.group(2))
            if key in result:
                result[key] = val
    return result


def main():
    print("=" * 60)
    print("  铰接车运动学仿真器")
    print("=" * 60)
    print(f"  初始: ({INIT_X}, {INIT_Y}), θ={math.degrees(INIT_THETA_FRONT):.1f}°")
    print(f"  目标: ({GOAL_X}, {GOAL_Y})")
    print(f"  仿真频率: {1/SIM_DT:.0f} Hz")
    print("=" * 60)

    cyber.init()
    node = cyber.Node("sim_vehicle")

    # Publishers
    loc_writer = node.create_writer(
        '/apollo/localization/pose', LocalizationEstimate)
    cmd_writer = node.create_writer(
        '/apollo/planning/command', PlanningCommand)
    chassis_writer = node.create_writer(
        '/apollo/canbus/chassis', Chassis)

    # Vehicle sim
    vehicle = ArticulatedVehicleSim()

    # Subscribe to control commands
    def on_control(msg):
        # 优先从 debug string 解析完整 4 路指令
        debug_str = msg.header.status.msg if msg.header.HasField('status') else ''
        if debug_str and 'v_front' in debug_str:
            ctrl = parse_debug_string(debug_str)
            vehicle.set_control(
                ctrl['v_front'], ctrl['delta_front'],
                ctrl['v_rear'], ctrl['delta_rear'])
        else:
            # Fallback: 只有前车指令
            vehicle.set_control(
                msg.speed, math.radians(msg.steering_target),
                msg.speed, 0.0)

    node.create_reader('/apollo/control', ControlCommand, on_control)

    time.sleep(0.5)

    # 发送目标点
    print("\n[1/2] 发送目标点...")
    goal_msg = PlanningCommand()
    goal_msg.header.timestamp_sec = cyber_time.Time.now().to_sec()
    goal_msg.header.module_name = 'sim_vehicle'
    wp = goal_msg.lane_follow_command.routing_request.waypoint.add()
    wp.pose.x = GOAL_X
    wp.pose.y = GOAL_Y
    wp.heading = GOAL_THETA
    cmd_writer.write(goal_msg)
    print(f"  ✓ 目标点: ({GOAL_X}, {GOAL_Y})")

    # 主循环
    print(f"\n[2/2] 仿真运行中...")
    print("  按 Ctrl+C 停止\n")

    tick = 0
    try:
        while not cyber.is_shutdown():
            # 运动学更新
            vehicle.step(SIM_DT)

            # 发布 localization
            loc = LocalizationEstimate()
            loc.header.timestamp_sec = cyber_time.Time.now().to_sec()
            loc.header.module_name = 'sim_vehicle'
            loc.pose.position.x = vehicle.x
            loc.pose.position.y = vehicle.y
            loc.pose.position.z = 0.0
            loc.pose.heading = vehicle.theta_front

            # 四元数
            half = vehicle.theta_front / 2.0
            loc.pose.orientation.qw = math.cos(half)
            loc.pose.orientation.qx = 0.0
            loc.pose.orientation.qy = 0.0
            loc.pose.orientation.qz = math.sin(half)

            # 速度
            loc.pose.linear_velocity.x = vehicle.v_front * math.cos(vehicle.theta_front)
            loc.pose.linear_velocity.y = vehicle.v_front * math.sin(vehicle.theta_front)

            loc_writer.write(loc)

            # 发布铰接角 γ（用 Chassis.steering_percentage 字段承载）
            chassis_msg = Chassis()
            chassis_msg.header.timestamp_sec = cyber_time.Time.now().to_sec()
            chassis_msg.header.module_name = 'sim_vehicle'
            chassis_msg.steering_percentage = math.degrees(vehicle.gamma)  # deg
            chassis_msg.speed_mps = vehicle.v_front
            chassis_writer.write(chassis_msg)

            tick += 1
            if tick % (LOC_PUBLISH_HZ * 2) == 0:  # 每 2 秒
                dist = math.sqrt((vehicle.x - GOAL_X)**2 + (vehicle.y - GOAL_Y)**2)
                print(f"  [{tick/LOC_PUBLISH_HZ:.0f}s] "
                      f"pos=({vehicle.x:.2f}, {vehicle.y:.2f}) "
                      f"θ={math.degrees(vehicle.theta_front):.1f}° "
                      f"γ={math.degrees(vehicle.gamma):.1f}° "
                      f"v={vehicle.v_front:.2f} m/s "
                      f"dist={dist:.1f}m")

                if dist < 1.0:
                    print(f"\n  ✓ 到达目标！距离 {dist:.2f}m")
                    break

            time.sleep(SIM_DT)

    except KeyboardInterrupt:
        print(f"\n  停止. 最终位置: ({vehicle.x:.2f}, {vehicle.y:.2f})")

    cyber.shutdown()


if __name__ == '__main__':
    main()
