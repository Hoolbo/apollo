#!/usr/bin/env python3
"""
实时可视化 CILQR 规划轨迹 + 车辆位置

在容器内运行（需要 X11 转发或 Xvfb + 保存到文件模式）：
  python3 modules/control/mpc_controller/test/plot_trajectory.py

如果没有显示器，自动保存到 /tmp/planner/trajectory.png
"""

import time
import math
import numpy as np

from cyber.python.cyber_py3 import cyber

from modules.common_msgs.localization_msgs.localization_pb2 import LocalizationEstimate
from modules.common_msgs.planning_msgs.planning_pb2 import ADCTrajectory

# 尝试 matplotlib
try:
    import matplotlib
    matplotlib.use('Agg')  # 无显示器模式
    import matplotlib.pyplot as plt
    HAS_MPL = True
except ImportError:
    HAS_MPL = False
    print("Warning: matplotlib not available, text-only mode")

# ── 全局数据 ──
vehicle_trail = []       # 车辆历史轨迹 [(x,y), ...]
latest_traj_pts = []     # 最新 planning 轨迹点 [(x,y), ...]
global_traj_pts = []     # 混合A* 规划出的全局路径 [(x,y), ...]
vehicle_pos = None       # 当前位置 (x, y, theta)
plot_counter = 0


def on_localization(msg):
    global vehicle_pos
    x = msg.pose.position.x
    y = msg.pose.position.y
    theta = msg.pose.heading
    vehicle_pos = (x, y, theta)
    vehicle_trail.append((x, y))
    # 保留最近 2000 个点
    if len(vehicle_trail) > 2000:
        vehicle_trail.pop(0)


def on_trajectory(msg):
    global latest_traj_pts
    pts = []
    for tp in msg.trajectory_point:
        pts.append((tp.path_point.x, tp.path_point.y))
    latest_traj_pts = pts


def on_global_trajectory(msg):
    global global_traj_pts
    pts = []
    for tp in msg.trajectory_point:
        pts.append((tp.path_point.x, tp.path_point.y))
    global_traj_pts = pts


def save_plot():
    global plot_counter
    if not HAS_MPL:
        return

    fig, ax = plt.subplots(1, 1, figsize=(12, 8))
    ax.set_aspect('equal')
    ax.set_facecolor('#1a1a2e')
    ax.grid(True, alpha=0.2, color='white')

    # 车辆历史轨迹（白色）
    if len(vehicle_trail) > 1:
        trail = np.array(vehicle_trail)
        ax.plot(trail[:, 0], trail[:, 1], 'w-', linewidth=1.5,
                alpha=0.6, label='Vehicle Trail')

    # Global 轨迹（深黄色/橙色虚线）
    if len(global_traj_pts) > 1:
        gtraj = np.array(global_traj_pts)
        ax.plot(gtraj[:, 0], gtraj[:, 1], '--', color='#ff9900',
                linewidth=2.0, alpha=0.8, label='Hybrid A* Global Path')

    # Planning 轨迹（蓝绿色）
    if len(latest_traj_pts) > 1:
        traj = np.array(latest_traj_pts)
        ax.plot(traj[:, 0], traj[:, 1], '-', color='#00d2ff',
                linewidth=2.5, label='CILQR Trajectory')
        ax.plot(traj[0, 0], traj[0, 1], 'o', color='#00ff88',
                markersize=8, label='Traj Start')
        ax.plot(traj[-1, 0], traj[-1, 1], 's', color='#ff4444',
                markersize=8, label='Traj End')

    # 当前位置（黄色三角）
    if vehicle_pos is not None:
        x, y, theta = vehicle_pos
        ax.plot(x, y, '^', color='#ffcc00', markersize=12, label='Vehicle')
        # 航向箭头
        dx = 1.5 * math.cos(theta)
        dy = 1.5 * math.sin(theta)
        ax.annotate('', xy=(x + dx, y + dy), xytext=(x, y),
                    arrowprops=dict(arrowstyle='->', color='#ffcc00', lw=2))

    ax.legend(loc='upper left', fontsize=9, facecolor='#16213e',
              edgecolor='white', labelcolor='white')
    ax.set_xlabel('X (m)', color='white')
    ax.set_ylabel('Y (m)', color='white')
    ax.tick_params(colors='white')
    ax.set_title(f'CILQR Planning Visualization  [frame {plot_counter}]',
                 color='white', fontsize=14)

    for spine in ax.spines.values():
        spine.set_edgecolor('#333')

    fig.patch.set_facecolor('#0f0f23')
    plt.tight_layout()

    path = f'/apollo_workspace/modules/control/mpc_controller/test/output/trajectory_{plot_counter:04d}.png'
    plt.savefig(path, dpi=100, facecolor=fig.get_facecolor())
    plt.close(fig)
    plot_counter += 1


def main():
    print("=" * 50)
    print("  CILQR Trajectory Visualizer")
    print("=" * 50)
    if not HAS_MPL:
        print("  matplotlib not found, install: pip3 install matplotlib")
        return

    cyber.init()
    node = cyber.Node("trajectory_visualizer")

    node.create_reader('/apollo/localization/pose',
                       LocalizationEstimate, on_localization)
    node.create_reader('/apollo/planning',
                       ADCTrajectory, on_trajectory)
    node.create_reader('/apollo/planning/global_path',
                       ADCTrajectory, on_global_trajectory)

    print("  Saving to /tmp/planner/trajectory.png every 2s")
    print("  Press Ctrl+C to stop\n")

    import os
    os.makedirs('/apollo_workspace/modules/control/mpc_controller/test/output', exist_ok=True)

    try:
        while not cyber.is_shutdown():
            time.sleep(2.0)
            save_plot()
            if vehicle_pos:
                x, y, theta = vehicle_pos
                n_trail = len(vehicle_trail)
                n_traj = len(latest_traj_pts)
                print(f"  [{plot_counter}] pos=({x:.2f}, {y:.2f}) "
                      f"θ={math.degrees(theta):.1f}° "
                      f"trail={n_trail} traj_pts={n_traj} "
                      f"→ /tmp/planner/trajectory.png")
    except KeyboardInterrupt:
        print("\n  Final plot saved.")

    cyber.shutdown()


if __name__ == '__main__':
    main()
