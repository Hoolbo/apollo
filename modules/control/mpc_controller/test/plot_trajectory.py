#!/usr/bin/env python3
"""
实时可视化 CILQR 规划轨迹 + 车辆位置 + 栅格地图背景

在容器内运行（需要 X11 转发或 Xvfb + 保存到文件模式）：
  python3 modules/control/mpc_controller/test/plot_trajectory.py

如果没有显示器，自动保存到 /tmp/planner/trajectory.png
"""

import time
import math
import json
import os
import numpy as np

from cyber.python.cyber_py3 import cyber

from modules.common_msgs.localization_msgs.localization_pb2 import LocalizationEstimate
from modules.common_msgs.planning_msgs.planning_pb2 import ADCTrajectory

# 尝试 matplotlib
try:
    import matplotlib
    matplotlib.use('Agg')  # 无显示器模式
    import matplotlib.pyplot as plt
    from matplotlib.colors import ListedColormap
    HAS_MPL = True
except ImportError:
    HAS_MPL = False
    print("Warning: matplotlib not available, text-only mode")

# ── 地图配置 ──
# 从 planner 配置中读取地图路径
PLANNER_CONFIG = 'modules/planning/cilqr_planner/conf/cilqr_planner_config.pb.txt'
MAP_PADDING = 20.0  # 地图裁剪时在轨迹包围盒外扩展的米数

# ── 全局数据 ──
vehicle_trail = []       # 车辆历史轨迹 [(x,y), ...]
latest_traj_pts = []     # 最新 planning 轨迹点 [(x,y), ...]
global_traj_pts = []     # 混合A* 规划出的全局路径 [(x,y), ...]
vehicle_pos = None       # 当前位置 (x, y, theta)
plot_counter = 0
map_data = None          # 加载的地图数据 dict


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


def load_grid_map():
    """从 planner 配置中读取地图路径并加载 JSON 栅格地图"""
    global map_data
    try:
        # 读取 planner 配置获取地图路径
        map_path = None
        if os.path.exists(PLANNER_CONFIG):
            with open(PLANNER_CONFIG, 'r') as f:
                for line in f:
                    if 'map_file' in line:
                        # 解析 protobuf text: map_file: "path/to/map.json"
                        map_path = line.split('"')[1].strip()
                        break

        if not map_path or not os.path.exists(map_path):
            print(f"  ⚠ 地图文件未找到: {map_path}")
            return

        print(f"  加载地图: {map_path} ...")
        with open(map_path, 'r') as f:
            raw = json.load(f)

        # 解析 metadata
        meta = raw.get('metadata', raw)
        dims = meta.get('dimensions', meta)
        width = dims.get('width', meta.get('width', 0))
        height = dims.get('height', meta.get('height', 0))
        resolution = dims.get('resolution', meta.get('resolution', 1.0))
        origin = meta.get('origin', [0.0, 0.0])

        # 构建 numpy 数组: data[row][col], row=y, col=x
        grid = np.array(raw['data'], dtype=np.float32)

        map_data = {
            'grid': grid,
            'width': width,
            'height': height,
            'resolution': resolution,
            'origin_x': origin[0],
            'origin_y': origin[1],
        }
        print(f"  ✓ 地图加载成功: {width}×{height}, res={resolution}m, "
              f"origin=({origin[0]:.1f}, {origin[1]:.1f})")

    except Exception as e:
        print(f"  ⚠ 地图加载失败: {e}")
        map_data = None


def draw_map_background(ax, x_min, x_max, y_min, y_max):
    """在 ax 上绘制指定范围内的栅格地图背景"""
    if map_data is None:
        return

    grid = map_data['grid']
    res = map_data['resolution']
    ox = map_data['origin_x']
    oy = map_data['origin_y']
    H, W = grid.shape

    # 世界坐标 → 像素索引（裁剪到范围）
    col_min = max(0, int((x_min - ox) / res))
    col_max = min(W, int((x_max - ox) / res) + 1)
    row_min = max(0, int((y_min - oy) / res))
    row_max = min(H, int((y_max - oy) / res) + 1)

    if col_min >= col_max or row_min >= row_max:
        return

    # 提取子区域
    sub = grid[row_min:row_max, col_min:col_max]

    # 构造 RGBA 图像
    rgba = np.zeros((sub.shape[0], sub.shape[1], 4), dtype=np.float32)
    obstacle_mask = sub < 0  # -1 = 障碍物
    free_mask = sub >= 0     # ≥0 = 可通行

    # 障碍物: 暗红色
    rgba[obstacle_mask] = [0.7, 0.15, 0.15, 0.7]
    # 可通行: 深绿偏暗
    rgba[free_mask] = [0.1, 0.25, 0.12, 0.35]

    # 世界坐标的 extent
    extent = [
        ox + col_min * res,  # left
        ox + col_max * res,  # right
        oy + row_min * res,  # bottom
        oy + row_max * res,  # top
    ]

    ax.imshow(rgba, extent=extent, origin='lower', aspect='equal',
              interpolation='nearest', zorder=0)


def get_trajectory_bounds():
    """计算所有轨迹点的包围盒，用于裁剪地图显示范围"""
    all_pts = []
    if vehicle_trail:
        all_pts.extend(vehicle_trail)
    if latest_traj_pts:
        all_pts.extend(latest_traj_pts)
    if global_traj_pts:
        all_pts.extend(global_traj_pts)
    if vehicle_pos:
        all_pts.append((vehicle_pos[0], vehicle_pos[1]))

    if not all_pts:
        return None

    pts = np.array(all_pts)
    return (
        pts[:, 0].min() - MAP_PADDING,
        pts[:, 0].max() + MAP_PADDING,
        pts[:, 1].min() - MAP_PADDING,
        pts[:, 1].max() + MAP_PADDING,
    )


def save_plot():
    global plot_counter
    if not HAS_MPL:
        return

    fig, ax = plt.subplots(1, 1, figsize=(12, 8))
    ax.set_aspect('equal')
    ax.set_facecolor('#1a1a2e')
    ax.grid(True, alpha=0.2, color='white')

    # 计算轨迹包围盒
    bounds = get_trajectory_bounds()

    # 绘制地图背景（裁剪到轨迹范围）
    if bounds is not None:
        draw_map_background(ax, *bounds)
        ax.set_xlim(bounds[0], bounds[1])
        ax.set_ylim(bounds[2], bounds[3])

    # 车辆历史轨迹（白色）
    if len(vehicle_trail) > 1:
        trail = np.array(vehicle_trail)
        ax.plot(trail[:, 0], trail[:, 1], 'w-', linewidth=1.5,
                alpha=0.6, label='Vehicle Trail', zorder=3)

    # Global 轨迹（深黄色/橙色虚线）
    if len(global_traj_pts) > 1:
        gtraj = np.array(global_traj_pts)
        ax.plot(gtraj[:, 0], gtraj[:, 1], '--', color='#ff9900',
                linewidth=2.0, alpha=0.8, label='Hybrid A* Global Path', zorder=4)

    # Planning 轨迹（蓝绿色）
    if len(latest_traj_pts) > 1:
        traj = np.array(latest_traj_pts)
        ax.plot(traj[:, 0], traj[:, 1], '-', color='#00d2ff',
                linewidth=2.5, label='CILQR Trajectory', zorder=5)
        ax.plot(traj[0, 0], traj[0, 1], 'o', color='#00ff88',
                markersize=8, label='Traj Start', zorder=6)
        ax.plot(traj[-1, 0], traj[-1, 1], 's', color='#ff4444',
                markersize=8, label='Traj End', zorder=6)

    # 当前位置（黄色三角）
    if vehicle_pos is not None:
        x, y, theta = vehicle_pos
        ax.plot(x, y, '^', color='#ffcc00', markersize=12,
                label='Vehicle', zorder=7)
        # 航向箭头
        dx = 1.5 * math.cos(theta)
        dy = 1.5 * math.sin(theta)
        ax.annotate('', xy=(x + dx, y + dy), xytext=(x, y),
                    arrowprops=dict(arrowstyle='->', color='#ffcc00', lw=2),
                    zorder=7)

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
    print("  CILQR Trajectory Visualizer + Grid Map")
    print("=" * 50)
    if not HAS_MPL:
        print("  matplotlib not found, install: pip3 install matplotlib")
        return

    # 加载栅格地图
    load_grid_map()

    cyber.init()
    node = cyber.Node("trajectory_visualizer")

    node.create_reader('/apollo/localization/pose',
                       LocalizationEstimate, on_localization)
    node.create_reader('/apollo/planning',
                       ADCTrajectory, on_trajectory)
    node.create_reader('/apollo/planning/global_path',
                       ADCTrajectory, on_global_trajectory)

    print("  Saving to output/ every 2s")
    print("  Press Ctrl+C to stop\n")

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
