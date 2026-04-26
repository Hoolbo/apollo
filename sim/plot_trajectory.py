#!/usr/bin/env python3
"""
实时可视化 CILQR 规划轨迹 + 车辆位置 + 栅格地图背景

在容器内运行（需要 X11 转发或 Xvfb + 保存到文件模式）：
  python3 sim/plot_trajectory.py

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
from modules.common_msgs.chassis_msgs.chassis_pb2 import Chassis

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
# 动态获取 Apollo 根目录：假设脚本在 /apollo_workspace/sim/ 下
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
APOLLO_ROOT = os.path.dirname(SCRIPT_DIR)

PLANNER_CONFIG = os.path.join(APOLLO_ROOT, 'modules/planning/cilqr_planner/conf/cilqr_planner_config.pb.txt')
OUTPUT_DIR = os.path.join(SCRIPT_DIR, 'output')
MAP_PADDING = 20.0  # 地图裁剪时在轨迹包围盒外扩展的米数

# ── 全局数据 ──
vehicle_trail = []       # 车辆历史轨迹 [(x,y), ...]
latest_traj_pts = []     # 最新 planning 轨迹点 [(x,y), ...]
global_traj_pts = []     # 混合A* 规划出的全局路径 [(x,y), ...]
vehicle_pos = None       # 当前位置 (x, y, theta)
current_gamma = 0.0      # 当前铰接角 (rad)
plot_counter = 0
map_data = None          # 加载的地图数据 dict
vehicle_params = None    # 车辆几何参数
needs_full_map_plot = True # 标记是否需要画全图（收到新全局路径时置True）


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
    global global_traj_pts, needs_full_map_plot
    pts = []
    for tp in msg.trajectory_point:
        pts.append((tp.path_point.x, tp.path_point.y))
    global_traj_pts = pts
    needs_full_map_plot = True


def on_chassis(msg):
    global current_gamma
    # chassis steering_percentage 转化为弧度得到 gamma
    current_gamma = msg.steering_percentage * math.pi / 180.0


def load_vehicle_config():
    global vehicle_params
    # 基于 APOLLO_ROOT 动态获取 vehicle.json 路径
    config_path = os.path.join(APOLLO_ROOT, 'modules/planning/cilqr_planner/conf/cilqr_json/vehicle.json')
    if not os.path.exists(config_path):
        # 兼容旧硬编码路径作 fallback
        fallback = '/apollo/modules/planning/cilqr_planner/conf/cilqr_json/vehicle.json'
        if os.path.exists(fallback):
            config_path = fallback
        
    if not os.path.exists(config_path):
        print(f"  ⚠ 找不到 vehicle.json: {config_path}")
        return

    try:
        import re
        with open(config_path, 'r') as f:
            raw = f.read()
            # 移除注释
            raw = re.sub(r'//.*', '', raw)
            data = json.loads(raw)
            vehicle_params = {
                'L_f': data['front_body']['pivot_distance'],
                'L_r': data['rear_body']['pivot_distance'],
                'L_f_body': data['front_body']['body_length'],
                'L_r_body': data['rear_body']['body_length'],
                'W_f_body': data['front_body']['body_width'],
                'W_r_body': data['rear_body']['body_width'],
            }
        print(f"  ✓ 车辆参数加载成功: {vehicle_params}")
    except Exception as e:
        print(f"  ⚠ 车辆参数加载失败: {e}")


def get_vehicle_corners(x_f, y_f, theta_f, gamma, params):
    cos_f = math.cos(theta_f)
    sin_f = math.sin(theta_f)
    theta_r = theta_f - gamma
    cos_r = math.cos(theta_r)
    sin_r = math.sin(theta_r)
    
    hl_f = params['L_f_body'] / 2.0
    hw_f = params['W_f_body'] / 2.0
    
    fc = [
        (x_f + hl_f*cos_f - hw_f*sin_f, y_f + hl_f*sin_f + hw_f*cos_f),
        (x_f + hl_f*cos_f + hw_f*sin_f, y_f + hl_f*sin_f - hw_f*cos_f),
        (x_f - hl_f*cos_f + hw_f*sin_f, y_f - hl_f*sin_f - hw_f*cos_f),
        (x_f - hl_f*cos_f - hw_f*sin_f, y_f - hl_f*sin_f + hw_f*cos_f)
    ]
    
    x_c = x_f - params['L_f'] * cos_f
    y_c = y_f - params['L_f'] * sin_f
    x_r = x_c - params['L_r'] * cos_r
    y_r = y_c - params['L_r'] * sin_r
    
    hl_r = params['L_r_body'] / 2.0
    hw_r = params['W_r_body'] / 2.0
    
    rc = [
        (x_r + hl_r*cos_r - hw_r*sin_r, y_r + hl_r*sin_r + hw_r*cos_r),
        (x_r + hl_r*cos_r + hw_r*sin_r, y_r + hl_r*sin_r - hw_r*cos_r),
        (x_r - hl_r*cos_r + hw_r*sin_r, y_r - hl_r*sin_r - hw_r*cos_r),
        (x_r - hl_r*cos_r - hw_r*sin_r, y_r - hl_r*sin_r + hw_r*cos_r)
    ]
    return fc, rc, (x_c, y_c)


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
    rgba[obstacle_mask] = [1.0, 1.0, 1.0, 0.7]
    # 可通行: 深绿偏暗
    rgba[free_mask] = [0, 0, 0, 0.35]

    # 世界坐标的 extent
    extent = [
        ox + col_min * res,  # left
        ox + col_max * res,  # right
        oy + row_min * res,  # bottom
        oy + row_max * res,  # top
    ]

    ax.imshow(rgba, extent=extent, origin='lower', aspect='equal',
              interpolation='nearest', zorder=0)


def get_trajectory_bounds(full=False):
    """计算包围盒，用于裁剪地图显示范围"""
    if not full and vehicle_pos is not None:
        # 只显示车辆附近 50m x 50m 的区域 (-50, +50)
        return (
            vehicle_pos[0] - 50.0,
            vehicle_pos[0] + 50.0,
            vehicle_pos[1] - 50.0,
            vehicle_pos[1] + 50.0
        )

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
    global plot_counter, needs_full_map_plot
    if not HAS_MPL:
        return

    fig, ax = plt.subplots(1, 1, figsize=(12, 8))
    ax.set_aspect('equal')
    ax.set_facecolor('#1a1a2e')
    ax.grid(True, alpha=0.2, color='white')

    # 计算轨迹包围盒
    bounds = get_trajectory_bounds(full=needs_full_map_plot)
    if needs_full_map_plot:
        needs_full_map_plot = False

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
                linewidth=1.0, label='CILQR Trajectory', zorder=5)
        ax.plot(traj[0, 0], traj[0, 1], 'o', color='#00ff88',
                markersize=1, label='Traj Start', zorder=1)
        ax.plot(traj[-1, 0], traj[-1, 1], 's', color='#ff4444',
                markersize=1, label='Traj End', zorder=1)

    # 当前位置（根据参数绘制车体或三角形）
    if vehicle_pos is not None:
        x, y, theta = vehicle_pos
        
        if vehicle_params is not None:
            # 获取前后车体角点
            fc, rc, pivot = get_vehicle_corners(x, y, theta, current_gamma, vehicle_params)
            fc.append(fc[0]) # 闭合多边形
            rc.append(rc[0])
            fc_arr = np.array(fc)
            rc_arr = np.array(rc)
            
            # 画前后车体框
            ax.plot(fc_arr[:, 0], fc_arr[:, 1], '-', color='#ffcc00', linewidth=2, zorder=7)
            ax.plot(rc_arr[:, 0], rc_arr[:, 1], '-', color='#ffcc00', linewidth=2, zorder=7)
            
            # 前车体 Label (只标一次)
            ax.plot([], [], '-', color='#ffcc00', linewidth=2, label='Vehicle Body')
            
            # 画中心连线（前质心 -> 铰接点 -> 后质心）
            x_r = pivot[0] - vehicle_params['L_r'] * math.cos(theta - current_gamma)
            y_r = pivot[1] - vehicle_params['L_r'] * math.sin(theta - current_gamma)
            ax.plot((x, pivot[0]), (y, pivot[1]), '--', color='#ffcc00', linewidth=1, zorder=7)
            ax.plot((pivot[0], x_r), (pivot[1], y_r), '--', color='#ffcc00', linewidth=1, zorder=7)
            
            # 铰接点标记
            ax.plot(pivot[0], pivot[1], 'o', color='white', markersize=4, zorder=8)
        else:
            ax.plot(x, y, '^', color='#ffcc00', markersize=12, label='Vehicle', zorder=7)

        # 航向箭头 (设于前车质心)
        dx = 1.5 * math.cos(theta)
        dy = 1.5 * math.sin(theta)
        ax.annotate('', xy=(x + dx, y + dy), xytext=(x, y),
                    arrowprops=dict(arrowstyle='->', color='#ffcc00', lw=2),
                    zorder=7)

    ax.set_xlabel('X (m)', color='white')
    ax.set_ylabel('Y (m)', color='white')
    ax.tick_params(colors='white')
    ax.set_title(f'CILQR Planning Visualization  [frame {plot_counter}]',
                 color='white', fontsize=14)

    for spine in ax.spines.values():
        spine.set_edgecolor('#333')

    fig.patch.set_facecolor('#0f0f23')
    plt.tight_layout()

    path = os.path.join(OUTPUT_DIR, f'trajectory_{plot_counter:04d}.png')
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
    node.create_reader('/apollo/canbus/chassis',
                       Chassis, on_chassis)

    # 加载车辆配置
    load_vehicle_config()

    print(f"  Saving to {OUTPUT_DIR}/ every 2s")
    print("  Press Ctrl+C to stop\n")

    # 清空旧图片
    if os.path.exists(OUTPUT_DIR):
        print(f"  Cleaning old images in {OUTPUT_DIR}...")
        for f in os.listdir(OUTPUT_DIR):
            if f.endswith(".png"):
                try:
                    os.remove(os.path.join(OUTPUT_DIR, f))
                except Exception as e:
                    print(f"  ⚠ Failed to remove {f}: {e}")
    else:
        os.makedirs(OUTPUT_DIR, exist_ok=True)

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
