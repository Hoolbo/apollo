#!/usr/bin/env python3
"""
将 Apollo HD Map (protobuf text) 转换为 CILQR 规划器的栅格地图 JSON 格式。

用法:
    python3 convert_hdmap_to_grid.py [--resolution 0.05] [--margin 5.0] [--output output.json]
"""

import re
import json
import argparse
import numpy as np
import os

def parse_lane_boundaries(txt):
    """从 base_map.txt 中解析每条 lane 的左右边界点。"""
    lanes = []
    
    # 按 lane { ... } 分割
    # 使用栈匹配大括号来分割 lane 块
    lane_blocks = []
    i = 0
    while i < len(txt):
        match = re.search(r'\nlane \{', txt[i:])
        if not match:
            break
        start = i + match.start() + 1  # skip the \n
        # 找到匹配的 }
        depth = 0
        j = start + len('lane {') - 1
        for j in range(start, len(txt)):
            if txt[j] == '{':
                depth += 1
            elif txt[j] == '}':
                depth -= 1
                if depth == 0:
                    lane_blocks.append(txt[start:j+1])
                    break
        i = j + 1
    
    # 如果上面的方法没找到，尝试用简单的正则
    if not lane_blocks:
        # 简单方法：按 "\nlane {" 分割
        parts = re.split(r'\nlane \{', txt)
        for p in parts[1:]:  # 跳过 header
            lane_blocks.append('lane {' + p.split('\nlane {')[0])
    
    for block in lane_blocks:
        lane = {'left': [], 'right': [], 'center': []}
        
        # 提取 central_curve 的点
        center_match = re.search(r'central_curve\s*\{(.*?)\}\s*\}', block, re.DOTALL)
        if center_match:
            center_txt = center_match.group(1)
            points = re.findall(r'point\s*\{\s*x:\s*([0-9.e+-]+)\s*y:\s*([0-9.e+-]+)', center_txt)
            lane['center'] = [(float(x), float(y)) for x, y in points]
        
        # 提取 left_boundary 的点
        left_match = re.search(r'left_boundary\s*\{(.*?)\n  \}\n', block, re.DOTALL)
        if left_match:
            left_txt = left_match.group(1)
            points = re.findall(r'point\s*\{\s*x:\s*([0-9.e+-]+)\s*y:\s*([0-9.e+-]+)', left_txt)
            lane['left'] = [(float(x), float(y)) for x, y in points]
        
        # 提取 right_boundary 的点
        right_match = re.search(r'right_boundary\s*\{(.*?)\n  \}\n', block, re.DOTALL)
        if right_match:
            right_txt = right_match.group(1)
            points = re.findall(r'point\s*\{\s*x:\s*([0-9.e+-]+)\s*y:\s*([0-9.e+-]+)', right_txt)
            lane['right'] = [(float(x), float(y)) for x, y in points]
        
        if lane['center'] or lane['left'] or lane['right']:
            lanes.append(lane)
    
    return lanes


def generate_boundary_from_center(center_points, half_width=2.0):
    """当 lane 没有边界数据时，从中心线生成左右边界。"""
    if len(center_points) < 2:
        return [], []
    
    left = []
    right = []
    pts = np.array(center_points)
    
    for i in range(len(pts)):
        if i == 0:
            dx = pts[1][0] - pts[0][0]
            dy = pts[1][1] - pts[0][1]
        elif i == len(pts) - 1:
            dx = pts[-1][0] - pts[-2][0]
            dy = pts[-1][1] - pts[-2][1]
        else:
            dx = pts[i+1][0] - pts[i-1][0]
            dy = pts[i+1][1] - pts[i-1][1]
        
        length = np.sqrt(dx*dx + dy*dy)
        if length < 1e-9:
            continue
        
        # 法向量（左手法则）
        nx = -dy / length
        ny = dx / length
        
        left.append((pts[i][0] + nx * half_width, pts[i][1] + ny * half_width))
        right.append((pts[i][0] - nx * half_width, pts[i][1] - ny * half_width))
    
    return left, right


def fill_polygon_on_grid(grid, polygon_pts, origin_x, origin_y, resolution, value=0.0):
    """使用扫描线算法在栅格上填充多边形区域。"""
    if len(polygon_pts) < 3:
        return
    
    pts = np.array(polygon_pts)
    # 转换到栅格坐标
    gx = (pts[:, 0] - origin_x) / resolution
    gy = (pts[:, 1] - origin_y) / resolution
    
    height, width = grid.shape
    
    min_y = max(0, int(np.floor(gy.min())))
    max_y = min(height - 1, int(np.ceil(gy.max())))
    
    for y in range(min_y, max_y + 1):
        # 找到与扫描线相交的边
        intersections = []
        n = len(gx)
        for i in range(n):
            j = (i + 1) % n
            y1, y2 = gy[i], gy[j]
            if y1 == y2:
                continue
            if y < min(y1, y2) or y > max(y1, y2):
                continue
            # 线性插值求交点的 x 坐标
            t = (y - y1) / (y2 - y1)
            x_intersect = gx[i] + t * (gx[j] - gx[i])
            intersections.append(x_intersect)
        
        intersections.sort()
        
        # 成对填充
        for k in range(0, len(intersections) - 1, 2):
            x_start = max(0, int(np.floor(intersections[k])))
            x_end = min(width - 1, int(np.ceil(intersections[k + 1])))
            grid[y, x_start:x_end + 1] = value


def fill_thick_polyline(grid, points, origin_x, origin_y, resolution, half_width_cells=2, value=0.0):
    """沿折线以一定宽度填充，确保不会有间隙。"""
    if len(points) < 2:
        return
    
    height, width = grid.shape
    
    for i in range(len(points) - 1):
        x1 = (points[i][0] - origin_x) / resolution
        y1 = (points[i][1] - origin_y) / resolution
        x2 = (points[i+1][0] - origin_x) / resolution
        y2 = (points[i+1][1] - origin_y) / resolution
        
        # Bresenham-style: 沿线段采样并填充周围
        dist = max(abs(x2 - x1), abs(y2 - y1))
        steps = max(int(dist * 2), 1)
        
        for s in range(steps + 1):
            t = s / steps
            cx = x1 + t * (x2 - x1)
            cy = y1 + t * (y2 - y1)
            
            for dy in range(-half_width_cells, half_width_cells + 1):
                for dx in range(-half_width_cells, half_width_cells + 1):
                    gx_i = int(round(cx + dx))
                    gy_i = int(round(cy + dy))
                    if 0 <= gx_i < width and 0 <= gy_i < height:
                        grid[gy_i, gx_i] = value


def main():
    parser = argparse.ArgumentParser(description='Convert Apollo HD Map to CILQR grid map')
    parser.add_argument('--input', default=None, help='Input base_map.txt path')
    parser.add_argument('--resolution', type=float, default=0.05, help='Grid resolution in meters')
    parser.add_argument('--margin', type=float, default=10.0, help='Margin around road network in meters')
    parser.add_argument('--lane_width', type=float, default=5.0, help='Default lane width when boundaries missing')
    parser.add_argument('--output', default=None, help='Output JSON file path')
    parser.add_argument('--use_local_coords', action='store_true', default=True,
                        help='Convert UTM coords to local coords relative to start point')
    parser.add_argument('--clip_xmin', type=float, default=None, help='Clip area min X (local coords)')
    parser.add_argument('--clip_xmax', type=float, default=None, help='Clip area max X (local coords)')
    parser.add_argument('--clip_ymin', type=float, default=None, help='Clip area min Y (local coords)')
    parser.add_argument('--clip_ymax', type=float, default=None, help='Clip area max Y (local coords)')
    parser.add_argument('--max_cells', type=int, default=50_000_000, help='Max grid cells (safety limit)')
    args = parser.parse_args()
    
    # 确定输入文件
    script_dir = os.path.dirname(os.path.abspath(__file__))
    if args.input is None:
        args.input = os.path.join(script_dir, 'base_map.txt')
    
    if args.output is None:
        args.output = os.path.join(script_dir, 'grid_map_201_0320.json')
    
    print(f"输入文件: {args.input}")
    print(f"分辨率: {args.resolution}m")
    print(f"边界余量: {args.margin}m")
    
    # 读取 base_map.txt
    with open(args.input, 'r') as f:
        txt = f.read()
    
    # 解析 lanes
    lanes = parse_lane_boundaries(txt)
    print(f"解析到 {len(lanes)} 条车道")
    
    if not lanes:
        print("错误: 未找到任何车道数据!")
        return
    
    # 收集所有点，确定范围
    all_points = []
    for lane in lanes:
        all_points.extend(lane['center'])
        all_points.extend(lane['left'])
        all_points.extend(lane['right'])
    
    all_points = np.array(all_points)
    print(f"总共 {len(all_points)} 个点")
    print(f"UTM X 范围: {all_points[:, 0].min():.2f} ~ {all_points[:, 0].max():.2f}")
    print(f"UTM Y 范围: {all_points[:, 1].min():.2f} ~ {all_points[:, 1].max():.2f}")
    
    # 读取起始点（用于坐标转换）
    start_point_file = os.path.join(script_dir, 'current_start_point.txt')
    ref_x, ref_y = all_points[:, 0].min(), all_points[:, 1].min()
    if os.path.exists(start_point_file):
        with open(start_point_file, 'r') as f:
            parts = f.read().strip().split(',')
            if len(parts) >= 2:
                ref_x = float(parts[0])
                ref_y = float(parts[1])
                print(f"使用起始点作为参考: ({ref_x:.2f}, {ref_y:.2f})")
    
    # 转换为局部坐标
    if args.use_local_coords:
        offset_x = ref_x
        offset_y = ref_y
    else:
        offset_x = 0
        offset_y = 0
    
    # 计算局部坐标范围
    local_min_x = all_points[:, 0].min() - offset_x - args.margin
    local_max_x = all_points[:, 0].max() - offset_x + args.margin
    local_min_y = all_points[:, 1].min() - offset_y - args.margin
    local_max_y = all_points[:, 1].max() - offset_y + args.margin
    
    # 应用裁剪区域
    if args.clip_xmin is not None:
        local_min_x = max(local_min_x, args.clip_xmin)
    if args.clip_xmax is not None:
        local_max_x = min(local_max_x, args.clip_xmax)
    if args.clip_ymin is not None:
        local_min_y = max(local_min_y, args.clip_ymin)
    if args.clip_ymax is not None:
        local_max_y = min(local_max_y, args.clip_ymax)
    
    print(f"\n局部坐标范围 (裁剪后):")
    print(f"  X: {local_min_x:.2f} ~ {local_max_x:.2f} ({local_max_x - local_min_x:.2f}m)")
    print(f"  Y: {local_min_y:.2f} ~ {local_max_y:.2f} ({local_max_y - local_min_y:.2f}m)")
    
    # 创建栅格
    grid_width = int(np.ceil((local_max_x - local_min_x) / args.resolution))
    grid_height = int(np.ceil((local_max_y - local_min_y) / args.resolution))
    
    print(f"\n栅格尺寸: {grid_width} x {grid_height} = {grid_width * grid_height:,} 格子")
    estimated_size_mb = grid_width * grid_height * 6 / 1024 / 1024
    print(f"预估 JSON 文件大小: {estimated_size_mb:.1f} MB")
    
    if grid_width * grid_height > args.max_cells:
        print(f"WARNING: 栅格过大 (>{args.max_cells:,} 格子)!")
        print(f"  建议: 使用 --clip_xmin/xmax/ymin/ymax 缩小范围")
        print(f"  或者: 使用 --resolution 0.1 或 --resolution 0.2 降低分辨率")
        return
    
    # 初始化为 -1.0 (未知/障碍)
    origin_x = local_min_x
    origin_y = local_min_y
    grid = np.full((grid_height, grid_width), -1.0, dtype=np.float64)
    
    # 填充车道区域
    print("\n填充车道区域...")
    filled_lanes = 0
    for idx, lane in enumerate(lanes):
        left_pts = [(x - offset_x, y - offset_y) for x, y in lane['left']]
        right_pts = [(x - offset_x, y - offset_y) for x, y in lane['right']]
        center_pts = [(x - offset_x, y - offset_y) for x, y in lane['center']]
        
        # 如果没有边界，从中心线生成
        if not left_pts or not right_pts:
            if center_pts:
                left_pts, right_pts = generate_boundary_from_center(
                    center_pts, half_width=args.lane_width / 2)
        
        if left_pts and right_pts and len(left_pts) >= 2 and len(right_pts) >= 2:
            # 构造多边形: 左边界正序 + 右边界逆序
            polygon = left_pts + list(reversed(right_pts))
            fill_polygon_on_grid(grid, polygon, origin_x, origin_y, args.resolution, value=0.0)
            filled_lanes += 1
        elif center_pts:
            # 退化: 沿中心线画粗线
            half_w_cells = max(1, int(args.lane_width / 2 / args.resolution))
            fill_thick_polyline(grid, center_pts, origin_x, origin_y, 
                              args.resolution, half_width_cells=half_w_cells, value=0.0)
            filled_lanes += 1
    
    print(f"填充了 {filled_lanes}/{len(lanes)} 条车道")
    
    # 统计
    passable = np.sum(grid == 0.0)
    obstacle = np.sum(grid == -1.0)
    total = grid.size
    print(f"\n栅格统计:")
    print(f"  可通行: {passable:,} ({passable/total*100:.1f}%)")
    print(f"  障碍/未知: {obstacle:,} ({obstacle/total*100:.1f}%)")
    
    # 输出 JSON (与 atv_terrain_global_map.json 格式一致)
    print(f"\n写入 JSON: {args.output}")
    
    output_data = {
        "metadata": {
            "dimensions": {
                "width": grid_width,
                "height": grid_height,
                "resolution": args.resolution
            },
            "origin": [origin_x, origin_y],
            "max_elevation": 10.0
        },
        "data": grid.tolist()
    }
    
    with open(args.output, 'w') as f:
        json.dump(output_data, f, separators=(',', ':'))
    
    file_size = os.path.getsize(args.output)
    print(f"文件大小: {file_size / 1024 / 1024:.1f} MB")
    print(f"完成! 输出到: {args.output}")
    
    # 生成预览图 (如果 matplotlib 可用)
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        
        preview_path = args.output.replace('.json', '_preview.png')
        fig, ax = plt.subplots(1, 1, figsize=(14, 10))
        
        # 显示栅格
        extent = [origin_x, origin_x + grid_width * args.resolution,
                  origin_y, origin_y + grid_height * args.resolution]
        
        # 自定义颜色: -1 -> 灰色 (障碍), 0 -> 白色 (可通行)
        display = np.where(grid == -1.0, 0.5, 1.0)
        ax.imshow(display, origin='lower', extent=extent, cmap='gray', vmin=0, vmax=1)
        
        # 叠加车道中心线
        for lane in lanes:
            if lane['center']:
                cx = [x - offset_x for x, y in lane['center']]
                cy = [y - offset_y for x, y in lane['center']]
                ax.plot(cx, cy, 'r-', linewidth=0.5, alpha=0.6)
        
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_title(f'Grid Map (res={args.resolution}m, {grid_width}x{grid_height})')
        ax.set_aspect('equal')
        plt.tight_layout()
        plt.savefig(preview_path, dpi=150)
        plt.close()
        print(f"预览图: {preview_path}")
    except ImportError:
        print("matplotlib 不可用，跳过预览图生成")


if __name__ == '__main__':
    main()
