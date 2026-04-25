"""
generate_corridor_map.py

Generates a corridor (valley) global map for Hybrid A* planner.
Only the y ∈ [-effective_half, +effective_half] band is free space;
everything outside is marked as obstacle (-1).

The effective free corridor is:
    effective_half = half_width - wall_buffer

This means the visual walls (in Gazebo) are at y = ±half_width,
but the planner treats y = ±(half_width - wall_buffer) as the boundary,
preventing Hybrid A* from planning paths that hug the walls.

Usage (standalone):
    python3 generate_corridor_map.py --half_width 4.0 --wall_buffer 1.5

Usage (from bringup launch):
    Called automatically by bringup.launch.py pre-launch step.
"""

import argparse
import json
import math
import os
import sys


def generate_corridor_map(half_width: float = 4.0,
                          wall_buffer: float = 1.5,
                          x_min: float = -5.0,
                          x_max: float = 50.0,
                          resolution: float = 0.2,
                          output_path: str = None):
    """
    Generate a corridor map JSON file for Hybrid A*.

    Parameters
    ----------
    half_width : float
        Half-width of the visual corridor (Gazebo walls) in meters.
    wall_buffer : float
        Inner buffer in meters.  The planner's free band is
        y ∈ [-(half_width-buffer), +(half_width-buffer)].
    x_min, x_max : float
        Longitudinal range of the map (meters).
    resolution : float
        Grid cell size in meters.
    output_path : str or None
        Where to write the JSON map.
    """
    if output_path is None:
        script_dir = os.path.dirname(os.path.abspath(__file__))
        output_path = os.path.join(script_dir, "atv_terrain_global_map.json")

    effective_half = half_width - wall_buffer
    if effective_half < 1.0:
        print(f"[corridor_map] WARNING: effective corridor half_width "
              f"({effective_half:.1f}m) is very narrow!")

    # Symmetric y extent — needs room for walls + margin
    y_extent = max(half_width + 8.0, 25.0)

    # Compute grid dimensions (must be odd for Gazebo heightmap compat)
    width = int(math.ceil((x_max - x_min) / resolution))
    height = int(math.ceil(2.0 * y_extent / resolution))
    if width % 2 == 0:
        width += 1
    if height % 2 == 0:
        height += 1

    origin_x = x_min
    origin_y = -y_extent

    print(f"[corridor_map] Generating corridor map:")
    print(f"  visual half_width = {half_width} m")
    print(f"  wall_buffer       = {wall_buffer} m")
    print(f"  effective free    = y ∈ [{-effective_half}, {effective_half}] m")
    print(f"  x range           = [{x_min}, {x_max}] m")
    print(f"  grid              = {width}x{height}, res={resolution} m/px")
    print(f"  output            = {output_path}")

    # Build grid data
    data = []
    for row in range(height):
        world_y = origin_y + (row + 0.5) * resolution
        if -effective_half <= world_y <= effective_half:
            # Free corridor
            data.append([0.0] * width)
        else:
            # Wall (obstacle)
            data.append([-1.0] * width)

    map_json = {
        "metadata": {
            "dimensions": {
                "width": width,
                "height": height,
                "resolution": resolution,
            },
            "origin": [origin_x, origin_y],
            "max_elevation": 10.0,
        },
        "data": data,
    }

    with open(output_path, "w") as f:
        json.dump(map_json, f)

    print(f"[corridor_map] Saved {width}x{height} map ({os.path.getsize(output_path)} bytes)")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Generate corridor map for Hybrid A*")
    parser.add_argument("--half_width", type=float, default=4.0)
    parser.add_argument("--wall_buffer", type=float, default=1.5)
    parser.add_argument("--x_min", type=float, default=-5.0)
    parser.add_argument("--x_max", type=float, default=50.0)
    parser.add_argument("--resolution", type=float, default=0.2)
    parser.add_argument("--output", type=str, default=None)
    args = parser.parse_args()

    generate_corridor_map(
        half_width=args.half_width,
        wall_buffer=args.wall_buffer,
        x_min=args.x_min,
        x_max=args.x_max,
        resolution=args.resolution,
        output_path=args.output,
    )
