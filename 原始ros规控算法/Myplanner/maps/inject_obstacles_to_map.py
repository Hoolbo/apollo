"""
inject_obstacles_to_map.py

Reads obstacles.json and injects them as circular obstacle regions (-1)
into atv_terrain_global_map.json so that Hybrid A* can see them.
"""
import json, math, sys, os

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
OBSTACLES_FILE = os.path.join(SCRIPT_DIR, "..", "config", "obstacles.json")
MAP_FILE = os.path.join(SCRIPT_DIR, "atv_terrain_global_map.json")

# Load obstacles
with open(OBSTACLES_FILE, "r") as f:
    obs_data = json.load(f)
obstacles = obs_data["obstacles"]

# Load map
with open(MAP_FILE, "r") as f:
    map_data = json.load(f)

meta = map_data["metadata"]
width = meta["dimensions"]["width"]
height = meta["dimensions"]["height"]
resolution = meta["dimensions"]["resolution"]
origin_x, origin_y = meta["origin"]
data = map_data["data"]

# Inflation margin (meters) - slightly larger than obstacle radius
# to account for vehicle body width
INFLATION = 0.5  # meters

count = 0
for obs in obstacles:
    ox, oy, r = obs["x"], obs["y"], obs["radius"]
    effective_r = r + INFLATION

    # Convert obstacle center to grid coordinates
    cx_grid = (ox - origin_x) / resolution
    cy_grid = (oy - origin_y) / resolution
    r_grid = effective_r / resolution

    # Fill circular region with -1
    r_cells = int(math.ceil(r_grid))
    for dy in range(-r_cells, r_cells + 1):
        for dx in range(-r_cells, r_cells + 1):
            if dx*dx + dy*dy <= r_grid*r_grid:
                gx = int(cx_grid) + dx
                gy = int(cy_grid) + dy
                if 0 <= gy < height and 0 <= gx < width:
                    if data[gy][gx] >= 0:  # only mark free cells
                        data[gy][gx] = -1
                        count += 1

print(f"Marked {count} cells as obstacles for {len(obstacles)} obstacles")

# Save
map_data["data"] = data
with open(MAP_FILE, "w") as f:
    json.dump(map_data, f)

print(f"Updated {MAP_FILE}")
