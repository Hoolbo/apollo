#!/usr/bin/env python3
"""
Reads config/obstacles.json and updates the SDF file with matching obstacle models.
Run this after editing obstacles.json, before colcon build.

Usage:
    python3 sync_obstacles.py
"""
import json
import os
import re

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
CONFIG_PATH = os.path.join(SCRIPT_DIR, 'config', 'obstacles.json')
SDF_PATH = os.path.join(
    SCRIPT_DIR, '..', 'description', 'models',
    'articulated_tracked_vehicle.sdf')


def generate_sdf_obstacles(obstacles):
    lines = ['    <!-- ======== Auto-generated from obstacles.json ======== -->']
    for i, obs in enumerate(obstacles):
        x = obs.get('x', 0.0)
        y = obs.get('y', 0.0)
        r = obs.get('radius', 0.5)
        lines.append(f'''    <model name="obstacle_{i+1}">
      <static>true</static>
      <pose>{x} {y} 0.75 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><cylinder><radius>{r}</radius><length>1.5</length></cylinder></geometry>
        </collision>
        <visual name="visual">
          <geometry><cylinder><radius>{r}</radius><length>1.5</length></cylinder></geometry>
          <material><ambient>0.8 0.2 0.2 1</ambient><diffuse>0.8 0.2 0.2 1</diffuse></material>
        </visual>
      </link>
    </model>
''')
    return '\n'.join(lines)


def main():
    # Read obstacles
    with open(CONFIG_PATH, 'r') as f:
        data = json.load(f)
    obstacles = data.get('obstacles', [])
    print(f'Read {len(obstacles)} obstacles from {CONFIG_PATH}')

    # Read SDF
    with open(SDF_PATH, 'r') as f:
        sdf = f.read()

    # Remove old auto-generated section (between markers or before </world>)
    pattern = r'    <!-- ======== Auto-generated from obstacles\.json ======== -->.*?(?=\s*</world>)'
    sdf = re.sub(pattern, '', sdf, flags=re.DOTALL)

    # Also remove the old comment placeholder
    sdf = sdf.replace(
        '    <!-- obstacles are spawned dynamically by obstacle_publisher.py from config/obstacles.json -->\n',
        '')

    # Insert new obstacles before </world>
    new_section = generate_sdf_obstacles(obstacles)
    sdf = sdf.replace('    </world>', f'{new_section}\n    </world>')

    # Write back
    with open(SDF_PATH, 'w') as f:
        f.write(sdf)
    print(f'Updated {SDF_PATH} with {len(obstacles)} obstacles')
    for i, obs in enumerate(obstacles):
        print(f'  obstacle_{i+1}: ({obs["x"]}, {obs["y"]}) r={obs.get("radius", 0.5)}')


if __name__ == '__main__':
    main()
