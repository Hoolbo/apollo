#!/usr/bin/env python3
"""
Publishes obstacle positions as visualization_msgs/MarkerArray
on the /obstacles topic for the CILQR planner.

Reads obstacle config from config/obstacles.json.
To sync obstacles to Gazebo SDF, run: python3 sync_obstacles.py
"""
import json
import os
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from visualization_msgs.msg import Marker, MarkerArray


class ObstaclePublisher(Node):
    def __init__(self):
        super().__init__('obstacle_publisher')

        config_dir = os.path.join(
            get_package_share_directory('planner'), 'config')
        config_path = os.path.join(config_dir, 'obstacles.json')

        try:
            with open(config_path, 'r') as f:
                data = json.load(f)
            self.obstacles = data.get('obstacles', [])
        except Exception as e:
            self.get_logger().error(f'Failed to load {config_path}: {e}')
            self.obstacles = []

        # Load map metadata for boundary display
        maps_dir = os.path.join(
            get_package_share_directory('planner'), 'maps')
        map_path = os.path.join(maps_dir, 'atv_terrain_global_map.json')
        self.map_bounds = None
        try:
            with open(map_path, 'r') as f:
                map_data = json.load(f)
            meta = map_data.get('metadata', {})
            dims = meta.get('dimensions', {})
            origin = meta.get('origin', [0.0, 0.0])
            w = dims.get('width', 0) * dims.get('resolution', 1.0)
            h = dims.get('height', 0) * dims.get('resolution', 1.0)
            self.map_bounds = {
                'x_min': origin[0], 'y_min': origin[1],
                'x_max': origin[0] + w, 'y_max': origin[1] + h,
            }
            self.get_logger().info(
                f'Map boundary: x=[{self.map_bounds["x_min"]:.1f}, {self.map_bounds["x_max"]:.1f}], '
                f'y=[{self.map_bounds["y_min"]:.1f}, {self.map_bounds["y_max"]:.1f}]')
        except Exception as e:
            self.get_logger().warn(f'Failed to load map for boundary: {e}')

        self.pub_ = self.create_publisher(MarkerArray, '/obstacles', 10)
        self.timer_ = self.create_timer(0.5, self.publish_obstacles)
        self.get_logger().info(
            f'Obstacle publisher started — {len(self.obstacles)} obstacles')

    def publish_obstacles(self):
        msg = MarkerArray()
        for i, obs in enumerate(self.obstacles):
            x = obs.get('x', 0.0)
            y = obs.get('y', 0.0)
            r = obs.get('radius', 0.5)

            m = Marker()
            m.header.frame_id = 'world'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'obstacles'
            m.id = i
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.pose.position.x = x
            m.pose.position.y = y
            m.pose.position.z = 0.75
            m.pose.orientation.w = 1.0
            m.scale.x = r * 2.0
            m.scale.y = r * 2.0
            m.scale.z = 1.5
            m.color.r = 1.0
            m.color.g = 0.2
            m.color.b = 0.2
            m.color.a = 0.8
            msg.markers.append(m)

        # Map boundary marker
        if self.map_bounds:
            from geometry_msgs.msg import Point
            b = self.map_bounds
            bm = Marker()
            bm.header.frame_id = 'world'
            bm.header.stamp = self.get_clock().now().to_msg()
            bm.ns = 'map_boundary'
            bm.id = 0
            bm.type = Marker.LINE_STRIP
            bm.action = Marker.ADD
            bm.pose.orientation.w = 1.0
            bm.scale.x = 0.15  # line width
            bm.color.r = 0.2
            bm.color.g = 1.0
            bm.color.b = 0.2
            bm.color.a = 0.9
            corners = [
                (b['x_min'], b['y_min']),
                (b['x_max'], b['y_min']),
                (b['x_max'], b['y_max']),
                (b['x_min'], b['y_max']),
                (b['x_min'], b['y_min']),  # close the loop
            ]
            for cx, cy in corners:
                p = Point()
                p.x = cx
                p.y = cy
                p.z = 0.1
                bm.points.append(p)
            msg.markers.append(bm)

        self.pub_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ObstaclePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
