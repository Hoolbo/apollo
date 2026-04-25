from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg = get_package_share_directory('planner')
    config_dir = os.path.join(pkg, 'config')
    map_file   = os.path.join(pkg, 'maps', 'atv_terrain_global_map.json')

    return LaunchDescription([
        DeclareLaunchArgument('publish_rate',  default_value='2.0'),
        DeclareLaunchArgument('desire_speed',  default_value='0.5'),
        DeclareLaunchArgument('horizon',       default_value='30'),

        Node(
            package='planner',
            executable='planner_node',
            name='cilqr_planner',
            output='screen',
            parameters=[{
                'config_dir':   config_dir,
                'map_file':     map_file,
                'publish_rate': LaunchConfiguration('publish_rate'),
                'desire_speed': LaunchConfiguration('desire_speed'),
                'horizon':      LaunchConfiguration('horizon'),
                'use_sim_time': True,
            }],
        ),
    ])
