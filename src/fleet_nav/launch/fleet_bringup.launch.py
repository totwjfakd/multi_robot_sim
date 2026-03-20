# fleet_nav/launch/fleet_bringup.launch.py
#
# Entry point for the entire fleet navigation system.
#
# Usage:
#   ros2 launch fleet_nav fleet_bringup.launch.py num_robots:=10
#   ros2 launch fleet_nav fleet_bringup.launch.py num_robots:=10 \
#     world_file:=/path/to/my.world world_name:=my_world \
#     map_file:=/path/to/map.yaml graph_file:=/path/to/graph.geojson
#
# Install nav2-route first:
#   sudo apt install ros-humble-nav2-route
#
# Sequence:
#   1. start_world          → Gazebo + shared servers (map, route, GT bridge)
#   2. spawn_robots         → N robots in Gazebo + per-robot Nav2 stacks
#   3. multi_robot_controller → cmd_vel 계산 + 외부 명령 수신 (/fleet_nav/command)

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_fleet  = get_package_share_directory('fleet_nav')
    pkg_gz     = get_package_share_directory('gz_bringup')

    # ── Default paths ─────────────────────────────────────────────────────────
    default_world_file  = os.path.join(pkg_gz, 'worlds', 'warehouse_only_map.world')
    default_map_file    = os.path.join(os.path.expanduser('~'), 'Fleet_sim', 'maps', 'v2', 'v2.yaml')
    default_graph_file  = os.path.join(pkg_fleet, 'params', 'nav_graph.geojson')

    # ── Arguments ─────────────────────────────────────────────────────────────
    return LaunchDescription([
        DeclareLaunchArgument('num_robots', default_value='10',
                              description='Number of robots to spawn'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='Launch Gazebo with GUI (false = headless)'),
        DeclareLaunchArgument('log_level', default_value='warn',
                              description='Log level (warn/error for large fleets)'),
        DeclareLaunchArgument('world_file', default_value=default_world_file,
                              description='Path to Gazebo .world file'),
        DeclareLaunchArgument('world_name', default_value='warehouse_world',
                              description='Gazebo world name (used in bridge topic names)'),
        DeclareLaunchArgument('map_file', default_value=default_map_file,
                              description='Path to occupancy map YAML'),
        DeclareLaunchArgument('graph_file', default_value=default_graph_file,
                              description='Path to navigation graph GeoJSON'),

        # ── 1. World + shared servers ──────────────────────────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_fleet, 'launch', 'start_world.launch.py')
            ),
            launch_arguments={
                'num_robots': LaunchConfiguration('num_robots'),
                'gui':        LaunchConfiguration('gui'),
                'world_file': LaunchConfiguration('world_file'),
                'world_name': LaunchConfiguration('world_name'),
                'map_file':   LaunchConfiguration('map_file'),
                'graph_file': LaunchConfiguration('graph_file'),
            }.items(),
        ),

        # ── 2. Spawn robots ────────────────────────────────────────────────────
        TimerAction(
            period=5.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(pkg_fleet, 'launch', 'spawn_robots.launch.py')
                    ),
                    launch_arguments={
                        'num_robots': LaunchConfiguration('num_robots'),
                        'log_level':  LaunchConfiguration('log_level'),
                    }.items(),
                )
            ],
        ),

        # ── 3. Multi-Robot Controller ──────────────────────────────────────────
        TimerAction(
            period=8.0,
            actions=[
                Node(
                    package='fleet_nav',
                    executable='multi_robot_controller',
                    name='multi_robot_controller',
                    parameters=[{
                        'num_robots':        LaunchConfiguration('num_robots'),
                        'robot_prefix':      'robot_',
                        'lookahead_dist':    0.8,
                        'linear_vel':        0.5,
                        'max_angular_vel':   1.5,
                        'goal_tolerance':    0.3,
                        'control_frequency': 10.0,
                        'graph_file':        LaunchConfiguration('graph_file'),
                    }],
                    output='screen',
                )
            ],
        ),

    ])
