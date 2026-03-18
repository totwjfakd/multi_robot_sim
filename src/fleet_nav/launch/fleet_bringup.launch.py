# fleet_nav/launch/fleet_bringup.launch.py
#
# Entry point for the entire fleet navigation system.
#
# Usage:
#   ros2 launch fleet_nav fleet_bringup.launch.py num_robots:=10
#   ros2 launch fleet_nav fleet_bringup.launch.py num_robots:=100
#
# Install nav2-route first:
#   sudo apt install ros-humble-nav2-route
#
# Sequence:
#   1. start_world  → Gazebo + shared servers (map, route, GT bridge)
#   2. spawn_robots → N robots in Gazebo + per-robot Nav2 stacks
#   3. fleet_commander (optional) → auto-assign routes to robots

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
    graph_file = os.path.join(pkg_fleet, 'params', 'nav_graph.geojson')

    # ── Arguments ─────────────────────────────────────────────────────────────
    arg_num_robots = DeclareLaunchArgument(
        'num_robots', default_value='10',
        description='Number of robots to spawn (tested up to 100)',
    )
    arg_start_commander = DeclareLaunchArgument(
        'start_commander', default_value='true',
        description='Launch fleet_commander to auto-assign routes',
    )
    arg_dispatch_interval = DeclareLaunchArgument(
        'dispatch_interval', default_value='2.0',
        description='Seconds between goal dispatches per robot',
    )
    arg_log_level = DeclareLaunchArgument(
        'log_level', default_value='warn',
        description='Log level (use warn/error for large fleets to reduce noise)',
    )

    # ── 1. World + shared servers ──────────────────────────────────────────────
    start_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_fleet, 'launch', 'start_world.launch.py')
        ),
        launch_arguments={
            'num_robots': LaunchConfiguration('num_robots'),
        }.items(),
    )

    # ── 2. Spawn robots (delayed to allow Gazebo to initialize) ───────────────
    spawn_robots = TimerAction(
        period=5.0,  # wait 5s for Gazebo to be ready
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
    )

    # ── 3. Multi-Robot Controller (단일 노드, 모든 로봇 cmd_vel 담당) ──────────
    multi_robot_controller = TimerAction(
        period=8.0,  # Gazebo + 로봇 스폰 후 실행
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
                }],
                output='screen',
            )
        ],
    )

    # ── 4. Fleet Commander (delayed to allow controller to initialize) ─────────
    fleet_commander = TimerAction(
        period=12.0,  # multi_robot_controller 기동 후 실행
        actions=[
            Node(
                package='fleet_nav',
                executable='fleet_commander',
                name='fleet_commander',
                parameters=[{
                    'num_robots':        LaunchConfiguration('num_robots'),
                    'robot_prefix':      'robot_',
                    'graph_file':        graph_file,
                    'dispatch_interval': LaunchConfiguration('dispatch_interval'),
                }],
                output='screen',
            )
        ],
    )

    return LaunchDescription([
        arg_num_robots,
        arg_start_commander,
        arg_dispatch_interval,
        arg_log_level,
        start_world,
        spawn_robots,
        multi_robot_controller,
        fleet_commander,
    ])
