# fleet_nav/launch/spawn_robots.launch.py
#
# N대 로봇 스폰 (로봇당 3개 노드):
#   - gz_spawner           (Gazebo SDF 스폰)
#   - cmd_vel bridge       (ROS → Gazebo DiffDrive)
#   - robot_state_publisher
#
# 제거됨:
#   - controller_server  )  → multi_robot_controller 단일 노드로 통합
#   - lifecycle_manager  )     (fleet_bringup.launch.py에서 실행)

import os
import math

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory


_PKG_FLEET = get_package_share_directory('fleet_nav')
_PKG_GZ    = get_package_share_directory('gz_bringup')

_SDF_FILE  = os.path.join(_PKG_FLEET, 'models', 'X1_NoSensor', 'model.sdf')
_URDF_FILE = os.path.join(_PKG_FLEET, 'urdf', 'x1_nosensor.urdf')
_WORLD_NAME = 'warehouse_world'


def _make_spawn_positions(num_robots: int):
    """N대 로봇의 초기 위치 생성 (그리드 배치)."""
    positions = []
    cols = max(1, int(math.ceil(math.sqrt(num_robots))))
    for i in range(num_robots):
        row = i // cols
        col = i % cols
        x   = -2.0 + col * 1.5
        y   = -2.0 + row * 1.5
        positions.append((x, y, 0.1, 0.0))
    return positions


def _robot_nodes(context, *args, **kwargs):
    num_robots = int(LaunchConfiguration('num_robots').perform(context))
    world      = LaunchConfiguration('world_name').perform(context)
    log_level  = LaunchConfiguration('log_level').perform(context)
    positions  = _make_spawn_positions(num_robots)

    with open(_URDF_FILE, 'r') as f:
        robot_desc = f.read()

    actions = []
    for i in range(num_robots):
        name = f'robot_{i + 1}'
        x, y, z, yaw = positions[i]

        robot_group = GroupAction([
            PushRosNamespace(name),

            # 1. Gazebo 스폰
            Node(
                package='gz_bringup',
                executable='gz_spawner.py',
                name=f'{name}_spawner',
                parameters=[{
                    'world_name': world,
                    'sdf_file':   _SDF_FILE,
                    'robot_name': name,
                    'x': x, 'y': y, 'z': z, 'yaw': yaw,
                }],
                output='screen',
            ),

            # 2. cmd_vel 브릿지 (ROS → Gazebo DiffDrive)
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name=f'{name}_bridge_cmd',
                arguments=[
                    f'/model/{name}/cmd_vel'
                    f'@geometry_msgs/msg/Twist'
                    f'@gz.msgs.Twist'
                ],
                remappings=[(f'/model/{name}/cmd_vel', 'cmd_vel')],
                output='screen',
            ),

            # 3. Robot State Publisher
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                parameters=[{
                    'robot_description': robot_desc,
                    'use_sim_time':      True,
                    'frame_prefix':      f'{name}/',
                }],
                arguments=['--ros-args', '--log-level', log_level],
                output='screen',
            ),
        ])
        actions.append(robot_group)

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('num_robots',  default_value='10'),
        DeclareLaunchArgument('world_name',  default_value=_WORLD_NAME),
        DeclareLaunchArgument('log_level',   default_value='warn'),

        OpaqueFunction(function=_robot_nodes),
    ])
