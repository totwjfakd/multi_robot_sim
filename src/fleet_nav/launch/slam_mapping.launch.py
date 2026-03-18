# fleet_nav/launch/slam_mapping.launch.py
#
# GT pose 기반 SLAM 맵핑 (듀얼 LiDAR + scan_merger)
#
# 사용법:
#   ros2 launch fleet_nav slam_mapping.launch.py
#
# 로봇 이동:
#   ros2 run teleop_twist_keyboard teleop_twist_keyboard \
#     --ros-args --remap cmd_vel:=/slam_1/cmd_vel
#
# 맵 저장:
#   ros2 run nav2_map_server map_saver_cli -f ~/Fleet_sim/maps/v1/v1_map

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, TimerAction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_fleet  = get_package_share_directory('fleet_nav')
    pkg_gz     = get_package_share_directory('gz_bringup')
    pkg_ros_gz = get_package_share_directory('ros_gz_sim')

    world_file  = os.path.join(pkg_gz,    'worlds', 'warehouse_only_map.world')
    sdf_file    = os.path.join(pkg_fleet, 'models', 'X1_Lidar', 'model.sdf')
    urdf_file   = os.path.join(pkg_fleet, 'urdf',   'x1_lidar.urdf')
    params_file = os.path.join(pkg_fleet, 'params', 'slam_params.yaml')
    rviz_file   = os.path.join(pkg_fleet, 'rviz', 'slam.rviz')

    fleet_models = os.path.join(pkg_fleet, 'models')
    gz_models    = os.path.join(pkg_gz,    'models')

    world_name = 'warehouse_world'
    robot_name = 'slam_1'

    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),

        # ── Environment ───────────────────────────────────────────────────────
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH',     f'{fleet_models}:{gz_models}'),
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', f'{fleet_models}:{gz_models}'),

        # ── 1. Gazebo ─────────────────────────────────────────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_ros_gz, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={'gz_args': f'-r -v 3 {world_file}'}.items(),
        ),

        # ── 2. Clock bridge ───────────────────────────────────────────────────
        Node(
            package='ros_gz_bridge', executable='parameter_bridge',
            name='clock_bridge',
            arguments=[f'/world/{world_name}/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock'],
            remappings=[(f'/world/{world_name}/clock', '/clock')],
            output='screen',
        ),

        # ── 3. World pose bridge ──────────────────────────────────────────────
        Node(
            package='ros_gz_bridge', executable='parameter_bridge',
            name='world_pose_bridge',
            arguments=[
                f'/world/{world_name}/pose/info'
                f'@tf2_msgs/msg/TFMessage'
                f'@ignition.msgs.Pose_V'
            ],
            remappings=[(f'/world/{world_name}/pose/info', '/world_poses')],
            output='screen',
        ),

        # ── 4. GT Pose Bridge (map→slam_1/odom→slam_1/base_link) ─────────────
        Node(
            package='fleet_nav', executable='gt_pose_bridge',
            name='gt_pose_bridge',
            parameters=[{
                'num_robots':   1,
                'robot_prefix': 'slam_',
                'world_frame':  'map',
                'use_sim_time': True,
            }],
            output='screen',
        ),

        # ── 5. Robot State Publisher (base_link→lidar_link, lidar_rear_link) ──
        Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            name='slam_robot_state_publisher',
            parameters=[{
                'robot_description': robot_desc,
                'use_sim_time':      True,
                'frame_prefix':      f'{robot_name}/',
            }],
            output='screen',
        ),

        # ── 6. Static TF: Ignition 센서 프레임 → URDF 링크 연결 ──────────────
        # Ignition은 LaserScan.header.frame_id = {model}/{link}/{sensor} 로 설정
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='lidar_front_tf',
            arguments=[
                '--x', '0.5', '--y', '0.0', '--z', '0.2',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id',       f'{robot_name}/base_link',
                '--child-frame-id', f'{robot_name}/base_link/gpu_lidar',
            ],
        ),
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='lidar_rear_tf',
            arguments=[
                '--x', '-0.5', '--y', '0.0', '--z', '0.2',
                '--roll', '0', '--pitch', '0', '--yaw', '3.14159',
                '--frame-id',       f'{robot_name}/base_link',
                '--child-frame-id', f'{robot_name}/base_link/gpu_lidar_rear',
            ],
        ),

        # ── 7. Spawn SLAM robot ───────────────────────────────────────────────
        TimerAction(period=5.0, actions=[
            Node(
                package='gz_bringup', executable='gz_spawner.py',
                name='slam_robot_spawner',
                parameters=[{
                    'world_name': world_name,
                    'sdf_file':   sdf_file,
                    'robot_name': robot_name,
                    'x': 2.0, 'y': 0.0, 'z': 0.1, 'yaw': 0.0,
                }],
                output='screen',
            ),
        ]),

        # ── 8. cmd_vel bridge ─────────────────────────────────────────────────
        Node(
            package='ros_gz_bridge', executable='parameter_bridge',
            name='slam_cmd_bridge',
            arguments=[
                f'/model/{robot_name}/cmd_vel'
                f'@geometry_msgs/msg/Twist'
                f'@gz.msgs.Twist'
            ],
            remappings=[(f'/model/{robot_name}/cmd_vel', f'/{robot_name}/cmd_vel')],
            output='screen',
        ),

        # ── 9. Front scan bridge ──────────────────────────────────────────────
        Node(
            package='ros_gz_bridge', executable='parameter_bridge',
            name='slam_scan_front_bridge',
            arguments=[
                f'/world/{world_name}/model/{robot_name}/link/base_link/sensor/gpu_lidar/scan'
                f'@sensor_msgs/msg/LaserScan'
                f'@gz.msgs.LaserScan'
            ],
            remappings=[(
                f'/world/{world_name}/model/{robot_name}/link/base_link/sensor/gpu_lidar/scan',
                f'/{robot_name}/scan_front',
            )],
            output='screen',
        ),

        # ── 10. Rear scan bridge ──────────────────────────────────────────────
        Node(
            package='ros_gz_bridge', executable='parameter_bridge',
            name='slam_scan_rear_bridge',
            arguments=[
                f'/world/{world_name}/model/{robot_name}/link/base_link/sensor/gpu_lidar_rear/scan'
                f'@sensor_msgs/msg/LaserScan'
                f'@gz.msgs.LaserScan'
            ],
            remappings=[(
                f'/world/{world_name}/model/{robot_name}/link/base_link/sensor/gpu_lidar_rear/scan',
                f'/{robot_name}/scan_rear',
            )],
            output='screen',
        ),

        # ── 11. Scan Merger (front + rear → /slam_1/scan) ─────────────────────
        # output_frame_id = slam_1/base_link → slam_toolbox TF 조회 단순화
        Node(
            package='gz_bringup', executable='scan_merger.py',
            name='slam_scan_merger',
            parameters=[{
                'use_sim_time':    True,
                'front_topic':     f'/{robot_name}/scan_front',
                'rear_topic':      f'/{robot_name}/scan_rear',
                'output_topic':    f'/{robot_name}/scan',
                'output_frame_id': f'{robot_name}/base_link',
                'output_samples':  720,
                'publish_rate_hz': 10.0,
                'front_x':  0.5,  'front_y': 0.0, 'front_yaw': 0.0,
                'rear_x':  -0.5,  'rear_y':  0.0, 'rear_yaw':  3.14159,
            }],
            output='screen',
        ),

        # ── 12. slam_toolbox ──────────────────────────────────────────────────
        TimerAction(period=12.0, actions=[
            Node(
                package='slam_toolbox',
                executable='async_slam_toolbox_node',
                name='slam_toolbox',
                parameters=[params_file, {'use_sim_time': True}],
                output='screen',
            ),
        ]),

        # ── 13. RViz ──────────────────────────────────────────────────────────
        TimerAction(period=3.0, actions=[
            Node(
                package='rviz2', executable='rviz2', name='rviz2',
                arguments=['-d', rviz_file],
                parameters=[{'use_sim_time': True}],
                output='screen',
            ),
        ]),
    ])
