# fleet_nav/launch/start_world.launch.py
#
# Launches:
#   1. Gazebo world (Ignition Fortress)
#   2. Global Map Server (shared by all robots)
#   3. World pose bridge (Gazebo → ROS TFMessage, for GT pose)
#   4. Clock bridge
#   5. GT Pose Bridge node (converts world poses → Nav2 TF frames)
#   6. Route Server (shared, 1 instance for all robots)
#   7. Lifecycle managers for shared nodes

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_fleet  = get_package_share_directory('fleet_nav')
    pkg_gz     = get_package_share_directory('gz_bringup')   # reuse world file
    pkg_ros_gz = get_package_share_directory('ros_gz_sim')

    world_file = os.path.join(pkg_gz, 'worlds', 'warehouse_only_map.world')
    map_file   = os.path.join(pkg_gz, 'maps', 'v1', 'v1_map.yaml')
    graph_file = os.path.join(pkg_fleet, 'params', 'nav_graph.geojson')
    world_name = 'warehouse_world'

    # Model paths: fleet_nav models + gz_bringup models (for mesh URIs)
    fleet_models = os.path.join(pkg_fleet, 'models')
    gz_models    = os.path.join(pkg_gz, 'models')
    resource_path = f'{fleet_models}:{gz_models}'

    return LaunchDescription([
        # ── Arguments ────────────────────────────────────────────────────────
        DeclareLaunchArgument('num_robots', default_value='10',
                              description='Total number of robots'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='Launch Gazebo with GUI'),

        # ── Environment ───────────────────────────────────────────────────────
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH',    resource_path),
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', resource_path),

        # ── Gazebo ────────────────────────────────────────────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_ros_gz, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={
                'gz_args': f'-r -v 3 {world_file}'
            }.items(),
        ),

        # ── Clock bridge ─────────────────────────────────────────────────────
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='clock_bridge',
            arguments=[
                f'/world/{world_name}/clock'
                f'@rosgraph_msgs/msg/Clock'
                f'@gz.msgs.Clock'
            ],
            remappings=[(f'/world/{world_name}/clock', '/clock')],
            output='screen',
        ),

        # ── World pose bridge (ALL model poses → single TFMessage) ───────────
        # This single bridge replaces per-robot odometry bridges.
        # gt_pose_bridge.py filters robot poses and re-publishes Nav2 TF frames.
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='world_pose_bridge',
            arguments=[
                f'/world/{world_name}/pose/info'
                f'@tf2_msgs/msg/TFMessage'
                f'@ignition.msgs.Pose_V'
            ],
            remappings=[(f'/world/{world_name}/pose/info', '/world_poses')],
            output='screen',
        ),

        # ── GT Pose Bridge node ───────────────────────────────────────────────
        # Converts /world_poses → map→robot_N/odom→robot_N/base_link TF + /robot_N/odom
        Node(
            package='fleet_nav',
            executable='gt_pose_bridge',
            name='gt_pose_bridge',
            parameters=[{
                'num_robots':   LaunchConfiguration('num_robots'),
                'robot_prefix': 'robot_',
                'world_frame':  'map',
            }],
            output='screen',
        ),

        # ── Global Map Server (shared) ─────────────────────────────────────────
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'use_sim_time':    True,
                'yaml_filename':   map_file,
            }],
        ),

        # ── Route Server (shared, 1 instance for all robots) ─────────────────
        # NOTE: Install first → sudo apt install ros-humble-nav2-route
        Node(
            package='nav2_route',
            executable='route_server',
            name='route_server',
            output='screen',
            parameters=[{
                'use_sim_time':    True,
                'graph_filepath':  graph_file,
                'route_frame':     'map',
            }],
        ),

        # ── Lifecycle Manager for shared nodes ────────────────────────────────
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_shared',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'autostart':    True,
                'bond_timeout': 4.0,
                'node_names':   ['map_server', 'route_server'],
            }],
        ),
    ])
