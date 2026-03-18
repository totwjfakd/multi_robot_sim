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
#
# Overridable arguments:
#   world_file  : path to .world file
#   world_name  : Gazebo world name (used in bridge topic names)
#   map_file    : path to map YAML
#   graph_file  : path to nav_graph GeoJSON

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription, TimerAction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_fleet  = get_package_share_directory('fleet_nav')
    pkg_gz     = get_package_share_directory('gz_bringup')
    pkg_ros_gz = get_package_share_directory('ros_gz_sim')

    # ── Default paths (computed at parse time) ────────────────────────────────
    default_world_file  = os.path.join(pkg_gz, 'worlds', 'warehouse_only_map.world')
    default_map_file    = os.path.join(os.path.expanduser('~'), 'Fleet_sim', 'maps', 'v2', 'v2.yaml')
    default_graph_file  = os.path.join(pkg_fleet, 'params', 'nav_graph.geojson')
    rviz_file           = os.path.join(pkg_fleet, 'rviz', 'fleet.rviz')

    fleet_models  = os.path.join(pkg_fleet, 'models')
    gz_models     = os.path.join(pkg_gz,    'models')
    resource_path = f'{fleet_models}:{gz_models}'

    return LaunchDescription([
        # ── Arguments ────────────────────────────────────────────────────────
        DeclareLaunchArgument('num_robots',  default_value='10',
                              description='Total number of robots'),
        DeclareLaunchArgument('gui',         default_value='true',
                              description='Launch Gazebo with GUI'),
        DeclareLaunchArgument('world_file',  default_value=default_world_file,
                              description='Path to Gazebo .world file'),
        DeclareLaunchArgument('world_name',  default_value='warehouse_world',
                              description='Gazebo world name (used in bridge topic names)'),
        DeclareLaunchArgument('map_file',    default_value=default_map_file,
                              description='Path to occupancy map YAML'),
        DeclareLaunchArgument('graph_file',  default_value=default_graph_file,
                              description='Path to navigation graph GeoJSON'),

        # ── Environment ───────────────────────────────────────────────────────
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH',    resource_path),
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', resource_path),

        # ── Gazebo ────────────────────────────────────────────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_ros_gz, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={
                'gz_args': ['-r -v 3 ', LaunchConfiguration('world_file')]
            }.items(),
        ),

        # ── Clock bridge ─────────────────────────────────────────────────────
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='clock_bridge',
            arguments=[[
                '/world/', LaunchConfiguration('world_name'),
                '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock'
            ]],
            remappings=[(['/world/', LaunchConfiguration('world_name'), '/clock'], '/clock')],
            output='screen',
        ),

        # ── World pose bridge (ALL model poses → single TFMessage) ───────────
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='world_pose_bridge',
            arguments=[[
                '/world/', LaunchConfiguration('world_name'),
                '/pose/info@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V'
            ]],
            remappings=[(['/world/', LaunchConfiguration('world_name'), '/pose/info'], '/world_poses')],
            output='screen',
        ),

        # ── GT Pose Bridge node ───────────────────────────────────────────────
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
                'use_sim_time': True,
                'yaml_filename': LaunchConfiguration('map_file'),
            }],
        ),

        # ── Route Server (shared, 1 instance for all robots) ─────────────────
        Node(
            package='nav2_route',
            executable='route_server',
            name='route_server',
            output='screen',
            parameters=[{
                'use_sim_time':   True,
                'graph_filepath': LaunchConfiguration('graph_file'),
                'route_frame':    'map',
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

        TimerAction(period=3.0, actions=[
            Node(
                package='rviz2', executable='rviz2', name='rviz2',
                arguments=['-d', rviz_file],
                parameters=[{'use_sim_time': True}],
                output='screen',
            ),
        ]),
    ])
