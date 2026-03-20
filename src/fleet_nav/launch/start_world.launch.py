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
import tempfile
import yaml
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, SetEnvironmentVariable,
    IncludeLaunchDescription, TimerAction, OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def _robot_model_display(name: str) -> dict:
    return {
        'Alpha': 1,
        'Class': 'rviz_default_plugins/RobotModel',
        'Collision Enabled': False,
        'Description File': '',
        'Description Source': 'Topic',
        'Description Topic': {
            'Depth': 5,
            'Durability Policy': 'Volatile',
            'History Policy': 'Keep Last',
            'Reliability Policy': 'Reliable',
            'Value': f'/{name}/robot_description',
        },
        'Enabled': True,
        'Links': {
            'All Links Enabled': True,
            'Expand Joint Details': False,
            'Expand Link Details': False,
            'Expand Tree': False,
            'Link Tree Style': 'Links in Alphabetic Order',
        },
        'Name': name,
        'TF Prefix': name,
        'Update Interval': 0,
        'Value': True,
        'Visual Enabled': True,
    }


def _make_rviz_config(base_rviz_file: str, num_robots: int) -> str:
    """base fleet.rviz에 RobotModel 디스플레이를 num_robots 만큼 추가한 임시 파일 생성."""
    with open(base_rviz_file) as f:
        config = yaml.safe_load(f)

    robot_models = [_robot_model_display(f'robot_{i + 1}') for i in range(num_robots)]

    displays = config['Visualization Manager']['Displays']
    # Path 디스플레이 앞에 삽입 (Map 다음)
    insert_idx = next(
        (i for i, d in enumerate(displays) if d.get('Class') == 'rviz_default_plugins/Path'),
        len(displays),
    )
    for j, model in enumerate(robot_models):
        displays.insert(insert_idx + j, model)

    tmp = tempfile.NamedTemporaryFile(
        mode='w', suffix='.rviz', prefix='fleet_', delete=False
    )
    yaml.dump(config, tmp, default_flow_style=False, allow_unicode=True)
    tmp.close()
    return tmp.name


def _launch_nodes(context, *args, **kwargs):
    """Resolve launch arguments and return nodes."""
    pkg_fleet  = get_package_share_directory('fleet_nav')
    pkg_ros_gz = get_package_share_directory('ros_gz_sim')

    world_file        = LaunchConfiguration('world_file').perform(context)
    world_name        = LaunchConfiguration('world_name').perform(context)
    map_file          = LaunchConfiguration('map_file').perform(context)
    graph_file        = LaunchConfiguration('graph_file').perform(context)
    num_robots        = int(LaunchConfiguration('num_robots').perform(context))
    gui               = LaunchConfiguration('gui').perform(context).lower() != 'false'
    base_rviz_file    = os.path.join(pkg_fleet, 'rviz', 'fleet.rviz')
    route_params_file = os.path.join(pkg_fleet, 'params', 'route_server_params.yaml')

    rviz_file = _make_rviz_config(base_rviz_file, num_robots)

    return [
        # ── Gazebo ────────────────────────────────────────────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_ros_gz, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={
                'gz_args': f'-r -v 3 {world_file}' if gui else f'-r -v 3 -s {world_file}'
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

        # ── World pose bridge ─────────────────────────────────────────────────
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

        # ── GT Pose Bridge ────────────────────────────────────────────────────
        Node(
            package='fleet_nav',
            executable='gt_pose_bridge',
            name='gt_pose_bridge',
            parameters=[{
                'num_robots':   num_robots,
                'robot_prefix': 'robot_',
                'world_frame':  'map',
            }],
            output='screen',
        ),

        # ── Global Map Server ─────────────────────────────────────────────────
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'yaml_filename': map_file,
            }],
        ),

        # ── Route Server ──────────────────────────────────────────────────────
        Node(
            package='nav2_route',
            executable='route_server',
            name='route_server',
            output='screen',
            parameters=[
                route_params_file,
                {'graph_filepath': graph_file},
            ],
        ),

        # ── Lifecycle Manager ─────────────────────────────────────────────────
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

        # ── RViz ──────────────────────────────────────────────────────────────
        TimerAction(period=3.0, actions=[
            Node(
                package='rviz2', executable='rviz2', name='rviz2',
                arguments=['-d', rviz_file],
                parameters=[{'use_sim_time': True}],
                output='screen',
            ),
        ]),
    ]


def generate_launch_description():
    pkg_fleet = get_package_share_directory('fleet_nav')
    pkg_gz    = get_package_share_directory('gz_bringup')

    default_world_file = os.path.join(pkg_gz, 'worlds', 'warehouse_only_map.world')
    default_map_file   = os.path.join(os.path.expanduser('~'), 'Fleet_sim', 'maps', 'v2', 'v2.yaml')
    default_graph_file = os.path.join(pkg_fleet, 'params', 'nav_graph.geojson')

    fleet_models  = os.path.join(pkg_fleet, 'models')
    gz_models     = os.path.join(pkg_gz,    'models')
    resource_path = f'{fleet_models}:{gz_models}'

    return LaunchDescription([
        # ── Arguments ─────────────────────────────────────────────────────────
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

        # ── Nodes (resolved via OpaqueFunction) ───────────────────────────────
        OpaqueFunction(function=_launch_nodes),
    ])
