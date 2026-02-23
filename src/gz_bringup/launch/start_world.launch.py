# gz_bringup/launch/start_world.launch.py
import os
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('gz_bringup')

    world_file = os.path.join(pkg_share, 'worlds', 'warehouse_only_map.world')
    res_path = os.path.join(pkg_share, 'models')

    world = 'warehouse_world'  

    return LaunchDescription([
        SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=res_path),
        SetEnvironmentVariable(name='IGN_GAZEBO_RESOURCE_PATH', value=res_path),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
            ),
            #launch_arguments={'gz_args': f'-s -r -v 4 {world_file}'}.items(), # GUI X
            launch_arguments={'gz_args': f'-r -v 4 {world_file}'}.items(), # GUI O
        ),
        # Map Server (전역 1개)
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': True, 'yaml_filename': os.path.join(pkg_share, 'maps', 'v1', 'v1_map.yaml')}],
        ),
        # Lifecycle Manager for map_server (auto configure/activate)
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'autostart': True,
                'bond_timeout': 2.0,
                'node_names': ['map_server']
            }],
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[f'/world/{world}/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock'],
            remappings=[(f'/world/{world}/clock', '/clock')],
            output='screen'
        ),
    ])
    
