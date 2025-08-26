# gz_bringup/launch/nav2_bringup_robot.launch.py
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    robot_name = LaunchConfiguration('robot_name')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    log_level = LaunchConfiguration('log_level')

    # Paths
    nav2_launcher_dir = get_package_share_directory('nav2_launcher')

    # Nav2 core stack (controller/planner/behavior/bt/lcmgr) without map_server
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_launcher_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'namespace': robot_name,
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'params_file': params_file,
            'use_composition': 'False',
            'use_respawn': 'False',
            'log_level': log_level,
        }.items(),
    )

    # AMCL per robot (parameters must match namespaced topics/frames)
    amcl_node = GroupAction([
        PushRosNamespace(robot_name),
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
        ),
    ])``

    return LaunchDescription([
        DeclareLaunchArgument('robot_name', default_value='robot_1'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(
                nav2_launcher_dir, 'params', 'nav2_multirobot_params_1.yaml'
            ),
        ),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('log_level', default_value='info'),

        # Launch Nav2 stack and AMCL for this robot
        navigation_launch,
        amcl_node,
    ])

