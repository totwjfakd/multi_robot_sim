import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_fleet = get_package_share_directory('fleet_nav')
    default_graph_file = os.path.join(pkg_fleet, 'params', 'nav_graph.geojson')

    return LaunchDescription([
        DeclareLaunchArgument(
            'num_robots',
            default_value='10',
            description='GUI에 표시할 로봇 수',
        ),
        DeclareLaunchArgument(
            'robot_prefix',
            default_value='robot_',
            description='로봇 이름 prefix',
        ),
        DeclareLaunchArgument(
            'graph_file',
            default_value=default_graph_file,
            description='목적지 선택용 nav graph GeoJSON 경로',
        ),
        DeclareLaunchArgument(
            'window_title',
            default_value='Fleet Command GUI',
            description='GUI 창 제목',
        ),
        Node(
            package='fleet_nav',
            executable='fleet_command_gui',
            name='fleet_command_gui',
            output='screen',
            parameters=[{
                'num_robots': LaunchConfiguration('num_robots'),
                'robot_prefix': LaunchConfiguration('robot_prefix'),
                'graph_file': LaunchConfiguration('graph_file'),
                'window_title': LaunchConfiguration('window_title'),
            }],
        ),
    ])
