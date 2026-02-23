from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import ReplaceString
import os


def generate_launch_description():
    namespace        = LaunchConfiguration('namespace')
    params_file      = LaunchConfiguration('params')
    map_yaml_file    = LaunchConfiguration('map')
    use_sim_time     = LaunchConfiguration('use_sim_time')
    autostart        = LaunchConfiguration('autostart')
    use_composition  = LaunchConfiguration('use_composition')
    use_respawn      = LaunchConfiguration('use_respawn')
    container_name   = LaunchConfiguration('container_name')
    start_localiz    = LaunchConfiguration('start_localization')

    nav2_bringup_dir = get_package_share_directory('nav2_launcher')
    launch_dir       = nav2_bringup_dir + '/launch'
    default_params   = os.path.join(nav2_bringup_dir, 'params', 'robot_config.yaml')

    # <NS> 플레이스홀더를 실제 네임스페이스로 치환한 파라미터 파일 경로
    params_preprocessed = ReplaceString(
        source_file=params_file,
        replacements={'<NS>': namespace}
    )

    return LaunchDescription([
        # 공통 인자
        DeclareLaunchArgument('namespace', default_value='robot_1'),
        DeclareLaunchArgument('params',    default_value=default_params),
        DeclareLaunchArgument('map',       default_value=''),  # 빈 값이면 글로벌 map_server 사용 가정
        DeclareLaunchArgument('use_sim_time',   default_value='true'),
        DeclareLaunchArgument('autostart',      default_value='true'),
        DeclareLaunchArgument('use_composition', default_value='False'),
        DeclareLaunchArgument('use_respawn',     default_value='False'),
        DeclareLaunchArgument('container_name',  default_value='nav2_container'),
        DeclareLaunchArgument('start_localization', default_value='false'),

        # Localization (map_server+amcl) 포함: 필요 시 비활성화 가능
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_dir + '/localization_launch.py'),
            condition=IfCondition(start_localiz),
            launch_arguments={
                'namespace': namespace,
                'map': map_yaml_file,
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'params_file': params_preprocessed,
                'use_composition': use_composition,
                'use_respawn': use_respawn,
                'container_name': container_name,
            }.items()
        ),

        # Navigation 서버군 포함
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_dir + '/navigation_launch.py'),
            launch_arguments={
                'namespace': namespace,
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'params_file': params_preprocessed,
                'use_composition': use_composition,
                'use_respawn': use_respawn,
                'container_name': container_name,
            }.items()
        ),
    ])
