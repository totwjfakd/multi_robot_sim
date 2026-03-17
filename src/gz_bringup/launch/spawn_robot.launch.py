# gz_bringup/launch/spawn_robot.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import LoadComposableNodes, Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory

from nav2_common.launch import RewrittenYaml, ReplaceString
from launch_ros.descriptions import ComposableNode, ParameterFile
import os

_GZ_SHARE = get_package_share_directory('gz_bringup')
_NAV2_SHARE = get_package_share_directory('nav2_launcher')

def launch_setup(context, *args, **kwargs):
    world = LaunchConfiguration('world_name').perform(context)
    name  = LaunchConfiguration('robot_name').perform(context)

    # URDF 파일 읽기
    urdf_file = os.path.join(_GZ_SHARE, 'urdf', 'x1_robot.urdf')
    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    # 센서 링크/센서 이름 (SDF에 맞춰 수정)
    lidar_link  = LaunchConfiguration('lidar_link').perform(context)
    lidar_name  = LaunchConfiguration('lidar_sensor').perform(context)
    rear_lidar_link = LaunchConfiguration('rear_lidar_link').perform(context)
    rear_lidar_name = LaunchConfiguration('rear_lidar_sensor').perform(context)
    imu_link    = LaunchConfiguration('imu_link').perform(context)
    imu_name    = LaunchConfiguration('imu_sensor').perform(context)
    use_dual_lidar = LaunchConfiguration('use_dual_lidar')
    use_composition = LaunchConfiguration('use_composition')
    container_name = LaunchConfiguration('container_name')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')
    container_name_value = container_name.perform(context)
    container_name_full = f'/{name}/{container_name_value}'

    # GZ 토픽 경로 구성
    gz_scan_front = f'/world/{world}/model/{name}/link/{lidar_link}/sensor/{lidar_name}/scan'
    gz_scan_rear = f'/world/{world}/model/{name}/link/{rear_lidar_link}/sensor/{rear_lidar_name}/scan'
    gz_imu  = f'/world/{world}/model/{name}/link/{imu_link}/sensor/{imu_name}/imu'
    
    # odom은 플러그인에 따라 경로가 다를 수 있음. 보편적으로 아래 둘 중 하나.
    gz_odom = f'/model/{name}/odometry'   # 필요시 '/world/{world}/model/{name}/odometry'

    # ROS 네임스페이스 토픽 (상대 경로로 정의하여 PushRosNamespace에 의해 /<ns>/... 로 적용)
    ros_scan = 'scan'
    ros_scan_front = 'scan_front'
    ros_scan_rear = 'scan_rear'
    ros_imu  = 'imu'
    ros_odom = 'odom'
    ros_cmd  = 'cmd_vel'

    # Nav2 파라미터 파일(<NS> 치환 포함) 준비: AMCL 등에서 공통 사용
    params_file_subst = ReplaceString(
        source_file=LaunchConfiguration('params_file'),
        replacements={'<NS>': name}
    )
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file_subst,
            root_key=name,
            param_rewrites={},
            convert_types=True
        ),
        allow_substs=True
    )
    nav2_bringup_launch = os.path.join(
        get_package_share_directory('nav2_launcher'),
        'launch', 'navigation_launch_multi.py'
    )

    # (중요) 여기선 'root_key'를 하위에서 다시 감싸므로
    # '<NS>' 치환만 적용된 원본 경로를 넘기면 충분합니다.
    nav2_params_for_include = ReplaceString(
        source_file=LaunchConfiguration('params_file'),
        replacements={'<NS>': name}
    )

    nav2_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_bringup_launch),
        launch_arguments={
            'namespace': name,                 # 로봇 네임스페이스
            'use_sim_time': 'true',
            'autostart': 'true',
            'params_file': nav2_params_for_include,  # bringup 내부에서 root_key=namespace로 감쌈
            'use_composition': use_composition,
            'container_name': container_name,
            'container_name_full': container_name_full,
            'use_respawn': use_respawn,
            'log_level': log_level,
        }.items(),
        condition=IfCondition(LaunchConfiguration('start_nav'))
    )
    return [
        # 1) 스포너 (SDF로 로봇 생성)
        Node(
            package='gz_bringup',
            executable='gz_spawner.py',
            name=f'{name}_spawner',
            parameters=[{
                'world_name': world,
                'sdf_file': LaunchConfiguration('sdf_file').perform(context),
                'robot_name': name,
                'x': float(LaunchConfiguration('x').perform(context)),
                'y': float(LaunchConfiguration('y').perform(context)),
                'z': float(LaunchConfiguration('z').perform(context)),
                'yaw': float(LaunchConfiguration('yaw').perform(context)),
            }],
            output='screen'),

        # 2) 브릿지들
        # LiDAR GZ->ROS (single mode: front scan directly to /scan)
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name=f'{name}_bridge_scan',
            arguments=[f'{gz_scan_front}@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan'],
            remappings=[(gz_scan_front, ros_scan)],
            condition=UnlessCondition(use_dual_lidar),
            output='screen'),
        # LiDAR GZ->ROS (dual mode: front/rear -> scan_front/scan_rear)
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name=f'{name}_bridge_scan_front',
            arguments=[f'{gz_scan_front}@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan'],
            remappings=[(gz_scan_front, ros_scan_front)],
            condition=IfCondition(use_dual_lidar),
            output='screen'),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name=f'{name}_bridge_scan_rear',
            arguments=[f'{gz_scan_rear}@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan'],
            remappings=[(gz_scan_rear, ros_scan_rear)],
            condition=IfCondition(use_dual_lidar),
            output='screen'),
        # Dual 180 LiDAR merge -> /scan
        Node(
            package='gz_bringup',
            executable='scan_merger.py',
            name='scan_merger',
            parameters=[{
                'front_topic': ros_scan_front,
                'rear_topic': ros_scan_rear,
                'output_topic': ros_scan,
                'output_frame_id': f'{name}/base_link',
                'front_x': float(LaunchConfiguration('front_lidar_x').perform(context)),
                'front_y': float(LaunchConfiguration('front_lidar_y').perform(context)),
                'front_yaw': float(LaunchConfiguration('front_lidar_yaw').perform(context)),
                'rear_x': float(LaunchConfiguration('rear_lidar_x').perform(context)),
                'rear_y': float(LaunchConfiguration('rear_lidar_y').perform(context)),
                'rear_yaw': float(LaunchConfiguration('rear_lidar_yaw').perform(context)),
                'publish_rate_hz': 20.0,
                'output_samples': 720,
            }],
            condition=IfCondition(use_dual_lidar),
            output='screen'),
        # IMU GZ->ROS
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name=f'{name}_bridge_imu',
            arguments=[f'{gz_imu}@sensor_msgs/msg/Imu@gz.msgs.IMU'],
            remappings=[(gz_imu, ros_imu)],
            output='screen'),
        # Odom GZ->ROS
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name=f'{name}_bridge_odom',
            arguments=[f'{gz_odom}@nav_msgs/msg/Odometry@gz.msgs.Odometry'],
            remappings=[(gz_odom, ros_odom)],
            output='screen'),
        # cmd_vel ROS->GZ
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name=f'{name}_bridge_cmd',
            arguments=[f'/model/{name}/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'],
            remappings=[(f'/model/{name}/cmd_vel', ros_cmd)],
            output='screen'),

        # 3) robot_state_publisher (URDF 프레임 퍼블리시) — 네임스페이스별 프레임
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_desc,
                         'use_sim_time': True,
                         'frame_prefix': f'{name}/'}],
            output='screen'),

        # 4) odom -> base_link TF 브로드캐스트 (네임스페이스별 프레임)
        Node(
            package='gz_bringup',
            executable='odom_to_tf.py',
            name='odom_to_tf',
            parameters=[{'odom_topic': ros_odom,
                         'odom_frame': f'{name}/odom',
                         'base_frame': f'{name}/base_link'}],
            output='screen'),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0','0','0','0','0','0', f'{name}/lidar_link', f'{name}/base_link/gpu_lidar'],
            output='screen'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0','0','0','0','0','0', f'{name}/lidar_rear_link', f'{name}/base_link/gpu_lidar_rear'],
            output='screen'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0','0','0','0','0','0', f'{name}/imu_link', f'{name}/base_link/imu_sensor'],
            output='screen'
        ),
        Node(
            condition=IfCondition(use_composition),
            package='rclcpp_components',
            executable='component_container_isolated',
            name=container_name_value,
            parameters=[configured_params, {'autostart': True}],
            arguments=['--ros-args', '--log-level', log_level],
            output='screen'
        ),
        Node(
            package='nav2_amcl', executable='amcl', name='amcl',
            parameters=[configured_params],
            remappings=[('map', '/map')],
            condition=UnlessCondition(use_composition),
            output='screen'
        ),
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_localization', output='screen',
            parameters=[{
                'use_sim_time': True, 'autostart': True, 'bond_timeout': 0.0,
                'node_names': ['amcl']
            }],
            condition=UnlessCondition(use_composition)
        ),
        LoadComposableNodes(
            condition=IfCondition(use_composition),
            target_container=container_name_full,
            composable_node_descriptions=[
                ComposableNode(
                    package='nav2_amcl',
                    plugin='nav2_amcl::AmclNode',
                    name='amcl',
                    parameters=[configured_params],
                    remappings=[('map', '/map')],
                ),
                ComposableNode(
                    package='nav2_lifecycle_manager',
                    plugin='nav2_lifecycle_manager::LifecycleManager',
                    name='lifecycle_manager_localization',
                    parameters=[{
                        'use_sim_time': True,
                        'autostart': True,
                        'bond_timeout': 0.0,
                        'node_names': ['amcl'],
                    }],
                ),
            ]
        ),
        nav2_include,
    ]
        # # 5) (옵션) 로봇 네임스페이스로 Nav2 포함 실행 (robot_nav.launch.py)
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         os.path.join(get_package_share_directory('nav2_launcher'), 'launch', 'robot_nav.launch.py')
        #     ),
        #     launch_arguments={
        #         'namespace': LaunchConfiguration('robot_name'),
        #         'params': LaunchConfiguration('params_file'),
        #         'use_sim_time': 'true',
        #         'autostart': 'true',
        #         'start_localization': LaunchConfiguration('start_localization'),
        #     }.items(),
        #     condition=IfCondition(LaunchConfiguration('start_nav')),
        # ),



def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world_name', default_value='warehouse_world'),
        DeclareLaunchArgument('robot_name', default_value='robot_1'),
        DeclareLaunchArgument('sdf_file',   default_value=os.path.join(_GZ_SHARE, 'models', 'X1_Config_5', 'model_multi.sdf')),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.02'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        # Nav2 실행 옵션 및 파라미터 (옵션)
        DeclareLaunchArgument('params_file', default_value=os.path.join(_NAV2_SHARE, 'params', 'robot_config.yaml')),
        DeclareLaunchArgument('start_nav', default_value='true'),  # ← Nav2 bringup 켜기/끄기 스위치
        DeclareLaunchArgument('use_composition', default_value='False'),
        DeclareLaunchArgument('container_name', default_value='nav2_container'),
        DeclareLaunchArgument('use_respawn', default_value='False'),
        DeclareLaunchArgument('log_level', default_value='info'),
        DeclareLaunchArgument('use_dual_lidar', default_value='true'),
        # 링크/센서 이름(SDF에 맞게)
        DeclareLaunchArgument('lidar_link',   default_value='base_link'),
        DeclareLaunchArgument('lidar_sensor', default_value='gpu_lidar'),
        DeclareLaunchArgument('rear_lidar_link', default_value='base_link'),
        DeclareLaunchArgument('rear_lidar_sensor', default_value='gpu_lidar_rear'),
        DeclareLaunchArgument('imu_link',     default_value='base_link'),
        DeclareLaunchArgument('imu_sensor',   default_value='imu_sensor'),
        DeclareLaunchArgument('front_lidar_x', default_value='0.5'),
        DeclareLaunchArgument('front_lidar_y', default_value='0.0'),
        DeclareLaunchArgument('front_lidar_yaw', default_value='0.0'),
        DeclareLaunchArgument('rear_lidar_x', default_value='-0.5'),
        DeclareLaunchArgument('rear_lidar_y', default_value='0.0'),
        DeclareLaunchArgument('rear_lidar_yaw', default_value='3.14159'),

        # URDF를 RSP에 넣을 내용(선택: 파일 읽어서 넘겨도 됨)
        DeclareLaunchArgument('robot_description', default_value=''),

        # 네임스페이스로 묶어서 노드 이름/파라미터 격리
        GroupAction([
            PushRosNamespace(LaunchConfiguration('robot_name')),
            OpaqueFunction(function=launch_setup),
        ])
    ])
