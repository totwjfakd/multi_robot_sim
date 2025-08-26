# gz_bringup/launch/spawn_robot.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory

from nav2_common.launch import RewrittenYaml, ReplaceString
from launch_ros.descriptions import ParameterFile
import os

def launch_setup(context, *args, **kwargs):
    world = LaunchConfiguration('world_name').perform(context)
    name  = LaunchConfiguration('robot_name').perform(context)
    
    # URDF 파일 읽기
    urdf_file = '/home/hanbaek/multirobot_sim/src/gz_bringup/urdf/x1_robot.urdf'
    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    # 센서 링크/센서 이름 (SDF에 맞춰 수정)
    lidar_link  = LaunchConfiguration('lidar_link').perform(context)
    lidar_name  = LaunchConfiguration('lidar_sensor').perform(context)
    imu_link    = LaunchConfiguration('imu_link').perform(context)
    imu_name    = LaunchConfiguration('imu_sensor').perform(context)

    # GZ 토픽 경로 구성
    gz_scan = f'/world/{world}/model/{name}/link/{lidar_link}/sensor/{lidar_name}/scan'
    gz_imu  = f'/world/{world}/model/{name}/link/{imu_link}/sensor/{imu_name}/imu'
    
    # odom은 플러그인에 따라 경로가 다를 수 있음. 보편적으로 아래 둘 중 하나.
    gz_odom = f'/model/{name}/odometry'   # 필요시 '/world/{world}/model/{name}/odometry'

    # ROS 네임스페이스 토픽 (상대 경로로 정의하여 PushRosNamespace에 의해 /<ns>/... 로 적용)
    ros_scan = 'scan'
    ros_imu  = 'imu'
    ros_odom = 'odom'
    ros_cmd  = 'cmd_vel'

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
        # LiDAR GZ->ROS
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name=f'{name}_bridge_scan',
            arguments=[f'{gz_scan}@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan'],
            remappings=[(gz_scan, ros_scan)],
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
            arguments=['0','0','0','0','0','0', f'{name}/imu_link', f'{name}/base_link/imu_sensor'],
            output='screen'
        ),
        Node(
            package='nav2_amcl', executable='amcl', name='amcl',
            parameters=[{
                'use_sim_time': True,
                'tf_broadcast': True,
                'global_frame_id': 'map',
                'odom_frame_id': f'{name}/odom',
                'base_frame_id': f'{name}/base_link',
                'scan_topic': 'scan',
            }],
            remappings=[('map', '/map')],
            output='screen'
        ),
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_localization', output='screen',
            parameters=[{
                'use_sim_time': True, 'autostart': True, 'bond_timeout': 0.0,
                'node_names': ['amcl']
            }]
        ),
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
        DeclareLaunchArgument('sdf_file',   default_value='/home/hanbaek/multirobot_sim/src/gz_bringup/models/X1_Config_5/model_multi.sdf'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.02'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        # Nav2 실행 옵션 및 파라미터 (옵션)
        # DeclareLaunchArgument('start_nav', default_value='true'),
        # DeclareLaunchArgument('params_file', default_value='/home/hanbaek/multirobot_sim/src/nav2_launcher/params/robot_config.yaml'),
        # DeclareLaunchArgument('start_localization', default_value='false'),

        # 링크/센서 이름(SDF에 맞게)
        DeclareLaunchArgument('lidar_link',   default_value='base_link'),
        DeclareLaunchArgument('lidar_sensor', default_value='gpu_lidar'),
        DeclareLaunchArgument('imu_link',     default_value='base_link'),
        DeclareLaunchArgument('imu_sensor',   default_value='imu_sensor'),

        # URDF를 RSP에 넣을 내용(선택: 파일 읽어서 넘겨도 됨)
        DeclareLaunchArgument('robot_description', default_value=''),

        # 네임스페이스로 묶어서 노드 이름/파라미터 격리
        GroupAction([
            PushRosNamespace(LaunchConfiguration('robot_name')),
            OpaqueFunction(function=launch_setup),
        ])
    ])
