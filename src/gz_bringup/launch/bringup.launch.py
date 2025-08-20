import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 패키지 경로 가져오기
    pkg_share = get_package_share_directory('gz_bringup')
    
    # world 파일 경로
    world_file = '/home/hanbaek/multirobot_sim/src/gz_bringup/worlds/warehouse.world'
    
    # URDF 파일 경로 (절대 경로 사용)
    urdf_file = '/home/hanbaek/multirobot_sim/src/gz_bringup/urdf/x1_robot.urdf'
    
    # 모델 경로 환경변수 설정
    model_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value='/home/hanbaek/multirobot_sim/src/gz_bringup/models'
    )
    
    # Gazebo 실행
    gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', world_file, '-v', '4'],
        output='screen'
    )
    
    # ROS2-Gazebo 브리지: cmd_vel 토픽 연결
    bridge_cmd_vel = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'],
        output='screen'
    )
    
    # ROS2-Gazebo 브리지: odometry 토픽 연결 (TF 브로드캐스트 포함)
    bridge_odom = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry'],
        parameters=[{'qos_overrides./odom.subscription.reliability': 'reliable'}],
        output='screen'
    )
    
    # ROS2-Gazebo 브리지: LiDAR와 IMU 토픽 연결
    bridge_scan = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan'],
        output='screen'
    )
    bridge_imu = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/imu@sensor_msgs/msg/Imu@gz.msgs.IMU'],
        output='screen'
    )
    
    # URDF 파일 읽기
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}],
    )
    
    # Odometry 토픽을 TF로 변환하는 Python 노드
    odom_to_tf_node = Node(
        package='gz_bringup',
        executable='odom_to_tf.py',
        name='odom_to_tf',
        output='screen'
    )
    
    # Gazebo frame을 ROS2 표준 frame으로 연결
    tf_gazebo_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'lidar_link', 'X1/base_link/gpu_lidar']
    )
    
    tf_gazebo_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'imu_link', 'X1/base_link/imu_sensor']
    )
    
    # RVIZ2 실행
    rviz_config_file = '/home/hanbaek/multirobot_sim/visualization.rviz'
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )
    
    return LaunchDescription([
        model_path,
        gazebo,
        bridge_cmd_vel,
        bridge_odom,
        bridge_scan,
        bridge_imu,
        robot_state_publisher,
        odom_to_tf_node,
        tf_gazebo_lidar,
        tf_gazebo_imu,
        rviz
    ])