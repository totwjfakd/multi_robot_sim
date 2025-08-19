import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 패키지 경로 가져오기
    pkg_share = get_package_share_directory('gz_bringup')
    
    # world 파일 경로
    world_file = '/home/hanbaek/multirobot_sim/src/gz_bringup/worlds/warehouse.world'
    
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
    
    # ROS2-Gazebo 브리지: odometry 토픽 연결
    bridge_odom = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry'],
        output='screen'
    )
    
    return LaunchDescription([
        model_path,
        gazebo,
        bridge_cmd_vel,
        bridge_odom
    ])