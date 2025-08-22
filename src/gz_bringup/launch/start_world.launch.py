# gz_bringup/launch/start_world.launch.py
import os
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    world_file = '/home/hanbaek/multirobot_sim/src/gz_bringup/worlds/warehouse_only_map.world'
    res_path = '/home/hanbaek/multirobot_sim/src/gz_bringup/models'

    world = 'warehouse_world'  

    return LaunchDescription([
        SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=res_path),
        SetEnvironmentVariable(name='IGN_GAZEBO_RESOURCE_PATH', value=res_path),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={'gz_args': f'-r -v 4 {world_file}'}.items(),
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[f'/world/{world}/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock'],
            remappings=[(f'/world/{world}/clock', '/clock')],
            output='screen'
        ),
    ])
    
