#!/usr/bin/env python3
import math
import subprocess
import rclpy
from rclpy.node import Node

class GzSpawner(Node):
    def __init__(self):
        super().__init__('gz_spawner')
        self.declare_parameter('world_name', 'warehouse_world')
        self.declare_parameter('robot_name', 'robot_1')
        self.declare_parameter('sdf_file', '')
        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('z', 0.02)
        self.declare_parameter('yaw', 0.0)

        # 파라미터 가져오기
        world = self.get_parameter('world_name').get_parameter_value().string_value
        name = self.get_parameter('robot_name').get_parameter_value().string_value
        sdf_path = self.get_parameter('sdf_file').get_parameter_value().string_value
        x = float(self.get_parameter('x').get_parameter_value().double_value)
        y = float(self.get_parameter('y').get_parameter_value().double_value)
        z = float(self.get_parameter('z').get_parameter_value().double_value)
        yaw = float(self.get_parameter('yaw').get_parameter_value().double_value)

        if not sdf_path:
            raise RuntimeError('sdf_file parameter is empty')

        # ign service 명령 구성
        service_name = f'/world/{world}/create'
        
        # 쿼터니언 계산
        qz = math.sin(yaw/2.0)
        qw = math.cos(yaw/2.0)
        
        # ign service 호출 명령 (올바른 메시지 타입 사용)
        req_msg = f'sdf_filename: "{sdf_path}" name: "{name}" pose {{position {{x: {x} y: {y} z: {z}}} orientation {{x: 0 y: 0 z: {qz} w: {qw}}}}}'
        
        cmd = [
            'ign', 'service',
            '-s', service_name,
            '--reqtype', 'ignition.msgs.EntityFactory',
            '--reptype', 'ignition.msgs.Boolean',
            '--timeout', '5000',
            '--req', req_msg
        ]

        self.get_logger().info(f'Spawning robot {name} at ({x}, {y}, {z}) with yaw={yaw}')
        self.get_logger().info(f'Command: {" ".join(cmd)}')

        try:
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
            if result.returncode == 0:
                self.get_logger().info(f'Successfully spawned robot {name}')
                self.get_logger().info(f'Output: {result.stdout.strip()}')
            else:
                self.get_logger().error(f'Failed to spawn robot {name}')
                self.get_logger().error(f'Error: {result.stderr.strip()}')
        except subprocess.TimeoutExpired:
            self.get_logger().error(f'Timeout while spawning robot {name}')
        except Exception as e:
            self.get_logger().error(f'Exception while spawning robot {name}: {e}')
        
        rclpy.shutdown()

def main():
    rclpy.init()
    node = GzSpawner()

if __name__ == '__main__':
    main()
