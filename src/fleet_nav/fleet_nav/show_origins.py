#!/usr/bin/env python3
"""
맵 중심과 Gazebo (0,0)을 RViz 마커로 표시.

  빨간 구 : Gazebo 월드 원점 (0, 0)
  파란 구 : map YAML에서 계산한 이미지 중심

두 구의 offset = map origin 보정값.
"""

import os
import yaml

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker


def _map_center(yaml_path: str):
    """map YAML + 이미지 크기로 맵 중심 좌표 계산."""
    with open(yaml_path, 'r') as f:
        data = yaml.safe_load(f)

    origin     = data['origin']       # [x, y, theta]
    resolution = data['resolution']   # m/px
    image_path = data['image']

    # 이미지 경로가 상대경로면 YAML 위치 기준으로 해석
    if not os.path.isabs(image_path):
        image_path = os.path.join(os.path.dirname(yaml_path), image_path)

    # PIL 없이 PGM 헤더에서 크기 파싱
    with open(image_path, 'rb') as f:
        line = f.readline().decode().strip()          # P5 or P2
        while True:
            line = f.readline().decode().strip()
            if not line.startswith('#'):
                break
        w, h = map(int, line.split())

    cx = origin[0] + (w / 2.0) * resolution
    cy = origin[1] + (h / 2.0) * resolution
    return cx, cy


class OriginMarkers(Node):
    def __init__(self):
        super().__init__('origin_markers')

        self.declare_parameter('map_yaml', '')

        yaml_path = self.get_parameter('map_yaml').value
        if yaml_path and os.path.isfile(yaml_path):
            self.cx, self.cy = _map_center(yaml_path)
            self.get_logger().info(
                f'맵 중심 계산: ({self.cx:.3f}, {self.cy:.3f})  [{yaml_path}]'
            )
        else:
            self.get_logger().warn('map_yaml 미지정 — 맵 중심 마커 비활성')
            self.cx = self.cy = None

        self.pub = self.create_publisher(Marker, '/origin_markers', 10)
        self.create_timer(1.0, self._publish)

    def _publish(self):
        self._sphere(0, 0.0, 0.0, 1.0, 0.0, 0.0, 'Gazebo_origin')
        if self.cx is not None:
            self._sphere(1, self.cx, self.cy, 0.0, 0.0, 1.0, 'Map_center')

    def _sphere(self, mid, x, y, r, g, b, ns):
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = ns
        m.id = mid
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = 0.3
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.5
        m.color.r = r
        m.color.g = g
        m.color.b = b
        m.color.a = 1.0
        self.pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = OriginMarkers()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
