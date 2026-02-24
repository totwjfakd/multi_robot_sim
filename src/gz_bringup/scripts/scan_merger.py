#!/usr/bin/env python3

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class DualScanMerger(Node):
    def __init__(self):
        super().__init__('dual_scan_merger')

        self.declare_parameter('front_topic', 'scan_front')
        self.declare_parameter('rear_topic', 'scan_rear')
        self.declare_parameter('output_topic', 'scan')
        self.declare_parameter('output_frame_id', 'base_link')
        self.declare_parameter('output_samples', 720)
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('stale_timeout_sec', 0.4)
        self.declare_parameter('front_x', 0.5)
        self.declare_parameter('front_y', 0.0)
        self.declare_parameter('front_yaw', 0.0)
        self.declare_parameter('rear_x', -0.5)
        self.declare_parameter('rear_y', 0.0)
        self.declare_parameter('rear_yaw', math.pi)

        self.front_topic = self.get_parameter('front_topic').value
        self.rear_topic = self.get_parameter('rear_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.output_frame_id = self.get_parameter('output_frame_id').value
        self.output_samples = int(self.get_parameter('output_samples').value)
        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.stale_timeout_sec = float(self.get_parameter('stale_timeout_sec').value)
        self.front_pose = (
            float(self.get_parameter('front_x').value),
            float(self.get_parameter('front_y').value),
            float(self.get_parameter('front_yaw').value),
        )
        self.rear_pose = (
            float(self.get_parameter('rear_x').value),
            float(self.get_parameter('rear_y').value),
            float(self.get_parameter('rear_yaw').value),
        )

        self.front_scan: Optional[LaserScan] = None
        self.rear_scan: Optional[LaserScan] = None
        self.front_rx_time = self.get_clock().now()
        self.rear_rx_time = self.get_clock().now()

        self.scan_pub = self.create_publisher(LaserScan, self.output_topic, 10)
        self.create_subscription(LaserScan, self.front_topic, self.front_cb, 10)
        self.create_subscription(LaserScan, self.rear_topic, self.rear_cb, 10)
        self.create_timer(1.0 / max(1.0, self.publish_rate_hz), self.on_timer)

        self.get_logger().info(
            f'Merging dual scans: front="{self.front_topic}", rear="{self.rear_topic}" -> "{self.output_topic}"'
        )

    def front_cb(self, msg: LaserScan):
        self.front_scan = msg
        self.front_rx_time = self.get_clock().now()

    def rear_cb(self, msg: LaserScan):
        self.rear_scan = msg
        self.rear_rx_time = self.get_clock().now()

    def on_timer(self):
        if self.front_scan is None or self.rear_scan is None:
            return

        now = self.get_clock().now()
        front_age = (now - self.front_rx_time).nanoseconds * 1e-9
        rear_age = (now - self.rear_rx_time).nanoseconds * 1e-9
        if front_age > self.stale_timeout_sec or rear_age > self.stale_timeout_sec:
            return

        angle_min = -math.pi
        angle_max = math.pi
        samples = max(8, self.output_samples)
        angle_inc = (angle_max - angle_min) / float(samples - 1)
        merged = [math.inf] * samples

        range_min = min(self.front_scan.range_min, self.rear_scan.range_min)
        range_max = max(self.front_scan.range_max, self.rear_scan.range_max)

        self.project_scan(self.front_scan, self.front_pose, merged, angle_min, angle_inc, range_min, range_max)
        self.project_scan(self.rear_scan, self.rear_pose, merged, angle_min, angle_inc, range_min, range_max)

        out = LaserScan()
        # Use the fresher stamp among front/rear scans.
        if (
            self.front_scan.header.stamp.sec > self.rear_scan.header.stamp.sec
            or (
                self.front_scan.header.stamp.sec == self.rear_scan.header.stamp.sec
                and self.front_scan.header.stamp.nanosec >= self.rear_scan.header.stamp.nanosec
            )
        ):
            out.header.stamp = self.front_scan.header.stamp
        else:
            out.header.stamp = self.rear_scan.header.stamp
        out.header.frame_id = self.output_frame_id
        out.angle_min = angle_min
        out.angle_max = angle_max
        out.angle_increment = angle_inc
        out.time_increment = 0.0
        out.scan_time = 1.0 / max(1.0, self.publish_rate_hz)
        out.range_min = range_min
        out.range_max = range_max
        out.ranges = merged
        out.intensities = []
        self.scan_pub.publish(out)

    @staticmethod
    def project_scan(scan: LaserScan, pose, merged, angle_min, angle_inc, out_range_min, out_range_max):
        sx, sy, syaw = pose
        c = math.cos(syaw)
        s = math.sin(syaw)
        angle = scan.angle_min
        for r in scan.ranges:
            if math.isfinite(r) and scan.range_min < r < scan.range_max:
                lx = r * math.cos(angle)
                ly = r * math.sin(angle)
                bx = sx + c * lx - s * ly
                by = sy + s * lx + c * ly
                br = math.hypot(bx, by)
                if not (out_range_min < br < out_range_max):
                    angle += scan.angle_increment
                    continue
                ba = math.atan2(by, bx)
                idx = int(round((ba - angle_min) / angle_inc))
                if 0 <= idx < len(merged) and br < merged[idx]:
                    merged[idx] = br
            angle += scan.angle_increment


def main():
    rclpy.init()
    node = DualScanMerger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
