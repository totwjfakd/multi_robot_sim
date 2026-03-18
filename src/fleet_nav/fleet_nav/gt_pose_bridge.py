#!/usr/bin/env python3
"""
GT Pose Bridge Node
-------------------
Subscribes to Gazebo world poses (bridged as TFMessage) and re-publishes them
as Nav2-compatible TF frames and Odometry topics for each robot.

TF tree produced per robot:
  map (static) → robot_N/odom → robot_N/base_link  ← GT pose from Gazebo

This replaces AMCL — no localization error, perfect ground truth.

Expected bridge (in launch file):
  /world/{world}/pose/info  @  tf2_msgs/msg/TFMessage  @  ignition.msgs.Pose_V
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

import tf2_ros
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry


class GtPoseBridge(Node):
    def __init__(self):
        super().__init__('gt_pose_bridge')

        self.declare_parameter('robot_prefix', 'robot_')
        self.declare_parameter('num_robots', 10)
        self.declare_parameter('world_frame', 'map')

        robot_prefix = self.get_parameter('robot_prefix').value
        num_robots   = self.get_parameter('num_robots').value
        self.world_frame = self.get_parameter('world_frame').value

        # Build set of expected robot model names: robot_1, robot_2, ...
        self.robot_names = [f'{robot_prefix}{i + 1}' for i in range(num_robots)]
        self.robot_set   = set(self.robot_names)

        self.tf_broadcaster     = tf2_ros.TransformBroadcaster(self)
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        # Publish static map → robot_N/odom (identity) for all robots at startup.
        # Since we use GT pose, odom frame = map frame (no drift).
        self._publish_static_odom_frames()

        # One Odometry publisher per robot  (e.g. /robot_1/odom)
        self.odom_pubs = {
            name: self.create_publisher(Odometry, f'/{name}/odom', 10)
            for name in self.robot_names
        }

        # Subscribe to bridged world poses
        # Bridge command (in launch):
        #   /world/warehouse_world/pose/info
        #   @ tf2_msgs/msg/TFMessage
        #   @ ignition.msgs.Pose_V
        self.create_subscription(
            TFMessage,
            '/world_poses',
            self._poses_callback,
            QoSProfile(depth=10,
                       reliability=QoSReliabilityPolicy.BEST_EFFORT),
        )

        self.get_logger().info(
            f'GT Pose Bridge started for {num_robots} robots: {self.robot_names[:3]}...'
        )

    def _publish_static_odom_frames(self):
        """Publish static map → robot_N/odom (identity) for every robot."""
        static_tfs = []
        stamp = self.get_clock().now().to_msg()
        for name in self.robot_names:
            tf = TransformStamped()
            tf.header.stamp        = stamp
            tf.header.frame_id     = self.world_frame   # 'map'
            tf.child_frame_id      = f'{name}/odom'
            tf.transform.rotation.w = 1.0               # identity quaternion
            static_tfs.append(tf)
        self.static_broadcaster.sendTransform(static_tfs)
        self.get_logger().info(
            f'Published static map → robot_N/odom for {len(static_tfs)} robots'
        )

    def _poses_callback(self, msg: TFMessage):
        """
        Receives all model poses from Gazebo world pose bridge.
        Filters robot models and re-publishes with correct Nav2 frame names.

        Gazebo publishes:
          header.frame_id  = 'warehouse_world'  (or world name)
          child_frame_id   = 'robot_1'           (model name)

        We remap to:
          header.frame_id  = 'robot_1/odom'
          child_frame_id   = 'robot_1/base_link'
        """
        dynamic_tfs = []

        for tf in msg.transforms:
            child = tf.child_frame_id

            # Skip link-level poses (e.g. 'robot_1::base_link') — use model pose only
            if '::' in child:
                continue

            if child not in self.robot_set:
                continue

            # --- TF: robot_N/odom → robot_N/base_link ---
            nav_tf = TransformStamped()
            nav_tf.header.stamp    = self.get_clock().now().to_msg()
            nav_tf.header.frame_id = f'{child}/odom'
            nav_tf.child_frame_id  = f'{child}/base_link'
            nav_tf.transform       = tf.transform
            dynamic_tfs.append(nav_tf)

            # --- Odometry topic: /robot_N/odom ---
            odom = Odometry()
            odom.header.stamp        = nav_tf.header.stamp
            odom.header.frame_id     = f'{child}/odom'
            odom.child_frame_id      = f'{child}/base_link'
            odom.pose.pose.position.x    = tf.transform.translation.x
            odom.pose.pose.position.y    = tf.transform.translation.y
            odom.pose.pose.position.z    = tf.transform.translation.z
            odom.pose.pose.orientation   = tf.transform.rotation
            # Covariance: very small (GT = perfect)
            odom.pose.covariance[0]  = 1e-6
            odom.pose.covariance[7]  = 1e-6
            odom.pose.covariance[35] = 1e-6

            if child in self.odom_pubs:
                self.odom_pubs[child].publish(odom)

        if dynamic_tfs:
            self.tf_broadcaster.sendTransform(dynamic_tfs)


def main(args=None):
    rclpy.init(args=args)
    node = GtPoseBridge()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
