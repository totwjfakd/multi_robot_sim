#!/usr/bin/env python3
"""
Multi-Robot Controller Node  (Differential Drive Pure Pursuit)
--------------------------------------------------------------
단일 노드에서 N대 로봇의 FollowPath action을 처리.
각 로봇 pose를 메모리에 유지하고, Diff-Drive Pure Pursuit으로 cmd_vel 계산.

Diff-Drive Pure Pursuit:
  alpha = atan2(ty - ry, tx - rx) - yaw   (lookahead 방향각)
  kappa = 2 * sin(alpha) / L              (경로 곡률)
  omega = v * kappa                        (각속도)

교체 대상:
  controller_server  × N  → 이 노드 1개
  lifecycle_manager  × N  → 불필요 (일반 노드)
"""

import math
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from nav2_msgs.action import FollowPath


class RobotState:
    """로봇 1대의 pose (thread-safe)."""
    def __init__(self):
        self.x   = 0.0
        self.y   = 0.0
        self.yaw = 0.0
        self.lock = threading.Lock()


class MultiRobotController(Node):

    def __init__(self):
        super().__init__('multi_robot_controller')

        self.declare_parameter('num_robots',        10)
        self.declare_parameter('robot_prefix',      'robot_')
        self.declare_parameter('lookahead_dist',    0.8)
        self.declare_parameter('linear_vel',        0.5)
        self.declare_parameter('max_angular_vel',   1.5)
        self.declare_parameter('goal_tolerance',    0.3)
        self.declare_parameter('control_frequency', 10.0)

        num_robots     = self.get_parameter('num_robots').value
        prefix         = self.get_parameter('robot_prefix').value
        self.lookahead = self.get_parameter('lookahead_dist').value
        self.lin_vel   = self.get_parameter('linear_vel').value
        self.max_ang   = self.get_parameter('max_angular_vel').value
        self.goal_tol  = self.get_parameter('goal_tolerance').value
        self.ctrl_freq = self.get_parameter('control_frequency').value

        self._cb_group = ReentrantCallbackGroup()
        self._states   = {}   # name → RobotState
        self._cmd_pubs = {}   # name → Publisher<Twist>

        for i in range(num_robots):
            name = f'{prefix}{i + 1}'
            self._states[name] = RobotState()

            self.create_subscription(
                Odometry,
                f'/{name}/odom',
                lambda msg, n=name: self._odom_cb(msg, n),
                10,
                callback_group=self._cb_group,
            )

            self._cmd_pubs[name] = self.create_publisher(
                Twist, f'/{name}/cmd_vel', 10
            )

            ActionServer(
                self,
                FollowPath,
                f'/{name}/follow_path',
                execute_callback=lambda gh, n=name: self._execute(gh, n),
                goal_callback=lambda _: GoalResponse.ACCEPT,
                cancel_callback=lambda _: CancelResponse.ACCEPT,
                callback_group=self._cb_group,
            )

        self.get_logger().info(
            f'MultiRobotController 시작: {num_robots}대 로봇'
        )

    # ── Pose 업데이트 ─────────────────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry, name: str):
        q    = msg.pose.pose.orientation
        yaw  = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        st = self._states[name]
        with st.lock:
            st.x   = msg.pose.pose.position.x
            st.y   = msg.pose.pose.position.y
            st.yaw = yaw

    # ── FollowPath 실행 ──────────────────────────────────────────────────────

    def _execute(self, goal_handle, name: str):
        path = goal_handle.request.path.poses
        pub  = self._cmd_pubs[name]

        if not path:
            goal_handle.succeed()
            return FollowPath.Result()

        st         = self._states[name]
        rate       = self.create_rate(self.ctrl_freq)
        target_idx = 0

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                pub.publish(Twist())
                goal_handle.canceled()
                return FollowPath.Result()

            with st.lock:
                rx, ry, ryaw = st.x, st.y, st.yaw

            # 목표 도달 확인
            gx = path[-1].pose.position.x
            gy = path[-1].pose.position.y
            if math.hypot(gx - rx, gy - ry) < self.goal_tol:
                pub.publish(Twist())
                goal_handle.succeed()
                return FollowPath.Result()

            # lookahead 포인트
            target_idx = self._find_lookahead(rx, ry, path, target_idx)
            tx = path[target_idx].pose.position.x
            ty = path[target_idx].pose.position.y

            pub.publish(self._diff_pure_pursuit(rx, ry, ryaw, tx, ty))
            rate.sleep()

        pub.publish(Twist())
        goal_handle.abort()
        return FollowPath.Result()

    # ── Diff-Drive Pure Pursuit ──────────────────────────────────────────────

    def _find_lookahead(self, rx: float, ry: float, path, start: int) -> int:
        """start 이후에서 lookahead_dist 이상 떨어진 첫 포인트 반환."""
        for i in range(start, len(path)):
            if math.hypot(path[i].pose.position.x - rx,
                          path[i].pose.position.y - ry) >= self.lookahead:
                return i
        return len(path) - 1

    def _diff_pure_pursuit(self, rx, ry, ryaw, tx, ty) -> Twist:
        """
        Differential Drive Pure Pursuit
          alpha : 로봇 진행 방향과 lookahead 방향의 각도 차
          kappa : 경로 곡률 = 2·sin(alpha) / L
          omega : 각속도   = v · kappa
        """
        alpha = math.atan2(ty - ry, tx - rx) - ryaw
        alpha = math.atan2(math.sin(alpha), math.cos(alpha))  # [-π, π] 정규화

        kappa = 2.0 * math.sin(alpha) / self.lookahead
        omega = self.lin_vel * kappa
        omega = max(-self.max_ang, min(self.max_ang, omega))

        cmd = Twist()
        cmd.linear.x  = self.lin_vel
        cmd.angular.z = omega
        return cmd


def main(args=None):
    rclpy.init(args=args)
    node = MultiRobotController()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
