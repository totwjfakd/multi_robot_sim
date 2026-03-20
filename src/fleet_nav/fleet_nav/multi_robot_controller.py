#!/usr/bin/env python3
"""
Multi-Robot Controller Node  (Differential Drive Pure Pursuit)
--------------------------------------------------------------
단일 타이머 + numpy 벡터 연산으로 N대 로봇의 cmd_vel을 계산.

흐름:
  _execute()     : FollowPath goal을 _active_goals에 등록 후 done_event 대기
  _control_loop(): 매 제어 주기마다 활성 로봇 전체를 벡터 연산으로 처리 → publish

Diff-Drive Pure Pursuit:
  alpha = atan2(ty - ry, tx - rx) - yaw
  kappa = 2 * sin(alpha) / L
  omega = v * kappa
"""

import math
import threading

import numpy as np
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


class ActiveGoal:
    """진행 중인 FollowPath goal."""
    def __init__(self, goal_handle, path):
        self.goal_handle = goal_handle
        self.path        = path   # list[PoseStamped]
        self.target_idx  = 0
        self.done_event  = threading.Event()
        self.result      = None   # 'succeeded' | 'canceled' | 'aborted'


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
        ctrl_freq      = self.get_parameter('control_frequency').value

        self._cb_group    = ReentrantCallbackGroup()
        self._states      = {}   # name → RobotState
        self._cmd_pubs    = {}   # name → Publisher<Twist>
        self._active_goals = {}  # name → ActiveGoal | None
        self._goals_lock  = threading.Lock()

        for i in range(num_robots):
            name = f'{prefix}{i + 1}'
            self._states[name]       = RobotState()
            self._active_goals[name] = None

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

        # 중앙 제어 타이머
        self.create_timer(1.0 / ctrl_freq, self._control_loop,
                          callback_group=self._cb_group)

        self.get_logger().info(
            f'MultiRobotController 시작: {num_robots}대 로봇 '
            f'(제어주기 {ctrl_freq}Hz)'
        )

    # ── Pose 업데이트 ─────────────────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry, name: str):
        q   = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        st = self._states[name]
        with st.lock:
            st.x   = msg.pose.pose.position.x
            st.y   = msg.pose.pose.position.y
            st.yaw = yaw

    # ── FollowPath 액션 ───────────────────────────────────────────────────────

    def _execute(self, goal_handle, name: str):
        """goal을 등록하고 완료 이벤트를 기다린다."""
        path = goal_handle.request.path.poses
        if not path:
            goal_handle.succeed()
            return FollowPath.Result()

        active = ActiveGoal(goal_handle, path)
        with self._goals_lock:
            self._active_goals[name] = active

        active.done_event.wait()   # _control_loop이 완료 시 set()

        with self._goals_lock:
            self._active_goals[name] = None

        if active.result == 'succeeded':
            goal_handle.succeed()
        elif active.result == 'canceled':
            goal_handle.canceled()
        else:
            goal_handle.abort()

        return FollowPath.Result()

    # ── 중앙 제어 루프 (벡터 연산) ────────────────────────────────────────────

    def _control_loop(self):
        with self._goals_lock:
            items = [(n, ag) for n, ag in self._active_goals.items()
                     if ag is not None]

        if not items:
            return

        names    = []
        states   = []   # (rx, ry, ryaw, tx, ty) per robot
        finished = []   # (name, active_goal, result) — 이번 주기에 완료된 것

        for name, ag in items:
            # 취소 요청 처리
            if ag.goal_handle.is_cancel_requested:
                self._cmd_pubs[name].publish(Twist())
                finished.append((name, ag, 'canceled'))
                continue

            st = self._states[name]
            with st.lock:
                rx, ry, ryaw = st.x, st.y, st.yaw

            path = ag.path

            # 목표 도달 확인
            gx = path[-1].pose.position.x
            gy = path[-1].pose.position.y
            if math.hypot(gx - rx, gy - ry) < self.goal_tol:
                self._cmd_pubs[name].publish(Twist())
                finished.append((name, ag, 'succeeded'))
                continue

            # lookahead 포인트
            ag.target_idx = self._find_lookahead(rx, ry, path, ag.target_idx)
            tx = path[ag.target_idx].pose.position.x
            ty = path[ag.target_idx].pose.position.y

            names.append(name)
            states.append((rx, ry, ryaw, tx, ty))

        # ── 완료 처리 ──────────────────────────────────────────────────────────
        for name, ag, result in finished:
            ag.result = result
            ag.done_event.set()

        if not names:
            return

        # ── 벡터 연산 ──────────────────────────────────────────────────────────
        arr    = np.array(states)                        # (N, 5)
        alphas = np.arctan2(arr[:, 4] - arr[:, 1],
                            arr[:, 3] - arr[:, 0]) - arr[:, 2]
        alphas = np.arctan2(np.sin(alphas), np.cos(alphas))   # [-π, π]
        omegas = np.clip(
            self.lin_vel * 2.0 * np.sin(alphas) / self.lookahead,
            -self.max_ang, self.max_ang,
        )

        # ── 순차 publish ───────────────────────────────────────────────────────
        for name, omega in zip(names, omegas):
            cmd = Twist()
            cmd.linear.x  = self.lin_vel
            cmd.angular.z = float(omega)
            self._cmd_pubs[name].publish(cmd)

    # ── 유틸 ─────────────────────────────────────────────────────────────────

    def _find_lookahead(self, rx: float, ry: float, path, start: int) -> int:
        for i in range(start, len(path)):
            if math.hypot(path[i].pose.position.x - rx,
                          path[i].pose.position.y - ry) >= self.lookahead:
                return i
        return len(path) - 1


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
