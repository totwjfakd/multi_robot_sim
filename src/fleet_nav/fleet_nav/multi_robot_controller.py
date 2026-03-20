#!/usr/bin/env python3
"""
Multi-Robot Controller Node  (Differential Drive Pure Pursuit)
--------------------------------------------------------------
fleet_commander 기능을 통합한 단일 노드.

외부 명령 (/fleet_nav/command, JSON)
  → ComputeRoute (route_server)
  → FollowPath (내부 액션 서버, intra-process)
  → 벡터 Pure Pursuit → cmd_vel

Diff-Drive Pure Pursuit:
  alpha = atan2(ty - ry, tx - rx) - yaw
  kappa = 2 * sin(alpha) / L
  omega = v * kappa
"""

import json
import math
import threading

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from nav2_msgs.action import ComputeRoute, FollowPath
from std_msgs.msg import String


COMMAND_TOPIC = '/fleet_nav/command'
STATUS_TOPIC  = '/fleet_nav/status'


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
        self.declare_parameter('graph_file',        '')

        num_robots     = self.get_parameter('num_robots').value
        prefix         = self.get_parameter('robot_prefix').value
        self.lookahead = self.get_parameter('lookahead_dist').value
        self.lin_vel   = self.get_parameter('linear_vel').value
        self.max_ang   = self.get_parameter('max_angular_vel').value
        self.goal_tol  = self.get_parameter('goal_tolerance').value
        ctrl_freq      = self.get_parameter('control_frequency').value
        graph_file     = self.get_parameter('graph_file').value

        # ── graph 로드 ────────────────────────────────────────────────────────
        self._node_map = self._load_graph(graph_file)

        # ── 상태/명령 토픽 ────────────────────────────────────────────────────
        self._status_pub = self.create_publisher(String, STATUS_TOPIC, 10)
        self._cb_group   = ReentrantCallbackGroup()
        self.create_subscription(String, COMMAND_TOPIC, self._command_cb, 10,
                                 callback_group=self._cb_group)

        # ── 로봇별 리소스 ─────────────────────────────────────────────────────
        self._states       = {}   # name → RobotState
        self._cmd_pubs     = {}   # name → Publisher<Twist>
        self._active_goals = {}   # name → ActiveGoal | None
        self._nav_busy     = {}   # name → bool  (ComputeRoute 진행 중 여부)
        self._route_clients = {}  # name → ActionClient<ComputeRoute>
        self._follow_clients = {} # name → ActionClient<FollowPath>
        self._goals_lock   = threading.Lock()

        for i in range(num_robots):
            name = f'{prefix}{i + 1}'
            self._states[name]        = RobotState()
            self._active_goals[name]  = None
            self._nav_busy[name]      = False

            self.create_subscription(
                Odometry, f'/{name}/odom',
                lambda msg, n=name: self._odom_cb(msg, n),
                10, callback_group=self._cb_group,
            )

            self._cmd_pubs[name] = self.create_publisher(
                Twist, f'/{name}/cmd_vel', 10
            )

            # FollowPath 액션 서버 (경로 추종 요청 수신)
            ActionServer(
                self, FollowPath, f'/{name}/follow_path',
                execute_callback=lambda gh, n=name: self._execute(gh, n),
                goal_callback=lambda _: GoalResponse.ACCEPT,
                cancel_callback=lambda _: CancelResponse.ACCEPT,
                callback_group=self._cb_group,
            )

            # ComputeRoute 클라이언트 (route_server 요청)
            self._route_clients[name] = ActionClient(
                self, ComputeRoute, '/compute_route',
                callback_group=self._cb_group,
            )

            # FollowPath 클라이언트 (자기 자신 서버로 intra-process)
            self._follow_clients[name] = ActionClient(
                self, FollowPath, f'/{name}/follow_path',
                callback_group=self._cb_group,
            )

        # 중앙 제어 타이머
        self.create_timer(1.0 / ctrl_freq, self._control_loop,
                          callback_group=self._cb_group)

        self.get_logger().info(
            f'MultiRobotController 시작: {num_robots}대 '
            f'(그래프 {len(self._node_map)}노드, 제어주기 {ctrl_freq}Hz)'
        )

    # ── Graph ─────────────────────────────────────────────────────────────────

    def _load_graph(self, graph_file: str) -> dict:
        """GeoJSON → {node_id: (x, y)}"""
        if not graph_file:
            self.get_logger().warn('graph_file 미설정 — 테스트 좌표 사용')
            return {0: (0.0, 0.0), 1: (5.0, 0.0), 2: (5.0, 5.0), 3: (0.0, 5.0)}
        try:
            with open(graph_file) as f:
                data = json.load(f)
            return {
                int(feat['properties']['id']): (
                    float(feat['geometry']['coordinates'][0]),
                    float(feat['geometry']['coordinates'][1]),
                )
                for feat in data.get('features', [])
                if feat['geometry']['type'] == 'Point'
            }
        except Exception as e:
            self.get_logger().error(f'graph_file 로드 오류: {e}')
            return {}

    # ── 상태 퍼블리시 ─────────────────────────────────────────────────────────

    def _publish_status(self, level: str, message: str,
                        robot_name: str = '', goal_node_id=None):
        payload = {'level': level, 'message': message}
        if robot_name:
            payload['robot_name'] = robot_name
        if goal_node_id is not None:
            payload['goal_node_id'] = int(goal_node_id)
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self._status_pub.publish(msg)

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

    def _get_pose_stamped(self, name: str) -> PoseStamped | None:
        st = self._states[name]
        with st.lock:
            if st.x == 0.0 and st.y == 0.0:
                return None
            pose = PoseStamped()
            pose.header.frame_id    = 'map'
            pose.pose.position.x    = st.x
            pose.pose.position.y    = st.y
            pose.pose.orientation.w = 1.0
            return pose

    # ── 외부 명령 처리 ────────────────────────────────────────────────────────

    def _command_cb(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f'명령 JSON 파싱 실패: {e}')
            self._publish_status('error', '명령 형식이 올바르지 않습니다.')
            return

        robot_name   = str(payload.get('robot_name', '')).strip()
        goal_node_id = payload.get('goal_node_id')

        if not robot_name:
            self._publish_status('error', 'robot_name이 비어 있습니다.')
            return
        try:
            goal_node_id = int(goal_node_id)
        except (TypeError, ValueError):
            self._publish_status('error', 'goal_node_id는 정수여야 합니다.',
                                 robot_name=robot_name)
            return

        self.navigate_robot(robot_name, goal_node_id)

    def navigate_robot(self, robot_name: str, goal_node_id: int):
        if robot_name not in self._states:
            self.get_logger().error(f'로봇 {robot_name} 없음')
            self._publish_status('error', f'로봇 {robot_name} 없음',
                                 robot_name=robot_name, goal_node_id=goal_node_id)
            return

        coords = self._node_map.get(goal_node_id)
        if coords is None:
            self.get_logger().error(f'node {goal_node_id} 그래프에 없음')
            self._publish_status('error', f'node {goal_node_id} 그래프에 없음',
                                 robot_name=robot_name, goal_node_id=goal_node_id)
            return

        if self._nav_busy[robot_name]:
            self.get_logger().warn(f'[{robot_name}] 이미 이동 중')
            self._publish_status('warn', '이미 이동 중입니다.',
                                 robot_name=robot_name, goal_node_id=goal_node_id)
            return

        pose = self._get_pose_stamped(robot_name)
        if pose is None:
            self.get_logger().warn(f'[{robot_name}] odom 미수신')
            self._publish_status('warn', 'odom을 아직 받지 못했습니다.',
                                 robot_name=robot_name, goal_node_id=goal_node_id)
            return

        self._nav_busy[robot_name] = True
        self._publish_status('info', f'node {goal_node_id} 이동을 시작합니다.',
                             robot_name=robot_name, goal_node_id=goal_node_id)
        self._send_compute_route(robot_name, pose, goal_node_id, *coords)

    # ── ComputeRoute ──────────────────────────────────────────────────────────

    def _send_compute_route(self, name: str, start: PoseStamped,
                            goal_node_id: int, gx: float, gy: float):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id    = 'map'
        goal_pose.pose.position.x    = gx
        goal_pose.pose.position.y    = gy
        goal_pose.pose.orientation.w = 1.0

        goal = ComputeRoute.Goal()
        goal.start     = start
        goal.goal      = goal_pose
        goal.use_start = True
        goal.use_poses = True

        self.get_logger().info(
            f'[{name}] Route 요청: '
            f'({start.pose.position.x:.2f}, {start.pose.position.y:.2f})'
            f' → node {goal_node_id} ({gx:.2f}, {gy:.2f})'
        )
        future = self._route_clients[name].send_goal_async(goal)
        future.add_done_callback(
            lambda f, n=name, nid=goal_node_id: self._route_goal_cb(f, n, nid)
        )

    def _route_goal_cb(self, future, name: str, goal_node_id: int):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn(f'[{name}] Route 요청 거절')
            self._publish_status('warn', 'Route 요청이 거절되었습니다.',
                                 robot_name=name, goal_node_id=goal_node_id)
            self._nav_busy[name] = False
            return
        handle.get_result_async().add_done_callback(
            lambda f, n=name, nid=goal_node_id: self._route_result_cb(f, n, nid)
        )

    def _route_result_cb(self, future, name: str, goal_node_id: int):
        result = future.result().result
        status = future.result().status

        if status != GoalStatus.STATUS_SUCCEEDED or not result.path.poses:
            self.get_logger().error(f'[{name}] Route 계산 실패 (status={status})')
            self._publish_status('error', f'Route 계산 실패 (status={status})',
                                 robot_name=name, goal_node_id=goal_node_id)
            self._nav_busy[name] = False
            return

        node_seq = [n.nodeid for n in result.route.nodes]
        self.get_logger().info(
            f'[{name}] Route 수신: {len(result.path.poses)}포즈, 노드={node_seq}'
        )
        self._publish_status('info', f'Route 계산 완료: {node_seq}',
                             robot_name=name, goal_node_id=goal_node_id)
        self._send_follow_path(name, result.path, goal_node_id)

    # ── FollowPath (자기 자신 액션 서버로) ────────────────────────────────────

    def _send_follow_path(self, name: str, path, goal_node_id: int):
        goal = FollowPath.Goal()
        goal.path = path
        future = self._follow_clients[name].send_goal_async(goal)
        future.add_done_callback(
            lambda f, n=name, nid=goal_node_id: self._follow_goal_cb(f, n, nid)
        )

    def _follow_goal_cb(self, future, name: str, goal_node_id: int):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn(f'[{name}] FollowPath 거절')
            self._publish_status('warn', 'FollowPath 요청이 거절되었습니다.',
                                 robot_name=name, goal_node_id=goal_node_id)
            self._nav_busy[name] = False
            return
        handle.get_result_async().add_done_callback(
            lambda f, n=name, nid=goal_node_id: self._follow_result_cb(f, n, nid)
        )

    def _follow_result_cb(self, future, name: str, goal_node_id: int):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'[{name}] node {goal_node_id} 도착 완료')
            self._publish_status('info', f'node {goal_node_id} 도착 완료',
                                 robot_name=name, goal_node_id=goal_node_id)
        else:
            self.get_logger().warn(f'[{name}] FollowPath 실패 (status={status})')
            self._publish_status('warn', f'FollowPath 실패 (status={status})',
                                 robot_name=name, goal_node_id=goal_node_id)
        self._nav_busy[name] = False

    # ── FollowPath 액션 서버 ──────────────────────────────────────────────────

    def _execute(self, goal_handle, name: str):
        path = goal_handle.request.path.poses
        if not path:
            goal_handle.succeed()
            return FollowPath.Result()

        active = ActiveGoal(goal_handle, path)
        with self._goals_lock:
            self._active_goals[name] = active

        active.done_event.wait()

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
        states   = []
        finished = []

        for name, ag in items:
            if ag.goal_handle.is_cancel_requested:
                self._cmd_pubs[name].publish(Twist())
                finished.append((ag, 'canceled'))
                continue

            st = self._states[name]
            with st.lock:
                rx, ry, ryaw = st.x, st.y, st.yaw

            path = ag.path
            gx = path[-1].pose.position.x
            gy = path[-1].pose.position.y
            if math.hypot(gx - rx, gy - ry) < self.goal_tol:
                self._cmd_pubs[name].publish(Twist())
                finished.append((ag, 'succeeded'))
                continue

            ag.target_idx = self._find_lookahead(rx, ry, path, ag.target_idx)
            tx = path[ag.target_idx].pose.position.x
            ty = path[ag.target_idx].pose.position.y

            names.append(name)
            states.append((rx, ry, ryaw, tx, ty))

        for ag, result in finished:
            ag.result = result
            ag.done_event.set()

        if not names:
            return

        arr    = np.array(states)
        alphas = np.arctan2(arr[:, 4] - arr[:, 1],
                            arr[:, 3] - arr[:, 0]) - arr[:, 2]
        alphas = np.arctan2(np.sin(alphas), np.cos(alphas))
        omegas = np.clip(
            self.lin_vel * 2.0 * np.sin(alphas) / self.lookahead,
            -self.max_ang, self.max_ang,
        )

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
