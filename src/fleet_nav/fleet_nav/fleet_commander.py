#!/usr/bin/env python3
"""
Fleet Commander Node
--------------------
각 로봇에게 nav_graph의 목적지 노드를 할당하고 이동시킵니다.

역할 분담:
  fleet_commander     : 지휘관 — 목적지 할당, 경로 계산 요청, 경로 추종 명령
  multi_robot_controller : 실행자 — FollowPath 액션 서버, Pure Pursuit cmd_vel 계산

흐름 (로봇 1대):
  1. odom으로 로봇 현재 pose 파악
  2. ComputeRoute(start=현재_pose, goal_id=목적지) → route_server → nav_msgs/Path
     └ route_server가 알아서 현재 위치에서 가장 가까운 그래프 노드 탐색
  3. FollowPath(path) → multi_robot_controller → cmd_vel
  4. 도착 후 다음 목적지 할당 → 반복
"""

import json
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from nav2_msgs.action import ComputeRoute, FollowPath
from action_msgs.msg import GoalStatus
from std_msgs.msg import String


COMMAND_TOPIC = '/fleet_nav/command'
STATUS_TOPIC = '/fleet_nav/status'


class RobotAgent:
    """단일 로봇의 Route → Follow 파이프라인 관리."""

    def __init__(self, name: str, node: Node):
        self.name         = name
        self.goal_node_id = None
        self.busy         = False
        self._node        = node
        self._cb_group    = ReentrantCallbackGroup()

        self._current_pose = None
        self._pose_lock    = threading.Lock()

        # 로봇 현재 pose 추적 (odom)
        node.create_subscription(
            Odometry,
            f'/{name}/odom',
            self._odom_cb,
            10,
            callback_group=self._cb_group,
        )

        # ComputeRoute 액션 클라이언트 (공유 route_server)
        self._route_client = ActionClient(
            node, ComputeRoute, '/compute_route',
            callback_group=self._cb_group,
        )

        # FollowPath 액션 클라이언트 (로봇별 multi_robot_controller)
        self._follow_client = ActionClient(
            node, FollowPath, f'/{name}/follow_path',
            callback_group=self._cb_group,
        )

    # ── Pose 추적 ─────────────────────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry):
        pose = PoseStamped()
        pose.header          = msg.header
        pose.header.frame_id = 'map'   # gt_pose_bridge: 좌표는 map 프레임 값
        pose.pose            = msg.pose.pose
        with self._pose_lock:
            self._current_pose = pose

    def _get_pose(self):
        with self._pose_lock:
            return self._current_pose

    def _publish_status(self, level: str, message: str, goal_node_id=None):
        publish_status = getattr(self._node, 'publish_status', None)
        if callable(publish_status):
            publish_status(
                level=level,
                message=message,
                robot_name=self.name,
                goal_node_id=goal_node_id,
            )

    # ── 목적지 할당 ───────────────────────────────────────────────────────────

    def is_ready(self) -> bool:
        return (self._route_client.server_is_ready() and
                self._follow_client.server_is_ready())

    def navigate_to(self, goal_node_id: int, goal_x: float, goal_y: float):
        """목적지 노드 ID와 좌표를 받아 이동 시작 (좌표 조회는 FleetCommander가 담당)."""
        if self.busy:
            message = f'[{self.name}] 이미 이동 중, 명령 무시'
            self._node.get_logger().warn(message)
            self._publish_status('warn', '이미 이동 중입니다.', goal_node_id)
            return False

        if not self.is_ready():
            message = f'[{self.name}] route/follow 서버 준비 전, 명령 무시'
            self._node.get_logger().warn(message)
            self._publish_status('warn', '내비게이션 서버가 아직 준비되지 않았습니다.', goal_node_id)
            return False

        pose = self._get_pose()
        if pose is None:
            message = f'[{self.name}] odom 미수신, 명령 무시'
            self._node.get_logger().warn(message)
            self._publish_status('warn', 'odom을 아직 받지 못했습니다.', goal_node_id)
            return False

        self.goal_node_id = goal_node_id
        self.busy         = True
        self._send_compute_route(pose, goal_node_id, goal_x, goal_y)
        self._publish_status('info', f'node {goal_node_id} 이동을 시작합니다.', goal_node_id)
        return True

    # ── 1단계: ComputeRoute ───────────────────────────────────────────────────

    def _send_compute_route(self, start_pose: PoseStamped, goal_id: int,
                            goal_x: float, goal_y: float):
        """
        use_poses=True  : start/goal 둘 다 pose 기반으로 가장 가까운 노드 탐색
        use_start=True  : start는 TF 아닌 제공된 pose 사용
        → start_id 기본값(0) 문제 회피, 항상 실제 현재 위치 기준으로 경로 계산
        """
        goal_pose = PoseStamped()
        goal_pose.header.frame_id    = 'map'
        goal_pose.pose.position.x    = float(goal_x)
        goal_pose.pose.position.y    = float(goal_y)
        goal_pose.pose.orientation.w = 1.0

        goal = ComputeRoute.Goal()
        goal.start     = start_pose
        goal.goal      = goal_pose
        goal.use_start = True
        goal.use_poses = True

        self._node.get_logger().info(
            f'[{self.name}] Route 요청: '
            f'({start_pose.pose.position.x:.2f}, {start_pose.pose.position.y:.2f})'
            f' → node {goal_id} ({goal_x:.2f}, {goal_y:.2f})'
        )
        future = self._route_client.send_goal_async(goal)
        future.add_done_callback(self._route_goal_response_cb)

    def _route_goal_response_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self._node.get_logger().warn(f'[{self.name}] Route 요청 거절')
            self._publish_status('warn', 'Route 요청이 거절되었습니다.', self.goal_node_id)
            self.busy = False
            return
        handle.get_result_async().add_done_callback(self._route_result_cb)

    def _route_result_cb(self, future):
        result = future.result().result
        status = future.result().status

        if status != GoalStatus.STATUS_SUCCEEDED or not result.path.poses:
            self._node.get_logger().error(
                f'[{self.name}] Route 계산 실패 (status={status})'
            )
            self._publish_status('error', f'Route 계산 실패 (status={status})', self.goal_node_id)
            self.busy = False
            return

        node_seq = [n.nodeid for n in result.route.nodes]
        self._node.get_logger().info(
            f'[{self.name}] Route 수신: {len(result.path.poses)}포즈, '
            f'노드 시퀀스={node_seq}'
        )
        self._publish_status('info', f'Route 계산 완료: {node_seq}', self.goal_node_id)
        self._send_follow_path(result.path)

    # ── 2단계: FollowPath ─────────────────────────────────────────────────────

    def _send_follow_path(self, path):
        goal = FollowPath.Goal()
        goal.path = path

        future = self._follow_client.send_goal_async(goal)
        future.add_done_callback(self._follow_goal_response_cb)

    def _follow_goal_response_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self._node.get_logger().warn(f'[{self.name}] FollowPath 거절')
            self._publish_status('warn', 'FollowPath 요청이 거절되었습니다.', self.goal_node_id)
            self.busy = False
            return
        handle.get_result_async().add_done_callback(self._follow_result_cb)

    def _follow_result_cb(self, future):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self._node.get_logger().info(
                f'[{self.name}] node {self.goal_node_id} 도착 완료'
            )
            self._publish_status('info', f'node {self.goal_node_id} 도착 완료', self.goal_node_id)
        else:
            self._node.get_logger().warn(
                f'[{self.name}] FollowPath 실패 (status={status})'
            )
            self._publish_status('warn', f'FollowPath 실패 (status={status})', self.goal_node_id)
        self.busy = False
        # self.assign_next_goal()  # 자동 순환 비활성화 — 외부 명령으로만 이동


# ─────────────────────────────────────────────────────────────────────────────

class FleetCommander(Node):

    def __init__(self):
        super().__init__('fleet_commander')

        self.declare_parameter('num_robots',        10)
        self.declare_parameter('robot_prefix',      'robot_')
        self.declare_parameter('graph_file',        '')
        self.declare_parameter('dispatch_interval', 2.0)

        num_robots   = self.get_parameter('num_robots').value
        robot_prefix = self.get_parameter('robot_prefix').value
        graph_file   = self.get_parameter('graph_file').value
        interval     = self.get_parameter('dispatch_interval').value

        self.node_coords = self._load_graph(graph_file)
        if not self.node_coords:
            self.get_logger().error('nav_graph 로드 실패. graph_file 파라미터 확인')
            return

        self._node_map = {nid: (x, y) for nid, x, y in self.node_coords}
        self._status_pub = self.create_publisher(String, STATUS_TOPIC, 10)
        self.create_subscription(String, COMMAND_TOPIC, self._command_cb, 10)

        self.get_logger().info(
            f'nav_graph 로드 완료: {len(self.node_coords)}개 노드'
        )

        # 로봇별 에이전트 생성
        self.agents = {
            f'{robot_prefix}{i + 1}': RobotAgent(f'{robot_prefix}{i + 1}', self)
            for i in range(num_robots)
        }

        self.get_logger().info(
            f'Fleet Commander 시작: {num_robots}대 (외부 명령 대기 중, dispatch_interval={interval}s)'
        )

    def publish_status(self, level: str, message: str,
                       robot_name: str = '', goal_node_id=None):
        payload = {
            'level': level,
            'message': message,
        }
        if robot_name:
            payload['robot_name'] = robot_name
        if goal_node_id is not None:
            payload['goal_node_id'] = int(goal_node_id)

        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self._status_pub.publish(msg)

    def _load_graph(self, graph_file: str) -> list:
        """GeoJSON에서 노드 [(id, x, y), ...] 로드."""
        if not graph_file:
            self.get_logger().warn('graph_file 미설정 — 테스트 좌표 사용')
            return [(0, 0.0, 0.0), (1, 5.0, 0.0), (2, 5.0, 5.0), (3, 0.0, 5.0)]
        try:
            with open(graph_file) as f:
                data = json.load(f)
            return [
                (int(feat['properties']['id']),
                 float(feat['geometry']['coordinates'][0]),
                 float(feat['geometry']['coordinates'][1]))
                for feat in data.get('features', [])
                if feat['geometry']['type'] == 'Point'
            ]
        except Exception as e:
            self.get_logger().error(f'graph_file 로드 오류: {e}')
            return []

    def _command_cb(self, msg: String):
        """GUI나 외부 노드에서 보내는 이동 명령을 처리."""
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().error(f'명령 JSON 파싱 실패: {exc}')
            self.publish_status('error', '명령 형식이 올바르지 않습니다.')
            return

        robot_name = str(payload.get('robot_name', '')).strip()
        goal_node_id = payload.get('goal_node_id')

        if not robot_name:
            self.publish_status('error', 'robot_name이 비어 있습니다.')
            return

        try:
            goal_node_id = int(goal_node_id)
        except (TypeError, ValueError):
            self.publish_status('error', 'goal_node_id는 정수여야 합니다.', robot_name=robot_name)
            return

        self.navigate_robot(robot_name, goal_node_id)

    def navigate_robot(self, robot_name: str, goal_node_id: int):
        """외부에서 특정 로봇을 특정 노드로 보내는 인터페이스."""
        agent = self.agents.get(robot_name)
        if agent is None:
            self.get_logger().error(f'로봇 {robot_name} 없음')
            self.publish_status('error', f'로봇 {robot_name} 없음', robot_name=robot_name,
                                goal_node_id=goal_node_id)
            return False

        coords = self._node_map.get(goal_node_id)
        if coords is None:
            self.get_logger().error(f'node {goal_node_id} 그래프에 없음')
            self.publish_status('error', f'node {goal_node_id} 그래프에 없음',
                                robot_name=robot_name, goal_node_id=goal_node_id)
            return False

        gx, gy = coords
        return agent.navigate_to(goal_node_id, gx, gy)


def main(args=None):
    rclpy.init(args=args)
    node = FleetCommander()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
