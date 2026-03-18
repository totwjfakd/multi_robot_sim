#!/usr/bin/env python3
"""
Fleet Commander Node
--------------------
각 로봇에게 nav_graph의 목적지 노드를 할당하고,
Route Server → Controller Server(RPP) 직접 호출로 이동시킵니다.

흐름:
  1. ComputeRoute(start_id, goal_id) → nav_msgs/Path
  2. FollowPath(path) → RPP가 cmd_vel 퍼블리시
  3. 도착 후 다음 노드로 반복

사용법:
  ros2 run fleet_nav fleet_commander \
    --ros-args -p num_robots:=10 -p graph_file:=/path/to/nav_graph.json
"""

import json
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputeRoute, FollowPath
from action_msgs.msg import GoalStatus


class RobotAgent:
    """단일 로봇의 Route → Follow 파이프라인을 관리합니다."""

    def __init__(self, name: str, node: Node, node_coords: list, start_node_id: int):
        self.name         = name
        self.node_coords  = node_coords          # list of (id, x, y)
        self.current_node = start_node_id        # 현재 위치한 그래프 노드 ID
        self.goal_node    = start_node_id        # 다음 목적지 노드 ID
        self.node_index   = 0                    # 순환 인덱스
        self.busy         = False
        self._node        = node
        self._cb_group    = ReentrantCallbackGroup()

        # Route Server (공유, 네임스페이스 없음)
        self._route_client = ActionClient(
            node,
            ComputeRoute,
            '/route_server',
            callback_group=self._cb_group,
        )

        # Controller Server follow_path (로봇별 네임스페이스)
        self._follow_client = ActionClient(
            node,
            FollowPath,
            f'/{name}/follow_path',
            callback_group=self._cb_group,
        )

    def is_ready(self) -> bool:
        return (self._route_client.server_is_ready() and
                self._follow_client.server_is_ready())

    def assign_next_goal(self):
        """다음 목적지 노드를 할당하고 이동 시작."""
        if self.busy or not self.is_ready():
            return

        # 노드 목록을 순환
        self.node_index = (self.node_index + 1) % len(self.node_coords)
        next_id, next_x, next_y = self.node_coords[self.node_index]
        self.goal_node = next_id

        self._send_compute_route(self.current_node, self.goal_node)

    def navigate_to(self, goal_node_id: int):
        """외부에서 특정 노드 ID로 이동 명령을 내릴 때 사용."""
        if self.busy:
            self._node.get_logger().warn(f'[{self.name}] 이미 이동 중. 명령 무시.')
            return
        self.goal_node = goal_node_id
        self._send_compute_route(self.current_node, goal_node_id)

    # ── 1단계: Route Server에 경로 요청 ──────────────────────────────────────

    def _send_compute_route(self, start_id: int, goal_id: int):
        goal = ComputeRoute.Goal()
        goal.start_id  = start_id
        goal.goal_id   = goal_id
        goal.use_start = True   # start_id 필드 사용 (공유 서버이므로 TF 자동탐색 불가)
        goal.use_poses = False  # 노드 ID 방식 사용

        self.busy = True
        self._node.get_logger().info(
            f'[{self.name}] Route 요청: node {start_id} → node {goal_id}'
        )
        future = self._route_client.send_goal_async(goal)
        future.add_done_callback(self._route_goal_response_cb)

    def _route_goal_response_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self._node.get_logger().warn(f'[{self.name}] Route 요청 거절됨')
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
            self.busy = False
            return

        self._node.get_logger().info(
            f'[{self.name}] Route 수신: {len(result.path.poses)}개 포즈'
        )
        self._send_follow_path(result.path)

    # ── 2단계: Controller Server에 경로 추종 요청 ────────────────────────────

    def _send_follow_path(self, path):
        goal = FollowPath.Goal()
        goal.path            = path
        goal.controller_id   = 'FollowPath'   # fleet_nav_params.yaml의 controller 이름
        goal.goal_checker_id = 'general_goal_checker'

        future = self._follow_client.send_goal_async(goal)
        future.add_done_callback(self._follow_goal_response_cb)

    def _follow_goal_response_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self._node.get_logger().warn(f'[{self.name}] FollowPath 거절됨')
            self.busy = False
            return
        handle.get_result_async().add_done_callback(self._follow_result_cb)

    def _follow_result_cb(self, future):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self._node.get_logger().info(
                f'[{self.name}] node {self.goal_node} 도착 완료'
            )
            self.current_node = self.goal_node
        else:
            self._node.get_logger().warn(
                f'[{self.name}] FollowPath 실패 (status={status}), 다음 목적지로 이동'
            )
        self.busy = False
        # 도착 후 자동으로 다음 목적지 할당
        self.assign_next_goal()


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
            self.get_logger().error('nav_graph 로드 실패. graph_file 파라미터를 확인하세요.')
            return

        self.get_logger().info(
            f'nav_graph 로드 완료: {len(self.node_coords)}개 노드'
        )

        # 로봇별 에이전트 생성 (시작 노드를 로봇마다 분산)
        self.agents = []
        for i in range(num_robots):
            name       = f'{robot_prefix}{i + 1}'
            start_node = self.node_coords[i % len(self.node_coords)][0]
            agent      = RobotAgent(name, self, self.node_coords, start_node)
            agent.node_index = i % len(self.node_coords)  # 시작 위치 분산
            self.agents.append(agent)

        # 주기적으로 idle 로봇에 다음 목적지 할당
        self.create_timer(interval, self._dispatch_tick)
        self.get_logger().info(
            f'Fleet Commander 시작: {num_robots}대, 주기={interval}s'
        )

    def _load_graph(self, graph_file: str) -> list:
        """nav_graph.geojson (GeoJSON FeatureCollection)에서 노드 (id, x, y) 리스트 반환."""
        if not graph_file:
            self.get_logger().warn('graph_file 미설정 — 테스트 좌표 사용')
            return [(0, 0.0, 0.0), (1, 5.0, 0.0), (2, 5.0, 5.0), (3, 0.0, 5.0)]
        try:
            with open(graph_file, 'r') as f:
                data = json.load(f)
            nodes = []
            for feature in data.get('features', []):
                if feature['geometry']['type'] == 'Point':
                    node_id = feature['properties']['id']
                    x, y   = feature['geometry']['coordinates'][:2]
                    nodes.append((node_id, x, y))
            return nodes
        except Exception as e:
            self.get_logger().error(f'graph_file 로드 오류: {e}')
            return []

    def _dispatch_tick(self):
        """idle 로봇에 다음 목적지 할당."""
        for agent in self.agents:
            if not agent.busy:
                agent.assign_next_goal()


def main(args=None):
    rclpy.init(args=args)
    node = FleetCommander()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
