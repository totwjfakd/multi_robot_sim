# Multi Robot Simulation

ROS2 Humble + Gazebo 기반 멀티 로봇 자율 주행 시뮬레이션 (창고 환경)

## 사전 요구사항

- Ubuntu 22.04
- ROS2 Humble
- Gazebo Sim (Garden 이상)
- Nav2, SLAM Toolbox

```bash
# colcon 설치 (없는 경우)
sudo apt install python3-colcon-common-extensions

# 의존성 설치
cd ~/multi_robot_sim
rosdep install --from-paths src -i -y --rosdistro=humble
```

## 빌드

```bash
cd ~/multi_robot_sim
colcon build --symlink-install
source install/setup.bash
```

## 실행

### 단일 로봇 (Gazebo + RViz + 브릿지)

```bash
ros2 launch gz_bringup bringup.launch.py
```

### 멀티 로봇

```bash
# 터미널 1: 월드 + 맵 서버
ros2 launch gz_bringup start_world.launch.py

# 터미널 2: robot_1 스폰 (Nav2 포함)
ros2 launch gz_bringup spawn_robot.launch.py \
  robot_name:=robot_1 x:=0.0 y:=0.0 yaw:=0.0

# 터미널 3: robot_2 스폰
ros2 launch gz_bringup spawn_robot.launch.py \
  robot_name:=robot_2 x:=2.0 y:=2.0 yaw:=1.57
```

### spawn_robot.launch.py 주요 인자

| 인자 | 기본값 | 설명 |
|------|--------|------|
| `robot_name` | `robot_1` | 로봇 네임스페이스 |
| `world_name` | `warehouse_world` | Gazebo 월드 이름 |
| `x`, `y`, `z` | `0.0, 0.0, 0.02` | 스폰 위치 |
| `yaw` | `0.0` | 초기 방향 (rad) |
| `start_nav` | `true` | Nav2 활성화 여부 |

## 네비게이션 목표 전송

RViz의 **2D Nav Goal** 도구를 사용하거나 CLI로 전송:

```bash
ros2 action send_goal /robot_1/navigate_to_pose \
  nav2_msgs/action/NavigateToPose \
  "{pose: {pose: {position: {x: 5.0, y: 5.0}, orientation: {w: 1.0}}}}"
```

## 패키지 구성

| 패키지 | 설명 |
|--------|------|
| `gz_bringup` | Gazebo 시뮬레이션 환경, 로봇 스폰, ROS-GZ 브릿지 |
| `nav2_launcher` | Nav2 네비게이션/로컬라이제이션 런치 및 파라미터 |
| `slam_toolbox` | SLAM 매핑 (slam_toolbox v2.6.10) |
