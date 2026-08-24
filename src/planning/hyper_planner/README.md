# hyper_planner

HYPER의 행동 결정과 차량 제어를 담당하는 C++ 패키지입니다. 교차로·경사로·장애물·주차 이벤트를 관리하고, 차선·정지선·신호·위치 정보를 바탕으로 조향과 속도 명령을 생성합니다.

## 구성

- `behavior_supervisor_with_parking_node`: 주행 모드와 이벤트 상태를 결정합니다.
- `controller_with_parking_node`: 차선 및 경로를 따라 `/cmd`, `/velocity`, `/steering_angle` 명령을 생성합니다.
- `follow_path_client_node`: 웨이포인트 CSV를 `nav_msgs/Path`로 변환해 nav2의 `follow_path` 액션에 목표로 보냅니다.
- `cmd_vel_to_ackermann_node`: nav2가 내는 `/cmd_vel`(Twist)을 `/velocity`, `/steering_angle`로 변환합니다.
- `config/parking_params.yaml`: 제어 및 이벤트 파라미터입니다.
- `config/nav2_controller.yaml`: nav2 `controller_server`(= `follow_path` 액션 서버) 파라미터입니다.

## 실행

```bash
ros2 launch hyper_planner parking_system_cpp.launch.py
```

이벤트 등록과 상세 운용 방법은 [README_KO.md](README_KO.md)를 참고하세요.

## nav2 경로 추종 (follow_path)

웨이포인트 CSV -> `nav_msgs/Path` -> nav2 `follow_path` 액션 -> `/cmd_vel` -> `/velocity`, `/steering_angle`.

### 설치 (최초 1회)

```bash
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
```

### 빌드

```bash
cd ~/HYPER
colcon build --packages-select hyper_planner
source install/setup.bash
```

### 실행

터미널마다 `source ~/HYPER/install/setup.bash`를 먼저 실행하세요.

```bash
# 1) 시뮬레이터 + 오도메트리 (TF: map -> odom -> body_link, /scan 필요)
ros2 launch hyper_launch sim.launch.py
ros2 launch hyper_launch odometry.launch.py

# 2) 액션 서버 (controller_server + lifecycle_manager + cmd_vel 변환)
ros2 launch hyper_planner nav2_controller.launch.py
# 실차는 use_sim_time:=false

# 3) 경로 전송
ros2 launch hyper_planner follow_path_client.launch.py \
  waypoint_csv:=$HOME/HYPER/src/planning/hyper_waypoint/waypoints/sim.csv
```

`controller_with_parking_node`도 `/velocity`, `/steering_angle`에 퍼블리시하므로
`parking_system_cpp.launch.py`와 동시에 실행하지 마세요.

### 재전송 / 취소

```bash
ros2 service call /follow_path_client/start std_srvs/srv/Trigger    # CSV 다시 읽어 재전송
ros2 service call /follow_path_client/cancel std_srvs/srv/Trigger   # 진행 중인 목표 취소
```

### RViz 시각화

```bash
rviz2 -d ~/HYPER/src/planning/hyper_planner/config/follow_path.rviz
```

Fixed Frame은 `map`입니다. 표시되는 항목:

| 표시 | 토픽 | 색 |
| --- | --- | --- |
| 웨이포인트 경로 (CSV) | `/follow_path_client/path` | 초록 |
| 컨트롤러가 받은 경로 | `/received_global_plan` | 파랑 |
| 로컬 플랜 | `/local_plan` | 노랑 |
| lookahead 점 | `/lookahead_point` | 빨강 |
| 로컬 코스트맵 | `/local_costmap/costmap` | costmap |
| footprint | `/local_costmap/published_footprint` | 자홍 |
| 라이다 | `/scan` | 주황 |

경로가 안 보이면 TF(`map` -> `odom` -> `body_link`)가 올라와 있는지 먼저 확인하세요.

### 상태 확인

```bash
ros2 action list | grep follow_path          # /follow_path 가 떠 있는지
ros2 topic echo /cmd_vel                     # nav2 출력
ros2 topic echo /velocity                    # 변환된 차량 명령
ros2 lifecycle get /controller_server        # active 여야 함
```

### 주요 파라미터

`follow_path_client_node` — `waypoint_csv`, `frame_id`(기본 `map`), `action_name`(기본 `follow_path`),
`controller_id`(기본 `FollowPath`), `goal_checker_id`(기본 `general_goal_checker`),
`min_spacing_m`(기본 `0.0`, 점 솎아내기), `auto_start`(기본 `true`), `shutdown_on_finish`(기본 `false`).

CSV는 헤더 이름으로 파싱하므로 `idx,x,y,yaw,frame_id` 형태와 레코더의 전체 열 형태 모두 됩니다.
`yaw`가 비어 있으면 다음 점을 향하는 방향으로 자동 계산합니다.

주행 튜닝은 [config/nav2_controller.yaml](config/nav2_controller.yaml),
`/cmd_vel` -> 조향 변환(`delta = atan(wheelbase * angular.z / linear.x)`)은 같은 파일의
`cmd_vel_to_ackermann` 항목을 보세요.

### 컨트롤러: MPPI

`FollowPath` 플러그인은 `nav2_mppi_controller`(motion_model `Ackermann`, `min_turning_r` 1.74 m)입니다.
RPP(Regulated Pure Pursuit)는 경로 추종 전용이라 장애물을 만나도 우회하지 않고 abort만 하므로,
연속 회피(슬라럼)를 위해 교체했습니다. MPPI는 CSV 경로를 참조로 두고 로컬 costmap을 보며
매 틱 궤적을 재최적화하므로 옆으로 비켰다가 경로로 복귀합니다.

회피가 약하면 `PathAlignCritic.cost_weight`를 낮추고 `ObstaclesCritic.critical_weight`를 올리세요.
반대로 경로를 너무 벗어나면 그 반대로 조정합니다.

**속도 튜닝**: MPPI에는 RPP의 `desired_linear_vel` 같은 목표 속도가 없습니다. `vx_max`는 상한일 뿐이고
실제 순항 속도는 다음 균형점으로 정해집니다.

```
순항속도 ~= PathFollowCritic.cost_weight * model_dt * vx_std^2 / gamma
```

nav2 기본값(cost_weight 5, gamma 0.015)은 vx_max 0.5인 차동구동 로봇 기준이라 이 식으로 0.67 m/s가
나옵니다. `vx_max: 2.5`로 올려도 0.86 m/s로 기어다니는 이유가 이것입니다(실측). 현재값
(cost_weight 15, gamma 0.010)은 3.0 m/s로 계산되어 `vx_max: 2.5`에 포화합니다 -- 직선에서 2.5 m/s,
곡선/장애물에서는 critic들이 자동 감속.

**주의**: critic의 `offset_from_furthest`는 미터가 아니라 **경로 점 개수**입니다. nav2 기본값은
전역 플래너의 촘촘한 경로(약 0.05 m 간격)를 가정하는데 우리 CSV는 0.6 m 간격이라, 기본값 20은
MPPI가 보는 pruned path(약 7.8 m / 13점)보다 길어서 PathAlignCritic이 조용히 통째로 비활성화됩니다.

**알려진 제약 (언덕)**: 라이다가 수평 고정인데 `ekf_local`이 `two_d_mode: true`라 tf에 피치가
0으로 들어갑니다. 그래서 driving_course의 언덕(x 37~46, y -16.5~+25) 오목 구간에서 노면 자체가
장애물로 마킹됩니다. 임시로 `obstacle_max_range: 6.0`으로 마킹 거리를 잘라 두었습니다(노면 히트는
대략 7~8 m 앞에서 생깁니다). 근본 해결은 tf에 실제 피치를 싣거나, 언덕 구간에서 obstacle_layer를
끄는 지오펜스입니다.
