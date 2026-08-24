# hyper_planner

HYPER의 행동 결정과 차량 제어를 담당하는 C++ 패키지입니다. 교차로·경사로·장애물·주차 이벤트를 관리하고, 차선·정지선·신호·위치 정보를 바탕으로 조향과 속도 명령을 생성합니다.

## 구성

- `follow_path_client_node`: 웨이포인트 CSV를 `nav_msgs/Path`로 변환해 nav2의 `follow_path` 액션에 목표로 보냅니다. 코스 전체를 목표 하나로 보내 한 바퀴 도는 용도입니다.
- `mission_manager_node`: `config/mission.yaml`의 스텝 큐(주행/정지/신호 대기)를 순서대로 실행합니다. 대회 미션 주행은 이쪽입니다.
- `cmd_vel_to_ackermann_node`: nav2가 내는 `/cmd_vel`(Twist)을 `/velocity`, `/steering_angle`로 변환합니다.
- `config/parking_params.yaml`: 제어 및 이벤트 파라미터입니다.
- `config/nav2_controller.yaml`: nav2 `controller_server`(= `follow_path` 액션 서버) 파라미터입니다.
- `config/mission.yaml`: 미션 시퀀스(`steps`)와 코스 위 이벤트 지점(`labels`) 정의입니다.

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
| 현재 미션 세그먼트 경로 | `/mission_manager/path` | 초록 |
| 웨이포인트 경로 (CSV) | `/follow_path_client/path` | 진초록 |
| MPPI가 추종 중인 참조 경로 | `/transformed_global_plan` | 파랑 |
| MPPI 후보/최적 궤적 | `/trajectories` | 마커 |
| RPP가 받은 경로 (후진 주차 구간만) | `/received_global_plan` | 노랑 |
| lookahead 점 (후진 주차 구간만) | `/lookahead_point` | 빨강 |
| 로컬 코스트맵 | `/local_costmap/costmap` | costmap |
| footprint | `/local_costmap/published_footprint` | 자홍 |
| 라이다 | `/scan` | 주황 |

`/received_global_plan`과 `/lookahead_point`은 RPP 전용 토픽입니다. 기본 주행 컨트롤러가
MPPI로 바뀐 뒤로는 `ReverseFollowPath`(후진 주차) 세그먼트가 돌 때만 나옵니다. 평상시
주행 중에 컨트롤러가 무엇을 보고 있는지는 `/transformed_global_plan`으로 확인하세요.

MPPI 쪽 두 토픽은 `nav2_controller.yaml`의 `FollowPath.visualize: true`일 때만 나갑니다.
경로가 안 보이면 그 값과 TF(`map` -> `odom` -> `body_link`)를 먼저 확인하세요.

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
`min_spacing_m`(기본 `0.0`, 점 솎아내기), `auto_start`(기본 `true`), `shutdown_on_finish`(기본 `false`),
`start_from_nearest`(기본 `true`), `robot_base_frame`(기본 `body_link`), `tf_timeout_sec`(기본 `5.0`),
`lead_in_spacing_m`(기본 `0.5`), `max_start_distance_m`(기본 `0.0` = 제한 없음).

CSV는 헤더 이름으로 파싱하므로 `idx,x,y,yaw,frame_id` 형태와 레코더의 전체 열 형태 모두 됩니다.
`yaw`가 비어 있으면 다음 점을 향하는 방향으로 자동 계산합니다.

주행 튜닝은 [config/nav2_controller.yaml](config/nav2_controller.yaml),
`/cmd_vel` -> 조향 변환(`delta = atan(wheelbase * angular.z / linear.x)`)은 같은 파일의
`cmd_vel_to_ackermann` 항목을 보세요.

### 가장 가까운 점부터 출발 (start_from_nearest)

차량이 CSV의 첫 점에서 멀리 떨어져 있으면 목표를 보내자마자 액션이 abort 됩니다. 원인은
nav2 MPPI의 `PathHandler`입니다.

1. 경로 앞머리에서 `max_robot_pose_search_dist`(기본값 = local costmap 반지름 = 10 m)까지만
   뒤져서 차량과 가장 가까운 점을 찾고,
2. 그 점부터 local costmap(20 x 20 m) 밖으로 나갈 때까지를 잘라 참조 경로로 씁니다.
3. 잘린 결과가 0점이면 `InvalidPath("Resulting plan has 0 poses in it")`를 던지고
   `controller_server`가 목표를 abort 합니다.

즉 코스 중간에서 출발하면 1번에서 찾은 "가장 가까운 점"이 여전히 경로 앞머리(= costmap 밖)라
참조가 통째로 비어 버립니다.

그래서 `follow_path_client_node`가 목표를 보내기 **전에** 경로를 손봅니다
(`start_from_nearest: true`, 기본값).

- `map` -> `robot_base_frame` tf로 현재 위치를 읽어 가장 가까운 웨이포인트를 찾고, 그 앞의
  이미 지나간 점들을 버립니다.
- 그 점이 `lead_in_spacing_m`보다 멀면 현재 위치에서 그 점까지 직선 진입 경로를 이 간격으로
  깔아 줍니다. 덕분에 경로가 항상 차량 발밑에서 시작해 costmap 안에 들어옵니다.
  (진입 경로는 차량 헤딩과 회전 반경을 고려하지 않는 순수 직선입니다. 경로가 차량 **뒤쪽**에서
  시작하면 MPPI가 후진하거나 크게 돌아 붙어야 합니다.)
- `max_start_distance_m`을 0보다 크게 주면, 가장 가까운 점이 그보다 멀 때 전송을 거부합니다.
  엉뚱한 코스 CSV를 보냈을 때 차가 멋대로 달려가는 것을 막는 안전장치입니다.

로그로 확인할 수 있습니다.

```
Starting at waypoint #65, 3.07 m from the vehicle (65 passed pose(s) dropped, 7 lead-in pose(s) prepended); 1314 poses remain.
```

`No 'map' -> 'body_link' transform ...`가 뜨면 odometry가 아직 안 올라온 것입니다
(`odometry.launch.py` 확인). tf 없이 원본 CSV를 그대로 보내고 싶으면
`start_from_nearest:=false`로 끄세요.

**주의**: 잘라 낸 앞부분은 다시 붙지 않습니다. 순환 코스에서 중간부터 출발하면 CSV의 마지막
점에서 끝나므로 한 바퀴가 완성되지 않습니다. 한 바퀴가 필요하면 CSV를 출발점 기준으로 회전시켜
저장하세요.

서버 쪽 `max_robot_pose_search_dist`를 코스 전체 길이만큼 키워도 같은 증상이 사라지지만,
순환/8자 코스에서는 매 틱 "전 구간 중 가장 가까운 점"을 찾다가 한 바퀴 건너뛴 지점에 붙어
버릴 수 있어 권하지 않습니다.

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
나옵니다. `vx_max`만 올려도 순항 속도는 안 오르는 이유가 이것입니다(실측) -- cost_weight/gamma가
같이 안 올라가면 균형점이 vx_max보다 한참 아래에 머뭅니다. 현재값(cost_weight 36, gamma 0.010)은
7.2 m/s로 계산되어 `vx_max: 6.0`에 포화합니다 -- 직선에서 6.0 m/s, 곡선/장애물에서는 critic들이
자동 감속.

**주의**: `vx_max`를 올릴 때는 같이 움직여야 하는 값이 있습니다. `cmd_vel_to_ackermann.max_velocity`
(하드웨어 안전 클램프, `vx_max`보다 위여야 함), `wz_max`(최고속에서의 최소 회전 반경 제약보다
살짝 위), `prune_distance`(예측 구간 = time_steps * model_dt * vx_max 보다 길어야 함). 속도를
올릴수록 `obstacle_max_range`(고정 6 m, 언덕 대책)가 주는 장애물 반응 시간은 줄어드니
(현재 6.0 m/s에서 약 1.0초) 실차 정지거리로 재검증하세요.

**주의**: critic의 `offset_from_furthest`는 미터가 아니라 **경로 점 개수**입니다. nav2 기본값은
전역 플래너의 촘촘한 경로(약 0.05 m 간격)를 가정하는데 우리 CSV는 0.6 m 간격이라, 기본값 20은
MPPI가 보는 pruned path(약 7.8 m / 13점)보다 길어서 PathAlignCritic이 조용히 통째로 비활성화됩니다.

**알려진 제약 (언덕)**: 라이다가 수평 고정인데 `ekf_local`이 `two_d_mode: true`라 tf에 피치가
0으로 들어갑니다. 그래서 driving_course의 언덕(x 37~46, y -16.5~+25) 오목 구간에서 노면 자체가
장애물로 마킹됩니다. 임시로 `obstacle_max_range: 6.0`으로 마킹 거리를 잘라 두었습니다(노면 히트는
대략 7~8 m 앞에서 생깁니다). 근본 해결은 tf에 실제 피치를 싣거나, 언덕 구간에서 obstacle_layer를
끄는 지오펜스입니다.


## 미션 실행 (mission_manager)

`follow_path_client_node`가 코스 전체를 목표 하나로 보내는 반면, `mission_manager_node`는
`config/mission.yaml`에 적힌 스텝 큐를 순서대로 실행합니다.

핵심 아이디어는 **한 스텝 = FollowPath 목표 하나**입니다. 정지선·신호등·주차 지점이 곧
세그먼트의 끝이므로 "도착했는가?"를 따로 판정할 필요가 없습니다 -- nav2의 goal checker가
목표를 성공 처리하는 순간이 도착입니다. 마찬가지로 정지에도 별도의 정지 명령이 필요 없습니다.
목표를 보내지 않으면 `/cmd_vel`이 끊기고 `cmd_vel_to_ackermann`의 워치독(`input_timeout` 0.3초)이
차를 세웁니다.

장애물 회피는 스텝이 아닙니다. MPPI가 해당 `drive` 스텝 안에서 로컬 costmap을 보며 알아서 처리합니다.

### 실행

```bash
# 1) 시뮬레이터 + 오도메트리
ros2 launch hyper_launch sim.launch.py
ros2 launch hyper_launch odometry.launch.py
# 2) 액션 서버
ros2 launch hyper_planner nav2_controller.launch.py
# 3) 신호등 인식 (/perception/sign) -- wait_signal 스텝이 이걸 봅니다
ros2 launch hyper_object_detection perception.launch.py
# 4) 미션 매니저 (기본은 대기 상태)
ros2 launch hyper_planner mission.launch.py
ros2 service call /mission_manager/start std_srvs/srv/Trigger
```

| 서비스 | 하는 일 |
| --- | --- |
| `/mission_manager/start` | 현재 스텝부터 미션 시작/재개 |
| `/mission_manager/cancel` | 진행 중인 목표를 취소하고 그 스텝에서 대기 |
| `/mission_manager/skip` | 현재 스텝을 포기하고 다음으로 |
| `/mission_manager/restart` | 스텝 0으로 되돌림 (`start`로 다시 시작) |

상태는 `/mission_manager/status`(latched `std_msgs/String`), 현재 세그먼트 경로는
`/mission_manager/path`(latched `nav_msgs/Path`)로 나갑니다.

### 라벨과 세그먼트

`mission.yaml`은 "어디서"(`labels`)와 "무엇을"(`steps`)로 나뉩니다. 라벨은 **웨이포인트 인덱스가
아니라 좌표**로 저장되므로 코스를 다시 녹화해도 물리적 위치가 같으면 살아남습니다. 노드는 로드
시점에 각 라벨을 CSV의 최근접 웨이포인트로 스냅하고, 거리가 `label_snap_tolerance_m`를 넘으면
미션을 **거부**합니다(라벨이 엉뚱한 데 붙은 채로 주행이 시작되는 사고 방지).

라벨은 코스를 따라 단조 증가해야 합니다. `until` 라벨이 직전 스텝보다 앞선 인덱스에 스냅되면
세그먼트가 비거나 뒤집히므로 로드가 실패합니다.

라벨 찍기는 `hyper_waypoint/scripts/label_waypoints.py`(코스 이미지 위에 웨이포인트를 겹쳐
띄우고 클릭으로 라벨을 배치 → `mission.yaml`의 `labels:` 블록만 다시 씁니다)를 쓰세요.

### 후진 세그먼트 (`reverse: true`)

주차는 후진으로 녹화한 구간을 되짚어 갑니다. 이 구간은 MPPI가 아니라 RPP를 씁니다
(`controller: ReverseFollowPath`). MPPI는 `vx_min`으로 후진을 허용할 뿐이고 `PreferForwardCritic`이
후진에 벌점을 주므로 주차칸까지 밀어 넣는 기동을 안정적으로 못 냅니다.

`reverse: true`가 실제로 하는 일은 두 가지뿐입니다.

1. 헤딩을 무시하는 직선 진입 경로(lead-in)를 끕니다. 후진 세그먼트에 붙이면 방향이 뒤집힙니다.
2. CSV에 `yaw`가 없을 때만, 이웃 점에서 유도한 진행 방향을 180도 뒤집어 차체 헤딩으로 만듭니다.

포즈 방향 자체는 **뒤집지 않습니다.** CSV의 `yaw`는 EKF가 준 실제 차체 헤딩이고, RPP는 진행
방향을 포즈 방향이 아니라 carrot 점의 차체 좌표계 x부호로 판단합니다. goal checker의 yaw 비교도
차체 헤딩 기준이라 뒤집으면 오히려 틀립니다. 노드는 로드 시점에 녹화된 방향과 `reverse` 플래그가
맞는지 검사해 어긋나면 경고합니다.

RPP 제약 두 가지를 기억하세요.

- `allow_reversing`과 `use_rotate_to_heading`은 동시에 true일 수 없습니다. 둘 다 켜면 nav2가
  configure에서 reversing을 꺼 버립니다.
- RPP는 **한 경로 안의 방향 전환**을 처리하지 못합니다. 전진↔후진 경계는 반드시 `mission.yaml`에서
  세그먼트를 쪼개야 합니다(그래서 주차 지점이 라벨이 됩니다).

### goal checker

정지선·신호등·주차처럼 정확히 서야 하는 스텝은 `goal_checker: precise_goal_checker`
(`xy_goal_tolerance` 0.25 m)를 씁니다. 나머지는 `general_goal_checker`(0.5 m)입니다.

양쪽 다 `yaw_goal_tolerance`는 열어 두었습니다(3.15). 아커만 차량은 제자리 회전이 불가능하므로
헤딩이 틀어진 채 도착하면 컨트롤러가 고칠 방법이 없고, 목표가 영원히 만족되지 않습니다.

공차를 더 조이면 마지막 몇 십 cm에서 기어가다가 `progress_checker`(0.5 m / 15초)에 걸려 abort
하기 쉽습니다. 그 경우를 위해 `arrival_slack_m`(기본 0.6 m)이 있습니다 -- abort가 났어도 차가
목표 이 거리 안에 있으면 도착으로 처리하고 다음 스텝으로 넘어갑니다. 같은 목표를 무한히
재전송하며 미션이 죽는 것을 막습니다.

정지선은 어차피 "넘지 않는 것"이 중요하므로, 공차를 조이기보다 라벨을 선보다 조금 앞에 찍으세요.
