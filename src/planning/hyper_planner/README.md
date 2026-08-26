# hyper_planner

HYPER의 행동 결정과 차량 제어를 담당하는 C++ 패키지입니다. 코스 위의 이벤트(정지선·신호등·주차)를
스텝 큐로 실행하고, nav2 `follow_path` 액션을 통해 조향과 속도 명령을 생성합니다.

## 구성

- `mission_manager_node`: `config/<mission>.yaml`의 스텝 큐(주행/정지/신호 대기)를 순서대로 실행합니다.
  대회 주행도, 한 바퀴 시험 주행도 전부 이 노드입니다.
- `cmd_vel_to_ackermann_node`: nav2가 내는 `/cmd_vel`(Twist)을 `/velocity`, `/steering_angle`로 변환합니다.
  `input_timeout`(0.3초) 워치독이 있어 목표가 없으면 차가 섭니다 -- 이것이 `stop` 스텝의 정지 방식입니다.
- `follow_path_client_node`: 코스 전체를 목표 하나로 보내던 예전 노드입니다. **레거시** -- 아래
  [follow_path_client_node (레거시)](#follow_path_client_node-레거시) 참고.
- `config/mission.yaml`: 대회 미션. 시퀀스(`steps`)와 코스 위 이벤트 지점(`labels`) 정의입니다.
- `config/simple.yaml`: 코스 한 바퀴. 골 하나짜리 미션이고, 정지도 신호도 주차도 없습니다.
- `config/nav2_controller.yaml`: nav2 `controller_server`(= `follow_path` 액션 서버) 파라미터입니다.
- `src/mission_manager_parameters.yaml`: `mission_manager_node`의 파라미터 정의
  (generate_parameter_library가 여기서 헤더를 생성합니다). 파라미터의 의미는 이 파일이 원본입니다.

## 빌드

```bash
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup   # 최초 1회
cd ~/HYPER
colcon build --packages-select hyper_planner
source install/setup.bash
```

터미널마다 `source ~/HYPER/install/setup.bash`를 먼저 실행하세요.

## 실행

경로: 웨이포인트 CSV -> `nav_msgs/Path` -> nav2 `follow_path` 액션 -> `/cmd_vel` -> `/velocity`, `/steering_angle`.

가장 짧은 길은 전체 스택을 한 번에 띄우는 것입니다. `mission:=`으로 어떤 미션을 실을지 고릅니다
(`hyper_planner/config/<이름>.yaml`로 풀립니다).

```bash
ros2 launch hyper_launch simulation.launch.py                 # config/mission.yaml (대회 미션)
ros2 launch hyper_launch simulation.launch.py mission:=simple # config/simple.yaml (한 바퀴)
ros2 service call /mission_manager/start std_srvs/srv/Trigger
```

`auto_start` 기본값이 `false`라 노드는 뜨자마자 달리지 않고 `~/start`를 기다립니다. 실차에서 이게
안전합니다. 같은 `mission:=` 인자가 `hyper_launch real.launch.py`와 `behavior.launch.py`에도 있습니다.

단계별로 띄우려면:

```bash
# 1) 시뮬레이터 + 오도메트리 (TF: map -> odom -> body_link, /scan 필요)
ros2 launch hyper_launch sim.launch.py
ros2 launch hyper_launch odometry.launch.py
# 2) 액션 서버 (controller_server + lifecycle_manager + cmd_vel 변환)
ros2 launch hyper_planner nav2_controller.launch.py      # 실차는 use_sim_time:=false
# 3) 신호등 인식 (/perception/sign) -- wait_signal 스텝이 이걸 봅니다
ros2 launch hyper_object_detection perception.launch.py
# 4) 미션 매니저
ros2 launch hyper_planner mission.launch.py mission:=simple
ros2 service call /mission_manager/start std_srvs/srv/Trigger
```

`mission_yaml:=`에 절대 경로를 주면 `mission:=`을 덮어씁니다(패키지 밖의 미션 파일을 쓸 때).
코스 CSV는 `waypoint_csv:=`이고 기본값은 `hyper_waypoint/waypoints/sim.csv`입니다.

`controller_with_parking_node`도 `/velocity`, `/steering_angle`에 퍼블리시하므로
`parking_system_cpp.launch.py`와 동시에 실행하지 마세요.

### 서비스와 토픽

| 서비스 | 하는 일 |
| --- | --- |
| `/mission_manager/start` | 현재 스텝부터 미션 시작/재개 |
| `/mission_manager/cancel` | 진행 중인 목표를 취소하고 그 스텝에서 대기 |
| `/mission_manager/skip` | 현재 스텝을 포기하고 다음으로 |
| `/mission_manager/restart` | 스텝 0으로 되돌림 (`start`로 다시 시작) |

상태는 `/mission_manager/status`(latched `std_msgs/String`), 현재 세그먼트 경로는
`/mission_manager/path`(latched `nav_msgs/Path`)로 나갑니다. decel 프로파일이 켜진 스텝에서는
`/speed_limit`(`nav2_msgs/SpeedLimit`)도 나갑니다.

## 미션 파일 (mission.yaml / simple.yaml)

핵심 아이디어는 **한 `drive` 스텝 = FollowPath 목표 하나**입니다. 정지선·신호등·주차 지점이 곧
세그먼트의 끝이므로 "도착했는가?"를 따로 판정할 필요가 없습니다 -- nav2의 goal checker가 목표를
성공 처리하는 순간이 도착입니다. 정지에도 별도의 정지 명령이 없습니다. 목표를 보내지 않으면
`/cmd_vel`이 끊기고 `cmd_vel_to_ackermann`의 워치독이 차를 세웁니다.

장애물 회피는 스텝이 아닙니다. MPPI가 해당 `drive` 스텝 안에서 로컬 costmap을 보며 알아서 처리합니다.

파일은 "어디서"(`labels`)와 "무엇을"(`steps`)로 나뉩니다. 각 필드의 의미와 튜닝 지침은
[config/mission.yaml](config/mission.yaml)의 주석이 원본입니다 -- 여기서는 구조만 설명합니다.

### 라벨과 세그먼트

라벨은 **웨이포인트 인덱스가 아니라 좌표**로 저장되므로 코스를 다시 녹화해도 물리적 위치가 같으면
살아남습니다. 노드는 로드 시점에 각 라벨을 CSV의 최근접 웨이포인트로 스냅하고, 거리가
`label_snap_tolerance_m`를 넘으면 미션을 **거부**합니다(라벨이 엉뚱한 데 붙은 채로 주행이 시작되는
사고 방지).

라벨은 코스를 따라 단조 증가해야 합니다. `until` 라벨이 직전 스텝보다 앞선 인덱스에 스냅되면
세그먼트가 비거나 뒤집히므로 로드가 실패합니다. 첫 `drive` 스텝만 CSV의 처음(#0)부터 시작하고,
이후 세그먼트는 직전 스텝의 도착점에서 이어집니다.

라벨 찍기는 `hyper_waypoint/scripts/label_waypoints.py`(코스 이미지 위에 웨이포인트를 겹쳐 띄우고
클릭으로 배치 -> `labels:` 블록만 다시 씁니다)를 쓰세요.

### drive 스텝: 감속과 도착 판정

기본 동작은 단순합니다 -- 세그먼트 끝까지 경로를 보내고 goal checker를 기다립니다. `simple.yaml`은
여기서 끝입니다. 정지선처럼 **정확히, 그리고 빨리** 서야 하는 스텝만 아래 두 옵션을 켭니다.

**`cancel_on_arrival_m`** -- 골까지 남은 거리가 이 값 이하이고 속도도 `cancel_on_arrival_speed`
(기본 0.5 m/s) 아래로 떨어졌으면, goal checker를 기다리지 않고 취소해 도착으로 칩니다. MPPI는 공차
안으로 들어가는 마지막 수십 cm를 기어가는데, 어차피 뒤에 `stop`/`wait_signal`이 붙는 스텝이라면
그 시간을 버릴 이유가 없습니다. 속도 조건이 있는 이유는 빠를 때 취소하면 0 속도가 실제로 나가기까지의
지연 동안 그만큼 굴러가기 때문입니다. **주차 스텝에는 켜지 마세요** -- 0.4 m 짧게 서면 칸에 덜
들어갑니다.

**`decel_profile_a`** -- 이 스텝을 등감속(m/s^2)으로 세웁니다. MPPI에는 가속도 제약이 없고, 감속이
"궤적의 마지막 점을 경로 끝점에 맞춘다"는 항에서 부수적으로 나옵니다. 그래서 속도가
`v = 남은거리 / (time_steps * model_dt)`라는 지수 감쇠가 되어 수학적으로 영영 도착하지 않고, local
costmap 반지름(10 m)부터 기어가기 시작해 정지까지 5초 넘게 씁니다. 이 값을 켜면 매니저가 두 가지를
합니다.

1. 골 경로를 라벨보다 `decel_profile_lookahead_m`(기본 12 m)만큼 뒤까지 늘립니다. MPPI가 보는 경로
   끝이 costmap 밖에 있으면 위의 감속 항이 아예 켜지지 않습니다.
2. 대신 `/speed_limit`으로 `v = sqrt(2*a*d)`를 실어 보내 `vx_max`를 직접 깎습니다.

늘어난 꼬리는 **녹화 코스를 이어 붙인 것이 아니라 라벨의 진행 방향으로 뻗은 직선**입니다
(`path_loader.hpp`의 `append_straight_tail`, 방향은 마지막 2 m의 평균 진행 방향). 이유는 세 가지입니다.
라벨 뒤가 후진 구간(주차 진입)이면 코스 꼬리는 방향이 180도 꺾인 채 되돌아와 한 목표 안에 방향
전환이 생기고, 코스 끝(`finish`)에서는 이어 붙일 코스가 모자라며, 직선이면 꼬리 길이가 항상 정확히
`decel_profile_lookahead_m`이라 보낸 경로 위에서 라벨을 "끝에서 남은 길이"로 되찾는 계산이
정확해집니다. 꼬리는 실제로 주행되지 않습니다 -- 차는 항상 그 앞의 라벨에서 취소로 섭니다.

그래서 **`decel_profile_a`는 반드시 `cancel_on_arrival_m`과 같이 씁니다.** 경로 끝이 라벨보다 뒤에
있으므로 goal checker는 라벨에서 절대 만족되지 않고, 도착 판정이 오직 취소로만 일어납니다. 취소가
안 걸리는 경우를 위한 하드 백스톱이 노드 안에 둘 있습니다.

- 정지점을 지났으면(`distance_to_stop_m` <= 0) 속도와 무관하게 취소 -- "정지선을 넘지 않는다"의
  마지막 보루입니다. WARN이 납니다.
- 프로파일 주행 중 tf가 `progress_stale_cancel_sec`(기본 1초) 동안 끊기면 취소 -- 여기서 손을 놓으면
  goal checker가 받쳐 주지 않으므로 차가 그냥 지나갑니다. ERROR가 납니다.

후진 세그먼트에서는 `decel_profile_a`가 로드 시점에 자동으로 꺼집니다(RPP는 goal checker에 도달해야
합니다). `a` 값은 2.0이면 6 m/s에서 제동거리 9 m입니다. "reached ... above cancel_on_arrival_speed"
경고가 나면 차가 프로파일만큼 못 줄이고 있다는 뜻이니 낮추세요.

### wait_signal 스텝과 prearm

`/perception/sign`이 `value`를 `debounce_frames` 연속으로 낼 때까지 대기합니다. `value`에 쉼표를
넣어 여러 신호를 허용할 수 있습니다(예: `"green,left_arrow"` -- 좌회전 신호등인 `light_3`가 이렇게
씁니다). `timeout_s` 안에 못 보면 기본값(`proceed_on_signal_timeout: true`)으로는 경고를 남기고 그냥
출발합니다 -- 대회에서 영영 멈춰 서는 것보다 낫습니다.

**`prearm_distance_m`** -- 0보다 크면 앞 `drive` 스텝의 골까지 이만큼 남았을 때부터 신호를 미리
봅니다. 통과 신호가 확인되면 서지 않고 그대로 통과합니다: 실행 중인 골을 "지금 위치 -> 다음 `drive`
스텝의 끝"까지의 골로 갈아끼우고(nav2가 `/cmd_vel`을 끊지 않고 갈아끼웁니다) 이 `wait_signal`을
건너뜁니다. 초록불에 굳이 정차했다 재출발하는 시간을 없애는 것이 목적입니다.

확인이 안 되면 -- 빨간불이든, 인식이 끊겼든, 애매하든 -- 아무 일도 일어나지 않고 원래 골 그대로
정지선에 섭니다. 즉 기본 동작이 "정지"이고 "통과"가 명시적 예외라, 인식이 흔들려도 안전한 쪽으로
실패합니다. 패턴이 `[drive -> wait_signal -> drive]`가 아니면 로드 시점에 경고를 남기고 그 자리의
prearm만 끕니다.

**주의**: 갈아끼우기는 되돌릴 수 없습니다. 통과 판정이 나면 그 자리에서 신호를 더 이상 보지 않으므로,
통과한 뒤 초록 -> 빨강으로 바뀌어도 그대로 지나갑니다. 15 m / 6 m/s면 약 2.5초의 노출입니다.

### 후진 세그먼트 (`reverse: true`)

주차는 후진으로 녹화한 구간을 되짚어 갑니다. 이 구간은 MPPI가 아니라 RPP를 씁니다
(`controller: ReverseFollowPath`). MPPI는 `vx_min`으로 후진을 허용할 뿐이고 `PreferForwardCritic`이
후진에 벌점을 주므로 주차칸까지 밀어 넣는 기동을 안정적으로 못 냅니다.

`reverse: true`가 실제로 하는 일은 세 가지뿐입니다.

1. 헤딩을 무시하는 직선 진입 경로(lead-in)를 끕니다. 후진 세그먼트에 붙이면 방향이 뒤집힙니다.
2. CSV에 `yaw`가 없을 때만, 이웃 점에서 유도한 진행 방향을 180도 뒤집어 차체 헤딩으로 만듭니다.
3. `decel_profile_a`를 끕니다(위 참고).

포즈 방향 자체는 **뒤집지 않습니다.** CSV의 `yaw`는 EKF가 준 실제 차체 헤딩이고, RPP는 진행 방향을
포즈 방향이 아니라 carrot 점의 차체 좌표계 x부호로 판단합니다. goal checker의 yaw 비교도 차체 헤딩
기준이라 뒤집으면 오히려 틀립니다. 노드는 로드 시점에 녹화된 방향과 `reverse` 플래그가 맞는지 검사해
어긋나면 경고합니다.

RPP 제약 두 가지를 기억하세요.

- `allow_reversing`과 `use_rotate_to_heading`은 동시에 true일 수 없습니다. 둘 다 켜면 nav2가
  configure에서 reversing을 꺼 버립니다.
- RPP는 **한 경로 안의 방향 전환**을 처리하지 못합니다. 전진<->후진 경계는 반드시 미션 파일에서
  세그먼트를 쪼개야 합니다(그래서 주차 지점이 라벨이 됩니다).

### goal checker

정지선·신호등·주차처럼 정확히 서야 하는 스텝은 `goal_checker: precise_goal_checker`
(`xy_goal_tolerance` 0.25 m)를 씁니다. 나머지는 `general_goal_checker`(0.5 m)입니다.

양쪽 다 `yaw_goal_tolerance`는 열어 두었습니다(3.15). 아커만 차량은 제자리 회전이 불가능하므로
헤딩이 틀어진 채 도착하면 컨트롤러가 고칠 방법이 없고, 목표가 영원히 만족되지 않습니다.

공차를 더 조이면 마지막 몇 십 cm에서 기어가다가 `progress_checker`(0.5 m / 15초)에 걸려 abort 하기
쉽습니다. 그 경우를 위해 `arrival_slack_m`(기본 0.6 m)이 있습니다 -- abort가 났어도 차가 목표 이
거리 안에 있으면 도착으로 처리하고 다음 스텝으로 넘어갑니다. 그 밖의 abort는 `goal_retry_limit`
(기본 2회)만큼 같은 목표를 다시 보냅니다.

정지선은 어차피 "넘지 않는 것"이 중요하므로, 공차를 조이기보다 라벨을 선보다 조금 앞에 찍으세요.

## 목표를 보내기 전의 경로 처리

CSV 세그먼트를 그대로 보내면 안 되는 이유가 있습니다. 차량이 경로의 첫 점에서 멀리 떨어져 있으면
목표를 보내자마자 액션이 abort 됩니다. 원인은 nav2 MPPI의 `PathHandler`입니다.

1. 경로 앞머리에서 `max_robot_pose_search_dist`(기본값 = local costmap 반지름 = 10 m)까지만 뒤져서
   차량과 가장 가까운 점을 찾고,
2. 그 점부터 local costmap(20 x 20 m) 밖으로 나갈 때까지를 잘라 참조 경로로 씁니다.
3. 잘린 결과가 0점이면 `InvalidPath("Resulting plan has 0 poses in it")`를 던지고
   `controller_server`가 목표를 abort 합니다.

즉 코스 중간에서 출발하면 1번에서 찾은 "가장 가까운 점"이 여전히 경로 앞머리(= costmap 밖)라 참조가
통째로 비어 버립니다. 그래서 `mission_manager_node`는 목표를 보내기 **전에** 매 세그먼트를 손봅니다
(끌 수 없습니다 -- 세그먼트 주행에서는 항상 필요합니다).

- `map` -> `robot_base_frame` tf로 현재 위치를 읽어 세그먼트에서 가장 가까운 점을 찾고, 그 앞의 이미
  지나간 점들을 버립니다.
- 그 점이 `lead_in_spacing_m`(기본 0.5 m)보다 멀면 현재 위치에서 그 점까지 직선 진입 경로를 이
  간격으로 깔아 줍니다. 덕분에 경로가 항상 차량 발밑에서 시작해 costmap 안에 들어옵니다. (진입
  경로는 차량 헤딩과 회전 반경을 고려하지 않는 순수 직선입니다. 경로가 차량 **뒤쪽**에서 시작하면
  MPPI가 후진하거나 크게 돌아 붙어야 합니다.) 후진 세그먼트에는 붙이지 않습니다.
- `path_resample_spacing_m`(기본 0.25 m)으로 경로를 다시 깝니다. 녹화 CSV의 약 0.59 m 간격은 MPPI
  `PathAlignCritic`이 "가장 가까운 경로 점까지의 거리"를 벌점으로 쓰기 때문에 ±0.3 m의 코너 컷
  사각지대를 만듭니다. **이 값을 바꾸면 `nav2_controller.yaml`에서 점 개수로 세는 파라미터
  (`offset_from_furthest`)도 같이 조정해야 합니다.**
- `max_start_distance_m`을 0보다 크게 주면, 가장 가까운 점이 그보다 멀 때 전송을 거부합니다. 엉뚱한
  코스 CSV를 보냈을 때 차가 멋대로 달려가는 것을 막는 안전장치입니다(기본 0 = 끔).

`No 'map' -> 'body_link' transform ...`가 뜨면 odometry가 아직 안 올라온 것입니다
(`odometry.launch.py` 확인).

CSV는 헤더 이름으로 파싱하므로 `idx,x,y,yaw,frame_id` 형태와 레코더의 전체 열 형태 모두 됩니다.
`yaw`가 비어 있으면 다음 점을 향하는 방향으로 자동 계산합니다. `min_spacing_m`으로 로드 시점에 너무
촘촘한 점을 솎아낼 수 있습니다(기본 0 = 끔).

## 컨트롤러: MPPI

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
살짝 위), `prune_distance`(예측 구간 = time_steps * model_dt * vx_max 보다 길어야 함),
그리고 미션 매니저의 `controller_vx_max`(기본 6.0 -- `/speed_limit`을 `vx_max` 기준 비율로
환산하므로 어긋나면 감속이 틀립니다). 속도를 올릴수록 `obstacle_max_range`(고정 6 m, 언덕 대책)가
주는 장애물 반응 시간은 줄어드니(현재 6.0 m/s에서 약 1.0초) 실차 정지거리로 재검증하세요.

**주의**: critic의 `offset_from_furthest`는 미터가 아니라 **경로 점 개수**입니다. nav2 기본값은
전역 플래너의 촘촘한 경로(약 0.05 m 간격)를 가정합니다. 우리가 보내는 경로 간격
(`path_resample_spacing_m`)과 같이 봐야 합니다.

**알려진 제약 (언덕)**: 라이다가 수평 고정인데 `ekf_local`이 `two_d_mode: true`라 tf에 피치가
0으로 들어갑니다. 그래서 driving_course의 언덕(x 37~46, y -16.5~+25) 오목 구간에서 노면 자체가
장애물로 마킹됩니다. 임시로 `obstacle_max_range: 6.0`으로 마킹 거리를 잘라 두었습니다(노면 히트는
대략 7~8 m 앞에서 생깁니다). 근본 해결은 tf에 실제 피치를 싣거나, 언덕 구간에서 obstacle_layer를
끄는 지오펜스입니다.

## RViz 시각화

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
| BEV Front (차선 IPM 디버그 뷰) | `/lane/bev/image_raw` | Image 패널 |
| BEV Rear (차선 IPM 디버그 뷰, 시뮬레이션 전용) | `/lane/rear_bev/image_raw` | Image 패널 |
| BEV 지면 오버레이 (전방) | `/lane/bev/points` | 원본 색 (RGB8) |
| 후방 깊이 포인트 클라우드 (시뮬레이션 전용) | `/camera_rear/depth/points` | 원본 색 (RGB8) |

BEV 두 패널은 `hyper_lane_detection`의 버드아이뷰 디버그 화면입니다. 그 노드는 자체 OpenCV 창을
열지 않으므로, 경로/코스트맵과 차선 인지 결과를 한 화면에서 같이 보려면 이 패널을 씁니다. QoS는
best-effort로 맞춰 두었습니다(발행 측이 SensorDataQoS라 Reliable로 두면 아무것도 안 나옵니다).
후방 패널은 `input_backend:=ros_raw`(시뮬레이션)에서만 채워지고, 실차에서는 `No Image`로 남습니다.
패널 크기·위치는 드래그로 바꾼 뒤 `File > Save Config`로 저장하면 됩니다 — 다만 RViz가 파일을
다시 쓰면서 이 설정 파일의 주석은 지워집니다.

**BEV 지면 오버레이 (전방)**는 같은 화면을 2D 패널이 아니라 3D 씬의 지면(`body_link`, z=0.02 m)에
깔아 코스트맵·경로·footprint와 직접 겹쳐 봅니다. `body_link` tf가 있어야 보입니다. 오버레이가
코스트맵과 어긋나 보이면 `hyper_lane_detection`의 지면 투영 파라미터(`bev.*` — 특히 장착 높이·피치,
`ground_projection.hpp` 참고)가 실제 카메라와 어긋난 것이고, 같은 투영이 `/lane/center`의
`offset_m`과 `/stopline/detection`의 `distance_m`도 만들어냅니다. 노드 시작 로그에 카메라별로
실제 만들어진 지면 범위·스케일·원점이 한 줄 찍히므로 먼저 그 줄과 비교해 보십시오.

**후방 깊이 포인트 클라우드**는 오버레이가 아니라 실제로 측정된 3D 점입니다. 후방 RGBD 센서가
내보내는 `/camera_rear/depth/points`를 그대로 그리므로, 지면을 평면으로 가정하는 IPM 오버레이와
달리 연석·주차 차량·라바콘처럼 지면 위로 솟은 것도 제 높이에 찍힙니다. 좌표는 `rear_camera_link`
프레임이고 그 프레임 규약(x 후방 = 카메라 정면)을 그대로 따르므로 별도 변환 노드가 필요 없습니다
(자세한 이유는 `vehicle.xacro`의 `<gz_frame_id>` 주석 참고). 먼 쪽 끝은 센서의 depth clip(10 m)이
자르고, 가까운 쪽은 장착 높이·하향각(15°)과 상하 화각이 겹쳐 차 바로 뒤 약 2 m가 비어 있습니다 --
후진 주차에서 차 뒤 1~2 m를 봐야 한다면 `vehicle.xacro`의 `rear_camera_pitch`를 키워야 합니다.
`hyper_lane_detection`이 후방 IPM으로 만드는 `/lane/rear_bev/points`는 여전히 발행되지만, 이 뷰에는
더 이상 올리지 않습니다 -- 차선 IPM 결과를 보고 싶으면 `BEV Rear` Image 패널을 쓰세요.

진초록 경로는 레거시 `follow_path_client_node`를 띄웠을 때만 나옵니다. 미션 주행에서 지금 무엇이
나가고 있는지는 초록(`/mission_manager/path`)을 보세요 -- decel 프로파일이 켜진 스텝에서는 이 경로가
정지선보다 12 m 더 뻗어 있는 것이 정상입니다.

`/received_global_plan`과 `/lookahead_point`은 RPP 전용 토픽입니다. 기본 주행 컨트롤러가
MPPI로 바뀐 뒤로는 `ReverseFollowPath`(후진 주차) 세그먼트가 돌 때만 나옵니다. 평상시
주행 중에 컨트롤러가 무엇을 보고 있는지는 `/transformed_global_plan`으로 확인하세요.

MPPI 쪽 두 토픽은 `nav2_controller.yaml`의 `FollowPath.visualize: true`일 때만 나갑니다.
경로가 안 보이면 그 값과 TF(`map` -> `odom` -> `body_link`)를 먼저 확인하세요.

## 상태 확인

```bash
ros2 action list | grep follow_path          # /follow_path 가 떠 있는지
ros2 topic echo /mission_manager/status      # 어느 스텝에서 무엇을 하고 있는지
ros2 topic echo /cmd_vel                     # nav2 출력
ros2 topic echo /velocity                    # 변환된 차량 명령
ros2 topic echo /speed_limit                 # decel 프로파일이 내려보내는 상한
ros2 lifecycle get /controller_server        # active 여야 함
```

## follow_path_client_node (레거시)

코스 전체를 목표 하나로 보내 한 바퀴 도는 노드입니다. 지금은 `mission:=simple`이 같은 일을 하고,
그쪽은 실주행과 **동일한** 경로 처리(`path_loader.hpp`)를 거치므로 컨트롤러 튜닝 결과가 대회 주행에
그대로 옮겨집니다. 이 노드는 CSV 로딩과 경로 다듬기를 자기 사본으로 들고 있어 리샘플링이 없고,
abort 복구나 off-path 백스톱도 없습니다.

```bash
ros2 launch hyper_planner follow_path_client.launch.py \
  waypoint_csv:=$HOME/HYPER/src/planning/hyper_waypoint/waypoints/sim.csv
ros2 service call /follow_path_client/start std_srvs/srv/Trigger    # CSV 다시 읽어 재전송
ros2 service call /follow_path_client/cancel std_srvs/srv/Trigger   # 진행 중인 목표 취소
```

`mission_manager_node`에 없는 것은 `shutdown_on_finish`(끝나면 프로세스 종료), `auto_start` 기본
true, `start_from_nearest:=false`(tf를 무시하고 CSV 첫 점부터 그대로 보내기) 셋입니다. 이 셋이
필요 없어지면 노드를 지워도 됩니다.
