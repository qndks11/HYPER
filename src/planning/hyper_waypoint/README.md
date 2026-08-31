# hyper_waypoint

`odometry/filtered_map`을 구독해서 `idx, x, y, yaw, frame_id`를 CSV로 기록하는 웨이포인트 레코더 패키지입니다.

## 실행 (권장) — GUI로 조작

실차 수동 주행 launch(`joystick_control_real.launch.py`, `phone_control_real.launch.py`)가
레코더와 조작판 GUI를 이미 포함합니다. 따로 띄우고 싶을 때만:

```bash
ros2 launch hyper_waypoint record.launch.py \
  waypoint_csv:=$HOME/HYPER/src/planning/hyper_waypoint/waypoints/real.csv
```

레코더가 `auto_start:=false`로 떠서 **GUI의 `● Record`를 누를 때까지 기다립니다.**
기록 시작이 곧 CSV truncate이므로, 스택을 띄우는 것만으로 지난 녹화본이 날아가지 않습니다.
`■ Stop`을 누르면 파일이 닫히고, 다시 `Record`를 누르면 idx 0부터 새로 녹화합니다.

GUI가 보여주는 것:

| 항목 | 의미 |
| --- | --- |
| `● REC` / `IDLE` | 기록 중인지 |
| 기록된 점 / 누적 거리 | 지금까지 몇 점, 몇 m |
| 다음 점까지 | 마지막 기록 지점에서 얼마나 왔는지 (`min_spacing_m`까지 남은 거리) |
| 현재 위치 (map) | `/odometry/filtered_map`의 x, y. 값이 없으면 EKF가 안 도는 것 |
| 속도 | `/odom`의 바퀴 속도 |
| GPS 상태 | `/gps/fix`의 status. **`RTK / GBAS`(초록)여야 쓸 만한 녹화**입니다 |
| EKF 공분산 xx / yy | 융합 위치의 불확실도 |
| 미니맵 | 지금까지 찍힌 점(파랑)과 현재 위치(보라)를 위에서 본 그림 |

같은 내용이 토픽으로도 나가므로 RViz에서도 볼 수 있습니다:

- `/waypoint_recorder/status` (`std_msgs/String`) — `key=value` 한 줄, 5Hz + 점이 찍힐 때마다
- `/waypoint_recorder/path` (`nav_msgs/Path`) — 지금까지 찍힌 점 전부

둘 다 `transient_local`이라 GUI나 RViz를 나중에 띄워도 현재 상태를 그대로 받습니다.

서비스로 직접 조작할 수도 있습니다:

```bash
ros2 service call /waypoint_recorder/start std_srvs/srv/Trigger
ros2 service call /waypoint_recorder/stop  std_srvs/srv/Trigger
```

## 실행 — 노드만 단독으로

```bash
colcon build --packages-select hyper_waypoint
source install/setup.bash
ros2 run hyper_waypoint waypoint_recorder_node --ros-args -p output_csv:=$HOME/HYPER/src/planning/hyper_waypoint/waypoints/real.csv -p min_spacing_m:=0.5
```

- `auto_start` 파라미터의 기본값이 `true`라 이렇게 띄우면 **즉시 기록을 시작**합니다(기존 사용법 그대로). `Ctrl-C`로 원하는 시점에 종료하세요.
- `output_csv` 파라미터를 생략하면 노드를 실행한 위치에 `waypoint_record.csv`로 저장됩니다.
- `min_spacing_m` 파라미터(기본값 `0.5`)는 직전 기록 지점으로부터 이 거리(m) 이상 이동했을 때만 새 줄을 기록합니다. 시간 간격이 아니라 이동 거리 기준으로 웨이포인트가 샘플링됩니다.
- 파일은 `idx,stamp_sec,x,y,yaw,frame_id,...` 헤더로 시작하며, 매 기록마다 flush되므로 중간에 종료해도 그때까지 기록된 내용은 남아 있습니다.

## 스크립트

`scripts/`의 파이썬 스크립트들은 `ros2 run` 대상이 아니라 `python3`로 직접 실행합니다.

### plot_waypoints.py — 기록 품질 확인

```bash
python3 scripts/plot_waypoints.py waypoints/sim.csv --jump-threshold 1.0
```

융합 경로 / 생 GPS / navsat_transform 출력을 겹쳐 그려서 위치 점프의 원인이 GPS 센서인지,
좌표 변환인지, EKF 융합인지 구분합니다.

### label_waypoints.py — 미션 이벤트 지점 라벨링

기록된 코스 위에 정지선·신호등·주차 구획 위치를 클릭으로 찍어
[hyper_planner/config/mission.yaml](../../planning/hyper_planner/config/mission.yaml)의
`labels:` 블록에 기록합니다.

```bash
# 시뮬레이션 코스 텍스처를 배경에 깔고 라벨링 (권장)
python3 scripts/label_waypoints.py waypoints/sim.csv --gazebo-course

python3 scripts/label_waypoints.py waypoints/sim.csv
python3 scripts/label_waypoints.py waypoints/track.csv --mission /path/to/mission.yaml
python3 scripts/label_waypoints.py waypoints/track.csv --background ortho.png --extent -50 -60 60 50
```

| 조작 | 동작 |
| --- | --- |
| 좌클릭 | 현재 선택된 라벨을 최근접 웨이포인트에 배치 (배치 후 다음 미배치 라벨로 자동 이동) |
| 우클릭 | 클릭 지점에서 가장 가까운 라벨 삭제 |
| `1`~`9` | 현재 페이지 내 라벨 선택 (`[` / `]` 페이지 이동) |
| `n` / `p` | 이전/다음 라벨 |
| `Tab` | 다음 미배치 라벨 |
| `u` | 되돌리기 |
| `s` | mission.yaml 저장 |
| `q` | 종료 |

- **찍어야 할 라벨 목록은 하드코딩돼 있지 않습니다.** mission.yaml의 `drive` 스텝이
  `until:`로 참조하는 이름이 곧 라벨 목록이므로, 스텝을 추가하면 이 툴이 자동으로 그
  위치를 요구합니다.
- 라벨은 웨이포인트 idx가 아니라 **map 프레임 좌표로 저장**됩니다. 코스를 다시 녹화하면
  idx는 전부 밀리지만 실제 정지선 위치는 그대로이므로, 좌표 라벨은 재녹화 후에도 살아남습니다.
- 저장 시 mission.yaml의 `labels:` 블록만 교체되고 나머지(주석, `steps:`, 튜닝 파라미터)는
  바이트 단위로 보존됩니다.
- 줌/팬 도구가 켜져 있는 동안의 클릭은 무시되므로, 정지선 근처를 확대하다가 라벨이
  잘못 찍히지 않습니다.
### 배경 깔기

- **시뮬레이션**: `--gazebo-course`가 `hyper_gazebo`의 코스 텍스처
  (`driving_course/meshes/course.png`)를 배경으로 깝니다. 배치 범위는 하드코딩이 아니라
  같은 폴더의 `ground.obj` 쿼드 정점에서 읽으므로, `build_course.py`로 메시를 다시 생성해도
  오버레이가 시뮬레이터와 어긋나지 않습니다. 확대하면 정지선·횡단보도가 보여서 클릭 지점을
  눈으로 확인할 수 있습니다.
  - 텍스처의 픽셀 종횡비(3937 x 4492)와 쿼드 종횡비(102.5 x 122.5 m)는 약 5% 다릅니다.
    Gazebo가 텍스처를 쿼드에 늘려 붙이므로 이 툴도 같은 범위를 그대로 써서 동일하게
    늘립니다 -- 기록된 웨이포인트가 차선 위에 정확히 얹히는 것을 확인했습니다.
  - 큰 텍스처는 `--background-max-px`(기본 2500)로 다운샘플해서 로드하므로 팬/줌이
    느려지지 않습니다.
- **실차**: 위성 정사영상을 `--background`와 `--extent`(map 프레임 미터 단위 경계)로
  주세요. 단 실차 좌표계가 성립하려면
  [hyper_localization/config/datums.yaml](../../localization/hyper_localization/config/datums.yaml)의
  `track` datum 실측이 먼저입니다(현재 `0.0` TODO 상태).
