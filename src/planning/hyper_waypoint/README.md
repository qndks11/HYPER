# hyper_waypoint

`odometry/filtered_map`을 구독해서 `idx, x, y, yaw, frame_id`를 CSV로 기록하는 웨이포인트 레코더 패키지입니다.
코스를 보고 고치는 GUI는 [hyper_waypoint_studio](../../tools/hyper_waypoint_studio/README.md)입니다.

## 실행 (권장) — GUI로 조작

실차 수동 주행 launch(`joystick_control_real.launch.py`, `phone_control_real.launch.py`)가
레코더와 조작판 GUI를 이미 포함합니다. 따로 띄우고 싶을 때만:

```bash
ros2 launch hyper_waypoint record.launch.py \
  waypoint_csv:=$HOME/HYPER/src/planning/hyper_waypoint/waypoints/track/real.csv
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
| 캔버스 | 지금까지 찍힌 점(파랑), 현재 위치(보라), **저장 파일 칸이 가리키는 CSV에 이미 들어
있는 이전 녹화본(회색)**, 그리고 같이 올려 둔 다른 코스와 배경 이미지 |

이전 녹화본은 창이 뜰 때, 그리고 저장 파일 이름을 바꿀 때마다 그 CSV를 직접 읽어
깔아 둡니다 -- `Record`가 무엇을 덮어쓰는지 누르기 전에 보이게 하려는 것입니다.
`Record`를 누르면(=truncate) 회색 선은 사라지고 이번 녹화만 남습니다.

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
ros2 run hyper_waypoint waypoint_recorder_node --ros-args -p output_csv:=$HOME/HYPER/src/planning/hyper_waypoint/waypoints/track/real.csv -p min_spacing_m:=0.5
```

- `auto_start` 파라미터의 기본값이 `true`라 이렇게 띄우면 **즉시 기록을 시작**합니다(기존 사용법 그대로). `Ctrl-C`로 원하는 시점에 종료하세요.
- `output_csv` 파라미터를 생략하면 노드를 실행한 위치에 `waypoint_record.csv`로 저장됩니다.
- `min_spacing_m` 파라미터(기본값 `0.5`)는 직전 기록 지점으로부터 이 거리(m) 이상 이동했을 때만 새 줄을 기록합니다. 시간 간격이 아니라 이동 거리 기준으로 웨이포인트가 샘플링됩니다.
- 파일은 `idx,x,y,yaw,frame_id` 헤더로 시작하며, 매 기록마다 flush되므로 중간에 종료해도
  그때까지 기록된 내용은 남아 있습니다.
- 예전에는 GPS/IMU/공분산까지 23컬럼을 같이 적었지만 더는 안 적습니다. 그 값들은 "이 점을
  녹화할 때 센서가 뭐라고 했는가"이지 "이 점이 어디인가"가 아니라, 스튜디오에서 점을 손으로
  옮기는 순간 전부 거짓말이 됩니다. 녹화 품질은 파일이 아니라 실시간 `~/status`(GPS 상태,
  EKF 공분산, 속도)로 봅니다 -- **`RTK / GBAS`(초록)여야 쓸 만한 녹화**입니다.
- 읽는 쪽은 전부 헤더 이름으로 찾으므로(`path_loader.hpp`, 스튜디오) 예전 23컬럼 녹화본도
  그대로 열리고 그대로 주행됩니다.

## 코스 보기 / 편집 / 라벨링

전부 [hyper_waypoint_studio](../../tools/hyper_waypoint_studio/README.md)로 옮겼습니다.
예전의 `label_waypoints.py`(라벨링)와 `waypoint_record_gui.py`(녹화 조작판)는 없습니다.

```bash
# 코스를 시뮬 텍스처 위에 올리고 미션 라벨을 찍기
ros2 run hyper_waypoint_studio waypoint_studio \
  src/planning/hyper_waypoint/waypoints/simulation/sim1.csv \
  --mission src/planning/hyper_planner/config/mission_sim.yaml \
  --overlay gazebo --mode edit

# 실차 코스를 항공사진 위에 (정렬값은 real_course.align.yaml)
ros2 run hyper_waypoint_studio waypoint_studio \
  src/planning/hyper_waypoint/waypoints/track/full_track.csv \
  --overlay src/simulator/hyper_gazebo/worlds/models/driving_course/meshes/real_course.png
```

여러 코스를 한 화면에 겹쳐 볼 수 있으므로, 분기 코스(`sim_left.csv` / `sim_right.csv`)의
이음매를 눈으로 확인할 수 있습니다. 점을 끌어 고칠 때 후진 녹화 구간의 헤딩이 보존되는
방식과 라벨 스냅 경고는 스튜디오 README를 보세요.

## 웨이포인트 폴더

```
waypoints/
  simulation/   시뮬 코스 (sim1.csv가 mission_sim.yaml의 짝입니다)
  track/        실차 트랙 코스 -- 녹화 원본과 손으로 정리한 코스가 함께 있습니다
                (스튜디오의 "다른 이름으로 저장" 기본 위치)
  school/       교내 코스
```
