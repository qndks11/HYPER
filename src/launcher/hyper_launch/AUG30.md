# 2026-08-30 실차 테스트 절차

오늘 목표 세 가지. **순서대로** 하세요 — 2번은 1번이 물려야 의미가 있고, 3번은 2번의 결과물을 씁니다.

1. [RTK fix 확인](#1-rtk-fix-확인) — `joystick_control_real.launch.py` (1번과 2번은 같은 스택,
   내렸다 다시 안 띄워도 됨)
2. [웨이포인트 기록](#2-웨이포인트-기록) — 같은 `joystick_control_real.launch.py` (레코더 + GUI 포함)
3. [웨이포인트 따라가는지 확인](#3-웨이포인트-따라가는지-확인) — `real.launch.py`

---

## 1. RTK fix 확인

2번과 같은 명령을 씁니다. `joystick_control_real.launch.py`가 센서(u-blox + NTRIP + IMU +
라이다) + GPS 정확도 모니터 GUI를 이미 같이 띄우므로, 별도로 `sensors.launch.py`를 따로
띄울 필요가 없습니다.

```bash
# 터미널 1 — 센서 + TF + EKF + 조이스틱 + GPS 모니터 + 웨이포인트 레코더
ros2 launch hyper_launch joystick_control_real.launch.py datum_site:=school
```

GPS 정확도 모니터(hAcc/vAcc/fix/RTK 비트를 큰 글씨로) GUI가 같이 뜹니다. 아래 확인이 끝나면
그대로 2번(웨이포인트 기록)으로 이어가면 됩니다 — 스택을 내렸다 다시 띄울 필요 없습니다.

> GUI가 안 뜨고 죽으면 snap GTK 충돌입니다. `unset GTK_PATH GIO_MODULE_DIR` 후 다시 실행하세요.

### 확인할 것

| 확인 | 명령 | 합격 기준 |
| --- | --- | --- |
| fix가 나오는가 | `ros2 topic hz /gps/fix` | **5 Hz** (`rtk.launch.py`의 `rate: 5.0`) |
| RTK 등급 | GUI의 우측 상태줄 | `RTK FIXED` (`FLOAT`면 아직 수렴 중) |
| 수평 정확도 | GUI의 `Horizontal (hAcc)` | **FIXED면 0.02 m 근처**, FLOAT는 0.3~1 m, 단독측위는 1~3 m |
| NTRIP 보정이 오는가 | `ros2 topic hz /rtcm` | 꾸준히 올라와야 함. 0이면 caster 연결 실패 |
| 바퀴 오도메트리 | `ros2 topic hz /odom` | 나와야 함. 없으면 아두이노 브리지가 죽은 것 |
| EKF가 실제로 도는가 | `ros2 topic hz /odometry/filtered_map` | **30 Hz**. 0이면 녹화해 봐야 소용없음 |

명령줄로 직접 보고 싶으면:

```bash
ros2 topic echo /ublox_gps_node/navpvt --field flags
#   flags & 192 -> 128 = RTK FIXED,  64 = RTK FLOAT,  0 = 보정 없음
#   flags & 1   -> GNSS_FIX_OK
```

### 막히면

- `/rtcm`이 0 Hz → [ntrip_params.yaml](../../sensing/hyper_rtk/config/ntrip_params.yaml)의
  host/mountpoint/계정. 이 파일은 gitignore 대상이라 장비마다 다릅니다.
- `/gps/fix`가 아예 없음 → `ls -l /dev/tty_Ardusimple` (udev 심볼릭 링크)
- FLOAT에서 FIXED로 안 올라감 → 하늘이 트인 곳에서 몇 분 더. 여기서 FIXED를 못 보면
  **2번으로 넘어가지 마세요.** 정확도 1 m짜리 fix로 녹화한 코스는 3번에서 쓸 수 없습니다.
- **IMU(WT901BLE)가 안 붙음** → GUI의 "IMU link"가 NO DATA이거나 터미널에 
  `No BLE device found matching ...`가 반복되면, 십중팔구 **이전 실행이 BLE 연결을 물고
  죽은 것**입니다. 연결된 WT901BLE는 advertise를 멈춰서 다음 스캔에 아예 안 잡힙니다.

  ```bash
  bluetoothctl info FD:C0:E8:FE:A9:58        # Connected: yes 면 이 경우가 맞습니다
  bluetoothctl disconnect FD:C0:E8:FE:A9:58  # 끊고 다시 launch
  ```

  그래도 안 되면 어댑터가 여러 개인지 봅니다(드라이버는 첫 번째 것만 씁니다):
  `bluetoothctl list`. 스택을 내릴 때 Ctrl-C 후 노드가 완전히 끝날 때까지 기다리세요 —
  중간에 강제로 죽이면 같은 상태가 다시 만들어집니다.

---

## 2. 웨이포인트 기록

1번에서 띄운 스택을 그대로 씁니다 — 내렸다 다시 띄울 필요 없습니다. 터미널 하나로 끝납니다.
센서 + TF + EKF + 아두이노 브리지 + 조이스틱 + GPS 모니터 + **웨이포인트 레코더와 녹화 조작판 GUI**까지 한
트리입니다. 카메라와 차선/객체 인식은 아예 안 뜹니다

```bash
ros2 launch hyper_launch joystick_control_real.launch.py datum_site:=school
```

| 인자 | 기본값 | 비고 |
| --- | --- | --- |
| `datum_site` | `track` | 대회장이 아니면 `school` |
| `waypoint_csv` | `.../waypoints/real.csv` | 녹화 결과를 쓸 파일 |
| `min_spacing_m` | `0.5` | 이 거리 이상 이동했을 때만 한 점 기록 |
| `use_record_gui` | `true` | 녹화 조작판 GUI |
| `joystick_publish_period` | `0.0` | 0.0이면 노드 기본값 100 Hz |

EKF가 뜨는 데 5초 걸립니다(`ODOMETRY_DELAY_S`).

### 녹화

**레코더는 떠 있어도 아직 기록하지 않습니다.** GUI의 `● Record`를 누르는 순간부터입니다 --
그리고 그 순간이 곧 `real.csv`를 truncate하는 순간이므로, 스택을 띄우는 것만으로 지난
녹화본이 날아가지 않습니다.

1. 차를 출발점에 세웁니다.
2. GUI에서 **GPS 상태가 `RTK / GBAS`(초록)** 인지 확인합니다. 여기가 노란색/빨간색이면
   녹화해 봐야 3번에서 못 씁니다.
3. `● Record` → 상태가 `● REC`(빨강)으로 바뀝니다.
4. 코스를 한 바퀴 돕니다.
5. `■ Stop` → 파일이 닫히고 "stopped: N points, M m" 메시지가 뜹니다.

다시 찍고 싶으면 스택을 내릴 필요 없이 `Record`를 다시 누르면 됩니다(idx 0부터 새로 시작).

### GUI가 실시간으로 보여주는 것

| 항목 | 녹화 중 봐야 할 것 |
| --- | --- |
| 기록된 점 / 누적 거리 | 점이 계속 늘어야 합니다. 멈춰 있으면 EKF가 죽었거나 차가 안 움직이는 것 |
| 다음 점까지 | 마지막 점에서 얼마나 왔는지. 0.5 m를 넘는 순간 한 점이 찍힙니다 |
| 현재 위치 (map) | 값이 없으면 `/odometry/filtered_map`이 안 나오는 것(= EKF 문제) |
| GPS 상태 | **`RTK / GBAS`(초록)를 유지해야 합니다.** 중간에 떨어지면 그 구간은 못 씁니다 |
| EKF 공분산 xx / yy | 커지면 융합이 흔들리는 중 |
| 미니맵 | 지금까지 찍힌 점(파랑)과 현재 위치(보라). 코스 모양이 실제와 맞는지 눈으로 확인 |

같은 내용이 `/waypoint_recorder/status`(String)와 `/waypoint_recorder/path`(Path)로도 나갑니다
(이 스택은 RViz를 띄우지 않습니다 -- 경로를 겹쳐 보려면 `rviz2`를 따로 실행하세요).

기존 `real.csv`는 `gps_status: -1`(fix 없음)에 위경도가 `-6.8, 134.8`인 쓰레기 기록이라
덮어써도 아깝지 않습니다. 그래도 불안하면 먼저 `cp real.csv real.csv.bak`.

### 녹화 중 별도 터미널에서 같이 볼 것

```bash
ros2 topic hz /imu/heading    # 주행 중 5 Hz 근처 (GPS fix rate와 같음)
```

`/imu/heading`이 주행 중에도 계속 10 Hz면 **초기 heading 시드가 안 꺼진 것**이고, 그건 GPS
코스 보정이 한 번도 안 들어왔다는 뜻입니다. 그대로 녹화하면 방위가 자이로 드리프트로만
갑니다. `min_fix_displacement`(0.5 m) 기본값 기준 **0.5 m/s 이상**으로 몰아야 매 fix마다
코스가 나옵니다.

### 녹화가 끝나면 — 품질 확인

```bash
python3 src/planning/hyper_waypoint/scripts/plot_waypoints.py \
  src/planning/hyper_waypoint/waypoints/real.csv --jump-threshold 1.0
```

융합 경로 / 생 GPS / navsat_transform 출력이 겹쳐 그려집니다. 점프가 있으면 그 원인이
GPS인지 좌표변환인지 EKF인지 여기서 갈립니다.

> `simple.yaml`은 이제 손댈 필요가 없습니다. 코스 끝 라벨이 좌표가 아니라 `course_end: last`
> (= CSV의 마지막 웨이포인트)라서, 새로 녹화한 `real.csv`에 그대로 맞습니다.

---

## 3. 웨이포인트 따라가는지 확인

**2번 수동 주행 스택을 먼저 Ctrl-C로 완전히 내리세요.** 조이스틱이 살아 있으면 차가
안 움직입니다. (내리기 전에 GUI에서 `■ Stop`을 눌러 파일을 닫아 두면 깔끔합니다.)

```bash
# 터미널 1 — 센서 + TF + EKF + 인지 + nav2 + 미션 매니저 + 아두이노 + RViz
ros2 launch hyper_launch real.launch.py \
  datum_site:=school \
  mission:=simple \
  waypoint_csv:=$HOME/HYPER/src/planning/hyper_waypoint/waypoints/real.csv
```

> `datum_site`는 1·2번과 같아야 합니다. 대회장이면 세 명령 모두 `track`.

### 출발 전 체크 (차를 세워 둔 채로)

behavior 스테이지는 9초 뒤에 뜹니다(`BEHAVIOR_DELAY_S`).

```bash
ros2 topic echo /mission_manager/status --once   # 미션이 로드됐는지
ros2 action list | grep follow_path              # 액션 서버가 떴는지
ros2 run tf2_ros tf2_echo map body_link          # map -> body_link TF가 사는지
```

터미널 1 로그에 이 줄이 보여야 합니다:

```
[mission_manager]: Label 'course_end' -> last waypoint #<N> (좌표 스냅 생략).
[mission_manager]: Mission '.../simple.yaml' loaded: 1 steps, 1 labels.
```

`#<N>`이 녹화한 CSV의 마지막 인덱스와 같은지 확인하세요
(`tail -1 .../real.csv | cut -d, -f1`).

RViz에서 계획 경로가 실제 녹화한 코스와 겹쳐 보이는지 눈으로 먼저 확인합니다. 코스가 엉뚱한
데 있으면 `datum_site`가 틀렸거나, 초기 yaw가 아직 확립되지 않은 것입니다
(gps_accuracy_gui의 "Calibrate initial yaw"를 먼저 돌리세요).

### 출발

미션 매니저는 `auto_start` 기본값이 `false`라 사람이 눌러야 출발합니다.
`real.launch.py`가 `use_panel:=true`(기본값)로 HYPER Panel을 behavior 스테이지와 같이
이미 띄워 두므로, 뜬 창에서 **Start**를 누르세요.

### 주행 중 볼 것

```bash
ros2 topic echo /cmd_vel --field linear.x            # nav2가 요구하는 속도
ros2 topic echo /odom --field twist.twist.linear.x   # 실제 바퀴 속도
```

이 둘이 크게 벌어지면(시뮬에서 2.03 대 0.92가 관측된 적 있음) 추종이 아니라 구동 쪽 문제입니다.

---

## 정리

세션이 끝나면 띄운 걸 **전부** 내리세요. 남은 ROS 노드는 놀고 있는 게 아니라 같은 토픽에
계속 publish하므로, 다음 실행이 중복 publisher나 낡은 TF를 조용히 물고 "확인"해 버립니다.

```bash
pgrep -af "ublox|ntrip|ekf_node|gps_heading|controller_server|mission_manager|witmotion|joy"
```
