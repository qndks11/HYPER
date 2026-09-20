# hyper_gazebo

HYPER 차량을 Gazebo에서 실행하기 위한 시뮬레이션 전용 패키지입니다. 트랙 월드, 차량 스폰, Gazebo-ROS 브리지, 시뮬레이션 센서와 제어 플러그인을 제공합니다.

## 실행

```bash
ros2 launch hyper_gazebo vehicle.launch.py
```

`worlds/track.world`가 기본 주행 환경이며, 초기 차량 위치와 자세는 launch 인자로 변경할 수 있습니다. 실차에서는 이 패키지 대신 실제 센서·제어 패키지를 사용합니다.

`site:=track|school`(기본 `track`)이 월드(`worlds/<site>.world`), datum 대조 대상
(`datums.yaml[<site>]`), 스폰 기본값(`<site>/start_left.csv` 0번 행), teleport_service가 읽는
미션(`mission_<site>.yaml`)을 한 번에 고릅니다. `world:=`, `x:=` 등을 직접 주면 그 값이 이깁니다.

```bash
ros2 launch hyper_gazebo vehicle.launch.py site:=school
ros2 launch hyper_launch simulation.launch.py site:=school   # 전체 스택 (mission_school, datum school)
```

## 학교 월드 (`school.world`)

학교 실습장의 빈 월드입니다 -- 라바콘, 신호등, 차로 표지판, 주차 차량이 하나도 없고 바닥에
항공사진만 깔려 있습니다. 트랙과 같은 원리로 실제 학교와 같은 map 좌표를 쓰므로
`waypoints/school/*.csv`와 `mission_school.yaml`을 그대로 씁니다.

- 바닥: `models/school_course` 모델이 `driving_course/meshes/school_ground.obj`(94.2045 x 53.1487 m)에
  `school.png`를 입힙니다. PNG를 옮기지 않은 것은 waypoint studio가 그 자리에서 배경을 찾기 때문입니다.
- 배치: 월드의 include `<pose>`는 `school.align.yaml`의 center(-6.2, 18.6), 회전 0을 그대로 씁니다.
  스튜디오에서 정렬을 다시 하면 이 pose와 obj 크기(`width_m`)도 같이 고치세요.
- 원점: `<spherical_coordinates>`는 `datums.yaml`의 `school` 항목과 반드시 같아야 합니다.
- world name은 `course_world` 그대로입니다(sim 서비스들의 기본값). lane_sign_service의 표지판은
  이 월드에 없으므로 Swap을 누르면 실패만 합니다.

## 용인 트랙의 디지털 트윈

**이 월드는 실차 트랙(용인)과 같은 map 좌표를 씁니다.** 그래서 `waypoints/track/*.csv`와
`mission_track.yaml`을 시뮬과 실차에서 그대로 같이 씁니다 -- 시뮬레이션 전용 코스도,
전용 미션도, 전용 datum도 없습니다.

성립하는 이유는 `sim에서 map 프레임 = Gazebo 월드 프레임`이기 때문입니다.
`datums.yaml`이 `heading_deg: 0`으로 map을 ENU에 정렬해 두었고(map X=East, Y=North),
gz navsat 센서는 월드 좌표에서 위경도를 만들며, 월드의 `<spherical_coordinates>`가
`datums.yaml`의 `track` 원점과 같은 값이기 때문입니다. **둘 중 한쪽만 고치면 map이
Gazebo 월드에서 통째로 밀립니다** -- 토픽도 TF도 멀쩡해 보이고 증상은 "웨이포인트를
따라가는데 자꾸 옆으로 샌다"로만 나타나므로, `vehicle.launch.py`가 띄울 때 둘을
대조해 경고합니다.

### 코스를 어떻게 맞췄나

`meshes/course.png`는 원래 용인 트랙을 옮겨 그린 그림인데, **약 70도 돌아가 있고
10.7% 크게** 그려져 있었습니다. `worlds/models/driving_course/fit_to_track.py`가 옛
시뮬 주행 기록을 실차 RTK 기록에 맞춰 닮음변환을 구했습니다:

| | |
| --- | --- |
| scale | 0.90320 (메시 좌표에 구워 넣음: `ground.obj`, `build_hill.py`의 `SCALE`) |
| rotation | +69.8424° CCW (월드의 `driving_course` include `<pose>`) |
| translation | (-12.1547, -6.0617) m (같은 `<pose>`) |
| 잔차 | mean 0.884 / median 0.715 / p90 1.919 m |

잔차 0.7 m는 변환으로 없앨 수 없습니다 -- 그림이 실제 아스팔트보다 곡선 반경을
예쁘게 이상화했고, 닮음변환은 회전/등방 스케일/평행이동만 할 수 있기 때문입니다.
미션 로직과 경로 추종에는 충분하지만, 차선 검출을 이 텍스처로 검증할 때는 구간에 따라
칠해진 차선이 실제 주행선에서 최대 2 m쯤 벗어날 수 있습니다(특히 `common_2`, `s_curve`,
`t_left`). 더 맞춰야 하면 `course.png`를 조각별로 휘어 실측 중심선에 박는 수밖에 없습니다.

스케일을 적용하면서 **차선 폭이 3.00 m에서 2.71 m로 줄었습니다.** 차량은 줄이지 않았으므로
(실물 1/5 차입니다) 시뮬이 그만큼 좁아졌고 실제에 가까워졌습니다 -- 조향/lookahead 감이
예전과 다릅니다. 경사로 마루 높이는 실물 높이라 1.5 m 그대로입니다.

소품(라바콘, 신호등, 주차 차량, 차로 표지판)은 위치와 방위만 같은 변환으로 옮겼고
크기는 그대로입니다 -- 코스가 줄어도 라바콘은 라바콘 크기입니다.

### 절대 방위 (`scripts/sim_heading.py`)

`ekf_global`의 절대 방위 관측은 `imu1` = `/imu/heading` 하나뿐이고, 실차에서는 듀얼 GNSS
moving-base 수신기가 그 토픽을 냅니다. 시뮬에는 두 번째 안테나가 없으므로
`sim_heading.py`가 gz IMU의 ENU yaw를 같은 토픽으로 중계합니다. 이게 없으면 시뮬의
map yaw가 0에서 시작해 월드와 어긋나고, 위의 트윈 전제가 깨집니다. 덕분에
`dual_ekf_navsat.yaml`은 시뮬과 실차가 한 글자도 다르지 않습니다.
