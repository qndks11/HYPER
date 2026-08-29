# hyper_lane_detection

카메라 영상에서 차선과 정지선을 검출하는 OpenCV 기반 인지 패키지입니다. 입력 영상을 버드아이뷰로 변환해 주행에 필요한 차선 중심·정지선 정보를 발행합니다. 차선/정지선 검출 알고리즘 자체는 `process_frame()` 한 곳에 모여 있고, 아래 두 입력 백엔드 모두 이 함수와 하나의 `sensor_msgs/Image` 구독 콜백(`raw_image_callback()`)을 그대로 공유합니다 — 차이는 그 토픽을 누가 발행하는지, 그리고 어느 카메라의 지면 투영 파라미터로 워프하는지뿐입니다.

`LaneDetection`은 `rclcpp_components`로 등록된 컴포넌트입니다. `ros2 run hyper_lane_detection lane_detection_node`로 단독 실행할 수도 있고, 다른 컴포넌트와 같은 `ComposableNodeContainer` 프로세스에 로드할 수도 있습니다 — `input_backend:=intra_process`는 항상 후자로 실행됩니다.

## input_backend 파라미터

카메라 영상이 어디서 들어오는지는 `input_backend` 파라미터로 고릅니다.

- `intra_process` — 실차용. `hyper_camera`의 `ElpCameraPublisherNode` 컴포넌트(`/dev/video_elp`를 열어 MJPEG 캡처·디코드하고 `ELP-USBGS1200P01-KL170.yaml` 보정값으로 rectify)가 이 노드와 같은 `ComposableNodeContainer`에 함께 로드되어 `image_raw`를 발행합니다. rclcpp의 intra-process 매니저가 그 프레임을 직렬화 없이 포인터로 바로 넘겨주므로, 별도 DDS 토픽 왕복이 없습니다. 후방 카메라 경로는 없습니다(실물이 없어도 실패하지 않음).
- `ros_raw` — 시뮬레이션(Gazebo)용. `/camera/image_raw`, `/camera_rear/image_raw`를 평범한 `sensor_msgs/Image`로 구독합니다. 시뮬레이션 카메라 영상은 이미 보정된 입력으로 취급하므로 rectify하지 않습니다.

두 백엔드 모두 노드 자체는 `/image_raw`, `/rear_image_raw`(둘 다 remap 대상)를 구독할 뿐이며, ELP 카메라를 여는 파라미터(`video_device`, `image_width`/`image_height`, `framerate`, `calibration_file`)는 이제 `hyper_camera`의 `ElpCameraPublisherNode`가 선언합니다.

## 입출력

- 입력: `input_backend`에 따라 다름 (위 참고)
- 출력: `/lane/center`, `/stopline/detection`
- 후방 출력: `/lane/rear_center`, `/stopline/rear_detection` (`intra_process`에서는 발행되지 않음)
- 주행가능영역: `/lane/drivable_area` (`nav_msgs/OccupancyGrid`, `drivable.enabled` 필요 -- 아래 참고)
- 디버그 영상: `/lane/bev/image_raw`, `/lane/rear_bev/image_raw`, `/lane/drivable/image_raw` (`sensor_msgs/Image`, BGR8)
- 지면 투영 디버그: `/lane/bev/points`, `/lane/rear_bev/points` (`sensor_msgs/PointCloud2`, XYZRGB)

## 디버그 뷰 (BEV / IPM)

`process_frame()`이 만드는 버드아이뷰 디버그 화면(차선·정지선 마스크와 주석, 하단 텍스트 패널까지 포함한 그대로)은 `/lane/bev/image_raw` 토픽으로만 나갑니다. 이 노드는 `cv::imshow` 창을 열지 않습니다 — GUI 창은 노드를 로컬 X 디스플레이에 묶어버리고(헤드리스 실차에는 없음), 차선 검출과 무관한 툴킷 문제로 노드 전체가 죽는 원인이 됩니다. 구독자가 없으면 발행 자체를 건너뛰므로 평소 오버헤드는 없습니다.

RViz2의 **Image** 디스플레이(또는 `rqt_image_view`)로 봅니다.

같은 화면을 3D 씬의 지면 위에 깔아서 코스트맵·경로와 겹쳐 보려면 `/lane/bev/points`(PointCloud2,
XYZRGB)를 씁니다. BEV 픽셀 하나가 점 하나이고, 위치는 아래 지면 투영이 정한 스케일·원점을 그대로
쓰므로 발행되는 `offset_m`/`distance_m`과 같은 좌표계입니다 — **지면 오버레이가 코스트맵과 눈에
띄게 어긋나면, 그 오차가 그대로 발행값에도 들어 있다는 뜻입니다.** 워프에서 원본 픽셀을 못 읽은
검은 모서리는 "데이터 없음"이라 점을 만들지 않습니다(코스트맵을 검게 덮지 않도록). 텍스트 패널도
지면에 있는 것이 아니므로 제외됩니다.

관련 파라미터:

| 파라미터 | 기본값 | 설명 |
| --- | --- | --- |
| `bev_cloud_frame_id` | `body_link` | 오버레이를 발행할 차량 프레임. 이 프레임 원점이 BEV 안 어디에 오는지는 `GroundProjection::origin_px()`가 계산합니다(보통 마지막 행보다 **아래** — 전방 카메라는 자기 발밑 지면을 못 보기 때문). 이 스택의 base 프레임 이름은 `base_link`가 아니라 `body_link`입니다(ekf `base_link_frame`, nav2 `robot_base_frame`). |
| `bev_cloud_stride` | `2` | 몇 픽셀마다 점을 만들지. 1이면 전체(640x260 BEV 기준 카메라당 16.6만 점/프레임). |
| `bev_cloud_z_m` | `-0.15` | 지면 기준 z 오프셋. RViz가 코스트맵도 z=0에 그리므로 z-fighting 방지용이며, **음수**라서 오버레이가 코스트맵·경로·footprint 뒤로 깔립니다(양수면 그 위를 덮어버림). 두 EKF 모두 `two_d_mode`라 `body_link` 자체가 z=0이므로 이 값이 곧 코스트맵 평면과의 높이차입니다. 예전 `-0.02`는 너무 얇았습니다 — 코스트맵도 오버레이도 반투명(alpha 0.4 / 0.9)이라 깊이를 기록하지 않고 카메라 거리순으로만 정렬되므로, 2 cm 차이로는 시점에 따라 오버레이가 위로 올라왔습니다. |

## 주행가능영역 (drivable area)

전방 BEV를 **주행 가능(0) / 진입 금지(60) / 주행 불가(100) / 데이터 없음(-1)** 으로 분류해
`/lane/drivable_area`
(`nav_msgs/OccupancyGrid`, `bev_cloud_frame_id` 기준)로 발행합니다. 이걸 로컬 코스트맵에 넣는
쪽은 `hyper_costmap_plugins`의 `DrivableAreaLayer`입니다 -- 그 README에 코스트 해석과 튜닝이
있습니다.

기본값은 **꺼짐**(`drivable.enabled: false`). 이 노드의 다른 출력은 구독자가 없으면 공짜인 디버그
뷰지만 이건 코스트맵을 거쳐 차량을 조종하므로, 켜는 것은 launch 파일이 명시적으로 할 일입니다:

```bash
ros2 launch hyper_launch simulation.launch.py drivable_area:=true
```

### 학습 모델이 아니라 색 규칙인 이유

시뮬레이터 코스 텍스처의 실제 색 분포(HSV):

| | H | S | V | 판정 |
| --- | --- | --- | --- | --- |
| 아스팔트 | 4 | **12** | 151 | 주행 가능 |
| 연석/갓길 | 90 | **1** | 178 | 주행 가능 |
| 흰 페인트 | 0 | **0** | 255 | 주행 가능 |
| 노란 차선 | 30 | 255 | 193 | **진입 금지**(넘으면 안 되는 선) |
| 잔디 | 74 | **205** | 127 | 주행 불가 |

노면 계열은 전부 무채색이고 코스를 가르는 것은 전부 유채색이라, 규칙 하나로 갈립니다:

```
surface = (S < surface_max_saturation) && !yellow_paint
```

밝기(V)가 아니라 **채도(S)**를 쓰는 이유는 그림자입니다 -- 그늘진 아스팔트는 어둡지만 여전히
무채색이라, 밝기 임계값이 정확히 실패하는 지점에서 채도는 거의 움직이지 않습니다.

노란 차선은 채도가 높아 이 규칙에서 이미 빠지고, **의도적으로 그대로 둡니다**. 노랑은 코스
경계와 중앙선, 즉 넘으면 안 되는 선이므로, 노면에 뚫린 구멍으로 남겨두면 3단계 flood fill이
반대 차선이나 코스 밖으로 새지 못하게 막는 벽이 됩니다. 다만 그게 전부입니다 -- 페인트 자체는
다른 모든 비주행 영역과 똑같이 `kOffLimits`(60)로 나갑니다(4단계 참고). 흰 페인트는 무채색이라 그대로 노면으로 남습니다 -- 흰 선은 경계가 아니라
차선 힌트입니다.
(노랑 범위는 `LaneDetector::yellow_mask()`와 같은 값이므로 한쪽을 바꾸면 같이 바꾸세요.)

### 네 단계

1. **색** -- 위 규칙.
2. **모폴로지** -- close 후 open, 반지름은 미터로 지정(`meters_per_pixel`이 바뀌어도 같은 뜻).
   close가 글레어가 뚫은 구멍을 메워 flood fill이 새거나 쪼개지지 않게 하고, open이
   잔디 위 고립된 회색 점을 지웁니다. 그다음 **노란 페인트를 다시 뺍니다** -- close 반지름은
   차선 하나를 통째로 메울 만큼 넓어서, 먼저 빼두면 다시 메워집니다.
3. **도달성** -- 차량 바로 앞 seed 패치가 닿는 연결 성분만 남깁니다. **이 단계가 핵심입니다**:
   색만 보면 잔디 건너편 회색 도로도 주행 가능으로 읽히고, 컨트롤러는 코스 안쪽을 가로질러
   계획합니다. BEV에서의 연결성이 곧 지면에서의 연결성이므로 "도달 가능"이 말 그대로 성립합니다.
   2단계가 노란 차선을 구멍으로 남겨두므로, 끊기지 않은 노란 선은 아스팔트 가장자리와 똑같이
   flood fill을 가둡니다.
4. **라벨링** -- fill이 닿은 곳은 `kDrivable`(0), 프레임 안의 나머지는 전부
   `kOffLimits`(60)입니다. 잔디든 페인트든 그 너머 반대 차선이든 세기를 나누지 않습니다.
   **이 레이어는 어떤 셀도 치명으로 내보내지 않습니다**: 지면 평면 호모그래피에는 높이 정보가
   없으므로, 한 프레임의 오분류(글레어, 그림자 경계, 보간된 원거리 행)가 충돌 판정이나 팽창
   헤일로를 만들 수 있으면 안 됩니다. 높이는 라이다 장애물 레이어의 주제이고, 치명을 낼 수
   있는 것도 그쪽입니다. 회피는 코스트맵의 `off_limits_cost`(200)가 MPPI에 주는 이격거리로
   냅니다.

   `kUndrivable`(100) 상수는 소비자 쪽(`DrivableAreaLayer`의 `lethal_threshold`) 때문에 남아
   있을 뿐, 지금 이 검출기는 그 값을 만들지 않습니다.

seed 패치에 노면이 `min_seed_coverage`만큼도 없으면 전부 **unknown**으로 발행하고 경고합니다 --
전부 주행 불가가 아닙니다. 그 상태에서 빠져나오는 방법은 주행이고, 차 주위를 코스트로 둘러싸면
그걸 막습니다.

### 한계

호모그래피 기반 지면 투영이 다 그렇듯 **세상이 지면이라고 가정**합니다. 높이가 있는 것(콘, 벽,
다른 차)은 제 발자국이 아니라 카메라에서 멀어지는 방향으로 늘어집니다. 이 레이어 목적에는
보수적인 방향의 오차지만(장애물 그림자가 주행 불가로 찍히고, 그 방향은 항상 차에서 *멀어지는*
쪽), 그래서 이건 라이다 장애물 레이어의 **보완**이지 대체가 아닙니다. 코스의 언덕(hill)은 평면
가정을 정면으로 깨므로 그 구간의 마스크는 틀립니다.

### 파라미터

| 파라미터 | 기본값 | 설명 |
| --- | --- | --- |
| `drivable.enabled` | `false` | |
| `drivable.max_range` | `6.0` | 발행할 전방 거리 [m]. local costmap의 `obstacle_max_range`와 맞췄습니다 |
| `drivable.max_lateral` | `4.0` | 좌우 [m]. `bev.half_width`(9.0)는 프레임 가장자리 몇 픽셀에서 늘려낸 것이라 그대로 쓰면 flood fill에 가장 못 믿을 입력을 주는 셈입니다 |
| `drivable.surface_max_saturation` | `40` | 분류기의 전부. 위 표 참고 |
| `drivable.paint_hue_min` / `paint_hue_max` / `paint_min_saturation` | `22` / `35` / `80` | 노란 차선. `LaneDetector::yellow_mask()`와 같은 값 |
| `drivable.close_radius` / `open_radius` | `0.12` / `0.06` | 모폴로지 반지름 [m] |
| `drivable.seed_width` / `seed_depth` | `0.81` / `0.60` | seed 패치 [m]. 폭 기본값은 차량 윤거 |
| `drivable.min_seed_coverage` | `0.25` | 이보다 낮으면 전부 unknown |

### 디버그 뷰

`/lane/drivable/image_raw` -- 크롭된 BEV 위에 초록(주행 가능)/주황(off-limits) 틴트와 seed 패치
사각형(흰색 = seed 성공, 빨강 = 실패). BEV 디버그 뷰와 마찬가지로 **Best Effort**입니다.
켜기 전에 이걸 `rqt_image_view`로 먼저 확인하세요.

## 지면 투영 (IPM) 파라미터

BEV 워프는 이미지 위에서 고른 사다리꼴 ROI가 아니라, **카메라가 실제로 어디에 있는지**에서 유도합니다
(`ground_projection.hpp`). 보고 싶은 지면 사각형을 미터로 지정하면 그 네 꼭짓점을 카메라 모델로
영상에 투영해 호모그래피를 만들므로, 출력 래스터는 만들기 전부터 미터 격자입니다. 그래서 픽셀→미터
스케일이 두 축에서 같고(등방), 원점 위치가 계산 가능하며, FOV·해상도·장착 높이/각도를 바꾸면
호모그래피가 따라 바뀝니다.

전방은 `bev.*`, 후방은 `bev_rear.*` 접두사를 씁니다. 기본값은 **시뮬레이터** 카메라 기준입니다
(`hyper_control/config/parameters.yaml`의 값과 일치). 실차 전방 카메라는 렌즈 모델이 달라
`config/bev_real.yaml`이 `intra_process` 경로에서 자동으로 덮어씁니다.

| 파라미터 | `bev` 기본값 | 설명 |
| --- | --- | --- |
| `horizontal_fov` | `2.67` | 수평 화각 [rad]. 왜곡 없고 주점이 화면 중앙인 이상적 핀홀을 프레임 크기로부터 유도합니다 — Gazebo가 렌더링하는 모델 그대로. `fx`가 설정되면 무시됩니다. |
| `fx`, `fy`, `cx`, `cy` | `0.0` (미사용) | rectify된 실제 렌즈용 명시적 내부 파라미터. 실렌즈는 `fx != fy`이고 주점도 중앙이 아니라서 화각만으로는 표현되지 않습니다. `fx > 0`이면 `horizontal_fov`보다 우선합니다. |
| `camera_height` | `1.5` | 광학 중심의 **지면** 위 높이 [m]. `parameters.yaml`의 `camera_height`(1.2)와 기준면이 다릅니다 — 그쪽은 `body_link` 기준 카메라 조인트 z이고, `body_link`는 바퀴 위에 0.3 m 떠 있습니다(`vehicle.xacro`가 바퀴 조인트를 `-wheel_radius/2`에 달고 바퀴 반지름이 0.2). 1.2를 그대로 옮겨 쓰면 오버레이 전체가 실제 거리의 0.80배로 그려집니다. |
| `camera_pitch` | `0.2617994` | 아래로 숙인 각 [rad]. **세 값 중 오차에 가장 민감합니다.** |
| `camera_longitudinal_offset` | `0.145` | 차량 프레임 원점에서 카메라까지, **카메라가 보는 방향으로** 잰 거리 [m]. 후방 카메라도 같은 부호 규약이라 음수를 넣지 않습니다. |
| `near` / `far` | `0.3` / `7.6` | BEV 맨 아래/맨 위 행이 보여줄 지면 거리 [m]. `near`를 카메라가 볼 수 있는 것보다 가깝게 잡으면 그 행들은 그냥 검게 남습니다. |
| `half_width` | `9.0` | 좌우로 각각 얼마나 넓게 볼지 [m]. |
| `meters_per_pixel` | `0.028125` | BEV 픽셀 하나가 덮는 지면 거리 [m/px]. 양 축 공통 — 이 값이 곧 발행되는 `offset_m`/`distance_m`의 환산 계수입니다. |

시작할 때 카메라별로 실제 만들어진 형상이 로그로 한 줄 남습니다(지면 범위, BEV 크기, 원점 위치,
지평선 행, 원본 프레임 밖으로 나간 꼭짓점 수). 장착 파라미터가 틀리면 RViz를 보기 전에 이 줄에서
먼저 드러납니다.

> `bev_rear` 기본값은 전방과 달리 예전 설정을 계승하지 않았습니다. 예전 후방 ROI는 24 m 뒤까지
> 잡으면서 스케일이 가로로 64% 넓고 세로로 54% 짧아 보존할 튜닝 자체가 없었기 때문에, 주차
> 기동에 맞춰 다시 잡았습니다(`near` 1.8 m는 이 카메라가 실제로 볼 수 있는 가장 가까운 지면입니다
> — 1.5 m 높이에 15도만 숙이면 화면 맨 아랫줄이 차량 뒤 1.74 m에 떨어집니다. 이 사각지대는 워프로
> 복구할 수 없고, 주차용 후방 카메라라면 더 낮게 달거나 훨씬 더 숙여야 합니다).

후방 카메라(`/lane/rear_bev/points`)는 뒤를 보므로 이미지 전체가 -x/-y로 매핑됩니다(차량 z축
기준 180도 회전).

```bash
ros2 run hyper_lane_detection lane_detection_node --ros-args -p input_backend:=ros_raw
rviz2
```

RViz2에서 `Add` → `Image` → Topic을 `/lane/bev/image_raw`로 지정하고, **Reliability Policy를 `Best Effort`로** 바꿔야 화면이 나옵니다. 이 토픽은 영상 스트림 관례대로 best-effort/depth 1로 발행되므로, RViz의 기본값(`System Default` = Reliable)과는 QoS가 맞지 않아 아무것도 표시되지 않습니다.

주의: 이 영상은 IPM(호모그래피)으로 펴진 평면 뷰일 뿐 그 자체가 3D 씬에 놓이는 것은 아니라, RViz의 2D 이미지 패널로만 보입니다. 차량 기준 지면 위에 겹쳐 보려면 같은 투영으로 `body_link` 상에 발행되는 `/lane/bev/points`를 쓰십시오(위 참고).

## 실행

```bash
ros2 run hyper_lane_detection lane_detection_node --ros-args -p input_backend:=ros_raw
```

`intra_process`는 `ComposableNodeContainer`로만 의미가 있으므로(zero-copy는 같은 프로세스일 때만 성립), 단독 `ros2 run`이 아니라 `hyper_object_detection`의 `perception.launch.py`(또는 `hyper_launch perception.launch.py`, `lane_input_backend` 인자로 전달)로 실행합니다. `hyper_launch`의 `real.launch.py`는 `intra_process`를, `simulation.launch.py`는 `ros_raw`를 기본으로 넘깁니다.
