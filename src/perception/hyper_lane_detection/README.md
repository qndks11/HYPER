# hyper_lane_detection

카메라 영상에서 차선과 정지선을 검출하는 OpenCV 기반 인지 패키지입니다. 입력 영상을 버드아이뷰로 변환해 주행에 필요한 차선 중심·정지선 정보를 발행합니다. 차선/정지선 검출 알고리즘 자체는 `process_frame()` 한 곳에 모여 있고, 아래 두 입력 백엔드 모두 이 함수와 하나의 `sensor_msgs/Image` 구독 콜백(`raw_image_callback()`)을 그대로 공유합니다 — 차이는 그 토픽을 누가 발행하는지, 그리고 어느 카메라의 지면 투영 파라미터로 워프하는지뿐입니다.

`LaneDetection`은 `rclcpp_components`로 등록된 컴포넌트입니다. `ros2 run hyper_lane_detection lane_detection_node`로 단독 실행할 수도 있고, 다른 컴포넌트와 같은 `ComposableNodeContainer` 프로세스에 로드할 수도 있습니다 — `input_backend:=intra_process`는 항상 후자로 실행됩니다.

## input_backend 파라미터

카메라 영상이 어디서 들어오는지는 `input_backend` 파라미터로 고릅니다.

- `intra_process` — 실차용. `hyper_camera`의 `LogitechCameraPublisherNode` 컴포넌트(`/dev/video_logitech`를 열어 MJPEG 캡처·디코드, rectify 없음)가 이 노드와 같은 `ComposableNodeContainer`에 함께 로드되어 `image_raw`를 발행합니다. rclcpp의 intra-process 매니저가 그 프레임을 직렬화 없이 포인터로 바로 넘겨주므로, 별도 DDS 토픽 왕복이 없습니다. 같은 발행이 DDS로도 나가서 `object_detection_node`가 **동일한 카메라의 동일한 프레임**을 구독합니다 — 차량 카메라는 이 한 대뿐입니다.
- `ros_raw` — 시뮬레이션(Gazebo)용. `/camera/image_raw`를 평범한 `sensor_msgs/Image`로 구독합니다. 카메라 드라이버는 아무것도 뜨지 않습니다.

두 백엔드 모두 노드 자체는 `/image_raw`(remap 대상)를 구독할 뿐이며, 카메라를 여는 파라미터(`video_device`, `image_width`/`image_height`, `framerate`)는 `hyper_camera`의 `LogitechCameraPublisherNode`가 선언합니다. 어느 백엔드도 rectify하지 않습니다 — 실차 카메라가 보정 파일 없는 일반 렌즈라, 양쪽 다 이상적 핀홀로 모델링합니다.

## 입출력

- 입력: `input_backend`에 따라 다름 (위 참고)
- 출력: `/lane/center`, `/stopline/detection`
- 디버그 영상: `/lane/bev/image_raw` (`sensor_msgs/Image`, BGR8)
- 지면 투영 디버그: `/lane/bev/points` (`sensor_msgs/PointCloud2`, XYZRGB)

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

## 지면 투영 (IPM) 파라미터

BEV 워프는 이미지 위에서 고른 사다리꼴 ROI가 아니라, **카메라가 실제로 어디에 있는지**에서 유도합니다
(`ground_projection.hpp`). 보고 싶은 지면 사각형을 미터로 지정하면 그 네 꼭짓점을 카메라 모델로
영상에 투영해 호모그래피를 만들므로, 출력 래스터는 만들기 전부터 미터 격자입니다. 그래서 픽셀→미터
스케일이 두 축에서 같고(등방), 원점 위치가 계산 가능하며, FOV·해상도·장착 높이/각도를 바꾸면
호모그래피가 따라 바뀝니다.

접두사는 `bev.*` 하나입니다(후방 카메라는 배터리 절약을 위해 제거되었습니다 — `bev_rear.*`는
더 이상 없습니다). 기본값은 **시뮬레이터** 카메라 기준입니다 (`hyper_control/config/parameters.yaml`의
값과 일치). 실차와 시뮬레이터가 **같은 카메라(Logitech C920)** 를 모델링하므로 `config/bev_real.yaml`이
`intra_process` 경로에서 덮어쓰는 값은 딱 하나, `camera_height`입니다 — 시뮬레이터의 `body_link`는
지면에서 0.3 m 떠 있고 실차 라이드 하이트는 0.171 m라 지면 기준 높이만 1.412 m vs 1.283 m로
갈립니다. 나머지는 전부 동일합니다.

| 파라미터 | `bev` 기본값 | 설명 |
| --- | --- | --- |
| `horizontal_fov` | `1.2217305` | 수평 화각 [rad]. 왜곡 없고 주점이 화면 중앙인 이상적 핀홀을 프레임 크기로부터 유도합니다 — Gazebo가 렌더링하는 모델 그대로. `fx`가 설정되면 무시됩니다. |
| `fx`, `fy`, `cx`, `cy` | `0.0` (미사용) | rectify된 실제 렌즈용 명시적 내부 파라미터. 실렌즈는 `fx != fy`이고 주점도 중앙이 아니라서 화각만으로는 표현되지 않습니다. `fx > 0`이면 `horizontal_fov`보다 우선합니다. |
| `camera_height` | `1.412` | 광학 중심의 **지면** 위 높이 [m]. `parameters.yaml`의 `camera_height`(1.112)와 기준면이 다릅니다 — 그쪽은 `body_link` 기준 카메라 조인트 z이고, `body_link`는 바퀴 위에 0.3 m 떠 있습니다(`vehicle.xacro`가 바퀴 조인트를 `-wheel_radius/2`에 달고 바퀴 반지름이 0.2). 1.112를 그대로 옮겨 쓰면 오버레이 전체가 실제 거리의 0.79배로 그려집니다. 실차는 `bev_real.yaml`이 1.283으로 덮어씁니다. |
| `camera_pitch` | `0.087` | 아래로 숙인 각 [rad]. **세 값 중 오차에 가장 민감합니다.** |
| `camera_longitudinal_offset` | `0.113` | 차량 프레임 원점에서 카메라까지, **카메라가 보는 방향으로** 잰 거리 [m]. |
| `near` / `far` | `2.9` / `7.6` | BEV 맨 아래/맨 위 행이 보여줄 지면 거리 [m]. `near`를 카메라가 볼 수 있는 것보다 가깝게 잡으면 그 행들은 그냥 검게 남습니다 — 70도 렌즈를 5도만 숙여 단 이 카메라는 지면을 **2.83 m(sim)/2.58 m(실차)** 보다 가까이 볼 수 없어서 `2.9`가 그 한계 바로 바깥입니다. 근거리를 되찾으려면 카메라를 더 숙여 달아야 합니다. |
| `half_width` | `5.5` | 좌우로 각각 얼마나 넓게 볼지 [m]. 70도 렌즈가 거리 `d`에서 보는 폭이 `±d·tan(35도)`라, 가장 먼 행(7.6 m)의 `±5.32 m`가 이 카메라가 볼 수 있는 최대 폭입니다. |
| `meters_per_pixel` | `0.028125` | BEV 픽셀 하나가 덮는 지면 거리 [m/px]. 양 축 공통 — 이 값이 곧 발행되는 `offset_m`/`distance_m`의 환산 계수입니다. |

시작할 때 실제 만들어진 형상이 로그로 한 줄 남습니다(지면 범위, BEV 크기, 원점 위치,
지평선 행, 원본 프레임 밖으로 나간 꼭짓점 수). 장착 파라미터가 틀리면 RViz를 보기 전에 이 줄에서
먼저 드러납니다.

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
