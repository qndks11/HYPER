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
- 디버그 영상: `/lane/bev/image_raw`, `/lane/rear_bev/image_raw` (`sensor_msgs/Image`, BGR8)
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
| `bev_cloud_z_m` | `0.02` | 지면에서 띄울 높이. RViz가 코스트맵도 z=0에 그리므로 z-fighting 방지용입니다. |

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
