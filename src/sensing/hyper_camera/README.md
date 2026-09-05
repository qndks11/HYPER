# hyper_camera

실차 카메라 드라이버 노드와 그 설정 파일의 배포처입니다. 소비하는 인지 노드(`lane_detection_node`, `object_detection_node`)는 카메라를 직접 열지 않고 이 드라이버가 발행하는 `sensor_msgs/Image` 토픽을 구독합니다.

차량 카메라는 **Logitech C920 한 대**입니다. 예전에는 차선 인식용 전방 ELP 어안 카메라와 객체 인식용 C920 두 대를 썼지만, 지금은 한 대가 두 인지 노드를 모두 먹여 살립니다 — ELP 드라이버·보정 파일(`ELP-USBGS1200P01-KL170.yaml`)·`params_elp.yaml`은 모두 제거되었습니다. 후방 RealSense D435i는 그 이전에 배터리 절약을 위해 실차·시뮬레이션 양쪽에서 제거되었습니다.

- `LogitechCameraPublisherNode` (C++ rclcpp 컴포넌트, 실행 파일 `logitech_camera_publisher_node`): `/dev/video_logitech`를 열어 MJPEG을 캡처·디코드하고 원본 프레임을 그대로 `image_raw`로 발행합니다. **rectify는 하지 않습니다** — C920은 어안이 아닌 일반 약 70도 렌즈이고 이 저장소에 보정 파일이 없습니다. `hyper_lane_detection`의 BEV 호모그래피는 이 카메라를 이상적인 핀홀로 모델링해서 세웁니다 (`hyper_lane_detection/config/bev_real.yaml` 참고).

이 노드는 `hyper_lane_detection`이 `input_backend:=intra_process`일 때 `lane_detection` 컴포넌트와 같은 `ComposableNodeContainer`에 함께 로드됩니다 — rclcpp의 intra-process 매니저가 발행한 `std::unique_ptr<Image>`를 직렬화 없이 그대로 구독 콜백에 넘겨줍니다. 같은 발행이 DDS로도 나가므로 별도 프로세스인 `object_detection_node`(rclpy, zero-copy 경로 없음)가 동일한 프레임을 일반 토픽으로 구독합니다. 컨테이너 구성은 `hyper_object_detection`의 `perception.launch.py`를 참고하세요.

**640x360@30**으로 엽니다. 센서 해상도(1280x720)의 절반이라 픽셀 수는 1/4이고, USB 대역폭·MJPEG 디코드 비용이 그만큼 줄어듭니다 — 이 카메라의 소비 전력을 좌우하는 지점입니다. 이 해상도는 이 패키지 바깥에도 영향을 줍니다: `hyper_lane_detection/config/bev_real.yaml`이 **가로 폭에서 초점거리를, 세로 높이에서 BEV 근거리 경계를** 유도하므로, 캡처 해상도를 바꾸면 그 파일도 같이 고쳐야 합니다.

- `config/params_logitech.yaml` — 위 값들을 문서화한 참고용 yaml (직접 로드되지는 않음 — 노드가 같은 값을 자체 파라미터 기본값으로 선언)

설치 방법(udev 심볼릭 링크 등)은 저장소 루트 README의 카메라 절을 참고하세요.
