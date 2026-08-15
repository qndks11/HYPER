# hyper_camera

실차의 두 USB 카메라를 직접 여는 드라이버 노드와 그 설정 파일의 배포처입니다. `usb_cam`은 이 저장소에서 완전히 제거되었고, 각 카메라는 이 패키지의 전용 노드가 엽니다 — 소비하는 인지 노드(`lane_detection_node`, `object_detection_node`)는 더 이상 카메라를 직접 열지 않고 이 노드들이 발행하는 `sensor_msgs/Image` 토픽을 구독합니다.

- 전방 ELP (`ElpCameraPublisherNode`, C++ rclcpp 컴포넌트, 실행 파일 `elp_camera_publisher_node`): `/dev/video_elp`를 열어 MJPEG을 캡처·디코드하고, `config/ELP-USBGS1200P01-KL170.yaml` 보정값으로 자체 rectify까지 수행한 뒤 `image_raw`로 발행합니다. `hyper_lane_detection`이 `input_backend:=intra_process`일 때 이 노드와 `lane_detection` 컴포넌트를 같은 `ComposableNodeContainer`에 함께 로드합니다 — rclcpp의 intra-process 매니저가 발행한 `std::unique_ptr<Image>`를 직렬화 없이 그대로 구독 콜백에 넘겨줍니다 (`hyper_object_detection`의 `perception.launch.py` 참고).
- 객체 인식용 Logitech C920 (`logitech_camera_publisher_node.py`, rclpy 노드): `/dev/video_logitech`를 열어 원본 프레임을 그대로 `image_raw`로 발행합니다 (rectify 없음, 이 카메라는 애초에 캘리브레이션 파일이 없음). `object_detection_node`가 `object_input_backend:=usb_camera`일 때 이 노드가 함께 실행됩니다. rclpy에는 rclcpp의 intra-process 통신에 해당하는 zero-copy 경로가 없으므로, 이 쪽은 일반 ROS 토픽으로만 연결됩니다.

이 패키지는 다음 파일들의 배포처이기도 합니다.

- `config/ELP-USBGS1200P01-KL170.yaml` — `ElpCameraPublisherNode`가 런타임에 참조하는 ELP 카메라 보정 파일
- `config/params_elp.yaml`, `config/params_logitech.yaml` — 각 카메라의 기본 장치 경로·해상도·프레임레이트를 문서화한 참고용 yaml (직접 로드되지는 않음 — 두 노드가 같은 값을 자체 파라미터 기본값으로 선언)

설치 방법(udev 심볼릭 링크 등)은 저장소 루트 README의 카메라 절을 참고하세요.
