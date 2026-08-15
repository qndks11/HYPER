# hyper_lane_detection

카메라 영상에서 차선과 정지선을 검출하는 OpenCV 기반 인지 패키지입니다. 입력 영상을 버드아이뷰로 변환해 주행에 필요한 차선 중심·정지선 정보를 발행합니다. 차선/정지선 검출 알고리즘 자체는 `process_frame()` 한 곳에 모여 있고, 아래 두 입력 백엔드 모두 이 함수와 하나의 `sensor_msgs/Image` 구독 콜백(`raw_image_callback()`)을 그대로 공유합니다 — 차이는 그 토픽을 누가 발행하는지, 그리고 (ROI 튜닝을 위해) 어느 카메라 모델로 취급하는지뿐입니다.

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

## 실행

```bash
ros2 run hyper_lane_detection lane_detection_node --ros-args -p input_backend:=ros_raw
```

`intra_process`는 `ComposableNodeContainer`로만 의미가 있으므로(zero-copy는 같은 프로세스일 때만 성립), 단독 `ros2 run`이 아니라 `hyper_object_detection`의 `perception.launch.py`(또는 `hyper_launch perception.launch.py`, `lane_input_backend` 인자로 전달)로 실행합니다. `hyper_launch`의 `real.launch.py`는 `intra_process`를, `simulation.launch.py`는 `ros_raw`를 기본으로 넘깁니다.
