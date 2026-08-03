# hyper_camera

실차 USB 카메라를 ROS 2 영상 토픽으로 제공하는 패키지입니다. `usb_cam` 드라이버와 이미지 처리 컴포넌트를 같은 컨테이너에서 실행해 카메라 영상을 토픽으로 전달합니다.

**차선 인식 경로에서는 더 이상 기본으로 쓰이지 않습니다.** `hyper_lane_detection`이 `input_backend:=direct_usb`일 때 `/dev/video_elp`를 직접 열고 이 패키지의 보정 파일(`config/ELP-USBGS1200P01-KL170.yaml`)로 자체 rectify까지 수행하므로, 실차 launch(`hyper_launch real.launch.py` → `sensors.launch.py`)는 전방 카메라용으로 이 패키지를 더 이상 띄우지 않습니다. 이 패키지는 다음 용도로 계속 남아 있습니다.

- `hyper_lane_detection`의 `input_backend:=direct_usb`가 런타임에 참조하는 보정 파일(`config/ELP-USBGS1200P01-KL170.yaml`)의 배포처
- 롤백용 `input_backend:=ros_compressed` 경로를 테스트할 때, 또는 이 카메라 영상을 다른 용도로 ROS 토픽으로 띄워야 할 때 독립 실행

## 실행

```bash
ros2 launch hyper_camera camera.launch.py
```

카메라 장치와 해상도·픽셀 포맷은 `config/ELP-USBGS1200P01-KL170.yaml` 또는 `config/params_1.yaml`에서 장비에 맞게 조정합니다.

`params_1.yaml`의 `video_device`는 `/dev/video_elp`(udev 심볼릭 링크)를 기본값으로 사용합니다. 설치 방법은 저장소 루트 README의 카메라 절을 참고하세요.
