# hyper_camera

실차 USB 카메라를 ROS 2 영상 토픽으로 제공하는 패키지입니다. `usb_cam` 드라이버와 이미지 처리 컴포넌트를 같은 컨테이너에서 실행해 인지 패키지에 카메라 영상을 전달합니다.

## 실행

```bash
ros2 launch hyper_camera camera.launch.py
```

카메라 장치와 해상도·픽셀 포맷은 `config/ELP-USBGS1200P01-KL170.yaml` 또는 `config/params_1.yaml`에서 장비에 맞게 조정합니다.
