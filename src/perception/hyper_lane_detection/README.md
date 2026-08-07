# hyper_lane_detection

카메라 영상에서 차선과 정지선을 검출하는 OpenCV 기반 인지 패키지입니다. 입력 영상을 버드아이뷰로 변환해 주행에 필요한 차선 중심·정지선 정보를 발행합니다. 차선/정지선 검출 알고리즘 자체는 `process_frame()` 한 곳에 모여 있고, 아래 두 입력 백엔드 모두 이 함수를 그대로 공유합니다.

## input_backend 파라미터

카메라 영상이 어디서 들어오는지는 `input_backend` 파라미터로 고릅니다.

- `direct_usb` — 실차용. `lane_detection_node`가 `/dev/video_elp`(ELP USB 카메라)를 직접 열어 MJPEG을 캡처·디코드하고, `hyper_camera/config/ELP-USBGS1200P01-KL170.yaml` 보정값으로 자체 rectify까지 수행합니다. 중간 ROS 이미지 토픽을 전혀 쓰지 않습니다. 후방 카메라 경로는 없습니다(실물이 없어도 실패하지 않음).
- `ros_raw` — 시뮬레이션(Gazebo)용. `/camera/image_raw`, `/camera_rear/image_raw`를 평범한 `sensor_msgs/Image`로 구독합니다. 시뮬레이션 카메라 영상은 이미 보정된 입력으로 취급하므로 rectify하지 않습니다.

`direct_usb` 전용 파라미터: `video_device`(기본 `/dev/video_elp`), `image_width`/`image_height`(기본 1280x720), `framerate`(기본 30.0), `calibration_file`(기본 `hyper_camera`의 `ELP-USBGS1200P01-KL170.yaml`).

## 입출력

- 입력: `input_backend`에 따라 다름 (위 참고)
- 출력: `/lane/center`, `/stopline/detection`
- 후방 출력: `/lane/rear_center`, `/stopline/rear_detection` (`direct_usb`에서는 발행되지 않음)

## 실행

```bash
ros2 run hyper_lane_detection lane_detection_node --ros-args -p input_backend:=direct_usb
```

보통은 `hyper_object_detection`의 `perception.launch.py` (또는 `hyper_launch perception.launch.py`, `lane_input_backend` 인자로 전달)로 객체 인지 노드와 함께 실행합니다. `hyper_launch`의 `real.launch.py`는 `direct_usb`를, `simulation.launch.py`는 `ros_raw`를 기본으로 넘깁니다.
