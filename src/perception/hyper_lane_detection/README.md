# hyper_lane_detection

카메라 영상에서 차선과 정지선을 검출하는 OpenCV 기반 인지 패키지입니다. 입력 영상을 버드아이뷰로 변환해 주행에 필요한 차선 중심·정지선 정보를 발행합니다.

## 입출력

- 입력: `/camera/image_raw` 및 후방 카메라 영상
- 출력: `/lane/center`, `/stopline/detection`
- 후방 출력: `/lane/rear_center`, `/stopline/rear_detection`

## 실행

```bash
ros2 run hyper_lane_detection lane_detection_node
```

보통은 `hyper_object_detection`의 `perception.launch.py` 또는 `hyper_launch perception.launch.py`로 객체 인지 노드와 함께 실행합니다.
