# hyper_camera

실차 USB 카메라 설정 파일의 배포처입니다. 더 이상 노드나 launch 파일을 갖지 않습니다 — `usb_cam`은 이 저장소에서 완전히 제거되었고, 두 카메라 모두 이를 소비하는 노드가 `/dev/videoN`을 직접 엽니다.

- 전방 ELP: `hyper_lane_detection`이 `input_backend:=direct_usb`일 때 `/dev/video_elp`를 직접 열고, 이 패키지의 `config/ELP-USBGS1200P01-KL170.yaml`을 보정 파일로 참조해 자체 rectify까지 수행합니다.
- 객체 인식용 Logitech C920: `object_detection_node`가 `object_input_backend:=direct_usb`일 때 `/dev/video_logitech`를 직접 열어 원본 프레임을 그대로 YOLO에 넣습니다 (rectify 없음, 이 카메라는 애초에 캘리브레이션 파일이 없음). 카메라 파라미터(`video_device`, `image_width`, `image_height`, `framerate`)는 `object_detection_node` 자체가 선언하며, 기본값은 이 패키지의 `config/params_logitech.yaml`과 동일합니다.

이 패키지는 다음 파일들의 배포처로만 남아 있습니다.

- `config/ELP-USBGS1200P01-KL170.yaml` — `hyper_lane_detection`의 `input_backend:=direct_usb`가 런타임에 참조하는 ELP 카메라 보정 파일
- `config/params_elp.yaml`, `config/params_logitech.yaml` — 각 카메라의 기본 장치 경로·해상도·프레임레이트를 문서화한 참고용 yaml (직접 로드되지는 않음 — `lane_detection_node`/`object_detection_node`가 같은 값을 자체 파라미터 기본값으로 선언)

설치 방법(udev 심볼릭 링크 등)은 저장소 루트 README의 카메라 절을 참고하세요.
