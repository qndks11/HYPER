# hyper_object_detection

YOLO 모델로 카메라 영상의 신호등과 표지를 인식하는 Python 패키지입니다. 인식 결과를 주행 행동 결정에 사용할 수 있도록 문자열 토픽으로 발행합니다.

## 구성

- `object_detection_node`: 영상 입력을 추론하고 `/perception/sign`에 결과를 발행합니다.
- `models/best.pt`: 추론에 사용하는 학습 모델입니다.
- `perception.launch.py`: 이 노드와 `hyper_lane_detection`을 함께 실행합니다.

## 실행

```bash
ros2 launch hyper_object_detection perception.launch.py
```

## `/perception/sign` 값

`hyper_planner`의 `mission_manager`가 구독합니다.

| 값 | 뜻 | 쓰는 곳 |
| --- | --- | --- |
| `red` | 빨간불 | -- |
| `green` | 초록불 | `wait_signal` 스텝 |
| `left_arrow` | 좌회전 화살표 | `wait_signal` (좌회전 신호등 `light_3`) |
| `ban` | 진입 금지 (차선 안내 표지) | `branch` 스텝 |
| `allow` | 진입 허용 (차선 안내 표지) | `branch` 스텝 |
| `none` | 유효한 신호 없음 | -- |

한 프레임에 여러 신호가 보이면 **화면 중앙 50% 안에서 중앙에 가장 가까운 것 하나**만 나갑니다
(같은 거리면 confidence가 높은 쪽). 신호등과 차선 안내 표지가 같이 보이면 둘이 번갈아 나갈 수
있는데, `mission_manager`의 판정이 "같은 값이 N프레임 연속"이라 그때는 어느 쪽도 확정되지 않습니다
-- 신호등 앞에서는 계속 서 있고, 갈림길에서는 timeout 뒤 `default` 갈래로 갑니다. 둘 다 안전한 쪽
실패입니다.

### YOLO 클래스 이름 맞추기 (`sign_class_map`)

현재 `models/best.pt`가 가진 클래스는 여섯입니다 -- `Allow`, `Ban`, `Go`, `LeftTurn`, `Stop`,
`Warn`. `SIGNAL_MAP`이 그 여섯을 전부 덮습니다.

| YOLO 클래스 | 신호 값 |
| --- | --- |
| `Stop` | `red` |
| `Go` | `green` |
| `LeftTurn` | `left_arrow` |
| `Warn` (구 `Yellow`) | `none` |
| `Ban` | `ban` |
| `Allow` | `allow` |

황색등을 `none`으로 두는 것은 "무시"가 아니라 "통과 신호가 아니다"입니다. 매핑에서 빼면 그 박스가
중앙 선택에서 아예 제외되어 화면 가장자리의 다른 표지가 대신 뽑힐 수 있습니다.

**모델을 다시 학습해 클래스 이름이 바뀌면 그 신호는 무시됩니다.** 코드를 고치지 않고 맞추려면:

```bash
ros2 run hyper_object_detection object_detection_node --ros-args \
    -p sign_class_map:="['LaneBan:ban','LaneAllow:allow']"
```

매핑에 없는 클래스를 만나면 이름마다 한 번씩 경고가 납니다. 노드 시작 시 로그의
`YOLO model classes:`(모델이 실제로 가진 이름)와 `Sign class map:`(적용된 매핑)을 비교하세요.
