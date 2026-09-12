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

이미지 수집만 할 때는 YOLO 노드를 빼고 띄웁니다. 카메라, `hyper_lane_detection`,
`image_saver_service`는 그대로 돌아가고 추론만 하지 않습니다:

```bash
ros2 launch hyper_object_detection perception.launch.py object_detection:=false
```

`/perception/sign`이 아예 발행되지 않으므로 `mission_manager`의 `wait_signal`, `branch` 스텝은
timeout까지 기다립니다. 미션 주행이 아니라 수집 주행에서만 쓰세요.

## `/perception/sign` 값

`hyper_planner`의 `mission_manager`가 구독합니다.

| 값 | 뜻 | 쓰는 곳 |
| --- | --- | --- |
| `red` | 빨간불 | -- |
| `green` | 초록불 | `wait_signal` 스텝 |
| `left_arrow` | 좌회전 화살표 | `wait_signal` (좌회전 신호등 `light_3`) |
| `ban` | 진입 금지 (차선 안내 표지 한 장) | -- |
| `allow` | 진입 허용 (차선 안내 표지 한 장) | -- |
| `allow_left` | **왼쪽** 차선이 허용 | `branch` 스텝 (마지막 차선 갈래) |
| `allow_right` | **오른쪽** 차선이 허용 | `branch` 스텝 (마지막 차선 갈래) |
| `none` | 유효한 신호 없음 | -- |

실제 미션에서 `branch`가 보는 값은 `allow_left` / `allow_right`입니다. `ban` / `allow`는 표지판 한
장만 읽혔을 때 중앙 선택으로 나가는 값이라 어느 `cases`에도 안 맞습니다(투표에서 세어지기는 하지만
이길 수 없습니다).

> `Blank`는 내부적으로 `blank`로 매핑되어 갈림길의 **꺼진 칸**을 채우지만, 토픽으로는 나가지
> 않습니다 -- 중앙 선택으로 갈 때 `none`으로 바꿔 냅니다. 즉 꺼진 신호등이 내는 값은 예전 그대로입니다.

### 갈림길 표지 읽기 (`allow_left` / `allow_right`)

갈림길에는 표지판이 **세 장** 나란히 서 있고 **깜빡입니다**. 그중 관심 있는 것은 **왼쪽 두 장**이고,
그 두 칸의 조합이 곧 답입니다. 한 칸이 꺼져 있어도(`blank`) 나머지 한 칸이 `allow`/`ban`이면 답이
정해지므로, 깜빡임이 판정을 막지 않습니다.

| 왼쪽 칸 | 오른쪽 칸 | 신호 |
| --- | --- | --- |
| `allow` | `ban` | `allow_left` |
| `allow` | `blank` | `allow_left` |
| `blank` | `ban` | `allow_left` |
| `ban` | `allow` | `allow_right` |
| `blank` | `allow` | `allow_right` |
| `ban` | `blank` | `allow_right` |
| 두 칸이 같은 값 | | 판정 없음 (중앙 선택으로 넘어감) |

후보 박스는 x축으로 정렬해 왼쪽부터 둘을 씁니다. 같은 표지판을 두 번 잡은 박스(NMS가 클래스별이라
한 장이 `Ban`과 `Blank`로 둘 다 살아남을 수 있습니다)는 위치로 합치고, **같은 줄에 나란히 선 두 장**만
짝으로 인정합니다 -- 꺼진 신호등도 `Blank`로 잡히는데 그것이 슬롯을 가로채면 보지도 않은 조합으로
차선이 정해지기 때문입니다. 후보가 둘이 안 되면 판정 없음입니다.

여기에는 중앙 50% 제한이 없습니다. 표지판은 나란히 붙어 있어 한쪽이 중앙 밖으로 밀리기 쉽고, 중요한
것은 화면 어디에 있느냐가 아니라 두 칸의 상대 위치이기 때문입니다.

**한계**: 1번 표지판이 화면 밖으로 벗어나면 왼쪽 두 칸이 사실은 2번과 3번이고, 그것을 알아챌 방법이
없습니다. 세 장이 다 화면에 들어오게 하는 것은 접근 각도와 `prearm_distance_m`의 몫입니다.

### 한 프레임에 여러 신호가 보이면

갈림길 판정이 서지 않은 프레임에서는 **화면 중앙 50% 안에서 중앙에 가장 가까운 것 하나**만 나갑니다
(같은 거리면 confidence가 높은 쪽). 신호등과 차선 안내 표지가 같이 보이면 둘이 번갈아 나갈 수 있는데,
그때는 `mission_manager`가 어느 쪽도 확정하지 못합니다 -- `wait_signal`은 "같은 값이 N프레임 연속"이라
끊기고, `branch`는 창 안의 다수결이라 이기는 값이 없습니다. 신호등 앞에서는 계속 서 있고, 갈림길에서는
timeout 뒤 `default` 갈래로 갑니다. 둘 다 안전한 쪽 실패입니다.

### YOLO 클래스 이름 맞추기 (`sign_class_map`)

현재 `models/best_track.pt`가 가진 클래스는 일곱입니다 -- `Allow`, `Ban`, `Blank`, `Go`,
`LeftTurn`, `Stop`, `Warn`. `models/best_sim.pt`는 `Blank`가 없는 나머지 여섯입니다.
`SIGNAL_MAP`이 그 일곱을 전부 덮습니다.

| YOLO 클래스 | 신호 값 |
| --- | --- |
| `Stop` | `red` |
| `Go` | `green` |
| `LeftTurn` | `left_arrow` |
| `Warn` (구 `Yellow`) | `none` |
| `Blank` (꺼진 칸: 꺼진 신호등 / 깜빡이는 중인 표지판) | `blank` (토픽으로는 `none`) |
| `Ban` | `ban` |
| `Allow` | `allow` |

황색등과 꺼진 등을 `none`으로 내보내는 것은 "무시"가 아니라 "통과 신호가 아니다"입니다. 매핑에서 빼면
그 박스가 중앙 선택에서 아예 제외되어 화면 가장자리의 다른 표지가 대신 뽑힐 수 있습니다.

`Blank`만 `none`이 아니라 `blank`로 매핑되는 이유는 갈림길의 꺼진 칸이 슬롯 하나를 차지해야 하기
때문입니다. 중앙 선택으로 나갈 때는 `none`으로 바꿔 내므로 신호등 쪽 동작은 예전과 같습니다.

**모델을 다시 학습해 클래스 이름이 바뀌면 그 신호는 무시됩니다.** 코드를 고치지 않고 맞추려면:

```bash
ros2 run hyper_object_detection object_detection_node --ros-args \
    -p sign_class_map:="['LaneBan:ban','LaneAllow:allow']"
```

매핑에 없는 클래스를 만나면 이름마다 한 번씩 경고가 납니다. 노드 시작 시 로그의
`YOLO model classes:`(모델이 실제로 가진 이름)와 `Sign class map:`(적용된 매핑)을 비교하세요.
