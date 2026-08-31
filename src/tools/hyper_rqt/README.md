# hyper_rqt

HYPER 스택을 운전할 때 실제로 부르는 서비스만 버튼으로 모아 둔 rqt 패널입니다.

`rqt_service_caller`를 대신합니다. 그쪽 드롭다운에는 실행 중인 *모든* 서비스가
나옵니다 -- 노드 하나마다 파라미터 서비스가 6개
(`get_parameters`, `set_parameters`, `set_parameters_atomically`,
`describe_parameters`, `get_parameter_types`, `list_parameters`) 붙고 여기에
Gazebo / RViz / `robot_state_publisher` 내부 서비스까지 합치면 `simulation.launch.py`
기준으로 100개가 넘습니다. 정작 사람이 부르는 것은 6개뿐입니다.

## 실행

```bash
colcon build --packages-select hyper_rqt --symlink-install
source install/setup.bash

ros2 run hyper_rqt hyper_panel        # 패널만 단독으로
rqt                                    # 다른 rqt 플러그인과 같이 (Plugins -> HYPER -> HYPER Panel)
```

빌드 직후 첫 실행에서 `qt_gui_main() found no plugin matching ...`가 나오면 rqt의
플러그인 캐시가 아직 이 패키지를 모르는 것입니다. 한 번만:

```bash
rqt --force-discover
```

## 화면

- **맨 위 상태 표시** -- `/mission_manager/status`(`std_msgs/String`)를 그대로 보여
  줍니다. 내용에 따라 색이 바뀝니다(실패=빨강, blocked/wait=주황, idle/cancel=회색,
  finished=파랑, 그 외 진행 중=초록). 이 토픽은 `transient_local`이라 패널을 나중에
  띄워도 마지막 상태가 바로 뜹니다.
- **버튼** -- 누르면 서비스를 호출합니다. 서버가 없으면 회색으로 죽어 있어서
  "스택이 아직 안 떴다"가 바로 보입니다(1초 주기 확인).
- **콤보 박스(selects)** -- 요청 필드가 없는 `Trigger` 서비스에 "어떤 대상인지"를
  주는 방법입니다. 고르는 즉시 대상 노드의 파라미터를 `set_parameters`로 바꿉니다.
  `model_service`가 이미 쓰고 있는 방식 그대로입니다.
- **로그** -- 호출 결과(`success` / `message`)를 시각과 함께 쌓습니다.

## 버튼 추가하기

코드가 아니라 [`config/panel.yaml`](config/panel.yaml)을 고칩니다. 스키마 전체는
그 파일 맨 위 주석에 있습니다. 가장 단순한 형태:

```yaml
groups:
  - name: 내 그룹
    buttons:
      - label: 눌러
        service: /my_node/do_it     # type 생략 시 std_srvs/srv/Trigger
        style: go                   # go / stop / warn / 생략
```

요청 필드가 있는 서비스라면:

```yaml
      - label: 지도 바꾸기
        service: /map_server/load_map
        type: nav2_msgs/srv/LoadMap
        request: {map_url: /path/to/map.yaml}
```

`--symlink-install`로 빌드해 두었다면 YAML만 고치고 rqt를 다시 띄우면 반영됩니다.

다른 파일을 쓰려면 `ros2 run hyper_rqt hyper_panel --config /경로/panel.yaml`,
환경 변수 `HYPER_RQT_CONFIG`, 또는 패널 우상단 톱니바퀴(설정)로 고를 수 있습니다.
고른 경로는 rqt 설정에 저장되지만 `--config`가 있으면 그쪽이 이깁니다.
