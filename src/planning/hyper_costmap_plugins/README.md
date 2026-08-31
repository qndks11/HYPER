# hyper_costmap_plugins

nav2 `costmap_2d` 레이어 플러그인 모음. 현재는 `DrivableAreaLayer` 하나입니다.

## DrivableAreaLayer

`hyper_lane_detection`이 발행하는 주행가능영역 분류(`/lane/drivable_area`,
`nav_msgs/OccupancyGrid`, `body_link` 기준)를 로컬 코스트맵에 반영합니다.

### 왜 obstacle_layer의 observation_source로 넣으면 안 되는가

`nav2_controller.yaml`의 `obstacle_layer`는 `scan`을 `clearing: true` /
`raytrace_max_range: 20.0`으로 돌립니다. 라이다는 평평한 지면 위를 그대로 지나가므로, 잔디를
가로지르는 모든 빔이 카메라가 방금 마킹한 셀을 **같은 사이클에** 다시 지웁니다. 점을 장애물
높이(0.3 m 등)로 띄워도 문제는 자리만 옮깁니다 -- 그 높이에는 실제로 아무것도 없으니 라이다가
정당하게 지웁니다. 지면 위의 페인트·잔디는 장애물이 아니고, 그걸 다루는 레이어가 장애물용
레이어일 수는 없습니다.

두 레이어는 서로 겹치지 않는 실패 모드를 덮습니다: 라이다는 높이가 있는 것(콘, 차량, 사람)을,
이 레이어는 높이가 없는 것(잔디, 런오프, 연석 너머)을 봅니다.

### 신뢰도 누적기

레이어는 "마지막 프레임이 말한 것"을 쓰지 않고 자체 코스트맵을 **신뢰도 누적기**로 굴립니다.

| 관측 | 동작 |
| --- | --- |
| `occupancy >= mark_threshold` | `cost += mark_step`, 위 두 단계 중 해당 코스트에서 포화 |
| `occupancy < mark_threshold` (0 이상) | `cost -= clear_step`, FREE에서 멈춤 |
| `occupancy < 0` (데이터 없음) | 아무것도 안 함 |
| 매 사이클, 관측 여부 무관 | `cost -= decay_step` |

기본값(mark 60 / clear 120 / decay 10, cost_value 254)이면 마킹에 5프레임, 클리어에 2프레임,
관측이 끊긴 셀은 약 4초에 걸쳐 사라집니다. 한 프레임의 오분류(글레어, 그림자 경계, 보간된 원거리
행)로는 차 앞에 단단한 셀이 생기지 않고, 차 뒤로 지나가 다시 관측될 일이 없는 셀은 한 바퀴 내내
남지 않습니다.

누적기는 **코스트맵 자신의 프레임**(rolling `odom`)에 삽니다. 차량 프레임에서 시간 필터를 걸면
2.22 m/s에서 프레임당 1.5셀씩 번지므로, 필터는 지면 셀이 가만히 있는 프레임에서 걸어야 합니다.
`updateBounds()`가 매 사이클 `updateOrigin()`으로 창을 따라갑니다.

### 두 단계: 벽과 선

`hyper_lane_detection`은 "가면 안 되는 곳"을 두 값으로 내보내고, 이 레이어는 둘을 구분합니다.
같은 금지가 아니기 때문입니다.

| 관측 | 정체 | 포화 코스트 | 충돌? | 팽창? |
| --- | --- | --- | --- | --- |
| `>= lethal_threshold`(100, `kUndrivable`) | 노면이 아닌 지면 -- 잔디, 흙, 런오프 | `cost_value` = **254** | O | O |
| `>= mark_threshold`(50, `kOffLimits`) | 노면이지만 쓰면 안 되는 곳 -- 차선 페인트, 그 너머 반대 차선 | `off_limits_cost` = **200** | X | X |

**254(코스 밖)** 에서만 켜지는 것 두 가지:

1. **충돌 판정.** MPPI `ObstaclesCritic`은 `consider_footprint: true`에서 footprint가 254 셀을
   덮을 때만 충돌로 칩니다(253은 아님). 코스를 벗어나는 궤적은 벌점이 아니라 폐기됩니다.
2. **팽창.** `InflationLayer`는 LETHAL 셀에서만 전파합니다.

**200(차선)** 은 253(`INSCRIBED_INFLATED_OBSTACLE`) 미만이라 둘 다 안 걸립니다. 의도한
것입니다 -- 선을 넘는 것은 규칙 위반이지 충돌이 아니고, 중앙선에 `inflation_radius`(1.0) 팽창이
붙으면 정작 주행 중인 차로가 통째로 막힙니다. 회피는 MPPI가 non-lethal 코스트를 거리로 역산해서
냅니다:

```
d = inscribed_radius + ln(253 / cost) / cost_scaling_factor
  = 0.405 + ln(253/200) / 3.0  =  약 0.48 m
```

즉 차선은 "0.48 m 앞의 장애물"만큼 밀어내고, 필요하면 넘을 수 있습니다. 더 넓게 돌게 하려면
`off_limits_cost`를 *내리세요*(100 -> 약 0.71 m). 올리면 붙습니다.

누적기 덕분에 254는 같은 셀을 **5프레임 연속** 관측해야(60 x 5 > 254) 나옵니다. 올라가는 중의
단계(60/120/180/240)는 전부 sub-lethal이라 충돌도 팽창도 아닙니다. 한 프레임의 오분류(글레어,
그림자 경계, 보간된 원거리 행)로 차 앞에 벽이 서지는 않습니다. 반대로 254까지 갔던 셀이
kOffLimits로 관측되면 그 자리에서 200으로 내려옵니다 -- 강한 주장은 빨리 놓는 쪽입니다.

**대가:** 좁은 구간에서 1500개 궤적이 전부 코스를 벗어나면 MPPI가 해를 못 내고
`controller_server`가 `failure_tolerance`(1.0초) 뒤 abort 합니다. 코스 폭이 빠듯하거나
오분류로 차가 서면 `cost_value`도 200으로 낮추세요 -- 코스 이탈도 충돌이 아니라 강한 선호가
됩니다.

### 파라미터

`local_costmap`의 `drivable_area_layer` 아래에 둡니다 --
`hyper_planner/config/nav2_controller.yaml` 참고.

| 파라미터 | 기본값 | 설명 |
| --- | --- | --- |
| `enabled` | `true` | |
| `topic` | `/lane/drivable_area` | |
| `cost_value` | `254` | 코스 밖 지면이 포화하는 코스트(LETHAL). 위 참고 |
| `off_limits_cost` | `200` | 차선/반대 차선이 포화하는 코스트. 253 미만으로 강제됩니다 |
| `mark_threshold` | `50` | 이 값 이상이면 마킹, 미만이면 클리어, 음수는 무시 |
| `lethal_threshold` | `100` | 이 값 이상이면 `cost_value`, 미만이면 `off_limits_cost` |
| `mark_step` / `clear_step` / `decay_step` | `60` / `120` / `10` | `decay_step < mark_step`이어야 포화 가능 |
| `message_timeout` | `1.0` | 이보다 오래된 그리드는 재반영하지 않음 [s] |
| `transform_tolerance` | `0.2` | tf 조회 여유 [s] |

### 레이어 순서

`plugins: ["obstacle_layer", "drivable_area_layer", "inflation_layer"]`.

`obstacle_layer` 뒤(비용을 올리기만 하므로 손해 없음), `inflation_layer` 앞(`cost_value`가
254이므로 팽창을 받으려면 반드시 앞이어야 함).

### 발행자가 없어도 안전합니다

`/lane/drivable_area`가 한 번도 안 들어오면 레이어는 아무것도 하지 않고 `isCurrent()`도 `true`로
보고합니다 -- 꺼져 있는 기능이 코스트맵 전체를 stale로 만들면 안 되기 때문입니다. 실제 스위치는
perception 스테이지의 `drivable_area` 인자입니다:

```bash
ros2 launch hyper_launch simulation.launch.py drivable_area:=true
```

한 번 들어오다가 끊긴 경우에만 not-current로 보고합니다.
