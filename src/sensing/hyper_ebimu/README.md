# hyper_ebimu

E2BOX **EBIMU-9DOFV5** AHRS 모듈용 ROS 2 드라이버입니다. 이 센서는 ROS 드라이버가
따로 없어서, 스펙시트(`EBIMU-9DOFV5_rev3.pdf`)에 정의된 UART/ASCII 프로토콜을 직접
파싱합니다. 모듈 자체엔 USB가 없고 `VCC/GND/TX/RX` 4핀만 있으므로, USB-UART 어댑터
(CP210x/FTDI/CH340 등)로 PC/차량 컴퓨터에 연결해서 씁니다.

## 배선

| EBIMU | 어댑터 |
|---|---|
| TX | RX |
| RX | TX |
| VCC | 3.3V 또는 5V (어댑터 로직 레벨에 맞춰서) |
| GND | GND |
| nPD/nRST | 미연결 |

## 동작 방식

노드가 켜지면 이전 세션이나 공장 출하 설정이 뭐였든 상관없이, 매번 센서를 아래 설정으로
강제로 맞춥니다(`<sor..>`, `<soc1>`, `<sof2>`, `<sog1>`, `<soa1>`, `<som0>`, `<sod0>`,
`<sot0>`, `<sots0>`):

- ASCII 출력 모드
- 자세는 **Quaternion**으로 출력 (Euler 대신 — gimbal-lock/wrap 문제가 없고, 어차피
  `sensor_msgs/Imu`도 quaternion을 원함)
- 자이로(각속도) ON, 가속도(중력 포함) ON
- 지자기/거리/온도/타임스탬프는 OFF

즉 매 줄은 `*qz,qy,qx,qw,gx,gy,gz,ax,ay,az` 10개 필드로 고정됩니다. 이 순서는 스펙
5-1절의 `sof,sog,soa,som,sod,sot,sots` 명령 순서를 그대로 따릅니다 — 지자기/온도 등을
추가로 쓰려면 `ebimu_node.py`의 `_SETUP_COMMANDS`와 `_handle_line()`을 같이 고쳐야
합니다.

**Quaternion 필드 순서가 `[z][y][x][w]`** 라는 점(스펙 5-1/6-1-4절) 외에는 축을 별도로
재매핑하지 않습니다 — 센서 자체 body frame 그대로 퍼블리시합니다. 이게 차량의
ENU/body_link 관례(특히 yaw 부호, angular_velocity.z 부호)와 맞는지는 실차에서 직접
확인해야 합니다. 이 플랫폼의 다른 IMU(WitMotion WT901BLE)에 대해 정확히 이 보정을 하는
`hyper_localization`의 `imu_enu_relay.py`가 참고할 패턴입니다 — 이 센서가 그걸 대체하거나
같이 쓰이게 되면 동일하게 적용하세요.

## 실행

```bash
ros2 launch hyper_ebimu ebimu.launch.py
```

포트, 보드레이트, 출력 주기, 토픽명, covariance는 `config/ebimu.yaml`에서 설정합니다.
covariance 기본값은 검증되지 않은 자리표시자이며, `sensors.launch.py`가 WitMotion의
`orientation_covariance`를 EKF 융합용으로 튜닝하는 것과 같은 방식으로 실차 데이터를 보고
다시 잡아야 합니다.

## 주의

- 센서가 뽑히거나 오류가 나면 자동으로 재연결을 시도합니다(`reconnect_wait_seconds`).
- 지자기센서는 끄지 않았습니다(기본값 유지, 능동형 ON) — yaw는 여전히 지자기 융합값입니다.
  차량/모터 근처 자기장 간섭이 크면 스펙 6-2-1절의 `sem` 명령으로 끄는 걸 검토하세요(단,
  끄면 yaw가 자이로 적분만으로 시간이 지날수록 드리프트합니다).
