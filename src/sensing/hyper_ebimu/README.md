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
확인해야 합니다. `ekf_global`이 이 센서의 yaw를 map 프레임 절대 방위로 그대로 먹기 때문에
(`dual_ekf_navsat.yaml`의 `imu0_config` 인덱스 5 = true) 여기가 어긋나면 추정 heading이
통째로 돌아갑니다.

보정이 필요하면 `hyper_localization`의 `imu_enu_relay.py`를 씁니다 — 이전 IMU를 ENU로
고치던 relay라 지금은 꺼져 있지만(`odometry.launch.py`에서 주석 처리),
`config/ebimu.yaml`의 `topic`을 `imu/raw`로 바꾸고 relay를 되살리면 그대로 재사용할 수
있습니다(`out_yaw = yaw_sign * in_yaw + yaw_offset_rad`, 자이로 z 부호 반전 포함).

## 실행

```bash
ros2 launch hyper_ebimu ebimu.launch.py
```

포트, 보드레이트, 출력 주기, 토픽명, covariance는 `config/ebimu.yaml`에서 설정합니다.
포트 기본값 `/dev/tty_ebimu`는 저장소의 `udev/99-hyper-serial.rules`가 만드는 고정
심볼릭 링크입니다(루트 README의 "USB 시리얼 포트 고정" 절).

covariance 기본값은 검증되지 않은 자리표시자입니다. 특히 `orientation_covariance[8]`
(yaw, 현재 0.02 rad^2 ≈ 8deg)은 `ekf_global`이 지자기 yaw를 얼마나 믿을지를 그대로
결정하므로, 실차 데이터(정지 상태 yaw 분산, 모터 근처 자기 간섭)를 보고 다시 잡아야
합니다.

## 주의

- 센서가 뽑히거나 오류가 나면 자동으로 재연결을 시도합니다(`reconnect_wait_seconds`).
- 지자기센서는 끄지 않았습니다(기본값 유지, 능동형 ON) — yaw는 여전히 지자기 융합값입니다.
  차량/모터 근처 자기장 간섭이 크면 스펙 6-2-1절의 `sem` 명령으로 끄는 걸 검토하세요(단,
  끄면 yaw가 자이로 적분만으로 시간이 지날수록 드리프트합니다).
