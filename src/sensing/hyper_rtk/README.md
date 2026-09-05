# hyper_rtk

u-blox **ZED-F9P-01B GNSS RTK 보드** 2대(base + rover)와 NTRIP 클라이언트를 함께 실행하는 패키지입니다. `ublox_gps`가 두 보드에서 GNSS 위치/헤딩을 읽고, `ntrip_client`가 RTCM 보정 데이터를 수신해 base에 전달합니다.

- **base** (뒤쪽 안테나): NTRIP RTK 보정으로 절대 위치(`/gps/fix`)를 낸다. 동시에 UART2로 RTCM3(+u-blox 전용 4072.0 메시지)를 rover에 relay한다.
- **rover** (앞쪽 안테나): UART2로 base의 보정을 받아 두 안테나 사이의 상대위치(NAV-RELPOSNED9)를 풀고, 그걸로 얻은 절대 헤딩을 `sensor_msgs/Imu`(yaw만 유효)로 `imu/heading`에 낸다 — **moving-base RTK 헤딩**. `hyper_localization`의 `ekf_global`이 이 헤딩을 절대 방위로 먹는다(EBIMU 지자기 yaw 대신, 정지 상태에서도 유효하고 자기 교란에 안 흔들림).

## 사용 하드웨어

- GNSS RTK 보드: u-blox ZED-F9P-01B (Ardusimple simpleRTK2B) x2
- 연결 장치: `/dev/tty_ublox_base`, `/dev/tty_ublox_rover` (udev 심볼릭 링크, launch 기본값)

## 배선

- 안테나 2개를 차량 세로축(진행방향)에 맞춰 강체로 고정. baseline이 길수록 헤딩이 정확하다(대략 1 m에서 ±0.6°, 2 m에서 ±0.3° RMS).
- 관례상 **뒤 = base, 앞 = rover** — `relPosHeading`은 base→rover 방향 벡터의 방위각이라, 이렇게 두면 별도 180° 보정 없이 그대로 차량 전방 헤딩이 된다.
- 두 보드 모두 USB로 차량 PC에 연결(전원 겸용).
- **UART2 링크(보정 relay, 필수)**: base의 `TXD2` → rover의 `RXD2`, 그리고 `GND`↔`GND` 공통. 한쪽 방향만 있으면 된다(base가 쏘기만, rover가 받기만).

## u-center 사전 설정 (보드당 1회, 플래시에 저장)

`ublox_gps` 드라이버는 moving base(TMODE3 Disabled) 경로에서 RTCM 메시지 구성을 자동으로 밀어주지 않으므로, u-center로 미리 잡아 보드 플래시에 저장해 둬야 한다.

두 보드 공통으로 먼저 맞춰야 하는 값:
- `CFG-RATE`: Measurement period **1000 ms** / Navigation rate **1 cycle** → 정확히 1 Hz. 이보다 빠르면 moving-base 상대위치 엔진이 아예 안 풀린다(RTCM은 정상 수신되는데 `relPosValid`/`headValid`가 영영 안 서는 걸로 나타남 — u-center 기본값을 그대로 두면 되고, ROS 쪽 launch 파라미터(`rate: 1.0`)도 이미 이 값과 맞춰져 있다).
- `CFG-PRT` (Target: **UART2**) → Baudrate **460800** — base/rover 양쪽 다 반드시 동일해야 한다(하나라도 다르면 바이트가 안 맞아 UART2 링크가 통째로 조용해진다).

**base**
- `CFG-TMODE3` mode = **Disabled** (moving base는 고정좌표 기준국이 아님)
- UART2 → Protocol in = None, Protocol out = **RTCM3**
- `CFG-MSG`에서 UART2 열에 rate=1로: `1077`/`1087`/`1097`/`1127`(MSM7, GPS/GLONASS/Galileo/BeiDou), `1230`(GLONASS code-phase biases), 그리고 **`4072.0`**(u-blox 전용 Reference Station PVT — moving base 헤딩에 필수, 놓치기 쉬움). 다른 포트(USB/UART1/I2C/SPI) 열은 0으로 둬서 안 섞이게 한다.
- 마지막으로 아래 "저장(Save)하기" 절차로 **Flash까지** 저장

**rover**
- `CFG-TMODE3` mode = **Disabled** (base와 동일)
- UART2 → Protocol in = **RTCM3**(+UBX), Protocol out = None
- `UBX-NAV-RELPOSNED` 출력을 ROS가 붙는 포트(USB)에 rate=1로 활성화 (Messages View > UBX > NAV > RELPOSNED)
- 마지막으로 아래 "저장(Save)하기" 절차로 **Flash까지** 저장

### 저장(Save)하기 — u-center 화면 조작 순서

위 값들은 다 잡아도 **그 자체로는 휘발성**이다 — 지금 켜져 있는 동안만 적용되고, `UBX-CFG-CFG`로 명시적으로 저장해야 재부팅 후에도 남는다. 값을 다 잡은 **뒤에**, 보드 1대당 한 번씩:

1. 메뉴 **View → Messages View** (또는 툴바의 메시지 목록 아이콘, 단축키는 버전마다 다름) 를 연다.
2. 왼쪽 트리에서 **UBX → CFG → CFG** 를 클릭한다. 오른쪽에 **Clear / Save / Load** 세 열과, 맨 아래 **Devices**(BBR / Flash / I2C-EEPROM / SPI Flash) 체크박스가 있는 패널이 뜬다.
3. **Save** 열의 항목을 전부 체크한다 — `ioPort`, `msgConf`, `infMsg`, `navConf`, `rxmConf`, `senConf`, `rinvConf`, `antConf`, `logConf` 등. ("Select All" 같은 일괄 체크 버튼이 있으면 그걸 눌러도 된다.) 방금 바꾼 게 `CFG-PRT`(ioPort)/`CFG-MSG`(msgConf)/`CFG-RATE`·`CFG-TMODE3`(navConf)라 이 세 개만 체크해도 되지만, 헷갈리지 않게 전부 체크해서 한 번에 다 저장하는 걸 권장한다.
4. **Clear**와 **Load** 열은 **아무것도 체크하지 않는다** — 여기 체크한 채로 Send를 누르면 저장이 아니라 설정을 지우거나(Clear) 저장된 옛날 값을 다시 불러오는(Load) 동작이 같이 실행된다.
5. 맨 아래 **Devices**에서 **BBR**과 **Flash**를 둘 다 체크한다. (Ardusimple 보드엔 I2C-EEPROM/SPI Flash가 없으니 그 두 개는 그대로 둔다 — 체크해도 무시되지만 헷갈릴 필요 없다.) **Flash를 빠뜨리면 안 된다** — 이 보드는 백업 배터리가 없어서 BBR만으로는 USB 전원이 끊기는 순간(케이블을 뽑거나, 다른 컴퓨터로 옮기거나) 설정이 통째로 사라진다.
6. 패널 하단의 **Send** 버튼을 누른다.
7. 화면 아래 메시지 로그(또는 Packet Console)에 `UBX-ACK-ACK`(클래스/ID `06-09`, 즉 CFG-CFG에 대한 ACK)가 찍히는지 확인한다. `UBX-ACK-NAK`이 뜨면 저장이 안 된 것이니 Devices 체크가 맞는지 다시 본다.
8. **실제로 저장됐는지 반드시 재부팅으로 확인한다**: USB 케이블을 뽑았다가 다시 꽂아 보드를 완전히 재부팅한 뒤, u-center를 재연결하고 `UBX → CFG → PRT`(Target: UART2)와 `UBX → CFG → MSG`, `UBX → CFG → RATE` 패널을 다시 열어본다. u-center는 패널을 열 때 현재 값을 자동으로 poll해서 보여주므로, 방금 잡았던 값(460800 baud, RTCM3 프로토콜, 메시지 rate=1, 1 Hz 등)이 그대로 남아 있으면 Flash 저장이 된 것이다. 기본값으로 돌아가 있으면 5~7번을 다시 한다.

base/rover 각각 이 절차를 따로 수행해야 한다 — 한쪽만 저장하고 다른 쪽을 빠뜨리면 그쪽만 재부팅 후 설정이 날아간다.

배선/설정 후 u-center에서 `flags.gnssFixOK`/`diffSoln`/`relPosValid`가 서고 `carrSoln`이 float→fixed로 수렴하는지, `relPosHeading`이 차량을 돌릴 때 그럴듯하게 바뀌는지 먼저 확인한다.

### (선택) 설정이 날아갔을 때 수동 복구: `push_uart2_config.py`

Ardusimple 보드는 백업 배터리가 없어서, `UBX-CFG-CFG`에서 **BBR만 체크하고 Flash를 빠뜨리면** USB 전원이 끊기는 순간(케이블을 뽑거나 다른 PC로 옮기는 등) 위 설정이 통째로 사라진다. `rtk.launch.py`는 이제 이 설정을 매 실행마다 다시 밀어넣지 않고 **보드 플래시에 저장된 값을 그대로 신뢰**하므로, Flash 저장을 빠뜨리면 다음 실행에서 UART2가 조용히 안 켜진 채로 뜬다.

u-center를 다시 붙이기 번거로운 상황이면, 같은 설정(UART2 out/in 프로토콜, RTCM 메시지, baud 460800)을 USB 제어 연결로 즉석에서 재주입하는 스크립트가 있다:

```bash
ros2 run hyper_rtk push_uart2_config --device /dev/tty_ublox_base  --role base  --uart2-baud 460800
ros2 run hyper_rtk push_uart2_config --device /dev/tty_ublox_rover --role rover --uart2-baud 460800
```

`ublox_gps_node`가 같은 포트를 열기 전에 먼저 끝나고 포트를 닫아야 하므로, `rtk.launch.py`를 띄우기 **전에** 따로 실행한다. 이건 어디까지나 응급 복구용이며, 정상적으로는 u-center 설정을 Flash까지 저장해 두면 다시 쓸 일이 없다.

## udev 규칙

`udev/99-hyper-serial.rules`가 `tty_ublox_base`/`tty_ublox_rover`를 만든다. 두 보드가 idVendor/idProduct까지 완전히 같고(`1546:01a9`) 이 보드는 `ATTRS{serial}`도 안 내므로(첫 시리얼 값이 PCI 호스트 컨트롤러 것으로 잡힘), 물리 USB 포트(`KERNELS`)로 구분한다 — 규칙 파일 상단 주석 참고. 자세한 설치 방법은 저장소 루트 README의 "USB 시리얼 포트 고정" 절 참고.

## 실행

```bash
ros2 launch hyper_rtk rtk.launch.py
```

실행 전 `config/ntrip_params.yaml`에 NTRIP 캐스터 주소, 마운트포인트, 계정 정보를 설정하고, `/dev/tty_ublox_base`/`/dev/tty_ublox_rover`가 잡혀 있는지 확인하세요. 자세한 설치 방법은 저장소 루트 README의 GPS(RTK) 절을 참고하세요.

```bash
ros2 topic hz /gps/fix
ros2 topic echo /imu/heading                        # rover 헤딩 Imu (yaw만 유효, ekf_global 입력)
ros2 topic echo /ublox_gps_node_rover/navrelposned  # relPosHeading 원본 (진단용)
```

## 문제 해결: diffSoln은 서는데 relPosValid/headValid가 안 뜬다

`ublox_gps`의 RTCM 입력 구독(`node.cpp`)이 `~/rtcm`이 아니라 **절대 경로 `/rtcm`으로 하드코딩**돼 있어서, remap 없이는 base와 rover 둘 다 `ntrip_client`가 내는 전역 `/rtcm`(NTRIP CORS 보정)을 그대로 받아 각자 USB 링크로 밀어넣는다. rover가 UART2로 받는 moving-base 전용 보정(baseline 수 미터)과 NTRIP망 보정(기준국까지 수 km)이 동시에 들어가 서로 경쟁하면서, NTRIP 보정 자체는 유효하니 `diffSoln`은 서지만 moving-base 상대위치는 영영 안 풀리는 걸로 나타난다.

증상 확인: `ros2 topic echo /ublox_gps_node_rover/rxmrtcm`의 `msg_type`에 `1013`/`1033`/MSM5(`1075`/`1085`/`1095`/`1115`/`1125`)처럼 UART2용으로 설정한 적 없는 메시지가 `crcFailed=0`(깨끗한 수신)으로 섞여 나오면 이 문제다 — NTRIP CORS망이 흔히 내보내는 조합과 일치한다.

`rtk.launch.py`의 `ublox_rover` remappings에 `('/rtcm', 'unused/rtcm')`을 넣어 rover의 NTRIP 구독 자체를 끊는 것으로 고친다(rover는 NTRIP이 필요 없다 — UART2로만 보정받는다).
