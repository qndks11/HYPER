# CP2102 USB 시리얼 문자열 다시 쓰기

EBIMU USB-UART 어댑터와 RPLidar는 **둘 다 CP2102**라 VID:PID가 `10c4:ea60`으로
같다. 게다가 공장 기본 USB 시리얼 문자열이 양쪽 다 `"0001"`이라(라이다만 그런 게
아니다) udev가 볼 수 있는 값 중 둘을 가르는 게 하나도 없었다.

그래서 각 칩의 EEPROM에 이름을 써 넣었다. RTK 보드 두 장에 `HYPER-GNSS-BASE` /
`HYPER-GNSS-ROVER`를 써 넣은 것과 같은 방식이다(`ublox-serial.py` 참고).

| 장치 | USB 시리얼 | 심볼릭 링크 |
| --- | --- | --- |
| EBIMU USB-UART 어댑터 | `HYPER-EBIMU` | `/dev/tty_ebimu` |
| RPLidar (model 24 / fw 1.29) | `HYPER-LIDAR` | `/dev/rplidar` |

`99-hyper-serial.rules`의 2)번과 5)번이 이 이름으로 잡는다. 포트 위치와 무관하므로
어느 USB 포트에 꽂아도 된다.

## 도구

`cp210x-cfg` (https://github.com/DiUS/cp210x-cfg) — libusb-1.0으로 CP210x EEPROM을
읽고 쓰는 단일 C 파일이다. 저장소에 넣어 두지 않았으니 필요할 때 받아서 빌드한다.

```bash
sudo apt install libusb-1.0-0-dev
curl -sSLO https://raw.githubusercontent.com/DiUS/cp210x-cfg/master/src/main.c
gcc -O2 -Wall -o cp210x-cfg main.c -lusb-1.0
```

`sudo`는 필요 없다. `/dev/bus/usb/*`가 `root:plugdev`이고 계정이 `plugdev`에 있다.

## 읽기

```bash
./cp210x-cfg -l          # 꽂혀 있는 CP210x 전부. bus/dev 번호만 나온다
./cp210x-cfg -d 5.19     # 그 장치 하나의 model/VID/PID/name/serial
```

대상 bus/dev는 이렇게 찾는다:

```bash
udevadm info -q property -p /sys/bus/usb/devices/5-1.2 | grep -E 'BUSNUM|DEVNUM'
```

### -d 옵션의 함정 두 개

- `--help`는 `-d bus:dev`라고 하지만 **파서는 점을 요구한다**: `-d 5.19`.
  콜론을 쓰면 그냥 에러로 끝난다.
- dev 번호가 `strtol(..., 0)`으로 파싱된다 -- **앞의 0이 8진수를 뜻한다**.
  `udevadm`은 `DEVNUM=019`로 찍어 주지만 `-d 5.019`는 `01`까지만 읽고 `9`에서
  멈춰서 **장치 1번(루트 허브)** 을 잡는다. 앞의 0을 반드시 떼고 `-d 5.19`로 쓸 것.

`-m 10c4:ea60`은 **쓰지 말 것**. 조건에 맞는 *첫* 장치를 잡는데, 애초에 둘이
구분이 안 돼서 이 문서가 있는 것이다. 반드시 `-d`로 정확히 찍어야 한다.

## 쓰기

```bash
./cp210x-cfg -d 5.11 -S HYPER-EBIMU
./cp210x-cfg -d 5.19 -S HYPER-LIDAR
```

`-S`는 EEPROM의 시리얼 항목 하나만 vendor control transfer 한 번으로 쓴다.
VID/PID/name은 각자 다른 플래그(`-V`/`-P`/`-N`)를 줘야 건드려지므로, 시리얼만
쓰는 이 명령이 VID/PID를 깨뜨릴 수는 없다. **`-V`/`-P`는 쓰지 말 것** -- VID/PID를
잘못 쓰면 `cp210x` 드라이버가 안 붙어서 복구가 번거로워진다.

### 쓴 직후 에러가 나는 건 정상이다

```
error: failed to read cfg item 370b: No such device (it may have been disconnected)
```

쓰기 다음에 `libusb_reset_device`가 불리는데, 그 뒤 설정을 다시 출력할 때 쓰는
디스크립터는 리셋 전에 캐시한 것이다. 장치가 재열거되면서 핸들이 죽어 저 에러가
난다. **쓰기는 리셋 전 control transfer에서 이미 끝났다.** 확인은 재열거로 한다:

```bash
udevadm info -q property -n /dev/tty_ebimu | grep ID_SERIAL_SHORT
ls -l /dev/tty_ebimu /dev/rplidar
```

## 안 될 때

- EEPROM이 잠겨 있으면(AN721 lock byte) control transfer가 에러로 끝나고 아무것도
  안 바뀐다. 그때는 `KERNELS=="5-1.2"`처럼 물리 포트로 가르는 수밖에 없다.
- 짝퉁 CP2102는 문자열이 깨져 들어갈 수 있다. 다시 쓰면 된다.
