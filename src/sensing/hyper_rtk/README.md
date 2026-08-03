# hyper_rtk

u-blox GNSS 수신기와 NTRIP 클라이언트를 함께 실행해 RTK 보정 위치를 제공하는 패키지입니다. `ublox_gps`가 GNSS 위치를 읽고, `ntrip_client`가 RTCM 보정 데이터를 수신합니다.

## 실행

```bash
ros2 launch hyper_rtk rtk.launch.py
```

실행 전 `config/ntrip_params.yaml`에 NTRIP 캐스터 주소, 마운트포인트, 계정 정보를 설정하고 GNSS 장치가 `/dev/tty_Ardusimple`로 연결되어 있는지 확인하세요. 자세한 설치 방법은 저장소 루트 README의 GPS(RTK) 절을 참고하세요.
