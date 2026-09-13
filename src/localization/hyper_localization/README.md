# hyper_localization

`robot_localization`을 이용해 차량의 위치와 자세를 추정하는 패키지입니다. 휠/오도메트리, IMU, GPS 정보를 dual EKF와 `navsat_transform`으로 융합합니다.

## 실행

```bash
ros2 launch hyper_localization odometry.launch.py datum_site:=track
```

`datum_site`는 `config/datums.yaml`에 등록된 GPS 원점(datum) 중 하나를 고릅니다 (`school`, `track`). 생략하면 `track`이 기본값입니다.

**시뮬레이션 전용 원점(`sim`)은 없습니다.** Gazebo 월드(`hyper_gazebo`의 `track.world`)가 용인 트랙을 그대로 옮겨 놓은 디지털 트윈이라, 시뮬도 실차와 같은 `track` 원점을 씁니다 -- 덕분에 `track/*.csv` 웨이포인트와 `mission_track.yaml`을 양쪽에서 그대로 씁니다. 월드의 `<spherical_coordinates>`는 `track` 항목의 위경도와 반드시 같아야 하고, 어긋나면 `vehicle.launch.py`가 띄울 때 경고합니다.

두 datum 모두 실측값입니다 -- `track`(37.2887947, 127.1072418)은 RTK fix 상태로 녹화한 기록에서 나왔고, 그 원본 위경도가 `waypoints/track/s_curve.csv`의 `gps_lat`/`gps_lon` 열에 아직 남아 있습니다.

각 datum은 위경도/heading뿐 아니라 `magnetic_declination_deg`/`yaw_offset_deg`도 갖고 있지만, `wait_for_datum: true`면 `navsat_transform`이 `/imu`를 아예 구독하지 않으므로 지금은 둘 다 아무 데도 영향을 주지 않습니다(0 유지).

설정은 `config/dual_ekf_navsat.yaml`에 있으며, 실제 센서 토픽 이름이나 좌표계가 달라지면 이 파일을 환경에 맞게 조정해야 합니다.
