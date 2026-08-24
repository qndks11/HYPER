# hyper_localization

`robot_localization`을 이용해 차량의 위치와 자세를 추정하는 패키지입니다. 휠/오도메트리, IMU, GPS 정보를 dual EKF와 `navsat_transform`으로 융합합니다.

## 실행

```bash
ros2 launch hyper_localization odometry.launch.py datum_site:=sim
```

`datum_site`는 `config/datums.yaml`에 등록된 GPS 원점(datum) 중 하나를 고릅니다 (`sim`, `school`, `track`). 생략하면 `sim`이 기본값입니다. 각 datum은 위경도/heading뿐 아니라 `magnetic_declination_deg`/`yaw_offset_deg`(IMU 보정값)도 사이트별로 갖고 있습니다 -- 시뮬레이션은 지자기 모델이 없는 Gazebo IMU라 둘 다 0이어야 하고, 실차는 자편각과 IMU 장착 offset이 필요합니다. `school`/`track`은 아직 실측값이 채워지지 않은 placeholder이므로, 학교나 실제 트랙에서 뛰기 전에 `config/datums.yaml`의 해당 항목을 실측값으로 교체하세요.

설정은 `config/dual_ekf_navsat.yaml`에 있으며, 실제 센서 토픽 이름이나 좌표계가 달라지면 이 파일을 환경에 맞게 조정해야 합니다.
