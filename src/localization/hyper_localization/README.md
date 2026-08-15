# hyper_localization

`robot_localization`을 이용해 차량의 위치와 자세를 추정하는 패키지입니다. 휠/오도메트리, IMU, GPS 정보를 dual EKF와 `navsat_transform`으로 융합합니다.

## 실행

```bash
ros2 launch hyper_localization odometry.launch.py
```

설정은 `config/dual_ekf_navsat.yaml`에 있으며, 실제 센서 토픽 이름이나 좌표계가 달라지면 이 파일을 환경에 맞게 조정해야 합니다.
