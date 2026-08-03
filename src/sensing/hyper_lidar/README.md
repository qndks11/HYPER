# hyper_lidar

실차에 연결된 RPLidar 등의 2D LiDAR 드라이버를 실행하기 위한 설정·launch 패키지입니다. 시뮬레이션에서는 사용하지 않으며, Gazebo가 `/scan`을 직접 발행합니다.

## 실행

```bash
ros2 launch hyper_lidar rplidar.launch.py
```

장치 포트, 보드레이트, 프레임 이름은 `config/rplidar_params.yaml`에서 실제 장비에 맞게 설정합니다.
