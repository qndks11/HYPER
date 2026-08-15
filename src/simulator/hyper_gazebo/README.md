# hyper_gazebo

HYPER 차량을 Gazebo에서 실행하기 위한 시뮬레이션 전용 패키지입니다. 트랙 월드, 차량 스폰, Gazebo-ROS 브리지, 시뮬레이션 센서와 제어 플러그인을 제공합니다.

## 실행

```bash
ros2 launch hyper_gazebo vehicle.launch.py
```

`worlds/track.world`가 기본 주행 환경이며, 초기 차량 위치와 자세는 launch 인자로 변경할 수 있습니다. 실차에서는 이 패키지 대신 실제 센서·제어 패키지를 사용합니다.
