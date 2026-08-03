# hyper_launch

HYPER 자율주행 스택을 단계별 또는 한 번에 실행하기 위한 launch 패키지입니다. 시뮬레이션, 위치 추정, 인지, 행동 계획을 정해진 순서로 시작합니다.

## 실행

전체 스택:

```bash
ros2 launch hyper_launch simulation.launch.py
```

단계별 실행:

```bash
ros2 launch hyper_launch sim.launch.py
ros2 launch hyper_launch odometry.launch.py
ros2 launch hyper_launch perception.launch.py
ros2 launch hyper_launch behavior.launch.py
```

각 launch 파일은 각각 `hyper_gazebo`, `hyper_localization`, 인지 패키지, `hyper_planner`를 호출합니다.
