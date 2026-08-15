# hyper_planner

HYPER의 행동 결정과 차량 제어를 담당하는 C++ 패키지입니다. 교차로·경사로·장애물·주차 이벤트를 관리하고, 차선·정지선·신호·위치 정보를 바탕으로 조향과 속도 명령을 생성합니다.

## 구성

- `behavior_supervisor_with_parking_node`: 주행 모드와 이벤트 상태를 결정합니다.
- `controller_with_parking_node`: 차선 및 경로를 따라 `/cmd`, `/velocity`, `/steering_angle` 명령을 생성합니다.
- `config/parking_params.yaml`: 제어 및 이벤트 파라미터입니다.

## 실행

```bash
ros2 launch hyper_planner parking_system_cpp.launch.py
```

이벤트 등록과 상세 운용 방법은 [README_KO.md](README_KO.md)를 참고하세요.
