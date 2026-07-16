# T8 Sport C++ 주차 시스템

기존 Python 주차 묶음을 실차용 C++ 노드로 옮긴 ROS 2 Humble 패키지다.

## 포함 노드

- `parking_scan_costmap`: `/scan`을 TF로 `base_link`에 변환하고 T8 차체 내부 점을 제거한 뒤 `/parking/local_costmap` 발행
- `hybrid_astar_planner`: Scan costmap에서 전진/후진 Hybrid A* 경로 생성
- `behavior_supervisor_with_parking`: GPS 이벤트, 교차로, 주차 상태 전환 및 `/parking/goal` 발행
- `controller_with_parking`: 차선/교차로/waypoint/주차 추종, 기어 변경 정지, 주차 중 `/scan` 즉시 정지

기존 `waypoint_recorder.py`는 저장 도구이므로 Python으로 그대로 사용해도 된다.

## 설치

```bash
sudo apt update
sudo apt install -y libyaml-cpp-dev

cd ~/HYPER/src
cp -a ~/Downloads/parking_cpp_t8_bundle ./parking_cpp

cd ~/HYPER
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select parking_cpp --symlink-install
source install/setup.bash
```

## 실행

기존 Python `behavior_supervisor_with_parking.py`와 `controller_with_parking.py`는 동시에 실행하면 안 된다. 같은 `/driving_mode`, `/cmd`, `/velocity`, `/steering_angle`을 발행하기 때문이다.

기록한 교차로·주차 통합 YAML을 다음 파일로 저장한다.

```text
~/HYPER/src/waypoint/course.yaml
```

기본 실행 시 `$HOME/HYPER/src/waypoint/course.yaml`을 자동으로 로드한다.

```bash
ros2 launch parking_cpp parking_system_cpp.launch.py
```

다른 YAML을 시험할 때만 `course_yaml:=절대경로`로 덮어쓸 수 있다.

```bash
ros2 launch parking_cpp parking_system_cpp.launch.py \
  course_yaml:=$HOME/HYPER/src/waypoint/other_course.yaml
```

## 필수 입력

```text
/scan                       sensor_msgs/LaserScan
/odometry/filtered_map      nav_msgs/Odometry
/gps/fix                    sensor_msgs/NavSatFix
/lane/center                std_msgs/Float64MultiArray
/stopline/detection         std_msgs/Float64MultiArray
/perception/sign            std_msgs/String
/mission/turn               std_msgs/String
```

## 주요 출력

```text
/driving_mode
/bridge_path
/parking/local_costmap
/parking/goal
/parking/path
/parking/directions
/parking/status
/parking/controller_status
/cmd
/velocity
/steering_angle
```

## 라이다 TF

`/scan.header.frame_id`가 `laser_link`라면 URDF에 다음 형태의 고정 TF가 필요하다.

```xml
<joint name="laser_joint" type="fixed">
  <parent link="base_link"/>
  <child link="laser_link"/>
  <origin xyz="0.30 0.0 0.55" rpy="0 0 0"/>
</joint>
```

실제 장착 위치를 측정해서 `xyz`를 수정한다.

```bash
ros2 topic echo /scan --once | grep frame_id
ros2 run tf2_ros tf2_echo base_link laser_link
```

## 안전 동작

주차 중 다음 조건이면 목표 속도를 즉시 0으로 발행한다.

- `/scan` 수신이 `scan_timeout_s`보다 오래 끊김
- 진행 방향 Scan 시야가 없음
- 전진 시 차체 앞끝 + `parking_obstacle_clearance_m` 안에 장애물
- 후진 시 차체 뒤끝 + `parking_obstacle_clearance_m` 안에 장애물

이 기능은 ROS 명령을 0으로 만드는 기능이다. 하위 MCU에도 통신 타임아웃, PWM 차단, 실제 브레이크 처리를 별도로 넣어야 한다.

## 확인

```bash
ros2 topic hz /scan
ros2 topic echo /parking/local_costmap --once
ros2 topic echo /parking/status
ros2 topic echo /parking/controller_status
ros2 topic echo /driving_mode
```

RViz에서 `/parking/local_costmap`, `/parking/path`, `/parking/debug_markers`를 추가해 확인한다.

## 현재 검증 범위

코드는 Python 버전과 동일한 토픽 구조를 기준으로 C++로 작성했다. 이 생성 환경에는 ROS 2 Humble 헤더와 실제 차량이 없어서 `colcon build`와 실차 주행은 수행하지 못했다. 첫 빌드 오류가 나오면 오류 전문과 현재 `CMakeLists.txt`, `package.xml`을 기준으로 맞춰야 한다.
