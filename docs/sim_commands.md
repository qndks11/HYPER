ros2 launch hyper_launch sim.launch.py        # Gazebo + 차량 스폰 + 브릿지만
ros2 launch hyper_launch odometry.launch.py datum_site:=track   # use_sim_time 기본 true
ros2 launch hyper_launch perception.launch.py lane_input_backend:=ros_raw
ros2 launch hyper_launch behavior.launch.py mission:=mission_track
