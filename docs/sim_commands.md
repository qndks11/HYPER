ros2 launch hyper_launch sim.launch.py site:=school       
ros2 launch hyper_launch odometry.launch.py datum_site:=school
ros2 launch hyper_launch perception.launch.py lane_input_backend:=ros_raw
ros2 launch hyper_launch behavior.launch.py mission:=mission_track
ros2 run hyper_waypoint_studio waypoint_studio
