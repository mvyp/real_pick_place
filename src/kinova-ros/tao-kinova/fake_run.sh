gnome-terminal -t "core" -x bash -c "roscore;exec bash;"
sleep 2s
gnome-terminal -t "gazebo" -x bash -c "roslaunch kinova_with_d435 run.launch;exec bash;"
# gnome-terminal -t "gazebo" -x bash -c "roslaunch kinova_gazebo robot_launch.launch;exec bash;"

sleep 8s
gnome-terminal -t "rviz" -x bash -c "roslaunch j2s7s300_moveit_config j2s7s300_gazebo_demo.launch; exec bash;"
sleep 5s
gnome-terminal -t "pick_place" -x bash -c "rosrun kinova_arm_moveit_demo my_pick_place;exec bash;"
# gnome-terminal -t "demo" -x bash -c "rosrun kinova_demo pose_action_client.py j2s7s300 mq 0 0.6 0.6 0 0 0 1;exec bash;"

sleep 3s
gnome-terminal -t "fake" -x bash -c "python3 -u '/home/tao/catkin_ws/src/gpd_ros/src/gpd_ros/test.py';exec bash;"
sleep 1s
gnome-terminal -t "teleop" -x bash -c "rosrun teleop_twist_keyboard teleop_twist_keyboard.py; exec bash;"
