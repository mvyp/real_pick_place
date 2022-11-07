gnome-terminal -t "core" -x bash -c "roscore;exec bash;"
sleep 2s
gnome-terminal -t "gazebo" -x bash -c "roslaunch kinova_with_d435 run.launch;exec bash;"
sleep 8s
gnome-terminal -t "rviz" -x bash -c "roslaunch j2s7s300_moveit_config j2s7s300_gazebo_demo.launch; exec bash;"
sleep 5s
gnome-terminal -t "pick_place" -x bash -c "rosrun kinova_arm_moveit_demo my_pick_place;exec bash;"
sleep 3s
gnome-terminal -t "filter" -x bash -c "roslaunch gpd_ros filter.launch;exec bash;"
sleep 1s
# gnome-terminal -t "gpd" -x bash -c "roslaunch gpd_ros ur5.launch;exec bash;"
gnome-terminal -t "keyboard" -x bash -c "rosrun teleop_twist_keyboard teleop_twist_keyboard.py; exec bash;"
