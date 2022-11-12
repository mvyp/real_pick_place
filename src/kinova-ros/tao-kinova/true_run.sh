
gnome-terminal -t "robot" -x bash -c "roslaunch kinova_bringup kinova_robot.launch;exec bash;"
 sleep 1
gnome-terminal -t "base" -x bash -c "roslaunch mrobot_bringup tidy_up.launch;exec bash;"

sleep 3
# gnome-terminal -t "camera" -x bash -c "roslaunch realsense2_camera rs_d435_camera_with_model.launch;exec bash"
gnome-terminal -t "camera" -x bash -c "roslaunch realsense2_camera rs_camera.launch ;exec bash"
# gnome-terminal -t "camera" -x bash -c "roslaunch azure_kinect_ros_driver hand_eye.launch;exec bash"

sleep 2s
gnome-terminal -t "moveit" -x bash -c "roslaunch j2s7s300_moveit_config j2s7s300_demo.launch; exec bash;"
sleep 1s
gnome-terminal -t "pick_place" -x bash -c "rosrun kinova_arm_moveit_demo closely_pick_place;exec bash;"
sleep 3s
# gnome-terminal -t "start tolerence" -x bash -c "rosparam set /move_group/trajectory_execution/allowed_start_tolerance 0.3;exec bash;"
# gnome-terminal -t "fake" -x bash -c "rosrun tao-kinova test.py;exec bash;"
gnome-terminal -t "tf" -x bash -c "roslaunch kinova_unit_app publish.launch ;exec bash;"
sleep 1s
gnome-terminal -t "cloud" -x bash -c "rosrun gpd_ros global_point ;exec bash;"
# gnome-terminal -t "goal" -x bash -c "rostopic echo /detect_grasps/goal_pose;exec bash;"
sleep 1s
gnome-terminal -t "gpd" -x bash -c "roslaunch gpd_ros ur5.launch ;exec bash;"
