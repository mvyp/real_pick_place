#! /bin/bash
# {
# gnome-terminal -x bash -c "roslaunch realsense2_camera rs_rgbd.launch;exec bash"
# }
# sleep 3
# {
# gnome-terminal -x bash -c "roslaunch kinova_bringup kinova_robot.launch;exec bash"
# }
# sleep 3
# {
# gnome-terminal -x bash -c "roslaunch j2s7s300_moveit_config j2s7s300_demo.launch;exec bash"
# }
#  sleep 3
#  {
#  gnome-terminal -x bash -c "roslaunch kinova_unit_app eye_on_hand.launch;exec bash"
#  }
#  sleep 3
# {
#  gnome-terminal -x bash -c "rqt_image_view;exec bash"
# }
gnome-terminal -x bash -c "roslaunch azure_kinect_ros_driver hand_eye.launch;exec bash"

sleep 2
{
gnome-terminal -x bash -c "roslaunch kinova_bringup kinova_robot.launch;exec bash"
}
sleep 3
{
gnome-terminal -x bash -c "roslaunch j2s7s300_moveit_config j2s7s300_demo.launch;exec bash"
}
 sleep 3
 {
 gnome-terminal -x bash -c "roslaunch kinova_unit_app eye_on_base.launch;exec bash"
 }
 sleep 3
{
 gnome-terminal -x bash -c "rqt_image_view;exec bash"
}