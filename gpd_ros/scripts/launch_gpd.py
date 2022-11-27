#! /usr/bin/env python3
import roslaunch
import rospy
from std_msgs.msg import String
rospy.init_node('en_Mapping', anonymous=True)
while not rospy.is_shutdown():
    data = String()
    while (data.data != "GPD_table") and (data.data != "GPD_ground"):
        try:
            data = rospy.wait_for_message("/pick_place/message",
                                          String,
                                          timeout=30)
        except:
            print("pass")
            pass
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    if data.data == "GPD_table":
        print("table")
        launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/msi/kinova_ws/src/gpd_ros/launch/ur5.launch"])
    else:
        print("ground")
        launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/msi/kinova_ws/src/gpd_ros/launch/ur6.launch"])
    print("START")
    launch.start()
    rospy.loginfo("started")
    rospy.sleep(30)
    launch.shutdown()