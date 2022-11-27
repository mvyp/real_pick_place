#! /usr/bin/env python3
import rospy
from std_msgs.msg import String
 
def talker():
    rospy.init_node('my_talker')
    pub = rospy.Publisher('/pick_place/target', String, queue_size=1)

    rospy.sleep(1)

    pub.publish("bottle")
 
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass