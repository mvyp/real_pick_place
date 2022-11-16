#! /usr/bin/env python3
import rospy
from std_msgs.msg import String
 
def talker():
    rospy.init_node('my_talker')
    pub = rospy.Publisher('/pick_place/target', String, queue_size=1)

    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():

        pub.publish("bottle")
        #print("aa")
        rate.sleep()
 
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass