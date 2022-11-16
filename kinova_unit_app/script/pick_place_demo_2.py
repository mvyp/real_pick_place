#! /usr/bin/env python

import sys

import rospy
import actionlib

import std_msgs.msg
import geometry_msgs.msg
import jaco2_pick_place

def main():
    
    rospy.init_node('pick_place_demo_node')

    # rospy.loginfo("set height to 0.2")
    # body_pub.publish(0.2)

    pick_place = jaco2_pick_place.Jaco2PickPlace('j2s7s300')
    rospy.sleep(2)

    # rospy.loginfo('set arm to ready pose succeed')
    # if not pick_place.ready():
    #     rospy.loginfo('set arm to ready pose fail')
    #     return
    # rospy.sleep(2)


    rospy.loginfo('pick')
    if not pick_place.pick():
        return

    # rospy.loginfo('place')
    # if not pick_place.place():
    #     return
    # rospy.sleep(1)

    # rospy.loginfo('set arm to home pose')
    # if not pick_place.ready():
    #     return

    # rospy.loginfo('move to home place')
    # # if not move_base_client.move('home'):
    #     # return

if __name__ == '__main__':
    main()
