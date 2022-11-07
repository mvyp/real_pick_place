#! /usr/bin/env python

import sys
import math
import numpy as np

import tf
import rospy
import actionlib

import std_msgs.msg
import kinova_msgs.msg
import geometry_msgs.msg

ready_angles = {}
ready_angles['j2s7s300'] = [283.118682861, 162.867752075, 43.4267578125, 265.080688477, 257.716766357, 288.920166016, 0.0]
ready_angles0 = {}
ready_angles0['j2n6s200'] = [34.5197105408, 90.3958053589, 281.551940918, 165.190322876, 236.576141357, 302.368469238, 0.0]

# ready_angles = {}
# ready_angles['j2n6s200'] = [2.72439885139, 180.53994751, 300.930175781, 129.935928345, 276.530944824, 260.175170898, 0.0]
home_angles = {}
home_angles['j2s7s300'] = [275.938201904, 212.978210449, 98.8227462769, 359.023712158, 240.306106567, 178.173019409, 0.0]
home_angles0 = {}
home_angles0['j2n6s200'] = [11.3516435623, 89.7998962402, 307.609283447, 150.992614746, 85.8830947876, 428.717071533, 0.0]

class Jaco2PickPlace():
    def __init__(self, arm_name):
        self.__arm_name = arm_name
        self.__pose_action_client = actionlib.SimpleActionClient('j2s7s300_driver/pose_action/tool_pose',
                                                               kinova_msgs.msg.ArmPoseAction)
        self.__joints_action_client = actionlib.SimpleActionClient('j2s7s300_driver/joints_action/joint_angles',
                                                                 kinova_msgs.msg.ArmJointAnglesAction)
        self.__gripper_action_client = actionlib.SimpleActionClient('j2s7s300_driver/fingers_action/finger_positions',
                                                                  kinova_msgs.msg.SetFingersPositionAction)

        self.__pose_action_client.wait_for_server()
        self.__joints_action_client.wait_for_server()
        self.__gripper_action_client.wait_for_server()

    def __set_joint_angle(self, target_angle):
        goal = kinova_msgs.msg.ArmJointAnglesGoal()
        goal.angles.joint1 = target_angle[0]
        goal.angles.joint2 = target_angle[1]
        goal.angles.joint3 = target_angle[2]
        goal.angles.joint4 = target_angle[3]
        goal.angles.joint5 = target_angle[4]
        goal.angles.joint6 = target_angle[5]
        goal.angles.joint7 = target_angle[6]

        self.__joints_action_client.send_goal(goal)
        if self.__joints_action_client.wait_for_result(rospy.Duration(20.0)):
            return self.__joints_action_client.get_result()
        else:
            print('the joint angle action timed-out')
            self.__joints_action_client.cancel_all_goals()
            return None

    def __set_pose(self, target_pose):
        goal = kinova_msgs.msg.ArmPoseGoal()
        goal.pose = target_pose

        self.__pose_action_client.send_goal(goal)
        if self.__pose_action_client.wait_for_result(rospy.Duration(10.0)):
            return self.__pose_action_client.get_result()
        else:
            self.__pose_action_client.cancel_all_goals()
            print('the cartesian action timed-out')
            return None

    def __set_gripper(self, target_percent):
        goal = kinova_msgs.msg.SetFingersPositionGoal()
        goal.fingers.finger1 = 6800.0 * target_percent
        goal.fingers.finger2 = 6800.0 * target_percent
        goal.fingers.finger3 = 0

        self.__gripper_action_client.send_goal(goal)
        if self.__gripper_action_client.wait_for_result(rospy.Duration(5.0)):
            return self.__gripper_action_client.get_result()
        else:
            self.__gripper_action_client.cancel_all_goals()
            rospy.logwarn('the gripper action timed-out')
            return None

    def home(self):
        try:
            result = self.__set_joint_angle(home_angles[self.__arm_name])
        except rospy.ROSInterruptException:
            print "program interrupted before completion"
            return False

        return True

    def ready(self):
        try:
            result = self.__set_joint_angle(ready_angles[self.__arm_name])
        except rospy.ROSInterruptException:
            print "program interrupted before completion"
            return False

        return True

    def pick(self):
        try:
            pose = geometry_msgs.msg.PoseStamped()
            pose.header = std_msgs.msg.Header(frame_id='object')

            t = tf.transformations.quaternion_from_euler(math.pi/2.0, math.pi/2.0, 0.0)
            # t = tf.transformations.quaternion_from_euler(math.pi/2.0, 0.0, math.pi/2.0)
            pose.pose.orientation = geometry_msgs.msg.Quaternion(x=t[0], y=t[1], z=t[2], w=t[3])

            # result = self.__set_joint_angle(ready_angles[self.__arm_name])
            # rospy.Rate(0.4).sleep()

            result = self.__set_gripper(0.1)
            # rospy.Rate(1).sleep()

            pose.pose.position = geometry_msgs.msg.Point(x=-0.0, y=0.1, z=0.05)
            result = self.__set_pose(pose)
            rospy.Rate(1).sleep()

            pose.pose.position = geometry_msgs.msg.Point(x=-0.0, y=0.0, z=0.05)
            result = self.__set_pose(pose)
            rospy.Rate(1).sleep()

            result = self.__set_gripper(0.65)
            # rospy.Rate(1).sleep()
            # pose.pose.position = geometry_msgs.msg.Point(x=-0.12, y=-0.015, z=0.1)
            # result = self.__set_pose(pose)

            # pose.pose.position = geometry_msgs.msg.Point(x=-0.28, y=-0.015, z=0.1)
            # result = self.__set_pose(pose)
            # rospy.Rate(1).sleep()

            # result = self.__set_joint_angle(ready_angles[self.__arm_name])
            # rospy.Rate(1).sleep()

            # result = self.__set_gripper(0.1)

        except rospy.ROSInterruptException:
            print "program interrupted before completion"
            return False

        return True

    def place(self):
        try:
            pose = geometry_msgs.msg.PoseStamped()
            pose.header = std_msgs.msg.Header(frame_id='plane')

            t = tf.transformations.quaternion_from_euler(0.0, math.pi/2.0, 0.0)
            pose.pose.orientation = geometry_msgs.msg.Quaternion(x=t[0], y=t[1], z=t[2], w=t[3])

            result = self.__set_joint_angle(ready_angles[self.__arm_name])
            rospy.Rate(0.4).sleep()

            pose.pose.position = geometry_msgs.msg.Point(x=-0.1, y=0.13, z=-0.05)
            result = self.__set_pose(pose)
            # rospy.Rate(1).sleep()

            pose.pose.position = geometry_msgs.msg.Point(x=-0.1, y=0.055, z=-0.05)
            result = self.__set_pose(pose)
            # rospy.Rate(1).sleep()

            result = self.__set_gripper(0.1)
            rospy.Rate(1).sleep()

            pose.pose.position = geometry_msgs.msg.Point(x=-0.20, y=0.14, z=-0.05)
            result = self.__set_pose(pose)
            # rospy.Rate(1).sleep()

            result = self.__set_joint_angle(ready_angles[self.__arm_name])
            rospy.Rate(1).sleep()

            # result = self.__set_gripper(0.1)

        except rospy.ROSInterruptException:
            print "program interrupted before completion"
            return False

        return True
