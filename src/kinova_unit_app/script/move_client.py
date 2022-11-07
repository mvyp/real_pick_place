#! /usr/bin/env python

import sys

import rospy
import actionlib

import std_msgs.msg
import geometry_msgs.msg
import move_base_msgs.msg

import jaco2_pick_place

class MoveClient():
    def __init__(self, server_name, map_name):
        self.__map_name = map_name
        self.__server_name = server_name
        self.__client = actionlib.SimpleActionClient(self.__server_name, move_base_msgs.msg.MoveBaseAction)

        self.__load_goals(self.__map_name)

        # self.__client.wait_for_server()

    def __load_goals(self, map_name):
        goal_list = rospy.get_param('goals/' + map_name + '/goal_list')

        self.__goals = {}
        for name in goal_list:
            frame_id_name = 'goals/' + map_name + '/'+ name + '/frame_id'
            pose_name = 'goals/' + map_name + '/'+ name + '/position'
            orientation_name = 'goals/' + map_name + '/'+ name + '/orientation'
            self.__goals[name] = geometry_msgs.msg.PoseStamped()
            self.__goals[name].header = std_msgs.msg.Header(frame_id=rospy.get_param(frame_id_name))
            self.__goals[name].pose.position = geometry_msgs.msg.Point(x=rospy.get_param(pose_name)[0],
                                                                  y=rospy.get_param(pose_name)[1],
                                                                  z=rospy.get_param(pose_name)[2])
            self.__goals[name].pose.orientation = geometry_msgs.msg.Quaternion(x=rospy.get_param(orientation_name)[0],
                                                                          y=rospy.get_param(orientation_name)[1],
                                                                          z=rospy.get_param(orientation_name)[2],
                                                                          w=rospy.get_param(orientation_name)[3])

        rospy.loginfo("get %d goals, %s", len(goal_list), goal_list)

    def move(self, goal_name):
        if not self.__goals.has_key(goal_name):
            rospy.logwarn('target not on the goal list')
            return False

        goal = move_base_msgs.msg.MoveBaseGoal()
        goal.target_pose = self.__goals[goal_name]
        goal.target_pose.header.stamp = rospy.Time.now()

        self.__client.send_goal(goal)
        self.__client.wait_for_result()

        result = self.__client.get_result()
        return result
        # if result == actionlib.SimpleActionClient.succeeded:
        #     return True
        # else:
        #     rospy.logwarn('move failed')
        #     return False
