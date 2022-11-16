#! /usr/bin/env python3

# Author: Nuha Nishat
# Date: 1/30/20

# Edited by Akshaya Agrawal
# Date: 05/25/21
'''
    The Door is placed at (-0.24, 0, 0) position. Initially the Handle is located at
    (-0.24,-0.36, 0) -> (x,y,z).
    The path of the door is such that it follows a circle centered at (0,-0.5).
    The Height of the handle from the base is 0.35
'''

import rospy
import actionlib
import kinova_msgs.msg
import geometry_msgs.msg
import std_msgs.msg
from kinova_msgs.srv import *
from sensor_msgs.msg import JointState
import argparse
import sys, os
import math
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
import tf, math
import tf.transformations
import pdb
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String

from moveit_commander.conversions import pose_to_list
from moveit_msgs.msg import RobotState, PlanningScene, PlanningSceneComponents, AllowedCollisionEntry, \
    AllowedCollisionMatrix
from moveit_msgs.srv import GetPlanningScene, ApplyPlanningScene
import numpy as np
import math
import copy
from tf.transformations import quaternion_from_euler

# move_group_python_interface_tutorial was used as reference
DL = 0.6
RPY = 0.35
RPX = 0.15


class MoveRobot():

    def __init__(self):
        # Initialize moveit commander and ros node for moveit

        # To read from redirected ROS Topic
        joint_state_topic = ['joint_states:=/j2s7s300_driver/out/joint_state']
        moveit_commander.roscpp_initialize(joint_state_topic)
        rospy.init_node('move-kinova', anonymous=False)
        moveit_commander.roscpp_initialize(sys.argv)

        # Define robot using RobotCommander. Provided robot info such as
        # kinematic model and current joint state
        self.robot = moveit_commander.RobotCommander()

        # Setting the world
        self.scene = moveit_commander.PlanningSceneInterface()

        # Define the planning group for the arm you are using
        # You can easily look it up on rviz under the MotionPlanning tab
        self.move_group = moveit_commander.MoveGroupCommander("arm")
        self.move_gripper = moveit_commander.MoveGroupCommander("gripper")

        self.move_group.set_goal_position_tolerance(0.001)
        self.move_group.set_goal_orientation_tolerance(0.001)
        # Set the precision of the robot
        self.client = actionlib.SimpleActionClient(
            '/j2s7s300_driver/pose_action/tool_pose',
            kinova_msgs.msg.ArmPoseAction)
        self.client.wait_for_server()

        rospy.set_param(
            '/move_group/trajectory_execution/allowed_start_tolerance', 0.3)

        rospy.wait_for_service("/apply_planning_scene", 10.0)
        rospy.wait_for_service("/get_planning_scene", 10.0)
        rospy.wait_for_service('/j2s7s300_driver/in/home_arm')
        self.apply_scene = rospy.ServiceProxy('/apply_planning_scene',
                                              ApplyPlanningScene)
        self.get_scene = rospy.ServiceProxy('/get_planning_scene',
                                            GetPlanningScene)
        rospy.sleep(2)

        # To see the trajectory
        self.disp = moveit_msgs.msg.DisplayTrajectory()

        self.disp.trajectory_start = self.robot.get_current_state()

        self.rate = rospy.Rate(10)

        self.move_group.allow_replanning(1)
        self.add_object()
        self.main()
        rospy.spin()

    def cartesian_pose_client(self, position, orientation, my_frame_id):
        """Send a cartesian goal to the action server."""
        goal = kinova_msgs.msg.ArmPoseGoal()
        goal.pose.header = std_msgs.msg.Header(frame_id=my_frame_id)
        goal.pose.pose.position = geometry_msgs.msg.Point(x=position[0],
                                                          y=position[1],
                                                          z=position[2])
        goal.pose.pose.orientation = geometry_msgs.msg.Quaternion(
            x=orientation[0],
            y=orientation[1],
            z=orientation[2],
            w=orientation[3])

        print('goal.pose in client 1: {}'.format(goal.pose.pose))  # debug

        self.client.send_goal(goal)

        if self.client.wait_for_result(rospy.Duration(15.0)):
            return self.client.get_result()
        else:
            self.client.cancel_all_goals()
            print('        the cartesian action timed-out')
            return None

    def homeRobot(self):
        try:
            home = rospy.ServiceProxy('/j2s7s300_driver/in/home_arm', HomeArm)
            home()
            return None
        except rospy.ServiceException as e:
            print("Service call failed: {}".format(e))
            home()

    def add_object(self):
        self.scene.remove_world_object('table')
        self.scene.remove_world_object('door')
        self.scene.remove_world_object('handle')
        self.scene.remove_world_object('wall')

        # 设置table和tool的三维尺寸
        table_size = [0.12, 0.09, 3]
        door_size = [3, 0.001, 2]
        handle_size = [0.3, 0.04, 0.02]
        wall_size = [0.6, 0.001, 2]

        # 将table加入场景当中
        table_pose = PoseStamped()
        table_pose.header.frame_id = 'root'
        table_pose.pose.position.x = 0.0
        table_pose.pose.position.y = 0.24
        table_pose.pose.position.z = -0.03

        door_pose = PoseStamped()
        door_pose.header.frame_id = 'root'
        door_pose.pose.position.x = 0
        door_pose.pose.position.y = -RPY - 0.13 / 2
        door_pose.pose.position.z = 1

        wall_pose = PoseStamped()
        wall_pose.header.frame_id = 'root'
        wall_pose.pose.position.x = 0.50
        wall_pose.pose.position.y = -RPY - 0.13 / 2
        wall_pose.pose.position.z = 1

        handle_pose = PoseStamped()
        handle_pose.header.frame_id = 'root'
        handle_pose.pose.position.x = RPX
        handle_pose.pose.position.y = -RPY
        handle_pose.pose.position.z = 1.15 - 0.53

        self.scene.add_box('table', table_pose, table_size)  #添加障碍物
        self.scene.add_box('door', door_pose, door_size)  #添加障碍物
        self.scene.add_box('handle', handle_pose, handle_size)  #添加障碍物
        self.scene.add_box('wall', wall_pose, wall_size)  #添加障碍物

        rospy.sleep(0.5)

    def set_planner_type(self, planner_name):
        if planner_name == "RRT":
            self.move_group.set_planner_id("RRTConnectkConfigDefault")
        if planner_name == "RRT*":
            self.move_group.set_planner_id("RRTstarkConfigDefault")
        if planner_name == "PRM*":
            self.move_group.set_planner_id("PRMstarkConfigDefault")

    def go_to_joint_state(self, joint_state):
        joint_goal = JointState()
        joint_goal.position = joint_state
        self.move_group.set_joint_value_target(joint_goal.position)

        self.plan = self.move_group.plan()
        self.move_group.go(wait=True)
        self.move_group.execute(self.plan, wait=True)

        self.move_group.stop()
        self.move_group.clear_pose_targets()
        rospy.sleep(2)

    def go_to_goal(self, ee_pose):
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.position.x = ee_pose[0]
        pose_goal.position.y = ee_pose[1]
        pose_goal.position.z = ee_pose[2]

        if len(ee_pose) == 6:
            quat = tf.transformations.quaternion_from_euler(
                math.radians(ee_pose[3]), math.radians(ee_pose[4]),
                math.radians(ee_pose[5]))
            pose_goal.orientation.x = quat[0]
            pose_goal.orientation.y = quat[1]
            pose_goal.orientation.z = quat[2]
            pose_goal.orientation.w = quat[3]

        else:
            pose_goal.orientation.x = ee_pose[3]
            pose_goal.orientation.y = ee_pose[4]
            pose_goal.orientation.z = ee_pose[5]
            pose_goal.orientation.w = ee_pose[6]

        self.move_group.set_pose_target(pose_goal)
        self.move_group.set_planning_time(20)
        rospy.sleep(2)
        self.move_group.go(wait=True)
        self.move_group.stop()

        self.move_group.clear_pose_targets()
        rospy.sleep(2)

    def move_gripper(self, cmd):
        if cmd == "Close":
            self.move_gripper.set_named_target("Close")
        elif cmd == "Open":
            self.move_gripper.set_named_target("Open")
        else:
            self.move_gripper.set_joint_value_target(cmd)
        self.move_gripper.go(wait=True)
        rospy.sleep(2)

    # def display_trajectory(self):
    #     self.disp_pub = rospy.Publisher("/move_group/display_planned_path",
    #                                     moveit_msgs.msg.DisplayTrajectory,
    #                                     queue_size=20)
    #     self.disp.trajectory.append(self.plan)
    #     print(self.disp.trajectory)
    #     self.disp_pub.publish(self.disp)

    def go_to_finger_state(self, cmd):
        if cmd == "Close":
            self.move_gripper.set_named_target("Close")
        elif cmd == "CurlClose":
            self.move_gripper.set_named_target("CurlClose")
        elif cmd == "Open":
            self.move_gripper.set_named_target("Open")
        else:
            """gripper_goal = JointState()
            gripper_goal.position = cmd"""
            self.move_gripper.set_joint_value_target(cmd)
        # self.plan_gripper = self.move_gripper.plan()

        self.move_gripper.go(wait=True)
        """self.move_gripper.execute(self.plan_gripper, wait=True)"""
        self.move_gripper.stop()
        self.move_gripper.clear_pose_targets()
        rospy.sleep(2)

    def go_to_finger_joint_state(self, joint_values):
        try:
            gripper_states = JointState()
            gripper_states.position = joint_values
            self.move_gripper.set_joint_value_target(gripper_states.position)
            self.move_gripper.go(wait=True)
            # self.move_gripper.execute(self.plan_gripper, wait=True)
            self.move_gripper.stop()
            self.move_gripper.clear_pose_targets()
            rospy.sleep(0.1)
            return True
        except:
            return False

    def go_to_arm_joint_state(self, joint_values):
        try:
            arm_states = JointState()
            arm_states.position = joint_values
            # try:
            self.move_group.set_joint_value_target(arm_states.position)
            # except Exception as e:
            #     rospy.loginfo(e)
            self.move_group.go(wait=True)
            """self.move_gripper.execute(self.plan_gripper, wait=True)"""
            self.move_group.stop()
            self.move_group.clear_pose_targets()
            rospy.sleep(2)
            return True
        except:
            return False

    def main(self):

        # Set up path here

        # Pick planner
        self.set_planner_type("RRT")
        self.move_group.set_named_target('Home')
        self.move_group.go(wait=True)

        ############## Grab the handle ###################

        # Open the Gripper
        rospy.loginfo('opening the gripper')
        self.go_to_finger_state('Open')

        # Go to handle
        rospy.loginfo('Going to palm link position')
        val = -RPY + 2 * 0.03
        for i in range(0, 2):
            val = val - i * 0.03
            # rospy.loginfo('Going to palm link position')
            # rospy.loginfo(val)
            self.go_to_goal([RPX, val, 1.15 - 0.53, 90, 90, 0])

        self.scene.remove_world_object('door')
        self.scene.remove_world_object('handle')

        ############# Going to finger link position ###################
        rospy.loginfo('Going to finger link position')
        self.go_to_finger_state("Close")
        ############# Press down the handle ###################

        rospy.loginfo('Press down the handle')

        cpose = self.move_group.get_current_pose().pose
        for i in range(0, 4):
            cpose.position.z -=  i * 0.01
            self.cartesian_pose_client(
                [cpose.position.x, cpose.position.y, cpose.position.z], [
                    cpose.orientation.x, cpose.orientation.y,
                    cpose.orientation.z, cpose.orientation.w
                ], "root")
        rospy.sleep(2)
        ###### Get Waypoints ###########

        ###### Plan path ##########

        waypoints = []
        thetas = []
        t = 0

        # First plan
        for i in range(0, 10):  # In current scene max value of theta is 46
            t = t + 2
            thetas.append((t / 180.0) * np.pi)

        # rospy.loginfo('current pose: ')
        wpose = self.move_group.get_current_pose().pose
        # rospy.loginfo(wpose)
        wx = wpose.position.x
        wy = wpose.position.y
        for val in thetas:
            wpose.position.x = DL * math.cos(val) - (DL - wx)
            wpose.position.y = wy - DL * math.sin(val)
            waypoints.append(copy.deepcopy(wpose))

        # rospy.loginfo('waypoints: ', waypoints)

        fraction = 0.0  #路径规划覆盖率
        maxtries = 100  #最大尝试规划次数
        attempts = 0  #已经尝试规划次数

        # 设置机器臂当前的状态作为运动初始状态
        self.move_group.set_start_state_to_current_state()

        # 尝试规划一条笛卡尔空间下的路径，依次通过所有路点，完成圆弧轨迹
        while fraction < 1.0 and attempts < maxtries:
            (plan, fraction) = self.move_group.compute_cartesian_path(
                waypoints,  # waypoint poses，路点列表
                0.01,  # eef_step，终端步进值
                0.0,  # jump_threshold，跳跃阈值
                True)  # avoid_collisions，避障规划

            # 尝试次数累加
            attempts += 1

            # 打印运动规划进程
            if attempts % 10 == 0:
                rospy.loginfo("Still trying after " + str(attempts) +
                              " attempts...")

        # 如果路径规划成功（覆盖率100%）,则开始控制机械臂运动
        if fraction == 1.0:
            rospy.loginfo("Path computed successfully. Moving the arm.")
            self.move_group.execute(plan, wait=True)
            rospy.loginfo("Path execution complete.")
        # 如果路径规划失败，则打印失败信息
        else:
            rospy.loginfo("Path planning failed with only " + str(fraction) +
                          " success after " + str(maxtries) + " attempts.")

        rospy.sleep(1)

        self.go_to_finger_state('Open')

        position = [
            0.8694887757301331, -0.12675708532333374, 0.5372458100318909
        ]
        quaternion = [-0.028, 0.724, 0.120, 0.678]
        rospy.loginfo("PUSH!")
        self.cartesian_pose_client(position, quaternion, 'j2s7s300_link_base')

        # self.move_group.set_goal_position_tolerance(0.30)
        # self.move_group.set_goal_orientation_tolerance(0.3)
        # waypoints = []

        # thetas = []
        # t = 0

        # # First plan
        # for i in range(10, 40):  # In current scene max value of theta is 46
        #     t = t + 2
        #     thetas.append((t / 180.0) * np.pi)

        # # rospy.loginfo('current pose: ')
        # wpose = self.move_group.get_current_pose().pose
        # # rospy.loginfo(wpose)
        # wx = wpose.position.x
        # wy = wpose.position.y
        # for val in thetas:

        #     wpose.position.x = DL * math.cos(val) - (DL - wx)
        #     wpose.position.y = wy - DL * math.sin(val)
        #     waypoints.append(copy.deepcopy(wpose))

        # # rospy.loginfo('waypoints: ', waypoints)

        # fraction = 0.0  #路径规划覆盖率
        # maxtries = 200  #最大尝试规划次数
        # attempts = 0  #已经尝试规划次数

        # # 设置机器臂当前的状态作为运动初始状态
        # self.move_group.set_start_state_to_current_state()

        # # 尝试规划一条笛卡尔空间下的路径，依次通过所有路点，完成圆弧轨迹
        # while fraction < 1.0 and attempts < maxtries:
        #     (plan, fraction) = self.move_group.compute_cartesian_path(
        #         waypoints,  # waypoint poses，路点列表
        #         0.01,  # eef_step，终端步进值
        #         0.0,  # jump_threshold，跳跃阈值
        #         True)  # avoid_collisions，避障规划

        #     # 尝试次数累加
        #     attempts += 1

        #     # 打印运动规划进程
        #     if attempts % 10 == 0:
        #         rospy.loginfo("Still trying after " + str(attempts) +
        #                       " attempts...")

        # # 如果路径规划成功（覆盖率100%）,则开始控制机械臂运动
        # if fraction > 0.688:
        #     rospy.loginfo("Path computed successfully. Moving the arm.")
        #     self.move_group.execute(plan, wait=True)
        #     rospy.loginfo("Path execution complete.")
        # # 如果路径规划失败，则打印失败信息
        # else:
        #     rospy.loginfo("Path planning failed with only " + str(fraction) +
        #                   " success after " + str(maxtries) + " attempts.")

        # rospy.sleep(1)

        # rospy.loginfo('Final pose: ')
        # wpose = self.move_group.get_current_pose().pose
        # rospy.loginfo(wpose)
        rospy.loginfo("Go home.")
        self.move_group.set_named_target('Home')
        self.move_group.go(wait=True)

        # self.homeRobot()


if __name__ == '__main__':
    MoveRobot()