#!/usr/bin/env python

import sys

import rospy
import math3d
import moveit_commander
import actionlib

import geometry_msgs.msg
from std_msgs.msg import Float64
from kinova_msgs.msg import SetFingersPositionAction
from kinova_msgs.msg import SetFingersPositionActionGoal

from kinova_unit_app.srv import GetObject
import jaco2_pick_place


class PickPlace:
    def __init__(self):
        self.__arm_group = moveit_commander.MoveGroupCommander('arm')
        self.__gripper_command_pub = rospy.Publisher('/j2s7s300_driver/fingers_action/finger_positions/goal', SetFingersPositionActionGoal, queue_size=10)
        self._grasp_client = actionlib.SimpleActionClient("j2s7s300_driver/finger_action/finger_positions", SetFingersPositionAction)
        self.object_pose_sub = rospy.Subscriber("/object_pose", geometry_msgs.msg.PoseArray, self._callback)
        
        self.pose_objects = geometry_msgs.msg.PoseArray()
        rospy.loginfo("arm setup")
        self.__set_gripper(False)
        rospy.sleep(1.0)
        self.__set_gripper(True)
        rospy.sleep(1.0)
        # self.__move_by_name('home')
        # rospy.sleep(1.0)

        self.__object_service_proxy = rospy.ServiceProxy('/object_detect_node/request_first_object', GetObject)

    def _callback(self, msg):
        self.pose_objects = msg

    def __convert_to_transform(self, pose_stamped):
        quaternion = math3d.UnitQuaternion(pose_stamped.pose.orientation.w, pose_stamped.pose.orientation.x, pose_stamped.pose.orientation.y, pose_stamped.pose.orientation.z)
        transform = math3d.Transform()
        transform.set_pos((pose_stamped.pose.position.x, pose_stamped.pose.position.y, pose_stamped.pose.position.z))
        transform.set_orient(quaternion.orientation)
        return transform

    def __convert_to_pose(self, t):
        pose = geometry_msgs.msg.PoseStamped()
        pose.pose.position.x = t.pos.x
        pose.pose.position.y = t.pos.y
        pose.pose.position.z = t.pos.z
        pose.pose.orientation.w = t.orient.quaternion[0]
        pose.pose.orientation.x = t.orient.quaternion[1]
        pose.pose.orientation.y = t.orient.quaternion[2]
        pose.pose.orientation.z = t.orient.quaternion[3]
        return pose

    def __set_gripper(self, state):
        finger_goal = SetFingersPositionActionGoal()
        if state:
            finger_goal.goal.fingers.finger1 = 1000.0
            finger_goal.goal.fingers.finger2 = 1000.0
            self.__gripper_command_pub.publish(finger_goal)
            # self._grasp_client.send_goal(finger_goal)
        else:
            finger_goal.goal.fingers.finger1 = 4500.0
            finger_goal.goal.fingers.finger2 = 4500.0
            self.__gripper_command_pub.publish(finger_goal)
            # self._grasp_client.send_goal(finger_goal)

    def __move_by_pose(self, target_pose):
        self.__arm_group.set_pose_target(target_pose)
        return self.__arm_group.go(wait=True)

    def __move_by_name(self, target_name):
        self.__arm_group.set_named_target(target_name)
        return self.__arm_group.go(wait=True)

    def pick_place(self, target_pose):
        pass

    def run(self):

        pick_place = jaco2_pick_place.Jaco2PickPlace('j2s7s300')
        rospy.sleep(2)
        rospy.loginfo("request object pose...")

        rospy.loginfo('set arm to home pose')
        if not pick_place.ready():
            return
        rospy.sleep(1.0)

        try:
            res = self.__object_service_proxy(request_type=0)
        except rospy.ServiceException:
            rospy.loginfo("no object")
            return


        rospy.loginfo("found object")

        object_transform = self.__convert_to_transform(res.object_pose)

        standby_offset = math3d.Transform()
        # standby_offset.pos = math3d.Vector(-0.03, 0.08, -0.05)
        standby_offset.pos = math3d.Vector(0.0, 0.0, -0.06)
        standby_transform = object_transform * standby_offset
        standby_pose_stamped = self.__convert_to_pose(standby_transform)
        standby_pose_stamped.header.frame_id = res.object_pose.header.frame_id

        pick_offset = math3d.Transform()
        pick_offset.pos = math3d.Vector(0.0, 0.0, 0.025)
        # pick_offset.pos = math3d.Vector(-0.03, 0.08, -0.025)
        pick_transform = object_transform * pick_offset
        pick_pose_stamped = self.__convert_to_pose(pick_transform)
        pick_pose_stamped.header.frame_id = res.object_pose.header.frame_id

        pose_place = geometry_msgs.msg.PoseStamped()
        pose_place.header.frame_id = "j2s7s300_link_base"
        pose_place.pose.position.x = -0.286990189552
        pose_place.pose.position.y = -0.453882825375
        pose_place.pose.position.z = 0.105741010904
        pose_place.pose.orientation.w = 0.0568502917886
        pose_place.pose.orientation.x =  0.94724547863
        pose_place.pose.orientation.y = -0.315277636051
        pose_place.pose.orientation.z = -0.00969966873527

        pose_place_stanby = geometry_msgs.msg.PoseStamped()
        pose_place_stanby.header.frame_id = "j2s7s300_link_base"
        pose_place_stanby.pose.position.x = -0.286990189552
        pose_place_stanby.pose.position.y = -0.453882825375
        pose_place_stanby.pose.position.z = 0.201631618738
        pose_place_stanby.pose.orientation.w = 0.0514585077763
        pose_place_stanby.pose.orientation.x =  0.94907283783
        pose_place_stanby.pose.orientation.y = -0.310767769814
        pose_place_stanby.pose.orientation.z = -0.00600713305175

        rospy.loginfo("object pose: %s, %s, %s", res.object_pose.pose.position.x, res.object_pose.pose.position.y, res.object_pose.pose.position.z)
        rospy.loginfo("standby pose: %s, %s, %s", standby_pose_stamped.pose.position.x, standby_pose_stamped.pose.position.y, standby_pose_stamped.pose.position.z)
        # rospy.loginfo("%s, %s, %s", pick_pose_stamped.pose.position.x, pick_pose_stamped.pose.position.y, pick_pose_stamped.pose.position.z)

        self.__set_gripper(True)
        rospy.sleep(2.0)

        if self.__move_by_pose(standby_pose_stamped) is not True:
            rospy.loginfo("unable to get standby pose")
            return
        # rospy.sleep(2.0)

        if self.__move_by_pose(pick_pose_stamped) is not True:
            rospy.loginfo("unable to get object pose")
            return
        # rospy.sleep(2.0)

        self.__set_gripper(False)
        rospy.sleep(2.0)

        if self.__move_by_pose(standby_pose_stamped) is not True:
            rospy.loginfo("unable to get standby pose")
            return

        # rospy.sleep(2.0)

        if self.__move_by_pose(pose_place_stanby) is not True:
            rospy.loginfo("unable to get place standby pose")
            return

        if self.__move_by_pose(pose_place) is not True:
            rospy.loginfo("unable to get place pose")
            return

        self.__set_gripper(True)
        rospy.sleep(2.0)

        if self.__move_by_pose(pose_place_stanby) is not True:
            rospy.loginfo("unable to get place standby pose")
            return

        rospy.loginfo('set arm to home pose')
        if not pick_place.home():
            return
        rospy.sleep(2.0)

        # if self.__move_by_name('home') is not True:
        #     rospy.loginfo("unable to get home pose")
        #     return

        # rospy.loginfo('set arm to home pose')
        # if not pick_place.ready():
        #     return

        rospy.loginfo("finish")


def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('kinova_unit_app_node')

    pick_place = PickPlace()

    rate = rospy.Rate(0.4)
    while not rospy.is_shutdown():
        pick_place.run()
        rate.sleep()


if __name__ == '__main__':
    main()
