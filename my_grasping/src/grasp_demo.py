#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

robot = moveit_commander.RobotCommander()

arm_group = moveit_commander.MoveGroupCommander("arm")
arm_group.set_named_target("home")
plan1 = arm_group.go()




pose_goal = geometry_msgs.msg.Pose()
pose_goal.orientation.w = 1.0
pose_goal.position.x = 0.4
pose_goal.position.y = 0
pose_goal.position.z = 0.2

arm_group.set_pose_target(pose_goal)
plan1 = arm_group.go()



rospy.sleep(5)
moveit_commander.roscpp_shutdown()
