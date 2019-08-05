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

# We can get the joint values from the group and adjust some of the values:
joint_goal = arm_group.get_current_joint_values()
joint_goal[0] = 0
joint_goal[1] = -pi/4
joint_goal[2] = 0
joint_goal[3] = -pi/2
joint_goal[4] = 0
joint_goal[5] = 0

# The go command can be called with joint values, poses, or without any
# parameters if you have already set the pose or joint target for the group
arm_group.go(joint_goal, wait=True)

# Calling ``stop()`` ensures that there is no residual movement
arm_group.stop()



rospy.sleep(5)
moveit_commander.roscpp_shutdown()
