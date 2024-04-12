#!/usr/bin/env python

import rospy
import moveit_commander
from math import pi
import sys

if __name__ == "__main__":
    # Initialize ROS node
    rospy.init_node('moveit_example', anonymous=True)

    # Initialize moveit_commander
    moveit_commander.roscpp_initialize(sys.argv)

    # Create RobotCommander object to interact with the robot model
    robot = moveit_commander.RobotCommander()

    # Create PlanningSceneInterface object to manage the scene
    scene = moveit_commander.PlanningSceneInterface()

    # Create MoveGroupCommander object to control the manipulator
    group_name = "tm_arm"  # moveit configured group name
    move_group = moveit_commander.MoveGroupCommander(group_name)


    # Set target joint values
    target_joint_values = [-1.57, -0.72, 2, -1.309, 3.1, 0] #[-1.57 -0.72 2 -1.309 3.1 0]
    move_group.set_joint_value_target(target_joint_values)

    # Plan and execute the movement of the manipulator
    plan = move_group.plan()
    move_group.execute(plan)

