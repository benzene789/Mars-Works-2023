#! /usr/bin/env python3

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import time

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()    
move_group = moveit_commander.MoveGroupCommander("rover_arm")
display_trajectory_publisher = rospy.Publisher(
    "/move_group/display_planned_path",
    moveit_msgs.msg.DisplayTrajectory,
    queue_size=20,
)


#random pose goal
pose_goal = geometry_msgs.msg.Pose()
pose_goal.orientation.w = 1.0
pose_goal.position.x = 0.8
pose_goal.position.y = 0
pose_goal.position.z = 0.71



# pose_goal.position.x = 0.278+0.3
# pose_goal.position.y = 0.073
# pose_goal.position.z = 0.268


move_group.set_pose_target(pose_goal)
plan = move_group.go(wait=True)

joint_goal = move_group.get_current_joint_values()
joint_goal[3] = 1.4
move_group.go(joint_goal, wait=True)
time.sleep(1)
time.sleep(1)

move_group.stop()

move_group.clear_pose_targets()

# plan2 = move_group.plan()
# group.go(wait=True)

rospy.sleep(5)

moveit_commander.roscpp_shutdown()