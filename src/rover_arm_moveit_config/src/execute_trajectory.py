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

joint_goal = move_group.get_current_joint_values()
# park
joint_goal[0] = 0
joint_goal[1] = 0.10472 # lower limit 0.10472
joint_goal[2] = 0
joint_goal[3] = 0
joint_goal[4] = 0
move_group.go(joint_goal, wait=True)
time.sleep(1)
# park2
joint_goal[0] = 0
joint_goal[1] = 0.261799 # lower limit 0.10472
joint_goal[2] = 0.767945
joint_goal[3] = 3.14159
joint_goal[4] = 0
move_group.go(joint_goal, wait=True)
time.sleep(1)
#random pose goal
# pose_goal = geometry_msgs.msg.Pose()
# pose_goal.orientation.w = 1.0
# pose_goal.position.x = 1
# pose_goal.position.y = 0
# pose_goal.position.z = 0.2

# move_group.set_pose_target(pose_goal)
# plan = move_group.go(wait=True)
scale = 1
pose_target = move_group.get_current_pose().pose
pose_target.position.z += scale *-0.75
waypoints = []
waypoints.append(pose_target)
(plan, fraction) = move_group.compute_cartesian_path(waypoints,0.01,0.0)
move_group.execute(plan, wait=True)
time.sleep(1)

joint_goal = move_group.get_current_joint_values()
joint_goal[3] = 0.25
move_group.go(joint_goal, wait=True)
time.sleep(1)

pose_target = move_group.get_current_pose().pose
pose_target.position.z += scale *0.75
waypoints = []
waypoints.append(pose_target)
(plan, fraction) = move_group.compute_cartesian_path(waypoints,0.01,0.0) 
move_group.execute(plan, wait=True)
time.sleep(1)
# park
joint_goal = move_group.get_current_joint_values()
joint_goal[0] = 0
joint_goal[1] = 0.10472 # lower limit 0.10472
joint_goal[2] = 0
joint_goal[3] = 0.75
joint_goal[4] = 0
move_group.go(joint_goal, wait=True)
time.sleep(1)

joint_goal = move_group.get_current_joint_values()
joint_goal[0] = 3.14159
joint_goal[1] = 0.10472 # lower limit 0.10472
joint_goal[2] = 0
joint_goal[3] = 0.75
joint_goal[4] = 0
move_group.go(joint_goal, wait=True)
time.sleep(1)

joint_goal = move_group.get_current_joint_values()
joint_goal[0] = 3.14159
joint_goal[1] = 0.872665 # lower limit 0.10472
joint_goal[2] = 0.872665
joint_goal[3] = 1
joint_goal[4] = 0
move_group.go(joint_goal, wait=True)
time.sleep(1)

joint_goal = move_group.get_current_joint_values()
joint_goal[0] = 3.14159
joint_goal[1] = 0.872665 # lower limit 0.10472
joint_goal[2] = 0.872665
joint_goal[3] = 2.44346
joint_goal[4] = 0
move_group.go(joint_goal, wait=True)
time.sleep(1)

joint_goal = move_group.get_current_joint_values()
# park
joint_goal[0] = 0
joint_goal[1] = 0.10472 # lower limit 0.10472
joint_goal[2] = 0
joint_goal[3] = 0
joint_goal[4] = 0
move_group.go(joint_goal, wait=True)
time.sleep(1)

move_group.stop()

move_group.clear_pose_targets()

# plan2 = move_group.plan()
# group.go(wait=True)

rospy.sleep(5)

moveit_commander.roscpp_shutdown()