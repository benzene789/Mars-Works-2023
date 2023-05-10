# #Return global coordinates of location of button/switch

# #!/user/bin/env
# import rospy
# from std_msgs.msg import String
# from panela.msg import Position
# def talk_to_me():
# 	pub = rospy.Publisher('MOVEIT TOPICCC', Position, queue_size=10)
# 	rospy.init_node('publisher_node', anonymous=True)
# 	rate = rospy.Rate(1)
# 	rospy.loginfo("Publisher Node Started, now publishing messages")
# 	while not rospy.is_shutdown():
# 		msg = Position()
# 		msg.x = 3.0
# 		msg.y = 3.0
# 		msg.z = 3.0
# 		pub.publish(msg)
# 		rate.sleep()

# if __name__ == '__main__':
# 	try:
# 		talk_to_me()
# 	except rospy.ROSInterruptException:
# 		pass

from __future__ import print_function
from six.moves import input

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))



from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list


moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("move_group_python_interface_tutorial", anonymous=True)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_name = "rover_arm"
move_group = moveit_commander.MoveGroupCommander(group_name)

display_trajectory_publisher = rospy.Publisher(
    "/move_group/display_planned_path",
    moveit_msgs.msg.DisplayTrajectory,
    queue_size=20,
)

joint_goal = move_group.get_current_joint_values()
joint_goal[0] = 0
joint_goal[1] = 0
joint_goal[2] = 0
joint_goal[3] = 0
joint_goal[4] = 0

# The go command can be called with joint values, poses, or without any
# parameters if you have already set the pose or joint target for the group
move_group.go(joint_goal, wait=True)

# Calling ``stop()`` ensures that there is no residual movement
move_group.stop()

move_group.clear_pose_targets()

# print("============ Printing robot state")
# print(robot.get_current_state())
# print("")

# pose_goal = geometry_msgs.msg.Pose()
# pose_goal.orientation.w = 1.0
# pose_goal.position.x = 0.1
# pose_goal.position.y = 0.1
# pose_goal.position.z = 0.1 
# move_group.set_pose_target(pose_goal)

# plan = move_group.go(wait=True)
# # Calling `stop()` ensures that there is no residual movement
# move_group.stop()
# # It is always good to clear your targets after planning with poses.
# # Note: there is no equivalent function for clear_joint_value_targets()
# move_group.clear_pose_targets()