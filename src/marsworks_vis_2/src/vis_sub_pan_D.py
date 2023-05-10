#! /usr/bin/env python3
import rospy
from std_msgs.msg import String
from marsworks_vis_2.msg import Tvec
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import time
import math
import os

class Subscriber():

    def send_arm_traj(self, tvec):
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.w = 1
        pose_goal.position.x = round((tvec.x/1000)+0.3,2)
        pose_goal.position.y = 0 #math.round(tvec.y/1000,2)
        pose_goal.position.z = round((tvec.z/1000),2)
        # pose_goal.position.x = 0.8
        # pose_goal.position.y = 0
        # pose_goal.position.z = 0.3
        self.move_group.set_pose_target(pose_goal)
        self.plan = self.move_group.go(wait=True)
        print(pose_goal)

        time.sleep(1)


        # manually adjusting joints when dealing with parameters
        joint_goal = self.move_group.get_current_joint_values()
        joint_goal[3] = 1
        self.move_group.go(joint_goal, wait=True)
        
        scale = 1
        self.pose_target = self.move_group.get_current_pose().pose
        self.pose_target.position.x += scale *0.1
        waypoints = []
        waypoints.append(self.pose_target)
        (plan, fraction) = self.move_group.compute_cartesian_path(waypoints,0.01,0.0)
        self.move_group.execute(plan, wait=True)
        time.sleep(5)



        self.move_group.stop()
        self.move_group.clear_pose_targets()


        

    def callback(self, tvec):
        self.send_arm_traj(tvec)
        rospy.sleep(5)
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(tvec)
        os._exit(0)

    def __init__(self):

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()    
        self.move_group = moveit_commander.MoveGroupCommander("rover_arm")
        self.display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        self.sub = rospy.Subscriber('realsense_tvec', Tvec, self.callback)

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook) 
        # rospy.loginfo(f"The '{self.node_name}' node is active...")

    def shutdownhook(self):
        # print(f"Stopping the '{self.node_name}' node at: {rospy.get_time()}")
        self.ctrl_c = True


    def start(self):
        rospy.spin()

if __name__ == '__main__':
    subscriber_instance = Subscriber()
    time.sleep(2)
    subscriber_instance.start()
