#!/user/bin/env

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import math


class Subscriber():

    def callback(self, data):
        y = [int(math.degrees(x)) for x in data.position]
        t = tuple(y)
        rospy.loginfo(t)

    def __init__(self):
        self.node_name = "Subscriber_Node"
        topic_name = "move_group/fake_controller_joint_states"

        rospy.init_node(self.node_name, anonymous=True)
        self.sub = rospy.Subscriber(topic_name, JointState, self.callback)

    def main_loop(self):
        rospy.spin()

if __name__ == '__main__':
    subscriber_instance = Subscriber()
    subscriber_instance.main_loop()

# import os
# stream = os.popen('dmesg')
# output = stream.readlines()
# # print(output)
# for x in output:
#     print(x)
# for i in reversed(output):
#     if "ttyACM0" in i:
#         print("/dev/ttyACM0")
#         break
#     elif "ttyACM1" in i:
#         print("/dev/ttyACM1")
#         break
