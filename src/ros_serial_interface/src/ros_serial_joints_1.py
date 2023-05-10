#!/user/bin/env
from pickle import FALSE
import serial
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import math
import time
import os

stream = os.popen('dmesg')
output = stream.readlines()
# print(output)
for i in reversed(output):
    if "cdc_acm 3-3.1:1.0: ttyACM0: USB ACM device" in i:
        print("/dev/ttyACM0")
        break
    elif "cdc_acm 3-3.1:1.0: ttyACM1: USB ACM device" in i:
        print("/dev/ttyACM0")
        break

arduino = serial.Serial(port="/dev/ttyACM1", baudrate=115200, timeout=.1)

class Subscriber():
    current_seq = 0
    move_joints = False
    def write_read(self,x):
        arduino.write(bytes(x, 'utf-7'))
        data = arduino.readline()
        print(x)
        time.sleep(0.00000001)
        return data

    def callback(self, data):
        y = [int(math.degrees(x)) for x in data.position]
        # y[1]= 104
        # y[2]= 65
        # y[3]= 0

        offseted_y = [40, 20, 0, 109-y[2], 111-y[1], y[3], y[4], 0]
        y_1 = [40, 30, 1, y[0], 0, 0, 0, 0]
        # y_1 = [40, 30, 0, y[0], 0, 0, 0, 0]
        t = tuple(offseted_y)
        t_1 = tuple(y_1)
        self.joints = str(offseted_y)[1:-1]
        self.base = str(y_1)[1:-1]
        rospy.loginfo(self.joints)
        self.value = self.write_read(self.joints)
        self.value = self.write_read(self.base)
        rospy.loginfo(self.value)

    def __init__(self):
        self.node_name = "Subscriber_Node"
        topic_name = "move_group/fake_controller_joint_states"

        # self.pub = rospy.Publisher(???, String, queue_size=10)
        rospy.init_node(self.node_name)
        self.rate = rospy.Rate(50) # hz
        self.sub = rospy.Subscriber(topic_name, JointState, self.callback)
        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook) 
        # rospy.loginfo(f"The '{self.node_name}' node is active...")

    def shutdownhook(self):
        print(f"Stopping the '{self.node_name}' node at: {rospy.get_time()}")
        self.ctrl_c = True


    def main_loop(self):
        # if not self.ctrl_c:
        #     # value = self.write_read(self.joints)
        #     self.write_read(self.joints)
        #     rospy.loginfo(self.joints)
        # self.rate.sleep()
        rospy.spin()

if __name__ == '__main__':
    subscriber_instance = Subscriber()
    time.sleep(1)
    subscriber_instance.main_loop()
