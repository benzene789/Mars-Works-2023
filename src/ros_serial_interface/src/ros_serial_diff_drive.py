#!/user/bin/env
from pickle import FALSE
import serial
import rospy
from std_msgs.msg import Int32
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

arduino = serial.Serial(port="/dev/ttyACM0", baudrate=115200, timeout=.1)

class Subscriber():
    def write_read(self,x):
        arduino.write(bytes(x, 'utf-7'))
        data = arduino.readline()
        print(x)
        time.sleep(0.00000001)
        return data

    def callback_l(self, data):
        if data >= 0:
            self.ldir = 1
        else:
            self.ldir = 2
        self.lspeed = abs(data)
        # rospy.loginfo(self.lspeed)

    def callback_r(self, data):
        if data >= 0:
            self.rdir = 1
        else:
            self.rdir = 2
        self.rspeed = abs(data)
        # rospy.loginfo(self.rspeed)

    def __init__(self):
        self.node_name = "diff_drive_arduino"
        rospy.init_node(self.node_name)
        self.rate = rospy.Rate(50) # hz
        self.sub_l = rospy.Subscriber("lwheel_ticks", Int32, self.callback_l)
        self.sub_r = rospy.Subscriber("rwheel_ticks", Int32, self.callback_r)
        y_1 = [20, 10, int(self.lspeed), int(self.rspeed), 0, 0, 0, 0]
        y_2 = [20, 20, self.ldir, self.rdir, 0, 0, 0, 0]
        speeds = str(y_1)[1:-1]
        dirs = str(y_2)[1:-1]
        speeds_arduino = self.write_read(speeds)
        dirs_arduino = self.write_read(dirs)
        rospy.loginfo(speeds_arduino)
        rospy.loginfo(dirs_arduino)
        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook) 

    def shutdownhook(self):
        print(f"Stopping the '{self.node_name}' node at: {rospy.get_time()}")
        self.ctrl_c = True


    def main_loop(self):
        rospy.spin()

if __name__ == '__main__':
    subscriber_instance = Subscriber()
    time.sleep(1)
    subscriber_instance.main_loop()
