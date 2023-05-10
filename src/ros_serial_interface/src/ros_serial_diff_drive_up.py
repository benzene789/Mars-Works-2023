#!/user/bin/env
from pickle import FALSE
import serial
import rospy
from std_msgs.msg import Int32
import math
import time
import os


arduino = serial.Serial(port="/dev/ttyACM1", baudrate=115200, timeout=.1)

class Subscriber():
    flspeed = 0
    rlspeed = 0
    frspeed = 0
    rrspeed = 0
    def write_read(self,x):
        arduino.write(bytes(x, 'utf-7'))
        data = arduino.readline()
        print(x)
        time.sleep(0.00000001)
        return data

    def callback_l(self, data):
        data = str(data).split(":")
        self.data = int(data[1])
        # print(self.data)
        if self.data >= 0:
            self.flspeed = abs(self.data)/100
        else:
            self.rlspeed = abs(self.data)/200




    def callback_r(self, data_1):
        data_1 = str(data_1).split(":")
        self.data_1 = int(data_1[1])
        if self.data_1 >= 0:
            self.frspeed = abs(self.data_1)/100
        else:
            self.rrspeed = abs(self.data_1)/200
    

    def __init__(self):
        self.node_name = "diff_drive_arduino"
        rospy.init_node(self.node_name)
        self.rate = rospy.Rate(0.25) # hz
        while 1<2:
            
            self.sub_l = rospy.Subscriber("lwheel_desired_rate", Int32, self.callback_l)
            # print("l")
            self.sub_r = rospy.Subscriber("rwheel_desired_rate", Int32, self.callback_r)
            # print("r")
            if self.sub_l is None or self.sub_r is None:
                print("none")
            else:
                if (85-int(self.flspeed))==85 and (85-int(self.frspeed))==85 and (85-int(self.rlspeed)) == 85 and (85-int(self.rrspeed))==85:
                    y = [20, 10, 0, 0, 0, 0, 0, 0]
                elif (85-int(self.flspeed))==85 and (85-int(self.frspeed))==85:
                    y = [20, 10, 0, 0, abs(85-int(self.rlspeed)), abs(85-int(self.rrspeed)), 0, 0]
                elif (85-int(self.rlspeed))==85 and (85-int(self.rrspeed))==85:
                    y = [20, 10, 85-int(self.flspeed), 85-int(self.frspeed), 0,0, 0, 0]
                else:
                    y = [20, 10, 85-int(self.flspeed), 85-int(self.frspeed), abs(85-int(self.rlspeed)), abs(85-int(self.rrspeed)), 0, 0]

                for x in range(2, len(y)):
                    if y[x]>85:
                        y[x]=85
                    if y[x]<65 and y[x]!=0:
                        y[x]=65
                speeds = str(y)[1:-1]
                print(speeds)
                speeds_arduino = self.write_read(speeds)
                rospy.loginfo(speeds_arduino)
            self.flspeed = 0
            self.rlspeed = 0
            self.frspeed = 0
            self.rrspeed = 0
            time.sleep(1)
            self.sub_l.unregister()
            self.sub_r.unregister()
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
