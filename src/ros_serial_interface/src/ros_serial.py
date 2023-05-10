#!/usr/bin/env python3
# Importing Libraries
import rospy
from std_msgs.msg import String
import serial
import time
arduino = serial.Serial(port='/dev/ttyACM0', baudrate=115200, timeout=.1)

class Publisher():

    def write_read(self,x):
        arduino.write(bytes(x, 'utf-8'))
        time.sleep(0.05)
        data = arduino.readline()
        return data
    
    def __init__(self):
        self.node_name = "simple_publisher"
        topic_name = "arduino"

        self.pub = rospy.Publisher(topic_name, String, queue_size=10)
        rospy.init_node(self.node_name, anonymous=True)
        self.rate = rospy.Rate(10) # hz
                
        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook) 
        
        rospy.loginfo(f"The '{self.node_name}' node is active...")

    def shutdownhook(self):
        print(f"Stopping the '{self.node_name}' node at: {rospy.get_time()}")
        self.ctrl_c = True

    def main_loop(self):
        while not self.ctrl_c:
            num = input("Enter a number: ") # Taking input from user
            value = self.write_read(num)
            #print(int(value)) # printing the value
            publisher_message = f"{int(value)}"
            self.pub.publish(publisher_message)
            self.rate.sleep()

if __name__ == '__main__':
    publisher_instance = Publisher()
    try:
        publisher_instance.main_loop()
    except rospy.ROSInterruptException:
        pass