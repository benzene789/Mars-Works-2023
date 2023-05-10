#!/usr/bin/env python3
# A simple ROS subscriber node in Python

#source ~/marsworks_official/catkin_ws/devel/setup.bash
#roslaunch realsense2_camera rs_camera.launch
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class Nodo(object):

    def __init__(self):
        # Params
        self.image = None
        self.br = CvBridge()
        # Node cycle rate (in Hz).
        self.loop_rate = rospy.Rate(1)
        self.pub = rospy.Publisher("realsense_viewer", Image, queue_size=10)
        rospy.Subscriber("/camera/color/image_raw",Image,self.callback)

    def callback(self, msg):
        rospy.loginfo('Image received...')
        self.image = cv2.cvtColor(self.br.imgmsg_to_cv2(msg), cv2.COLOR_RGB2BGR)
        height = self.image.shape[0]
        width = self.image.shape[1]
        # print(height,width)
        cv2.line(self.image,(int(width/2),height),(int(width/2),0),(0,0,255),1)
        # self.show_image(self.image)
        
    def show_image(self, img):
        cv2.imshow("Image Window", img)
        cv2.waitKey(3)        

    def start(self):
        # rospy.spin()
        while not rospy.is_shutdown():
            if self.image is not None:
                self.pub.publish(self.br.cv2_to_imgmsg(self.image, encoding="rgb8"))
            self.loop_rate.sleep()

if __name__ == '__main__':
    rospy.init_node("realsense_viewer")
    my_node = Nodo()
    my_node.start()
