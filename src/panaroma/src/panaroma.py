#!/usr/bin/env python3
# A simple ROS subscriber node in Python

#source ~/marsworks_official/catkin_ws/devel/setup.bash
#roslaunch realsense2_camera rs_camera.launch
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import numpy as np
import time
import imutils


class Nodo(object):

    def __init__(self):
        # Params
        self.image = None
        self.br = CvBridge()
        # Node cycle rate (in Hz).
        self.loop_rate = rospy.Rate(1)
        self.t = 0
        self.imgs = []
        time.sleep(2)
        # Subscribers
        rospy.Subscriber("/camera/color/image_raw",Image,self.callback)

    def callback(self, msg):
        rospy.loginfo('Image received...')
        self.image = cv2.cvtColor(self.br.imgmsg_to_cv2(msg), cv2.COLOR_RGB2BGR)
        time.sleep(1)
        self.imgs.append(self.image)
        self.t += 1
        print(self.t)
        if self.t == 25:
            print("stich")
            img = self.stitch(self.imgs)
            
            img = cv2.copyMakeBorder(img, 10, 10, 10, 10,cv2.BORDER_CONSTANT, (0, 0, 0))
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY)[1]
            cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cnts = imutils.grab_contours(cnts)
            c = max(cnts, key=cv2.contourArea)
            mask = np.zeros(thresh.shape, dtype="uint8")
            (x, y, w, h) = cv2.boundingRect(c)
            img = img[y:y + h, x:x + w]
            cv2.imwrite('output.png',img)

            # cv2.rectangle(mask, (x, y), (x + w, y + h), 255, -1)
            # minRect = mask.copy()
            # sub = mask.copy()
            # while cv2.countNonZero(sub) > 0:
            #     minRect = cv2.erode(minRect, None)
            #     sub = cv2.subtract(minRect, thresh)
            os._exit(0)
        # self.show_image(self.image)

    def stitch(self,images):
        stitchy=cv2.Stitcher.create()
        (dummy,output)=stitchy.stitch(images)
        if dummy != cv2.STITCHER_OK:
        # checking if the stitching procedure is successful
        # .stitch() function returns a true value if stitching is
        # done successfully
            print("stitching ain't successful")
            os._exit(0)
        else:
            print('Your Panorama is ready!!!')
            return output
        
    def show_image(self, img):
        cv2.imshow("Image Window", img)
        cv2.waitKey(3)        

    def start(self):
        rospy.spin()
        while not rospy.is_shutdown():
            self.loop_rate.sleep()

if __name__ == '__main__':
    rospy.init_node("panaroma")
    my_node = Nodo()
    my_node.start()
