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
import cv2.aruco as aruco
import webbrowser
import time
import math
from marsworks_vis_2.msg import Tvec
class Nodo(object):

    x_mid = 0
    y_mid = 0
    counter = 1
    sum_mid_x = 0
    sum_mid_y = 0
    tvec_sum = 0
    def __init__(self):
        # Params
        self.image = None
        self.br = CvBridge()
        # Node cycle rate (in Hz).
        self.loop_rate = rospy.Rate(1)
        # Publishers
        self.pub = rospy.Publisher('imagetimer', Image,queue_size=10)
        
        # publish tvec values
        self.pub2 = rospy.Publisher('/realsense_tvec', Tvec, queue_size=10)
        
        # Subscribers
        rospy.Subscriber("/camera/color/image_raw",Image,self.callback)
        

    
    def callback(self, msg):
        
        # rospy.loginfo('Image received...')
        self.image = cv2.cvtColor(self.br.imgmsg_to_cv2(msg), cv2.COLOR_RGB2BGR)
        # qrFound = self.findQRCode(self.image)
        arucofound, tvec = self.findArucoMarkers(self.image)

        ## only if there are two aruco markers
        if  len(arucofound[0])==2: 
            self.tvec_sum = np.floor((tvec[1][0] + tvec[0][0])/2)
            # print(tvec_sum)

            x_mid = math.floor((((arucofound[0][0][0][0][0] + arucofound[0][0][0][2][0])/2) + ((arucofound[0][1][0][0][0] + arucofound[0][1][0][2][0])/2))/2)
            y_mid = math.floor((((arucofound[0][0][0][0][1] + arucofound[0][0][0][2][1])/2) + ((arucofound[0][1][0][0][1] + arucofound[0][1][0][2][1])/2))/2)

            cv2.circle(self.image,(x_mid,y_mid),10,(255,0,0),-1)
            

            # for bbox, id in zip(arucofound[0], arucofound[1]):
            #     # print(bbox[0][0])
            #     # print(arucofound[0][0][0][0]) #top left corner
            #     # print(arucofound[0][0][0][2]) #top left corner
            #     # print(arucofound[0][1][0][0]) #top left corner
            #     # print(arucofound[0][1][0][2]) #bottom right corner

        ## only if there are two aruco markers
        # if  len(arucofound[0])==5: 
        #         # bottom 2 markers
        #         x_mid = math.floor((((arucofound[0][0][0][0][0] + arucofound[0][0][0][2][0])/2) + ((arucofound[0][1][0][0][0] + arucofound[0][1][0][2][0])/2))/2)
        #         y_mid = math.floor((((arucofound[0][0][0][0][1] + arucofound[0][0][0][2][1])/2) + ((arucofound[0][1][0][0][1] + arucofound[0][1][0][2][1])/2))/2)
        #         cv2.circle(self.image,(x_mid,y_mid),10,(0,255,0),-1)
        #         # bottom left and top left markers
        #         x_mid = math.floor((((arucofound[0][0][0][0][0] + arucofound[0][0][0][2][0])/2) + ((arucofound[0][4][0][0][0] + arucofound[0][4][0][2][0])/2))/2)
        #         y_mid = math.floor((((arucofound[0][0][0][0][1] + arucofound[0][0][0][2][1])/2) + ((arucofound[0][4][0][0][1] + arucofound[0][4][0][2][1])/2))/2)
        #         cv2.circle(self.image,(x_mid,y_mid),10,(0,0,255),-1)
        #         x_mid = math.floor((((arucofound[0][1][0][0][0] + arucofound[0][1][0][2][0])/2) + ((arucofound[0][3][0][0][0] + arucofound[0][3][0][2][0])/2))/2)
        #         y_mid = math.floor((((arucofound[0][1][0][0][1] + arucofound[0][1][0][2][1])/2) + ((arucofound[0][3][0][0][1] + arucofound[0][3][0][2][1])/2))/2)
        #         cv2.circle(self.image,(x_mid,y_mid),10,(255,0,0),-1)
        
        ## estimating 4 buttons
        # if  len(arucofound[0])==5: 
        #     # print(math.floor((((arucofound[0][0][0][0][0] + arucofound[0][0][0][2][0])/2) + ((arucofound[0][1][0][0][0] + arucofound[0][1][0][2][0])/2))/2))
        #     # print(math.floor((((arucofound[0][0][0][0][1] + arucofound[0][0][0][2][1])/2) + ((arucofound[0][1][0][0][1] + arucofound[0][1][0][2][1])/2))/2))
        #     self.x_mid = math.floor((((arucofound[0][2][0][0][0] + arucofound[0][2][0][2][0])/2)))
        #     self.y_mid = math.floor((((arucofound[0][2][0][0][1] + arucofound[0][2][0][2][1])/2)))
        #     self.sum_mid_x = self.sum_mid_x + self.x_mid
        #     self.sum_mid_y = self.sum_mid_y + self.y_mid
        #     print(self.sum_mid_x, self.sum_mid_y, self.counter)
        #     cv2.circle(self.image,(math.floor(self.sum_mid_x/self.counter),math.floor(self.sum_mid_y/self.counter)),10,(100,100,0),-1)
        #     self.counter+=1

        # # DETECTING COLOURS
        # # lower bound and upper bound for red color
        # lower_bound = np.array([0,50,50])
        # upper_bound = np.array([10,255,255])
        # # find the colors within the boundaries
        # mask = cv2.inRange(self.image, lower_bound, upper_bound)


        self.show_image(self.image)

    def send_tvec_msg(self,tvec_sum):
        msg = Tvec()
        msg.x = int(tvec_sum[0])
        msg.y = int(tvec_sum[1])
        msg.z = int(tvec_sum[2])
        self.pub2.publish(msg)
    
    def findArucoMarkers(self, img, markerSize = 4, totalMarkers=1000, draw=True):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        key = getattr(aruco, f'DICT_{markerSize}X{markerSize}_{totalMarkers}')
        arucoDict = aruco.Dictionary_get(key)
        arucoParam = aruco.DetectorParameters_create()
        bboxs, ids, rejected = aruco.detectMarkers(gray, arucoDict, parameters = arucoParam)

        mtx = [[451.217323, 0, 324.155156], [0, 452.905225, 234.169672], [0, 0, 1]]
        mtx = np.array(mtx)
        dist = [-0.040918, 0.028285, -0.005463, 0.000918, 0]
        dist = np.array(dist)
        markerSizeInMM = 41.5
        rvec , tvec, _ = aruco.estimatePoseSingleMarkers(bboxs, markerSizeInMM, mtx, dist)
        # print(tvec,ids)
        if draw:
            aruco.drawDetectedMarkers(img, bboxs)
        return [bboxs, ids], tvec
    
    # def findQRCode(self, img):
    #     gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    #     ##
    #     # qr code detection code
    #     detector = cv2.QRCodeDetector()
    #     data, points, _ = detector.detectAndDecode(gray)
    #     if points is not None:
    #         print('Decoded data: ' + data)
    #         points = points[0]
    #         for i in range(len(points)):
    #             pt1 = [int(val) for val in points[i]]
    #             pt2 = [int(val) for val in points[(i + 1) % 4]]
    #             cv2.line(img, pt1, pt2, color=(255, 0, 0), thickness=1)
    #     ##
    #     return
        
    def show_image(self, img):
        cv2.imshow("Image Window", img)
        cv2.waitKey(3)        

    def start(self):
        rospy.loginfo("Timing images")
        #rospy.spin()
        while not rospy.is_shutdown():
            rospy.loginfo('publishing image')
            #br = CvBridge()
            if self.image is not None:
                self.pub.publish(self.br.cv2_to_imgmsg(self.image))
                self.send_tvec_msg(self.tvec_sum)
            self.loop_rate.sleep()

if __name__ == '__main__':
    rospy.init_node("imagetimer111", anonymous=True)
    my_node = Nodo()
    my_node.start()
