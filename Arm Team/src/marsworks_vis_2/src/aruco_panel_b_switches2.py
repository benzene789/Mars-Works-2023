#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import time
import numpy as np
import math
import cv2.aruco as aruco
from marsworks_vis_2.msg import Tvec
import sys
bridge = CvBridge()

class panel(object):
    
    def image_callback(self,msg):
        print("Received an image!")
        try:
            cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)
        else:
            self.aruco_QR(cv2_img)
            if  len(self.arucofound[0])==1: 
                self.sub.unregister()
                print(self.arucofound)            

    def aruco_QR(self,image):

        self.arucofound, aruco_tvec, img= self.findArucoMarkers(image)
        if  len(self.arucofound[0])==1: 
            tvec_sum = np.floor(aruco_tvec[0][0])
            # print(tvec_sum)
            x_mid_aruco = math.floor((self.arucofound[0][0][0][0][0] + self.arucofound[0][0][0][2][0])/2)
            y_mid_aruco = math.floor((self.arucofound[0][0][0][0][1] + self.arucofound[0][0][0][2][1])/2)
            x_line_aruco = math.floor((self.arucofound[0][0][0][0][0] + self.arucofound[0][0][0][1][0])/2)
            y_line_aruco = math.floor((self.arucofound[0][0][0][0][1] + self.arucofound[0][0][0][1][1])/2)

            cv2.circle(img,(x_mid_aruco,y_mid_aruco),2,(255,0,0),-1)

            # self.show_image(image)

        # https://dev.to/erol/object-detection-with-color-knl
        # https://stackoverflow.com/questions/51531414/opencv-findcontours-just-returning-one-external-contour
        # https://stackoverflow.com/questions/36817133/identifying-the-range-of-a-color-in-hsv-using-opencv
        # RED HSV
        
        redLower = (0,50,50)
        redUpper = (10,255,255)

        purpleLower = (130, 50, 50)
        purpleUpper = (145, 255, 255)

        # redLower = (0,50,50)
        # redUpper = (10,255,255)

        # purpleLower = (130, 50, 50)
        # purpleUpper = (145, 255, 255)


        # blur
        blurred = cv2.GaussianBlur(image, (11,11), 0)

        # hsv
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # mask for red
        mask1 = cv2.inRange(hsv, redLower, redUpper)
        # deleting noises which are in area of mask
        mask1 = cv2.erode(mask1, None, iterations=2)
        mask1 = cv2.dilate(mask1, None, iterations=2)


        # mask for purple
        mask2 = cv2.inRange(hsv, purpleLower, purpleUpper)
        # deleting noises which are in area of mask
        mask2 = cv2.erode(mask2, None, iterations=2)
        mask2 = cv2.dilate(mask2, None, iterations=2)


        # target = cv2.bitwise_or(mask1, mask2)
        # # deleting noises which are in area of mask
        # target = cv2.erode(target, None, iterations=2)
        # target = cv2.dilate(target, None, iterations=2)




        for mask in [ mask2]:
            contours, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for i, contour in enumerate(contours):
                center = None
                rect = cv2.minAreaRect(contour)

                # box
                box = cv2.boxPoints(rect)
                box = np.int64(box)
                
                # moment
                M = cv2.moments(contour)
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

                # point in center
                cv2.circle(image, center, 5, (255, 0, 255), -1)

                # draw contour
                cv2.drawContours(image, [box], 0, (0, 255, 255), 2)
        
        tvec_dial = [center[0]*tvec_sum[0]/x_mid_aruco,center[1]*tvec_sum[1]/y_mid_aruco,tvec_sum[2]]
        print(tvec_dial)

        cv2.imwrite('switches.jpeg', img)


    def findArucoMarkers(self, img, markerSize = 4, totalMarkers=1000, draw=True):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        key = getattr(aruco, f'DICT_{markerSize}X{markerSize}_{totalMarkers}')
        arucoDict = aruco.Dictionary_get(key)
        arucoParam = aruco.DetectorParameters_create()
        bboxs, ids, rejected = aruco.detectMarkers(gray, arucoDict, parameters = arucoParam)
        # print(bboxs)
        mtx = [[451.217323, 0, 324.155156], [0, 452.905225, 234.169672], [0, 0, 1]]
        mtx = np.array(mtx)
        dist = [-0.040918, 0.028285, -0.005463, 0.000918, 0]
        dist = np.array(dist)
        markerSizeInMM = 41.5
        rvec , tvec, _ = aruco.estimatePoseSingleMarkers(bboxs, markerSizeInMM, mtx, dist)
        if draw:
            aruco.drawDetectedMarkers(img, bboxs)
        return [bboxs, ids], tvec, img

    def show_image(self, img):
        cv2.imshow("Image Window", img)
        cv2.waitKey(3)  

    def send_tvec_msg(self,tvec_sum,c):
        msg = Tvec()
        msg.x = int(tvec_sum[0])
        msg.y = int(tvec_sum[1])
        msg.z = int(tvec_sum[2])
        if c == 1:
            self.pub1.publish(msg)
        elif c == 2:
            self.pub2.publish(msg)
        elif c == 3:
            self.pub3.publish(msg)
        elif c == 4:
            self.pub4.publish(msg)

    def main(self):
        self.pub1 = rospy.Publisher('/realsense_tvec_panel_b_switch_1', Tvec, queue_size=10)
        self.pub2 = rospy.Publisher('/realsense_tvec_panel_b_switch_2', Tvec, queue_size=10)
        self.pub3 = rospy.Publisher('/realsense_tvec_panel_b_switch_3', Tvec, queue_size=10)
        self.pub4 = rospy.Publisher('/realsense_tvec_panel_b_switch_4', Tvec, queue_size=10)
        time.sleep(2)
        self.sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('panel_b_switches')
    my_node = panel()
    my_node.main()