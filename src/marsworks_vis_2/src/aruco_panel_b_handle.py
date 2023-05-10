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
            self.sub.unregister()
            
    def aruco_QR(self,image):
        # image = cv2.imread("panel assem-1.png")
        # output = image.copy()
        # gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        # circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 0.1, 500)
        # if circles is not None:
        #     circles = np.round(circles[0, :]).astype("int")
        #     for (x, y, r) in circles:
        #         cv2.circle(output, (x, y), r, (0, 255, 0), 4)
        #         cv2.rectangle(output, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)
        #         self.show_image(output)
        #         cv2.imwrite('switches.jpeg', output)
        arucofound, aruco_tvec, img= self.findArucoMarkers(image)
        if  len(arucofound[0])==1: 
            tvec_sum = np.floor(aruco_tvec[0][0])
            # print(tvec_sum)
            x_mid_aruco = math.floor((arucofound[0][0][0][0][0] + arucofound[0][0][0][2][0])/2)
            y_mid_aruco = math.floor((arucofound[0][0][0][0][1] + arucofound[0][0][0][2][1])/2)
            x_line_aruco = math.floor((arucofound[0][0][0][0][0] + arucofound[0][0][0][1][0])/2)
            y_line_aruco = math.floor((arucofound[0][0][0][0][1] + arucofound[0][0][0][1][1])/2)
            cv2.circle(img,(x_mid_aruco,y_mid_aruco),2,(255,0,0),-1)
            theta = math.atan2((y_line_aruco-y_mid_aruco),(x_line_aruco-x_mid_aruco))
            # cv2.line(img,(x_mid_aruco,y_mid_aruco),(math.floor(x_mid_aruco+100*math.cos(theta)),math.floor(y_mid_aruco+100*math.sin(theta))),(255,0,0),2)
            dist_aruco = math.sqrt((y_line_aruco-y_mid_aruco)*(y_line_aruco-y_mid_aruco)+(x_line_aruco-x_mid_aruco)*(x_line_aruco-x_mid_aruco))
            factor = dist_aruco/25
            dist_1 = 76.25+5 # offset
            theta_1 = math.atan2(0,76.25)
            x_handle = math.floor(x_mid_aruco-factor*dist_1*math.cos(theta-theta_1))
            y_handle = math.floor(y_mid_aruco-factor*dist_1*math.sin(theta-theta_1))
            cv2.circle(img,(x_handle,y_handle),2,(255,255,0),-1)
            cv2.imwrite('handle.jpeg', img)
            ############### to be pratically tested #################
            xtvec_handle = math.floor(tvec_sum[0]+dist_1*math.cos(theta-theta_1))
            ytvec_handle = math.floor(tvec_sum[0]+dist_1*math.sin(theta-theta_1))
            switch_1 = [xtvec_handle,ytvec_handle,tvec_sum[2]]
            print(switch_1)
            # time.sleep(2)
            self.send_tvec_msg(switch_1,1)

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
        self.pub.publish(msg)

    def main(self):
        self.pub = rospy.Publisher('/realsense_tvec_panel_b_handle', Tvec, queue_size=10)
        time.sleep(2)
        self.sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('panel_b_handle')
    my_node = panel()
    my_node.main()