#!/usr/bin/env python3

from cv2 import QRCodeDetector
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

            # self.sub.unregister()
            
    def show_image(self, img):
        cv2.imshow("Image Window", img)
        cv2.waitKey(3)  

    def aruco_QR(self,image):
        arucofound, aruco_tvec, img= self.findArucoMarkers(image)

        self.show_image(image)
        if  len(arucofound[0])==5: #need to change to 5
            # print(arucofound[0][1])
            # print(aruco_tvec[2][0])
            ############# buttons ###################
            data, points = self.findQRCode(img)
            x_mid_aruco = math.floor((arucofound[0][2][0][0][0] + arucofound[0][2][0][2][0])/2)
            y_mid_aruco = math.floor((arucofound[0][2][0][0][1] + arucofound[0][2][0][2][1])/2)
            x_line_aruco = math.floor((arucofound[0][2][0][0][0] + arucofound[0][2][0][1][0])/2)
            y_line_aruco = math.floor((arucofound[0][2][0][0][1] + arucofound[0][2][0][1][1])/2)
            cv2.circle(image,(x_mid_aruco,y_mid_aruco),2,(255,0,0),-1)
            # cv2.circle(image,(x_line_aruco,y_line_aruco),2,(255,0,0),-1)
            theta = math.atan2((y_line_aruco-y_mid_aruco),(x_line_aruco-x_mid_aruco))
            dist_aruco = math.sqrt((y_line_aruco-y_mid_aruco)*(y_line_aruco-y_mid_aruco)+(x_line_aruco-x_mid_aruco)*(x_line_aruco-x_mid_aruco))
            factor = dist_aruco/25
            dist_1 = math.sqrt((50*50)+(50*50)) # offset
            theta_1 = math.atan2(50,50)
            dim1, dim2 = (2, 4) 
            button = [[0 for i in range(dim1)] for j in range(dim2)] 
            button[0][0] = math.floor(x_mid_aruco+factor*dist_1*math.cos(theta-theta_1))
            button[0][1] = math.floor(y_mid_aruco+factor*dist_1*math.sin(theta-theta_1))
            button[1][0] = math.floor(x_mid_aruco+factor*dist_1*math.cos(theta+theta_1))
            button[1][1] = math.floor(y_mid_aruco+factor*dist_1*math.sin(theta+theta_1))
            button[2][0] = math.floor(x_mid_aruco+factor*dist_1*math.cos(-theta+theta_1))
            button[2][1] = math.floor(y_mid_aruco+factor*dist_1*math.sin(-theta+theta_1))
            button[3][0] = math.floor(x_mid_aruco+factor*dist_1*math.cos(-theta-theta_1))
            button[3][1] = math.floor(y_mid_aruco+factor*dist_1*math.sin(-theta-theta_1))
            data = "2-1-4-3" ######### just an example & qr doesn't work
            data = data.split("-")
            data = [int(item)-1 for item in data]
            # print(data)
            dim1, dim2 = (3, 4) 
            tvec_button = [[0 for i in range(dim1)] for j in range(dim2)] 
            for i in range(0,4):
                cv2.circle(img,(button[i][0],button[i][1]),2,(255,0,0),-1)
                tvec_button[data[i]][0] = button[data[i]][0]*aruco_tvec[2][0][0]/x_mid_aruco
                tvec_button[data[i]][1] = button[data[i]][1]*aruco_tvec[2][0][1]/y_mid_aruco
                tvec_button[data[i]][2] = aruco_tvec[2][0][2]
            # print(tvec_button)

            ############ usb ################
            x1_mid_aruco = math.floor((arucofound[0][0][0][0][0] + arucofound[0][0][0][2][0])/2)
            y1_mid_aruco = math.floor((arucofound[0][0][0][0][1] + arucofound[0][0][0][2][1])/2)
            x2_mid_aruco = math.floor((arucofound[0][1][0][0][0] + arucofound[0][1][0][2][0])/2)
            y2_mid_aruco = math.floor((arucofound[0][1][0][0][1] + arucofound[0][1][0][2][1])/2)
            x_midpt = math.floor((x1_mid_aruco + x2_mid_aruco)/2)
            y_midpt = math.floor((y1_mid_aruco + y2_mid_aruco)/2)
            cv2.circle(img,(x_midpt,y_midpt),2,(255,0,0),-1)
            dim1, dim2 = (3, 1) 
            tvec_usb = [[0 for i in range(dim1)] for j in range(dim2)] 
            tvec_usb[0][0] = x_midpt*aruco_tvec[0][0][0]/x1_mid_aruco
            tvec_usb[0][1] = y_midpt*aruco_tvec[0][0][1]/y1_mid_aruco
            tvec_usb[0][2] = aruco_tvec[0][0][2]
            self.panel_A_execution(tvec_button,tvec_usb)

            cv2.imwrite('usb.jpeg', img)
            self.sub.unregister()

    def panel_A_execution(self, button, usb):
        print(button)
        print(usb)


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

    def findQRCode(self, img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # qr code detection code
        detector = cv2.QRCodeDetector()
        data, points, _ = detector.detectAndDecode(gray)
        if points is not None:
            print('Decoded data: ' + data)
            points = points[0]
            for i in range(len(points)):
                pt1 = [int(val) for val in points[i]]
                pt2 = [int(val) for val in points[(i + 1) % 4]]
                cv2.line(img, pt1, pt2, color=(255, 0, 0), thickness=1)
        ##
        return data, points

    def send_tvec_msg(self,tvec_sum,c):
        msg = Tvec()
        msg.x = int(tvec_sum[0])
        msg.y = int(tvec_sum[1])
        msg.z = int(tvec_sum[2])
        self.pub.publish(msg)

    def main(self):
        self.pub = rospy.Publisher('/realsense_tvec_panel_a', Tvec, queue_size=10)
        print("main")
        time.sleep(2)
        self.sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('panel_a_usb')
    my_node = panel()
    my_node.main()