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
        arucofound, aruco_tvec, img= self.findArucoMarkers(image)
        # qrFound, qr_points = self.findQRCode(image)
        if  len(arucofound[0])==1: 
            tvec_sum = np.floor(aruco_tvec[0][0])
            # print(tvec_sum)
            x_mid_aruco = math.floor((arucofound[0][0][0][0][0] + arucofound[0][0][0][2][0])/2)
            y_mid_aruco = math.floor((arucofound[0][0][0][0][1] + arucofound[0][0][0][2][1])/2)
            x_line_aruco = math.floor((arucofound[0][0][0][0][0] + arucofound[0][0][0][1][0])/2)
            y_line_aruco = math.floor((arucofound[0][0][0][0][1] + arucofound[0][0][0][1][1])/2)
            # x_mid_qr = math.floor((qr_points[0][0] + qr_points[1][0] + qr_points[2][0] + qr_points[3][0])/4)
            # y_mid_qr = math.floor((qr_points[0][1] + qr_points[1][1] + qr_points[2][1] + qr_points[3][1])/4)
            # print(qr_points)
            cv2.circle(img,(x_mid_aruco,y_mid_aruco),2,(255,0,0),-1)
            theta = math.atan2((y_line_aruco-y_mid_aruco),(x_line_aruco-x_mid_aruco))
            dist_aruco = math.sqrt((y_line_aruco-y_mid_aruco)*(y_line_aruco-y_mid_aruco)+(x_line_aruco-x_mid_aruco)*(x_line_aruco-x_mid_aruco))
            factor = dist_aruco/25
            dist_1 = math.sqrt((100*100)+(100*100))+5 # offset
            theta_1 = math.atan2(100,100)
            x_dial_l = math.floor(x_mid_aruco+factor*dist_1*math.cos(theta-theta_1))
            y_dial_l = math.floor(y_mid_aruco+factor*dist_1*math.sin(theta-theta_1))
            x_dial_r = math.floor(x_mid_aruco+factor*dist_1*math.cos(theta+theta_1))
            y_dial_r = math.floor(y_mid_aruco+factor*dist_1*math.sin(theta+theta_1))
            cv2.circle(img,(x_dial_l,y_dial_l),2,(255,0,0),-1)
            cv2.circle(img,(x_dial_r,y_dial_r),2,(255,0,0),-1)
            # cv2.circle(img,(x_mid_qr,y_mid_qr),2,(0,0,255),-1)
            cv2.imwrite('dials.jpeg', img)
            ############### to be pratically tested #################
            xtvec_dial_l = math.floor(tvec_sum[0]+dist_1*math.cos(theta-theta_1))
            ytvec_dial_l = math.floor(tvec_sum[0]+dist_1*math.sin(theta-theta_1))
            xtvec_dial_r = math.floor(tvec_sum[0]+dist_1*math.cos(theta+theta_1))
            ytvec_dial_r = math.floor(tvec_sum[0]+dist_1*math.sin(theta+theta_1))
            tvec_dial_l = [xtvec_dial_l,ytvec_dial_l,tvec_sum[2]]
            tvec_dial_r = [xtvec_dial_r,ytvec_dial_r,tvec_sum[2]]
            print([tvec_dial_l,tvec_dial_r])
            time.sleep(2)
            self.send_tvec_msg(tvec_dial_l,1)
            self.send_tvec_msg(tvec_dial_r,2)

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
        detector = cv2.QRCodeDetector()
        data, points, _ = detector.detectAndDecode(gray)
        if points is not None:
            # print('Decoded data: ' + data)
            points = points[0]
            for i in range(len(points)):
                pt1 = [int(val) for val in points[i]]
                pt2 = [int(val) for val in points[(i + 1) % 4]]
                cv2.line(img, pt1, pt2, color=(255, 0, 0), thickness=1)
        return data, points

    def send_tvec_msg(self,tvec_sum,c):
        msg = Tvec()
        msg.x = int(tvec_sum[0])
        msg.y = int(tvec_sum[1])
        msg.z = int(tvec_sum[2])
        if c == 1:
            self.pub1.publish(msg)
        elif c == 2:
            self.pub2.publish(msg)

    def main(self):
        self.pub1 = rospy.Publisher('/realsense_tvec_panel_b_dial_l', Tvec, queue_size=10)
        self.pub2 = rospy.Publisher('/realsense_tvec_panel_b_dial_r', Tvec, queue_size=10)
        time.sleep(2)
        self.sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('panel_b_dial')
    my_node = panel()
    my_node.main()