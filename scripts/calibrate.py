#!/usr/bin/env python3

"""! This script is a test of camera calibration for openCV.

"""

import sys
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import numpy as np
import time
import sympy as sym
class calibrate_test:

    def __init__(self):
        self.bridge = CvBridge()
        self.head_sub = rospy.Subscriber("cameras/head_camera/image",
                                         Image,
                                         self.save_head)
        self.real_sub = rospy.Subscriber("camera/color/image_raw",
                                         Image,
                                         self.save_real)
        self.real_image = ''
        self.head_image = ''
        self.head_valid = False
        self.real_valid = False

        self.m_per_side = .02237 #m per side of square
        self.rows = 6
        self.cols = 9
        
    def save_head(self, data):
        """! Process new incomming images from the head camera. 
        New images are saved for later use in calibration. 
        """
        try:
            self.head_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.head_valid = True
        except CvBridgeError as e: 
            rospy.logwarn("Image processing failure: " + str(e))

    def save_real(self, data):
        """! Process new incomming images from the head camera. 
        New images are saved for later use in calibration. 
        """
        try:
            self.real_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.real_valid = True
        except CvBridgeError as e: 
            rospy.logwarn("Image processing failure: " + str(e))

    def images_valid(self):
        return((self.head_valid and self.real_valid))

    def chessboard(self, images, dt = 0.5):
        imgs = []
        print("Collecting Images")
        for i in range (0, images):
            imgs.append([self.head_image, self.real_image])
            time.sleep(dt)
        print("Images Collected") 
        # Setup some requied params for calibration. 
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        objp = np.zeros((self.rows*self.cols,3), np.float32)
        objp[:,:2] = np.mgrid[0:self.rows,0:self.cols].T.reshape(-1,2)*self.m_per_side 
        objpoints = []
        imgpoints_h = []
        imgpoints_r = []
       
        for img in imgs:
            print("Processing Image")
            gray_h = cv2.cvtColor(img[0], cv2.COLOR_BGR2GRAY) 
            gray_r = cv2.cvtColor(img[1], cv2.COLOR_BGR2GRAY) 
            ret_h, corners_h = cv2.findChessboardCorners(gray_h, (self.cols,self.rows))
            ret_r, corners_r = cv2.findChessboardCorners(gray_r, (self.cols,self.rows))
        
            # If we have two valid detections then record.
            print(ret_h)
            print(ret_r)
            if ret_h and ret_r:
                print("Image Valid")
                objpoints.append(objp)
                corners_h = cv2.cornerSubPix(gray_h, corners_h,(3,3),(-1,-1),criteria)
                corners_r = cv2.cornerSubPix(gray_r, corners_r,(3,3),(-1,-1),criteria)
                img = cv2.drawChessboardCorners(img[1], (self.cols, self.rows),corners_r, ret_r)
                cv2.imshow('img',img)
                cv2.waitKey(1000)
                imgpoints_r.append(corners_r)
                imgpoints_h.append(corners_h)
        
        print("collection complete")
        retval_h, mtx_h, dist_h, rvecs_h, tvecs_h = cv2.calibrateCamera(objpoints, 
                                                                        imgpoints_h, 
                                                                        gray_h.shape[::-1],
                                                                        None,None)

        retval_r, mtx_r, dist_r, rvecs_r, tvecs_r = cv2.calibrateCamera(objpoints, 
                                                                        imgpoints_r, 
                                                                        gray_r.shape[::-1],
                                                                        None,None)

         
        print("translation")
        print(tvecs_h)
        print("Rotation")
        rotation_matrix = np.zeros(shape=(3,3))
        R_ave = rotation_matrix
        i = 0
        for r in rvecs_h:
            i += 1
            R_ave = R_ave +  cv2.Rodrigues(np.float32(r), rotation_matrix)[0]
        print(sym.Matrix(R_ave/i).inv()*sym.Matrix(-1*tvecs_r[0])) 

    
def main(): 
    ic = calibrate_test()
    rospy.init_node('calibrate_test')
    r = rospy.Rate(3)
    while not ic.images_valid() and not rospy.is_shutdown():
        r.sleep()
        print("waiting")

    ic.chessboard(15)

if __name__ == "__main__":
    main()
