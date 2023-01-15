#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Jan  14 10:24:00 2022
@author: ghak
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from road_bot.reg_defines import *


class StereoVisionNode(Node):

    def __init__(self):
        super().__init__('stereo_vision_node')
        
        self.subscription1 = self.create_subscription(Image, 'camera1', self.callback_camera1, 10)
        self.subscription2 = self.create_subscription(Image, 'camera2', self.callback_camera2, 10)
        self.cvbr = CvBridge()
        
        self.calibrate = False
        self.frame1 = ""
        self.frame2 = ""
        self.init1 = False
        self.init2 = False
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.checkerboard = (4,6)
        self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        
        self.flags = 0
        self.flags |= cv2.CALIB_FIX_INTRINSIC
 
        self.criteria_stereo= (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
 
        self.objp = np.zeros((1, self.checkerboard[0] * self.checkerboard[1], 3), np.float32)
        self.objp[0,:,:2] = np.mgrid[0:self.checkerboard[0], 0:self.checkerboard[1]].T.reshape(-1, 2)
        self.objp=self.objp*4.0
        
        self.imgpts1 = []
        self.imgpts2 = []
        self.objpts = []
        
        self.Left_Stereo_Map = 0
        self.Left_Stereo_Map = 0

  

    def timer_callback(self):
        if self.init1 and self.init2:
            frame1 = self.frame1.copy()
            frame2 = self.frame2.copy()
            
            h1,w1,_ = frame1.shape
            h2,w2,_ = frame2.shape
            if h1>h2 :
                frame2= cv2.resize(frame2, (w1,h1), interpolation = cv2.INTER_AREA)
            elif h2>h1 :
                frame1= cv2.resize(frame1, (w2,h2), interpolation = cv2.INTER_AREA)
            
            self.init1=False
            self.init1=False
            if not self.calibrate:
                gray1 = cv2.cvtColor(frame1, cv2.COLOR_BGR2GRAY)
                gray2 = cv2.cvtColor(frame2, cv2.COLOR_BGR2GRAY)
                ret1, corners1 = cv2.findChessboardCorners(gray1, self.checkerboard, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)
                ret2, corners2 = cv2.findChessboardCorners(gray2, self.checkerboard, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)
                if ret1 and ret2:
                        self.objpts.append(self.objp)
                        cornersSubPix1 = cv2.cornerSubPix(gray1, corners1, (11,11),(-1,-1), self.criteria)
                        cornersSubPix2 = cv2.cornerSubPix(gray2, corners2, (11,11),(-1,-1), self.criteria)
                        
                        frame1 = cv2.drawChessboardCorners(frame1, self.checkerboard, cornersSubPix1, ret1)
                        frame2 = cv2.drawChessboardCorners(frame2, self.checkerboard, cornersSubPix2, ret2)
                        
                        self.imgpts1.append(cornersSubPix1)
                        self.imgpts2.append(cornersSubPix2)
                        
                if len(self.objpts) == 25:
                    self.calibrate = True
                    
                    ret1, mtx1, dist1, rvecs1, tvecs1 = cv2.calibrateCamera(self.objpts,self.imgpts1,gray1.shape[::-1],None,None)
                    h1,w1= gray1.shape[:2]
                    new_mtx1, roi1= cv2.getOptimalNewCameraMatrix(mtx1,dist1,(w1,h1),1,(w1,h1))
                    
                    ret2, mtx2, dist2, rvecs2, tvecs2 = cv2.calibrateCamera(self.objpts,self.imgpts2,gray2.shape[::-1],None,None)
                    h2,w2= gray2.shape[:2]
                    new_mtx2, roi2= cv2.getOptimalNewCameraMatrix(mtx2,dist2,(w2,h2),1,(w2,h2))
                    
                    retS, new_mtx1, dist1, new_mtx2, dist2, Rot, Trns, Emat, Fmat = cv2.stereoCalibrate(self.objpts, self.imgpts1, self.imgpts2, new_mtx1, dist1, new_mtx2, dist2, gray1.shape[::-1], self.criteria_stereo, self.flags)
                    
                    rectify_scale= 1
                    rect_1, rect_2, proj_mat_1, proj_mat_2, Q, roi1, roi2= cv2.stereoRectify(new_mtx1, dist1, new_mtx2, dist2, gray1.shape[::-1], Rot, Trns, rectify_scale,(0,0))
                    
                    self.Left_Stereo_Map= cv2.initUndistortRectifyMap(new_mtx1, dist1, rect_1, proj_mat_1,
                                             gray1.shape[::-1], cv2.CV_16SC2)
                    
                    self.Right_Stereo_Map= cv2.initUndistortRectifyMap(new_mtx2, dist2, rect_2, proj_mat_2,
                                                                  gray2.shape[::-1], cv2.CV_16SC2)
                     
                    print("Saving paraeters ......")
                    cv_file = cv2.FileStorage("improved_params2.xml", cv2.FILE_STORAGE_WRITE)
                    cv_file.write("Left_Stereo_Map_x",self.Left_Stereo_Map[0])
                    cv_file.write("Left_Stereo_Map_y",self.Left_Stereo_Map[1])
                    cv_file.write("Right_Stereo_Map_x",self.Right_Stereo_Map[0])
                    cv_file.write("Right_Stereo_Map_y",self.Right_Stereo_Map[1])
                    cv_file.release()

            if self.calibrate:
                #cv2.imshow("Left image before rectification", frame1)
                #cv2.imshow("Right image before rectification", frame2)
                 
                Left_nice= cv2.remap(frame1,self.Left_Stereo_Map[0],self.Left_Stereo_Map[1], cv2.INTER_LANCZOS4, cv2.BORDER_CONSTANT, 0)
                Right_nice= cv2.remap(frame2,self.Right_Stereo_Map[0],self.Right_Stereo_Map[1], cv2.INTER_LANCZOS4, cv2.BORDER_CONSTANT, 0)
                 
                cv2.imshow("Left image after rectification", Left_nice)
                cv2.imshow("Right image after rectification", Right_nice)
                
                 
                out = Right_nice.copy()
                out[:,:,0] = Right_nice[:,:,0]
                out[:,:,1] = Right_nice[:,:,1]
                out[:,:,2] = Left_nice[:,:,2]
                 
                cv2.imshow("Output image", out)
            
            conc = cv2.hconcat([frame1, frame2])
            cv2.imshow('frame', conc)
            cv2.waitKey(1)
        
        
    def callback_camera1(self, data):
        self.init1=True
        self.frame1 = self.cvbr.imgmsg_to_cv2(data)
        
    def callback_camera2(self, data):
        self.init2=True
        self.frame2 = self.cvbr.imgmsg_to_cv2(data)
        

def main(args=None):
    rclpy.init(args=args)
    
    stereo_vision_node = StereoVisionNode()

    rclpy.spin(stereo_vision_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    rpi_camera_publisher.cap.release()
    cv2.destroyAllWindows()
    stereo_vision_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
