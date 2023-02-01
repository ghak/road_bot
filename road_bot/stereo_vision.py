#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Jan  14 10:24:00 2022
@author: ghak
"""


import glob
import sys
import os
import cv2
import numpy as np

from scipy.spatial.transform import Rotation as R

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge


def save_xml_file(path, list_def, list_val):
    cv_file = cv2.FileStorage(path, cv2.FILE_STORAGE_WRITE)
    for i in range(len(list_val)):
        cv_file.write(list_def[i], list_val[i])
    cv_file.release()


def drawLines(img, lines, colors):
    _, c, _ = img.shape
    for r, color in zip(lines, colors):
        x0, y0 = map(int, [0, -r[2]/r[1]])
        x1, y1 = map(int, [c, -(r[2]+r[0]*c)/r[1]])
        cv2.line(img, (x0, y0), (x1, y1), color, 2)


def detect_chess_board(frame, size_CB, option):
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    h1, w1, _ = frame.shape
    resized = cv2.resize(frame, (w1*2, h1*2))
    gray_resized = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(resized, size_CB, None)
    if ret:
        cv2.cornerSubPix(gray_resized, corners, (11, 11), (-1, -1), criteria)
        corners = corners/2
        if option == 4:
            return corners[[0, size_CB[0]-1, (size_CB[1]-1)*size_CB[0], size_CB[0]*size_CB[1]-1], 0]
        else:
            return corners[:, :]

    return []


def custom_resize(frame, w2):
    h, w = frame.shape[0:2]
    ratio = h/w
    return cv2.resize(frame, (w2, int(w2*ratio)))


class StereoVisionNode(Node):

    def __init__(self):
        super().__init__('stereo_vision_node')

        self.subscription1 = self.create_subscription(
            Image, 'camera1', self.callback_camera1, 10)
        self.subscription2 = self.create_subscription(
            Image, 'camera2', self.callback_camera2, 10)
        self.depth_publisher = self.create_publisher(
            Image, 'depth_map', 10)
        self.imageL_publisher = self.create_publisher(
            Image, 'imageL', 10)
        self.imageR_publisher = self.create_publisher(
            Image, 'imageR', 10)
        self.cvbr = CvBridge()

        self.frame1 = ""
        self.frame2 = ""
        self.init1 = False
        self.init2 = False
        self.calibrate = False
        self.force_overwrite = False

        version = 'to_use'
        self.checkerboard = (7, 10)
        self.checkerboard_box_size = 2.5
        self.calib_path = "./calibration_images/" + \
            str(self.checkerboard[0])+"x" + \
            str(self.checkerboard[1])+"/"+version+"/"

        self.frame_id = 0
        self.Left_Stereo_Map = []
        self.Right_Stereo_Map = []
        self.cameraL_calibration = {}
        self.cameraR_calibration = {}
        self.stereo_calibration = {}
        self.rectification_vals = {}
        self.h_mats = []

        self.disparity = 0
        self.focale = 0
        self.baseline = 0
        self.stereo_matcher = 0

        self.landmark_frames1 = np.zeros((480, 640, 3), np.uint8)
        self.landmark_frames2 = np.zeros((480, 640, 3), np.uint8)

        self.calibrate_stereo()

        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def calibrate_stereo(self):

        if not self.force_overwrite:
            if (os.path.exists(self.calib_path+'stereo_calibration.xml')
                and os.path.exists(self.calib_path+'rectification_values.xml')
                and os.path.exists(self.calib_path+'improved_params2.xml')
                    and os.path.exists(self.calib_path+'homography_matrices.xml')):
                self.load_cameras_data(self.calib_path)
                self.calibrate = True
                return

        pathL = self.calib_path + "cameraL/"
        pathR = self.calib_path + "cameraR/"
        if not(os.path.exists(pathL) and os.path.exists(pathR)):
            self.get_logger().error(
                f'Folder \'{pathL}\' or \'{pathR}\' not found')
            sys.exit(1)
        fnamesL = sorted(glob.glob(pathL+'/*.png'))
        fnamesR = sorted(glob.glob(pathR+'/*.png'))
        if (len(fnamesL) == 0 or len(fnamesR) == 0):
            self.get_logger().error(
                f'No images in folder \'{pathL}\' or \'{pathR}\'found')
            sys.exit(1)

        error = 0.01
        criteria = (cv2.TERM_CRITERIA_EPS +
                    cv2.TERM_CRITERIA_MAX_ITER, 30, error)
        criteria_stereo = (cv2.TERM_CRITERIA_EPS +
                           cv2.TERM_CRITERIA_MAX_ITER, 30, error)
        conv_size = (11, 11)

        objp = np.zeros((np.prod(self.checkerboard), 3), np.float32)
        objp[:, :2] = np.indices(self.checkerboard).T.reshape(-1, 2)
        objp = objp*self.checkerboard_box_size

        good_list_l = []
        good_list_r = []
        good_list_all = []

        img_ptsL_alone = []
        img_ptsR_alone = []
        img_ptsL_group = []
        img_ptsR_group = []
        obj_ptsL = []
        obj_ptsR = []
        obj_pts = []

        h1, w1 = (0, 0)
        h2, w2 = (0, 0)

        list_image = range(len(fnamesL))

        print(self.calib_path)
        print(list_image)

        for i in list_image:
            imgL = cv2.imread(fnamesL[i])
            h1, w1, _ = imgL.shape
            resizedL = cv2.resize(imgL, (w1*2, h1*2))
            gray_resizedL = cv2.cvtColor(resizedL, cv2.COLOR_BGR2GRAY)
            retL, cornersL = cv2.findChessboardCorners(
                resizedL, self.checkerboard, None)
            if retL:
                good_list_l.append(i)
                cv2.cornerSubPix(gray_resizedL, cornersL,
                                 conv_size, (-1, -1), criteria)
                cornersL = cornersL/2
                obj_ptsL.append(objp)
                img_ptsL_alone.append(cornersL)

                cv2.drawChessboardCorners(
                    self.landmark_frames1, self.checkerboard, cornersL, retL)

            imgR = cv2.imread(fnamesR[i])
            h2, w2, _ = imgR.shape
            resizedR = cv2.resize(imgR, (w2*2, h2*2))
            gray_resizedR = cv2.cvtColor(resizedR, cv2.COLOR_BGR2GRAY)
            retR, cornersR = cv2.findChessboardCorners(
                resizedR, self.checkerboard, None)
            if retR:
                good_list_r.append(i)
                cv2.cornerSubPix(gray_resizedR, cornersR,
                                 conv_size, (-1, -1), criteria)
                cornersR = cornersR/2
                obj_ptsR.append(objp)
                img_ptsR_alone.append(cornersR)

                cv2.drawChessboardCorners(
                    self.landmark_frames2, self.checkerboard, cornersR, retR)

            cv2.imshow('Landmarks', np.vstack(
                (self.landmark_frames1, self.landmark_frames2)))

            if retR and retL:
                good_list_all.append(i)
                obj_pts.append(objp)
                img_ptsL_group.append(cornersL)
                img_ptsR_group.append(cornersR)

            cv2.waitKey(10)

        print("only left :" + str(list(set(good_list_l) - set(good_list_r))))
        print("only right :" + str(set(good_list_r) - set(good_list_l)))
        print("anywhere :" + str(set(list_image) -
              set(set(good_list_r+good_list_l))))

        print("*************************Intrinsic Calibration************************")
        retL, mtxL, distL, rvecsL, tvecsL = cv2.calibrateCamera(
            obj_ptsL, img_ptsL_alone, (w1, h1), None, None)
        print("Camera1 Error = "+str(retL))
        # mtxL, roiL= cv2.getOptimalNewCameraMatrix(mtxL,distL,(w1,h1),0,(w1,h1))

        # Calibrating right camera
        retR, mtxR, distR, rvecsR, tvecsR = cv2.calibrateCamera(
            obj_ptsR, img_ptsR_alone, (w2, h2), None, None)
        print("Camera2 Error = "+str(retR))
        # mtxR, roiR= cv2.getOptimalNewCameraMatrix(mtxR,distR,(w2,h2),0,(w2,h2))

        print("*************************Extrinsic Calibration************************")
        flags = 0
        flags |= cv2.CALIB_FIX_INTRINSIC
# =============================================================================
#         flags = (cv2.CALIB_FIX_INTRINSIC
#                  + cv2.CALIB_FIX_ASPECT_RATIO
#                  + cv2.CALIB_ZERO_TANGENT_DIST
#                  + cv2.CALIB_RATIONAL_MODEL
#                  + cv2.CALIB_FIX_K3 + cv2.CALIB_FIX_K4 + cv2.CALIB_FIX_K5)
# =============================================================================
        # This step is performed to transformation between the two cameras and calculate Essential and Fundamenatl matrix
        retS, mtxL, distL, mtxR, distR, Rot, Trns, Emat, Fmat = cv2.stereoCalibrate(obj_pts,
                                                                                    img_ptsL_group,
                                                                                    img_ptsR_group,
                                                                                    mtxL,
                                                                                    distL,
                                                                                    mtxR,
                                                                                    distR,
                                                                                    (w1, h1),
                                                                                    criteria_stereo, flags)
        save_xml_file(self.calib_path+"stereo_calibration.xml", ['retL', 'retR', 'retS', 'mtxL', 'distL', 'mtxR',
                                                                 'distR', 'Rot', 'Trns', 'Emat', 'Fmat'],
                      [retL, retR, retS, mtxL, distL, mtxR, distR, Rot, Trns, Emat, Fmat])
        print("Stereo Error = "+str(retS))

        self.cameraL_calibration['intrinsic_matrix'] = mtxL
        self.cameraL_calibration['distortion_coef'] = distL
        self.cameraR_calibration['intrinsic_matrix'] = mtxR
        self.cameraR_calibration['distortion_coef'] = distR

        self.stereo_calibration["rotation_matrix"] = Rot
        self.stereo_calibration["translation_vector"] = Trns
        self.stereo_calibration["essential_matrix"] = Emat
        self.stereo_calibration["fundamental_matrix"] = Fmat

        # pdb.set_trace()
        print("*************************Homography matrixs calculation************************")
        img_ptsL_group = np.asarray(img_ptsL_group).reshape((-1, 2))
        img_ptsR_group = np.asarray(img_ptsR_group).reshape((-1, 2))
        retH, H1, H2 = cv2.stereoRectifyUncalibrated(np.float32(
            img_ptsL_group), np.float32(img_ptsR_group), Fmat, (w1, h1))
        save_xml_file(self.calib_path+"homography_matrices.xml", ['retH', 'camera1_homography_matrix',
                                                                  'camera2_homography_matrix'],
                      [retH, H1, H2])
        self.h_mats.append(H1)
        self.h_mats.append(H2)
        print("Homography error = "+str(retH))

        print("*************************Stereo Rectification calculation************************")
        rect_l, rect_r, proj_mat_l, proj_mat_r, Q, roiL, roiR = cv2.stereoRectify(self.cameraL_calibration['intrinsic_matrix'],
                                                                                  self.cameraL_calibration['distortion_coef'],
                                                                                  self.cameraR_calibration['intrinsic_matrix'],
                                                                                  self.cameraR_calibration['distortion_coef'],
                                                                                  (w1, h1),
                                                                                  self.stereo_calibration["rotation_matrix"],
                                                                                  self.stereo_calibration["translation_vector"],
                                                                                  flags=cv2.CALIB_ZERO_DISPARITY,
                                                                                  alpha=1,
                                                                                  newImageSize=(0, 0))

        self.rectification_vals["rect_l"] = rect_l
        self.rectification_vals["rect_r"] = rect_r
        self.rectification_vals["proj_mat_l"] = proj_mat_l
        self.rectification_vals["proj_mat_r"] = proj_mat_r
        self.rectification_vals["disparity"] = Q
        self.rectification_vals["roiL"] = roiL
        self.rectification_vals["roiR"] = roiR
        save_xml_file(self.calib_path+"rectification_values.xml", ['rect_l', 'rect_r', 'proj_mat_l', 'proj_mat_r',
                                                                   'disparity', 'roiL', 'roiR'],
                      [rect_l, rect_r, proj_mat_l, proj_mat_r, Q, roiL, roiR])

        # self.rectification_vals ["proj_mat_l"][1,2] = 0
        # self.rectification_vals ["proj_mat_r"][1,2] = 0

        self.Left_Stereo_Map = cv2.initUndistortRectifyMap(self.cameraL_calibration['intrinsic_matrix'],
                                                           self.cameraL_calibration['distortion_coef'],
                                                           self.rectification_vals["rect_l"],
                                                           self.rectification_vals["proj_mat_l"],
                                                           (w1*2, h1*2), cv2.CV_16SC2)
        self.Right_Stereo_Map = cv2.initUndistortRectifyMap(self.cameraR_calibration['intrinsic_matrix'],
                                                            self.cameraR_calibration['distortion_coef'],
                                                            self.rectification_vals["rect_r"],
                                                            self.rectification_vals["proj_mat_r"],
                                                            (w1*2, h1*2), cv2.CV_16SC2)

        save_xml_file(self.calib_path+"improved_params2.xml", ['Left_Stereo_Map_x', 'Left_Stereo_Map_y',
                                                               'Right_Stereo_Map_x', 'Right_Stereo_Map_y'],
                      [self.Left_Stereo_Map[0], self.Left_Stereo_Map[1], self.Right_Stereo_Map[0], self.Right_Stereo_Map[1]])
        self.focale = self.focale = self.rectification_vals["proj_mat_l"][0][0]
        self.baseline = np.linalg.norm(self.stereo_calibration["translation_vector"])
        self.calibrate = True

    def load_cameras_data(self, path):
        print("*************************Loading cameras calibration consts************************")

        print("*** stereo_calibration.xml")
        cv_file = cv2.FileStorage(
            path+"stereo_calibration.xml", cv2.FILE_STORAGE_READ)
        retL = cv_file.getNode('retL').real()
        self.cameraL_calibration['intrinsic_matrix'] = cv_file.getNode(
            'mtxL').mat()
        self.cameraL_calibration['distortion_coef'] = cv_file.getNode(
            'distL').mat()
        print("Left error = "+str(retL))
        retR = cv_file.getNode('retR').real()
        self.cameraR_calibration['intrinsic_matrix'] = cv_file.getNode(
            'mtxR').mat()
        self.cameraR_calibration['distortion_coef'] = cv_file.getNode(
            'distR').mat()
        print("Right error = "+str(retR))

        retS = cv_file.getNode('retS').real()
        self.stereo_calibration["rotation_matrix"] = cv_file.getNode(
            'Rot').mat()
        self.stereo_calibration["translation_vector"] = cv_file.getNode(
            'Trns').mat()
        self.stereo_calibration["essential_matrix"] = cv_file.getNode(
            'Emat').mat()
        self.stereo_calibration["fundamental_matrix"] = cv_file.getNode(
            'Fmat').mat()
        print("Stereo error = "+str(retS))
        cv_file.release()

        print("*** Homography matrixs")
        cv_file = cv2.FileStorage(
            path+"homography_matrices.xml", cv2.FILE_STORAGE_READ)
        retH = cv_file.getNode('retH').mat()
        self.h_mats.append(cv_file.getNode('camera1_homography_matrix').mat())
        self.h_mats.append(cv_file.getNode('camera2_homography_matrix').mat())
        print("Stereo error = "+str(retH))
        cv_file.release()

        print("*** rectification_values.xml")
        cv_file = cv2.FileStorage(
            path+"rectification_values.xml", cv2.FILE_STORAGE_READ)
        self.rectification_vals["rect_l"] = cv_file.getNode('rect_l').mat()
        self.rectification_vals["rect_r"] = cv_file.getNode('rect_r').mat()
        self.rectification_vals["proj_mat_l"] = cv_file.getNode(
            'proj_mat_l').mat()
        self.rectification_vals["proj_mat_r"] = cv_file.getNode(
            'proj_mat_r').mat()
        self.rectification_vals["disparity"] = cv_file.getNode(
            'disparity').mat()
        self.rectification_vals["roiL"] = cv_file.getNode('roiL').mat()
        self.rectification_vals["roiR"] = cv_file.getNode('roiR').mat()
        cv_file.release()

        print("*** improved_params2.xml")
        cv_file = cv2.FileStorage(
            path+"improved_params2.xml", cv2.FILE_STORAGE_READ)
        self.Left_Stereo_Map.append(cv_file.getNode('Left_Stereo_Map_x').mat())
        self.Left_Stereo_Map.append(cv_file.getNode('Left_Stereo_Map_y').mat())
        self.Right_Stereo_Map.append(
            cv_file.getNode('Right_Stereo_Map_x').mat())
        self.Right_Stereo_Map.append(
            cv_file.getNode('Right_Stereo_Map_y').mat())
        cv_file.release()
        self.focale = self.rectification_vals["proj_mat_l"][0][0]

        self.baseline = np.linalg.norm(self.stereo_calibration["translation_vector"])

    def timer_callback(self):
        if self.init1 and self.init2:

            frame1 = self.frame1.copy()
            frame2 = self.frame2.copy()

            self.init1 = False
            self.init2 = False
            frame2 = custom_resize(frame2, 640)

            # self.save_on_click(frame2, frame1)
            self.stereo_vision_3rd(frame1, frame2)

    def save_on_click(self, frameL, frameR):
        checkerboard = (7, 10)

        if cv2.waitKey(25) & 0xFF == ord('s'):

            h1, w1, _ = frameL.shape
            h2, w2, _ = frameR.shape

            outputL = frameL.copy()
            outputR = frameR.copy()

            resizedL = cv2.resize(frameL, (w1*2, h1*2))
            resizedR = cv2.resize(frameR, (w2*2, h2*2))

            criteria = (cv2.TERM_CRITERIA_EPS +
                        cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

            gray_resizedL = cv2.cvtColor(resizedL, cv2.COLOR_BGR2GRAY)
            gray_resizedR = cv2.cvtColor(resizedR, cv2.COLOR_BGR2GRAY)

            retL, cornersL = cv2.findChessboardCorners(
                resizedL, checkerboard, None)
            retR, cornersR = cv2.findChessboardCorners(
                resizedR, checkerboard, None)

            conv_size = (11, 11)

            if retL:
                cv2.cornerSubPix(gray_resizedL, cornersL,
                                 conv_size, (-1, -1), criteria)
                cornersL = cornersL/2
                cv2.drawChessboardCorners(
                    outputL, checkerboard, cornersL, retL)
                cv2.drawChessboardCorners(
                    self.landmark_frames1, checkerboard, cornersL, retL)

            if retR:
                cv2.cornerSubPix(gray_resizedR, cornersR,
                                 conv_size, (-1, -1), criteria)
                cornersR = cornersR/2
                cv2.drawChessboardCorners(
                    outputR, checkerboard, cornersR, retR)
                cv2.drawChessboardCorners(
                    self.landmark_frames2, checkerboard, cornersR, retR)

            cv2.imshow('LandmarkOnFrame', np.vstack((outputL, outputR)))
            cv2.imshow('Landmarks', np.vstack(
                (self.landmark_frames1, self.landmark_frames2)))
            path_L = "cameraL/"
            path_R = "cameraR/"
            os.makedirs(path_L, exist_ok=True)
            os.makedirs(path_R, exist_ok=True)
            print("frame_"+f"{self.frame_id:03d}"+".png")
            cv2.imwrite(path_L+"frame_"+f"{self.frame_id:03d}"+".png", frameL)
            cv2.imwrite(path_R+"frame_"+f"{self.frame_id:03d}"+".png", frameR)
            self.frame_id += 1

    def stereo_vision_3rd(self, frameL, frameR):
        if self.calibrate:
            h1, w1 = frameL.shape[0:2]
# =============================================================================
#
#             frameL, frameR = self.test_epilines_calibration(frameL, frameR,
#                                                             self.cameraL_calibration,
#                                                             self.cameraR_calibration,
#                                                             self.stereo_calibration, 1,
#                                                             self.checkerboard)
#
# =============================================================================
            frameL, frameR = self.rectification_method(frameL, frameR, 1)
            self.imageL_publisher.publish(self.cvbr.cv2_to_imgmsg(frameL, encoding="bgr8"))
            self.imageR_publisher.publish(self.cvbr.cv2_to_imgmsg(frameR, encoding="bgr8"))

            self.disparity = self.calculate_disparity(cv2.rotate(frameL, cv2.ROTATE_90_COUNTERCLOCKWISE),
                                                      cv2.rotate(frameR, cv2.ROTATE_90_COUNTERCLOCKWISE), 'sgbm')
            self.disparity = cv2.rotate(self.disparity, cv2.ROTATE_90_CLOCKWISE)
            depth_map = self.baseline*self.focale / (self.disparity*1.0)
            self.depth_publisher.publish(self.cvbr.cv2_to_imgmsg(depth_map))

    def calculate_disparity(self, frameL, frameR, matcher):
        if not self.calibrate:
            return
        if not self.stereo_matcher:
            if matcher == 'bm':
                #self.stereo_matcher = cv2.StereoBM_create(128, 51)
                self.stereo_matcher = cv2.StereoBM_create(96, 11)
            else:
                self.stereo_matcher = cv2.StereoSGBM_create(numDisparities=96,
                                                            minDisparity=0,
                                                            blockSize=11,
                                                            P1=8 * 3 * 6 ** 2,
                                                            P2=32 * 3 * 6 ** 2,
                                                            mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
                                                            )
        imgL_gray = cv2.cvtColor(frameL, cv2.COLOR_BGR2GRAY)
        imgR_gray = cv2.cvtColor(frameR, cv2.COLOR_BGR2GRAY)
        disparity = self.stereo_matcher.compute(imgL_gray, imgR_gray)
        disparity = disparity.astype(np.float32)
        disparity = disparity/16.0
        disparity[disparity == 0.0] = 0.1
        disparity[disparity == -1.0] = 0.1

        return disparity

    def rectification_method(self, frameL, frameR, idx_resize):
        if self.calibrate:
            frameL = cv2.remap(
                frameL, self.Left_Stereo_Map[0], self.Left_Stereo_Map[1], cv2.INTER_LANCZOS4, cv2.BORDER_CONSTANT, 0)
            frameR = cv2.remap(
                frameR, self.Right_Stereo_Map[0], self.Right_Stereo_Map[1], cv2.INTER_LANCZOS4, cv2.BORDER_CONSTANT, 0)
            """if idx_resize == 1:
                min_x1 = np.where(frameR > 0.0)[0].min()
                max_x1 = np.where(frameR > 0.0)[0].max()

                min_y1 = np.where(frameR > 0.0)[1].min()
                max_y1 = np.where(frameR > 0.0)[1].max()

                frameL = frameL[min_x1:max_x1, min_y1:max_y1]
                frameR = frameR[min_x1:max_x1, min_y1:max_y1]
            else:
                min_x1 = np.where(frameL > 0.0)[0].min()
                max_x1 = np.where(frameL > 0.0)[0].max()

                min_y1 = np.where(frameL > 0.0)[1].min()
                max_y1 = np.where(frameL > 0.0)[1].max()

                frameL = frameL[min_x1:max_x1, min_y1:max_y1]
                frameR = frameR[min_x1:max_x1, min_y1:max_y1]
                """
            return frameL, frameR

    def test_epilines_calibration(self, frame1, frame2, calibration1, calibration2, stereo_calibration, source_in_FM, cb):

        if self.calibrate:
            h, w = frame1.shape[:2]
            # newcameramtx, _ = cv2.getOptimalNewCameraMatrix(
            #    calibration1['intrinsic_matrix'], calibration1['distortion_coef'], (w, h), 1, (w, h))
            undistorded_frame1 = cv2.undistort(
                frame1, calibration1['intrinsic_matrix'], calibration1['distortion_coef'], None, None)

            h, w = frame2.shape[:2]
            # newcameramtx, _ = cv2.getOptimalNewCameraMatrix(
            #    calibration2['intrinsic_matrix'], calibration2['distortion_coef'], (w, h), 1, (w, h))
            undistorded_frame2 = cv2.undistort(
                frame2, calibration2['intrinsic_matrix'], calibration2['distortion_coef'], None, None)

            points1 = detect_chess_board(undistorded_frame1, cb, 4)
            points2 = detect_chess_board(undistorded_frame2, cb, 4)

            colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 0)]

            for i in range(len(points1)):
                point = points1[i]
                undistorded_frame1 = cv2.circle(undistorded_frame1, (int(
                    point[0]), int(point[1])), radius=3, color=colors[i], thickness=2)
                undistorded_frame1 = cv2.putText(undistorded_frame1, str(i), (int(point[0]), int(
                    point[1])), cv2.FONT_HERSHEY_SIMPLEX, 1, colors[i], 2, cv2.LINE_AA, False)
            for i in range(len(points2)):
                point = points2[i]
                undistorded_frame2 = cv2.circle(undistorded_frame2, (int(
                    point[0]), int(point[1])), radius=3, color=colors[i], thickness=2)
                undistorded_frame2 = cv2.putText(undistorded_frame2, str(i), (int(point[0]), int(
                    point[1])), cv2.FONT_HERSHEY_SIMPLEX, 1, colors[i], 2, cv2.LINE_AA, False)

            if len(points1):
                epilines_on_frame_2 = cv2.computeCorrespondEpilines(
                    points1.reshape(-1, 1, 2), source_in_FM, stereo_calibration["fundamental_matrix"])
                epilines_on_frame_2 = epilines_on_frame_2.reshape(-1, 3)
                drawLines(undistorded_frame2, epilines_on_frame_2, colors)

            if len(points2):
                epilines_on_frame_1 = cv2.computeCorrespondEpilines(
                    points2.reshape(-1, 1, 2), (source_in_FM % 2)+1, stereo_calibration["fundamental_matrix"])
                epilines_on_frame_1 = epilines_on_frame_1.reshape(-1, 3)
                drawLines(undistorded_frame1, epilines_on_frame_1, colors)

            return (undistorded_frame1, undistorded_frame2)

    def callback_camera1(self, data):
        self.init1 = True
        self.frame1 = self.cvbr.imgmsg_to_cv2(data)

    def callback_camera2(self, data):
        self.init2 = True
        self.frame2 = self.cvbr.imgmsg_to_cv2(data)


def main(args=None):
    rclpy.init(args=args)

    stereo_vision_node = StereoVisionNode()

    rclpy.spin(stereo_vision_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    stereo_vision_node.cap.release()
    cv2.destroyAllWindows()
    stereo_vision_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
