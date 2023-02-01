#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Jan 30 02:38:01 2023

@author: ghak
"""

import os
import sys

import pdb
import cv2
import numpy as np
import time
from scipy.spatial.transform import Rotation as R


import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, Quaternion
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Path

from cv_bridge import CvBridge


class Odometry(Node):

    def __init__(self):
        super().__init__('stereo_vision_node')

        self.subscription1 = self.create_subscription(Image, 'imageL', self.callback_frameL, 10)
        self.subscription2 = self.create_subscription(Image, 'imageR', self.callback_frameR, 10)
        self.subscription3 = self.create_subscription(Image, 'depth_map', self.callback_depth, 10)
        self.RIS_subscriber = self.create_subscription(Float64MultiArray, 'robot_state', self.callback_ris, 10)
        self.visual_pose_publisher = self.create_publisher(PoseStamped, 'visual_robot_position', 10)
        self.visual_path_publisher = self.create_publisher(Path, 'visual_robot_path', 10)
        self.wheels_pose_publisher = self.create_publisher(PoseStamped, 'wheels_robot_position', 10)
        self.wheels_path_publisher = self.create_publisher(Path, 'wheels_robot_path', 10)
        self.cvbr = CvBridge()

        self.visual_robot_path = Path()
        self.init = False

        self.wheels_positionning = {'x': 0.0, 'y': 0.0, 'r': 0.0, 't': 0.0}
        self.wheels_robot_path = Path()
        self.prev_ris = 0

        version = 'to_use'
        self.checkerboard = (7, 10)
        self.checkerboard_box_size = 2.5
        self.calib_path = "./calibration_images/" + \
            str(self.checkerboard[0])+"x" + \
            str(self.checkerboard[1])+"/"+version+"/"

        self.Left_Stereo_Map = []
        self.Right_Stereo_Map = []
        self.cameraL_calibration = {}
        self.cameraR_calibration = {}
        self.stereo_calibration = {}
        self.rectification_vals = {}
        self.h_mats = []

        self.init0 = False
        self.init1 = False
        self.init2 = False

        self.frameL = 0
        self.frameR = 0
        self.depth_map = 0
        self.prevL = 0
        self.prevR = 0
        self.prev_depth = 0

        self.focale = 0
        self.baseline = 0

        self.detector = 0
        self.matcher = 0
        self.visual_pose = np.eye(4)

        self.load_cameras_data(self.calib_path)
        self.time = 0
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        max_depth = 200
        dist_threshold = 0.5
        # mtx = self.cameraL_calibration["intrinsic_matrix"]
        mtx = self.rectification_vals["proj_mat_l"][:, 0:3]
        if not self.init and (self.init0 and self.init1 and self.init2):
            matching = 'BF'

            self.prevL = self.frameL.copy()
            self.prev_depth = self.depth_map.copy()

            detector = 'sift'
            if detector == 'sift':
                self.detector = cv2.SIFT_create()
            elif detector == 'orb':
                self.detector = cv2.ORB_create()
            elif detector == 'surf':
                self.detector = cv2.xfeatures2d.SURF_create()

            if matching == 'BF':
                if detector == 'sift':
                    self.matcher = cv2.BFMatcher_create(cv2.NORM_L2, crossCheck=False)
                elif detector == 'orb':
                    self.matcher = cv2.BFMatcher_create(cv2.NORM_HAMMING2, crossCheck=False)

            elif matching == 'FLANN':
                FLANN_INDEX_KDTREE = 1
                index_params = dict(algorithm=FLANN_INDEX_KDTREE, trees=5)
                search_params = dict(checks=50)
                self.matcher = cv2.FlannBasedMatcher(index_params, search_params)
            self.init = True

        if not self.init:
            return

        depth = self.prev_depth

        img_prec = cv2.cvtColor(self.prevL, cv2.COLOR_BGR2GRAY)  # queryImage
        img_now = cv2.cvtColor(self.frameL, cv2.COLOR_BGR2GRAY)  # trainImage

        mask = cv2.inRange(depth, 0, max_depth)
        kp_prec, des_prec = self.detector.detectAndCompute(img_prec, mask)
        kp0, des0 = self.detector.detectAndCompute(img_now, mask)

        # print(f'Number Points : Prec_fram - {len(kp_prec)}    Curr_frame - {len(kp0)}')
        matches = self.matcher.knnMatch(des_prec, des0, k=2)
        matches = sorted(matches, key=lambda x: x[0].distance)
        # print(f'Number matches = {len(matches)}')
        filtered_match = []
        for m, n in matches:
            if m.distance <= dist_threshold*n.distance:
                filtered_match.append(m)
        # print(f'Number filtred matches = {len(filtered_match)}')
        image_matches = cv2.drawMatches(img_prec, kp_prec, img_now, kp0, filtered_match, None, flags=2)
        cv2.imshow('matchs', image_matches)

        object_points = np.zeros((0, 3))
        # delete = []
        prev_points = np.float32([kp_prec[m.queryIdx].pt for m in filtered_match])
        now_points = np.float32([kp0[m.trainIdx].pt for m in filtered_match])

        cx = mtx[0, 2]
        cy = mtx[1, 2]
        fx = mtx[0, 0]
        fy = mtx[1, 1]

        for i, (u, v) in enumerate(prev_points):
            z = depth[int(v), int(u)]
            """if z > max_depth:
                delete.append(i)
                continue"""

            x = z*(u-cx)/fx
            y = z*(v-cy)/fy
            object_points = np.vstack([object_points, np.array([x, y, z])])

        # prev_points = np.delete(prev_points, delete, 0)
        # now_points = np.delete(now_points, delete, 0)
        # print(f'Number final matches = {len(object_points)}')
        if(len(object_points) < 4):
            self.prevL = self.frameL.copy()
            return
        _, rvec, tvec, inliers = cv2.solvePnPRansac(object_points, now_points, mtx, None)
        rmat = cv2.Rodrigues(rvec)[0]

        Tmat = np.eye(4)
        Tmat[:3, :3] = rmat
        Tmat[:3, 3] = tvec.T
        new_pose = self.visual_pose.dot(np.linalg.inv(Tmat))[:3, :]
        # print(new_pose)
        # print(self.visual_pose)
        distance = np.linalg.norm(new_pose[0:3, 3] - self.visual_pose[0:3, 3])
        speed = distance/(time.time()-self.time)*36/10

        if speed > 5:
            return
        self.visual_pose = new_pose
        if distance > 0.005:
            print(f'Deplacement = {distance}, Vitesse = {speed}')
        r = (R.from_matrix([self.visual_pose[0, 0:3], self.visual_pose[1, 0:3], self.visual_pose[2, 0:3]])).as_quat()

        pose_msg = PoseStamped()
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.position.x = self.visual_pose[:, 3][2]
        pose_msg.pose.position.y = -self.visual_pose[:, 3][0]
        pose_msg.pose.position.z = -self.visual_pose[:, 3][1]
        pose_msg.pose.orientation.x = r[2]
        pose_msg.pose.orientation.y = -r[0]
        pose_msg.pose.orientation.z = -r[1]
        pose_msg.pose.orientation.w = r[3]
        self.visual_pose_publisher.publish(pose_msg)
        self.visual_robot_path.header.frame_id = 'map'
        self.visual_robot_path.poses.append(pose_msg)
        self.visual_path_publisher.publish(self.visual_robot_path)

        self.time = time.time()
        self.prevL = self.frameL.copy()
        self.prev_depth = self.depth_map.copy()
        if cv2.waitKey(1) & 0xFF == ord('q'):
            sys.exit(0)

    def callback_frameL(self, data):
        self.frameL = self.cvbr.imgmsg_to_cv2(data)
        self.init0 = True

    def callback_frameR(self, data):
        self.frameR = self.cvbr.imgmsg_to_cv2(data)
        self.init1 = True

    def callback_depth(self, data):
        self.depth_map = self.cvbr.imgmsg_to_cv2(data)
        self.init2 = True

    def callback_ris(self, data):
        coef_speed = 1/5000
        coef_orientation = 1/1.3
        if type(self.prev_ris) == type(0):
            self.prev_ris = np.array(data.data)
            return
        # convert -1:stop, 0:backward, 1:forward => -1:backward 0:stop, 1:forward
        self.prev_ris[0] = [-1, 1, 0][int(self.prev_ris[0])]
        # correct angles offset
        data.data[2] = data.data[2]-95
        # time delta bettween two messages
        delta = data.data[-1]-self.prev_ris[-1]

        th = np.radians(self.prev_ris[2]*delta*coef_orientation)
        d = (self.prev_ris[0]*self.prev_ris[1])*delta*coef_speed
        if d:
            x = np.cos(th) * d
            y = -np.sin(th) * d
            self.wheels_positionning['x'] = self.wheels_positionning['x'] + \
                (np.cos(self.wheels_positionning['r']) * x - np.sin(self.wheels_positionning['r']) * y)
            self.wheels_positionning['y'] = self.wheels_positionning['y'] + \
                (np.sin(self.wheels_positionning['r']) * x + np.cos(self.wheels_positionning['r']) * y)*2
            if th != 0:
                if d > 0:
                    self.wheels_positionning['r'] = self.wheels_positionning['r'] + th
                else:
                    self.wheels_positionning['r'] = self.wheels_positionning['r'] - th

        quaternion = Quaternion()
        quaternion.x = 0.0
        quaternion.y = 0.0
        quaternion.z = np.sin(self.wheels_positionning['r'] / 2)
        quaternion.w = np.cos(self.wheels_positionning['r'] / 2)

        pose_msg = PoseStamped()
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.position.x = self.wheels_positionning['x']
        pose_msg.pose.position.y = self.wheels_positionning['y']
        pose_msg.pose.position.z = 0.0
        pose_msg.pose.orientation = quaternion
        self.wheels_pose_publisher.publish(pose_msg)
        self.wheels_robot_path.header.frame_id = 'map'
        self.wheels_robot_path.poses.append(pose_msg)
        self.wheels_path_publisher.publish(self.wheels_robot_path)

        self.prev_ris = np.array(data.data)

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


def main(args=None):
    rclpy.init(args=args)

    odometry = Odometry()

    rclpy.spin(odometry)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    odometry.cap.release()
    cv2.destroyAllWindows()
    odometry.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
