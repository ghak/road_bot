#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Jan  14 10:24:00 2022
@author: ghak
"""

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from road_bot.reg_defines import *


class RpiCameraPublisher(Node):

    def __init__(self):
        super().__init__('rpi_camera_publisher')
        
        self.declare_parameter('ip_addr', ip_addr)
        self.declare_parameter('camera_port', camera_port)
        
        self.ip_addr = self.get_parameter('ip_addr').get_parameter_value().string_value
        self.camera_port = self.get_parameter('camera_port').get_parameter_value().integer_value
        
        #print(self.camera_topic, self.ip_addr, self.camera_port)
        
        self.publisher_ = self.create_publisher(Image, "rpi_camera_topic", 10)
        self.cvbr = CvBridge()

        self.cap = cv2.VideoCapture('tcpclientsrc host='+self.ip_addr+' port='+str(self.camera_port)+' ! jpegdec ! appsink', cv2.CAP_GSTREAMER)
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)


    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            self.publisher_.publish(self.cvbr.cv2_to_imgmsg(frame, encoding="bgr8"))


def main(args=None):
    rclpy.init(args=args)
    
    rpi_camera_publisher = RpiCameraPublisher()

    rclpy.spin(rpi_camera_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    rpi_camera_publisher.cap.release()
    cv2.destroyAllWindows()
    rpi_camera_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
