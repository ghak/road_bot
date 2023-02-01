#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Jan  14 08:20:00 2022
@author: ghak
"""

import rclpy
import socket
import time
import numpy as np
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from sensor_msgs.msg import Joy
from sensor_msgs.msg import LaserScan
from road_bot.reg_defines import *
from std_msgs.msg import Float64MultiArray


t_pred = time.time()

dic_keys = ['f/w', 'vitesse', 'rotation_direction', 'rotation_cam_x',
            'rotation_cam_y', 'buzzer', 'light_red', 'light_green', 'light_blue']


dic_vals = {
    'f/w': [-1, -1],  # -1: stop, 0:backward, 1:forward
    'vitesse': [350, 350],
    'rotation_direction': [0, 95],
    'rotation_cam_x': [0, 95],
    'rotation_cam_y': [0, 95],
    'buzzer': [0, 0],
    'light_red': [1, 1],
    'light_green': [1, 1],
    'light_blue': [1, 1],
}

flag = '0'.encode('utf-8')


class RoadBotBridge(Node):

    def __init__(self):
        super().__init__('road_bot_bridge')

        joystick_group = MutuallyExclusiveCallbackGroup()
        sonar_group = MutuallyExclusiveCallbackGroup()

        self.subscription = self.create_subscription(Joy, "joy", self.callback, 10, callback_group=joystick_group)
        self.publisher_ = self.create_publisher(LaserScan, "sonar", 10)
        self.RIS_publisher = self.create_publisher(Float64MultiArray, "robot_state", 10)
        self.udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp.settimeout(1)
        self.remote_addr = (ip_addr, ip_port)
        self.t_pred = time.time()
        self.sonic = True
        self.sonic_direction = 1
        timer_period = 3600  # 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback, callback_group=sonar_group)
        self.subscription  # prevent unused variable warning

    def send_udp(self, l):
        msg = ';'.join([str(x) for x in l])
        self.udp.sendto(msg.encode('utf-8'), self.remote_addr)

    def timer_callback(self):
        if self.sonic:
            step = 26
            min_ = 25
            max_ = 155
            sleep_time = 0.2

            current_time = time.time()
            scan = LaserScan()
            scan.header.frame_id = 'sonar_frames'
            scan.angle_min = np.radians(min_)
            scan.angle_max = np.radians(max_)
            scan.angle_increment = np.radians(step)
            scan.time_increment = sleep_time
            scan.scan_time = (max_-min_)/step*sleep_time
            scan.range_min = 0.0
            scan.range_max = 10000.0

            scan.ranges = []
            scan.intensities = []

            # print "scan Sonic...."
            if self.sonic_direction == 1:
                for angle in np.arange(min_, max_+1, step):
                    self.send_udp((0, REG_SERVO4, numMap(angle, 0, 180, 500, 2500)))
                    time.sleep(sleep_time)
                    data = b''
                    self.send_udp((1, REG_ECHO))
                    for i in range(10):
                        try:
                            data, _ = self.udp.recvfrom(1024)
                        except TimeoutError:
                            self.get_logger().error('TimeOut on sonic reception')
                        if data.startswith(b'value ='):
                            break
                    if i == 9:
                        return
                    distance = float(data.decode('utf-8').split("=")[-1])
                    distance = distance * 17.0 / 1000.0
                    scan.ranges.append(distance)

            else:
                for angle in np.arange(max_, min_-1, -step):
                    self.send_udp((0, REG_SERVO4, numMap(angle, 0, 180, 500, 2500)))
                    time.sleep(sleep_time)
                    data = b''
                    self.send_udp((1, REG_ECHO))
                    for i in range(10):
                        try:
                            data, _ = self.udp.recvfrom(1024)
                        except TimeoutError:
                            self.get_logger().error('TimeOut on sonic reception')
                        if data.startswith(b'value ='):
                            break
                    if i == 9:
                        return
                    distance = float(data.decode('utf-8').split("=")[-1])
                    distance = distance * 17.0 / 1000.0
                    scan.ranges.append(distance)
                scan.ranges.reverse()
            self.sonic_direction = (self.sonic_direction+1) % 2
            self.publisher_.publish(scan)

    def callback(self, data):
        if data.buttons[0] == 1:  # forward
            dic_vals['f/w'][1] = 1
        elif data.buttons[1] == 1:  # stop
            dic_vals['f/w'][1] = -1
            print('test1'+str(dic_vals['f/w']))
        elif data.buttons[2] == 1:  # backward
            dic_vals['f/w'][1] = 0

        if data.buttons[4] == 1:
            value = dic_vals['vitesse'][0] + 100
            if(value > 950):
                value = 950
            if time.time() - self.t_pred > 0.2:
                self.t_pred = time.time()
                dic_vals['vitesse'][1] = value
                print(value)
        elif data.buttons[5] == 1:
            value = dic_vals['vitesse'][0] - 100
            if(value < 350):
                value = 350
            if time.time() - self.t_pred > 0.2:
                self.t_pred = time.time()
                dic_vals['vitesse'][1] = value
                print(value)

        dic_vals['rotation_direction'][1] = int((data.axes[0]+1) * 90/2) + 50
        dic_vals['rotation_cam_x'][1] = int((data.axes[3]+1) * 90/2) + 50
        dic_vals['rotation_cam_y'][1] = int((data.axes[4]+1) * 85/2) + 48

        if data.axes[6] == 1:
            if time.time() - self.t_pred > 0.2:
                self.t_pred = time.time()
                dic_vals['light_red'][1] = (dic_vals['light_red'][0]+1) % 2
        elif data.axes[6] == -1:
            if time.time() - self.t_pred > 0.2:
                self.t_pred = time.time()
                dic_vals['light_green'][1] = (dic_vals['light_green'][0]+1) % 2
        if data.axes[7] == -1:
            dic_vals['buzzer'][1] = 2000
        else:
            dic_vals['buzzer'][1] = 0
            if data.axes[7] == 1:
                if time.time() - self.t_pred > 0.2:
                    self.t_pred = time.time()
                    dic_vals['light_blue'][1] = (dic_vals['light_blue'][0]+1) % 2

        if dic_vals['f/w'][0] != -1 and dic_vals['f/w'][1] == -1:
            self.send_udp((0, REG_PWM1, 0))
            self.send_udp((0, REG_PWM2, 0))
            dic_vals['f/w'][0] = -1
            print('test1'+str(dic_vals['f/w']))
        elif dic_vals['f/w'][0] != dic_vals['f/w'][1]:
            self.udp.sendto(("0;"+str(REG_PWM1)+"0").encode('utf-8'), self.remote_addr)
            self.send_udp((0, REG_PWM1, dic_vals['vitesse'][1]))
            self.send_udp((0, REG_PWM2, dic_vals['vitesse'][1]))
            self.send_udp((0, REG_DIR1, dic_vals['f/w'][1]))
            self.send_udp((0, REG_DIR2, dic_vals['f/w'][1]))
            dic_vals['f/w'][0] = dic_vals['f/w'][1]

        if dic_vals['vitesse'][0] != dic_vals['vitesse'][1]:
            dic_vals['vitesse'][0] = dic_vals['vitesse'][1]
            if dic_vals['f/w'][0] != -1:
                self.send_udp((0, REG_PWM1, dic_vals['vitesse'][1]))
                self.send_udp((0, REG_PWM2, dic_vals['vitesse'][1]))
                self.send_udp((0, REG_DIR1, dic_vals['f/w'][0]))
                self.send_udp((0, REG_DIR2, dic_vals['f/w'][0]))

        if dic_vals['rotation_direction'][0] != dic_vals['rotation_direction'][1]:
            self.send_udp((0, REG_SERVO1, numMap(dic_vals['rotation_direction'][1], 0, 180, 500, 2500)))
            self.send_udp((0, REG_SERVO2, numMap(dic_vals['rotation_direction'][1], 0, 180, 500, 2500)))
            dic_vals['rotation_direction'][0] = dic_vals['rotation_direction'][1]
            print(dic_vals['rotation_direction'][1])

        if dic_vals['rotation_cam_x'][0] != dic_vals['rotation_cam_x'][1] and dic_vals['rotation_direction'][0] == 95:
            self.send_udp((0, REG_SERVO2, numMap(dic_vals['rotation_cam_x'][1], 0, 180, 500, 2500)))
            dic_vals['rotation_cam_x'][0] = dic_vals['rotation_cam_x'][1]

        if dic_vals['rotation_cam_y'][0] != dic_vals['rotation_cam_y'][1] and dic_vals['rotation_direction'][0] == 95:
            self.send_udp((0, REG_SERVO3, numMap(dic_vals['rotation_cam_y'][1], 0, 180, 500, 2500)))
            dic_vals['rotation_cam_y'][0] = dic_vals['rotation_cam_y'][1]

        if dic_vals['light_green'][0] != dic_vals['light_green'][1]:
            self.send_udp((0, REG_IO1, dic_vals['light_green'][1]))
            dic_vals['light_green'][0] = dic_vals['light_green'][1]
        elif dic_vals['light_blue'][0] != dic_vals['light_blue'][1]:
            self.send_udp((0, REG_IO2, dic_vals['light_blue'][1]))
            dic_vals['light_blue'][0] = dic_vals['light_blue'][1]
        elif dic_vals['light_red'][0] != dic_vals['light_red'][1]:
            self.send_udp((0, REG_IO3, dic_vals['light_red'][1]))
            dic_vals['light_red'][0] = dic_vals['light_red'][1]
        elif dic_vals['buzzer'][0] != dic_vals['buzzer'][1]:
            self.send_udp((0, REG_BUZZER, dic_vals['buzzer'][1]))
            dic_vals['buzzer'][0] = dic_vals['buzzer'][1]

        ris_msg = Float64MultiArray()
        a = time.time()
        ris_msg.data = [np.float64(dic_vals[k][0]) for k in dic_keys]+[np.float64(a)]

        self.RIS_publisher.publish(ris_msg)
        """if cmd.CMD_CAMERA_UP[1:]   in RecvData:
            value = int(filter(str.isdigit, RecvData))
            mdev.writeReg(mdev.CMD_SERVO3,numMap(value,0,180,500,2500))
            pass

        if cmd.CMD_CAMERA_DOWN[1:]   in RecvData:
            value = int(filter(str.isdigit, RecvData))
            mdev.writeReg(mdev.CMD_SERVO3,numMap(value,0,180,500,2500))
            pass

        if cmd.CMD_CAMERA_LEFT[1:]   in RecvData:
            value = int(filter(str.isdigit, RecvData))
            mdev.writeReg(mdev.CMD_SERVO2,numMap(value,0,180,500,2500))
            pass

        if cmd.CMD_CAMERA_RIGHT[1:]   in RecvData:
            value = int(filter(str.isdigit, RecvData))
            mdev.writeReg(mdev.CMD_SERVO2,numMap(value,0,180,500,2500))
            pass

        elif data.buttons[3] == 1:
            sonic = mdev.getSonic()
            self.sendData(str(sonic))"""


def main(args=None):
    rclpy.init(args=args)

    road_bot_bridge = RoadBotBridge()
    executor = MultiThreadedExecutor()
    executor.add_node(road_bot_bridge)
    executor.spin()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    road_bot_bridge.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
