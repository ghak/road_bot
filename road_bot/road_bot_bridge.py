#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Jan  14 08:20:00 2022
@author: ghak
"""

import rclpy
import socket
import time
from rclpy.node import Node
from sensor_msgs.msg import Joy
from sensor_msgs.msg import LaserScan
from road_bot.reg_defines import *


t_pred = time.time()

dic_vals = {
        'f/w' : [-1,-1], #-1: stop, 0:backward, 1:forward
        'vitesse': [350,350],
        'rotation_direction':[0,95],
        'rotation_cam_x':[0,95],
        'rotation_cam_y':[0,95],
        'buzzer' : [False, False],
        'light_red' : [False, False],
        'light_green' : [False, False],
        'light_blue' : [False, False],
}

flag= '0'.encode('utf-8')

class RoadBotBridge(Node):

    def __init__(self, topic, topic2):
        super().__init__('road_bot_bridge')
        self.subscription = self.create_subscription(Joy, topic, self.callback, 10)
        self.publisher_ = self.create_publisher(LaserScan, topic2, 10)
        self.udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.remote_addr = (ip_addr, ip_port)
        self.t_pred = time.time()
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.subscription  # prevent unused variable warning
    
    def send_udp(self, l):
        msg = ';'.join([str(x) for x in l])
        self.udp.sendto(msg.encode('utf-8'), self.remote_addr)
    
    def timer_callback(self):
        current_time = time.time()
        scan = LaserScan()
        scan.header.frame_id = 'sonar_frame'
        scan.angle_min = 0.611
        scan.angle_max = 2.55
        scan.angle_increment = 0.175
        scan.time_increment = 0.05
        scan.scan_time = 0.3
        scan.range_min = 0.0
        scan.range_max = 10000.0

        scan.ranges = []
        scan.intensities = []
        
        
        #print "scan Sonic...."
        for angle in range(35, 146, 10):
            self.send_udp((0, REG_SERVO4, numMap(angle,0,180,500,2500)))
            self.send_udp((0, REG_SERVO4, numMap(angle,0,180,500,2500)))
            data = flag
            while data==flag:
                self.send_udp((1,REG_ECHO))
                data, _ = self.udp.recvfrom(1024)
            distance = (float(data.decode('utf-8')) * 17.0 / 1000.0)
            scan.ranges.append(distance)
            time.sleep(0.05)
        self.publisher_.publish(scan)
        
            
            
    
    def callback(self, data):
        if data.buttons[0] == 1: #forward
            dic_vals['f/w'][1] = 1
        elif data.buttons[1] == 1:#stop
            dic_vals['f/w'][1] = -1
            print('test1'+str(dic_vals['f/w']))
        elif data.buttons[2] == 1:#backward
            dic_vals['f/w'][1] = 0
        
        if data.buttons[4] == 1:
            value = dic_vals['vitesse'][0] + 100
            if(value>950):
                value = 950
            if time.time() - self.t_pred > 0.2:
                self.t_pred = time.time()
                dic_vals['vitesse'][1] = value
                print(value)
        elif data.buttons[5] == 1:
            value = dic_vals['vitesse'][0] - 100
            if(value<350):
                value = 350
            if time.time() - self.t_pred > 0.2:
                self.t_pred = time.time()
                dic_vals['vitesse'][1] = value
                print(value)
        
        dic_vals['rotation_direction'][1] = int((data.axes[0]+1) * 90/2) + 50
        dic_vals['rotation_cam_x'][1] = int((data.axes[3]+1) * 90/2) + 50
        dic_vals['rotation_cam_y'][1] = int((data.axes[4]+1) * 85/2) + 60
        
        
        if data.axes[6]== 1:
            if time.time() - self.t_pred > 0.5:
                self.t_pred = time.time()
                dic_vals['light_red'][1] = not(dic_vals['light_red'][0])  
        elif data.axes[6]== -1:
            if time.time() - self.t_pred > 0.5:
                self.t_pred = time.time()
                dic_vals['light_green'][1] = not(dic_vals['light_green'][0])  
        elif data.axes[7]== 1:
            if time.time() - self.t_pred > 0.5:
                self.t_pred = time.time()
                dic_vals['light_blue'][1] = not(dic_vals['light_blue'][0])
        """elif  data.axes[7]== -1:
             if time.time() - t_pred > 0.2:
                t_pred = time.time()
                dic_vals['buzzer'] = not(dic_vals['buzzer'])"""  
        
            
        if dic_vals['f/w'][0]!=-1 and dic_vals['f/w'][1] == -1:
            self.send_udp((0, REG_PWM1, 0))
            self.send_udp((0, REG_PWM2, 0))
            dic_vals['f/w'][0] = -1
            print('test1'+str(dic_vals['f/w']))
        elif  dic_vals['f/w'][0] != dic_vals['f/w'][1] :  
            self.udp.sendto(("0;"+str(REG_PWM1)+"0").encode('utf-8'), self.remote_addr)
            self.send_udp((0, REG_PWM1, dic_vals['vitesse'][1]))
            self.send_udp((0, REG_PWM2, dic_vals['vitesse'][1]))
            self.send_udp((0, REG_DIR1, dic_vals['f/w'][1]))
            self.send_udp((0, REG_DIR2, dic_vals['f/w'][1]))
            dic_vals['f/w'][0] = dic_vals['f/w'][1]
        
        if  dic_vals['vitesse'][0] != dic_vals['vitesse'][1]:
            dic_vals['vitesse'][0] = dic_vals['vitesse'][1]
            if dic_vals['f/w'][0] != -1:
                self.send_udp((0, REG_PWM1, dic_vals['vitesse'][1]))
                self.send_udp((0, REG_PWM2, dic_vals['vitesse'][1]))
                self.send_udp((0, REG_DIR1, dic_vals['f/w'][0]))
                self.send_udp((0, REG_DIR2, dic_vals['f/w'][0]))
                
        if dic_vals['rotation_direction'][0] != dic_vals['rotation_direction'][1]:
            self.send_udp((0, REG_SERVO1, numMap(dic_vals['rotation_direction'][1],0,180,500,2500)))
            self.send_udp((0, REG_SERVO2, numMap(dic_vals['rotation_direction'][1],0,180,500,2500)))
            dic_vals['rotation_direction'][0]=dic_vals['rotation_direction'][1]
            print (dic_vals['rotation_direction'][1])
        
        if dic_vals['rotation_cam_x'][0] != dic_vals['rotation_cam_x'][1] and dic_vals['rotation_direction'][0] == 95:
            self.send_udp((0, REG_SERVO2, numMap(dic_vals['rotation_cam_x'][1],0,180,500,2500)))
            dic_vals['rotation_cam_x'][0]=dic_vals['rotation_cam_x'][1]
        
        if dic_vals['rotation_cam_y'][0] != dic_vals['rotation_cam_y'][1] and dic_vals['rotation_direction'][0] == 95:
            self.send_udp((0, REG_SERVO3, numMap(dic_vals['rotation_cam_y'][1],0,180,500,2500)))
            dic_vals['rotation_cam_y'][0]=dic_vals['rotation_cam_y'][1]

                                
        if dic_vals['light_green'][0] != dic_vals['light_green'][1]:
            if dic_vals['light_green'][0] is True:
                self.send_udp((0, REG_IO1, 1))
                dic_vals['light_green'][0] = False
            elif dic_vals['light_green'][0] is False:
                self.send_udp((0, REG_IO1, 0))
                dic_vals['light_green'][0] = True
        elif dic_vals['light_blue'][0] != dic_vals['light_blue'][1]:
            if dic_vals['light_blue'][0] is True:
                self.send_udp((0, REG_IO2, 1))
                dic_vals['light_blue'][0] = False
            elif dic_vals['light_blue'][0] is False:
                self.send_udp((0, REG_IO2, 0))
                dic_vals['light_blue'][0] = True
        elif dic_vals['light_red'][0] != dic_vals['light_red'][1]:
            if dic_vals['light_red'][0] is True:
                self.send_udp((0, REG_IO3, 1))
                dic_vals['light_red'][0] = False
            elif dic_vals['light_red'][0] is False:
                self.send_udp((0, REG_IO3, 0))
                dic_vals['light_red'][0] = True
        
        """if  mdev.Is_Buzzer_State_True != dic_vals['buzzer']:
            if mdev.Is_Buzzer_State_True is True:
                mdev.Is_Buzzer_State_True = False
                mdev.writeReg(mdev.REG_BUZZER,0)
            elif mdev.Is_Buzzer_State_True is False:                        
                mdev.Is_Buzzer_State_True = True
                mdev.writeReg(mdev.REG_BUZZER,2000)"""  
        
        if data.axes[7]== -1 :
            self.send_udp((0, REG_BUZZER, 2000))
        else:
            self.send_udp((0, REG_BUZZER, 0))



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

    road_bot_bridge = RoadBotBridge("joy", "Sonar")

    rclpy.spin(road_bot_bridge)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    road_bot_bridge.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
