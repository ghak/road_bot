#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Sat Apr  4 01:13:12 2020

@author: ghak
"""

#!/usr/bin/env python
import rospy
import time
from mDev import *
from sensor_msgs.msg import Joy

print("v0.3")

t_pred = time.time()
mdev = mDEV()
dic_vals = {
        'f/w' : [-1,-1], #-1: stop, 0:backward, 1:forward
        'vitesse': [350,350],
        'rotation_direction':[0,95],
        'rotation_cam_x':[0,95],
        'rotation_cam_y':[0,95],
        'buzzer' : False,
        'light_red' : False,
        'light_green' : False,
        'light_blue' : False,
}

def callback(data):
    global dic_vals
    global t_pred
    global mDev
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)
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
        if time.time() - t_pred > 0.2:
            t_pred = time.time()
            dic_vals['vitesse'][1] = value
            print value
    elif data.buttons[5] == 1:
        value = dic_vals['vitesse'][0] - 100
        if(value<350):
            value = 350
        if time.time() - t_pred > 0.2:
            t_pred = time.time()
            dic_vals['vitesse'][1] = value
            print value        
    
    dic_vals['rotation_direction'][1] = int((data.axes[0]+1) * 90/2) + 50
    dic_vals['rotation_cam_x'][1] = int((data.axes[3]+1) * 90/2) + 50
    dic_vals['rotation_cam_y'][1] = int((data.axes[4]+1) * 85/2) + 60
    
    
    if data.axes[6]== 1:
        if time.time() - t_pred > 0.2:
            t_pred = time.time()
            dic_vals['light_red'] = not(dic_vals['light_red'])  
    elif data.axes[6]== -1:
        if time.time() - t_pred > 0.2:
            t_pred = time.time()
            dic_vals['light_green'] = not(dic_vals['light_green'])  
    elif data.axes[7]== 1:
        if time.time() - t_pred > 0.2:
            t_pred = time.time()
            dic_vals['light_blue'] = not(dic_vals['light_blue'])  
    """elif  data.axes[7]== -1:
         if time.time() - t_pred > 0.2:
            t_pred = time.time()
            dic_vals['buzzer'] = not(dic_vals['buzzer'])"""  
    
        
    if dic_vals['f/w'][0]!=-1 and dic_vals['f/w'][1] == -1:
        mdev.writeReg(mdev.CMD_PWM1,0)
        mdev.writeReg(mdev.CMD_PWM2,0)
        dic_vals['f/w'][0] = -1
        print('test1'+str(dic_vals['f/w']))
    elif  dic_vals['f/w'][0] != dic_vals['f/w'][1] :  
        mdev.writeReg(mdev.CMD_PWM1,dic_vals['vitesse'][1])
        mdev.writeReg(mdev.CMD_PWM2,dic_vals['vitesse'][1])
        mdev.writeReg(mdev.CMD_DIR1,dic_vals['f/w'][1])
        mdev.writeReg(mdev.CMD_DIR2,dic_vals['f/w'][1])
        dic_vals['f/w'][0] = dic_vals['f/w'][1]
    
    if  dic_vals['vitesse'][0] != dic_vals['vitesse'][1]:
        dic_vals['vitesse'][0] = dic_vals['vitesse'][1]
        if dic_vals['f/w'][0] != -1:
            mdev.writeReg(mdev.CMD_PWM1,dic_vals['vitesse'][1])
            mdev.writeReg(mdev.CMD_PWM2,dic_vals['vitesse'][1])
            mdev.writeReg(mdev.CMD_DIR1,dic_vals['f/w'][0])
            mdev.writeReg(mdev.CMD_DIR2,dic_vals['f/w'][0])
        
        
    
    if dic_vals['rotation_direction'][0] != dic_vals['rotation_direction'][1]:
        mdev.writeReg(mdev.CMD_SERVO1,numMap(dic_vals['rotation_direction'][1],0,180,500,2500))
        mdev.writeReg(mdev.CMD_SERVO2,numMap(dic_vals['rotation_direction'][1],0,180,500,2500))
        dic_vals['rotation_direction'][0]=dic_vals['rotation_direction'][1]
        print (dic_vals['rotation_direction'][1])
    
    if dic_vals['rotation_cam_x'][0] != dic_vals['rotation_cam_x'][1] and dic_vals['rotation_direction'][0] == 95:
        mdev.writeReg(mdev.CMD_SERVO2,numMap(dic_vals['rotation_cam_x'][1],0,180,500,2500))
        dic_vals['rotation_cam_x'][0]=dic_vals['rotation_cam_x'][1]
    
    if dic_vals['rotation_cam_y'][0] != dic_vals['rotation_cam_y'][1] and dic_vals['rotation_direction'][0] == 95:
        mdev.writeReg(mdev.CMD_SERVO3,numMap(dic_vals['rotation_cam_y'][1],0,180,500,2500))
        dic_vals['rotation_cam_y'][0]=dic_vals['rotation_cam_y'][1]

                            
    if mdev.Is_IO3_State_True != dic_vals['light_red']:
        if mdev.Is_IO3_State_True is True:
            mdev.Is_IO3_State_True = False
            mdev.writeReg(mdev.CMD_IO3,0)
        elif mdev.Is_IO3_State_True is False:
            mdev.Is_IO3_State_True = True
            mdev.writeReg(mdev.CMD_IO3,1)
    elif mdev.Is_IO1_State_True != dic_vals['light_green']:
        if mdev.Is_IO1_State_True is True:
            mdev.Is_IO1_State_True = False
            mdev.writeReg(mdev.CMD_IO1,0)
        elif mdev.Is_IO1_State_True is False:
            mdev.Is_IO1_State_True = True
            mdev.writeReg(mdev.CMD_IO1,1)
    elif mdev.Is_IO2_State_True != dic_vals['light_blue']:
        if mdev.Is_IO2_State_True is True:
            mdev.Is_IO2_State_True = False
            mdev.writeReg(mdev.CMD_IO2,0)
        elif mdev.Is_IO2_State_True is False:
            mdev.Is_IO2_State_True = True
            mdev.writeReg(mdev.CMD_IO2,1)
    """if  mdev.Is_Buzzer_State_True != dic_vals['buzzer']:
        if mdev.Is_Buzzer_State_True is True:
            mdev.Is_Buzzer_State_True = False
            mdev.writeReg(mdev.CMD_BUZZER,0)
        elif mdev.Is_Buzzer_State_True is False:                        
            mdev.Is_Buzzer_State_True = True
            mdev.writeReg(mdev.CMD_BUZZER,2000)"""  
    
    if data.axes[7]== -1 :
        mdev.writeReg(mdev.CMD_BUZZER,2000)
    else:
        mdev.writeReg(mdev.CMD_BUZZER,0)



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
    
def road_bot_server():
    rospy.init_node('road_bot_server', anonymous=True)
    rospy.Subscriber("joy", Joy, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    road_bot_server()
