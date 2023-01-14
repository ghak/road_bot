#!/usr/bin/env python3
#-*- coding: utf-8 -*-

ip_addr = "192.168.0.25"
ip_port = 12345
camera_port = 5000


def numMap(value,fromLow,fromHigh,toLow,toHigh):
    return int((toHigh-toLow)*(value-fromLow) / (fromHigh-fromLow) + toLow)

REG_SERVO1      =   0
REG_SERVO2      =   1
REG_SERVO3      =   2
REG_SERVO4      =   3
REG_PWM1        =   4
REG_PWM2        =   5
REG_DIR1        =   6
REG_DIR2        =   7
REG_BUZZER      =   8
REG_IO1         =   9
REG_IO2         =   10
REG_IO3         =   11
REG_ECHO        =   12
SERVO_MAX_PULSE_WIDTH = 2500
SERVO_MIN_PULSE_WIDTH = 500
SONIC_MAX_HIGH_BYTE = 50
Is_IO1_State_True = False
Is_IO2_State_True = False
Is_IO3_State_True = False
Is_Buzzer_State_True = False
handle = True
#mutex = Lock()
