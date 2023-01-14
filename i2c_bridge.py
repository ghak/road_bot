#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Jan  14 01:13:12 2022
@author: ghak

I2C variables and adresses
"""
import smbus
import socket
import traceback

i2c_addr = 0x18
ip_addr = "192.168.0.25"
ip_port = 12345

class I2C_bridge:
    def __init__(self, addr=i2c_addr):
        self.address = addr 
        self.bus=smbus.SMBus(1)
        self.bus.open(1)
       
    def i2c_write(self, reg, value):
        try:
            self.bus.write_i2c_block_data(self.address,reg,[value>>8,value&0xff])
            return '0'
        except Exception as e:
            print(Exception,"I2C write Error :",e)
            traceback.print_exc()
            return("I2C write error :"+str(e))
    
    def i2c_read(self, reg):
        try:
            return self.bus.read_byte_data(self.address,reg)
        except Exception as e:
            print (Exception,"I2C read Error :",e)
            traceback.print_exc()
            return("I2C read error : "+str(e))
    
class UDP_bridge:
    def __init__(self, addr=ip_addr, port=ip_port):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind((addr, port))

def help_():
    return "Error : Usage : IOcode register [value(needed if write)]"

def main():
    i2c = I2C_bridge()
    udp = UDP_bridge()
    print("Connection started")
    while(True):
        mess, addr = udp.socket.recvfrom(1024)
        print(addr)
        mess = mess.decode('utf-8').split(';')
        if (len(mess) > 3):
            udp.socket.sendto(help_().encode('utf-8'), addr)
            continue
        if mess[0] == '0':
            if (len(mess)!=3):
                udp.socket.sendto(help_().encode('utf-8'), addr)
                continue
            answ = i2c.i2c_write(int(mess[1]), int(mess[2]))
            udp.socket.sendto(answ.encode('utf-8'), addr)
            continue
        elif mess[0] == '1':
            if len(mess)!=2:
                udp.socket.sendto(help_().encode('utf-8'), addr)
                continue
            answ = i2c.i2c_read(int(mess[1]))
            udp.socket.sendto(answ.encode('utf-8'), addr)
            continue   
        else : 
            udp.socket.sendto(help_().encode('utf-8'), addr)
            continue

if __name__ == '__main__':
    main()
