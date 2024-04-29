# -*- coding: utf-8 -*-
import sys
import time      
from lib64 import jkrc
PI=3.1415926

robot = jkrc.RC("192.168.0.124")
ret = robot.login()
ret = robot.get_joint_position()
if ret[0] == 0:
    print("the joint position is :",ret[1])
else:
    print("some things happend,the errcode is: ",ret[0])
robot.logout()   

#containing 6 elements (x,y,z,rx,ry,rz), with x,y,z,rx,ry,rz representing the pose value of robot tool end