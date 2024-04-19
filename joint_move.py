# -*- coding: utf-8 -*-
# import sys
from lib64 import jkrc
PI=3.1415926
import time

ABS = 0
INCR= 1
joint_pos1=[PI/20,PI/17,PI/130,PI/4,0,0]
joint_pos2=[1,2.5,1,1,1,1]
robot = jkrc.RC("192.168.0.111")#Return robot object
robot.login()
robot.power_on() 
robot.enable_robot()

for i in range(50):
    print(f"Round {i+1}")
    print("move1")
    robot.joint_move(joint_pos1,ABS,True,0.9)
    time.sleep(3)

    print("move2")
    robot.joint_move(joint_pos2,ABS,True,0.1)
    time.sleep(3)

robot.logout()