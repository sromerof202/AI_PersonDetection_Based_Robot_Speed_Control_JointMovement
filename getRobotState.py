# -*- coding: utf-8 -*-
import sys
#sys.path.append('D:\\vs2019ws\PythonCtt\lib')
import time      
from lib64 import jkrc

robot = jkrc.RC("192.168.0.124")
ret = robot.login()

ret = robot.get_robot_state()
if ret[0] == 0:
    print("the robot state is :",ret[1])
else:
    print("some things happend,the errcode is: ",ret[0])
robot.logout() 

#3 elements, representing whether emergency stop, whether power on, and whether servo enable, respectively. 1 for yes, 0 for no