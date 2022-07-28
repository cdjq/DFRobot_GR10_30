# -*- coding: utf-8 -*
'''!
  @file  soft_get_data.py
  @brief 运行本例程可以获取手势
  @copyright   Copyright (c) 2021 DFRobot Co.Ltd (http://www.dfrobot.com)
  @license     The MIT License (MIT)
  @author      zhixinliu(zhixinliu@dfrobot.com)
  @version     V0.1
  @date        2022-07-28
  @url         https://github.com/DFRobor/DFRobot_GR10_30
'''
from __future__ import print_function
import sys
import os
sys.path.append("../")
import time
import RPi.GPIO as GPIO

from DFRobot_GR10_30 import *
ctype=1
ADDRESS = 0x73
I2C_1   = 0x01

if ctype==0:
  GR30_10 = DFRobot_GR30_10_I2C(I2C_1 ,ADDRESS)
else:
  GR30_10 = DFRobot_GR30_10_UART(9600, ADDRESS)


def setup():
  while (GR30_10.begin() == False):
    print("Sensor initialize failed!!")
  time.sleep(1)
  print("Sensor  initialize success!!")
  '''
    # GESTURE_UP
    # GESTURE_DOWN
    # GESTURE_LEFT
    # GESTURE_RIGHT
    # GESTURE_FORWARD
    # GESTURE_BACKWARD
    # GESTURE_CLOCKWISE
    # GESTURE_COUNTERCLOCKWISE
    # GESTURE_WAVE
    # GESTURE_HOVER
    # GESTURE_UNKNOWN
    # GESTURE_CLOCKWISE_C
    # GESTURE_COUNTERCLOCKWISE_C
  '''
  GR30_10.set_mode(GESTURE_UP|GESTURE_DOWN|GESTURE_LEFT|GESTURE_RIGHT|GESTURE_FORWARD|GESTURE_BACKWARD|GESTURE_CLOCKWISE|GESTURE_COUNTERCLOCKWISE|GESTURE_CLOCKWISE_C|GESTURE_COUNTERCLOCKWISE_C)
  
def loop():
  #GR30_10.get_exist()
  if GR30_10.get_data_ready() == True:
    gestrue = GR30_10.get_gestures()
    if gestrue&GESTURE_UP != False:
      print("up\r\n")
    if gestrue&GESTURE_DOWN != False:
      print("down\r\n")
    if gestrue&GESTURE_LEFT != False:
      print("left\r\n")
    if gestrue&GESTURE_RIGHT != False:
      print("right\r\n")
    if gestrue&GESTURE_FORWARD != False:
      print("forward\r\n")
    if gestrue&GESTURE_BACKWARD != False:
      print("backward\r\n")
    if gestrue&GESTURE_CLOCKWISE != False:
      print("clockwise\r\n")
    if gestrue&GESTURE_COUNTERCLOCKWISE != False:
      print("counter clockwise\r\n")
    if gestrue&GESTURE_WAVE != False:
      print("wave\r\n")
    if gestrue&GESTURE_HOVER != False:
      print("hover\r\n")
    if gestrue&GESTURE_UNKNOWN != False:
      print("unknown\r\n")
    if gestrue&GESTURE_CLOCKWISE_C != False:
      print("continue clockwise\r\n")
    if gestrue&GESTURE_COUNTERCLOCKWISE_C != False:
      print("counter continue clockwise\r\n")
  
  time.sleep(0.1)

if __name__ == "__main__":
  setup()
  while True:
    loop()