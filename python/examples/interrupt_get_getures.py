# -*- coding: utf-8 -*
'''!
  @file  interrupt_get_gestures.py
  @brief 运行本例程可以通过树莓派的io口获取数据是否准备好，来获取手势数据
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
ctype=0
ADDRESS = 0x73
I2C_1   = 0x01


RASPBERRY_INT_PIN = 25              #DRDY Interrupt connect pin, BCM25 i.e. GPIO 6
if ctype==0:
  GR30_10 = DFRobot_GR30_10_I2C(I2C_1, ADDRESS)
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
    # GESTURE_WAVE              It is not suggested to enable rotation gesture (CW/CCW) and wave gesture at the same time.
    # GESTURE_HOVER             Disable other gestures when hover gesture enables.
    # GESTURE_UNKNOWN
    # GESTURE_CLOCKWISE_C        连续顺时针旋转
    # GESTURE_COUNTERCLOCKWISE_C 连续逆时针旋转
  '''
  GR30_10.en_gestures(GESTURE_UP|GESTURE_DOWN|GESTURE_LEFT|GESTURE_RIGHT|GESTURE_FORWARD|GESTURE_BACKWARD|GESTURE_CLOCKWISE|GESTURE_COUNTERCLOCKWISE|GESTURE_CLOCKWISE_C|GESTURE_COUNTERCLOCKWISE_C)
  
  GPIO.setmode(GPIO.BCM)
  '''
    Set pin gestures, configure input gestures,
      pull_up_down=GPIO.PUD_DOWN   When pin DRDY is configured high polarity, pin RASPBERR_PIN_DRDY is configured pull-down input.
      pull_up_down=GPIO.PUD_UP     When pin DRDY is configured low polarity, pin RASPBERR_PIN_DRDY is configured pull-up input.    
  '''
  GPIO.setup(RASPBERRY_INT_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
  
  GR30_10.set_udlr_win(20, 20)
  GR30_10.set_left_range(1)
  GR30_10.set_right_range(10)
  GR30_10.set_up_range(10)
  GR30_10.set_down_range(10)

  GR30_10.set_forward_range(10)
  GR30_10.set_backward_range(10)
  
  GR30_10.set_wave_number(5)
  GR30_10.set_hover_win(30, 30)
  GR30_10.set_hover_timer(20)    # 每个值大约10ms
  GR30_10.set_cws_angle(16)
  GR30_10.set_ccw_angle(16)
  GR30_10.set_cws_angle_count(4)
  GR30_10.set_ccw_angle_count(4)
    
  
def loop():
  if GPIO.input(RASPBERRY_INT_PIN) == GPIO.LOW:
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

if __name__ == "__main__":
  setup()
  while True:
    loop()