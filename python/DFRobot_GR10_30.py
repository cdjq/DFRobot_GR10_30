# -*- coding: utf-8 -*
'''!
  @file        DFRobot_GR10_30.py
  @brief       这是GR30_10传感器基库
  @copyright   Copyright (c) 2021 DFRobot Co.Ltd (http://www.dfrobot.com)
  @license     The MIT License (MIT)
  @author      zhixinliu(zhixinliu@dfrobot.com)
  @version     V1.0
  @date        2022-07-27
  @url         https://github.com/DFRobor/DFRobot_GR10_30
'''

import serial
import time
import smbus
import os
import math
import RPi.GPIO as GPIO
import math

import modbus_tk
import modbus_tk.defines as cst
from modbus_tk import modbus_rtu

I2C_MODE                  = 0x01
UART_MODE                 = 0x02
DEV_ADDRESS               = 0x73

#输入寄存器
GR30_10_INPUTREG_PID                           =0x00   #设备PID
GR30_10_INPUTREG_VID                           =0x01   #设备的VID,固定为0x3343
GR30_10_INPUTREG_ADDR                          =0x02   #模块的设备地址
GR30_10_INPUTREG_BAUDRATE                      =0x03   #串口波特率
GR30_10_INPUTREG_STOPBIT                       =0x04   #串口校验位和停止位
GR30_10_INPUTREG_VERSION                       =0x05   #固件版本信息
GR30_10_INPUTREG_DATA_READY                    =0x06   #数据准备好的寄存器
GR30_10_INPUTREG_INTERRUPT_STATE               =0x07   #手势中断状态
GR30_10_INPUTREG_EXIST_STATE                   =0x08   #存在状态
#保持寄存器
GR30_10_HOLDINGREG_INTERRUPT_MODE               =0x09   #产生中断的手势
GR30_10_HOLDINGREG_LRUP_WIN                     =0x0a   #上下左右感兴趣的窗口
GR30_10_HOLDINGREG_L_RANGE                      =0x0b   #向左滑动的距离
GR30_10_HOLDINGREG_R_RANGE                      =0x0c   #向右滑动的距离
GR30_10_HOLDINGREG_U_RANGE                      =0x0d   #向上滑动的距离
GR30_10_HOLDINGREG_D_RANGE                      =0x0e   #向下滑动的距离
GR30_10_HOLDINGREG_FORWARD_RANGE                =0x0f   #向前滑动的距离
GR30_10_HOLDINGREG_BACKWARD_RANGE               =0x10   #向后滑动的距离
GR30_10_HOLDINGREG_WAVE_COUNT                   =0x11   #挥手次数
GR30_10_HOLDINGREG_HOVR_WIN                     =0x12   #悬停感兴趣的窗口
GR30_10_HOLDINGREG_HOVR_TIMER                   =0x13   #悬停的时间
GR30_10_HOLDINGREG_CWS_ANGLE                    =0x14   #顺时针旋转的角度 每个值代表 22.5度
GR30_10_HOLDINGREG_CCW_ANGLE                    =0x15   #逆时针旋转的角度 每个值代表 22.5度
GR30_10_HOLDINGREG_CWS_ANGLE_COUNT              =0x16   #顺时针连续旋转的角度 每个值代表 22.5度
GR30_10_HOLDINGREG_CCW_ANGLE_COUNT              =0x17   #逆时针连续旋转的角度 每个值代表 22.5度
GR30_10_HOLDINGREG_RESET                        =0x18   #复位传感器

GESTURE_UP                      = (1<<0)
GESTURE_DOWN                    = (1<<1)
GESTURE_LEFT                    = (1<<2)
GESTURE_RIGHT                   = (1<<3)
GESTURE_FORWARD                 = (1<<4)
GESTURE_BACKWARD                = (1<<5)
GESTURE_CLOCKWISE               = (1<<6)
GESTURE_COUNTERCLOCKWISE        = (1<<7)
GESTURE_WAVE                    = (1<<8)
GESTURE_HOVER                   = (1<<9)
GESTURE_UNKNOWN                 = (1<<10)
GESTURE_CLOCKWISE_C             = (1<<14)
GESTURE_COUNTERCLOCKWISE_C      = (1<<15)

class DFRobot_GR10_30():
  __temp_buffer = [0]*2
  def __init__(self ,bus = 0 ,baud = 9600, mode = I2C_MODE):
    self.mode = 0
    self.resolution = 0
    self.gain = 0
    
    if mode == I2C_MODE:
      self.i2cbus = smbus.SMBus(bus)
      self._uart_i2c = I2C_MODE
    else:
      self.master = modbus_rtu.RtuMaster(serial.Serial(port="/dev/ttyAMA0",baudrate=baud, bytesize=8, parity='N', stopbits=1))
      self.master.set_timeout(1.0)
      self._uart_i2c = UART_MODE

  def _detect_device_address(self):
    '''!
      @brief Get sensor address
      @return  Return sensor address
    '''
    if self._uart_i2c == I2C_MODE:
      rbuf = self._read_reg(GR30_10_INPUTREG_ADDR, 2)
      data = rbuf[0]<<8 | rbuf[1]
    elif self._uart_i2c == UART_MODE:
      rbuf = self._read_reg(GR30_10_INPUTREG_ADDR, 1)
      data = rbuf[0]
    return data

  def begin(self):
    '''!
      @brief 初始化传感器
    '''
    if self._detect_device_address() != DEV_ADDRESS:
      return False
    self.reset_sensor()
    time.sleep(0.5)
    return True

  def set_mode(self, mode):
    '''!
      @brief 设置模块可以识别什么手势，才触发中断
      @param mode 想要识别的手势
      @n     GESTURE_UP
      @n     GESTURE_DOWN
      @n     GESTURE_DOWN
      @n     GESTURE_LEFT
      @n     GESTURE_RIGHT
      @n     GESTURE_FORWARD
      @n     GESTURE_BACKWARD
      @n     GESTURE_CLOCKWISE
      @n     GESTURE_COUNTERCLOCKWISE
      @n     GESTURE_WAVE
      @n     GESTURE_HOVER
      @n     GESTURE_UNKNOWN
      @n     GESTURE_CLOCKWISE_C
      @n     GESTURE_COUNTERCLOCKWISE_C
      @return NONE
    '''
    mode = mode&0xc7ff
    if self._uart_i2c == I2C_MODE:
      self.__temp_buffer[0] = (mode>>8)&0xC7  # 为了改为8bit
      self.__temp_buffer[1] = mode&0x00ff
      self._write_reg(GR30_10_HOLDINGREG_INTERRUPT_MODE, self.__temp_buffer)
    else:
      buffer = [mode]
      self._write_reg(GR30_10_HOLDINGREG_INTERRUPT_MODE, buffer)
    time.sleep(0.1)

  def set_udlr_win(self, ud_size, lr_size):
    '''!
      @brief 设置上下左右感兴趣的窗口
      @param udSize 上下的距离      最大距离为31
      @param lrSize 左右的距离      最大距离为31
      @return NONE
    '''
    lr_size = lr_size&0x001f
    ud_size = ud_size&0x001f
    if self._uart_i2c == I2C_MODE:
      self.__temp_buffer[0] = lr_size
      self.__temp_buffer[1] = ud_size
      self._write_reg(GR30_10_HOLDINGREG_LRUP_WIN, self.__temp_buffer)
    else:
      buffer = [lr_size<<8|ud_size]
      self._write_reg(GR30_10_HOLDINGREG_LRUP_WIN, buffer)
    time.sleep(0.1)

  def set_left_range(self, range):
    '''!
      @brief 设置向左滑动多少距离才能识别
      @param range
      @n     最大距离为31,必须小于感兴趣的左右距离
      @return NONE
    '''
    range = range&0x1f
    if self._uart_i2c == I2C_MODE:
      self.__temp_buffer[0] = 0
      self.__temp_buffer[1] = range
      self._write_reg(GR30_10_HOLDINGREG_L_RANGE, self.__temp_buffer)
    else:
      buffer = [range]
      self._write_reg(GR30_10_HOLDINGREG_L_RANGE, buffer)
    time.sleep(0.1)

  def set_right_range(self, range):
    '''!
      @brief 设置向右滑动多少距离才能识别
      @param range
      @n     最大距离为31,必须小于感兴趣的左右距离
    '''
    range = range&0x1f
    if self._uart_i2c == I2C_MODE:
      self.__temp_buffer[0] = 0
      self.__temp_buffer[1] = range
      self._write_reg(GR30_10_HOLDINGREG_R_RANGE, self.__temp_buffer)
    else:
      buffer = [range]
      self._write_reg(GR30_10_HOLDINGREG_R_RANGE, buffer)
    time.sleep(0.1)

  def set_up_range(self, range):
    '''!
      @brief 设置向上滑动多少距离才能识别
      @param range
      @n     最大距离为31,必须小于感兴趣的上下距离
    '''
    range = range&0x1f
    if self._uart_i2c == I2C_MODE:
      self.__temp_buffer[0] = 0
      self.__temp_buffer[1] = range
      self._write_reg(GR30_10_HOLDINGREG_U_RANGE, self.__temp_buffer)
    else:
      buffer = [range]
      self._write_reg(GR30_10_HOLDINGREG_U_RANGE, buffer)
    time.sleep(0.1)

  def set_down_range(self, range):
    '''!
      @brief 设置向下滑动多少距离才能识别
      @param range
      @n     最大距离为31,必须小于感兴趣的上下距离
    '''
    range = range&0x1f
    if self._uart_i2c == I2C_MODE:
      self.__temp_buffer[0] = 0
      self.__temp_buffer[1] = range
      self._write_reg(GR30_10_HOLDINGREG_D_RANGE, self.__temp_buffer)
    else:
      buffer = [range]
      self._write_reg(GR30_10_HOLDINGREG_D_RANGE, buffer)
    time.sleep(0.1)

  def set_forward_range(self, range):
    '''!
      @brief 设置向前移动多少距离才能识别
      @param range
      @n     最大距离为31
    '''
    range = range&0x1f
    if self._uart_i2c == I2C_MODE:
      self.__temp_buffer[0] = 0
      self.__temp_buffer[1] = range
      self._write_reg(GR30_10_HOLDINGREG_FORWARD_RANGE, self.__temp_buffer)
    else:
      buffer = [range]
      self._write_reg(GR30_10_HOLDINGREG_FORWARD_RANGE, buffer)
    time.sleep(0.1)

  def set_backward_range(self, range):
    '''!
      @brief 设置向后移动多少距离才能识别
      @param range
      @n     最大距离为31
    '''
    range = range&0x1f
    if self._uart_i2c == I2C_MODE:
      self.__temp_buffer[0] = 0
      self.__temp_buffer[1] = range
      self._write_reg(GR30_10_HOLDINGREG_BACKWARD_RANGE, self.__temp_buffer)
    else:
      buffer = [range]
      self._write_reg(GR30_10_HOLDINGREG_BACKWARD_RANGE, buffer)
    time.sleep(0.1)

  def set_wave_number(self, number):
    '''!
      @brief 设置挥手多少次才能识别
      @param number
      @n     最大次数为15
      @return NONE
    '''
    number = number&0x0f
    if self._uart_i2c == I2C_MODE:
      self.__temp_buffer[0] = 0
      self.__temp_buffer[1] = number
      self._write_reg(GR30_10_HOLDINGREG_WAVE_COUNT, self.__temp_buffer)
    else:
      buffer = [number]
      self._write_reg(GR30_10_HOLDINGREG_WAVE_COUNT, buffer)
    time.sleep(0.1)

  def set_hover_win(self, ud_size, lr_size):
    '''!
      @brief 设置上下左右感兴趣的窗口
      @param udSize 上下的距离      最大距离为31
      @param lrSize 左右的距离      最大距离为31
      @return NONE
    '''
    lr_size = lr_size&0x001f
    ud_size = ud_size&0x001f
    if self._uart_i2c == I2C_MODE:
      self.__temp_buffer[0] = lr_size
      self.__temp_buffer[1] = ud_size
      self._write_reg(GR30_10_HOLDINGREG_HOVR_WIN, self.__temp_buffer)
    else:
      buffer = [lr_size<<8|ud_size]
      self._write_reg(GR30_10_HOLDINGREG_HOVR_WIN, buffer)
    time.sleep(0.1)

  def set_hover_timer(self, timer):
    '''!
      @brief 设置悬停多少时间才能触发手势
      @param timer
      @n     timer 最大0x03ff 默认为0X3c
    '''
    timer = timer&0x03FF
    if self._uart_i2c == I2C_MODE:
      self.__temp_buffer[0] = (timer>>8)&0x03 # 为了改为8bit
      self.__temp_buffer[1] = timer&0x00ff    # 为了改为8bit
      self._write_reg(GR30_10_HOLDINGREG_HOVR_TIMER, self.__temp_buffer)
    else:      
      buffer = [timer]
      self._write_reg(GR30_10_HOLDINGREG_HOVR_TIMER, buffer)
    time.sleep(0.1) 
    
  def set_cws_angle(self, count):
    '''!
      @brief 设置顺时针旋转多少角度才能触发手势
      @param count 默认为 16 最大为31
      @n     count 旋转的度数为22.5 * count
      @n     例: count = 16 22.5*count = 360  旋转360度触发手势
      @return NONE
    '''
    count = count&0x1f
    if self._uart_i2c == I2C_MODE:
      self.__temp_buffer[0] = 0
      self.__temp_buffer[1] = count
      self._write_reg(GR30_10_HOLDINGREG_CWS_ANGLE, self.__temp_buffer)
    else:
      buffer = [count]
      self._write_reg(GR30_10_HOLDINGREG_CWS_ANGLE, buffer)
    time.sleep(0.1)  

  def set_ccw_angle(self, count):
    '''!
      @brief 设置逆时针旋转多少角度才能触发手势
      @param count 默认为 16 最大为31
      @n     count 旋转的度数为22.5 * count
      @n     例: count = 16 22.5*count = 360  旋转360度触发手势
      @return NONE
    '''
    count = count&0x1f
    if self._uart_i2c == I2C_MODE:
      self.__temp_buffer[0] = 0
      self.__temp_buffer[1] = count
      self._write_reg(GR30_10_HOLDINGREG_CCW_ANGLE, self.__temp_buffer)
    else:
      buffer = [count]
      self._write_reg(GR30_10_HOLDINGREG_CCW_ANGLE, buffer)
    time.sleep(0.1)
    
  def set_cws_angle_count(self, count):
    '''!
      @brief 设置顺时针连续旋转多少角度才能触发手势
      @param count 默认为 4 最大为31
      @n     count 连续旋转的度数为22.5 * count
      @n     例: count = 4 22.5*count = 90
      @n     先触发顺/逆时针旋转手势, 当还继续旋转时, 每90度触发一次手势
      @return NONE
    '''
    count = count&0x1f 
    if self._uart_i2c == I2C_MODE:
      self.__temp_buffer[0] = 0
      self.__temp_buffer[1] = count
      self._write_reg(GR30_10_HOLDINGREG_CWS_ANGLE_COUNT, self.__temp_buffer)
    else:
      buffer = [count]
      self._write_reg(GR30_10_HOLDINGREG_CWS_ANGLE_COUNT, buffer)
    time.sleep(0.1)  

  def set_ccw_angle_count(self, count):
    '''!
      @brief 设置逆时针连续旋转多少角度才能触发手势
      @param count 默认为 4 最大为31
      @n     count 连续旋转的度数为22.5 * count
      @n     例: count = 4 22.5*count = 90
      @n     先触发顺/逆时针旋转手势, 当还继续旋转时, 每90度触发一次手势
      @return NONE
    '''
    count = count&0x1f
    if self._uart_i2c == I2C_MODE:
      self.__temp_buffer[0] = 0
      self.__temp_buffer[1] = count
      self._write_reg(GR30_10_HOLDINGREG_CCW_ANGLE_COUNT, self.__temp_buffer)
    else:
      buffer = [count]
      self._write_reg(GR30_10_HOLDINGREG_CCW_ANGLE_COUNT, buffer)
    time.sleep(0.1)


  def reset_sensor(self):
    '''!
      @brief 复位传感器
      @return NONE
    '''
    if self._uart_i2c == I2C_MODE:
      self.__temp_buffer[0] = 0x55
      self.__temp_buffer[1] = 0x00
      self._write_reg(GR30_10_HOLDINGREG_RESET, self.__temp_buffer)
    else:
      buffer = [0x55]
      self._write_reg(GR30_10_HOLDINGREG_RESET, buffer)
    
    time.sleep(0.1)
    
    
  def get_data_ready(self):
    '''!
      @brief 获取是否检测到手势了
      @return 是否检测到手势
      @retval True  检测到手势
      @retval False  未检测到手势
    '''
    if self._uart_i2c == I2C_MODE:
      rbuf = self._read_reg(GR30_10_INPUTREG_DATA_READY, 2)
      data = rbuf[0]*256 + rbuf[1]
      #print "data ready"
      #print data
    elif self._uart_i2c == UART_MODE:
      rbuf = self._read_reg(GR30_10_INPUTREG_DATA_READY, 2)
      data = rbuf[0]
    if data == 0x01:
      return True
    else:
      return False
    
  def get_gestures(self):
    '''!
      @brief 获取手势类型
      @return 手势类型
      @retval GESTURE_UP
      @retval GESTURE_DOWN
      @retval GESTURE_DOWN
      @retval GESTURE_LEFT
      @retval GESTURE_RIGHT
      @retval GESTURE_FORWARD
      @retval GESTURE_BACKWARD
      @retval GESTURE_CLOCKWISE
      @retval GESTURE_COUNTERCLOCKWISE
      @retval GESTURE_WAVE
      @retval GESTURE_HOVER
      @retval GESTURE_UNKNOWN
      @retval GESTURE_CLOCKWISE_C
      @retval GESTURE_COUNTERCLOCKWISE_C
    '''
    if self._uart_i2c == I2C_MODE:
      rbuf = self._read_reg(GR30_10_INPUTREG_INTERRUPT_STATE, 2)
      data = (rbuf[0]&0xff)*256 + rbuf[1]
    elif self._uart_i2c == UART_MODE:
      rbuf = self._read_reg(GR30_10_INPUTREG_INTERRUPT_STATE, 1)
      data = rbuf[0]
    return data
  
  
  def get_exist(self):
    '''!
      @brief 获取物体是否在传感器检测范围内
      @return 是否存在
      @retval 0 不存在
      @retval 1 存在
    '''
    if self._uart_i2c == I2C_MODE:
      rbuf = self._read_reg(GR30_10_INPUTREG_EXIST_STATE, 2)
      data = rbuf[0]*256 + rbuf[1]
    elif self._uart_i2c == UART_MODE:
      rbuf = self._read_reg(GR30_10_INPUTREG_EXIST_STATE, 1)
      data = rbuf[0]
    return data
  
class DFRobot_GR30_10_I2C(DFRobot_GR10_30):
  '''!
    @brief An example of an i2c interface module
  '''
  def __init__(self ,bus ,addr):
    self._addr = addr
    DFRobot_GR10_30.__init__(self,bus,0,I2C_MODE)   
    
  
  def _read_reg(self, reg_addr ,length):
    '''!
      @brief read the data from the register
      @param reg_addr register address
      @param length read data
    '''
    reg = reg_addr
    try:
      rslt = self.i2cbus.read_i2c_block_data(self._addr ,reg , length)
    except:
      rslt = [0,0]
    return rslt

  def _write_reg(self, reg_addr ,data):
    '''!
      @brief write the data from the register
      @param reg_addr register address
      @param data 写入寄存器数据
    '''
    self._reg = reg_addr
    try:
      rslt = self.i2cbus.write_i2c_block_data(self._addr ,self._reg , data)
    except:
      rslt = -1
    return rslt        


class DFRobot_GR30_10_UART(DFRobot_GR10_30):
  '''!
    @brief An example of an UART interface module
  '''
  def __init__(self ,baud, addr):
    self._baud = baud
    self._addr = addr
    try:
      DFRobot_GR10_30.__init__(self,0,self._baud,UART_MODE)
    except:
      print ("plese get root!")
   
  
  def _read_reg(self, reg_addr ,length):
    '''!
      @brief Read data from the sensor
    '''
    return list(self.master.execute(self._addr, cst.READ_INPUT_REGISTERS, reg_addr, length))
  
  def _write_reg(self, reg_addr ,data):
    '''!
      @brief write data from the sensor
    '''
    return list(self.master.execute(self._addr, cst.WRITE_MULTIPLE_REGISTERS, reg_addr, output_value=data))