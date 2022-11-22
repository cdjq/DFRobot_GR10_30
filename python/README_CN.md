DFRobot_GR10_30
===========================

* [English Version](./README.md)

SEN0543是一个用于图像分析传感器系统的集成姿态识别传感器。它可以识别10种手势，如上、下、左、右、前、后、圆顺时针、圆逆时针、摆动和悬停。

![产品效果图片](../../resources/images/SEN0543.png)

## 产品链接（https://www.dfrobot.com）

    SKU：SEN0543
  
## 目录

  * [概述](#概述)
  * [库安装](#库安装)
  * [方法](#方法)
  * [兼容性](#兼容性)
  * [历史](#历史)
  * [创作者](#创作者)

## 概述

最远识别距离30cm
可识别12种手势
识别阈值参数可配置
支持UART、I2C通讯


## 库安装

本库使用到了modbus_tk, 使用本库前先检测树莓派是否成功导入modbus_tk, 若导入失败, 请通过以下命令安装modbus_tk库 python2: pip install modbus_tk python3: pip3 install modbus_tk

使用库, 首先下载库文件, 将其粘贴到指定的目录中, 然后打开Examples文件夹并在该文件夹中运行演示。


## 方法

```python
  def begin(self):
    '''!
      @brief 初始化传感器
    '''

  def en_gestures(self, gestures):
    '''!
      @brief 设置模块可以识别什么手势，才触发中断
      @param gestures 想要识别的手势
      @n     GESTURE_UP
      @n     GESTURE_DOWN
      @n     GESTURE_DOWN
      @n     GESTURE_LEFT
      @n     GESTURE_RIGHT
      @n     GESTURE_FORWARD
      @n     GESTURE_BACKWARD
      @n     GESTURE_CLOCKWISE
      @n     GESTURE_COUNTERCLOCKWISE
      @n     GESTURE_WAVE               It is not suggested to enable rotation gesture (CW/CCW) and wave gesture at the same time.
      @n     GESTURE_HOVER             Disable other gestures when hover gesture enables.
      @n     GESTURE_UNKNOWN
      @n     GESTURE_CLOCKWISE_C        连续正转
      @n     GESTURE_COUNTERCLOCKWISE_C 连续反转
      @return NONE
    '''

  def set_udlr_win(self, ud_size, lr_size):
    '''!
      @brief 设置上下左右感兴趣的窗口
      @param udSize 上下的距离      距离范围 1-30
      @param lrSize 左右的距离      距离范围 1-30
      @return NONE
    '''

  def set_left_range(self, range):
    '''!
      @brief 设置向左滑动多少距离才能识别
      @param range
      @n     距离范围 5-25,必须小于感兴趣的左右距离
      @return NONE
    '''

  def set_right_range(self, range):
    '''!
      @brief 设置向右滑动多少距离才能识别
      @param range
      @n     距离范围 5-25,必须小于感兴趣的左右距离
    '''

  def set_up_range(self, range):
    '''!
      @brief 设置向上滑动多少距离才能识别
      @param range
      @n     距离范围 5-25,必须小于感兴趣的上下距离
    '''

  def set_down_range(self, range):
    '''!
      @brief 设置向下滑动多少距离才能识别
      @param range
      @n     距离范围 5-25,必须小于感兴趣的上下距离
    '''

  def set_forward_range(self, range):
    '''!
      @brief 设置向前移动多少距离才能识别
      @param range
      @n     距离范围 1-15
    '''

  def set_backward_range(self, range):
    '''!
      @brief 设置向后移动多少距离才能识别
      @param range
      @n     距离范围 1-15
    '''

  def set_wave_number(self, number):
    '''!
      @brief 设置挥手多少次才能识别
      @param number
      @n     次数范围 1-15
      @return NONE
    '''

  def set_hover_win(self, ud_size, lr_size):
    '''!
      @brief 设置上下左右感兴趣的窗口
      @param udSize 上下的距离      距离范围 1-30
      @param lrSize 左右的距离      距离范围 1-30
      @return NONE
    '''

  def set_hover_timer(self, timer):
    '''!
      @brief 设置悬停多少时间才能触发手势
      @param timer 每个值代表10ms
      @n     timer 最大 为200  默认为60 600ms
    '''

  def set_cws_angle(self, count):
    '''!
      @brief 设置顺时针旋转多少角度才能触发手势
      @param count 默认为 16 最大为31
      @n     count 旋转的度数为22.5 * count
      @n     例: count = 16 22.5*count = 360  旋转360度触发手势
      @return NONE
    '''

  def set_ccw_angle(self, count):
    '''!
      @brief 设置逆时针旋转多少角度才能触发手势
      @param count 默认为 16 最大为31
      @n     count 旋转的度数为22.5 * count
      @n     例: count = 16 22.5*count = 360  旋转360度触发手势
      @return NONE
    '''
    
  def set_cws_angle_count(self, count):
    '''!
      @brief 设置顺时针连续旋转多少角度才能触发手势
      @param count 默认为 4 最大为31
      @n     count 连续旋转的度数为22.5 * count
      @n     例: count = 4 22.5*count = 90
      @n     先触发顺/逆时针旋转手势, 当还继续旋转时, 每90度触发一次连续旋转手势
      @return NONE
    '''
  def set_ccw_angle_count(self, count):
    '''!
      @brief 设置逆时针连续旋转多少角度才能触发手势
      @param count 默认为 4 最大为31
      @n     count 连续旋转的度数为22.5 * count
      @n     例: count = 4 22.5*count = 90
      @n     先触发顺/逆时针旋转手势, 当还继续旋转时, 每90度触发一次连续旋转手势
      @return NONE
    '''
```

## Compatibility

* RaspberryPi Version

| Board        | Work Well | Work Wrong | Untested | Remarks |
| ------------ | :-------: | :--------: | :------: | ------- |
| RaspberryPi2 |           |            |    √     |         |
| RaspberryPi3 |     √     |            |          |         |
| RaspberryPi4 |           |            |     √    |         |

* Python Version

| Python  | Work Well | Work Wrong | Untested | Remarks |
| ------- | :-------: | :--------: | :------: | ------- |
| Python2 |     √     |            |          |         |
| Python3 |     √     |            |          |         |

## History

- 2022-07-28 - 1.0.0 版本

## Credits

Written by zhixinliu(zhixinliu@dfrobot.com), 2021. (Welcome to our [website](https://www.dfrobot.com/))