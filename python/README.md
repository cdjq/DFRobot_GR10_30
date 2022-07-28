DFRobot_GR10_30
===========================

- [中文版](./README_CN.md)

SEN0543是一个用于图像分析传感器系统的集成姿态识别传感器。它可以识别10种手势，如上、下、左、右、前、后、圆顺时针、圆逆时针、摆动和悬停。

![产品效果图](../../resources/images/SEN0543.png)

## Product Link (https://www.dfrobot.com)

    SKU：SEN0543

## Table of Contents

  * [summary](#summary)
  * [installation](#installation)
  * [methods](#methods)
  * [compatibility](#compatibility)
  * [history](#history)
  * [credits](#credits)

## Summary

SEN0543是一个可以用软件和硬件io口获得数据的高性能手势识别传感器

## Installation

Download this library to Raspberry Pi before use, then open the routine folder. Type python demox.py on the command line to execute a routine demox.py. For example, to execute the control_led.py routine, you need to enter:

```python
python interrupt_get_data.py
python soft_get_data.py
```

## Methods

```python
  def begin(self):
    '''!
      @brief 初始化传感器
    '''

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

  def set_udlr_win(self, ud_size, lr_size):
    '''!
      @brief 设置上下左右感兴趣的窗口
      @param udSize 上下的距离      最大距离为31
      @param lrSize 左右的距离      最大距离为31
      @return NONE
    '''

  def set_left_range(self, range):
    '''!
      @brief 设置向左滑动多少距离才能识别
      @param range
      @n     最大距离为31,必须小于感兴趣的左右距离
      @return NONE
    '''

  def set_right_range(self, range):
    '''!
      @brief 设置向右滑动多少距离才能识别
      @param range
      @n     最大距离为31,必须小于感兴趣的左右距离
    '''

  def set_up_range(self, range):
    '''!
      @brief 设置向上滑动多少距离才能识别
      @param range
      @n     最大距离为31,必须小于感兴趣的上下距离
    '''

  def set_down_range(self, range):
    '''!
      @brief 设置向下滑动多少距离才能识别
      @param range
      @n     最大距离为31,必须小于感兴趣的上下距离
    '''

  def set_forward_range(self, range):
    '''!
      @brief 设置向前移动多少距离才能识别
      @param range
      @n     最大距离为31
    '''

  def set_backward_range(self, range):
    '''!
      @brief 设置向后移动多少距离才能识别
      @param range
      @n     最大距离为31
    '''

  def set_wave_number(self, number):
    '''!
      @brief 设置挥手多少次才能识别
      @param number
      @n     最大次数为15
      @return NONE
    '''

  def set_hover_win(self, ud_size, lr_size):
    '''!
      @brief 设置上下左右感兴趣的窗口
      @param udSize 上下的距离      最大距离为31
      @param lrSize 左右的距离      最大距离为31
      @return NONE
    '''

  def set_hover_timer(self, timer):
    '''!
      @brief 设置悬停多少时间才能触发手势
      @param timer
      @n     timer 最大0x03ff 默认为0X3c
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
      @n     先触发顺/逆时针旋转手势, 当还继续旋转时, 每90度触发一次手势
      @return NONE
    '''
  def set_ccw_angle_count(self, count):
    '''!
      @brief 设置逆时针连续旋转多少角度才能触发手势
      @param count 默认为 4 最大为31
      @n     count 连续旋转的度数为22.5 * count
      @n     例: count = 4 22.5*count = 90
      @n     先触发顺/逆时针旋转手势, 当还继续旋转时, 每90度触发一次手势
      @return NONE
    '''
```

## Compatibility

* RaspberryPi Version

| Board        | Work Well | Work Wrong | Untested | Remarks |
| ------------ | :-------: | :--------: | :------: | ------- |
| Raspberry Pi2 |           |            |    √     |         |
| Raspberry Pi3 |           |            |    √     |         |
| Raspberry Pi4 |       √   |            |          |         |

* Python Version

| Python  | Work Well | Work Wrong | Untested | Remarks |
| ------- | :-------: | :--------: | :------: | ------- |
| Python2 |     √     |            |          |         |
| Python3 |     √     |            |          |         |

## History

- 2022-07-28 - Version 1.0.0 released.

## Credits

Written by zhixinliu(zhixinliu@dfrobot.com), 2021. (Welcome to our [website](https://www.dfrobot.com/))
