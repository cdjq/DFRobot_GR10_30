DFRobot_GR10_30
===========================

- [中文版](./README_CN.md)

The GR10-30 gesture sensor is an integrated gesture recognition sensor based on image analysis. It is capable of recognizing 10 hand gestures: move up, down, left, right, forward & backward, rotate clockwise & counterclockwise (continuously), hover, and wave. 

![产品效果图](../../resources/images/SEN0543.png)

## Product Link (https://www.dfrobot.com)

    SKU：SEN0543

## Table of Contents

  * [Summary](#summary)
  * [Installation](#installation)
  * [Methods](#methods)
  * [Compatibility](#compatibility)
  * [History](#history)
  * [Credits](#credits)

## Summary

* Maximum Recognition distance of 30cm<br/>
* Capable of recognizing 12 gestures<br/>
* Configurable recognition threshold & other parameters<br/>
* Support UART & I2C communication

## Installation

The library has used modbus_tk, detect whether modbus_tk has been imported into Raspberry Pi before use, if not, run the following command to install modbus_tk library. python2: pip install modbus_tk python3: pip3 install modbus_tk

Download the library file before use, paste them into the specified directory, then open the Examples folder and run the demo in the folder.

## Methods

```python
  def begin(self):
    '''!
      @brief Init the sensor
    '''

  def en_gestures(self, gestures):
    '''!
      @brief Set what gestures the module can recognize to trigger interrupt
      @param gestures Set the gesture to be enabled
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
      @n     GESTURE_CLOCKWISE_C        Rotate clockwise continuously
      @n     GESTURE_COUNTERCLOCKWISE_C Rotate counterclockwise continuously
      @return NONE
    '''

  def set_udlr_win(self, ud_size, lr_size):
    '''!
      @brief Set the detection window 
      @param udSize Distance from top to bottom      distance range 1-30
      @param lrSize Distance from left to right      distance range 1-30
      @return NONE
    '''

  def set_left_range(self, range):
    '''!
      @brief Set how far your hand should move to the left so the sensor can recognize it
      @param range
      @n     Distance range 5-25, must be less than distance from left to right of the detection window
      @return NONE
    '''

  def set_right_range(self, range):
    '''!
      @brief Set how far your hand should move to the right so the sensor can recognize it
      @param range
      @n     Distance range 5-25, must be less than distance from left to right of the detection window
    '''

  def set_up_range(self, range):
    '''!
      @brief Set how far your hand should move up so the sensor can recognize it
      @param range
      @n     Distance range 5-25, must be less than distance from top to bottom of the detection window
    '''

  def set_down_range(self, range):
    '''!
      @brief Set how far your hand should move down so the sensor can recognize it
      @param range
      @n     Distance range 5-25, must be less than distance from top to bottom of the detection window
    '''

  def set_forward_range(self, range):
    '''!
      @brief Set how far your hand should move forward so the sensor can recognize it
      @param range
      @n     Distance range 1-15
    '''

  def set_backward_range(self, range):
    '''!
      @brief Set how far your hand should move backward so the sensor can recognize it
      @param range
      @n     Distance range 1-15
    '''

  def set_wave_number(self, number):
    '''!
      @brief Set how many times you need to wave hands so the sensor can recognize it
      @param number
      @n     Number range 1-15
      @return NONE
    '''

  def set_hover_win(self, ud_size, lr_size):
    '''!
      @brief Set hover detection window
      @param udSize Distance from top to bottom      distance range 1-30
      @param lrSize Distance from left to right      distance range 1-30
      @return NONE
    '''

  def set_hover_timer(self, timer):
    '''!
      @brief Set how long your hand should hover to trigger the gesture
      @param timer Each value represents 10ms
      @n     timer Maximum is 200  default is 60 600ms
    '''

  def set_cws_angle(self, count):
    '''!
      @brief Set how many degrees your hand should rotate clockwise to trigger the gesture
      @param count Default 16, maximum 31
      @n     count Rotation angle = 22.5 * count
      @n     For example: count = 16, 22.5*count = 360, rotate 360° to trigger the gesture
      @return NONE
    '''

  def set_ccw_angle(self, count):
    '''!
      @brief Set how many degrees your hand should rotate counterclockwise to trigger the gesture
      @param count Default 16, maximum 31
      @n     count Rotation angle = 22.5 * count
      @n     For example: count = 16, 22.5*count = 360, rotate 360° to trigger the gesture
      @return NONE
    '''
    
  def set_cws_angle_count(self, count):
    '''!
      @brief Set how many degrees your hand should rotate clockwise continuously to trigger the gesture
      @param count Default 4, maximum 31
      @n     count Continuous rotation angle = 22.5 * count
      @n     For example: count = 4 22.5*count = 90
      @n     Trigger the clockwise/counterclockwise rotation gesture first, 
      @n     if keep rotating, then the continuous rotation gesture will be triggered once every 90°
      @return NONE
    '''
  def set_ccw_angle_count(self, count):
    '''!
      @brief Set how many degrees your hand should rotate counterclockwise continuously to trigger the gesture
      @param count Default 4, maximum 31
      @n     count Continuous rotation angle = 22.5 * count
      @n     For example: count = 4 22.5*count = 90
      @n     Trigger the clockwise/counterclockwise rotation gesture first, 
      @n     if keep rotating, then the continuous rotation gesture will be triggered once every 90°
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
