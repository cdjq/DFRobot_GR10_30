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

要使用这个库, 首先下载库文件( https://github.com/DFRobot/DFRobot_GR10_30 )和依赖文件( https://github.com/DFRobot/DFRobot_RTU ), 将其粘贴到\Arduino\libraries目录中, 然后打开示例文件夹并在文件夹中运行演示。

## 方法

```C++
  /**
   * @fn begin
   * @brief Init SEN0543 device
   * @return Return value init status
   * @retval 0  Succeed
   * @retval -1 Failed
   */
  int8_t begin(void);

  /**
   * @fn enGestures
   * @brief 设置模块可以识别什么手势，才触发中断
   * @param gestures
   *  GESTURE_UP
   *  GESTURE_DOWN
   *  GESTURE_LEFT
   *  GESTURE_RIGHT
   *  GESTURE_FORWARD
   *  GESTURE_BACKWARD
   *  GESTURE_CLOCKWISE
   *  GESTURE_COUNTERCLOCKWISE
   *  GESTURE_WAVE              It is not suggested to enable rotation gesture (CW/CCW) and wave gesture at the same time.
   *  GESTURE_HOVER             Disable other gestures when hover gesture enables.
   *  GESTURE_UNKNOWN
   *  GESTURE_CLOCKWISE_C           // 连续正转
   *  GESTURE_COUNTERCLOCKWISE_C    // 连续反转
   * @return NONE
   */
  void enGestures(uint16_t gestures);

  /**
   * @fn setUdlrWin
   * @brief 设置上下左右感兴趣的窗口
   * @param udSize 上下的距离      距离范围 1-30
   * @param lrSize 左右的距离      距离范围 1-30
   * @return NONE
   */
  void setUdlrWin(uint8_t udSize, uint8_t lrSize);

  /**
   * @fn setLeftRange
   * @brief 设置向左滑动多少距离才能识别
   * @param range
   * @n     距离范围 5-25,必须小于感兴趣的左右距离
   * @return NONE
   */
  void setLeftRange(uint8_t range);

  /**
   * @fn setRightRange
   * @brief 设置向右滑动多少距离才能识别
   * @param range
   * @n     距离范围 5-25,必须小于感兴趣的左右距离
   * @return NONE
   */
  void setRightRange(uint8_t range);

  /**
   * @fn setUpRange
   * @brief 设置向上滑动多少距离才能识别
   * @param range
   * @n     距离范围 5-25,必须小于感兴趣的上下距离
   * @return NONE
   */
  void setUpRange(uint8_t range);

  /**
   * @fn setDownRange
   * @brief 设置向下滑动多少距离才能识别
   * @param range
   * @n     距离范围 5-25,必须小于感兴趣的上下距离
   * @return NONE
   */
  void setDownRange(uint8_t range);

  /**
   * @fn setForwardRange
   * @brief 设置向前移动多少距离才能识别
   * @param range
   * @n     距离范围 1-15
   * @return NONE
   */
  void setForwardRange(uint8_t range);

  /**
   * @fn setBackwardRange
   * @brief 设置向后移动多少距离才能识别
   * @param range
   * @n     距离范围 1-15
   * @return NONE
   */
  void setBackwardRange(uint8_t range);

  /**
   * @fn setWaveNumber
   * @brief 设置挥手多少次才能识别
   * @param number
   * @n     次数范围 1-15
   * @return NONE
   */
  void setWaveNumber(uint8_t number);

  /**
   * @fn setHovrWin
   * @brief 设置悬停感兴趣的窗口
   * @param udSize 上下的距离      距离范围 1-30
   * @param lrSize 左右的距离      距离范围 1-30
   * @return NONE
   */
  void setHovrWin(uint8_t udSize, uint8_t lrSize);

  /**
   * @fn setHovrTimer
   * @brief 设置悬停多少时间才能触发手势
   * @param timer
   * @n     timer 1-200  10ms-2s  默认为 60 600ms
   * @return NONE
   */
  void setHovrTimer(uint16_t timer);

  /**
   * @fn setCwsAngle
   * @brief 设置顺时针旋转多少角度才能触发手势
   * @param count 默认为 16 范围1-31
   * @n     count 旋转的度数为22.5 * count
   * @n     例: count = 16 22.5*count = 360  旋转360度触发手势
   * @return NONE
   */
  void setCwsAngle(uint8_t count);

  /**
   * @fn setCcwAngle
   * @brief 设置逆时针旋转多少角度才能触发手势
   * @param count 默认为 16 范围1-31
   * @n     count 旋转的度数为22.5 * count
   * @n     例: count = 16 22.5*count = 360  旋转360度触发手势
   * @return NONE
   */
  void setCcwAngle(uint8_t count);

  /**
   * @fn setCwsAngleCount
   * @brief 设置顺时针连续旋转多少角度才能触发手势
   * @param count 默认为 4 范围1-31
   * @n     count 连续旋转的度数为22.5 * count
   * @n     例: count = 4 22.5*count = 90
   * @n     先触发顺/逆时针旋转手势, 当还继续旋转时, 每90度触发一次连续旋转手势
   * @return NONE
   */
  void setCwsAngleCount(uint8_t count);

  /**
   * @fn setCcwAngleCount
   * @brief 设置逆时针连续旋转多少角度才能触发手势
   * @param count 默认为 4 范围1-31
   * @n     count 连续旋转的度数为22.5 * count
   * @n     例: count = 4 22.5*count = 90
   * @n     先触发顺/逆时针旋转手势, 当还继续旋转时, 每90度触发一次连续旋转手势
   * @return NONE
   */
  void setCcwAngleCount(uint8_t count);

  /**
   * @fn getExist
   * @brief 获取物体是否在传感器检测范围内
   * @return 是否存在
   * @retval 1  存在
   * @retval 0  不存在
   */
  uint16_t getExist(void);

  /**
   * @fn getDataReady
   * @brief 获取是否检测到手势了
   * @return 是否检测到手势
   * @retval 1  检测到手势
   * @retval 0  未检测到手势
   */
  uint16_t getDataReady(void);

  /**
   * @fn getGesturesState
   * @brief 获取手势类型
   * @return 手势类型
   * @retval GESTURE_UP
   * @retval GESTURE_DOWN
   * @retval GESTURE_DOWN
   * @retval GESTURE_LEFT
   * @retval GESTURE_RIGHT
   * @retval GESTURE_FORWARD
   * @retval GESTURE_BACKWARD
   * @retval GESTURE_CLOCKWISE
   * @retval GESTURE_COUNTERCLOCKWISE
   * @retval GESTURE_WAVE
   * @retval GESTURE_HOVER
   * @retval GESTURE_UNKNOWN
   * @retval GESTURE_CLOCKWISE_C
   * @retval GESTURE_COUNTERCLOCKWISE_C
   */
  uint16_t getGesturesState(void);

```


## 兼容性

MCU                | SoftwareSerial | HardwareSerial |      IIC      |
------------------ | :----------: | :----------: | :----------: | 
Arduino Uno        |      √       |      X       |      √       |
Mega2560           |      √       |      √       |      √       |
Leonardo           |      √       |      √       |      √       |
ESP32              |      X       |      √       |      √       |
ESP8266            |      √       |      X       |      √       |
micro:bit          |      X       |      X       |      √       |
FireBeetle M0      |      X       |      √       |      √       |
raspberry          |      X       |      √       |      √       |

## 历史
- 2022-07-26 - 1.0.0 版本

## 创作者

Written by zhixinliu(zhixinliu@dfrobot.com), 2021. (Welcome to our [website](https://www.dfrobot.com/))