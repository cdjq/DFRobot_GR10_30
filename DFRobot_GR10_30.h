/*!
 * @file DFRobot_GR10_30.h
 * @brief 这是GR10_30的方法说明文件
 * @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license     The MIT License (MIT)
 * @author [zhixinliu](zhixinliu@dfrobot.com)
 * @version  V0.1
 * @date  2022-07-24
 * @url https://github.com/DFRobor/DFRobot_GR10_30
 */
#ifndef DFROBOT_GR10_30_H
#define DFROBOT_GR10_30_H

#include "Arduino.h"
#include "Wire.h"
#include "DFRobot_RTU.h"
#include "String.h"
#if (defined ARDUINO_AVR_UNO) && (defined ESP8266)
#include "SoftwareSerial.h"
#else
#include "HardwareSerial.h"
#endif

//#define ENABLE_DBG ///< 打开这个宏, 可以看到程序的详细运行过程
#ifdef ENABLE_DBG
#define DBG(...) {Serial.print("[");Serial.print(__FUNCTION__); Serial.print("(): "); Serial.print(__LINE__); Serial.print(" ] "); Serial.println(__VA_ARGS__);}
#else
#define DBG(...)
#endif

class DFRobot_GR10_30:public DFRobot_RTU{
public:
  #define GR10_30_DEVICE_ADDR          0x73
  //输入寄存器
  #define GR10_30_INPUTREG_PID         0x00   ///< 设备PID
  #define GR10_30_INPUTREG_VID         0x01   ///<设备的VID,固定为0x3343
  #define GR10_30_INPUTREG_ADDR        0x02   ///<模块的设备地址
  #define GR10_30_INPUTREG_BAUDRATE    0x03   ///<串口波特率
  #define GR10_30_INPUTREG_STOPBIT     0x04   ///<串口校验位和停止位
  #define GR10_30_INPUTREG_VERSION     0x05   ///<固件版本信息
  #define R_DATA_READY                 0X06   ///<数据准备好的寄存器
  #define R_INTERRUPT_STATE            0X07   ///<中断状态的寄存器
  #define R_EXIST_STATE                0X08   ///<物体是否存在的寄存器

  //保持寄存器
  #define R_INTERRUPT_MODE        0X09    ///> 产生中断的手势
  #define R_LRUP_WIN              0X0A    ///> 上下左右感兴趣的窗口
  #define R_L_RANGE               0X0B    ///> 向左滑动的距离
  #define R_R_RANGE               0X0C    ///> 向右滑动的距离
  #define R_U_RANGE               0X0D    ///> 向上滑动的距离
  #define R_D_RANGE               0X0E    ///> 向下滑动的距离
  #define R_FORWARD_RANGE         0X0F    ///> 向前滑动的距离
  #define R_BACKUP_RANGE          0X10    ///> 向后滑动的距离
  #define R_WAVE_COUNT            0X11    ///> 挥手次数
  #define R_HOVR_WIN              0X12    ///> 悬停感兴趣的窗口
  #define R_HOVR_TIMER            0X13    ///> 悬停的时间
  #define R_CWS_ANGLE             0X14    ///> 顺时针旋转的角度 每个值代表 22.5度
  #define R_CCW_ANGLE             0X15    ///> 逆时针旋转的角度 每个值代表 22.5度
  #define R_CWS_ANGLE_COUNT       0X16    ///> 顺时针连续旋转的角度 每个值代表 22.5度
  #define R_CCW_ANGLE_COUNT       0X17    ///> 逆时针连续旋转的角度 每个值代表 22.5度
  #define R_RESET                 0X18    ///> 复位传感器
  

  #define GESTURE_UP                      (1<<0)
  #define GESTURE_DOWN                    (1<<1)
  #define GESTURE_LEFT                    (1<<2)
  #define GESTURE_RIGHT                   (1<<3)
  #define GESTURE_FORWARD                 (1<<4)
  #define GESTURE_BACKWARD                (1<<5)
  #define GESTURE_CLOCKWISE               (1<<6)
  #define GESTURE_COUNTERCLOCKWISE        (1<<7)
  #define GESTURE_WAVE                    (1<<8)
  #define GESTURE_HOVER                   (1<<9)
  #define GESTURE_UNKNOWN                 (1<<10)
  #define GESTURE_CLOCKWISE_C             (1<<14)
  #define GESTURE_COUNTERCLOCKWISE_C      (1<<15)

  /**
   * @fn DFRobot_GR10_30
   * @brief DFRobot_GR10_30 constructor
   * @param pWire I2C pointer to the TowWire stream, which requires calling begin in the demo to init Arduino I2C config.
   * @param addr  I2C communication address of SEN0543 device
   */
  DFRobot_GR10_30(uint8_t addr, TwoWire *pWire = &Wire);

  /**
   * @fn DFRobot_GR10_30
   * @brief DFRobot_GR10_30 constructor
   * @param addr: The device address of the communication between the host computer and SEN0543 slave device
   * @n     SEN0543_DEVICE_ADDR or 115(0X73): Default address of SEN0543 device, if users do not change the device address, it's default to 115.
   * @param s   : The serial port pointer to the Stream, which requires calling begin in the demo to init communication serial port config of Arduino main controller, in line with that of SEN0543 device slave.
   * @n SEN0543 serial port config: 9600 baud rate, 8-bit data bit, no check bit, 1 stop bit, the parameters can't be changed.
   */
  DFRobot_GR10_30(uint8_t addr, Stream *s);
  ~DFRobot_GR10_30(){};

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
   *  GESTURE_CLOCKWISE_C
   *  GESTURE_COUNTERCLOCKWISE_C
   * @return NONE
   */
  void enGestures(uint16_t gestures);

  /**
   * @fn setUdlrWin
   * @brief 设置上下左右感兴趣的窗口
   * @param udSize 上下的距离      距离范围 0-31
   * @param lrSize 左右的距离      距离范围 0-31
   * @return NONE
   */
  void setUdlrWin(uint8_t udSize, uint8_t lrSize);

  /**
   * @fn setLeftRange
   * @brief 设置向左滑动多少距离才能识别
   * @param range
   * @n     距离范围 0-31,必须小于感兴趣的左右距离
   * @return NONE
   */
  void setLeftRange(uint8_t range);

  /**
   * @fn setRightRange
   * @brief 设置向右滑动多少距离才能识别
   * @param range
   * @n     距离范围 0-31,必须小于感兴趣的左右距离
   * @return NONE
   */
  void setRightRange(uint8_t range);

  /**
   * @fn setUpRange
   * @brief 设置向上滑动多少距离才能识别
   * @param range
   * @n     距离范围 0-31,必须小于感兴趣的上下距离
   * @return NONE
   */
  void setUpRange(uint8_t range);

  /**
   * @fn setDownRange
   * @brief 设置向下滑动多少距离才能识别
   * @param range
   * @n     距离范围 0-31,必须小于感兴趣的上下距离
   * @return NONE
   */
  void setDownRange(uint8_t range);

  /**
   * @fn setForwardRange
   * @brief 设置向前移动多少距离才能识别
   * @param range
   * @n     距离范围 0-31
   * @return NONE
   */
  void setForwardRange(uint8_t range);

  /**
   * @fn setBackwardRange
   * @brief 设置向后移动多少距离才能识别
   * @param range
   * @n     距离范围 0-31
   * @return NONE
   */
  void setBackwardRange(uint8_t range);

  /**
   * @fn setWaveNumber
   * @brief 设置挥手多少次才能识别
   * @param number
   * @n     次数范围 0-15
   * @return NONE
   */
  void setWaveNumber(uint8_t number);

  /**
   * @fn setHovrWin
   * @brief 设置悬停感兴趣的窗口
   * @param udSize 上下的距离      距离范围 0-31
   * @param lrSize 左右的距离      距离范围 0-31
   * @return NONE
   */
  void setHovrWin(uint8_t udSize, uint8_t lrSize);

  /**
   * @fn setHovrTimer
   * @brief 设置悬停多少时间才能触发手势
   * @param timer
   * @n     timer 1-0x3ff  10ms-10s  默认为0X3c 600ms
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

protected:
  bool detectDeviceAddress(uint8_t addr);
  void setDefaultConfig(void);
  void resetSensor(void);
  uint8_t readReg(uint16_t reg, void *pBuf, uint8_t size);
  uint8_t writeReg(uint8_t reg, void *pBuf, size_t size);
  TwoWire   *_pWire = NULL;
  Stream    *_s = NULL;
  uint8_t   _addr;
};


#endif
