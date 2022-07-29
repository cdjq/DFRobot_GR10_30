/*!
 * @file  getGestures.ino
 * @brief 运行本例程读取手势的类型
 * @n
 * @n connected table
 * ---------------------------------------------------------------------------------------------------------------
 *    board   |             MCU                | Leonardo/Mega2560/M0 |    UNO    | ESP8266 | ESP32 |  microbit  |
 *     VCC    |            3.3V/5V             |        VCC           |    VCC    |   VCC   |  VCC  |     X      |
 *     GND    |              GND               |        GND           |    GND    |   GND   |  GND  |     X      |
 *     RX     |              TX                |     Serial1 TX1      |     5     |   5/D6  |  D2   |     X      |
 *     TX     |              RX                |     Serial1 RX1      |     4     |   4/D7  |  D3   |     X      |
 * ---------------------------------------------------------------------------------------------------------------
 * 
 * @copyright   Copyright (c) 2021 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license     The MIT License (MIT)
 * @author      [zhixinliu](zhixinliu@dfrobot.com)
 * @version     V0.1
 * @date        2022-07-25
 * @url         https://github.com/DFRobor/DFRobot_GR10_30
 */
#include "DFRobot_GR10_30.h"

#if defined(ARDUINO_AVR_UNO)||defined(ESP8266)
#include <SoftwareSerial.h>
#endif

//#define UARTMODE // 串口模式
#define I2CMODE // i2c模式
#if defined UARTMODE
#if defined(ARDUINO_AVR_UNO)||defined(ESP8266)
  SoftwareSerial mySerial(/*rx =*/4, /*tx =*/5);
  DFRobot_GR10_30 gr10_30(/*addr =*/GR10_30_DEVICE_ADDR, /*s =*/&mySerial);
#else
  DFRobot_GR10_30 gr10_30(/*addr =*/GR10_30_DEVICE_ADDR, /*s =*/&Serial1);
#endif
#endif
#if defined I2CMODE
DFRobot_GR10_30 gr10_30(/*addr = */GR10_30_DEVICE_ADDR, /*pWire = */&Wire);
#endif

void setup()
{
#if defined UARTMODE
  //Init MCU communication serial port
#if defined(ARDUINO_AVR_UNO)||defined(ESP8266)
  mySerial.begin(9600);
#elif defined(ESP32)
  Serial1.begin(9600, SERIAL_8N1, /*rx =*/D3, /*tx =*/D2);
#else
  Serial1.begin(9600);
#endif
#endif
  Serial.begin(115200);
  while(gr10_30.begin() != 0){
    Serial.println(" Sensor initialize failed!!");
    delay(1000);
  }
  Serial.println(" Sensor  initialize success!!");
/** 设置想要获取的手势
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
 */
  gr10_30.enGestures(GESTURE_UP|GESTURE_DOWN|GESTURE_LEFT|GESTURE_RIGHT|GESTURE_FORWARD|GESTURE_BACKWARD|GESTURE_CLOCKWISE|GESTURE_COUNTERCLOCKWISE|GESTURE_CLOCKWISE_C|GESTURE_COUNTERCLOCKWISE_C);

// 开启后使用更加详细的配置，不开启使用默认的配置
/**
 * 设置感兴趣的窗口,只在此范围内能采集的数据有效
 * 窗口最大为31 配置的的数字代表 中心距离上下左右的距离
 * 例如 配置上下的距离为30 中心距离上的距离为 15  距离下的范围也为15
 * udSize 上下的距离范围      0-31
 * lrSize 左右的距离范围      0-31
 */
//   gr10_30.setUdlrWin(30, 30);
//   gr10_30.setHovrWin(20, 20);

/**
 * 设置滑动多少距离才能识别为手势
 * 距离范围0-31, 必须小于感兴趣窗口的距离
 */
//   gr10_30.setLeftRange(10);
//   gr10_30.setRightRange(10);
//   gr10_30.setUpRange(10);
//   gr10_30.setDownRange(10);
//   gr10_30.setForwardRange(10);
//   gr10_30.setBackwardRange(10);

/**
 * 设置挥手多少次才能识别
 * 次数范围 0-15
 */
//   gr10_30.setWaveNumber(2);

/**
 * 设置悬停多少时间才能触发手势
 * 1-0x3ff  10ms-10s  默认为0X3c 600ms
 */
//   gr10_30.setHovrTimer(0x3C);

/**
 * 设置旋转多少角度才能触发手势
 * count 默认为 16 范围 0-31
 * count 旋转的度数为22.5 * count
 * count = 16 22.5*count = 360  旋转360度触发手势
 */
//   gr10_30.setCwsAngle(/*count*/16);
//   gr10_30.setCcwAngle(/*count*/16);

/**
 * 设置连续旋转多少角度才能触发手势
 * count 默认为 4  范围 0-31
 * count 连续旋转的度数为22.5 * count
 * 例: count = 4 22.5*count = 90
 * 先触发顺/逆时针旋转手势, 当还继续旋转时, 每90度触发一次连续旋转手势
 */
//   gr10_30.setCwsAngleCount(/*count*/8);
//   gr10_30.setCcwAngleCount(/*count*/8);
}

void loop()
{
  if(gr10_30.getDataReady()){
    uint16_t  gestures = gr10_30.getGesturesState();
    if(gestures&GESTURE_UP){
      Serial.println("Up");
    }
    if(gestures&GESTURE_DOWN){
      Serial.println("Down");
    }
    if(gestures&GESTURE_LEFT){
      Serial.println("Left");
    }
    if(gestures&GESTURE_RIGHT){
      Serial.println("Right");
    }
    if(gestures&GESTURE_FORWARD){
      Serial.println("Forward");
    }
    if(gestures&GESTURE_BACKWARD){
      Serial.println("Backward");
    }
    if(gestures&GESTURE_CLOCKWISE){
      Serial.println("Clockwise");
    }
    if(gestures&GESTURE_COUNTERCLOCKWISE){
      Serial.println("Contrarotate");
    }
    if(gestures&GESTURE_WAVE){
      Serial.println("Wave");
    }
    if(gestures&GESTURE_HOVER){
      Serial.println("Hover");
    }
    if(gestures&GESTURE_CLOCKWISE_C){
      Serial.println("Continuous clockwise");
    }
    if(gestures&GESTURE_COUNTERCLOCKWISE_C){
      Serial.println("Continuous counterclockwise");
    }
  }
  delay(1);
}
