/*!
 * @file  softGetData.ino
 * @brief 运行本例程可以使用中断读取手势
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

#define INT_PIN 9         // 连接传感器的中断 INT 引脚

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

/** 设置想要采集到的手势
 *  GESTURE_UP
 *  GESTURE_DOWN
 *  GESTURE_LEFT
 *  GESTURE_RIGHT
 *  GESTURE_FORWARD
 *  GESTURE_BACKWARD
 *  GESTURE_CLOCKWISE
 *  GESTURE_COUNTERCLOCKWISE
 *  GESTURE_WAVE
 *  GESTURE_HOVER
 *  GESTURE_UNKNOWN
 *  GESTURE_CLOCKWISE_C
 *  GESTURE_COUNTERCLOCKWISE_C
 */
  gr10_30.setMode(GESTURE_UP|GESTURE_DOWN|GESTURE_LEFT|GESTURE_RIGHT|GESTURE_FORWARD|GESTURE_BACKWARD|GESTURE_CLOCKWISE|GESTURE_COUNTERCLOCKWISE|GESTURE_CLOCKWISE_C|GESTURE_COUNTERCLOCKWISE_C);

//   开启后使用更加详细的配置，不开启使用默认的配置
//   gr10_30.setUdlrWin(30, 30);
//   gr10_30.setLeftRange(10);
//   gr10_30.setRightRange(10);
//   gr10_30.setUpRange(10);
//   gr10_30.setDownRange(10);
//   gr10_30.setForwardRange(10);
//   gr10_30.setBackwardRange(10);
//   gr10_30.setWaveNumber(2);
//   gr10_30.setHovrWin(20,20);
//   gr10_30.setHovrTimer(0x3C);
//   gr10_30.setCwsAngle(16);
//   gr10_30.setCcwAngle(16);
//   gr10_30.setCwsAngleCount(8);
//   gr10_30.setCcwAngleCount(8);

  /* 连接开发板的中断引脚 */
  pinMode(/*Pin */INT_PIN ,INPUT);
}
void loop()
{
  uint16_t gestures;
  if(digitalRead(INT_PIN) == LOW){
    gestures = gr10_30.getGesturesState();
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