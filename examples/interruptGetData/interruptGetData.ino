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

volatile uint8_t interruptFlag = 0;
void myInterrupt(void)
{
  interruptFlag = 1;    // Interrupt flag
  #if defined(ESP32) || defined(ESP8266) || defined(ARDUINO_SAM_ZERO)
    detachInterrupt(13);   // Detach interrupt
  #else
    detachInterrupt(0);   // Detach interrupt
  #endif
}

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

// 连接开发板的中断引脚
#if defined(ESP32) || defined(ESP8266)
  /**!
    Select according to the set DADY pin polarity
      INPUT_PULLUP    // Low polarity, set pin 13 to pull-up input
      INPUT_PULLDOWN  // High polarity, set pin 13 to pull-down input
    interput io
      All pins can be used. Pin 13 is recommended
  */
  pinMode(/*Pin */13 ,INPUT_PULLUP);
  attachInterrupt(/*interput io*/13, myInterrupt, FALLING);
#elif defined(ARDUINO_SAM_ZERO)
  pinMode(/*Pin */13 ,INPUT_PULLUP);
  attachInterrupt(/*interput io*/13, myInterrupt, FALLING);
#else
  /**!    The Correspondence Table of AVR Series Arduino Interrupt Pins And Terminal Numbers
   * ---------------------------------------------------------------------------------------
   * |                                        |    Pin       | 2  | 3  |                   |
   * |    Uno, Nano, Mini, other 328-based    |--------------------------------------------|
   * |                                        | Interrupt No | 0  | 1  |                   |
   * |-------------------------------------------------------------------------------------|
   * |                                        |    Pin       | 2  | 3  | 21 | 20 | 19 | 18 |
   * |               Mega2560                 |--------------------------------------------|
   * |                                        | Interrupt No | 0  | 1  | 2  | 3  | 4  | 5  |
   * |-------------------------------------------------------------------------------------|
   * |                                        |    Pin       | 3  | 2  | 0  | 1  | 7  |    |
   * |    Leonardo, other 32u4-based          |--------------------------------------------|
   * |                                        | Interrupt No | 0  | 1  | 2  | 3  | 4  |    |
   * |--------------------------------------------------------------------------------------
   */

  /**!    The Correspondence Table of micro:bit Interrupt Pins And Terminal Numbers
   * ---------------------------------------------------------------------------------------------------------------------------------------------
   * |             micro:bit                       | DigitalPin |P0-P20 can be used as an external interrupt                                     |
   * |  (When using as an external interrupt,      |---------------------------------------------------------------------------------------------|
   * |no need to set it to input mode with pinMode)|Interrupt No|Interrupt number is a pin digital value, such as P0 interrupt number 0, P1 is 1 |
   * |-------------------------------------------------------------------------------------------------------------------------------------------|
   */
  /**!
       Select according to the set DADY pin polarity
      INPUT_PULLUP    // Low polarity, set pin 2 to pull-up input
   */
  pinMode(/*Pin */2 ,INPUT_PULLUP);

  /**!
    Set the pin to interrupt mode
    // Open the external interrupt 0, connect INT1/2 to the digital pin of the main control:
      function
        callback function
      state
        LOW            // When the pin is at low level, the interrupt occur, enter interrupt function
  */
  attachInterrupt(/*Interrupt No*/0, /*function*/myInterrupt ,/*state*/FALLING );
#endif
}

void loop()
{
  /**!
    When the interrupt occur in DRDY IO, get the geomagnetic data (get the data ready status through hardware)
    Enable interrupt again
  */  
  if(interruptFlag == 1){
    uint16_t gestures = gr10_30.getGesturesState();
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
    interruptFlag = 0;
    #if defined(ESP32) || defined(ESP8266)
      attachInterrupt(13, myInterrupt, FALLING);
    #elif defined(ARDUINO_SAM_ZERO)
      attachInterrupt(13, myInterrupt, FALLING);
    #else
      attachInterrupt(0, myInterrupt, FALLING);
    #endif
  }
  delay(1);
}