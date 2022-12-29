/*!
 * @file  getGestures.ino
 * @brief Run the routine to read gesture type
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

//#define UARTMODE // UART mode
#define I2CMODE // I2C mode
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
/** Set the gesture to be enabled
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
 *  GESTURE_CLOCKWISE_C          Rotate clockwise continuously
 *  GESTURE_COUNTERCLOCKWISE_C   Rotate counterclockwise continuously
 */
  gr10_30.enGestures(GESTURE_UP|GESTURE_DOWN|GESTURE_LEFT|GESTURE_RIGHT|GESTURE_FORWARD|GESTURE_BACKWARD|GESTURE_CLOCKWISE|GESTURE_COUNTERCLOCKWISE|GESTURE_CLOCKWISE_C|GESTURE_COUNTERCLOCKWISE_C);

// Use detailed config if enabled; use the default config if not enabled
/**
 * Set the detection window, only data collected in this range are valid
 * The largest window is 30, the configured number represents distance from the ceter to top & bottom or left & right
 * For example, if the configured distance from top to bottom is 30, then the distance from center to top is 15, and distance from center to bottom is also 15
 * udSize Range of distance from top to bottom      1-30
 * lrSize Range of distance from left to right      1-30
 */
//   gr10_30.setUdlrWin(30, 30);
//   gr10_30.setHovrWin(20, 20);

/**
 * Set how far your hand should move left/right/up/down so the sensor can recognize it as gesture
 * Distance range 5-25, must be less than distances of the detection window
 */
//   gr10_30.setLeftRange(10);
//   gr10_30.setRightRange(10);
//   gr10_30.setUpRange(10);
//   gr10_30.setDownRange(10);
/**
 * Set how far your hand should move forward/backward so the sensor can recognize it as gesture
 * Distance range 1-15
 */
//   gr10_30.setForwardRange(10);
//   gr10_30.setBackwardRange(10);

/**
 * Set how many times you need to wave hands so the sensor can recognize it
 * Number range 1-15
 */
//   gr10_30.setWaveNumber(2);

/**
 * Set how long your hand should hover to trigger the gesture
 * 1 - 200  10ms-2s  Default is 60 600ms
 */
//   gr10_30.setHovrTimer(60);

/**
 * Set how many degrees your hand should rotate clockwise/cunterclockwise to trigger the gesture
 * count Default is 16 range 0-31
 * count Rotation angle is 22.5 * count
 * count = 16 22.5*count = 360  Rotate 360° to trigger the gesture
 */
//   gr10_30.setCwsAngle(/*count*/16);
//   gr10_30.setCcwAngle(/*count*/16);

/**
 * Set how many degrees your hand should rotate clockwise/cunterclockwise continuously to trigger the gesture
 * count Default is 4  range 0-31
 * count The degrees of continuous rotation is 22.5 * count
 * For example: count = 4 22.5*count = 90
 * Trigger the clockwise/counterclockwise rotation gesture first, if keep rotating, then the continuous rotation gesture will be triggered once every 90 degrees
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
