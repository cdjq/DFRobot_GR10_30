/*!
 * @file  softGetData.ino
 * @brief Run the routine and you can use interrupt to read gesture
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
DFRobot_GR10_30 gr10_30(/*addr = */, /*pWire = */&Wire);
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
 * Set the detection window you want, only data collected in the range are valid
 * The largest window is 30, the configured number represents distance from the ceter to top & bottom or left & right
 * For example, if the configured distance from top to bottom is 30, then the distance from center to top is 15, and distance from center to bottom is also 15
 * udSize Distance from top to bottom      distance range 1-30
 * lrSize Distance from left to right      distance range 1-30
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
 * 1 - 200  10ms-2s  default is 60 600ms
 */
//   gr10_30.setHovrTimer(60);


/**
 * Set rotation angle that can trigger the gesture
 * count Default 16, range 1-31
 * count Rotation angle = 22.5 * count
 * count = 16, 22.5*count = 360  Rotate 360° to trigger the gesture
 */
//   gr10_30.setCwsAngle(/*count*/16);
//   gr10_30.setCcwAngle(/*count*/16);

/**
 * Set how many degrees your hand should rotate clockwise/cunterclockwise to trigger the gesture
 * count Default 4, range 1-31
 * count Continuous rotation angle = 22.5 * count
 * For example: count = 4, 22.5*count = 90
 * Trigger the clockwise/counterclockwise rotation gesture first, if keep rotating, then the continuous rotation gesture will be triggered once every 90 degrees
 */
//   gr10_30.setCwsAngleCount(/*count*/4);
//   gr10_30.setCcwAngleCount(/*count*/4);



// Connect the interrupt pin of the development board
#if defined(ESP32) || defined(ESP8266)
  /**!
    Select according to the set DADY pin polarity
      INPUT_PULLUP    // Low polarity, set pin 13 to pull-up input
      INPUT_PULLDOWN  // High polarity, set pin 13 to pull-down input
    interput io
      All pins can be used. Pin 13 is recommended
  */
  pinMode(/*Pin */13 ,INPUT_PULLUP);
  attachInterrupt(/*interput io*/13, myInterrupt, ONLOW);
#elif defined(ARDUINO_SAM_ZERO)
  pinMode(/*Pin */13 ,INPUT_PULLUP);
  attachInterrupt(/*interput io*/13, myInterrupt, FALLING);
#elif defined(ARDUINO_BBC_MICROBIT_V2) || defined(ARDUINO_BBC_MICROBIT)
  pinMode(/*Pin */0 ,INPUT_PULLUP);
  attachInterrupt(/*Interrupt No*/0, /*function*/myInterrupt ,/*state*/FALLING );
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
  attachInterrupt(/*Interrupt No*/0, /*function*/myInterrupt ,/*state*/LOW );
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
      attachInterrupt(13, myInterrupt, ONLOW);
    #elif defined(ARDUINO_SAM_ZERO)
      attachInterrupt(13, myInterrupt, FALLING);
    #elif defined(ARDUINO_BBC_MICROBIT_V2) || defined(ARDUINO_BBC_MICROBIT)
      attachInterrupt(0, myInterrupt, FALLING);
    #else
      attachInterrupt(0, myInterrupt, LOW);
    #endif
  }
  delay(1);
}
