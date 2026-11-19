/*
 * Library KB_IR_BUMPER_H
 * Version 2 2026-04-06
 * - byte encoded data saved into variable, getter added
 * Version 1 2026-03-18
 *  - basic
 * This library is to implement Arduino IR bumper sensors.
 * 
 * Refs:
 * [1] PCBSynk, “IR Proximity Sensor Arduino: Object Detection Projects and Complete Tutorial,” PCBSynk. 
 *     Accessed: Feb. 27, 2026. [Online]. Available: https://pcbsync.com/ir-proximity-sensor-arduino/
 * [2] Playwithcircuit, “Interfacing IR Sensor Module with Arduino,” Playwithcircuit. Accessed: Feb. 27, 2026.
 *     [Online]. Available: https://playwithcircuit.com/interfacing-ir-sensor-module-with-arduino/
 *
 */

#ifndef KB_IR_BUMPER_H
#define KB_IR_BUMPER_H

#include <Arduino.h>

#define BAD_BOARD_1     // !!! Bad Arduino board, cant use A2...A4 pins, forsed to use pure analog pins A6 and A7 and analogRead() function
#define IS_ARDUINO_NANO // !!! Can't use D13 pin (not isolated from LED circuity in Nano board)

// ******************************
// *HARDWARE SETUP: ARDUINO PINS*
// ******************************
// IR sensor pins
#define IRB_T_PIN 2          // top sensor

#ifdef IS_ARDUINO_NANO
  #define IRB_F_PIN A5       
#else
  #define IRB_F_PIN 13
#endif

#ifdef BAD_BOARD_1
  #define IRB_L_PIN A6   
  #define IRB_R_PIN A7
#else 
  #define IRB_L_PIN A2 
  #define IRB_R_PIN A3
#endif


// OTHER PARAMETERS
// ----------------
// Bit masks to encode obstacle detections from
// different directions. Each direction sets corresponding bit.
// Left | Right | Forward | Top
#define IRB_L_MASK 8 // 1000
#define IRB_R_MASK 4 // 0100
#define IRB_F_MASK 2 // 0010
#define IRB_T_MASK 1 // 0001


// FUNCTION DECLARATIONS
// ---------------------
void IRB_setup();           // Call it in Ardiono setup() function. Sets pin modes for sensors
int IRB_checkSensors();     // check for obstacles, all IR sectors
bool IRB_quickCheckFront();  // check for obstacles, all IR sectors
int IRB_getBits();


#endif
