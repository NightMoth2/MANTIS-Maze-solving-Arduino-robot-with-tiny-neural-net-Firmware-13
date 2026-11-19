/*
 * Library KB_Chassis_DC2_H
 * VERSION 3 - 2026-04-07
 *- Added Getter and Setter for DC2_isMovingForward
 *
 * Version 2 - 2026-03-28
 * - Added function void DC2_moveA(int progL, int progR, int spdMin, int spdMax, int incrStep, int incrTime)
 *
 * Version 1 - 2026-03-18
 * - Basic
 *
 * This library is to control two DC motor robot car chassis,
 * using Adafruit Arduino Motor Control Shiels (also known
 * as L293D Arduino Shield).
 * 
 * Depends on AFMotor.h Adafruit Arduino Motor Control Shiels library
 * [1] Adafruit industries, “L293D Quadruple H-Bridge Motor Shield For Arduino.” Handson Technology, 2015-06-07. 
 *     Accessed: Feb. 27, 2026. [Online]. Available: https://www.handsontec.com/dataspecs/module/L293D%20Motor%20Shield.pdf
 * [2] Rajguru Electronics, “L293D Based Arduino Motor Shield.” Rajguru Electronics. Accessed: Feb. 27, 2026. [Online]. 
 *     Available: https://5.imimg.com/data5/PX/UK/MY-1833510/l293d-based-arduino-motor-shield.pdf
 * [3] S. Prabhu, “Control DC, Stepper & Servo with L293D Motor Driver Shield & Arduino,” Last Minute Engineers. 
 *     Accessed: Feb. 27, 2026. [Online]. Available: https://lastminuteengineers.com/l293d-motor-driver-shield-arduino-tutorial/
 * [4] PCBSynk, “L293D Arduino: Dual Motor Driver Shield Guide,” PCBSynk. Accessed: Feb. 27, 2026. [Online]. Available: https://pcbsync.com/l293d-arduino/
 *
 */

#ifndef KB_Chassis_DC2_H
#define KB_Chassis_DC2_H

#include <AFMotor.h>    // Motor shield control library
#include <Arduino.h>


// ******************************
// *HARDWARE SETUP: MOTOR SHIELD*
// ******************************
#define DC2_CONN_MOTOR_L 4    // connector for left motor
#define DC2_CONN_MOTOR_R 2    // connector for right motor

// ************************
// *HARDWARE SETUP: MOTORS*
// ************************
#define DC2_AFTERSTOP_TIME 100      // mS, idle time after motors stop
#define DC2_SPD_OFFSET_R  0       // duty cycle offset for right motor in a case if it works not similar 0-255


// OTHER PARAMETERS
// ----------------
// DC motors control parameters: 
// Forward, Backward
#define DC2_MIN_SPD_MV 40           // min PWM duty cycle 0-255
#define DC2_MAX_SPD_MV 250          // max PWM duty cyvle 0-255
#define DC2_SPD_INCR_STEP_MV 10     // duty cycle step 
#define DC2_SPD_INCR_TIME_MV 2      // mS, time between duty cycle changes
// Turning
#define DC2_MIN_SPD_TRN 75          // min PWM duty cycle 0-255
#define DC2_MAX_SPD_TRN 250         // max PWM duty cyvle 0-255
#define DC2_SPD_INCR_STEP_TRN 10    // duty cycle step
#define DC2_SPD_INCR_TIME_TRN 2     // mS, time between duty cycle changes



// FUNCTION DECLARATIONS
// ---------------------
void DC2_motorsSoftStart(int spdMin, int spdMax, int incrStep, int incrTime); // slowly turn motors on
void DC2_moveA(int progL, int progR, int spdMin, int spdMax, int incrStep, int incrTime); // Arbitary move, (slow start)
//void DC2_moveF();
//void DC2_moveB();
//void DC2_turnL();
//void DC2_turnR();
void DC2_Stop();
bool DC2_getIsMovingFW();
void DC2_setIsMovingFW(bool x);



#endif
