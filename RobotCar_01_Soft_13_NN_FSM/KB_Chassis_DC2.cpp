#include "KB_Chassis_DC2.h"
#include <Arduino.h>


// Variables to store data
bool DC2_isMovingForward = false;   // Current status
int DC2_speedSet = 0;               // Current speed of DC motors (PWM duty cycle 0-255) Not used now

// Create DC motor and servo motor objects
AF_DCMotor DC2_motorL(DC2_CONN_MOTOR_L);
AF_DCMotor DC2_motorR(DC2_CONN_MOTOR_R);

// ------------------------------------------------------------------------------------------------------------
// FUNCTION DEFINITIONS
// ------------------------------------------------------------------------------------------------------------
void DC2_motorsSoftStart(int spdMin, int spdMax, int incrStep, int incrTime)
{
  /*
   * This function is to soft start dc motors
   * 
   * Arguments
   * - int spdMin - min PWM duty cycle to start with (0-255)
   * - int spdMax - max PWM duty cycle (0-255)
   * - int incrStep - duty cycle increment step
   * - int incrTime - pause between duty cycle increments
   * 
   */

  for(int spd = spdMin; spd<=spdMax; spd += incrStep)
  {
    DC2_motorL.setSpeed(spd);
    DC2_motorR.setSpeed(spd + DC2_SPD_OFFSET_R);
    DC2_speedSet = spd;
    delay(incrTime);
  }
}

// ------------------------------------------------------------------------------------------------------------
void DC2_moveA(int progL, int progR, int spdMin, int spdMax, int incrStep, int incrTime)
{
   /*
   * This function is to control motors independently, with arbitary speed and direction for each motor
   * 
   * Arguments
   * - int spdL - encoded PWM duty cycle for left motor, -255...255. Negative value - spin Backward. Positive value - spin Forward
   * - int spdR - encoded PWM duty cycle for right motor, -255...255. Negative value - spin Backward. Positive value - spin Forward
   * - int spdMin - min PWM duty cycle to start with (0-255)
   * - int spdMax - max PWM duty cycle (0-255)
   * - int incrStep - duty cycle increment step
   * - int incrTime - pause between duty cycle increments
   * 
   */

  // Define directions
  if(progL >= 0){DC2_motorL.run(FORWARD);}else{DC2_motorL.run(BACKWARD);}
  if(progR >= 0){DC2_motorR.run(FORWARD);}else{DC2_motorR.run(BACKWARD);}

  // take absolute values of input speed and trim
  progL = abs(progL);
  if(progL < spdMin){progL = 0;}
  if(progL > spdMax){progL = spdMax;}

  progR = abs(progR);
  if(progR < spdMin){progR = 0;}
  if(progR > spdMax){progR = spdMax;}

  // find which motor speed to be higher
  int spdHigh = max(progL, progR);

  // increase motor speeds to their values
  for(int spd = spdMin; spd<=spdHigh; spd += incrStep)
  {
    // update motors speed until it reaches necessary value
    if(spd < progL) DC2_motorL.setSpeed(spd);
    if(spd < progR) DC2_motorR.setSpeed(spd);    
  }
  
}

// ------------------------------------------------------------------------------------------------------------
//void DC2_moveF()
//{
// /* 
//   *  This function is to move robot car forvard
//   */
//
//  if(!DC2_isMovingForward)
//  {
//    DC2_isMovingForward = true;
//    DC2_motorL.run(FORWARD);
//    DC2_motorR.run(FORWARD);
//    DC2_motorsSoftStart(DC2_MIN_SPD_MV, DC2_MAX_SPD_MV, DC2_SPD_INCR_STEP_MV, DC2_SPD_INCR_TIME_MV);
//  }  
//}

// ------------------------------------------------------------------------------------------------------------
//void DC2_moveB()
//{
//   /* 
//   *  This function is to move robot car backvard
//   */
//
//  DC2_isMovingForward = false;  
//  DC2_motorL.run(BACKWARD);
//  DC2_motorR.run(BACKWARD);
//  DC2_motorsSoftStart(DC2_MIN_SPD_MV, DC2_MAX_SPD_MV, DC2_SPD_INCR_STEP_MV, DC2_SPD_INCR_TIME_MV);
//}

// ------------------------------------------------------------------------------------------------------------
void DC2_Stop()
{
   /* 
   *  This function is to stop robot car
   */

  DC2_isMovingForward = false;  
  DC2_motorL.run(RELEASE);
  DC2_motorR.run(RELEASE);
  DC2_speedSet = 0;
  delay(DC2_AFTERSTOP_TIME);
}

// ------------------------------------------------------------------------------------------------------------
//void DC2_turnL()
//{
//   /* 
//   *  This function is to turn robot car left
//   */  
//
//  DC2_isMovingForward = false; 
//  DC2_motorL.run(BACKWARD);
//  DC2_motorR.run(FORWARD);
//  DC2_motorsSoftStart(DC2_MIN_SPD_TRN, DC2_MAX_SPD_TRN, DC2_SPD_INCR_STEP_TRN, DC2_SPD_INCR_TIME_TRN);
//}

// ------------------------------------------------------------------------------------------------------------
//void DC2_turnR()
//{
//   /* 
//   *  This function is to turn robot car right
//   */  
//
//  DC2_isMovingForward = false;
//  
//  DC2_motorL.run(FORWARD);
//  DC2_motorR.run(BACKWARD);
//
//  DC2_motorsSoftStart(DC2_MIN_SPD_TRN, DC2_MAX_SPD_TRN, DC2_SPD_INCR_STEP_TRN, DC2_SPD_INCR_TIME_TRN);
//}

// ------------------------------------------------------------------------------------------------------------
bool DC2_getIsMovingFW()
{
  return DC2_isMovingForward;
}

// ------------------------------------------------------------------------------------------------------------
void DC2_setIsMovingFW(bool x)
{
  DC2_isMovingForward = x;
}
