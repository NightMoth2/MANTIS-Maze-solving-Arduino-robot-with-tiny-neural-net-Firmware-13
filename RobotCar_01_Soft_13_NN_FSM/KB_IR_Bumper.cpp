#include "KB_IR_Bumper.h"
#include <Arduino.h>

int IRB_bits = 0;

// ------------------------------------------------------------------------------------------------------------
// FUNCTION DEFINITIONS
// ------------------------------------------------------------------------------------------------------------
void IRB_setup()
{

   /*
   * Sets pin modes for sensors
   * Call it in Ardiono setup() function. 
   */
  pinMode(IRB_F_PIN, INPUT_PULLUP);
  pinMode(IRB_T_PIN, INPUT_PULLUP);
  
  #ifdef BAD_BOARD_1
    // Do nothing for A6, A7 pure analog pins
  #else 
    pinMode(IRB_L_PIN, INPUT_PULLUP);  
    pinMode(IRB_R_PIN, INPUT_PULLUP); 
  #endif
}

// ------------------------------------------------------------------------------------------------------------
int IRB_checkSensors()
{
   /* 
    *  This function is to check for obstacles in all IR sensors
    * 
    * Arguments:
    * 
    * Return:
    * - int irBits - int walue to represent obstacle detections from ultrasonic sensor
    * detection from each sensor sets corresponding bit
   */
   
  IRB_bits = 0;   

  // Polling IR semnsors

  // LEFT sensor
  #ifdef BAD_BOARD_1
    if(analogRead(IRB_L_PIN) < 330)
    {   
      IRB_bits = IRB_bits | IRB_L_MASK; 
    }
  #else  
    if(digitalRead(IRB_L_PIN) == LOW)
    {
      IRB_bits = IRB_bits | IRB_L_MASK; 
    }
  #endif
  

  // RIGHT sensor
  #ifdef BAD_BOARD_1 
    if(analogRead(IRB_R_PIN) < 330)
    {
      IRB_bits = IRB_bits | IRB_R_MASK; 
    }
  #else
    if(digitalRead(IRB_R_PIN) == LOW)
    {
      IRB_bits = IRB_bits | IRB_R_MASK; 
    }
  #endif

  
  // CENTER sensor
  if(digitalRead(IRB_F_PIN) == LOW)
  {
    IRB_bits = IRB_bits | IRB_F_MASK; 
  }

  // TOP sensor
  if(digitalRead(IRB_T_PIN) == LOW)
  {
    IRB_bits = IRB_bits | IRB_T_MASK; 
  }

  return IRB_bits;  
}

// ------------------------------------------------------------------------------------------------------------
bool IRB_quickCheckFront()
{
  if(digitalRead(IRB_F_PIN) == LOW)
  {
    return true; 
  }
  else
  {
    return false;
  }
}

int IRB_getBits()
{
  return IRB_bits;
}
