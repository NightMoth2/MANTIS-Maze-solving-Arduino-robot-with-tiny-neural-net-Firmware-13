#include "KB_Sonar2.h"
#include <Arduino.h>

// ------------------------------------------------------------------------------------------------------------
// Constructor
// ------------------------------------------------------------------------------------------------------------
SNR::SNR(int trig_pin, int echo_pin, int servo_pin)
{
  this->trig_pin = trig_pin;
  this->echo_pin = echo_pin;
  this->servo_pin = servo_pin;

  this->distF = 0;
  this->distL = 0;
  this->distR = 0;
  this->bits = 0;

  this->distThresh = SNR_DIST_THRESHOLD;  // Set default value, can be changed in setup function
  
}

// ------------------------------------------------------------------------------------------------------------
// FUNCTION DEFINITIONS
// ------------------------------------------------------------------------------------------------------------
void SNR::setupSnr()
{
  /*
   * Sets pin modes for sensor and attaches servo
   * Call it in Ardiono setup() function. 
   */


  
  SNR_servo.attach(servo_pin);
  pinMode(trig_pin, OUTPUT);
  pinMode(echo_pin, INPUT); 
}

// ------------------------------------------------------------------------------------------------------------
double SNR::ping()
{
  /* This function is to perform one ping by ultrasonic sensor
   * 
   * Arguments:
   * 
   * Return:
   * - double dist_cm - measured distance in cm
   */
  
  
  // Trigger (several times)
  for(byte i = 0; i < 10; i++)
  {
      digitalWrite(trig_pin, HIGH);
      delayMicroseconds(SNR_TRIG_PULSE_WIDTH);
      digitalWrite(trig_pin, LOW);
      delayMicroseconds(SNR_TRIG_PULSE_WIDTH);
  }

  // Echo
  unsigned long duration_us = pulseIn(echo_pin, HIGH);

  delay(SNR_PAUSE_TIME); // pause between measurments to prevent trigger signal to the echo signal

  // double dist_cm = 0.017 * (double) duration_us;
  return 0.017 * (double) duration_us;
}

// ------------------------------------------------------------------------------------------------------------
double SNR::pingAvg()
{
  /* This function is to perform series of pings, 
   * discard outliers and average results
   * 
   * Arguments:
   * 
   * Return:
   * - double dist - averaged distance in cm
   */

  // Just in case clear array with records
  for(byte i = 0; i < SNR_RECORDS_COUNT; i++)
  {
    Records[i] = 0;    
  }

  // perform series of measurments and save results into the array
  for(byte i = 0; i < SNR_RECORDS_COUNT; i++)
  {
    Records[i] = SNR::ping();
  }

  double swap;

  // sort the array
  for(byte i = 0; i < SNR_RECORDS_COUNT - 1; i++)
  {
    for(byte j = i + 1; j < SNR_RECORDS_COUNT; j++)
    {
      if(Records[i] > Records[j])
      {
        swap = Records[i];
        Records[i] = Records[j];
        Records[j] = swap;
      }
    }
  }

  // ignore outliers and average data 
  double sum = 0;
  for(byte i = SNR_OUTLIERS_COUNT; i < SNR_RECORDS_COUNT - SNR_OUTLIERS_COUNT; i++)
  {
    sum += Records[i];  
  }
  
  //double dist = sum / (SNR_RECORDS_COUNT - 2*SNR_OUTLIERS_COUNT);
  return sum / (SNR_RECORDS_COUNT - 2*SNR_OUTLIERS_COUNT);
}

// ------------------------------------------------------------------------------------------------------------
double SNR::checkAngle(int ang)
{
  /* This function is to measure distance, 
   * in specified direction (angle)
   * 
   * Arguments:
   *  - int ang - angle to turn servo motor
   * Return:
   * - double dist - measured distance in cm
   */

  SNR::turnDir(ang);
  //SNR_servo.write(ang);
  //delay(SNR_SERVO_AFTER_STOP_TIME);
  // double dist = SNR_pingAvg();
  return SNR::pingAvg();
}

// ------------------------------------------------------------------------------------------------------------
double SNR::checkSector(int *arrAng, byte n)
{
  /* This function is to find minimal distance 
   * measured in specified sector (array of angles)
   * 
   * Arguments:
   *  - int *arrAng - array with angles ("Sector")
   *  - int n - size of this array
   * Return:
   * - double minDist - min distance detected in cm
   */

  double minDist = SNR::checkAngle(arrAng[0]);
  double newDist = 0;
  for(int i = 1; i < n; i++)
  {
    newDist = SNR::checkAngle(arrAng[i]);
    if(newDist < minDist)
    {
      minDist = newDist;
    }
  }

  return minDist;
}

// ------------------------------------------------------------------------------------------------------------
bool SNR::quickCheckSectorF()
{
  /* This function is to quick check for obstacle in forward sector
   * 
   * Arguments:
   * 
   * Return:
   * - int isDetected - "1" obstacle detected, "0" - not detected
   */

  bool isDetected = false;
  
  for(byte i = 0; i < SNR_SECTOR_F_ANG_COUNT; i++)
  {
    distF = SNR::checkAngle(SectorF[i]);

    if(distF <= distThresh)
    {
      // do not check next angle, obstacle is already in the sector      
      isDetected = true;
      break;   
    }
  }

  return isDetected; 
}

// ------------------------------------------------------------------------------------------------------------
bool SNR::quickCheckF()
{
    /* This function is to quick check for obstacle in forward direction
   * 
   * Arguments:
   * 
   * Return:
   * - int isDetected - "1" obstacle detected, "0" - not detected
   */

   bool isDetected = false;
   distF = SNR::checkAngle(SNR_ANG_F);
   if(distF <= distThresh)
   {
      isDetected = true;
   }

   return isDetected; 
  
}

// ------------------------------------------------------------------------------------------------------------
void SNR::forwardPos()
{
   /* This function is to Return sensor to forward position
   * 
   * Arguments:
   * 
   * Return:
   * 
   */
   SNR::turnDir(SNR_ANG_F);

   //delay(SNR_SERVO_AFTER_STOP_TIME);
}

// ------------------------------------------------------------------------------------------------------------
int SNR::fullScan()
{
   /* 
    *  This function is to measure distance and check for obstacles in all sectors
    * 
    * Arguments:
    * 
    * Return:
    * - int usBits - int walue to represent obstacle detections from ultrasonic sensor
    * detection from each sensor sets corresponding bit
   */
   
  bits = 0;  

  // Check left sector
  distL = SNR::checkSector(SectorL, SNR_SECTOR_L_ANG_COUNT);

  // Check forward sector
  distF = SNR::checkSector(SectorF, SNR_SECTOR_F_ANG_COUNT);

  // Check right sector  
  distR = SNR::checkSector(SectorR, SNR_SECTOR_R_ANG_COUNT);

  // Return sensor in forward position
  SNR::forwardPos();

  // Set corresponding bit if there is an obstacke in Left direction
  if(distL <= distThresh)
  {    
    bits = bits | SNR_L_MASK; 
  }

  // Set corresponding bit if there is an obstacke in Right direction
  if(distR <= distThresh)
  {
    bits = bits | SNR_R_MASK; 
  }

  // Set corresponding bit if there is an obstacke in Forward direction
  if(distF <= distThresh)
  {
    bits = bits | SNR_F_MASK;
  }

  return bits;
}

// ------------------------------------------------------------------------------------------------------------
double SNR::getDistF()
{
  return distF;  
}

// ------------------------------------------------------------------------------------------------------------
double SNR::getDistL()
{
  return distL;  
}

// ------------------------------------------------------------------------------------------------------------
double SNR::getDistR()
{
  return distR;  
}

// ------------------------------------------------------------------------------------------------------------
int SNR::getBits()
{
  return bits;
}
// ------------------------------------------------------------------------------------------------------------
void SNR::turnDir(int newAng)
{

  SNR_servo.write(newAng);
  delay(SNR_SERVO_AFTER_STOP_TIME);
/*   int currAng = SNR_servo.read();

   if(newAng == currAng)
   {
    // do nothing
   }
   else if(newAng > currAng)
   {
      for(int i = currAng+1; i <= newAng; i++)
      {
        SNR_servo.write(i); 
        delay(15);
      }
   }
   else if(newAng < currAng)
   {
      for(int i = currAng-1; i >= newAng; i--)
      {
        SNR_servo.write(i); 
        delay(15);
      }
    
   }
   */   
}

// ------------------------------------------------------------------------------------------------------------
void SNR::setThreshold(double dist)
{
  distThresh = dist;  
}
