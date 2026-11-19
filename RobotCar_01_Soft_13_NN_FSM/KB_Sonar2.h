/*
 * Library KB_SONAR_H
 * 
 * Version 2.1 - 2026-04-25
 * - setupSnr() now without arguments
 * 
 * Version 2.0 - 2026-04-08
 * - implemented as class
 *
 * Version 1.2 - 2026-04-06
 * - SNR_bits variable added, getter added
 *
 * Version 1.1 - 2026-03-29
 * - #define SNR_DIST_THRESHOLD used as default value, 
 *   actual value can be set using void SNR_setThreshold(double dist)
 * - void SNR_setThreshold(double dist) to set distance threshold added
 * - #define SNR_DIST_MAX added
 * - added function int SNR_quickCheckF() - check for obstacle in forward direction
 * - trying to use as less memory as possible (change int to byte where it possible)
 *
 * Version 1.0 - 2026-03-18
 * - Basic
 *
 * This library is to implement Arduino sonar
 * based on HC-SR04 or similar ultrasonic sensors,
 * mounted on small servo motor like MG90S or similar.
 * Depends on Servo.h standard arduino library.
 *
 * Refs
 *  [1] GeeksForGeeks, “Arduino - Ultrasonic Sensor,” GeeksForGeeks. Accessed: Feb. 27, 2026. [Online]. 
 *      Available: https://www.geeksforgeeks.org/electronics-engineering/arduino-ultrasonic-sensor/
 *  [2] Arduino - Get Started, “Arduino - ultrasonic sensor,” Arduino - Get Started. Accessed: Feb. 27, 2026. [Online]. 
 *      Available: https://arduinogetstarted.com/tutorials/arduino-ultrasonic-sensor
 *
 */


#ifndef KB_SONAR_H
#define KB_SONAR_H

#include <Arduino.h>
#include <Servo.h>      // Servo motor control library

// ******************************
// *HARDWARE SETUP: ARDUINO PINS*
// ******************************
// ultrasonic sensor Tig and Echo Pins 
#define SNR_TRIG_PIN A0
#define SNR_ECHO_PIN A1
// Servo PWM pin
#define SNR_SERVO_PIN 9

// ***********************************
// *HARDWARE SETUP: ULTRASONIC SENSOR*
// ***********************************
#define SNR_TRIG_PULSE_WIDTH 10   // uS, length of trigger pulse
#define SNR_PAUSE_TIME 40         // mS, pause between measurments to prevent trigger signal to the echo signal

// OTHER PARAMETERS
// ----------------
#define SNR_DIST_MIN  2.0           // cm, min dist accoding to datasheet
#define SNR_DIST_MAX  400.0         // cm, max dist accoding to datasheet
#define SNR_DIST_THRESHOLD 15.0     // cm, default value, it can be changed by if an object detected closer than that distance - report it as an obstacle

// Multiply pings, avaraging, clear noise
#define SNR_RECORDS_COUNT 4     // 1...255 Quantity of pings for same measurement. Size of an array with records from multiply pings.
#define SNR_OUTLIERS_COUNT 1    // Number of outliers to discard from BOTH min and max sides of an array with records.

// Servo motor control parameters
#define SNR_SERVO_AFTER_STOP_TIME 100 // mS, idle time after servo stops
// Some aliases for angles to rotate servo
#define SNR_ANG_F 90
#define SNR_ANG_L 170
#define SNR_ANG_R 10

// Quantity of angles in "Sectors"
// (Arrays of angles represent "Sector")
#define SNR_SECTOR_F_ANG_COUNT 3
#define SNR_SECTOR_L_ANG_COUNT 3
#define SNR_SECTOR_R_ANG_COUNT 3

// Bit masks to encode obstacle detections from
// different directions. Each direction sets corresponding bit.
// (Try to keep compatible with other sensors in syctem)
#define SNR_L_MASK 8 // 1000
#define SNR_R_MASK 4 // 0100
#define SNR_F_MASK 2 // 0010

class SNR
{
  private:

    // pins
    int trig_pin;
    int echo_pin;
    int servo_pin;

    Servo SNR_servo;  // Servo motor
    
    // "Sectors" (Arrays of angles represent "Sector")
    const int SectorF[SNR_SECTOR_F_ANG_COUNT] = {50, 90, 130};    // Forward sector
    const int SectorL[SNR_SECTOR_L_ANG_COUNT] = {180, 160, 140};  // Left sector
    const int SectorR[SNR_SECTOR_R_ANG_COUNT] = {40, 20, 0};      // Right sector

    double Records[SNR_RECORDS_COUNT]; // Array to store records for multiply measurments
    
    double distF;    // cm, measured distance in forward direction
    double distL;    // cm, measured distance in left direction
    double distR;    // cm, measured distance in right direction

    double distThresh;   // Set default value, can be changed using void SNR_setThreshold(double dist)

    int bits;

  public:

    // COnstructor
    SNR(int trig_pin, int echo_pin, int servo_pin);
    // setup
    void setupSnr();
    void setThreshold(double dist);
    // Getters
    double getDistF();
    double getDistL();
    double getDistR();
    int getBits();
    // moves
    void turnDir(int newAng);
    void forwardPos();
    //COre
    double ping();
    double pingAvg();
    double checkAngle(int ang);
    double checkSector(int *arrAng, byte n);
    int fullScan();
    bool quickCheckSectorF();
    bool quickCheckF(); 
    // Add
   
    

    
    

  
};

#endif
