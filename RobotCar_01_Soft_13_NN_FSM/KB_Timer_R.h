/*
 * Class to implement timer for arduino with "on-pause" feature
 * 
 * Ver 2.0 - 2026-04-20
 * 
 * Supposed usage
 * ======================================================
 * Timer_R testTimer;
 * 
 * testTimer.clearTimer();      // clear all values and stop
 * testTimer.startTimer();      // Start timer
 * 
 * // some other process here
 * 
 * testTimer.pauseTimer();      // pause (pause-resume can be used many times)
 * 
 * // some other process here
 * 
 * testTimer.resumeTimer();     // OR testTimer.startTimer();
 * 
 * // some other process here
 * 
 *  * testTimer.pauseTimer();      // pause (pause-resume can be used many times)
 * 
 * // some other process here
 * 
 * testTimer.resumeTimer();     // OR testTimer.startTimer();
 * 
 * // some other process here
 * 
 * testTimer.stopTimer();       // stop timer
 * 
 * // Get information
 * unsigned long totTime = testTimer.getTotTime();        // time between start and stop including pauses
 * unsigned long idleTime = testTimer.getIdleTime();      // time the timer was on pause
 * unsigned long activeTime = testTimer.getWorkTime(0);   // timer active time (NOT including pause time)
 * 
 * ======================================================
 * 
 */


#ifndef KB_TIMER_R_H
#define KB_TIMER_R_H

#include <Arduino.h>



class Timer_R
{
  private: 

    bool is_paused;
    bool is_started;
    unsigned long t_start;
    unsigned long t_stop;
    unsigned long t_pause;
    unsigned long t_resume;
    unsigned long pause_length;

  public:

    // Constructor
    Timer_R();

    void startTimer();
    void clearTimer();
    void stopTimer();
    void pauseTimer();
    void resumeTimer();
    unsigned long getTotTime();
    unsigned long getIdleTime();
    unsigned long getWorkTime(bool roundup);


  
};



#endif
