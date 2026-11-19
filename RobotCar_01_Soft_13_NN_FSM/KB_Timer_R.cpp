#include <Arduino.h>
#include "KB_Timer_R.h"

// Constructor
Timer_R::Timer_R()
{
    // Set all values to zero
    Timer_R::clearTimer();
}

void Timer_R::clearTimer()
{
    // Set all values to zero
    this->is_paused = false;
    this->is_started = false;
    this->t_start = 0;
    this->t_stop = 0;
    this->t_pause = 0;
    this->t_resume = 0;
    this->pause_length = 0;
}


void Timer_R::startTimer()
{
    // if the timer is OFF
    if(!is_started)
    {
      // Change status and remember starting time
      is_started = true;
      t_start = millis();        
    }
    else
    {
      // Resume if it was on pause
      if(is_paused)
      {
        // update pause length
        t_resume = millis();
        pause_length = pause_length +  (t_resume - t_pause);
      }
    }
}

void Timer_R::stopTimer()
{
    // If the timer is started
    if(is_started)
    {
      // Change status and remember stop time
      t_stop = millis();  
      is_started = false;

      // If the timer was on pause
      if(is_paused)
      {
        // update pause length
        t_resume = t_stop;
        pause_length = pause_length +  (t_resume - t_pause);
      }      
    }
}

void Timer_R::pauseTimer()
{
    // If the timer is started
    if(is_started)
    {
      // remember pause time
      t_pause = millis();  
    }
}

void Timer_R::resumeTimer()
{
    // If the timer is started
    if(is_started)
    {
      // update pause length
      t_resume = millis(); 
      pause_length = pause_length +  (t_resume - t_pause);
    }
}


unsigned long Timer_R::getTotTime()
{
    if(is_started)
    {
      return millis() - t_start;
    }
    else
    {
      return t_stop - t_start;
    }
}

unsigned long Timer_R::getIdleTime()
{
  return pause_length;
}

unsigned long Timer_R::getWorkTime(bool roundup)
{
  if(roundup)
  {
    if((Timer_R::getTotTime() - Timer_R::getIdleTime()) == 0)
    {
      return 1;
    }
    else
    {
      return Timer_R::getTotTime() - Timer_R::getIdleTime();
    }    
  }
  else
  {
    return Timer_R::getTotTime() - Timer_R::getIdleTime();
  }
}
