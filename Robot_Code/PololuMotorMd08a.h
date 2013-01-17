/*
 Motor.h - Robot motor control
 
 This sketch is used to control a Pololu md08a Mototr controller from an arduino mega
 
 Authour: Alex Angell
 Date: October 2011
 Version: 1.0
 
 */
#ifndef PololuMotorMd08a_h
#define PololuMotorMd08a_h

#include "WProgram.h"

class Motor
{
  public:
    Motor();
    void dot();
    void dash();
  private:
    int _pin;
};


#endif

