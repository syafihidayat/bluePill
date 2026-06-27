#include <Arduino.h>
#include "motor.h"

Motor::Motor(uint8_t pinCW, uint8_t pinCCW)
{
  CWpin = pinCW;
  CCWpin = pinCCW;
}

void Motor::begin()
{
  pinMode(CWpin, OUTPUT);
  pinMode(CCWpin, OUTPUT);
  analogWriteFrequency(25000);
}

void Motor::setMotor(int pwm)
{
  if(pwm > 0)
  {
    analogWrite(CWpin, pwm);
    analogWrite(CCWpin, 0);
  }
  else if(pwm < 0)
  {
    analogWrite(CWpin, 0);
    analogWrite(CCWpin, abs(pwm));
  }
  else
  {
    analogWrite(CWpin, 0);
    analogWrite(CCWpin, 0);
  }
}

void Motor::stopMotor()
{
  analogWrite(CWpin, 0);
  analogWrite(CCWpin, 0);
}