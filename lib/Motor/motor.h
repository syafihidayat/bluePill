#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include <cstdint>

class Motor {
  private:
   uint8_t CWpin;
   uint8_t CCWpin;
  public:
   Motor(uint8_t pinCW, uint8_t pinCCW);
   void begin();
   void setMotor(int pwm);
   void stopMotor();
};

#endif