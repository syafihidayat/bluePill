#include <Arduino.h>
#include <stdint.h>
#include <cstdint>
#include "encoder.h"

Encoder* Encoder::instances[12] = {nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr};

Encoder::Encoder(uint8_t pinA, uint8_t pinB)
{
  pina = pinA;
  pinb = pinB;
  counter = 0;
  counterDouble = 0;
  counterQuadrature = 0;
  lastEncoded = 0;
  last_time = 0;
  last_position = 0;
  current_speed = 0;
  current_rps = 0;
  count_mode = COUNT_SINGLE;
  initialized = false;
}

void Encoder::begin(int index)
{
  pinMode(pina, INPUT_PULLUP);
  pinMode(pinb, INPUT_PULLUP);

    instances[index] = this; 

    switch(index) {
      //untuk baca satu pin interrupt saja
      case 0: count_mode = COUNT_SINGLE; attachInterrupt(digitalPinToInterrupt(pina), count<0>, RISING); break;
      case 1: count_mode = COUNT_SINGLE; attachInterrupt(digitalPinToInterrupt(pina), count<1>, RISING); break;
      case 2: count_mode = COUNT_SINGLE; attachInterrupt(digitalPinToInterrupt(pina), count<2>, RISING); break;
      case 3: count_mode = COUNT_SINGLE; attachInterrupt(digitalPinToInterrupt(pina), count<3>, RISING); break;

      //untuk baca dua kali satu pin interrupt
      case 4: count_mode = COUNT_DOUBLE; attachInterrupt(digitalPinToInterrupt(pina), countDouble<4>, CHANGE); break;
      case 5: count_mode = COUNT_DOUBLE; attachInterrupt(digitalPinToInterrupt(pina), countDouble<5>, CHANGE); break;
      case 6: count_mode = COUNT_DOUBLE; attachInterrupt(digitalPinToInterrupt(pina), countDouble<6>, CHANGE); break;
      case 7: count_mode = COUNT_DOUBLE; attachInterrupt(digitalPinToInterrupt(pina), countDouble<7>, CHANGE); break;

      //untuk baca dua pin interrupt dua kali
      case 8: 
        count_mode = COUNT_QUADRATURE;
        attachInterrupt(digitalPinToInterrupt(pina), countQuadrature<8>, CHANGE); 
        attachInterrupt(digitalPinToInterrupt(pinb), countQuadrature<8>, CHANGE); 
        break;
      case 9: 
        count_mode = COUNT_QUADRATURE;
        attachInterrupt(digitalPinToInterrupt(pina), countQuadrature<9>, CHANGE); 
        attachInterrupt(digitalPinToInterrupt(pinb), countQuadrature<9>, CHANGE); 
        break;
      case 10: 
        count_mode = COUNT_QUADRATURE;
        attachInterrupt(digitalPinToInterrupt(pina), countQuadrature<10>, CHANGE); 
        attachInterrupt(digitalPinToInterrupt(pinb), countQuadrature<10>, CHANGE); 
        break;
      case 11: 
        count_mode = COUNT_QUADRATURE;
        attachInterrupt(digitalPinToInterrupt(pina), countQuadrature<11>, CHANGE); 
        attachInterrupt(digitalPinToInterrupt(pinb), countQuadrature<11>, CHANGE); 
        break;
    }

  last_time = micros();
  lastEncoded = (digitalRead(pina) << 1) | digitalRead(pinb);
  last_position = getActiveCount();
  initialized = true;
}

long Encoder::getActiveCount()
{
  switch (count_mode) {
    case COUNT_DOUBLE:
      return counterDouble;
    case COUNT_QUADRATURE:
      return counterQuadrature;
    case COUNT_SINGLE:
    default:
      return counter;
  }
}

void Encoder::counting()
{
  int LSB = digitalRead(pinb);

  if (LSB == HIGH) {
    counter++; 
  } else {
    counter--; 
  }
}

void Encoder::countingDouble()
{
  int MSB = digitalRead(pina);
  int LSB = digitalRead(pinb);

  if(LSB == MSB) {
    counterDouble--;
  } else {
    counterDouble++;
  }
}

void Encoder::countingQuadrature()
{
  int MSB = digitalRead(pina);
  int LSB = digitalRead(pinb);

  int encoded = (MSB << 1) | LSB; 
  int sum = (lastEncoded << 2) | encoded;

  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) counterQuadrature++;
  if(sum == 0b1110 || sum == 0b1000 || sum == 0b0001 || sum == 0b0111) counterQuadrature--;

  lastEncoded = encoded;
}

void Encoder::updateSpeed()  // Call this regularly, e.g., every 100ms
{
  if (!initialized) {
    return;  // Skip if not initialized
  }
  
  uint32_t current_time = micros();
  float current_position = (float)getActiveCount();
  
  // Calculate time difference
  float dt = (float)(current_time - this->last_time) / 1000000.0f;
  
  if(dt < MIN_DT){
    return;  // Don't update speed, keep previous value
  }
  
  // Calculate speed
  float delta_position = current_position - this->last_position;
  float speed_pps = delta_position / dt;
  float speed_rps = speed_pps / (encoder_resolution * gear_ratio);
  float speed_rpm = (speed_pps * 60.0f) / (encoder_resolution * gear_ratio);
  
  this->current_speed = speed_rpm;
  this->current_rps = speed_rps;
  this->last_position = current_position;
  this->last_time = current_time;
}

long Encoder::getCount() 
{
  return counter;
}

long Encoder::getCountDouble() 
{
  return counterDouble;
}

long Encoder::getCountQuadrature() 
{
  return counterQuadrature;
}

float Encoder::getCurrentSpeed() 
{
  return current_speed;
}

float Encoder::getCurrentRPS()
{
  return current_rps;
}