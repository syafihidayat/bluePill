#include <Arduino.h>
#include <Wire.h>
#include "config.h"
#include "pid.h"
#include "Adafruit_VL53L0X.h"
#include <IRremote.hpp>

PID arm1(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID arm2(PWM_MIN, PWM_MAX, K_P, K_I, K_D);

volatile long pos[3];

template <int j>
void readEncoder();
void setMotor(int cwPin, int ccwPin, float pwmVal);
bool move(int index, uint8_t limit, float pwm);
void put(PID *index_enc, float angle, float pwm);
void limitMotor(int target);

unsigned long pos_prevT = 0;
unsigned long prevT = 0;
int state = 0;
unsigned long homingDoneTime = 0;
int slide_target = 0;
bool slide_homed = false;
bool slide_active = false;

TwoWire MyWire(PB9, PB8); // SDA, SCL
Adafruit_VL53L0X lox;

int last_dist = -1;
bool prevLimit1 = true, prevLimit2 = true, prevLimit3 = true, prevLimit4 = true; // idle HIGH (pullup)
bool prevProxy1L = true, prevProxy1R = true, prevProxy2L = true, prevProxy2R = true;

inline void sendState(uint8_t id, bool value)
{
  uint8_t data = (id << 1) | (value ? 1 : 0);
  Serial1.write(0xAA); // sync byte 1
  Serial1.write(0x55); // sync byte 2
  Serial1.write(data); // isi event (id + value)
}

inline void sendTOF(uint8_t status, uint16_t dist_mm)
{
  Serial1.write(0xAA);
  Serial1.write(0x55);
  Serial1.write(0xFE); // ID khusus TOF (sebelumnya 0x08, collide dgn sendState id=4,val=0)
  Serial1.write(status);
  Serial1.write((uint8_t)(dist_mm >> 8));   // MSB
  Serial1.write((uint8_t)(dist_mm & 0xFF)); // LSB
}

enum TeensyRxState
{
  TX_WAIT_SYNC1,
  TX_WAIT_SYNC2,
  TX_WAIT_CMD
};
TeensyRxState teensyRxState = TX_WAIT_SYNC1;

void readTeensySerial()
{
  while (Serial1.available())
  {
    uint8_t b = Serial1.read();
    switch (teensyRxState)
    {
    case TX_WAIT_SYNC1:
      if (b == 0xBB)
        teensyRxState = TX_WAIT_SYNC2;
      break;
    case TX_WAIT_SYNC2:
      if (b == 0x44)
        teensyRxState = TX_WAIT_CMD;
      else if (b != 0xBB)
        teensyRxState = TX_WAIT_SYNC1;
      break;
    case TX_WAIT_CMD:
      if (b == 0x01)
      {
        slide_active = true;
        slide_target = 0;
      } // ke LIMIT3
      if (b == 0x02)
      {
        slide_active = true;
        slide_target = 1;
      } // ke LIMIT4
      if (b == 0x03)
        digitalWrite(solenoidHolder, HIGH); // grip ON, disuruh Teensy
      if (b == 0x04)
        digitalWrite(solenoidHolder, LOW); // grip OFF, disuruh Teensy
      teensyRxState = TX_WAIT_SYNC1;
      break;
    }
  }
}

void setup()
{
  Serial.begin(9600);
  Serial1.setTx(PB6);
  Serial1.setRx(PB7);
  Serial1.begin(115200);

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LIMIT1, INPUT_PULLUP);
  pinMode(LIMIT2, INPUT_PULLUP);
  pinMode(LIMIT3, INPUT_PULLUP);
  pinMode(LIMIT4, INPUT_PULLUP);
  pinMode(PROXY1_LEFT, INPUT); // atau INPUT, lihat catatan di bawah
  pinMode(PROXY1_RIGHT, INPUT);
  pinMode(PROXY2_LEFT, INPUT);
  pinMode(PROXY2_RIGHT, INPUT);

  sendState(0, digitalRead(LIMIT1));
  sendState(1, digitalRead(LIMIT2));
  sendState(2, digitalRead(LIMIT3));
  sendState(3, digitalRead(LIMIT4));
  sendState(4, digitalRead(PROXY1_LEFT));
  sendState(5, digitalRead(PROXY1_RIGHT));
  sendState(6, digitalRead(PROXY2_LEFT));
  sendState(7, digitalRead(PROXY2_RIGHT));

  // analogWriteFrequency(PWM_FREQUENCY);

  for (int i = 0; i < 4; i++)
  {
    pinMode(cw[i], OUTPUT);
    pinMode(ccw[i], OUTPUT);

    // analogWriteResolution(PWM_BITS);
    // analogWrite(cw[i], 0);
    // analogWrite(ccw[i], 0);

    pinMode(enca[i], INPUT);
    pinMode(encb[i], INPUT);
  }

  pinMode(solenoidHolder, OUTPUT);
  pinMode(solenoidGrip, OUTPUT);
  pinMode(solenoidExtend, OUTPUT);

  digitalWrite(solenoidHolder, HIGH);
  digitalWrite(solenoidGrip, LOW);
  digitalWrite(solenoidExtend, LOW);

  MyWire.begin();
  IrReceiver.begin(IR_PIN, ENABLE_LED_FEEDBACK);

  slide_homed = false;
  while (digitalRead(LIMIT3) == HIGH)
  {
    setMotor(cw[1], ccw[1], 100);
  }

  setMotor(cw[1], ccw[1], 0);
  slide_homed = true;
  slide_target = 0;
  // homingDoneTime = millis();
  sendState(8, true);

  if (!lox.begin(0x29, false, &MyWire))
  {
    Serial.println("VL53L0X GAGAL");
    while (1)
      ;
  }

  Serial.println("VL53L0X READY");

  arm1.ppr_total(COUNTS_PER_REV1);
  arm2.ppr_total(COUNTS_PER_REV2);
}

void loop()
{

  readTeensySerial();
  if (slide_active)
    limitMotor(slide_target);

  // digitalWrite(solenoidHolder, HIGH);
  // digitalWrite(solenoidGrip, HIGH);
  // digitalWrite(solenoidExtend, HIGH);
  // digitalWrite(solenoidHolder, HIGH);
  // Serial.println("Relay ON");
  // delay(500);

  // Matikan relay
  // digitalWrite(solenoidHolder, LOW);
  // digitalWrite(solenoidGrip, LOW);
  // digitalWrite(solenoidExtend, LOW);
  // Serial.println("Relay OFF");
  // delay(500);

  VL53L0X_RangingMeasurementData_t measure;

  lox.rangingTest(&measure, false);
  bool limit1 = digitalRead(LIMIT1);
  bool limit2 = digitalRead(LIMIT2);
  bool limit3 = digitalRead(LIMIT3);
  bool limit4 = digitalRead(LIMIT4);
  bool proxy1L = digitalRead(PROXY1_LEFT);
  bool proxy1R = digitalRead(PROXY1_RIGHT);
  bool proxy2L = digitalRead(PROXY2_LEFT);
  bool proxy2R = digitalRead(PROXY2_RIGHT);

  if (limit1 != prevLimit1)
    sendState(0, limit1);
  if (limit2 != prevLimit2)
    sendState(1, limit2);
  if (limit3 != prevLimit3)
    sendState(2, limit3);
  if (limit4 != prevLimit4)
    sendState(3, limit4);
  if (proxy1L != prevProxy1L)
    sendState(4, proxy1L);
  if (proxy1R != prevProxy1R)
    sendState(5, proxy1R);
  if (proxy2L != prevProxy2L)
    sendState(6, proxy2L);
  if (proxy2R != prevProxy2R)
    sendState(7, proxy2R);

  prevLimit1 = limit1;
  prevLimit2 = limit2;
  prevLimit3 = limit3;
  prevLimit4 = limit4;
  prevProxy1L = proxy1L;
  prevProxy1R = proxy1R;
  prevProxy2L = proxy2L;
  prevProxy2R = proxy2R;

  // delay(10);

  // Data valid
  if (measure.RangeStatus == 0)
  {
    int dist = measure.RangeMilliMeter;

    // Filter loncatan data
    if (last_dist != -1 &&
        abs(dist - last_dist) > TOF_JUMP_MAX)
    {
      Serial.print("JUMP DIABAIKAN: ");
      Serial.println(dist);
      delay(50);
      return;
    }

    last_dist = dist;

    // Dalam range yang diinginkan
    if (dist >= TOF_MIN_DIST &&
        dist <= TOF_MAX_DIST)
    {
      Serial.print("OBJEK: ");
      Serial.print(dist);
      Serial.println(" mm");

      sendTOF(1, dist); // Kirim ke RX/TX
      Serial.print("send tof");
    }
    else
    {
      Serial.print("DI LUAR RANGE: ");
      Serial.print(dist);
      Serial.println(" mm");

      sendTOF(0, dist);
      Serial.print("send tof OUT OF RANGE");
    }
  }
  else
  {
    Serial.println("DATA TIDAK VALID");

    sendTOF(0, 0);
  }

  bool a = digitalRead(LIMIT1);
  bool b = digitalRead(LIMIT2);
  bool c = digitalRead(LIMIT3);
  bool d = digitalRead(LIMIT4);
  bool e = digitalRead(PROXY1_LEFT); //belakang
  bool f = digitalRead(PROXY1_RIGHT); //belakang  tengah
  bool g = digitalRead(PROXY2_LEFT); //depan tengah
  bool h = digitalRead(PROXY2_RIGHT);  //depan 

  Serial.print(a);
  Serial.print("  ");
  Serial.print(b);
  Serial.print("  ");
  Serial.print(c);
  Serial.print("  ");
  Serial.print(d);
  Serial.print("  ");
  Serial.print(e);
  Serial.print("  ");
  Serial.print(f);
  Serial.print("  ");
  Serial.print(g);
  Serial.print("  ");
  Serial.println(h);

  if (IrReceiver.decode())
  {
    uint32_t code = IrReceiver.decodedIRData.decodedRawData;

    if (code == 4111135500 || code == 0x05FA05FA)
    {
      sendState(67, 1);
      Serial.println("data receiver 1");
    }
    else if (code == 1604345760 || code == 0x30CF50AF)
    {
      sendState(68, 1);
      Serial.println("data receiver 2");
    }

    IrReceiver.resume();
  }
}

void limitMotor(int target)
{
  slide_target = target;

  bool limit3 = digitalRead(LIMIT3);
  bool limit4 = digitalRead(LIMIT4);

  if (slide_target == 0 && limit3 == LOW)
  {
    setMotor(cw[1], ccw[1], 0);
    slide_homed = true;
    slide_active = false;
    sendState(8, true);
    return;
  }
  else if (slide_target == 1 && limit4 == LOW)
  {
    setMotor(cw[1], ccw[1], 0);
    slide_homed = true;
    slide_active = false;
    sendState(8, true);
    return;
  }

  slide_homed = false;
  if (slide_target == 0)
    setMotor(cw[1], ccw[1], 100);
  else
    setMotor(cw[1], ccw[1], -100);
}

bool move(int index, uint8_t limit, float pwm)
{
  if (digitalRead(LIMIT1) == LOW)
  {
    setMotor(cw[index], ccw[index], 0);
    pos[index] = 0;
    return true;
  }
  else
  {
    setMotor(cw[index], ccw[index], 50);
    return false;
  }
}

void put(PID *index_enc, float angle, float pwm)
{
  unsigned long pos_currT = micros();
  float deltaT = ((float)(pos_currT - pos_prevT)) / 1.0e6;
  float pos_controlled = index_enc->control_count(angle, pos[6], pwm, deltaT);
  setMotor(cw[0], ccw[0], pos_controlled);

  pos_prevT = pos_currT;
}

void setMotor(int cwPin, int ccwPin, float pwmVal)
{
  if (pwmVal > 0)
  {
    analogWrite(cwPin, fabs(pwmVal));
    analogWrite(ccwPin, 0);
  }
  else if (pwmVal < 0)
  {
    analogWrite(cwPin, 0);
    analogWrite(ccwPin, fabs(pwmVal));
  }
  else
  {
    analogWrite(cwPin, 0);
    analogWrite(ccwPin, 0);
  }
}

template <int j>
void readEncoder()
{
  int b = digitalRead(encb[j]);
  if (b > 0)
  {
    pos[j]++;
  }
  else
  {
    pos[j]--;
  }
}