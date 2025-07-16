#include <Arduino.h>
#include <esp32-hal.h>
#include <esp32-hal-gpio.h>
#include <esp32-hal-ledc.h>

// --- LEFT MOTOR ---
const int pwmL = 11;
const int dirL = 12;
const int encAL = 14;
const int encBL = 13;

// --- RIGHT MOTOR ---
const int pwmR = 41;
const int dirR = 42;
const int encAR = 1;
const int encBR = 2;

// --- ENCODER STATE ---
volatile long ticksL = 0;
volatile long ticksR = 0;

// --- PID CONFIG ---
const float kp = 0.4;
const float ki = 0.02;
const float kd = 0.01;

// --- TICKS PER REV ---
const int TICKS_PER_REV = 9600;  // 64 CPR × 150 gear ratio

// --- CONTROL TARGETS ---
int targetSpeedL = 300;  // ticks/sec
int targetSpeedR = 300;

// --- PID STATE ---
float integralL = 0, integralR = 0;
float lastErrL = 0, lastErrR = 0;
long lastTicksL = 0, lastTicksR = 0;
unsigned long lastTime = 0;

void IRAM_ATTR handleEncL() {
  bool A = digitalRead(encAL);
  bool B = digitalRead(encBL);
  ticksL += (A == B) ? 1 : -1;
}

void IRAM_ATTR handleEncR() {
  bool A = digitalRead(encAR);
  bool B = digitalRead(encBR);
  ticksR += (A == B) ? 1 : -1;
}

// Generic motor controller (handles forward/reverse)
void setMotor(int pwmPin, int dirPin, float pwmVal, int channel) {
  pwmVal = constrain(pwmVal, -255, 255);
  if (pwmVal >= 0) {
    digitalWrite(dirPin, HIGH);
    ledcWrite(channel, (int)pwmVal);
  } else {
    digitalWrite(dirPin, LOW);
    ledcWrite(channel, (int)(-pwmVal));
  }
}


void setup() {
  Serial.begin(115200);

  // Motor setup
  pinMode(pwmL, OUTPUT); pinMode(dirL, OUTPUT);
  pinMode(pwmR, OUTPUT); pinMode(dirR, OUTPUT);

  ledcAttachPin(pwmL, 0); ledcSetup(0, 1000, 8);
  ledcAttachPin(pwmR, 1); ledcSetup(1, 1000, 8);

  // Encoder setup
  pinMode(encAL, INPUT_PULLUP); pinMode(encBL, INPUT_PULLUP);
  pinMode(encAR, INPUT_PULLUP); pinMode(encBR, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(encAL), handleEncL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encAR), handleEncR, CHANGE);

  lastTime = millis();
}



// PID control function
void loop() {
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;

  if (dt >= 0.1) {  // Run PID every 100ms
    long currentTicksL = ticksL;
    long currentTicksR = ticksR;

    float speedL = (currentTicksL - lastTicksL) / dt;
    float speedR = (currentTicksR - lastTicksR) / dt;

    // --- LEFT PID ---
    float errL = targetSpeedL - speedL;
    integralL += errL * dt;
    float dErrL = (errL - lastErrL) / dt;
    float pwmLout = kp * errL + ki * integralL + kd * dErrL;
    lastErrL = errL;

    // --- RIGHT PID ---
    float errR = targetSpeedR - speedR;
    integralR += errR * dt;
    float dErrR = (errR - lastErrR) / dt;
    float pwmRout = kp * errR + ki * integralR + kd * dErrR;
    lastErrR = errR;

    // --- Motor control ---
    setMotor(pwmL, dirL, pwmLout, 0);
    setMotor(pwmR, dirR, pwmRout, 1);

    // Save state
    lastTicksL = currentTicksL;
    lastTicksR = currentTicksR;
    lastTime = now;

    // Debug
    Serial.printf("L: %ld (%.1f)  R: %ld (%.1f)\n", currentTicksL, speedL, currentTicksR, speedR);
  }
}