#include <Arduino.h>
#include <esp32-hal.h>
#include <esp32-hal-gpio.h>
#include <esp32-hal-ledc.h>

// Motor Driver Pins
const int pwmPin = 41;   // PWM speed control
const int dirPin = 42;   // Direction control

// Encoder Pins
const int encoderPinA = 1;
const int encoderPinB = 2;

volatile long encoderTicks = 0;  // Total encoder ticks
float rotations = 0.0;

const int TICKS_PER_REV = 3840; // PPR 13 PPR × 2 (quadrature) × 104 (gear ratio) = 2704 ticks/rev at output


void IRAM_ATTR handleEncoderA() {
  bool A = digitalRead(encoderPinA);
  bool B = digitalRead(encoderPinB);
  if (A == B) {
    encoderTicks++;
  } else {
    encoderTicks--;
  }
}

void setup() {
  Serial.begin(115200);

  // Motor setup
  pinMode(pwmPin, OUTPUT);
  pinMode(dirPin, OUTPUT);

  digitalWrite(dirPin, HIGH);         // Direction
  ledcAttachPin(pwmPin, 0);           // PWM channel 0
  ledcSetup(0, 1000, 8);              // 1kHz PWM, 8-bit
  ledcWrite(0, 128);                  // 50% duty cycle (adjust speed)

  // Encoder setup
  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderPinA), handleEncoderA, CHANGE);
}

void loop() {
  static long lastTicks = 0;

  if (lastTicks != encoderTicks) {
    rotations = (float)encoderTicks / TICKS_PER_REV;
    Serial.print("Ticks: ");
    Serial.print(encoderTicks);
    Serial.print("  Rotations: ");
    Serial.println(rotations, 3); // Print 3 decimal places
    lastTicks = encoderTicks;
  }

  delay(10); // To reduce Serial output spam
}

