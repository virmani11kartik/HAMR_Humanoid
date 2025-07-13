#include <Arduino.h>
#include <esp32-hal.h>
#include <esp32-hal-gpio.h>
#include <esp32-hal-ledc.h>
// Motor Driver Pins
const int pwmPin = 12;   // Motor speed (PWM)
const int dirPin = 11;   // Motor direction

// Encoder Pins
const int encoderPinA = 14;
const int encoderPinB = 13;

volatile int encoderPos = 0;

void IRAM_ATTR handleEncoderA() {
  bool A = digitalRead(encoderPinA);
  bool B = digitalRead(encoderPinB);
  if (A == B) {
    encoderPos++;
  } else {
    encoderPos--;
  }
}

void setup() {
  Serial.begin(115200);

  // Motor driver setup
  pinMode(pwmPin, OUTPUT);
  pinMode(dirPin, OUTPUT);

  // Encoder setup
  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderPinA), handleEncoderA, CHANGE);

  // Set direction and PWM
  digitalWrite(dirPin, HIGH); // HIGH or LOW sets motor direction
  ledcAttachPin(pwmPin, 0);   // Attach pin to PWM channel 0
  ledcSetup(0, 1000, 8);      // Channel 0, 1kHz, 8-bit resolution
  ledcWrite(0, 128);          // 0-255: motor speed (128 = ~50%)
}

void loop() {
  static int lastPos = 0;
  if (lastPos != encoderPos) {
    Serial.print("Encoder Position: ");
    Serial.println(encoderPos);
    lastPos = encoderPos;
  }
  delay(10);
}
