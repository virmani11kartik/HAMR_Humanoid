#include <Arduino.h>
#include <esp32-hal.h>
#include <esp32-hal-gpio.h>
#include <esp32-hal-ledc.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <WebServer.h>

// Motor Driver Pins
const int pwmPin = 7;   // PWM speed control
const int dirPin = 6;   // Direction control

// Encoder Pins
const int encoderPinA = 5;
const int encoderPinB = 4;

volatile long encoderTicks = 0;  // Total encoder ticks
float rotations = 0.0;
float rpm = 0.0;
float output_rpm = 0.0; // Output RPM after gear ratio
float output_rotations = 0.0; // Output RPM after gear ratio

const float Angle=-90.0; // Target angle in degrees
const int motorSpeed = 4000; // Motor speed in PWM value (0-4095)

const int TICKS_PER_REV = 2704; // PPR 13 PPR × 2 (quadrature) × 104 (gear ratio) = 2704 ticks/rev at output
const float DEGREES_PER_TICK = 360.0 / TICKS_PER_REV; // Degrees per tick

const float motorGearTeeth = 41.0; // Motor gear teeth    
const float outputGearTeeth = 130.0; // Output gear teeth
const float gearRatio = outputGearTeeth / motorGearTeeth; // Gear ratio
const float targetAngle= Angle * gearRatio; // Target angle in degrees

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
  ledcAttachPin(pwmPin, 0);           // PWM channel 0
  ledcSetup(0, 5000, 12);              // 1kHz PWM, 8-bit

  // Encoder setup
  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderPinA), handleEncoderA, CHANGE);

  encoderTicks = 0; // Reset encoder ticks

  //set direction based on target angle
  if(targetAngle > 0) {
    digitalWrite(dirPin, HIGH); // Set direction forward
  } else {
    digitalWrite(dirPin, LOW); // Set direction backward
  }

  ledcWrite(0, motorSpeed); // Set motor speed

}

void loop() {
  // static long lastTicks = 0;
  // static unsigned long lastTime = millis();
  // unsigned long currentTime = millis();
  // unsigned long elapsedTime = currentTime - lastTime;
  // long deltaTicks = encoderTicks - lastTicks;

  // if (deltaTicks > 100) {
  //   rotations = (float)encoderTicks / TICKS_PER_REV;
  //   rpm = (deltaTicks / (float)TICKS_PER_REV) * (60000.0 / elapsedTime); // RPM calculation
  //   // convert to output RPM
  //   output_rpm = rpm / gearRatio;
  //   output_rotations = rotations / gearRatio;
  //   // Serial.print("Ticks: ");
  //   // Serial.print(encoderTicks);
  //   Serial.print("  Rotations: ");
  //   Serial.print(rotations, 3); // Print 3 decimal places
  //   Serial.print("  RPM: ");
  //   Serial.print(rpm, 2); // Print 2 decimal places
  //   Serial.print("  Output Rotations: ");
  //   Serial.print(output_rotations, 3); // Print 3 decimal places
  //   Serial.print("  Output RPM: ");
  //   Serial.println(output_rpm, 2); // Print 2 decimal places
  //   lastTime = currentTime;
  //   lastTicks = encoderTicks;
  float currentAngle = encoderTicks * DEGREES_PER_TICK;

  // Check if we've reached or passed the target angle
  if ((targetAngle > 0 && currentAngle >= targetAngle) ||
      (targetAngle < 0 && currentAngle <= targetAngle)) {
    ledcWrite(0, 0);  // Stop motor
    Serial.println("Target reached!");
    Serial.print("Final angle: ");
    Serial.println(currentAngle);
    while (true); // Stop execution
  }

  // Print status
  Serial.print("Ticks: ");
  Serial.print(encoderTicks);
  Serial.print("  Angle: ");
  Serial.println(currentAngle);
  delay(10);
}