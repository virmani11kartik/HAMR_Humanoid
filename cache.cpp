#include <Arduino.h>
#include <esp32-hal.h>
#include <esp32-hal-gpio.h>
#include <esp32-hal-ledc.h>
// Pin definitions
const int pwmPin = 14;    // PWM control pin
const int dirPin = 13;    // Direction control pin

void setup() {
  pinMode(pwmPin, OUTPUT);
  pinMode(dirPin, OUTPUT);

  // Start serial for debugging
  Serial.begin(115200);
  Serial.println("MD30C motor control test");
}

void loop() {
  // Example: spin forward at half speed
  digitalWrite(dirPin, HIGH);        // Set direction forward
  analogWrite(pwmPin, 128);          // PWM 50% duty cycle (0-255)

  Serial.println("Forward at 50%");
  delay(3000);

  // Stop motor
  analogWrite(pwmPin, 0);
  Serial.println("Motor stopped");
  delay(1000);

  // Spin backward at full speed
  digitalWrite(dirPin, LOW);         // Set direction backward
  analogWrite(pwmPin, 255);          // PWM 100% duty cycle

  Serial.println("Backward at 100%");
  delay(1000);

  // Stop motor again
  analogWrite(pwmPin, 0);
  Serial.println("Motor stopped");
  delay(2000);
}