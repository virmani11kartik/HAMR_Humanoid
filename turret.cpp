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




// Motor Driver Pins
const int pwmPin = 7;   // PWM speed control
const int dirPin = 6;   // Direction control

// Encoder Pins
const int encoderPinA = 5;
const int encoderPinB = 4;

volatile long encoderTicks = 0;  // Total encoder ticks

// Gear and encoder constants
const int TICKS_PER_REV = 2704;
const float DEGREES_PER_TICK = 360.0 / TICKS_PER_REV;

const float motorGearTeeth = 41.0;
const float outputGearTeeth = 136.4; // Adjusted for 1.5:1 gear ratio
const float gearRatio = outputGearTeeth / motorGearTeeth;

// Target
const float inputAngle = 360.0;  // Desired output angle in degrees
const float targetAngle = inputAngle * gearRatio;  // Motor shaft angle to reach that output

// PID variables
float Kp = 12.0;
float Ki = 0.8;
float Kd = 0.8;

float prevError = 0;
float integral = 0;

unsigned long lastTime = 0;

// Limits
const int maxPWM = 4095;
const int minPWM = 800;  // Minimum speed to overcome static friction

void IRAM_ATTR handleEncoderA() {
  bool A = digitalRead(encoderPinA);
  bool B = digitalRead(encoderPinB);
  encoderTicks += (A == B) ? 1 : -1;
}

void setup() {
  Serial.begin(115200);

  pinMode(pwmPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderPinA), handleEncoderA, CHANGE);

  ledcAttachPin(pwmPin, 0);           // Channel 0
  ledcSetup(0, 5000, 12);             // 5kHz, 12-bit resolution

  encoderTicks = 0;
  lastTime = millis();
}

void loop() {
  float currentAngle = encoderTicks * DEGREES_PER_TICK;
  float error = targetAngle - currentAngle;

  // Stop if very close
  if (abs(error) < 1.0) {
    ledcWrite(0, 0);  // Stop motor
    Serial.println("Target reached!");
    Serial.print("Final angle: ");
    Serial.println(currentAngle);
    while (true); // Stop
  }

  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;  // Time in seconds

  if (dt == 0) dt = 0.001; // Avoid division by zero

  integral += error * dt;
  float derivative = (error - prevError) / dt;

  float output = Kp * error + Ki * integral + Kd * derivative;

  // Set direction
  if (output > 0) {
    digitalWrite(dirPin, HIGH);
  } else {
    digitalWrite(dirPin, LOW);
    output = -output;  // Make PWM value positive
  }

  // Constrain PWM
  int pwmValue = constrain((int)output, minPWM, maxPWM);
  ledcWrite(0, pwmValue);

  // Debug
  Serial.print("Angle: ");
  Serial.print(currentAngle);
  Serial.print("  Error: ");
  Serial.print(error);
  Serial.print("  PWM: ");
  Serial.println(pwmValue);

  prevError = error;
  lastTime = now;
  delay(10);
}