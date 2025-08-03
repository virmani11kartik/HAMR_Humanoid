#include <Arduino.h>
#include <esp32-hal.h>
#include <esp32-hal-gpio.h>
#include <esp32-hal-ledc.h>

// Motor Pins
const int pwmL = 11;
const int dirL = 12;
const int encAL = 14;
const int encBL = 13;

const int pwmR = 41;
const int dirR = 42;
const int encAR = 1;
const int encBR = 2;

// Encoder counts
volatile long ticksL = 0;
volatile long ticksR = 0;

// PID constants - more conservative for power supply protection
const float Kp = 1.0;
const float Ki = 0.2;
const float Kd = 0.05;

// Power management
const int MAX_PWM = 150;  // Limit PWM to prevent power supply overload
const int MIN_PWM = 30;   // Minimum PWM to overcome static friction

// Encoder & motor specs
const int CPR = 64;        // Encoder counts per motor shaft rev
const int GEAR_RATIO = 150;
const int TICKS_PER_WHEEL_REV = CPR * GEAR_RATIO; // 9600

// Control interval
const unsigned long PID_INTERVAL = 100; // ms

// PID state
float integralL = 0, integralR = 0;
float lastErrL = 0, lastErrR = 0;
long lastTicksL = 0, lastTicksR = 0;
unsigned long lastPidTime = 0;

// Target speed in ticks/sec
float targetTicksL = 0;
float targetTicksR = 0;

// Control variables to be set manually:
char command = 'f';    // 'f' = forward, 'b' = backward, 'l' = left, 'r' = right, 's' = stop
int desiredRPM = 25;   // Start with lower RPM to test power supply

// Convert RPM to ticks per second
float rpmToTicksPerSec(float rpm) {
  return (rpm * TICKS_PER_WHEEL_REV) / 60.0;
}

// Convert ticks per second to output shaft RPM
float ticksPerSecToRPM(float ticksPerSec) {
  return (ticksPerSec * 60.0) / TICKS_PER_WHEEL_REV;
}

// FIXED: Interrupt service routines for encoders - now using both channels
void IRAM_ATTR handleEncL() {
  bool A = digitalRead(encAL);
  bool B = digitalRead(encBL);
  
  // Read previous state stored in static variables
  static bool lastA = false;
  static bool lastB = false;
  
  // Quadrature decoding
  if (A != lastA) {  // A changed
    if (A == B) {
      ticksL++;
    } else {
      ticksL--;
    }
  }
  if (B != lastB) {  // B changed
    if (A != B) {
      ticksL++;
    } else {
      ticksL--;
    }
  }
  
  lastA = A;
  lastB = B;
}

void IRAM_ATTR handleEncR() {
  bool A = digitalRead(encAR);
  bool B = digitalRead(encBR);
  
  static bool lastA = false;
  static bool lastB = false;
  
  if (A != lastA) {  // A changed
    if (A == B) {
      ticksR--;  // Reversed for right motor
    } else {
      ticksR++;
    }
  }
  if (B != lastB) {  // B changed
    if (A != B) {
      ticksR--;
    } else {
      ticksR++;
    }
  }
  
  lastA = A;
  lastB = B;
}

// FIXED: Set motor PWM and direction with power supply protection
void setMotor(int pwmPin, int dirPin, float pidOutput, int channel) {
  // Constrain PID output to prevent power supply overload
  pidOutput = constrain(pidOutput, -MAX_PWM, MAX_PWM);
  
  if (pidOutput >= 0) {
    digitalWrite(dirPin, HIGH);
    // Apply minimum PWM to overcome static friction
    int pwmValue = (pidOutput > 0) ? max((int)pidOutput, MIN_PWM) : 0;
    ledcWrite(channel, pwmValue);
  } else {
    digitalWrite(dirPin, LOW);
    // Apply minimum PWM to overcome static friction
    int pwmValue = (pidOutput < 0) ? max((int)(-pidOutput), MIN_PWM) : 0;
    ledcWrite(channel, pwmValue);
  }
}

void setup() {
  Serial.begin(115200);

  // Motor pins
  pinMode(pwmL, OUTPUT);
  pinMode(dirL, OUTPUT);
  pinMode(pwmR, OUTPUT);
  pinMode(dirR, OUTPUT);

  // Setup PWM channels
  ledcSetup(0, 1000, 8);
  ledcSetup(1, 1000, 8);
  ledcAttachPin(pwmL, 0);
  ledcAttachPin(pwmR, 1);

  // Encoder pins
  pinMode(encAL, INPUT_PULLUP);
  pinMode(encBL, INPUT_PULLUP);
  pinMode(encAR, INPUT_PULLUP);
  pinMode(encBR, INPUT_PULLUP);

  // FIXED: Attach interrupts to both encoder channels
  attachInterrupt(digitalPinToInterrupt(encAL), handleEncL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encBL), handleEncL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encAR), handleEncR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encBR), handleEncR, CHANGE);

  lastPidTime = millis();

  Serial.println("Starting control with fixed command and RPM in code.");
}

void loop() {
  // Set target speeds based on fixed command and desiredRPM
  float ticksPerSec = rpmToTicksPerSec(desiredRPM);

  switch (command) {
    case 'f': // Forward
      targetTicksL = ticksPerSec;
      targetTicksR = ticksPerSec;
      break;
    case 'b': // Backward
      targetTicksL = -ticksPerSec;
      targetTicksR = -ticksPerSec;
      break;
    case 'l': // Turn left (left wheel backward, right wheel forward)
      targetTicksL = -ticksPerSec * 0.6;
      targetTicksR = ticksPerSec * 0.6;
      break;
    case 'r': // Turn right (left wheel forward, right wheel backward)
      targetTicksL = ticksPerSec * 0.6;
      targetTicksR = -ticksPerSec * 0.6;
      break;
    case 's': // Stop
    default:
      targetTicksL = 0;
      targetTicksR = 0;
      break;
  }

  unsigned long now = millis();
  if (now - lastPidTime >= PID_INTERVAL) {
    float dt = (now - lastPidTime) / 1000.0;

    long currentTicksL = ticksL;
    long currentTicksR = ticksR;

    // Calculate speeds (ticks per second)
    float speedL = (currentTicksL - lastTicksL) / dt;
    float speedR = (currentTicksR - lastTicksR) / dt;

    // PID error
    float errL = targetTicksL - speedL;
    integralL += errL * dt;
    // Add integral windup protection
    integralL = constrain(integralL, -50, 50);
    float dErrL = (errL - lastErrL) / dt;
    float pwmLout = Kp * errL + Ki * integralL + Kd * dErrL;
    lastErrL = errL;

    float errR = targetTicksR - speedR;
    integralR += errR * dt;
    // Add integral windup protection
    integralR = constrain(integralR, -50, 50);
    float dErrR = (errR - lastErrR) / dt;
    float pwmRout = Kp * errR + Ki * integralR + Kd * dErrR;
    lastErrR = errR;

    // Output to motors (power limiting happens in setMotor function)
    setMotor(pwmL, dirL, pwmLout, 0);
    setMotor(pwmR, dirR, pwmRout, 1);

    // Update for next iteration
    lastTicksL = currentTicksL;
    lastTicksR = currentTicksR;
    lastPidTime = now;

    // Debug print: show RPM instead of raw ticks/s
    float rpmL = ticksPerSecToRPM(speedL);
    float rpmR = ticksPerSecToRPM(speedR);
    float rotL = currentTicksL / (float)TICKS_PER_WHEEL_REV;
    float rotR = currentTicksR / (float)TICKS_PER_WHEEL_REV;

    Serial.printf("L speed: %.1f RPM, R speed: %.1f RPM, PWM L: %.1f, PWM R: %.1f\n",
                  rpmL, rpmR, pwmLout, pwmRout);
    Serial.printf("L rotations: %.2f, R rotations: %.2f\n", rotL, rotR);
  }
}