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

// PID constants
const float Kp = 1.2;    // Slightly increased for better response
const float Ki = 0.08;   // Increased integral for better tracking
const float Kd = 0.03;   // Slightly increased derivative

// Encoder & motor specs
const int CPR = 64;
const int GEAR_RATIO = 150;
const int TICKS_PER_WHEEL_REV = CPR * GEAR_RATIO; // 9600

// Control interval
const unsigned long PID_INTERVAL = 50;

// PID state
float integralL = 0, integralR = 0;
float lastErrL = 0, lastErrR = 0;
long lastTicksL = 0, lastTicksR = 0;
unsigned long lastPidTime = 0;

// Target speed in ticks/sec
float targetTicks = 0;  // Single target for both wheels

// Control variables
char command = 'f';
float desiredRPM = 50;

// Motor synchronization
float actualSpeedL = 0;
float actualSpeedR = 0;
float maxDeviation = 5.0;  // Max allowed speed difference (RPM)

// Encoder interrupts
void IRAM_ATTR handleEncL() {
  bool A = digitalRead(encAL);
  bool B = digitalRead(encBL);
  ticksL += (A == B) ? 1 : -1;
}

void IRAM_ATTR handleEncR() {
  bool A = digitalRead(encAR);
  bool B = digitalRead(encBR);
  ticksR += (A == B) ? -1 : 1;  // Inverted to fix direction issue
}

// Set motor PWM and direction
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

// Convert RPM to ticks per second
float rpmToTicksPerSec(float rpm) {
  return (rpm * TICKS_PER_WHEEL_REV) / 60.0;
}

// Convert ticks per second to output shaft RPM
float ticksPerSecToRPM(float ticksPerSec) {
  return (ticksPerSec * 60.0) / TICKS_PER_WHEEL_REV;
}

void setup() {
  Serial.begin(115200);

  // Motor pins
  pinMode(pwmL, OUTPUT);
  pinMode(dirL, OUTPUT);
  pinMode(pwmR, OUTPUT);
  pinMode(dirR, OUTPUT);

  // Setup PWM channels
  ledcSetup(0, 5000, 8);
  ledcSetup(1, 5000, 8);
  ledcAttachPin(pwmL, 0);
  ledcAttachPin(pwmR, 1);

  // Encoder pins
  pinMode(encAL, INPUT_PULLUP);
  pinMode(encBL, INPUT_PULLUP);
  pinMode(encAR, INPUT_PULLUP);
  pinMode(encBR, INPUT_PULLUP);

  // Interrupt setup
  attachInterrupt(digitalPinToInterrupt(encAL), handleEncL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encAR), handleEncR, CHANGE);

  lastPidTime = millis();
  
  Serial.println("Coordinated Motor Control Ready");
  Serial.println("Commands: 'f'=forward, 'b'=backward, 's'=stop, '+'=inc RPM, '-'=dec RPM");
  Serial.printf("Initial Target RPM: %.1f\n", desiredRPM);
}

void loop() {
  // Handle serial commands
  if (Serial.available()) {
    char newCommand = Serial.read();
    
    if (newCommand == 'f' || newCommand == 'b' || newCommand == 's') {
      command = newCommand;
      Serial.printf("Command: %c\n", command);
      
      // Reset PID state when changing commands
      integralL = 0;
      integralR = 0;
      lastErrL = 0;
      lastErrR = 0;
    }
    else if (newCommand == '+') {
      desiredRPM += 5.0;
      Serial.printf("RPM+: %.1f\n", desiredRPM);
    }
    else if (newCommand == '-') {
      // Fixed: Use conditional instead of max() with mismatched types
      desiredRPM -= 5.0;
      if (desiredRPM < 0) desiredRPM = 0;
      Serial.printf("RPM-: %.1f\n", desiredRPM);
    }
  }

  // Set target speed based on command
  switch (command) {
    case 'f': // Forward
      targetTicks = rpmToTicksPerSec(desiredRPM);
      break;
    case 'b': // Backward
      targetTicks = -rpmToTicksPerSec(desiredRPM);
      break;
    case 's': // Stop
    default:
      targetTicks = 0;
      break;
  }

  // Run PID control at fixed interval
  unsigned long now = millis();
  if (now - lastPidTime >= PID_INTERVAL) {
    float dt = (now - lastPidTime) / 1000.0;

    // Get current encoder counts
    long currentTicksL = ticksL;
    long currentTicksR = ticksR;

    // Calculate actual speeds (ticks per second)
    actualSpeedL = (currentTicksL - lastTicksL) / dt;
    actualSpeedR = (currentTicksR - lastTicksR) / dt;

    // Calculate average actual speed
    float actualSpeedAvg = (actualSpeedL + actualSpeedR) / 2.0;

    // PID control for both motors using average speed as reference
    float err = targetTicks - actualSpeedAvg;
    
    // Left motor PID
    float errL = targetTicks - actualSpeedL;
    integralL += errL * dt;
    integralL = constrain(integralL, -100, 100);
    float dErrL = (errL - lastErrL) / dt;
    float pwmLout = Kp * errL + Ki * integralL + Kd * dErrL;
    pwmLout = constrain(pwmLout, -255, 255);
    lastErrL = errL;

    // Right motor PID
    float errR = targetTicks - actualSpeedR;
    integralR += errR * dt;
    integralR = constrain(integralR, -100, 100);
    float dErrR = (errR - lastErrR) / dt;
    float pwmRout = Kp * errR + Ki * integralR + Kd * dErrR;
    pwmRout = constrain(pwmRout, -255, 255);
    lastErrR = errR;

    // Synchronization logic: If one wheel is significantly slower, reduce both speeds
    float rpmL = ticksPerSecToRPM(actualSpeedL);
    float rpmR = ticksPerSecToRPM(actualSpeedR);
    float rpmDiff = abs(rpmL - rpmR);
    
    if (rpmDiff > maxDeviation) {
      // Slow down the faster motor to match the slower one
      if (rpmL > rpmR) {
        pwmLout = max(pwmLout - 20, pwmRout);
      } else {
        pwmRout = max(pwmRout - 20, pwmLout);
      }
      Serial.print("SYNC: ");
    }

    // Apply motor outputs
    setMotor(pwmL, dirL, pwmLout, 0);
    setMotor(pwmR, dirR, pwmRout, 1);

    // Update for next iteration
    lastTicksL = currentTicksL;
    lastTicksR = currentTicksR;
    lastPidTime = now;

    // Calculate rotations
    float rotL = currentTicksL / (float)TICKS_PER_WHEEL_REV;
    float rotR = currentTicksR / (float)TICKS_PER_WHEEL_REV;

    // Enhanced output
    Serial.printf("L: %5.1f RPM (%5.1f%%) | R: %5.1f RPM (%5.1f%%) | PWM: %4.0f,%4.0f | T: %5.1f RPM\n",
                  rpmL, (rpmL/desiredRPM)*100,
                  rpmR, (rpmR/desiredRPM)*100,
                  pwmLout, pwmRout,
                  ticksPerSecToRPM(targetTicks));
  }
}


// #include <Arduino.h>
// #include <esp32-hal.h>
// #include <esp32-hal-gpio.h>
// #include <esp32-hal-ledc.h>


// // ==== MOTOR & ENCODER PINS ====
// #define LEFT_PWM     11
// #define LEFT_DIR     12
// #define LEFT_ENC_A   14
// #define LEFT_ENC_B   13

// #define RIGHT_PWM    41
// #define RIGHT_DIR    42
// #define RIGHT_ENC_A  1
// #define RIGHT_ENC_B  2

// // ==== PID CONFIG ====
// float targetSpeed = 100.0; // ticks per second
// float Kp = 1.0, Ki = 0.2, Kd = 0.05;

// // ==== ENCODER STATE ====
// volatile long leftTicks = 0, rightTicks = 0;
// long prevLeftTicks = 0, prevRightTicks = 0;

// // PID state
// float leftIntegral = 0, rightIntegral = 0;
// float leftPrevError = 0, rightPrevError = 0;

// unsigned long prevTime = 0;

// // ==== ENCODER INTERRUPTS ====
// void IRAM_ATTR leftEncoderISR() {
//   leftTicks++;
// }

// void IRAM_ATTR rightEncoderISR() {
//   rightTicks++;
// }

// // ==== SETUP ====
// void setup() {
//   Serial.begin(115200);

//   pinMode(LEFT_PWM, OUTPUT);
//   pinMode(LEFT_DIR, OUTPUT);
//   pinMode(RIGHT_PWM, OUTPUT);
//   pinMode(RIGHT_DIR, OUTPUT);

//   pinMode(LEFT_ENC_A, INPUT_PULLUP);
//   pinMode(LEFT_ENC_B, INPUT_PULLUP);
//   pinMode(RIGHT_ENC_A, INPUT_PULLUP);
//   pinMode(RIGHT_ENC_B, INPUT_PULLUP);

//   attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), leftEncoderISR, RISING);
//   attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), rightEncoderISR, RISING);

//   prevTime = millis();
// }

// // ==== HELPER ====
// void setMotor(int pwmPin, int dirPin, int pwmVal, bool forward = true) {
//   digitalWrite(dirPin, forward ? HIGH : LOW);
//   ledcWrite(pwmPin, constrain(pwmVal, 0, 255));
// }

// // ==== MAIN LOOP ====
// void loop() {
//   unsigned long now = millis();
//   float dt = (now - prevTime) / 1000.0; // seconds

//   if (dt >= 0.1) { // every 100ms
//     long leftDelta = leftTicks - prevLeftTicks;
//     long rightDelta = rightTicks - prevRightTicks;

//     float leftSpeed = leftDelta / dt;   // ticks/sec
//     float rightSpeed = rightDelta / dt;

//     // === LEFT PID ===
//     float l_error = targetSpeed - leftSpeed;
//     leftIntegral += l_error * dt;
//     float l_derivative = (l_error - leftPrevError) / dt;
//     float l_output = Kp * l_error + Ki * leftIntegral + Kd * l_derivative;
//     leftPrevError = l_error;

//     // === RIGHT PID ===
//     float r_error = targetSpeed - rightSpeed;
//     rightIntegral += r_error * dt;
//     float r_derivative = (r_error - rightPrevError) / dt;
//     float r_output = Kp * r_error + Ki * rightIntegral + Kd * r_derivative;
//     rightPrevError = r_error;

//     // === MOTOR COMMANDS ===
//     setMotor(LEFT_PWM, LEFT_DIR, (int)l_output);
//     setMotor(RIGHT_PWM, RIGHT_DIR, (int)r_output);

//     // Debugging
//     Serial.printf("L: %.1f  R: %.1f  -> PWM L: %.1f  R: %.1f\n", leftSpeed, rightSpeed, l_output, r_output);

//     // Update for next cycle
//     prevLeftTicks = leftTicks;
//     prevRightTicks = rightTicks;
//     prevTime = now;
//   }
// }
// // ==== END OF FILE ====