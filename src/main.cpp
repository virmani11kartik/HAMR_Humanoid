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

// // Motor Pins
// const int pwmL = 11;
// const int dirL = 12;
// const int encAL = 14;
// const int encBL = 13;

// const int pwmR = 41;
// const int dirR = 42;
// const int encAR = 1;
// const int encBR = 2;

// // Encoder counts
// volatile long ticksL = 0;
// volatile long ticksR = 0;

// // PID constants - More conservative for initial tuning
// const float Kp = 1.0;    // Reduced from 2.0
// const float Ki = 0.05;   // Reduced from 0.1
// const float Kd = 0.02;   // Reduced from 0.05

// // Encoder & motor specs
// const int CPR = 64;
// const int GEAR_RATIO = 150;
// const int TICKS_PER_WHEEL_REV = CPR * GEAR_RATIO; // 9600

// // Control interval
// const unsigned long PID_INTERVAL = 50; // Reduced from 100ms to 50ms for better response

// // PID state
// float integralL = 0, integralR = 0;
// float lastErrL = 0, lastErrR = 0;
// long lastTicksL = 0, lastTicksR = 0;
// unsigned long lastPidTime = 0;

// // Target speed in ticks/sec
// float targetTicksL = 0;
// float targetTicksR = 0;

// // Control variables
// char command = 'f';    // 'f' = forward, 'b' = backward, 'l' = left, 'r' = right, 's' = stop, 't' = test
// float desiredRPM = 50; // Changed to spec speed (67 RPM no-load at 12V)

// // Test mode variables
// bool testMode = false;
// unsigned long testStartTime = 0;
// const unsigned long TEST_DURATION = 3000; // 3 seconds

// // Convert RPM to ticks per second
// float rpmToTicksPerSec(float rpm) {
//   return (rpm * TICKS_PER_WHEEL_REV) / 60.0;
// }

// // Convert ticks per second to output shaft RPM
// float ticksPerSecToRPM(float ticksPerSec) {
//   return (ticksPerSec * 60.0) / TICKS_PER_WHEEL_REV;
// }

// // Encoder interrupts - Fixed right motor direction
// void IRAM_ATTR handleEncL() {
//   bool A = digitalRead(encAL);
//   bool B = digitalRead(encBL);
//   ticksL += (A == B) ? 1 : -1;
// }

// void IRAM_ATTR handleEncR() {
//   bool A = digitalRead(encAR);
//   bool B = digitalRead(encBR);
//   ticksR += (A == B) ? -1 : 1;  // Inverted to fix direction issue
// }

// // Set motor PWM and direction
// void setMotor(int pwmPin, int dirPin, float pwmVal, int channel) {
//   pwmVal = constrain(pwmVal, -255, 255);
//   if (pwmVal >= 0) {
//     digitalWrite(dirPin, HIGH);
//     ledcWrite(channel, (int)pwmVal);
//   } else {
//     digitalWrite(dirPin, LOW);
//     ledcWrite(channel, (int)(-pwmVal));
//   }
// }

// // Open loop test function
// void runOpenLoopTest() {
//   if (!testMode) {
//     testMode = true;
//     testStartTime = millis();
//     ticksL = 0;  // Reset encoder counts
//     ticksR = 0;
//     Serial.println("Starting open loop test (255 PWM for 3 seconds)...");
    
//     // Set full PWM (255 = 100% duty cycle = 12V)
//     setMotor(pwmL, dirL, 255, 0);
//     setMotor(pwmR, dirR, 255, 1);
//   }
  
//   // Check if test duration completed
//   if (millis() - testStartTime >= TEST_DURATION) {
//     // Stop motors
//     setMotor(pwmL, dirL, 0, 0);
//     setMotor(pwmR, dirR, 0, 1);
    
//     // Calculate RPM over test duration
//     float testDurationSec = TEST_DURATION / 1000.0;
//     float rpmL = ticksPerSecToRPM(ticksL / testDurationSec);
//     float rpmR = ticksPerSecToRPM(ticksR / testDurationSec);
    
//     Serial.printf("OPEN LOOP TEST RESULTS:\n");
//     Serial.printf("Left Motor:  %.1f RPM (Expected: ~67 RPM)\n", rpmL);
//     Serial.printf("Right Motor: %.1f RPM (Expected: ~67 RPM)\n", rpmR);
//     Serial.printf("Test completed. Switching to PID control.\n\n");
    
//     // Reset for PID control
//     testMode = false;
//     command = 'f';  // Switch to forward
//     ticksL = 0;
//     ticksR = 0;
//     lastTicksL = 0;
//     lastTicksR = 0;
//     integralL = 0;
//     integralR = 0;
//     lastErrL = 0;
//     lastErrR = 0;
//     lastPidTime = millis();
//   }
// }

// void setup() {
//   Serial.begin(115200);

//   // Motor pins
//   pinMode(pwmL, OUTPUT);
//   pinMode(dirL, OUTPUT);
//   pinMode(pwmR, OUTPUT);
//   pinMode(dirR, OUTPUT);

//   // Setup PWM channels - Higher frequency for smoother operation
//   ledcSetup(0, 5000, 8);  // Increased frequency from 1000Hz to 5000Hz
//   ledcSetup(1, 5000, 8);
//   ledcAttachPin(pwmL, 0);
//   ledcAttachPin(pwmR, 1);

//   // Encoder pins
//   pinMode(encAL, INPUT_PULLUP);
//   pinMode(encBL, INPUT_PULLUP);
//   pinMode(encAR, INPUT_PULLUP);
//   pinMode(encBR, INPUT_PULLUP);

//   // Interrupt setup
//   attachInterrupt(digitalPinToInterrupt(encAL), handleEncL, CHANGE);
//   attachInterrupt(digitalPinToInterrupt(encAR), handleEncR, CHANGE);

//   lastPidTime = millis();
  
//   Serial.println("Motor control ready");
//   Serial.println("Commands: 'f'=forward, 'b'=backward, 'l'=left, 'r'=right, 's'=stop, 't'=test");
//   Serial.printf("Target RPM: %.1f\n", desiredRPM);
//   Serial.println("Will run open loop test in 2 seconds...");
//   delay(2000);
  
//   // Start with open loop test
//   command = 't';
// }

// void loop() {
//   // Check for serial commands
//   if (Serial.available()) {
//     char newCommand = Serial.read();
//     if (newCommand == 'f' || newCommand == 'b' || newCommand == 'l' || 
//         newCommand == 'r' || newCommand == 's' || newCommand == 't') {
//       command = newCommand;
//       Serial.printf("Command changed to: %c\n", command);
      
//       // Reset PID state when changing commands
//       integralL = 0;
//       integralR = 0;
//       lastErrL = 0;
//       lastErrR = 0;
//     }
//   }
  
//   // Handle open loop test
//   if (command == 't') {
//     runOpenLoopTest();
//     return;
//   }
  
//   // Set target speeds based on command
//   float desiredSpeed = rpmToTicksPerSec(desiredRPM);
  
//   switch (command) {
//     case 'f': // Forward
//       targetTicksL = desiredSpeed;
//       targetTicksR = desiredSpeed;
//       break;
//     case 'b': // Backward
//       targetTicksL = -desiredSpeed;
//       targetTicksR = -desiredSpeed;
//       break;
//     case 'l': // Turn left (left wheel slower, right wheel faster)
//       targetTicksL = desiredSpeed * 0.3;
//       targetTicksR = desiredSpeed * 0.8;
//       break;
//     case 'r': // Turn right (left wheel faster, right wheel slower)
//       targetTicksL = desiredSpeed * 0.8;
//       targetTicksR = desiredSpeed * 0.3;
//       break;
//     case 's': // Stop
//     default:
//       targetTicksL = 0;
//       targetTicksR = 0;
//       break;
//   }

//   unsigned long now = millis();
//   if (now - lastPidTime >= PID_INTERVAL) {
//     float dt = (now - lastPidTime) / 1000.0;

//     long currentTicksL = ticksL;
//     long currentTicksR = ticksR;

//     // Calculate speeds (ticks per second)
//     float speedL = (currentTicksL - lastTicksL) / dt;
//     float speedR = (currentTicksR - lastTicksR) / dt;

//     // PID control for left motor
//     float errL = targetTicksL - speedL;
//     integralL += errL * dt;
//     integralL = constrain(integralL, -100, 100);  // Prevent integral windup
//     float dErrL = (errL - lastErrL) / dt;
//     float pwmLout = Kp * errL + Ki * integralL + Kd * dErrL;
//     pwmLout = constrain(pwmLout, -255, 255);  // Critical: constrain PWM output
//     lastErrL = errL;

//     // PID control for right motor
//     float errR = targetTicksR - speedR;
//     integralR += errR * dt;
//     integralR = constrain(integralR, -100, 100);  // Prevent integral windup
//     float dErrR = (errR - lastErrR) / dt;
//     float pwmRout = Kp * errR + Ki * integralR + Kd * dErrR;
//     pwmRout = constrain(pwmRout, -255, 255);  // Critical: constrain PWM output
//     lastErrR = errR;

//     // Output to motors
//     setMotor(pwmL, dirL, pwmLout, 0);
//     setMotor(pwmR, dirR, pwmRout, 1);

//     // Update for next iteration
//     lastTicksL = currentTicksL;
//     lastTicksR = currentTicksR;
//     lastPidTime = now;

//     // Enhanced output with target and actual speeds
//     float rpmL = ticksPerSecToRPM(speedL);
//     float rpmR = ticksPerSecToRPM(speedR);
//     float targetRpmL = ticksPerSecToRPM(targetTicksL);
//     float targetRpmR = ticksPerSecToRPM(targetTicksR);
//     float rotL = currentTicksL / (float)TICKS_PER_WHEEL_REV;
//     float rotR = currentTicksR / (float)TICKS_PER_WHEEL_REV;

//     Serial.printf("L: %.1f/%.1f RPM (%.2f rot) PWM:%.0f | R: %.1f/%.1f RPM (%.2f rot) PWM:%.0f\n", 
//                   rpmL, targetRpmL, rotL, pwmLout, rpmR, targetRpmR, rotR, pwmRout);
//   }
// }
// ==== END OF FILE ====