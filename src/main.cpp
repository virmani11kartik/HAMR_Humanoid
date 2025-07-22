#include <Arduino.h>
#include <esp32-hal.h>
#include <esp32-hal-gpio.h>
#include <esp32-hal-ledc.h>


// void setup() {
//   pinMode(LED_BUILTIN, OUTPUT);
//   Serial.begin(115200);
//   Serial.println("Setup complete. LED will blink every second.");
// }

// void loop() {
//   digitalWrite(LED_BUILTIN, HIGH);
//   delay(1000);
//   digitalWrite(LED_BUILTIN, LOW);
//   delay(1000);
// }

// Motor Pins
const int pwmL = 11;
const int dirL = 12;
const int encAL = 14;
const int encBL = 13;

const int pwmR = 41;
const int dirR = 42;
const int encAR = 1;
const int encBR = 2;

// Encoder counts (volatile for ISR)
volatile long ticksL = 0;
volatile long ticksR = 0;

// PID constants for synchronization
const float Kp_sync = 5.0;
const float Ki_sync = 0.05;
const float Kd_sync = 1.0;

// Encoder & motor specs
const int CPR = 64;
const int GEAR_RATIO = 150;
const int TICKS_PER_WHEEL_REV = CPR * GEAR_RATIO; // 9600 ticks per wheel revolution

// Control interval (ms)
const unsigned long PID_INTERVAL = 50;

// PID state variables
float integralSync = 0;
float lastErrorSync = 0;

// Timing variables
unsigned long lastPidTime = 0;
long lastTicksL = 0;
long lastTicksR = 0;

// Base PWM speed (0-4095)
float basePWM = 4095;

// Joystick control variables
float ly = 0.0f;  // left stick vertical (forward/back)
float rx = 0.0f;  // right stick horizontal (turn)

// Current command
// char command = 'f'; // start forward

// Encoder interrupts (quadrature decoding)
void IRAM_ATTR handleEncL() {
  bool A = digitalRead(encAL);
  bool B = digitalRead(encBL);
  ticksL += (A == B) ? 1 : -1;
}

void IRAM_ATTR handleEncR() {
  bool A = digitalRead(encAR);
  bool B = digitalRead(encBR);
  ticksR += (A == B) ? -1 : 1;  // Inverted to fix direction issue if needed
}

// Set motor PWM and direction
// Set motor PWM and direction
void setMotor(int pwmPin, int dirPin, float pwmVal, int channel) {
  pwmVal = constrain(pwmVal, -4095, 4095);
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

  // Motor pins
  pinMode(pwmL, OUTPUT);
  pinMode(dirL, OUTPUT);
  pinMode(pwmR, OUTPUT);
  pinMode(dirR, OUTPUT);

  // Setup PWM channels at 5 kHz, 12-bit resolution
  ledcSetup(0, 5000, 12);
  ledcSetup(1, 5000, 12);
  ledcAttachPin(pwmL, 0);
  ledcAttachPin(pwmR, 1);

  // Encoder pins with pull-ups
  pinMode(encAL, INPUT_PULLUP);
  pinMode(encBL, INPUT_PULLUP);
  pinMode(encAR, INPUT_PULLUP);
  pinMode(encBR, INPUT_PULLUP);

  // Attach interrupts on channel A for both encoders
  attachInterrupt(digitalPinToInterrupt(encAL), handleEncL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encAR), handleEncR, CHANGE);

  lastPidTime = millis();

  Serial.println("Motor Sync Control Ready");
//   Serial.println("Commands: f=forward, b=backward, r=right, l=left, s=stop, +=faster, -=slower");
  Serial.println("Send joystick data like: LX:0.00 LY:0.00 RX:0.00 RY:0.00 LT:0.00 RT:0.00 A:0 B:0 X:0 Y:0");
  Serial.printf("Initial Speed PWM: %.0f\n", basePWM);
}

void loop() {
  // Handle serial commands
//   if (Serial.available()) {
//     char newCommand = Serial.read();
//     if (newCommand == 'f' || newCommand == 'b' || newCommand == 'r' || newCommand == 'l' || newCommand == 's') {
//       command = newCommand;
//       integralSync = 0;
//       lastErrorSync = 0;
//       Serial.printf("Command: %c\n", command);
//     } else if (newCommand == '+') {
//       basePWM += 10;
//       if (basePWM > 4095) basePWM = 4095;
//       Serial.printf("Speed increased: %d\n", (int)basePWM);
//     } else if (newCommand == '-') {
//       basePWM -= 10;
//       if (basePWM < 0) basePWM = 0;
//       Serial.printf("Speed decreased: %d\n", (int)basePWM);
//     }
//   }

    // Read joystick data from serial
    if (Serial.available()) {
    String msg = Serial.readStringUntil('\n');

    int lyIndex = msg.indexOf("LY:");
    int rxIndex = msg.indexOf("RX:");

    if (lyIndex != -1 && rxIndex != -1) {
      // Extract LY and RX values as floats
      // Find next space or end of line after LY:
      int lyEnd = msg.indexOf(' ', lyIndex);
      if (lyEnd == -1) lyEnd = msg.length();
      int rxEnd = msg.indexOf(' ', rxIndex);
      if (rxEnd == -1) rxEnd = msg.length();

      String lyStr = msg.substring(lyIndex + 3, lyEnd);
      String rxStr = msg.substring(rxIndex + 3, rxEnd);

      ly = lyStr.toFloat();
      rx = rxStr.toFloat();
        }
    }

  // PID loop timing
  unsigned long now = millis();
  if (now - lastPidTime >= PID_INTERVAL) {
    float dt = (now - lastPidTime) / 1000.0;

    // Read encoder counts atomically
    noInterrupts();
    long currentTicksL = ticksL;
    long currentTicksR = ticksR;
    interrupts();

    // Calculate RPM for each motor
    float rpmL = ((currentTicksL - lastTicksL) / dt) * 60.0 / TICKS_PER_WHEEL_REV;
    float rpmR = ((currentTicksR - lastTicksR) / dt) * 60.0 / TICKS_PER_WHEEL_REV;

    lastTicksL = currentTicksL;
    lastTicksR = currentTicksR;
    lastPidTime = now;

    // Calculate error between motors (left RPM minus right RPM)
    float errorSync = rpmL - rpmR;

    integralSync += errorSync * dt;
    integralSync = constrain(integralSync, -100, 100);
    float dErrorSync = (errorSync - lastErrorSync) / dt;

    float correctionPWM = Kp_sync * errorSync + Ki_sync * integralSync + Kd_sync * dErrorSync;
    lastErrorSync = errorSync;

    // Calculate base PWM values per motor based on command
    // float pwmL_base = 0, pwmR_base = 0;
    // switch (command) {
    //   case 'f': // forward both motors
    //     pwmL_base = basePWM;
    //     pwmR_base = basePWM;
    //     break;
    //   case 'b': // backward both motors
    //     pwmL_base = -basePWM;
    //     pwmR_base = -basePWM;
    //     break;
    //   case 'r': // right turn: left forward, right backward
    //     pwmL_base = basePWM;
    //     pwmR_base = -basePWM;
    //     break;
    //   case 'l': // left turn: left backward, right forward
    //     pwmL_base = -basePWM;
    //     pwmR_base = basePWM;
    //     break;
    //   case 's': // stop
    //   default:
    //     pwmL_base = 0;
    //     pwmR_base = 0;
    //     break;
    // }

    // Joystick-based differential drive control:
    // Negative ly because joystick up may be negative, adjust if needed
    float forward = ly;  
    float turn = rx;

    // Combine for left and right motor base PWM
    float pwmL_base = (forward + turn) * basePWM;
    float pwmR_base = (forward - turn) * basePWM;

    // Apply PID correction only to left motor PWM to sync speeds
    float pwmL_out = constrain(pwmL_base - correctionPWM, -4095, 4095);
    float pwmR_out = constrain(pwmR_base, -4095, 4095);

    // Set motor speeds
    setMotor(pwmL, dirL, pwmL_out, 0);
    setMotor(pwmR, dirR, pwmR_out, 1);
    // Calculate rotations
    float rotL = currentTicksL / (float)TICKS_PER_WHEEL_REV;
    float rotR = currentTicksR / (float)TICKS_PER_WHEEL_REV;

    // Debug output
    // Serial.printf("L: %5.1f RPM | R: %5.1f RPM | PWM: L=%4.0f, R=%4.0f | Rot: L=%.2f, R=%.2f\n",
    //               rpmL, rpmR, pwmL_out, pwmR_out, rotL, rotR);

    Serial.printf("L: %5.1f RPM | R: %5.1f RPM | PWM: L=%d, R=%d | Rot: L=%.2f, R=%.2f\n",
              rpmL, rpmR, (int)pwmL_out, (int)pwmR_out, rotL, rotR);

    // Print synchronization status
    // Serial.printf("Cmd:%c Speed:%d L_RPM:%.1f R_RPM:%.1f Corr:%.1f PWM_L:%.0f PWM_R:%.0f\n",
    //               command, (int)basePWM, rpmL, rpmR, correctionPWM, pwmL_out, pwmR_out);
  }
}



