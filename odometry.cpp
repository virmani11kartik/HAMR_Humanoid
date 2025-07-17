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

// Encoder counts (volatile for ISR)
volatile long ticksL = 0;
volatile long ticksR = 0;

// PID constants for synchronization
const float Kp_sync = 2.0;
const float Ki_sync = 0.05;
const float Kd_sync = 0.01;

// Encoder & motor specs
const int CPR = 64;
const int GEAR_RATIO = 150;
const int TICKS_PER_WHEEL_REV = CPR * GEAR_RATIO; // 9600 ticks per wheel revolution

// Wheel and robot physical parameters (in meters)
const float WHEEL_DIAMETER = 0.1524;      // 6 inches in meters
const float WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * PI; // ~0.4788 meters
const float WHEEL_BASE = 0.3048;          // 12 inches in meters (track width)

// Distance traveled per tick
const float DIST_PER_TICK = WHEEL_CIRCUMFERENCE / TICKS_PER_WHEEL_REV; // ~4.9875e-5 meters/tick

// Control interval (ms)
const unsigned long PID_INTERVAL = 50;

// PID state variables
float integralSync = 0;
float lastErrorSync = 0;

// Timing variables
unsigned long lastPidTime = 0;
long lastTicksL = 0;
long lastTicksR = 0;

// Base PWM speed (0-255)
float basePWM = 150;

// Current command
char command = 's'; // start stopped

// Robot pose variables (meters and radians)
float x = 0.0;
float y = 0.0;
float theta = 0.0; // robot heading in radians

// Encoder interrupts (quadrature decoding)
void IRAM_ATTR handleEncL() {
  bool A = digitalRead(encAL);
  bool B = digitalRead(encBL);
  ticksL += (A == B) ? 1 : -1;
}

void IRAM_ATTR handleEncR() {
  bool A = digitalRead(encAR);
  bool B = digitalRead(encBR);
  ticksR += (A == B) ? -1 : 1;  // Inverted to fix direction if needed
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

// Update odometry function
void updateOdometry(long currTicksL, long currTicksR) {
  static long prevTicksL = 0;
  static long prevTicksR = 0;

  long deltaTicksL = currTicksL - prevTicksL;
  long deltaTicksR = currTicksR - prevTicksR;

  prevTicksL = currTicksL;
  prevTicksR = currTicksR;

  float dSL = deltaTicksL * DIST_PER_TICK;
  float dSR = deltaTicksR * DIST_PER_TICK;

  float dS = (dSR + dSL) / 2.0;
  float dTheta = (dSR - dSL) / WHEEL_BASE;

  float thetaMid = theta + dTheta / 2.0;

  x += dS * cos(thetaMid);
  y += dS * sin(thetaMid);
  theta += dTheta;

  // Normalize theta to -PI..PI
  if (theta > PI) theta -= 2 * PI;
  else if (theta < -PI) theta += 2 * PI;
}

void setup() {
  Serial.begin(115200);

  // Motor pins
  pinMode(pwmL, OUTPUT);
  pinMode(dirL, OUTPUT);
  pinMode(pwmR, OUTPUT);
  pinMode(dirR, OUTPUT);

  // Setup PWM channels at 5 kHz, 8-bit resolution
  ledcSetup(0, 5000, 8);
  ledcSetup(1, 5000, 8);
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

  Serial.println("Motor Sync & Odometry Control Ready");
  Serial.println("Commands: f=forward, b=backward, r=right, l=left, s=stop, +=faster, -=slower");
  Serial.printf("Initial Speed PWM: %.0f\n", basePWM);
}

void loop() {
  // Handle serial commands
  if (Serial.available()) {
    char newCommand = Serial.read();
    if (newCommand == 'f' || newCommand == 'b' || newCommand == 'r' || newCommand == 'l' || newCommand == 's') {
      command = newCommand;
      integralSync = 0;
      lastErrorSync = 0;
      Serial.printf("Command: %c\n", command);
    } else if (newCommand == '+') {
      basePWM += 10;
      if (basePWM > 255) basePWM = 255;
      Serial.printf("Speed increased: %d\n", (int)basePWM);
    } else if (newCommand == '-') {
      basePWM -= 10;
      if (basePWM < 0) basePWM = 0;
      Serial.printf("Speed decreased: %d\n", (int)basePWM);
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

    // Update odometry
    updateOdometry(currentTicksL, currentTicksR);

    // Calculate PID correction only when moving straight (forward/backward)
    float correctionPWM = 0;
    if (command == 'f' || command == 'b') {
      float errorSync = rpmL - rpmR;

      integralSync += errorSync * dt;
      integralSync = constrain(integralSync, -100, 100);
      float dErrorSync = (errorSync - lastErrorSync) / dt;

      correctionPWM = Kp_sync * errorSync + Ki_sync * integralSync + Kd_sync * dErrorSync;
      lastErrorSync = errorSync;
    } else {
      // No PID correction during turns or stop
      integralSync = 0;
      lastErrorSync = 0;
    }

    // Base PWM values per motor based on command
    float pwmL_base = 0, pwmR_base = 0;
    switch (command) {
      case 'f': // forward both motors
        pwmL_base = basePWM;
        pwmR_base = basePWM;
        break;
      case 'b': // backward both motors
        pwmL_base = -basePWM;
        pwmR_base = -basePWM;
        break;
      case 'r': // right turn: left forward, right backward
        pwmL_base = basePWM;
        pwmR_base = -basePWM;
        break;
      case 'l': // left turn: left backward, right forward
        pwmL_base = -basePWM;
        pwmR_base = basePWM;
        break;
      case 's': // stop
      default:
        pwmL_base = 0;
        pwmR_base = 0;
        break;
    }

    // Apply PID correction only to left motor PWM to sync speeds
    float pwmL_out = constrain(pwmL_base - correctionPWM, -255, 255);
    float pwmR_out = constrain(pwmR_base, -255, 255);

    // Set motor speeds
    setMotor(pwmL, dirL, pwmL_out, 0);
    setMotor(pwmR, dirR, pwmR_out, 1);

    // Debug output: speeds, PWM and pose
    Serial.printf("Cmd:%c Speed:%d L_RPM:%.1f R_RPM:%.1f Corr:%.1f PWM_L:%.0f PWM_R:%.0f\n",
                  command, (int)basePWM, rpmL, rpmR, correctionPWM, pwmL_out, pwmR_out);

    Serial.printf("Pose -> X: %.3f m, Y: %.3f m, Theta: %.3f rad\n", x, y, theta);
  }
}
