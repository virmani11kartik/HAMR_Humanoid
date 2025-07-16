#include <Arduino.h>
#include <esp32-hal.h>
#include <esp32-hal-gpio.h>
#include <esp32-hal-ledc.h>


// ==== MOTOR & ENCODER PINS ====
#define LEFT_PWM     11
#define LEFT_DIR     12
#define LEFT_ENC_A   14
#define LEFT_ENC_B   13

#define RIGHT_PWM    41
#define RIGHT_DIR    42
#define RIGHT_ENC_A  1
#define RIGHT_ENC_B  2

// ==== PID CONFIG ====
float targetSpeed = 100.0; // ticks per second
float Kp = 1.0, Ki = 0.2, Kd = 0.05;

// ==== ENCODER STATE ====
volatile long leftTicks = 0, rightTicks = 0;
long prevLeftTicks = 0, prevRightTicks = 0;

// PID state
float leftIntegral = 0, rightIntegral = 0;
float leftPrevError = 0, rightPrevError = 0;

unsigned long prevTime = 0;

// ==== ENCODER INTERRUPTS ====
void IRAM_ATTR leftEncoderISR() {
  leftTicks++;
}

void IRAM_ATTR rightEncoderISR() {
  rightTicks++;
}

// ==== SETUP ====
void setup() {
  Serial.begin(115200);

  pinMode(LEFT_PWM, OUTPUT);
  pinMode(LEFT_DIR, OUTPUT);
  pinMode(RIGHT_PWM, OUTPUT);
  pinMode(RIGHT_DIR, OUTPUT);

  pinMode(LEFT_ENC_A, INPUT_PULLUP);
  pinMode(LEFT_ENC_B, INPUT_PULLUP);
  pinMode(RIGHT_ENC_A, INPUT_PULLUP);
  pinMode(RIGHT_ENC_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), rightEncoderISR, RISING);

  prevTime = millis();
}

// ==== HELPER ====
void setMotor(int pwmPin, int dirPin, int pwmVal, bool forward = true) {
  digitalWrite(dirPin, forward ? HIGH : LOW);
  ledcWrite(pwmPin, constrain(pwmVal, 0, 255));
}

// ==== MAIN LOOP ====
void loop() {
  unsigned long now = millis();
  float dt = (now - prevTime) / 1000.0; // seconds

  if (dt >= 0.1) { // every 100ms
    long leftDelta = leftTicks - prevLeftTicks;
    long rightDelta = rightTicks - prevRightTicks;

    float leftSpeed = leftDelta / dt;   // ticks/sec
    float rightSpeed = rightDelta / dt;

    // === LEFT PID ===
    float l_error = targetSpeed - leftSpeed;
    leftIntegral += l_error * dt;
    float l_derivative = (l_error - leftPrevError) / dt;
    float l_output = Kp * l_error + Ki * leftIntegral + Kd * l_derivative;
    leftPrevError = l_error;

    // === RIGHT PID ===
    float r_error = targetSpeed - rightSpeed;
    rightIntegral += r_error * dt;
    float r_derivative = (r_error - rightPrevError) / dt;
    float r_output = Kp * r_error + Ki * rightIntegral + Kd * r_derivative;
    rightPrevError = r_error;

    // === MOTOR COMMANDS ===
    setMotor(LEFT_PWM, LEFT_DIR, (int)l_output);
    setMotor(RIGHT_PWM, RIGHT_DIR, (int)r_output);

    // Debugging
    Serial.printf("L: %.1f  R: %.1f  -> PWM L: %.1f  R: %.1f\n", leftSpeed, rightSpeed, l_output, r_output);

    // Update for next cycle
    prevLeftTicks = leftTicks;
    prevRightTicks = rightTicks;
    prevTime = now;
  }
}
// ==== END OF FILE ====