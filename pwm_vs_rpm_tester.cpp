#include <Arduino.h>
#include <esp32-hal-ledc.h>

// Motor pins
const int pwmPin = 11;  // Choose one motor (Left or Right)
const int dirPin = 12;
const int encA = 14;
const int encB = 13;

// PID not used here — just testing motor response
volatile long ticks = 0;

const int CPR = 64;
const int GEAR_RATIO = 150;
const int TICKS_PER_REV = CPR * GEAR_RATIO;

unsigned long lastPrint = 0;
long lastTicks = 0;

// PWM sweep
int pwm = 0;
int step = 100;
unsigned long lastUpdate = 0;
const int dwell = 1000;  // ms to hold each PWM step

void IRAM_ATTR encoderISR() {
  bool A = digitalRead(encA);
  bool B = digitalRead(encB);
  ticks += (A == B) ? 1 : -1;
}

void setMotor(int pwmVal) {
  pwmVal = constrain(pwmVal, 0, 4095);  // Positive only for now
  digitalWrite(dirPin, HIGH);
  ledcWrite(0, pwmVal);
}

void setup() {
  Serial.begin(115200);

  pinMode(pwmPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(encA, INPUT_PULLUP);
  pinMode(encB, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(encA), encoderISR, CHANGE);

  ledcSetup(0, 5000, 12);  // 5kHz, 12-bit
  ledcAttachPin(pwmPin, 0);

  Serial.println("PWM vs RPM Sweep Starting...");
}

void loop() {
  unsigned long now = millis();

  // Print every 200ms
  if (now - lastPrint >= 200) {
    noInterrupts();
    long tickSnapshot = ticks;
    interrupts();

    float dt = (now - lastPrint) / 1000.0;
    float rpm = ((tickSnapshot - lastTicks) / dt) * 60.0 / TICKS_PER_REV;

    lastTicks = tickSnapshot;
    lastPrint = now;

    Serial.printf("PWM: %d | RPM: %.2f\n", pwm, rpm);
  }

  // Increase PWM every `dwell` milliseconds
  if (now - lastUpdate >= dwell) {
    pwm += step;
    if (pwm > 4095) {
      pwm = 0;
    }
    setMotor(pwm);
    lastUpdate = now;
  }
}
