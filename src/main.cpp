#include <Arduino.h>
#include <esp32-hal.h>
#include <esp32-hal-gpio.h>
#include <esp32-hal-ledc.h>

#include <WiFi.h>
#include <body.h>
#include <html510.h>
// HTML Server
HTML510Server h(80);

// Motor Driver Pins
const int PWM_PIN   = 7;  // PWM speed control
const int DIR_PIN   = 6;  // Direction control

// PWM Parameters
constexpr uint8_t  LEDC_CH      = 0;    // LEDC channel 0‑7
constexpr uint32_t PWM_FREQ_HZ  = 1000; // 1 kHz
constexpr uint8_t  PWM_RES_BITS = 8;    // 0‑255 counts
// Current State of the HTML App
int duty_percent   = 50; // 0‑100 %
int direction_cmd  = 0;  // -1, 0, +1

// Encoder Pins and Ticks
const int encoderPinA = 5;
const int encoderPinB = 4;
volatile long encoderTicks = 0;  // Total encoder ticks
float rotations = 0.0;
const int TICKS_PER_REV = 3840; // PPR 13 PPR × 2 (quadrature) × 104 (gear ratio) = 2704 ticks/rev at output

// Wi‑Fi AP credentials
constexpr char     WIFI_SSID[]  = "ESP32_Cedric_AP";
constexpr char     WIFI_PASS[]  = "102030405";
IPAddress myIp(192, 168, 1, 134); // IP address - put in browser

/* * * * * HTTP Handlers * * * * * */
void pageRoot() { h.sendhtml(body); }

/* Direction slider: expects -1, 0, or +1 */
void onDirection()
{
  direction_cmd = h.getVal();           // -1 / 0 / +1

  if (direction_cmd == 0) {
    ledcWrite(LEDC_CH, 0);              // motor coast / brake
  } else {
    digitalWrite(DIR_PIN, direction_cmd > 0 ? HIGH : LOW);
    int pwm = map(duty_percent, 0, 100, 0, (1<<PWM_RES_BITS) - 1);
    ledcWrite(LEDC_CH, pwm);
  }
}

/* Duty‑cycle slider: 0 ‑ 100 % */
void onDuty()
{
  duty_percent = h.getVal();
  if (direction_cmd != 0) {
    int pwm = map(duty_percent, 0, 100, 0, (1<<PWM_RES_BITS) - 1);
    ledcWrite(LEDC_CH, pwm);
  }
}


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
  pinMode(PWM_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(DIR_PIN, HIGH); // Direction
  
  ledcSetup(LEDC_CH, PWM_FREQ_HZ, PWM_RES_BITS); // 1kHz PWM, 8-bit
  ledcAttachPin(PWM_PIN, LEDC_CH); // PWM channel 0
  ledcWrite(LEDC_CH, map(duty_percent, 0, 100, 0, (1<<PWM_RES_BITS)-1)); // 50% duty cycle (adjust speed)

  // Encoder setup
  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderPinA), handleEncoderA, CHANGE);

  // Wi-Fi AP
  WiFi.mode(WIFI_MODE_AP);
  Serial.println("Setting AP…");
  WiFi.softAP(WIFI_SSID, WIFI_PASS);
  WiFi.softAPConfig(myIp, IPAddress(192,168,1,2), IPAddress(255,255,255,0));
  Serial.print("AP IP address: "); Serial.println(myIp);

  // Web server
  h.begin();
  h.attachHandler("/",                          pageRoot);
  h.attachHandler("/set?type=direction&value=", onDirection);
  h.attachHandler("/set?type=duty&value=",      onDuty);

}

void loop() {
  h.serve();

  static long lastTicks = 0;

  // if (lastTicks != encoderTicks) {
  //   rotations = (float)encoderTicks / TICKS_PER_REV;
  //   Serial.print("Ticks: ");
  //   Serial.print(encoderTicks);
  //   Serial.print("  Rotations: ");
  //   Serial.println(rotations, 3); // Print 3 decimal places
  //   lastTicks = encoderTicks;
  // }

  delay(10); // To reduce Serial output spam
}

