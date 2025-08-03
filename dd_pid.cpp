// licensed under the University of Pennsylvania, Version 1.0 (the "License");
// Kartik Virmani MODLAB-UPENN
// you may not use this file except in compliance with the License.

#include <Arduino.h>
#include <esp32-hal.h>
#include <esp32-hal-gpio.h>
#include <esp32-hal-ledc.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <WebServer.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include "odometry.h"

// Your existing variables...
float lx, ly, rx, ry, lt, rt;
int a, b, x, y;

const char* webpage = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>HAMR Joystick</title>
  <style>
    body {
      font-family: sans-serif;
      text-align: center;
      background: #f0f0f0;
      margin: 0;
      padding: 0;
    }

    h2 {
      margin-top: 20px;
    }

    #joystickZone {
      width: 200px;
      height: 200px;
      margin: 20px auto;
      background: #ccc;
      border-radius: 50%;
      position: relative;
      touch-action: none;
    }

    #stick {
      width: 50px;
      height: 50px;
      background: #555;
      border-radius: 50%;
      position: absolute;
      top: 75px;
      left: 75px;
    }

    .controls {
      margin-top: 30px;
    }

    button {
      padding: 10px 20px;
      font-size: 16px;
      margin: 10px;
      background: #007bff;
      color: white;
      border: none;
      border-radius: 5px;
      cursor: pointer;
    }

    button:hover {
      background: #0056b3;
    }

    input[type="number"] {
      width: 80px;
      padding: 5px;
      font-size: 16px;
      margin-right: 10px;
    }
  </style>
</head>
<body>
  <h2>HAMR Virtual Joystick</h2>

  <div id="joystickZone">
    <div id="stick"></div>
  </div>

  <div class="controls">
    <button onclick="sendTrigger('lt')">LT</button>
    <button onclick="sendTrigger('rt')">RT</button>
  </div>

  <div class="controls">
    <input type="number" id="angleInput" placeholder="Angle" min="0" max="360">
    <button onclick="sendTurretAngle()">Set Turret</button>
  </div>

  <script>
    const stick = document.getElementById('stick');
    const zone = document.getElementById('joystickZone');
    let dragging = false;

    zone.addEventListener('touchstart', startDrag);
    zone.addEventListener('touchmove', drag);
    zone.addEventListener('touchend', endDrag);
    zone.addEventListener('mousedown', startDrag);
    zone.addEventListener('mousemove', drag);
    zone.addEventListener('mouseup', endDrag);
    zone.addEventListener('mouseleave', endDrag);

    function startDrag(e) {
      dragging = true;
    }

    function drag(e) {
      if (!dragging) return;
      e.preventDefault();
      let x = (e.touches ? e.touches[0].clientX : e.clientX) - zone.getBoundingClientRect().left;
      let y = (e.touches ? e.touches[0].clientY : e.clientY) - zone.getBoundingClientRect().top;
      let dx = x - 100;
      let dy = y - 100;
      let dist = Math.min(Math.sqrt(dx * dx + dy * dy), 75);
      let angle = Math.atan2(dy, dx);
      let stickX = Math.cos(angle) * dist + 75;
      let stickY = Math.sin(angle) * dist + 75;
      stick.style.left = stickX + 'px';
      stick.style.top = stickY + 'px';
      let normX = ((stickX - 75) / 75).toFixed(2);
      let normY = ((stickY - 75) / 75).toFixed(2);
      fetch(`/move?x=${normX}&y=${normY}`);
    }

    function endDrag() {
      dragging = false;
      stick.style.left = '75px';
      stick.style.top = '75px';
      fetch(`/move?x=0&y=0`);
    }

    function sendTrigger(trigger) {
      fetch(`/trigger?btn=${trigger}`);
    }

    function sendTurretAngle() {
      const angle = document.getElementById('angleInput').value;
      fetch(`/setTurretAngle?angle=${angle}`);
    }
  </script>
</body>
</html>
)rawliteral";

WebServer server(80);

// WIFI CREDENTIALS
const char* ssid = "HAMR";
const char* password = "123571113";

// UDP SETUP
WiFiUDP udp;
const int port = 12345;
char incoming[256];

// Motor pins
const int pwmL = 11;
const int dirL = 12;
const int encAL = 14;  // These match odometry.h now
const int encBL = 13;
const int pwmR = 41;
const int dirR = 42;
const int encAR = 1;   // These match odometry.h now
const int encBR = 2;
const int pwmT = 7;
const int dirT = 6;
const int enAT = 5;
const int enBT = 4;

// Encoder counts (volatile for ISR) - these are used by odometry.h
volatile long ticksL = 0;
volatile long ticksR = 0;
volatile long ticksT = 0;

// All your existing PID and control variables...
const float Kp_sync = 0.0;
const float Ki_sync = 0.0;
const float Kd_sync = 0.0;

const int CPR = 64;
const int GEAR_RATIO = 150;
const int TICKS_PER_WHEEL_REV = CPR * GEAR_RATIO;

const int TICKS_PER_TURRET_REV = 2704;
const float DEGREES_PER_TURRET_TICK = 360.0 / TICKS_PER_TURRET_REV;
const float motorGearTeeth = 41.0;
const float outputGearTeeth = 130.0;
const float turretGearRatio = outputGearTeeth / motorGearTeeth;

float currentAngleT = 0;
float inputTurretAngle = 0.0;
float targetTurretAngle = inputTurretAngle * turretGearRatio;

const float Kp_turret = 12.0;
const float Ki_turret = 0.8;
const float Kd_turret = 0.8;

float integralT = 0.0;
float lastErrorT = 0.0;
unsigned long lastTurretTime = 0;
const int minPWM = 200;
const int maxPWM = 4095;

const unsigned long PID_INTERVAL = 50;
static unsigned long lastUdpTime = 0;

float integralSync = 0;
float lastErrorSync = 0;
float errorT = 0;

unsigned long lastPidTime = 0;
long lastTicksL = 0;
long lastTicksR = 0;

float basePWM = 4095;
float pwmT_out = 0;

float ly = 0.0f;
float rx = 0.0f;
float lt = 0.0f;
float rt = 0.0f;
float joyX = 0.0f;
float joyY = 0.0f;
float joyturretX = 0.0f;
float joyturretY = 0.0f;

// Odometry timing
unsigned long lastOdometryTime = 0;
const unsigned long ODOMETRY_INTERVAL = 100; // Update odometry every 100ms

// Your existing encoder interrupts
void IRAM_ATTR handleEncL() {
  bool A = digitalRead(encAL);
  bool B = digitalRead(encBL);
  ticksL += (A == B) ? 1 : -1;
}

void IRAM_ATTR handleEncR() {
  bool A = digitalRead(encAR);
  bool B = digitalRead(encBR);
  ticksR += (A == B) ? -1 : 1;
}

void IRAM_ATTR handleEncT() {
  bool A = digitalRead(enAT);
  bool B = digitalRead(enBT);
  ticksT += (A == B) ? 1 : -1;
}

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
  Serial.println("ESP32 Ready");
  
  // Initialize odometry FIRST before setting up other interrupts
  initOdometry();
  Serial.println("Odometry system initialized");
  
  WiFi.softAP(ssid, password, 5, 0, 2);
  IPAddress myIP = WiFi.softAPIP();
  Serial.print("ESP IP: ");
  Serial.println(myIP);

  // Web server setup
  server.on("/", HTTP_GET, []() {
    server.send(200, "text/html", webpage);
  });

  server.on("/move", HTTP_GET, []() {
    float x = server.arg("x").toFloat();
    float y = server.arg("y").toFloat();
    joyX = x;
    joyY = -y;
    server.send(200, "text/plain", "Movement received");
  });

  server.on("/trigger", HTTP_GET, []() {
    String btn = server.arg("btn");
    if (btn == "lt") {
      pwmT_out = -maxPWM;
      delay(5000);
    } else if (btn == "rt") {
      pwmT_out = maxPWM;
      delay(5000);
    }
    setMotor(pwmT, dirT, pwmT_out, 2);
    server.send(200, "text/plain", "Trigger received: " + btn);
  });

  server.on("/setTurretAngle", HTTP_GET, []() {
    float angle = server.arg("angle").toFloat();
    inputTurretAngle = angle;
    targetTurretAngle = inputTurretAngle * turretGearRatio;
    server.send(200, "text/plain", "Turret angle set.");
  });

  server.onNotFound([]() {
    server.send(404, "text/plain", "404 Not Found");
  });
  server.begin();
  Serial.println("HTTP server started");
  
  udp.begin(port);
  Serial.printf("Listening for UDP on port %d\n", port);

  // Motor pins
  pinMode(pwmL, OUTPUT);
  pinMode(dirL, OUTPUT);
  pinMode(pwmR, OUTPUT);
  pinMode(dirR, OUTPUT);
  pinMode(pwmT, OUTPUT);
  pinMode(dirT, OUTPUT);

  // Setup PWM channels
  ledcSetup(0, 5000, 12);
  ledcSetup(1, 5000, 12);
  ledcSetup(2, 5000, 12);
  ledcAttachPin(pwmL, 0);
  ledcAttachPin(pwmR, 1);
  ledcAttachPin(pwmT, 2);

  // Turret encoder pins (these are separate from the drive encoders)
  pinMode(enAT, INPUT_PULLUP);
  pinMode(enBT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(enAT), handleEncT, CHANGE);

  lastPidTime = millis();
  lastTurretTime = millis();
  lastOdometryTime = millis(); // Initialize odometry timing

  Serial.println("Motor Sync Control Ready");
  Serial.println("Send joystick data like: LX:0.00 LY:0.00 RX:0.00 RY:0.00 LT:0.00 RT:0.00 A:0 B:0 X:0 Y:0");
  Serial.printf("Initial Speed PWM: %.0f\n", basePWM);
}

void loop() {
  server.handleClient();
  
  // Handle UDP input
  int len = udp.parsePacket();
  if (len > 0) {
    udp.read(incoming, sizeof(incoming));
    incoming[len] = '\0';
    String msg = String(incoming);

    int lyIndex = msg.indexOf("LY:");
    int rxIndex = msg.indexOf("RX:");
    int ltIndex = msg.indexOf("LT:");
    int rtIndex = msg.indexOf("RT:");

    if (lyIndex != -1 && rxIndex != -1 && ltIndex != -1 && rtIndex != -1) {
      int lyEnd = msg.indexOf(' ', lyIndex);
      if (lyEnd == -1) lyEnd = msg.length();
      int rxEnd = msg.indexOf(' ', rxIndex);
      if (rxEnd == -1) rxEnd = msg.length();
      int ltEnd = msg.indexOf(' ', ltIndex);
      if (ltEnd == -1) ltEnd = msg.length();
      int rtEnd = msg.indexOf(' ', rtIndex);
      if (rtEnd == -1) rtEnd = msg.length();

      String lyStr = msg.substring(lyIndex + 3, lyEnd);
      String rxStr = msg.substring(rxIndex + 3, rxEnd);
      String ltStr = msg.substring(ltIndex + 3, ltEnd);
      String rtStr = msg.substring(rtIndex + 3, rtEnd);

      ly = lyStr.toFloat();
      rx = rxStr.toFloat();
      lt = ltStr.toFloat();
      rt = rtStr.toFloat();
      lastUdpTime = millis();
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

    // Calculate error between motors
    float errorSync = rpmL - rpmR;

    integralSync += errorSync * dt;
    integralSync = constrain(integralSync, -100, 100);
    float dErrorSync = (errorSync - lastErrorSync) / dt;

    float correctionPWM = Kp_sync * errorSync + Ki_sync * integralSync + Kd_sync * dErrorSync;
    lastErrorSync = errorSync;

    // Turret control logic (your existing code)...
    if (lt > 0.1 || rt > 0.1) {
      float turretSpeed = (lt > 0.1) ? -lt * maxPWM : rt * maxPWM;
      pwmT_out = turretSpeed;
      currentAngleT = ticksT * DEGREES_PER_TURRET_TICK;
      currentAngleT = fmod(currentAngleT/turretGearRatio, 360.0);
      setMotor(pwmT, dirT, turretSpeed, 2);
    }
    else if (abs(targetTurretAngle) > 0.1) {
      noInterrupts();
      long currentTicksT = ticksT;
      interrupts();

      float currentAngleT = currentTicksT * DEGREES_PER_TURRET_TICK;
      float errorT = targetTurretAngle - currentAngleT;
      float pwmT_out = 0;
      if(abs(errorT) < 1.0) {
        setMotor(pwmT, dirT, 0, 2);
      }
      else{
        float dtT = (now - lastTurretTime) / 1000.0;
        if (dtT == 0) dtT = 0.001;
        integralT += errorT * dtT;
        integralT = constrain(integralT, -100, 100);
        float dErrorT = (errorT - lastErrorT) / dtT;
        float pwmT_out = Kp_turret * errorT + Ki_turret * integralT + Kd_turret * dErrorT;
        pwmT_out = constrain(pwmT_out, -maxPWM, maxPWM);
        setMotor(pwmT, dirT, pwmT_out, 2);
        lastErrorT = errorT;
        lastTurretTime = now;
      }
    }
    else{
      setMotor(pwmT, dirT, 0, 2);
    }

    // Joystick-based differential drive control
    float forward = ly;
    float turn = rx;

    float pwmL_base = (forward + turn) * basePWM;
    float pwmR_base = (forward - turn) * basePWM;

    float pwmL_out = constrain(pwmL_base - correctionPWM, -4095, 4095);
    float pwmR_out = constrain(pwmR_base, -4095, 4095);

    setMotor(pwmL, dirL, pwmL_out, 0);
    setMotor(pwmR, dirR, pwmR_out, 1);

    float rotL = currentTicksL / (float)TICKS_PER_WHEEL_REV;
    float rotR = currentTicksR / (float)TICKS_PER_WHEEL_REV;

    Serial.printf("L: %5.1f RPM | R: %5.1f RPM | Error_Turret: %5.1f  | PWM: L=%d, R=%d, T=%d | Rot: L=%.2f, R=%.2f, T_angle=%.2f\n", 
              rpmL, rpmR, errorT, (int)pwmL_out, (int)pwmR_out, (int)pwmT_out, rotL, rotR, currentAngleT);
  }

  // UPDATE ODOMETRY HERE - in the main loop
  if (now - lastOdometryTime >= ODOMETRY_INTERVAL) {
    updateOdometry();
    printPose(); // Print robot pose
    lastOdometryTime = now;
  }
}