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
#include <string.h>
#include "pid_webpage.h"

WebServer server(80);

// WIFI CREDENTIALS
const char* ssid = "HAMR";
const char* password = "123571113";

// UDP SETUP
WiFiUDP udp;
const int port = 12345;  // Port to listen on
char incoming[256];  // Buffer for incoming data
IPAddress remoteIP;
unsigned int remotePort;
// Left
const int pwmL = 11;
const int dirL = 12;
const int encAL = 14;
const int encBL = 13;
// Right
const int pwmR = 41;
const int dirR = 42;
const int encAR = 1;
const int encBR = 2;
// Turret
const int pwmT = 7;  
const int dirT = 6;   
const int enAT = 5;
const int enBT = 4;

// Encoder counts (volatile for ISR)
volatile long ticksL = 0;
volatile long ticksR = 0;
volatile long ticksT = 0;

// PID constants for synchronization
float Kp_L = 120.0f, Ki_L = 40.0f, Kd_L = 0.0f;   // tune per wheel
float Kp_R = 120.0f, Ki_R = 40.0f, Kd_R = 0.0f;   // tune per wheel

// Encoder & motor specs
const int CPR = 64;
const int GEAR_RATIO = 150;
const int TICKS_PER_WHEEL_REV = CPR * GEAR_RATIO; // 9600 ticks per wheel revolution

// Turret motor specs
const int TICKS_PER_TURRET_REV = 2704; // 13 PPR × 2 (quadrature) × 104 (gear ratio) = 2704 ticks/rev at output
const float DEGREES_PER_TURRET_TICK = 360.0 / TICKS_PER_TURRET_REV; // Degrees per tick
const float motorGearTeeth = 40.0; // Motor gear teeth
const float outputGearTeeth = 136.0; // Output gear teeth
const float turretGearRatio = outputGearTeeth / motorGearTeeth; // Turret gear ratio

// Target angle for turret in degrees
float currentAngleT = 0.0;
float inputTurretAngle = 0.0;  // Desired turret angle in degrees
float targetTurretAngle = inputTurretAngle * turretGearRatio; // Target angle in degrees

// PID constants for turret control
const float Kp_turret = 18.0;
const float Ki_turret = 0.05;
const float Kd_turret = 0.1;

float integralT = 0.0; // Integral term for turret PID0
float lastErrorT = 0.0; // Last error for turret PID
unsigned long lastTurretTime = 0; // Last time turret PID was updated
// PWM limits
const int minPWM = 200;   // Minimum PWM value
const int maxPWM = 4095; // Maximum PWM value (12-bit resolution)
const int maxPWM_D = 4095;
float pwmL_out = 0.0;
float pwmR_out = 0.0;

// Control interval (ms)
const unsigned long PID_INTERVAL = 50;
static unsigned long lastUdpTime = 0;

// PID state variables
float integralL = 0.0f, integralR = 0.0f;
float lastErrL = 0.0f, lastErrR = 0.0f;
float errorT = 0;

// Timing variables
unsigned long lastPidTime = 0;
long lastTicksL = 0;
long lastTicksR = 0;

// Base PWM speed (0-4095)
float basePWM = 3500;
const float MAX_RPM_CMD = 28.0f;
float pwmT_out = 0;
float scaleFactor = 1.0; //131.67;
// float turretSpeed = 0.0;

float test = 0.0f;

// Joystick control variables
float ly = 0.0f;  // left stick vertical (forward/back)
float rx = 0.0f;  // right stick horizontal (turn)
float lt = INT32_MIN; // left trigger
float rt = INT32_MIN; // left/right triggers
// HTTML Joystick control variables
float joyX = 0.0f;  // Joystick X-axis
float joyY = 0.0f;  // Joystick Y-axis
float joyturretX = 0.0f;  // Turret joystick X-axis
float joyturretY = 0.0f;  // Turret joystick Y-axis
String btn = "stop"; // Button pressed (e.g., 'f' for forward, 'b' for backward)
float value = 0.0f; // Value of the button pressed

// Odometry timing
unsigned long lastOdometryTime = 0;
const unsigned long ODOMETRY_INTERVAL = 100; // Odometry update interval in ms

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

// Turret encoder interrupt
void IRAM_ATTR handleEncT() {
  bool A = digitalRead(enAT);
  bool B = digitalRead(enBT);
  ticksT += (A == B) ? 1 : -1; // Adjust based on your encoder wiring
}

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

void sendUDP(String msg) {
  if (remoteIP && remotePort) {
    udp.beginPacket(remoteIP, remotePort);
    udp.print(msg);
    udp.endPacket();
  }
}

void setupProbabilisticEndpoints() {
  // Endpoint to get current pose with uncertainty
  server.on("/pose", HTTP_GET, []() {
    String json = "{";
    json += "\"x\":" + String(getRobotX(), 6) + ",";
    json += "\"y\":" + String(getRobotY(), 6) + ",";
    json += "\"theta\":" + String(getRobotTheta(), 6) + ",";
    json += "\"uncertainty_x\":" + String(getUncertaintyX(), 6) + ",";
    json += "\"uncertainty_y\":" + String(getUncertaintyY(), 6) + ",";
    json += "\"uncertainty_theta\":" + String(getUncertaintyTheta(), 6);
    json += "}";
    server.send(200, "application/json", json);
  });
  
  // Endpoint to reset odometry
  server.on("/reset", HTTP_GET, []() {
    resetOdometry();
    server.send(200, "text/plain", "Odometry reset");
  });
  
  // Endpoint to sample from pose distribution
  server.on("/sample", HTTP_GET, []() {
    float sample_x, sample_y, sample_theta;
    samplePose(sample_x, sample_y, sample_theta);
    String json = "{";
    json += "\"sample_x\":" + String(sample_x, 6) + ",";
    json += "\"sample_y\":" + String(sample_y, 6) + ",";
    json += "\"sample_theta\":" + String(sample_theta, 6);
    json += "}";
    server.send(200, "application/json", json);
  });
}

void setup() {

  Serial.begin(115200);
  Serial.println("ESP32 Ready");
  initOdometry(); // Initialize odometry
  WiFi.softAP(ssid, password, 4, 0, 2);
  IPAddress myIP = WiFi.softAPIP();
  Serial.print("ESP IP: ");
  Serial.println(myIP);

  server.on("/", HTTP_GET, []() {
    server.send(200, "text/html", pid_webpage);  // Serve the HTML page
  });
  
  server.on("/move", HTTP_GET, []() {
    float x = server.arg("x").toFloat();
    float y = server.arg("y").toFloat();
    joyX = x;
    joyY = -y;
    server.send(200, "text/plain", "Movement received");
  });

// Trigger buttons: LT or RT
  server.on("/trigger", HTTP_GET, []() {
    btn = server.arg("btn");
    value = server.arg("value").toFloat();  // Use 'value' param
    server.send(200, "text/plain", "Trigger received: " + btn);
  });

// Turret angle input
  server.on("/setTurretAngle", HTTP_GET, []() {
    float angle = server.arg("angle").toFloat();
    inputTurretAngle = -angle;
    targetTurretAngle = inputTurretAngle * turretGearRatio; // Apply gear ratio
    server.send(200, "text/plain", "Turret angle set.");
  });

  // GET current values (already suggested)
  server.on("/getPID", HTTP_GET, []() {
    String json = "{";
    json += "\"Kp\":" + String(Kp_L) + ",";
    json += "\"Ki\":" + String(Ki_L) + ",";
    json += "\"Kd\":" + String(Kd_L) + ",";
    json += "\"Test\":" + String(test, 4); // 4 decimals for float
    json += "}";
    server.send(200, "application/json", json);
});

  // POST updates
  server.on("/updatePID", HTTP_POST, []() {
    if (server.hasArg("Kp")) Kp_L = server.arg("Kp").toFloat();
    if (server.hasArg("Ki")) Ki_L = server.arg("Ki").toFloat();
    if (server.hasArg("Kd")) Kd_L = server.arg("Kd").toFloat();
    if (server.hasArg("Test")) test = server.arg("Test").toFloat();
    String response = "Updated PID values:\nKp=" + String(Kp_L) + "\nKi=" + String(Ki_L) + "\nKd=" + String(Kd_L) +
                      "\nTest=" + String(test, 4);
    server.send(200, "text/plain", response);
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

  // Turret motor pins
  pinMode(pwmT, OUTPUT);
  pinMode(dirT, OUTPUT);

  // Setup PWM channels at 5 kHz, 12-bit resolution
  ledcSetup(0, 5000, 12);
  ledcSetup(1, 5000, 12);
  ledcSetup(2, 5000, 12); // Turret PWM channel
  ledcAttachPin(pwmL, 0);
  ledcAttachPin(pwmR, 1);
  ledcAttachPin(pwmT, 2); // Attach turret PWM pin to channel 2

  // Encoder pins with pull-ups
  pinMode(encAL, INPUT_PULLUP);
  pinMode(encBL, INPUT_PULLUP);
  pinMode(encAR, INPUT_PULLUP);
  pinMode(encBR, INPUT_PULLUP);
  // Turret encoder pins with pull-ups
  pinMode(enAT, INPUT_PULLUP);
  pinMode(enBT, INPUT_PULLUP);
  
  // Attach interrupts on channel A for both encoders
  attachInterrupt(digitalPinToInterrupt(encAL), handleEncL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encAR), handleEncR, CHANGE);
  // Attach turret encoder interrupt
  attachInterrupt(digitalPinToInterrupt(enAT), handleEncT, CHANGE);

  lastPidTime = millis();
  lastTurretTime = millis();

  Serial.println("Motor Sync Control Ready");
  //Serial.println("Commands: f=forward, b=backward, r=right, l=left, s=stop, +=faster, -=slower");
  Serial.println("Send joystick data like: LX:0.00 LY:0.00 RX:0.00 RY:0.00 LT:0.00 RT:0.00 A:0 B:0 X:0 Y:0");
  Serial.printf("Initial Speed PWM: %.0f\n", basePWM);
}

void loop() {

  // Handle serial commands
  //   Read joystick data from serial
  //   if (Serial.available()) {
  //   String msg = Serial.readStringUntil('\n');}

    server.handleClient(); // Handle HTTP requests
    int len = udp.parsePacket();
    if (len > 0) {
      udp.read(incoming, sizeof(incoming));
      incoming[len] = '\0';  // null-terminate
      // Serial.printf("Received: %s\n", incoming);
      String msg = String(incoming); 

      remoteIP = udp.remoteIP();
      remotePort = udp.remotePort();

      int lyIndex = msg.indexOf("LY:");
      int rxIndex = msg.indexOf("RX:");
      int ltIndex = msg.indexOf("LT:");
      int rtIndex = msg.indexOf("RT:");

    if (lyIndex != -1 && rxIndex != -1 && ltIndex != -1 && rtIndex != -1) {
      // Extract LY and RX values as floats
      // Find next space or end of line after LY:
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

  ////=======================PID loop timing==================////
  unsigned long now = millis();
  if (now - lastPidTime >= PID_INTERVAL) {
    float dt = (now - lastPidTime) / 1000.0;

    ////=================== DRIVE CONTROL =================////
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

    // Joystick-based differential drive control:
    // Negative ly because joystick up may be negative, adjust if needed
    // Normalize joystick values to -1 to 1 range
    // float forward = ly;  
    // float turn = rx;

    // // HTTML Joystick control
    // float forward = joyY;  // Forward/backward control from joystick
    // float turn = joyX;     // Left/right control from joystick

    bool useUdp = (millis() - lastUdpTime < 100);
    float forward = useUdp ? ly : joyY;
    float turn = useUdp ? rx : joyX;

    forward = 0.8f;
    turn    *= 0.8f;

    // Combine for left and right motor base PWM
    float rpmTargetL = (forward + turn) * MAX_RPM_CMD;
    float rpmTargetR = (forward - turn) * MAX_RPM_CMD;

    float pwmFF_L = (forward + turn) * basePWM;
    float pwmFF_R = (forward - turn) * basePWM;

    // Calculate error in motord
    float errL = rpmTargetL - rpmL;
    float errR = rpmTargetR - rpmR;
    
    bool satL = (pwmL_out >=  maxPWM_D) || (pwmL_out <= -maxPWM_D);
    bool satR = (pwmR_out >=  maxPWM_D) || (pwmR_out <= -maxPWM_D);
    if (!satL) integralL += errL * dt;
    if (!satR) integralR += errR * dt;

    // Clamp integrators a bit
    integralL = constrain(integralL, -300.0f, 300.0f);
    integralR = constrain(integralR, -300.0f, 300.0f);
    float dErrL = (errL - lastErrL) / dt;
    float dErrR = (errR - lastErrR) / dt;
    lastErrL = errL;
    lastErrR = errR;

    float deltaPWM_L = Kp_L * errL + Ki_L * integralL + Kd_L * dErrL;
    float deltaPWM_R = Kp_R * errR + Ki_R * integralR + Kd_R * dErrR;

    // 4) Final PWM = feed-forward + PID correction, with saturation & deadband
    pwmL_out = constrain(rpmTargetL + deltaPWM_L, -maxPWM_D, maxPWM_D);
    pwmR_out = constrain(rpmTargetR + deltaPWM_R, -maxPWM_D, maxPWM_D);

    if (fabsf(pwmL_out) < minPWM) pwmL_out = 0.0f;
    if (fabsf(pwmR_out) < minPWM) pwmR_out = 0.0f;

    ////=================== TURRET CONTROL =================////
    if (btn == "l") {
      pwmT_out = -value * maxPWM;
    } else if (btn == "r") {
      pwmT_out = value * maxPWM;
    } else if (btn == "stop") {
      pwmT_out = 0;
    }
    setMotor(pwmT, dirT, pwmT_out, 2);
    
    if (lt > 0.1 || rt > 0.1){
    // Simple turret control based on triggers
      targetTurretAngle = 0.0;
      float turretSpeed = (lt > 0.1) ? -lt * maxPWM : rt * maxPWM;
      pwmT_out = turretSpeed;
      currentAngleT = ticksT * DEGREES_PER_TURRET_TICK; // Calculate turret angle in degrees
      currentAngleT = fmod(currentAngleT/turretGearRatio, 360.0);
    }
    else if (abs(targetTurretAngle)) {
      // Turret PID control
      // Read turret encoder counts atomically
      noInterrupts();
      long currentTicksT = ticksT;
      interrupts();
      // Calculate turret angle in degrees
      float currentAngleT = currentTicksT * DEGREES_PER_TURRET_TICK;
      // Calculate error for turret PID
      float errorT = targetTurretAngle - currentAngleT;
      if(abs(errorT) < 1.0 ) {
        pwmT_out = 0; // Reset output if within threshold
        integralT = 0; // Reset integral term if within threshold
      }
      else{
      float dtT = (now - lastTurretTime) / 1000.0; // Time in seconds
      if (dtT == 0) dtT = 0.001; // Avoid division by zero
      integralT += errorT * dtT;
      integralT = constrain(integralT, -100, 100);
      float dErrorT = (errorT - lastErrorT) / dtT;
      // Calculate turret PWM output
      pwmT_out = Kp_turret * errorT + Ki_turret * integralT + Kd_turret * dErrorT;
      pwmT_out = constrain(pwmT_out, -maxPWM, maxPWM);
      if (abs(pwmT_out) < 400) pwmT_out = 0;
      lastErrorT = errorT;
      lastTurretTime = now;
      }
    }
    else if ((lt < 0.1 || rt < 0.1) && (lt >= -1.5 || rt >= -1.5)) {
      pwmT_out = 0.0;
      pwmT_out = 0;
    }

    //// ======================== CONTROL END ===================////

    // Set motor speeds
    setMotor(pwmL, dirL, pwmL_out, 0);
    setMotor(pwmR, dirR, pwmR_out, 1);
    setMotor(pwmT, dirT, pwmT_out, 2);

    // Calculate rotations
    float rotL = currentTicksL / (float)TICKS_PER_WHEEL_REV;
    float rotR = currentTicksR / (float)TICKS_PER_WHEEL_REV;

    String status = String("L: ") + String(rpmL, 1) + " RPM | R: " + String(rpmR, 1) +
                " RPM | Error_Turret: " + String(errorT, 1) +
                " | PWM: L=" + String((int)pwmL_out) +
                ", R=" + String((int)pwmR_out) +
                ", T=" + String((int)pwmT_out) +
                " | Rot: L=" + String(rotL, 2) +
                ", R=" + String(rotR, 2) +
                ", T_angle=" + String(currentAngleT, 2);
    Serial.println(status);
    // sendUDP(status);

  }

  /////// ================= LOCALIZATION START =====================////

  if(now- lastOdometryTime >= ODOMETRY_INTERVAL) {
    // Update odometry every ODOMETRY_INTERVAL ms
    updateOdometry();
    // updateSampledPoseFromLastDelta();

    static unsigned long lastDetailedPrint = 0;
    if (now - lastDetailedPrint >= 1000) { // Print every 1-second
      Serial.println("\n PROBABILISTIC ODOM ESTIMATION:");
      printPose();
      // printMotionModel();

      static unsigned long lastCovPrint = 0;
      if (now - lastCovPrint >= 5000) { // Print covariance every 5 seconds
        // printCovariance();
        lastCovPrint = now;
      }

      float sample_x, sample_y, sample_theta;
      samplePose(sample_x, sample_y, sample_theta); 
      // Serial.printf("Sampled Pose: X=%.2f, Y=%.2f, Theta=%.2f\n", sample_x, sample_y, sample_theta * 180.0 / PI);
      // Serial.println("--------------------------------------------------");
      lastDetailedPrint = now;
    }
    lastOdometryTime = now;
  }
}


