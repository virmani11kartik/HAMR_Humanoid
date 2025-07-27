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

// // // float lx, ly, rx, ry, lt, rt;
int a, b, x, y;

const char* webpage = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>HAMR</title>
  <style>
    body {
      font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
      text-align: center;
      background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
      margin: 0;
      padding: 20px;
      color: white;
      min-height: 100vh;
    }

    .container {
      max-width: 500px;
      margin: 0 auto;
      background: rgba(255, 255, 255, 0.1);
      border-radius: 20px;
      padding: 30px;
      backdrop-filter: blur(10px);
      box-shadow: 0 8px 32px rgba(0, 0, 0, 0.3);
      border: 1px solid rgba(255, 255, 255, 0.2);
    }

    h2 {
      margin-top: 0;
      margin-bottom: 30px;
      font-size: 2.2em;
      text-shadow: 2px 2px 4px rgba(0, 0, 0, 0.3);
      display: flex;
      align-items: center;
      justify-content: center;
      gap: 15px;
    }

    #joystickZone {
      width: 200px;
      height: 200px;
      margin: 20px auto;
      background: linear-gradient(145deg, #e0e0e0, #c0c0c0);
      border-radius: 50%;
      position: relative;
      touch-action: none;
      box-shadow: inset 8px 8px 16px rgba(0, 0, 0, 0.2),
                  inset -8px -8px 16px rgba(255, 255, 255, 0.7);
    }

    #stick {
      width: 50px;
      height: 50px;
      background: linear-gradient(145deg, #555, #333);
      border-radius: 50%;
      position: absolute;
      top: 75px;
      left: 75px;
      cursor: pointer;
      transition: all 0.1s ease;
      box-shadow: 4px 4px 8px rgba(0, 0, 0, 0.3),
                  -2px -2px 4px rgba(255, 255, 255, 0.1);
    }

    #stick:hover {
      transform: scale(1.1);
    }

    .control-group {
      margin: 25px 0;
      padding: 20px;
      background: rgba(255, 255, 255, 0.05);
      border-radius: 15px;
      border: 1px solid rgba(255, 255, 255, 0.1);
    }

    .control-group h3 {
      margin: 0 0 15px 0;
      font-size: 1.3em;
      color: #fff;
    }

    .slider-container {
      display: flex;
      align-items: center;
      gap: 15px;
      justify-content: center;
      margin: 15px 0;
    }

    .slider {
      -webkit-appearance: none;
      width: 200px;
      height: 8px;
      border-radius: 4px;
      background: linear-gradient(90deg, #ff6b6b 0%, #4ecdc4 50%, #45b7d1 100%);
      outline: none;
      position: relative;
    }

    .slider::-webkit-slider-thumb {
      -webkit-appearance: none;
      appearance: none;
      width: 24px;
      height: 24px;
      border-radius: 50%;
      background: linear-gradient(145deg, #fff, #ddd);
      cursor: pointer;
      box-shadow: 0 4px 8px rgba(0, 0, 0, 0.3);
      transition: all 0.2s ease;
    }

    .slider::-webkit-slider-thumb:hover {
      transform: scale(1.2);
      box-shadow: 0 6px 12px rgba(0, 0, 0, 0.4);
    }

    .slider::-moz-range-thumb {
      width: 24px;
      height: 24px;
      border-radius: 50%;
      background: linear-gradient(145deg, #fff, #ddd);
      cursor: pointer;
      border: none;
      box-shadow: 0 4px 8px rgba(0, 0, 0, 0.3);
    }

    .slider-labels {
      display: flex;
      justify-content: space-between;
      width: 200px;
      margin: 10px auto 0;
      font-size: 0.9em;
      color: rgba(255, 255, 255, 0.8);
    }

    .slider-value {
      min-width: 60px;
      padding: 8px 12px;
      background: rgba(255, 255, 255, 0.1);
      border-radius: 8px;
      font-family: monospace;
      font-size: 1.1em;
      border: 1px solid rgba(255, 255, 255, 0.2);
      margin-top: 10px;
    }

    button {
      padding: 12px 24px;
      font-size: 16px;
      margin: 10px;
      background: linear-gradient(145deg, #4ecdc4, #44a08d);
      color: white;
      border: none;
      border-radius: 25px;
      cursor: pointer;
      transition: all 0.3s ease;
      font-weight: 600;
      box-shadow: 0 4px 15px rgba(68, 160, 141, 0.3);
    }

    button:hover {
      transform: translateY(-2px);
      box-shadow: 0 6px 20px rgba(68, 160, 141, 0.4);
      background: linear-gradient(145deg, #5dd4cc, #4ecdc4);
    }

    button:active {
      transform: translateY(0);
    }

    input[type="number"] {
      width: 100px;
      padding: 12px;
      font-size: 16px;
      margin-right: 10px;
      border: 2px solid rgba(255, 255, 255, 0.2);
      border-radius: 12px;
      background: rgba(255, 255, 255, 0.1);
      color: white;
      text-align: center;
    }

    input[type="number"]::placeholder {
      color: rgba(255, 255, 255, 0.6);
    }

    input[type="number"]:focus {
      outline: none;
      border-color: #4ecdc4;
      box-shadow: 0 0 10px rgba(78, 205, 196, 0.3);
    }

    .status-indicator {
      display: inline-block;
      width: 12px;
      height: 12px;
      border-radius: 50%;
      background: #4ecdc4;
      animation: pulse 2s infinite;
    }

    @keyframes pulse {
      0% { opacity: 1; }
      50% { opacity: 0.5; }
      100% { opacity: 1; }
    }

    .info-panel {
      margin-top: 20px;
      padding: 15px;
      background: rgba(0, 0, 0, 0.2);
      border-radius: 10px;
      font-family: monospace;
      font-size: 0.9em;
      text-align: left;
    }

    .trigger-control {
      width: 100%;
      max-width: 300px;
      margin: 0 auto;
    }

    .trigger-end-label {
      font-size: 1.1em;
      font-weight: 600;
      color: rgba(255, 255, 255, 0.9);
      min-width: 30px;
    }
  </style>
</head>
<body>
  <div class="container">
    <h2>HAMR Robot Control <span class="status-indicator"></span></h2>

    <div class="control-group">
      <h3>Drive Control</h3>
      <div id="joystickZone">
        <div id="stick"></div>
      </div>
    </div>

    <div class="control-group">
      <h3>Turret Control</h3>
      <div class="trigger-control">
        <div class="slider-container">
          <span class="trigger-end-label">LT</span>
          <input type="range" id="turretSlider" class="slider" min="-100" max="100" value="0" step="1">
          <span class="trigger-end-label">RT</span>
        </div>
        <div class="slider-labels">
          <span>-100%</span>
          <span>0%</span>
          <span>+100%</span>
        </div>
        <div class="slider-value" id="turretValue">0%</div>
      </div>
    </div>

    <div class="control-group">
      <h3>Turret Position</h3>
      <input type="number" id="angleInput" placeholder="Angle (0-360)" min="0" max="360" step="1">
      <button onclick="sendTurretAngle()">Set Position</button>
    </div>

    <div class="control-group">
      <button onclick="resetOdometry()">Reset Odometry</button>
      <button onclick="getPose()">Get Pose</button>
    </div>

    <div class="info-panel" id="infoPanel">
      Robot Status: Ready<br>
      Position: X=0.000, Y=0.000, θ=0.0°<br>
      LT: 0.00 | RT: 0.00<br>
      Last Command: None
    </div>
  </div>

  <script>
    const stick = document.getElementById('stick');
    const zone = document.getElementById('joystickZone');
    const turretSlider = document.getElementById('turretSlider');
    const turretValue = document.getElementById('turretValue');
    const infoPanel = document.getElementById('infoPanel');
    
    let dragging = false;
    let currentTurretValue = 0;

    // Joystick event listeners
    zone.addEventListener('touchstart', startDrag);
    zone.addEventListener('touchmove', drag);
    zone.addEventListener('touchend', endDrag);
    zone.addEventListener('mousedown', startDrag);
    zone.addEventListener('mousemove', drag);
    zone.addEventListener('mouseup', endDrag);
    zone.addEventListener('mouseleave', endDrag);

    // Turret slider event listeners
    turretSlider.addEventListener('input', function() {
      currentTurretValue = parseInt(this.value);
      turretValue.textContent = currentTurretValue + '%';
      
      if (currentTurretValue < 0) {
        // Left side = LT trigger
        const ltValue = Math.abs(currentTurretValue) / 100.0;
        sendTrigger('lt', ltValue);
      } else if (currentTurretValue > 0) {
        // Right side = RT trigger  
        const rtValue = currentTurretValue / 100.0;
        sendTrigger('rt', rtValue);
      } else {
        // Center = stop
        sendTrigger('stop', 0);
      }
      
      updateInfoPanel();
    });

    // Auto-return to center when released
    turretSlider.addEventListener('mouseup', returnToCenter);
    turretSlider.addEventListener('touchend', returnToCenter);

    function returnToCenter() {
      setTimeout(() => {
        if (!turretSlider.matches(':active')) {
          turretSlider.value = 0;
          currentTurretValue = 0;
          turretValue.textContent = '0%';
          sendTrigger('stop', 0);
          updateInfoPanel();
        }
      }, 100);
    }

    function startDrag(e) {
      dragging = true;
      e.preventDefault();
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
      
      fetch(`/move?x=${normX}&y=${normY}`)
        .then(response => response.text())
        .then(data => {
          updateInfoPanel(`Drive: X=${normX}, Y=${normY}`);
        })
        .catch(err => console.error('Drive command failed:', err));
    }

    function endDrag() {
      dragging = false;
      stick.style.left = '75px';
      stick.style.top = '75px';
      fetch(`/move?x=0&y=0`)
        .then(() => updateInfoPanel('Drive: Stopped'))
        .catch(err => console.error('Stop command failed:', err));
    }

    function sendTrigger(trigger, value) {
      if (trigger === 'stop') {
        fetch(`/trigger?btn=stop`)
          .then(response => response.text())
          .then(data => {
            console.log('Turret stopped');
          })
          .catch(err => console.error('Stop command failed:', err));
      } else {
        fetch(`/trigger?btn=${trigger}&value=${value}`)
          .then(response => response.text())
          .then(data => {
            console.log(`${trigger.toUpperCase()} set to ${value.toFixed(2)}`);
          })
          .catch(err => console.error(`${trigger} command failed:`, err));
      }
    }

    function sendTurretAngle() {
      const angle = document.getElementById('angleInput').value;
      if (angle === '') {
        alert('Please enter an angle value');
        return;
      }
      
      fetch(`/setTurretAngle?angle=${angle}`)
        .then(response => response.text())
        .then(data => {
          updateInfoPanel(`Turret angle set to ${angle}°`);
        })
        .catch(err => {
          console.error('Turret angle command failed:', err);
          updateInfoPanel('Turret angle command failed');
        });
    }

    function resetOdometry() {
      fetch('/reset')
        .then(response => response.text())
        .then(data => {
          updateInfoPanel('Odometry reset successfully');
          getPose();
        })
        .catch(err => {
          console.error('Reset failed:', err);
          updateInfoPanel('Reset command failed');
        });
    }

    function getPose() {
      fetch('/pose')
        .then(response => response.json())
        .then(data => {
          updateInfoPanel(`Position: X=${data.x.toFixed(3)}, Y=${data.y.toFixed(3)}, θ=${(data.theta * 180 / Math.PI).toFixed(1)}°`);
        })
        .catch(err => {
          console.error('Get pose failed:', err);
          updateInfoPanel('Failed to get pose data');
        });
    }

    function updateInfoPanel(message) {
      const timestamp = new Date().toLocaleTimeString();
      let triggerStatus = 'None';
      
      if (currentTurretValue < 0) {
        triggerStatus = `LT: ${(Math.abs(currentTurretValue) / 100).toFixed(2)}`;
      } else if (currentTurretValue > 0) {
        triggerStatus = `RT: ${(currentTurretValue / 100).toFixed(2)}`;
      }
      
      infoPanel.innerHTML = `
        Robot Status: Active<br>
        Turret Control: ${triggerStatus}<br>
        Last Command: ${message || 'None'}<br>
        Time: ${timestamp}
      `;
    }

    // Initialize display
    window.addEventListener('load', function() {
      turretSlider.value = 0;
      turretValue.textContent = '0%';
      updateInfoPanel('System initialized');
    });

    // Auto-update pose every 2 seconds
    setInterval(getPose, 2000);
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
const int port = 12345;  // Port to listen on
char incoming[256];  // Buffer for incoming data

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
const float Kp_sync = 0.0;
const float Ki_sync = 0.0;
const float Kd_sync = 0.0;

// Encoder & motor specs
const int CPR = 64;
const int GEAR_RATIO = 150;
const int TICKS_PER_WHEEL_REV = CPR * GEAR_RATIO; // 9600 ticks per wheel revolution

// Turret motor specs
const int TICKS_PER_TURRET_REV = 2704; // 13 PPR × 2 (quadrature) × 104 (gear ratio) = 2704 ticks/rev at output
const float DEGREES_PER_TURRET_TICK = 360.0 / TICKS_PER_TURRET_REV; // Degrees per tick
const float motorGearTeeth = 41.0; // Motor gear teeth
const float outputGearTeeth = 130.0; // Output gear teeth
const float turretGearRatio = outputGearTeeth / motorGearTeeth; // Turret gear ratio

// Target angle for turret in degrees
float currentAngleT = 0;
float inputTurretAngle = 0.0;  // Desired turret angle in degrees
float targetTurretAngle = inputTurretAngle * turretGearRatio; // Target angle in degrees

// PID constants for turret control
const float Kp_turret = 12.0;
const float Ki_turret = 0.8;
const float Kd_turret = 0.8;

float integralT = 0.0; // Integral term for turret PID0
float lastErrorT = 0.0; // Last error for turret PID
unsigned long lastTurretTime = 0; // Last time turret PID was updated
// PWM limits
const int minPWM = 200;   // Minimum PWM value
const int maxPWM = 4095; // Maximum PWM value (12-bit resolution)

// Control interval (ms)
const unsigned long PID_INTERVAL = 50;
static unsigned long lastUdpTime = 0;

// PID state variables
float integralSync = 0;
float lastErrorSync = 0;
float errorT = 0;

// Timing variables
unsigned long lastPidTime = 0;
long lastTicksL = 0;
long lastTicksR = 0;

// Base PWM speed (0-4095)
float basePWM = 4095;
float pwmT_out = 0;
// float turretSpeed = 0.0;

// Joystick control variables
float ly = 0.0f;  // left stick vertical (forward/back)
float rx = 0.0f;  // right stick horizontal (turn)
float lt = 0.0f;
float rt = 0.0f; // left/right triggers
// HTTML Joystick control variables
float joyX = 0.0f;  // Joystick X-axis
float joyY = 0.0f;  // Joystick Y-axis
float joyturretX = 0.0f;  // Turret joystick X-axis
float joyturretY = 0.0f;  // Turret joystick Y-axis

// Odometry timing
unsigned long lastOdometryTime = 0;
const unsigned long ODOMETRY_INTERVAL = 100; // Odometry update interval in ms

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
  WiFi.softAP(ssid, password, 5, 0, 2);
  IPAddress myIP = WiFi.softAPIP();
  Serial.print("ESP IP: ");
  Serial.println(myIP);

  server.on("/", HTTP_GET, []() {
    server.send(200, "text/html", webpage);  // Serve the HTML page
  });
  // server.on("/move", HTTP_GET, []() {
  //   String xVal = server.arg("x");
  //   String yVal = server.arg("y");
  //   joyX= xVal.toFloat();
  //   joyY= -yVal.toFloat();
  //   Serial.printf("Joystick X: %.2f, Y: %.2f\n", joyX, joyY);
  //   server.send(200, "text/plain", "OK");
  // });
  // Joystick movement (single virtual joystick)
server.on("/move", HTTP_GET, []() {
  float x = server.arg("x").toFloat();
  float y = server.arg("y").toFloat();
  joyX = x;
  joyY = -y;
  server.send(200, "text/plain", "Movement received");
});

// Trigger buttons: LT or RT
server.on("/trigger", HTTP_GET, []() {
  String btn = server.arg("btn");
  float value = server.arg("value").toFloat();  // Use 'value' param

  if (btn == "lt") {
    pwmT_out = -value * maxPWM;
  } else if (btn == "rt") {
    pwmT_out = value * maxPWM;
  } else if (btn == "stop") {
    pwmT_out = 0;
  }

  setMotor(pwmT, dirT, pwmT_out, 2);
  server.send(200, "text/plain", "Trigger received: " + btn);
});

// Turret angle input
server.on("/setTurretAngle", HTTP_GET, []() {
  float angle = server.arg("angle").toFloat();
  inputTurretAngle = angle;
  targetTurretAngle = inputTurretAngle * turretGearRatio; // Apply gear ratio
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
    // if (Serial.available()) {
    // String msg = Serial.readStringUntil('\n');

    server.handleClient(); // Handle HTTP requests
    int len = udp.parsePacket();
    if (len > 0) {
      udp.read(incoming, sizeof(incoming));
      incoming[len] = '\0';  // null-terminate
      // Serial.printf("Received: %s\n", incoming);
      String msg = String(incoming); 

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

    // Turret control based on HTML joystick input
    // float turretSpeed = joyturretX * maxPWM;  // or joyturretY * maxPWM;
    // pwmT_out = turretSpeed;
    // noInterrupts();
    // long currentTicksT = ticksT;
    // interrupts();
    // currentAngleT = currentTicksT * DEGREES_PER_TURRET_TICK;
    // currentAngleT = fmod(currentAngleT / turretGearRatio, 360.0);
    // setMotor(pwmT, dirT, turretSpeed, 2);

    if (lt > 0.1 || rt > 0.1){
    // Simple turret control based on triggers
      float turretSpeed = (lt > 0.1) ? -lt * maxPWM : rt * maxPWM;
      pwmT_out = turretSpeed;
      currentAngleT = ticksT * DEGREES_PER_TURRET_TICK; // Calculate turret angle in degrees
      currentAngleT = fmod(currentAngleT/turretGearRatio, 360.0);
    setMotor(pwmT, dirT, turretSpeed, 2);
    }
    else if (abs(targetTurretAngle) > 0.1) {
      // Turret PID control
      // Read turret encoder counts atomically
      noInterrupts();
      long currentTicksT = ticksT;
      interrupts();

      // Calculate turret angle in degrees
      float currentAngleT = currentTicksT * DEGREES_PER_TURRET_TICK;
      // Calculate error for turret PID
      float errorT = targetTurretAngle - currentAngleT;
      float pwmT_out = 0;
      if(abs(errorT) < 1.0 || abs(pwmT_out) < 100) {
        setMotor(pwmT, dirT, 0, 2);  // Stop turret
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
        float pwmT_out = Kp_turret * errorT + Ki_turret * integralT + Kd_turret * dErrorT;
        pwmT_out = constrain(pwmT_out, -maxPWM, maxPWM);
        // if (abs(pwmT_out) < minPWM) pwmT_out = 0;
        // Set turret motor PWM
        setMotor(pwmT, dirT, pwmT_out, 2); 
        lastErrorT = errorT;
        lastTurretTime = now;
      }
    }
    else{
      setMotor(pwmT, dirT, 0, 2);  // Stop turret
    }
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
    // Normalize joystick values to -1 to 1 range
    // float forward = ly;  
    // float turn = rx;

    // // HTTML Joystick control
    // float forward = joyY;  // Forward/backward control from joystick
    // float turn = joyX;     // Left/right control from joystick

    bool useUdp = (millis() - lastUdpTime < 100);
    float forward = useUdp ? ly : joyY;
    float turn = useUdp ? rx : joyX;

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

    Serial.printf("L: %5.1f RPM | R: %5.1f RPM | Error_Turret: %5.1f  | PWM: L=%d, R=%d, T=%d | Rot: L=%.2f, R=%.2f, T_angle=%.2f\n", 
              rpmL, rpmR, errorT, (int)pwmL_out, (int)pwmR_out, (int)pwmT_out, rotL, rotR, currentAngleT);

  }
  if(now- lastOdometryTime >= ODOMETRY_INTERVAL) {
    // Update odometry every ODOMETRY_INTERVAL ms
    updateOdometry();

    static unsigned long lastDetailedPrint = 0;
    if (now - lastDetailedPrint >= 2000) { // Print every 2-second
      Serial.println("\n PROBABILISTIC ODOM ESTIMATION:");
      printPose();
      printMotionModel();

      static unsigned long lastCovPrint = 0;
      if (now - lastCovPrint >= 5000) { // Print covariance every 5 seconds
        printCovariance();
        lastCovPrint = now;
      }

      float sample_x, sample_y, sample_theta;
      samplePose(sample_x, sample_y, sample_theta); 
      Serial.printf("Sampled Pose: X=%.2f, Y=%.2f, Theta=%.2f\n", sample_x, sample_y, sample_theta * 180.0 / PI);
      Serial.println("--------------------------------------------------");
      lastDetailedPrint = now;
    }
    lastOdometryTime = now;
  }
}



