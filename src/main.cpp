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

// Motor Driver Pins
const int pwmPin = 7;   // PWM speed control
const int dirPin = 6;   // Direction control

// Encoder Pins
const int encoderPinA = 5;
const int encoderPinB = 4;

volatile long encoderTicks = 0;  // Total encoder ticks

// Gear and encoder constants
const int TICKS_PER_REV = 2704;
const float DEGREES_PER_TICK = 360.0 / TICKS_PER_REV;

const float motorGearTeeth = 41.0;
const float outputGearTeeth = 136.5; // Adjusted for 1.5:1 gear ratio
const float gearRatio = outputGearTeeth / motorGearTeeth;

// Target
const float inputAngle = 90.0;  // Desired output angle in degrees
const float targetAngle = inputAngle * gearRatio;  // Motor shaft angle to reach that output

// PID variables
float Kp = 12.0;
float Ki = 0.8;
float Kd = 0.5;

float prevError = 0;
float integral = 0;

unsigned long lastTime = 0;

// Limits
const int maxPWM = 4095;
const int minPWM = 800;  // Minimum speed to overcome static friction

void IRAM_ATTR handleEncoderA() {
  bool A = digitalRead(encoderPinA);
  bool B = digitalRead(encoderPinB);
  encoderTicks += (A == B) ? 1 : -1;
}

void setup() {
  Serial.begin(115200);

  pinMode(pwmPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderPinA), handleEncoderA, CHANGE);

  ledcAttachPin(pwmPin, 0);           // Channel 0
  ledcSetup(0, 5000, 12);             // 5kHz, 12-bit resolution

  encoderTicks = 0;
  lastTime = millis();
}

void loop() {
  float currentAngle = encoderTicks * DEGREES_PER_TICK;
  float error = targetAngle - currentAngle;

  // Stop if very close
  if (abs(error) < 1.0) {
    ledcWrite(0, 0);  // Stop motor
    Serial.println("Target reached!");
    Serial.print("Final angle: ");
    Serial.println(currentAngle);
    while (true); // Stop
  }

  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;  // Time in seconds

  if (dt == 0) dt = 0.001; // Avoid division by zero

  integral += error * dt;
  float derivative = (error - prevError) / dt;

  float output = Kp * error + Ki * integral + Kd * derivative;

  // Set direction
  if (output > 0) {
    digitalWrite(dirPin, HIGH);
  } else {
    digitalWrite(dirPin, LOW);
    output = -output;  // Make PWM value positive
  }

  // Constrain PWM
  int pwmValue = constrain((int)output, minPWM, maxPWM);
  ledcWrite(0, pwmValue);

  // Debug
  Serial.print("Angle: ");
  Serial.print(currentAngle);
  Serial.print("  Error: ");
  Serial.print(error);
  Serial.print("  PWM: ");
  Serial.println(pwmValue);

  prevError = error;
  lastTime = now;
  delay(10);
}


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

// // float lx, ly, rx, ry, lt, rt;
// int a, b, x, y;

// const char* webpage = R"rawliteral(
// <!DOCTYPE html>
// <html>
// <head>
//   <meta name="viewport" content="width=device-width, initial-scale=1">
//   <title>HAMR Joystick</title>
//   <style>
//     body { font-family: sans-serif; text-align: center; background: #f0f0f0; }
//     #joystickZone { width: 200px; height: 200px; margin: auto; margin-top: 50px; background: #ccc; border-radius: 50%; position: relative; touch-action: none; }
//     #stick { width: 50px; height: 50px; background: #555; border-radius: 50%; position: absolute; top: 75px; left: 75px; }
//   </style>
// </head>
// <body>
//   <h2>HAMR Virtual Joystick</h2>
//   <div id="joystickZone">
//     <div id="stick"></div>
//   </div>
//   <script>
//     const stick = document.getElementById('stick');
//     const zone = document.getElementById('joystickZone');

//     let dragging = false;

//     zone.addEventListener('touchstart', startDrag);
//     zone.addEventListener('touchmove', drag);
//     zone.addEventListener('touchend', endDrag);
//     zone.addEventListener('mousedown', startDrag);
//     zone.addEventListener('mousemove', drag);
//     zone.addEventListener('mouseup', endDrag);

//     function startDrag(e) {
//       dragging = true;
//     }

//     function drag(e) {
//       if (!dragging) return;
//       e.preventDefault();

//       let x = (e.touches ? e.touches[0].clientX : e.clientX) - zone.getBoundingClientRect().left;
//       let y = (e.touches ? e.touches[0].clientY : e.clientY) - zone.getBoundingClientRect().top;

//       let dx = x - 100;
//       let dy = y - 100;

//       let dist = Math.min(Math.sqrt(dx*dx + dy*dy), 75);
//       let angle = Math.atan2(dy, dx);

//       let stickX = Math.cos(angle) * dist + 75;
//       let stickY = Math.sin(angle) * dist + 75;

//       stick.style.left = stickX + 'px';
//       stick.style.top = stickY + 'px';

//       // Normalize and send
//       let normX = ((stickX - 75) / 75).toFixed(2);
//       let normY = ((stickY - 75) / 75).toFixed(2);

//       fetch(`/move?x=${normX}&y=${normY}`);
//     }

//     function endDrag() {
//       dragging = false;
//       stick.style.left = '75px';
//       stick.style.top = '75px';
//       fetch(`/move?x=0&y=0`);
//     }
//   </script>
// </body>
// </html>
// )rawliteral";

// WebServer server(80);

// // WIFI CREDENTIALS
// const char* ssid = "HAMR";
// const char* password = "123571113";

// // UDP SETUP
// WiFiUDP udp;
// const int port = 12345;  // Port to listen on
// char incoming[255];  // Buffer for incoming data

// // Motor Pins
// const int pwmL = 11;
// const int dirL = 12;
// const int encAL = 14;
// const int encBL = 13;

// const int pwmR = 41;
// const int dirR = 42;
// const int encAR = 1;
// const int encBR = 2;

// // Encoder counts (volatile for ISR)
// volatile long ticksL = 0;
// volatile long ticksR = 0;

// // PID constants for synchronization
// const float Kp_sync = 0.0;
// const float Ki_sync = 0.0;
// const float Kd_sync = 0.0;

// // Encoder & motor specs
// const int CPR = 64;
// const int GEAR_RATIO = 150;
// const int TICKS_PER_WHEEL_REV = CPR * GEAR_RATIO; // 9600 ticks per wheel revolution

// // Control interval (ms)
// const unsigned long PID_INTERVAL = 50;
// static unsigned long lastUdpTime = 0;

// // PID state variables
// float integralSync = 0;
// float lastErrorSync = 0;

// // Timing variables
// unsigned long lastPidTime = 0;
// long lastTicksL = 0;
// long lastTicksR = 0;

// // Base PWM speed (0-4095)
// float basePWM = 4095;

// // Joystick control variables
// float ly = 0.0f;  // left stick vertical (forward/back)
// float rx = 0.0f;  // right stick horizontal (turn)
// // HTTML Joystick control variables
// float joyX = 0.0f;  // Joystick X-axis
// float joyY = 0.0f;  // Joystick Y-axis

// // Current command
// // char command = 'f'; // start forward

// // Encoder interrupts (quadrature decoding)
// void IRAM_ATTR handleEncL() {
//   bool A = digitalRead(encAL);
//   bool B = digitalRead(encBL);
//   ticksL += (A == B) ? 1 : -1;
// }

// void IRAM_ATTR handleEncR() {
//   bool A = digitalRead(encAR);
//   bool B = digitalRead(encBR);
//   ticksR += (A == B) ? -1 : 1;  // Inverted to fix direction issue if needed
// }

// // Set motor PWM and direction
// void setMotor(int pwmPin, int dirPin, float pwmVal, int channel) {
//   pwmVal = constrain(pwmVal, -4095, 4095);
//   if (pwmVal >= 0) {
//     digitalWrite(dirPin, HIGH);
//     ledcWrite(channel, (int)pwmVal);
//   } else {
//     digitalWrite(dirPin, LOW);
//     ledcWrite(channel, (int)(-pwmVal));
//   }
// }

// void setup() {

//   Serial.begin(115200);
//   Serial.println("ESP32 Ready");
//   WiFi.softAP(ssid, password, 6, 0, 2);
//   IPAddress myIP = WiFi.softAPIP();
//   Serial.print("ESP IP: ");
//   Serial.println(myIP);

//   server.on("/", HTTP_GET, []() {
//     server.send(200, "text/html", webpage);  // Serve the HTML page
//   });
//   server.on("/move", HTTP_GET, []() {
//     String xVal = server.arg("x");
//     String yVal = server.arg("y");
//     joyX= xVal.toFloat();
//     joyY= -yVal.toFloat();
//     Serial.printf("Joystick X: %.2f, Y: %.2f\n", joyX, joyY);
//     server.send(200, "text/plain", "OK");
//   });
//   server.onNotFound([]() {
//     server.send(404, "text/plain", "404 Not Found");
//   });
//   server.begin();
//   Serial.println("HTTP server started");
//   udp.begin(port);
//   Serial.printf("Listening for UDP on port %d\n", port);

//   // Motor pins
//   pinMode(pwmL, OUTPUT);
//   pinMode(dirL, OUTPUT);
//   pinMode(pwmR, OUTPUT);
//   pinMode(dirR, OUTPUT);

//   // Setup PWM channels at 5 kHz, 12-bit resolution
//   ledcSetup(0, 5000, 12);
//   ledcSetup(1, 5000, 12);
//   ledcAttachPin(pwmL, 0);
//   ledcAttachPin(pwmR, 1);

//   // Encoder pins with pull-ups
//   pinMode(encAL, INPUT_PULLUP);
//   pinMode(encBL, INPUT_PULLUP);
//   pinMode(encAR, INPUT_PULLUP);
//   pinMode(encBR, INPUT_PULLUP);

//   // Attach interrupts on channel A for both encoders
//   attachInterrupt(digitalPinToInterrupt(encAL), handleEncL, CHANGE);
//   attachInterrupt(digitalPinToInterrupt(encAR), handleEncR, CHANGE);

//   lastPidTime = millis();

//   Serial.println("Motor Sync Control Ready");
// //   Serial.println("Commands: f=forward, b=backward, r=right, l=left, s=stop, +=faster, -=slower");
//   Serial.println("Send joystick data like: LX:0.00 LY:0.00 RX:0.00 RY:0.00 LT:0.00 RT:0.00 A:0 B:0 X:0 Y:0");
//   Serial.printf("Initial Speed PWM: %.0f\n", basePWM);
// }

// void loop() {
//   // Handle serial commands
// //   if (Serial.available()) {
// //     char newCommand = Serial.read();
// //     if (newCommand == 'f' || newCommand == 'b' || newCommand == 'r' || newCommand == 'l' || newCommand == 's') {
// //       command = newCommand;
// //       integralSync = 0;
// //       lastErrorSync = 0;
// //       Serial.printf("Command: %c\n", command);
// //     } else if (newCommand == '+') {
// //       basePWM += 10;
// //       if (basePWM > 4095) basePWM = 4095;
// //       Serial.printf("Speed increased: %d\n", (int)basePWM);
// //     } else if (newCommand == '-') {
// //       basePWM -= 10;
// //       if (basePWM < 0) basePWM = 0;
// //       Serial.printf("Speed decreased: %d\n", (int)basePWM);
// //     }
// //   }
//     // Read joystick data from serial
//     // if (Serial.available()) {
//     // String msg = Serial.readStringUntil('\n');

//     server.handleClient(); // Handle HTTP requests
//     int len = udp.parsePacket();
//     if (len > 0) {
//       udp.read(incoming, sizeof(incoming));
//       incoming[len] = '\0';  // null-terminate
//       // Serial.printf("Received: %s\n", incoming);
//       String msg = String(incoming); 

//     int lyIndex = msg.indexOf("LY:");
//     int rxIndex = msg.indexOf("RX:");

//     if (lyIndex != -1 && rxIndex != -1) {
//       // Extract LY and RX values as floats
//       // Find next space or end of line after LY:
//       int lyEnd = msg.indexOf(' ', lyIndex);
//       if (lyEnd == -1) lyEnd = msg.length();
//       int rxEnd = msg.indexOf(' ', rxIndex);
//       if (rxEnd == -1) rxEnd = msg.length();

//       String lyStr = msg.substring(lyIndex + 3, lyEnd);
//       String rxStr = msg.substring(rxIndex + 3, rxEnd);

//       ly = lyStr.toFloat();
//       rx = rxStr.toFloat();
//       lastUdpTime = millis();
//         }
//     }

//   // PID loop timing
//   unsigned long now = millis();
//   if (now - lastPidTime >= PID_INTERVAL) {
//     float dt = (now - lastPidTime) / 1000.0;

//     // Read encoder counts atomically
//     noInterrupts();
//     long currentTicksL = ticksL;
//     long currentTicksR = ticksR;
//     interrupts();

//     // Calculate RPM for each motor
//     float rpmL = ((currentTicksL - lastTicksL) / dt) * 60.0 / TICKS_PER_WHEEL_REV;
//     float rpmR = ((currentTicksR - lastTicksR) / dt) * 60.0 / TICKS_PER_WHEEL_REV;

//     lastTicksL = currentTicksL;
//     lastTicksR = currentTicksR;
//     lastPidTime = now;

//     // Calculate error between motors (left RPM minus right RPM)
//     float errorSync = rpmL - rpmR;

//     integralSync += errorSync * dt;
//     integralSync = constrain(integralSync, -100, 100);
//     float dErrorSync = (errorSync - lastErrorSync) / dt;

//     float correctionPWM = Kp_sync * errorSync + Ki_sync * integralSync + Kd_sync * dErrorSync;
//     lastErrorSync = errorSync;

//     // Calculate base PWM values per motor based on command
//     // float pwmL_base = 0, pwmR_base = 0;
//     // switch (command) {
//     //   case 'f': // forward both motors
//     //     pwmL_base = basePWM;
//     //     pwmR_base = basePWM;
//     //     break;
//     //   case 'b': // backward both motors
//     //     pwmL_base = -basePWM;
//     //     pwmR_base = -basePWM;
//     //     break;
//     //   case 'r': // right turn: left forward, right backward
//     //     pwmL_base = basePWM;
//     //     pwmR_base = -basePWM;
//     //     break;
//     //   case 'l': // left turn: left backward, right forward
//     //     pwmL_base = -basePWM;
//     //     pwmR_base = basePWM;
//     //     break;
//     //   case 's': // stop
//     //   default:
//     //     pwmL_base = 0;
//     //     pwmR_base = 0;
//     //     break;
//     // }

//     // Joystick-based differential drive control:
//     // Negative ly because joystick up may be negative, adjust if needed
//     // Normalize joystick values to -1 to 1 range
//     float forward = ly;  
//     float turn = rx;

//     // HTTML Joystick control
//     // float forward = joyY;  // Forward/backward control from joystick
//     // float turn = joyX;     // Left/right control from joystick

//     // bool useUdp = (millis() - lastUdpTime < 100);
//     // float forward = useUdp ? ly : joyY;
//     // float turn = useUdp ? rx : joyX;

//     // Combine for left and right motor base PWM
//     float pwmL_base = (forward + turn) * basePWM;
//     float pwmR_base = (forward - turn) * basePWM;

//     // Apply PID correction only to left motor PWM to sync speeds
//     float pwmL_out = constrain(pwmL_base - correctionPWM, -4095, 4095);
//     float pwmR_out = constrain(pwmR_base, -4095, 4095);

//     // Set motor speeds
//     setMotor(pwmL, dirL, pwmL_out, 0);
//     setMotor(pwmR, dirR, pwmR_out, 1);
//     // Calculate rotations
//     float rotL = currentTicksL / (float)TICKS_PER_WHEEL_REV;
//     float rotR = currentTicksR / (float)TICKS_PER_WHEEL_REV;

//     // Debug output
//     // Serial.printf("L: %5.1f RPM | R: %5.1f RPM | PWM: L=%4.0f, R=%4.0f | Rot: L=%.2f, R=%.2f\n",
//     //               rpmL, rpmR, pwmL_out, pwmR_out, rotL, rotR);

//     Serial.printf("L: %5.1f RPM | R: %5.1f RPM | PWM: L=%d, R=%d | Rot: L=%.2f, R=%.2f\n",
//               rpmL, rpmR, (int)pwmL_out, (int)pwmR_out, rotL, rotR);

//     // Print synchronization status
//     // Serial.printf("Cmd:%c Speed:%d L_RPM:%.1f R_RPM:%.1f Corr:%.1f PWM_L:%.0f PWM_R:%.0f\n",
//     //               command, (int)basePWM, rpmL, rpmR, correctionPWM, pwmL_out, pwmR_out);
//   }
// }



