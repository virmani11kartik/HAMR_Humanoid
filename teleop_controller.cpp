// #include <Arduino.h>
// #include <esp32-hal.h>
// #include <esp32-hal-gpio.h>
// #include <esp32-hal-ledc.h>

// float lx, ly, rx, ry, lt, rt;
// int a, b, x, y;

// void setup() {
//   Serial.begin(115200);
//   Serial.println("ESP32 Ready");
// }

// void loop() {
//   if (Serial.available()) {
//     String msg = Serial.readStringUntil('\n');

//     // Basic parsing example — you might want to improve parsing robustness
//     int lxIndex = msg.indexOf("LX:");
//     int lyIndex = msg.indexOf("LY:");
//     int rxIndex = msg.indexOf("RX:");
//     int ryIndex = msg.indexOf("RY:");
//     int ltIndex = msg.indexOf("LT:");
//     int rtIndex = msg.indexOf("RT:");
//     int aIndex  = msg.indexOf("A:");
//     int bIndex  = msg.indexOf("B:");
//     int xIndex  = msg.indexOf("X:");
//     int yIndex  = msg.indexOf("Y:");

//     if (lxIndex == -1 || lyIndex == -1 || rxIndex == -1 || ryIndex == -1 ||
//         ltIndex == -1 || rtIndex == -1 || aIndex == -1 || bIndex == -1 ||
//         xIndex == -1 || yIndex == -1) {
//       Serial.println("ESP: Parsing error!");
//       return;
//     }

//     // lx = msg.substring(lxIndex + 3, lyIndex).toFloat();
//     ly = msg.substring(lyIndex + 3, rxIndex).toFloat();
//     rx = msg.substring(rxIndex + 3, ryIndex).toFloat();
//     // ry = msg.substring(ryIndex + 3, ltIndex).toFloat();
//     lt = msg.substring(ltIndex + 3, rtIndex).toFloat();
//     rt = msg.substring(rtIndex + 3, aIndex).toFloat();

//     a = msg.substring(aIndex + 2, bIndex).toInt();
//     // b = msg.substring(bIndex + 2, xIndex).toInt();
//     // x = msg.substring(xIndex + 2, yIndex).toInt();
//     // y = msg.substring(yIndex + 2).toInt();

//     Serial.printf("ESP:  LY: %.2f RX: %.2f LT: %.2f RT: %.2f A: %d \n", ly, rx, lt, rt, a);

//     // Serial.printf("ESP: LX: %.2f LY: %.2f RX: %.2f RY: %.2f LT: %.2f RT: %.2f A: %d B: %d X: %d Y: %d\n",
//     //               lx, ly, rx, ry, lt, rt, a, b, x, y);
//   }
// }


#include <Arduino.h>
#include <USB.h>
#include <USBHIDGamepad.h>

USBHIDGamepad Gamepad;

void setup() {
  Serial.begin(115200);
  USB.begin();
  Serial.println("ESP32 OTG Gamepad Reader Ready");
}

void loop() {
  if (Gamepad.connected()) {
    // Read axes
    float lx = Gamepad.getAxis(0) / 32767.0;  // Left X
    float ly = Gamepad.getAxis(1) / 32767.0;  // Left Y
    float rx = Gamepad.getAxis(2) / 32767.0;  // Right X
    float ry = Gamepad.getAxis(3) / 32767.0;  // Right Y
    float lt = Gamepad.getAxis(4) / 32767.0;  // Left Trigger
    float rt = Gamepad.getAxis(5) / 32767.0;  // Right Trigger

    // Read buttons
    int a = Gamepad.getButton(0);  // A button
    int b = Gamepad.getButton(1);  // B button
    int x = Gamepad.getButton(2);  // X button
    int y = Gamepad.getButton(3);  // Y button

    Serial.printf("ESP: LX: %.2f LY: %.2f RX: %.2f RY: %.2f LT: %.2f RT: %.2f A:%d B:%d X:%d Y:%d\n",
                  lx, ly, rx, ry, lt, rt, a, b, x, y);
    delay(50);  // Limit refresh rate
  } else {
    Serial.println("Waiting for gamepad connection...");
    delay(500);
  }
}

#include <Arduino.h>
#include "USB.h"
#include "USBHIDHost.h"

USBHIDHost hid;
USBHIDInputJoystick* joystick = nullptr;

void setup() {
  Serial.begin(115200);
  USB.begin();
  Serial.println("ESP32-S2 USB Host Initialized");
}

void loop() {
  USBHIDInputJoystick* newJoy = hid.getJoystick();
  if (newJoy && newJoy != joystick) {
    joystick = newJoy;
    Serial.println("Gamepad connected!");
  }

  if (joystick) {
    int16_t lx = joystick->axisX();
    int16_t ly = joystick->axisY();
    int16_t rx = joystick->axisZ();
    int16_t ry = joystick->rz();

    uint16_t buttons = joystick->buttons();

    Serial.printf("LX: %d  LY: %d  RX: %d  RY: %d  Buttons: 0x%04X\n", lx, ly, rx, ry, buttons);
    delay(50);
  } else {
    delay(100);
  }
}