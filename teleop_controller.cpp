#include <Arduino.h>
#include <esp32-hal.h>
#include <esp32-hal-gpio.h>
#include <esp32-hal-ledc.h>

float lx, ly, rx, ry, lt, rt;
int a, b, x, y;

void setup() {
  Serial.begin(115200);
  Serial.println("ESP32 Ready");
}

void loop() {
  if (Serial.available()) {
    String msg = Serial.readStringUntil('\n');

    // Basic parsing example — you might want to improve parsing robustness
    int lxIndex = msg.indexOf("LX:");
    int lyIndex = msg.indexOf("LY:");
    int rxIndex = msg.indexOf("RX:");
    int ryIndex = msg.indexOf("RY:");
    int ltIndex = msg.indexOf("LT:");
    int rtIndex = msg.indexOf("RT:");
    int aIndex  = msg.indexOf("A:");
    int bIndex  = msg.indexOf("B:");
    int xIndex  = msg.indexOf("X:");
    int yIndex  = msg.indexOf("Y:");

    if (lxIndex == -1 || lyIndex == -1 || rxIndex == -1 || ryIndex == -1 ||
        ltIndex == -1 || rtIndex == -1 || aIndex == -1 || bIndex == -1 ||
        xIndex == -1 || yIndex == -1) {
      Serial.println("ESP: Parsing error!");
      return;
    }

    lx = msg.substring(lxIndex + 3, lyIndex).toFloat();
    ly = msg.substring(lyIndex + 3, rxIndex).toFloat();
    rx = msg.substring(rxIndex + 3, ryIndex).toFloat();
    ry = msg.substring(ryIndex + 3, ltIndex).toFloat();
    lt = msg.substring(ltIndex + 3, rtIndex).toFloat();
    rt = msg.substring(rtIndex + 3, aIndex).toFloat();

    a = msg.substring(aIndex + 2, bIndex).toInt();
    b = msg.substring(bIndex + 2, xIndex).toInt();
    x = msg.substring(xIndex + 2, yIndex).toInt();
    y = msg.substring(yIndex + 2).toInt();

    Serial.printf("ESP: LX: %.2f LY: %.2f RX: %.2f RY: %.2f LT: %.2f RT: %.2f A: %d B: %d X: %d Y: %d\n",
                  lx, ly, rx, ry, lt, rt, a, b, x, y);
  }
}