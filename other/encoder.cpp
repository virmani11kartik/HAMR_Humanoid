#include <Arduino.h>
#include <esp32-hal.h>
#include <esp32-hal-gpio.h>
#include <esp32-hal-ledc.h>
// Encoder example for ESP32
// This example reads an incremental rotary encoder and prints its position to the Serial Monitor.

// Define encoder pins
const int encoderPinA = 14; // Channel A
const int encoderPinB = 13; // Channel B

volatile int encoderPosition = 0;  // Position counter
volatile bool lastEncoded = 0;

void IRAM_ATTR handleEncoder() {
  bool A = digitalRead(encoderPinA);
  bool B = digitalRead(encoderPinB);

  if (A == B) {
    encoderPosition++;
  } else {
    encoderPosition--;
  }
}

void setup() {
  Serial.begin(115200);

  // Set encoder pins as input
  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);

  // Attach interrupt to channel A
  attachInterrupt(digitalPinToInterrupt(encoderPinA), handleEncoder, CHANGE);
}

void loop() {
  static int lastPosition = 0;

  if (lastPosition != encoderPosition) {
    Serial.print("Position: ");
    Serial.println(encoderPosition);
    lastPosition = encoderPosition;
  }

  delay(10); // Reduce serial spam
}
