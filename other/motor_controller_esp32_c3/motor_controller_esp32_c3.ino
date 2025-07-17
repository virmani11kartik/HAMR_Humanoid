#define MOTOR_FWD 5       // Pin for the potentiometer (ADC1_4)
#define MOTOR_BWD 6        // Pin for the LED to control its brightness
#define PWM_PIN 1  // PWM channel for the LED
#define LEDC_PWM_CHANNEL 0  // PWM channel for the LEDC

#include <WiFi.h>
#include "body.h"
#include "html510.h"
HTML510Server h(80);

// WiFi credentials
const char* ssid = "ESP32_Cedric_AP";
const char* password = "easy";

// Global variables for direction and duty cycle
int dutyCycle = 50; // Default Duty cycle
int mapDutyCycle; // map % of duty cycle from slider to pwm based on res bits
int direction; // rotating direction

IPAddress myIp(192, 168, 1, 134); // IP address for Cedric Hollande

void handleRoot(){
  h.sendhtml(body);
}

void handleDirectionSlider() {
  // Get the direction value from the slider (-1 to 1)
  direction = h.getVal();  

  // Res Bits based on frequency
  if (direction == -1) { // direction -1: rotate bwd
    digitalWrite(MOTOR_BWD, HIGH);
    digitalWrite(MOTOR_FWD, LOW);
  } else if (direction == 0) { // direction 0: no rotation
    digitalWrite(MOTOR_BWD, LOW);
    digitalWrite(MOTOR_FWD, LOW);
  } else if (direction == 1) { // direction 1: rotate fwd
    digitalWrite(MOTOR_BWD, LOW);
    digitalWrite(MOTOR_FWD, HIGH);
  }
}

void handleDutySlider() {
  // Get the duty cycle value from the slider
  dutyCycle = h.getVal();
  // Update PWM duty cycle
  mapDutyCycle = map(dutyCycle, 0, 100, 0, 2047); // Map 0-100% to 0-2047 (11-bit resolution for PWM)
  ledcWrite(LEDC_PWM_CHANNEL, mapDutyCycle);
}

void setup() {
  // Initialize serial communication at 115200 bits per second:
  Serial.begin(115200);

  // Setting up the ESP as an AP (Access Point)
  WiFi.mode(WIFI_MODE_AP);
  Serial.println("Setting AP…");
  WiFi.softAP(ssid, password);
  WiFi.softAPConfig(myIp, IPAddress(192, 168, 1, 2), IPAddress(255, 255, 255, 0));

  Serial.print("AP IP address: ");
  Serial.println(myIp);

  // Initialize HTML510Server
  h.begin();
  h.attachHandler("/", handleRoot);
  h.attachHandler("/set?type=direction&value=", handleDirectionSlider); // Attach handler for direction
  h.attachHandler("/set?type=duty&value=", handleDutySlider); // Attach handler for duty cycle

  // Set up the FWD, BWD and PWM
  pinMode(PWM_PIN, OUTPUT);
  pinMode(MOTOR_FWD, OUTPUT);
  pinMode(MOTOR_BWD, OUTPUT);

  ledcSetup(LEDC_PWM_CHANNEL, 30, 11); // 11-bit resolution, 30 Hz frequency
  ledcAttachPin(PWM_PIN, LEDC_PWM_CHANNEL);
}

void loop() {
  h.serve();
  delay(10);
}