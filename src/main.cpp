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
#include <Adafruit_BNO08x.h>
#include <string.h>
#include "pid_webpage.h"
#include "imu_55.h"
#include "imu_85.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <Adafruit_VL53L0X.h>

WebServer server(80);

// WIFI CREDENTIALS
const char* ssid = "HAMR";
const char* password = "123571113";

//------------IMU OBJECT--------------
IMU55 sens(33,34);                 // SDA=4, SCL=5, addr=0x28 by default
float roll_b,pitch_b,yaw_b;
// ===== IMU ↔ control =====
volatile float    g_yaw_latest = 0.0f;      // last IMU yaw (wrapped)
volatile uint32_t g_yaw_latest_us = 0;      // micros() when last sample arrived
volatile float g_roll_latest = 0.0f;
volatile float g_pitch_latest = 0.0f;
volatile bool     g_yaw_valid = false;      // becomes true after 1st good sample
static uint32_t   g_yaw_last_used_us = 0;   // EKF consumer: last stamp consumed
volatile float g_ax_latest = 0.0f, g_ay_latest = 0.0f, g_az_latest = 0.0f;
volatile float g_gx_latest = 0.0f, g_gy_latest = 0.0f, g_gz_latest = 0.0f;
portMUX_TYPE g_imuMux = portMUX_INITIALIZER_UNLOCKED;

// TOF OBJECTS
#define TOF_SDA_PIN 36
#define TOF_SCL_PIN 35
#define XSHUT_1 37
#define XSHUT_2 38
#define XSHUT_3 39
#define XSHUT_4 40
#define ADDR_1 0x30
#define ADDR_2 0x31
#define ADDR_3 0x32
#define ADDR_4 0x33

Adafruit_VL53L0X tof1;
Adafruit_VL53L0X tof2;
Adafruit_VL53L0X tof3;
Adafruit_VL53L0X tof4;

// ===== TOF ↔ control =====
volatile int      g_tof_mm[4]       = {99999,99999,99999,99999};
volatile uint32_t g_tof_stamp_us[4] = {0,0,0,0};
volatile bool     g_tof_valid[4]    = {false,false,false,false};
portMUX_TYPE      g_tofMux = portMUX_INITIALIZER_UNLOCKED;
TwoWire WireTof = TwoWire(1);
SemaphoreHandle_t i2cMutex = NULL;
constexpr uint32_t TOF_FRESHNESS_US = 250000;  // 150 ms max age for using a reading

//---------------------------GLOBALS----------------------
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
const unsigned long PID_INTERVAL = 10;
static unsigned long lastUdpTime = 0;
static unsigned long lastSerialCtlTime = 0;

// PID state variables
float integralL = 0.0f, integralR = 0.0f;
float lastErrL = 0.0f, lastErrR = 0.0f;
float errorT = 0;

// Timing variables
unsigned long lastPidTime = 0;
long lastTicksL = 0;
long lastTicksR = 0;
long lastTicksT = 0;

// Base PWM speed (0-4095)
float basePWM = 3500;
const float MAX_RPM_CMD = 28.0f;
float pwmT_out = 0;
float scaleFactor = 1.0; //131.67;
// float turretSpeed = 0.0;

float test = 0.0f;

// TOF Emergency Stop Variables
const int TOF_STOP_THRESH  = 200;  // stop if any sensor <= this
const int TOF_CLEAR_THRESH = 240;  // resume only when all >= this (hysteresis)
bool tof_stop_latched = false;

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

// ----------------- UART Protocol -----------------
static const uint16_t MAGIC = 0xCAFE;
static const uint16_t VER   = 1;
static const uint16_t TYPE_CMD  = 0x0001; // PC->ESP : left,right
static const uint16_t TYPE_CMD3 = 0x0011; // PC->ESP : left,right,turret
static const uint16_t TYPE_ENC  = 0x0003; // ESP->PC : encoders
static const uint16_t TYPE_IMU = 0x0004; // ESP->PC : raw IMU data

// Latest commands received over UART (ROS)
volatile float uart_left_cmd = 0.0f;
volatile float uart_right_cmd = 0.0f;
volatile float uart_turret_cmd = 0.0f;
volatile uint32_t last_uart_cmd_ms = 0;
// Enc packet sequence
static uint32_t enc_seq = 0;
// IMU packet sequence
static uint32_t imu_seq = 0;
#pragma pack(push,1)

struct CmdPacket {
  uint16_t magic, ver, type;
  uint32_t seq;
  uint64_t t_tx_ns;
  float left, right;
  uint16_t crc16;
};
struct Cmd3Packet {
  uint16_t magic, ver, type;
  uint32_t seq;
  uint64_t t_tx_ns;
  float left, right, turret;
  uint16_t crc16;
};
struct EncPacket {
  uint16_t magic, ver, type;
  uint32_t seq;
  uint64_t t_tx_ns;
  int32_t ticksL, ticksR, ticksT;
  uint16_t crc16;
};
struct IMUPacket {
  uint16_t magic, ver, type;
  uint32_t seq;
  uint64_t t_tx_ns;
  float roll, pitch, yaw; // radians (BNO055 Euler, wrapped to ±π)
  float ax, ay, az;       // m/s² linear acceleration (gravity removed)
  float gx, gy, gz;       // rad/s angular velocity
  uint16_t crc16;
};

#pragma pack(pop)

static const size_t CMD_SIZE  = sizeof(CmdPacket);   // 2-float
static const size_t CMD3_SIZE = sizeof(Cmd3Packet);  // 3-float
static const size_t ENC_SIZE  = sizeof(EncPacket);
static const size_t IMU_SIZE = sizeof(IMUPacket);

// CRC32->16 surrogate (must match Pi side)
uint16_t crc16_surrogate(const uint8_t* data, size_t n) {
  uint32_t c = 0xFFFFFFFFu;
  for (size_t i=0;i<n;i++) {
    c ^= data[i];
    for (int k=0;k<8;k++) {
      c = (c & 1) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
    }
  }
  c ^= 0xFFFFFFFFu;
  return (uint16_t)(c & 0xFFFF);
}


void transmitIMUData()
{
  float r, p, y, ax, ay, az, gx, gy, gz;
  taskENTER_CRITICAL(&g_imuMux);
  r  = g_roll_latest;  p  = g_pitch_latest; y  = g_yaw_latest;
  ax = g_ax_latest;    ay = g_ay_latest;    az = g_az_latest;
  gx = g_gx_latest;    gy = g_gy_latest;    gz = g_gz_latest;
  taskEXIT_CRITICAL(&g_imuMux);

  IMUPacket pkt;
  pkt.magic = MAGIC;
  pkt.ver   = VER;
  pkt.type  = TYPE_IMU;
  pkt.seq   = ++imu_seq;
  pkt.t_tx_ns = (uint64_t)micros() * 1000ull;
  pkt.roll = r; pkt.pitch = p; pkt.yaw = y;
  pkt.ax = ax;  pkt.ay = ay;  pkt.az = az;
  pkt.gx = gx;  pkt.gy = gy;  pkt.gz = gz;
  pkt.crc16 = crc16_surrogate((uint8_t*)&pkt, IMU_SIZE - 2);
  Serial0.write((uint8_t*)&pkt, IMU_SIZE);
}


// ------------- Units & conversion -------------
constexpr float WHEEL_RADIUS_M = 0.0762f;      // your wheel radius
constexpr float MAX_WHEEL_RPM  = 30.0f;       // safety clamp (tune)
enum WheelCmdUnits { CMD_MPS, CMD_RAD_PER_S, CMD_RPM };
constexpr WheelCmdUnits CMD_UNITS = CMD_RAD_PER_S;   // ROS cmd units

inline float wheel_rpm_from_cmd(float v) {
  switch (CMD_UNITS) {
    case CMD_MPS:      return v * 60.0f / (2.0f * (float)M_PI * WHEEL_RADIUS_M);
    case CMD_RAD_PER_S:return v * 60.0f / (2.0f * (float)M_PI);
    case CMD_RPM:
    default:           return v;
  }
}
template<typename T> inline T clamp(T x, T lo, T hi) {
  return x < lo ? lo : (x > hi ? hi : x);
}

// ------------------------ ENCODERS------------------
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

//-----------------------------SET MOTORS---------------------
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

//---------------------------IMU FREE RTOS TASK---------------------
void imu_task(void*){
  if (!sens.begin()) {
    Serial.println("IMU init failed");
    for(;;){ vTaskDelay(pdMS_TO_TICKS(1000)); }
  }
  // sens.setMinCalibrationLevel(2);
  // sens.setDataTimeout(5);  // <= 5–10 ms, NOT 1000 ms
  const TickType_t period = pdMS_TO_TICKS(5);  // ~200 Hz poll cadence
  TickType_t last = xTaskGetTickCount();
  for(;;){
      sens.update();
      float r, p, y, ax, ay, az, gx, gy, gz;
      sens.getRPY(r, p, y);
      sens.getLinearAccel(ax, ay, az);
      sens.getAngularVel(gx, gy, gz);
      taskENTER_CRITICAL(&g_imuMux);
      g_roll_latest   = r;
      g_pitch_latest  = p;
      g_yaw_latest    = y;
      g_yaw_latest_us = micros();
      g_yaw_valid     = true;
      g_ax_latest = ax; g_ay_latest = ay; g_az_latest = az;
      g_gx_latest = gx; g_gy_latest = gy; g_gz_latest = gz;
      taskEXIT_CRITICAL(&g_imuMux);

      static uint32_t last_print_ms = 0;
      // print statements
      if (millis() - last_print_ms >= 200) {
        last_print_ms = millis();
        Serial.printf("RPY: %.2f %.2f %.2f | Accel: %.3f %.3f %.3f | Gyro: %.3f %.3f %.3f\n",
          r, p, y, ax, ay, az, gx, gy, gz);
      }

    vTaskDelayUntil(&last, period);
  }
}

//---------------------------TOF FREE RTOS TASK-------------------------------

bool initOneTof(Adafruit_VL53L0X &lox, int xshutPin, uint8_t newAddr) {
  pinMode(xshutPin, OUTPUT);
  digitalWrite(xshutPin, LOW);
  delay(5);
  digitalWrite(xshutPin, HIGH);
  delay(50);                

  if (!lox.begin(0x29, false, &WireTof)) {
    Serial.printf("[ToF 0x%02X] begin @0x29 FAILED\n", newAddr);
    return false;
  }
  lox.setAddress(newAddr);
  delay(10);                
  Serial.printf("[ToF 0x%02X] addressed OK\n", newAddr);
  return true;
}

bool initAllTof() {
  pinMode(XSHUT_1, OUTPUT);
  pinMode(XSHUT_2, OUTPUT);
  pinMode(XSHUT_3, OUTPUT);
  pinMode(XSHUT_4, OUTPUT);
  digitalWrite(XSHUT_1, LOW);
  digitalWrite(XSHUT_2, LOW);
  digitalWrite(XSHUT_3, LOW);
  digitalWrite(XSHUT_4, LOW);
  delay(10);

  bool ok = true;
  ok &= initOneTof(tof1, XSHUT_1, ADDR_1);
  ok &= initOneTof(tof2, XSHUT_2, ADDR_2);
  ok &= initOneTof(tof3, XSHUT_3, ADDR_3);
  ok &= initOneTof(tof4, XSHUT_4, ADDR_4);

  digitalWrite(XSHUT_1, HIGH);
  digitalWrite(XSHUT_2, HIGH);
  digitalWrite(XSHUT_3, HIGH);
  digitalWrite(XSHUT_4, HIGH);
  return ok;
}

inline int readTofMm_nb(Adafruit_VL53L0X &lox, uint8_t &statusOut) {
  VL53L0X_RangingMeasurementData_t m;
  lox.rangingTest(&m, false);           
  statusOut = m.RangeStatus;
  if (m.RangeStatus == 4)   return 99999;  
  if (m.RangeMilliMeter > 8000) return 99999;  
  return (int)m.RangeMilliMeter;
}

void tof_task(void*){
  int stuck_count = 0;
  int bad_burst   = 0;
  int last_d[4] = { -1, -1, -1, -1 };
  uint8_t s1,s2,s3,s4;

  WireTof.setClock(400000);
  WireTof.setTimeOut(5);

  for(;;){
    int d1 = readTofMm_nb(tof1, s1);
    int d2 = readTofMm_nb(tof2, s2);
    int d3 = readTofMm_nb(tof3, s3);
    int d4 = readTofMm_nb(tof4, s4);

    uint32_t now_us = micros();
    taskENTER_CRITICAL(&g_tofMux);
    g_tof_mm[0] = d1; g_tof_stamp_us[0] = now_us; g_tof_valid[0] = (d1 != 99999);
    g_tof_mm[1] = d2; g_tof_stamp_us[1] = now_us; g_tof_valid[1] = (d2 != 99999);
    g_tof_mm[2] = d3; g_tof_stamp_us[2] = now_us; g_tof_valid[2] = (d3 != 99999);
    g_tof_mm[3] = d4; g_tof_stamp_us[3] = now_us; g_tof_valid[3] = (d4 != 99999);
    taskEXIT_CRITICAL(&g_tofMux);

    int invalids = (d1==99999) + (d2==99999) + (d3==99999) + (d4==99999);
    
    if (invalids == 4) {
      bad_burst++;
    } else {
      bad_burst = max(0, bad_burst - 1); 
    }

    if (bad_burst > 20) {
      pinMode(XSHUT_1, OUTPUT); pinMode(XSHUT_2, OUTPUT);
      pinMode(XSHUT_3, OUTPUT); pinMode(XSHUT_4, OUTPUT);
      digitalWrite(XSHUT_1, LOW); digitalWrite(XSHUT_2, LOW);
      digitalWrite(XSHUT_3, LOW); digitalWrite(XSHUT_4, LOW);
      delay(50);  
      WireTof.begin(TOF_SDA_PIN, TOF_SCL_PIN);
      WireTof.setClock(100000);
      WireTof.setTimeOut(50);
      initAllTof();
      bad_burst = 0;
      vTaskDelay(pdMS_TO_TICKS(50));
    }
    
    static uint32_t last_dbg_ms = 0; 
    if (millis() - last_dbg_ms > 500)  
    { 
      if (invalids > 0) {  
        // Serial.printf("[TOF] %d %d %d %d (bad_burst=%d invalids=%d)\n", 
        //               d1, d2, d3, d4, bad_burst, invalids); 
        continue;
      }
      last_dbg_ms = millis(); 
    }
    
    vTaskDelay(pdMS_TO_TICKS(10));  
  }
}

//---------------------------UDP TRANSMISSION (IF WIFI)-------------
void sendUDP(String msg) {
  if (remoteIP && remotePort) {
    udp.beginPacket(remoteIP, remotePort);
    udp.print(msg);
    udp.endPacket();
  }
}

//-----------------------------------ESP SETUP-------------------------
void setup() {

  Serial.begin(115200);   // USB CDC logs
  Serial0.begin(460800);
  Serial.println("ESP32 bidirectional UART Ready");
  Serial.println("Pulling Micro-ROS");

  // ----------------WIFI SETUP----------------------------------------
  WiFi.softAP(ssid, password, 4, 0, 2);
  IPAddress myIP = WiFi.softAPIP();
  Serial.print("ESP IP: ");
  Serial.println(myIP);

  //--------------------------------ESP SERVER---------------------------
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
    json += "\"Kp\":" + String(Kp_R) + ",";
    json += "\"Ki\":" + String(Ki_R) + ",";
    json += "\"Kd\":" + String(Kd_R) + ",";
    json += "\"Test\":" + String(test, 4); // 4 decimals for float
    json += "}";
    server.send(200, "application/json", json);
  });

  // POST updates
  server.on("/updatePID", HTTP_POST, []() {
    if (server.hasArg("Kp")) Kp_R = server.arg("Kp").toFloat();
    if (server.hasArg("Ki")) Ki_R = server.arg("Ki").toFloat();
    if (server.hasArg("Kd")) Kd_R = server.arg("Kd").toFloat();
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

  //---------------------------------PIN DEFINITIONS------------------------
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

  ///TOF Pins and Setup
  WireTof.begin(TOF_SDA_PIN, TOF_SCL_PIN);
  WireTof.setClock(100000);  
  WireTof.setTimeOut(50);
  pinMode(XSHUT_1, OUTPUT); 
  pinMode(XSHUT_2, OUTPUT);
  pinMode(XSHUT_3, OUTPUT); 
  pinMode(XSHUT_4, OUTPUT);
  i2cMutex = xSemaphoreCreateMutex();
  if (!initAllTof()) {
    Serial.println("ToF init had errors; watchdog will retry.");
  }
  xTaskCreatePinnedToCore(tof_task, "tof", 4096, nullptr, 2, nullptr, 0); 

  ///=========== IMU_BASE SETUP =========////
  xTaskCreatePinnedToCore(imu_task, "imu", 4096, nullptr, 1, nullptr, 0);
}

void loop() {

  //-------------------------MICRO_ROS_PROTCOL-------------------------------
  // ---- RX: parse commands (robust to CMD or CMD3) ----
  static uint8_t buf[64];      // big enough for CMD3 (48 bytes) + slack
  static size_t  have = 0;

    // accumulate
  while (Serial0.available() && have < sizeof(buf)) {
      buf[have++] = (uint8_t)Serial0.read();
    }
    // try to parse while we have at least a header
  while (have >= 6) { // magic(2)+ver(2)+type(2)
      uint16_t magic = (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
      if (magic != MAGIC) {
        // resync: drop 1 byte
        memmove(buf, buf+1, --have);
        continue;
      }
      uint16_t ver  = (uint16_t)buf[2] | ((uint16_t)buf[3] << 8);
      uint16_t type = (uint16_t)buf[4] | ((uint16_t)buf[5] << 8);

      if (ver != VER) {
        memmove(buf, buf+1, --have);
        continue;
      }
      size_t need = (type == TYPE_CMD) ? CMD_SIZE :
                  (type == TYPE_CMD3)? CMD3_SIZE : 0;
      if(need==0){
        // unknown type; drop 1 byte
        memmove(buf, buf+1, --have);
        continue;
      }
      if (have < need) break; // incomplete frame
      if (type == TYPE_CMD && need == CMD_SIZE) {
        CmdPacket cmd; memcpy(&cmd, buf, CMD_SIZE);
        uint16_t calc = crc16_surrogate((uint8_t*)&cmd, CMD_SIZE - 2);
      if (calc == cmd.crc16) {
        noInterrupts();
        uart_left_cmd  = cmd.left;
        uart_right_cmd = cmd.right;
        last_uart_cmd_ms = millis();
        interrupts();
        // (optional) Serial.printf("UART CMD: L=%.3f R=%.3f\n", cmd.left, cmd.right);
      }
      memmove(buf, buf + CMD_SIZE, have - CMD_SIZE); have -= CMD_SIZE;
    } else if (type == TYPE_CMD3 && need == CMD3_SIZE) {
      Cmd3Packet cmd3; memcpy(&cmd3, buf, CMD3_SIZE);
      uint16_t calc = crc16_surrogate((uint8_t*)&cmd3, CMD3_SIZE - 2);
      if (calc == cmd3.crc16) {
        noInterrupts();
        uart_left_cmd   = cmd3.left * -1;
        uart_right_cmd  = cmd3.right * -1;
        uart_turret_cmd = cmd3.turret;
        last_uart_cmd_ms = millis();
        interrupts();
        // (optional) Serial.printf("UART CMD3: L=%.3f R=%.3f T=%.3f\n", cmd3.left, cmd3.right, cmd3.turret);
      }
      memmove(buf, buf + CMD3_SIZE, have - CMD3_SIZE); have -= CMD3_SIZE;
    } else {
      // shouldn’t happen
      memmove(buf, buf+1, --have);
    }
  }
    
  //-----------------------TELEOP PROTOCOL------------------------------
  if (Serial.available()) {          
    String msg = Serial.readStringUntil('\n');
    msg.trim();

    int lyIndex = msg.indexOf("LY:");
    int rxIndex = msg.indexOf("RX:");
    int ltIndex = msg.indexOf("LT:");
    int rtIndex = msg.indexOf("RT:");
    if (lyIndex < 0 || rxIndex < 0 || ltIndex < 0 || rtIndex < 0);

    auto seg = [&](int idx)->String {
      int end = msg.indexOf(' ', idx);
      if (end < 0) end = msg.length();
      return msg.substring(idx + 3, end);
    };

    ly = seg(lyIndex).toFloat();
    rx = seg(rxIndex).toFloat();
    lt = seg(ltIndex).toFloat();
    rt = seg(rtIndex).toFloat();
    lastSerialCtlTime = millis();
  }

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
    ////=================== SAFETY STOP ==================////
    int d[4]; uint32_t s[4]; bool v[4];
    taskENTER_CRITICAL(&g_tofMux);
    for (int i=0;i<4;i++){ d[i]=g_tof_mm[i]; s[i]=g_tof_stamp_us[i]; v[i]=g_tof_valid[i]; }
    taskEXIT_CRITICAL(&g_tofMux);

    uint32_t now_us = micros();
    for (int i=0;i<4;i++){
      if (!v[i] || (now_us - s[i]) > TOF_FRESHNESS_US) d[i] = 99999;
    }

    int min_d = min(min(d[0], d[1]), min(d[2], d[3]));
    if (!tof_stop_latched && min_d <= TOF_STOP_THRESH) {
      tof_stop_latched = true;
    } else if (tof_stop_latched &&
              d[0] >= TOF_CLEAR_THRESH && d[1] >= TOF_CLEAR_THRESH &&
              d[2] >= TOF_CLEAR_THRESH && d[3] >= TOF_CLEAR_THRESH) {
      tof_stop_latched = false;
    }

    if (tof_stop_latched) {
      integralL = integralR = 0.0f;
      lastErrL = lastErrR = 0.0f;
      pwmL_out = pwmR_out = 0.0f;
      setMotor(pwmL, dirL, pwmL_out, 0);
      setMotor(pwmR, dirR, pwmR_out, 1);
      lastPidTime = now;
      return;   
    }

    ////=================== DRIVE CONTROL =================////
    // Read encoder counts atomically
    noInterrupts();
    long currentTicksL = ticksL;
    long currentTicksR = ticksR;
    long currentTicksT = ticksT;
    interrupts();

    // Calculate RPM for each motor
    float rpmL = ((currentTicksL - lastTicksL) / dt) * 60.0 / TICKS_PER_WHEEL_REV;
    float rpmR = ((currentTicksR - lastTicksR) / dt) * 60.0 / TICKS_PER_WHEEL_REV;
    float rpmT = ((currentTicksT - lastTicksT) / dt) * 60.0 / TICKS_PER_TURRET_REV;
    float rpmT_platform = rpmT * turretGearRatio; // Platform RPM

    lastTicksL = currentTicksL;
    lastTicksR = currentTicksR;
    lastPidTime = now;

    // Calculate ticks and pose send to Micro ROS Bridge to Publish 
    static uint32_t last_tx_ms = 0;
    if (millis() - last_tx_ms >= 10) {
      last_tx_ms = millis();

      EncPacket enc;
      enc.magic = MAGIC; enc.ver = VER; enc.type = TYPE_ENC;
      enc.seq = ++enc_seq;
      enc.t_tx_ns = (uint64_t)micros() * 1000ull;
      noInterrupts(); enc.ticksL = ticksL; enc.ticksR = ticksR; enc.ticksT = ticksT; interrupts();
      enc.crc16 = crc16_surrogate((uint8_t*)&enc, ENC_SIZE - 2);
      
      Serial0.write((uint8_t*)&enc, ENC_SIZE); // binary out on the data UART
    }
    static uint32_t last_imu_tx_ms = 0;
    if (millis() - last_imu_tx_ms >= 10) {
        last_imu_tx_ms = millis();
        transmitIMUData();
    }

    bool useUdp = (millis() - lastUdpTime < 100);
    bool useUart = (millis() - last_uart_cmd_ms < 100);
    bool useSerialCtl = (millis() - lastSerialCtlTime  < 1000);

    float rpmTargetL = 0.0f, rpmTargetR = 0.0f;
    float pwmFF_L = 0.0f, pwmFF_R = 0.0f;
    
    if (useUart) {
      // Convert ROS cmd vel -> wheel RPM, then clamp to your controller’s max
      float Lrpm = clamp(wheel_rpm_from_cmd(uart_left_cmd),  -MAX_RPM_CMD,  MAX_RPM_CMD);
      float Rrpm = clamp(wheel_rpm_from_cmd(uart_right_cmd), -MAX_RPM_CMD,  MAX_RPM_CMD);
      rpmTargetL = Lrpm;
      rpmTargetR = Rrpm;

      // Keep your existing FF style: scale basePWM by the normalized target
      pwmFF_L = basePWM * (rpmTargetL / MAX_RPM_CMD);
      pwmFF_R = basePWM * (rpmTargetR / MAX_RPM_CMD);
    }
    else if (useSerialCtl) {
      float forward = ly * 0.8f;
      float turn    = rx * 0.8f;
      rpmTargetL = (forward + turn) * MAX_RPM_CMD;
      rpmTargetR = (forward - turn) * MAX_RPM_CMD;
      pwmFF_L = (forward + turn) * basePWM;
      pwmFF_R = (forward - turn) * basePWM;
    } else {
      // UDP / HTML fallback
      float forward = useUdp ? ly : joyY;
      float turn    = useUdp ? rx : joyX;
      forward *= 0.8f; turn *= 0.8f;
      rpmTargetL = (forward + turn) * MAX_RPM_CMD;
      rpmTargetR = (forward - turn) * MAX_RPM_CMD;
      pwmFF_L = (forward + turn) * basePWM;
      pwmFF_R = (forward - turn) * basePWM;
    }

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
    pwmL_out = constrain(pwmFF_L + deltaPWM_L, -maxPWM_D, maxPWM_D);
    pwmR_out = constrain(pwmFF_R + deltaPWM_R, -maxPWM_D, maxPWM_D);

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
    
    if (useUart) {
      // ===== Open-loop turret drive: cmd [rad/s] -> PWM =====
      // 1) clamp incoming command to a safe range
      const float MAX_CMD_RAD_S = 2.0f;                // tune: your max platform speed
      float cmd = constrain(uart_turret_cmd, -MAX_CMD_RAD_S, MAX_CMD_RAD_S);

      // 2) deadband for tiny commands
      const float CMD_DEADBAND = 0.05f;                // rad/s (ignore tiny commands)
      if (fabsf(cmd) < CMD_DEADBAND) cmd = 0.0f;

      // 3) map |cmd| -> PWM with static friction compensation
      //    - PWM_STATIC: minimal PWM to break static friction
      //    - remaining range scales linearly with |cmd|
      const float PWM_STATIC = 350.0f;                 // tune: 250–600 typical
      float pwm_mag = 0.0f;
      if (cmd != 0.0f) {
        float frac = fabsf(cmd) / MAX_CMD_RAD_S;       // 0..1
        pwm_mag = PWM_STATIC + frac * (maxPWM - PWM_STATIC);
      }

      float pwm_target = copysignf(pwm_mag, cmd);

      // 4) slew limit to avoid jerks
      static float pwm_prev = 0.0f;
      const float PWM_SLEW_PER_S = 8000.0f;            // tune: PWM units per second
      float max_step = PWM_SLEW_PER_S * dt;
      float step = constrain(pwm_target - pwm_prev, -max_step, max_step);
      pwm_prev += step;

      // 5) apply limits and output
      pwmT_out = constrain(pwm_prev, -maxPWM, maxPWM);

      // Optional: status (platform angle, since ticks are output-side)
      currentAngleT = fmodf(ticksT * DEGREES_PER_TURRET_TICK, 360.0f);
    }

    if (lt > 0.1f || rt > 0.1f){
    // Simple turret control based on triggers
      targetTurretAngle = 0.0;
      float turretSpeed = (lt > 0.1) ? -lt * maxPWM : rt * maxPWM;
      pwmT_out = turretSpeed;
      currentAngleT = ticksT * DEGREES_PER_TURRET_TICK; // Calculate turret angle in degrees
      currentAngleT = fmod(currentAngleT/turretGearRatio, 360.0);
    }
    else if (fabs(targetTurretAngle)>0.0f) {
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
    // Serial.println(status);
    // sendUDP(status);
  }
  
  // delay(100);

  delay(10); 
}




