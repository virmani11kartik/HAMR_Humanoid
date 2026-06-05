#ifndef IMU_55_H
#define IMU_55_H

#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_BNO08x.h>

enum IMUStatus {
    IMU_OK = 0,
    IMU_NOT_CALIBRATED,
    IMU_COMM_ERROR,
    IMU_INIT_FAILED,
    IMU_DATA_INVALID
};

class IMU55 {
public:
  IMU55(uint8_t sda=4, uint8_t scl=5, uint8_t addr=0x28, uint16_t sample_ms=100);
  bool begin();                 // init I2C + BNO + restore calib if present
  void update();                // call each loop (drives fusion)
  void getRPY(float& roll, float& pitch, float& yaw);
  void getLinearAccel(float& ax, float& ay, float& az);
  void getAngularVel(float& gx, float& gy, float& gz);
  bool isCalibrated();          // true when SYS/G/A/M == 3
  void clearCalNVS();           // manual wipe of saved calibration

  // Enhanced health monitoring
  IMUStatus getStatus() const { return status_; }
  void getCalibrationLevels(uint8_t& sys, uint8_t& gyro, uint8_t& accel, uint8_t& mag);
  bool isDataValid() const { return data_valid_; }
  unsigned long getLastValidDataTime() const { return last_valid_data_ms_; }
  float getDataAge() const { return (millis() - last_valid_data_ms_) / 1000.0f; }
  
  // Configuration
  void setMinCalibrationLevel(uint8_t level) { min_calib_level_ = level; }
  void setDataTimeout(unsigned long timeout_ms) { data_timeout_ms_ = timeout_ms; }
  
  // Debug/diagnostics
  void printStatus() const;
  void forceRecalibration();

private:
  bool loadCalibrationFromNVS();   // CONFIG -> offsets -> ext crystal -> NDOF + kick
  void saveCalibrationToNVS();     // save once when fully calibrated
  void i2c_init(int sda, int scl);
  void updateStatus();

  Adafruit_BNO055 bno_;
  Preferences prefs_;
  uint8_t sda_, scl_, addr_;
  uint16_t sample_ms_;

  IMUStatus status_ = IMU_INIT_FAILED;
  bool stored_ = false;
  bool resequenced_ = false;
  uint32_t zeroStart_ = 0;
  bool data_valid_ = true;
  unsigned long last_valid_data_ms_ = 0;
  unsigned long data_timeout_ms_ = 1000; // 1 second timeout
  uint8_t min_calib_level_ = 2;

  sensors_event_t e_;        // Euler angles
  sensors_event_t e_accel_;  // linear acceleration (gravity removed), m/s²
  sensors_event_t e_gyro_;   // angular velocity, rad/s
  uint8_t last_sys_ = 0, last_gyro_ = 0, last_accel_ = 0, last_mag_ = 0;
  
  // Error tracking
  unsigned long comm_error_count_ = 0;
  unsigned long last_comm_error_ms_ = 0;
};

#endif
