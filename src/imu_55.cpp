#include "imu_55.h"

IMU55::IMU55(uint8_t sda, uint8_t scl, uint8_t addr, uint16_t sample_ms)
: bno_(55, addr, &Wire), sda_(sda), scl_(scl), addr_(addr), sample_ms_(sample_ms) {}

void IMU55::i2c_init(int sda, int scl) {
  pinMode(sda, INPUT_PULLUP);
  pinMode(scl, INPUT_PULLUP);
  delay(2);
  if (digitalRead(sda) == LOW) {
    pinMode(scl, OUTPUT);
    for (int i = 0; i < 9; ++i) { digitalWrite(scl, LOW); delayMicroseconds(5); digitalWrite(scl, HIGH); delayMicroseconds(5); }
    pinMode(sda, OUTPUT);
    digitalWrite(sda, LOW); delayMicroseconds(5);
    digitalWrite(scl, HIGH); delayMicroseconds(5);
    digitalWrite(sda, HIGH); delayMicroseconds(5);
  }
}

bool IMU55::begin() {
  Serial.println("Init BNO055…");
  status_ = IMU_INIT_FAILED;

  i2c_init(sda_, scl_);
  Wire.begin(sda_, scl_, 100000);
  Wire.setTimeOut(50);
  delay(800);

  Serial.printf("Probing 0x%02X… ", addr_);
  Wire.beginTransmission(addr_);
  uint8_t error = Wire.endTransmission(true);
  Serial.printf("err=%u\n", error);

  if (error != 0) {
    status_ = IMU_COMM_ERROR;
    Serial.println("IMU: I2C communication error");
    return false;
  }

  bool ok=false;
  for (int tries=1; tries<=5; ++tries) {
    Serial.printf("  Attempt %d: bno.begin()… ", tries);
    ok = bno_.begin();
    Serial.println(ok ? "OK" : "NOT DETECTED");
    if (ok) break;
    delay(500);
  }
  if (!ok) {
    status_ = IMU_INIT_FAILED;
    return false;
  }

  bool restored = loadCalibrationFromNVS();
  Serial.println(restored ? "Calibration restored from NVS. Move slightly to settle."
                          : "No saved calibration. Do gyro still / 6 faces / figure-8.");

  sensor_t s; bno_.getSensor(&s);
  Serial.printf("Sensor ID: %ld\n", (long)s.sensor_id);
  status_ = IMU_NOT_CALIBRATED;
  last_valid_data_ms_ = millis();
  return true;
}

bool IMU55::loadCalibrationFromNVS() {
  adafruit_bno055_offsets_t offs;
  prefs_.begin("imu", true);
  bool has = prefs_.isKey("calib");
  if (has) prefs_.getBytes("calib", &offs, sizeof(offs));
  prefs_.end();

  bno_.setMode(OPERATION_MODE_CONFIG); delay(25);
  if (has) {
    bno_.setSensorOffsets(offs); delay(10);
    Serial.printf("Restored Offsets | ACC:%d,%d,%d  GYR:%d,%d,%d  MAG:%d,%d,%d  RAD(acc,mag):%d,%d\n",
      offs.accel_offset_x, offs.accel_offset_y, offs.accel_offset_z,
      offs.gyro_offset_x,  offs.gyro_offset_y,  offs.gyro_offset_z,
      offs.mag_offset_x,   offs.mag_offset_y,   offs.mag_offset_z,
      offs.accel_radius,   offs.mag_radius);
  }
  bno_.setExtCrystalUse(true);  delay(10);
  bno_.setMode(OPERATION_MODE_NDOF); delay(20);

  // kick fusion
  for (int i=0;i<3;++i){ bno_.getEvent(&e_, Adafruit_BNO055::VECTOR_EULER); delay(10); }
  return has;
}

void IMU55::saveCalibrationToNVS() {
  if (!isCalibrated()) return;
  adafruit_bno055_offsets_t offs;
  bno_.getSensorOffsets(offs);
  prefs_.begin("imu", false);
  prefs_.putBytes("calib", &offs, sizeof(offs));
  prefs_.end();
  Serial.println("Calibration saved to NVS.");
}

void IMU55::clearCalNVS() {
  prefs_.begin("imu", false);
  prefs_.remove("calib");
  prefs_.end();
  Serial.println("Calibration offsets cleared from NVS.");
}

void IMU55::updateStatus() {
  bno_.getCalibration(&last_sys_, &last_gyro_, &last_accel_, &last_mag_);
  if (last_sys_ == 0 && last_gyro_ == 0 && last_accel_ == 0 && last_mag_ == 0) {
    if (millis() - last_valid_data_ms_ > data_timeout_ms_) {
      status_ = IMU_COMM_ERROR;
      data_valid_ = false;
      return;
    }
  }
  uint8_t min_required = min_calib_level_;
  if (last_sys_ < min_required || last_gyro_ < min_required || 
      last_accel_ < min_required || last_mag_ < min_required) {
    status_ = IMU_NOT_CALIBRATED;
    data_valid_ = false;
    return;
  }
  if (!isfinite(e_.orientation.x) || !isfinite(e_.orientation.y) || !isfinite(e_.orientation.z)) {
    status_ = IMU_DATA_INVALID;
    data_valid_ = false;
    return;
  }
  status_ = IMU_OK;
  data_valid_ = true;
  last_valid_data_ms_ = millis();
}

void IMU55::update() {
  bno_.getEvent(&e_,       Adafruit_BNO055::VECTOR_EULER);
  bno_.getEvent(&e_accel_, Adafruit_BNO055::VECTOR_LINEARACCEL);
  bno_.getEvent(&e_gyro_,  Adafruit_BNO055::VECTOR_GYROSCOPE);

  uint8_t sys=0,g=0,a=0,m=0;
  bno_.getCalibration(&sys,&g,&a,&m);
  if (!resequenced_) {
    if (sys==0 && g==0 && a==0 && m==0) {
      if (zeroStart_==0) zeroStart_=millis();
      if (millis()-zeroStart_>2000) {
        Serial.println("Calib stuck at zeros; re-sequencing NDOF…");
        bno_.setMode(OPERATION_MODE_CONFIG); delay(25);
        bno_.setExtCrystalUse(true);         delay(10);
        bno_.setMode(OPERATION_MODE_NDOF);   delay(25);
        for (int i=0;i<3;++i){ bno_.getEvent(&e_, Adafruit_BNO055::VECTOR_EULER); delay(10); }
        resequenced_ = true;
      }
    } else {
      zeroStart_ = 0;
    }
  }

  if (!stored_ && sys==3 && g==3 && a==3 && m==3) {
    saveCalibrationToNVS();
    stored_ = true;
  }
  updateStatus();
}

void IMU55::getRPY(float& roll, float& pitch, float& yaw) {
  yaw = e_.orientation.x * M_PI / 180.0f;
  while (yaw > M_PI) yaw -= 2.0f * M_PI;
  while (yaw < -M_PI) yaw += 2.0f * M_PI;
  roll = e_.orientation.y * M_PI / 180.0f;
  pitch = e_.orientation.z * M_PI / 180.0f;
}

void IMU55::getLinearAccel(float& ax, float& ay, float& az) {
  ax = e_accel_.acceleration.x;
  ay = e_accel_.acceleration.y;
  az = e_accel_.acceleration.z;
}

void IMU55::getAngularVel(float& gx, float& gy, float& gz) {
  gx = e_gyro_.gyro.x;
  gy = e_gyro_.gyro.y;
  gz = e_gyro_.gyro.z;
}

bool IMU55::isCalibrated() {
  uint8_t sys,g,a,m;
  bno_.getCalibration(&sys,&g,&a,&m);
  Serial.printf("Calibration (SYS,G,A,M)=(%u,%u,%u,%u)\n", sys,g,a,m);
  return (sys==3 && g==3 && a==3 && m==3);
}

void IMU55::getCalibrationLevels(uint8_t& sys, uint8_t& gyro, uint8_t& accel, uint8_t& mag) {
  sys = last_sys_;
  gyro = last_gyro_;
  accel = last_accel_;
  mag = last_mag_;
}

void IMU55::printStatus() const {
  const char* status_strings[] = {"OK", "NOT_CALIBRATED", "COMM_ERROR", "INIT_FAILED", "DATA_INVALID"};
  Serial.printf("IMU Status: %s\n", status_strings[status_]);
  Serial.printf("Calibration (SYS,G,A,M): (%u,%u,%u,%u)\n", last_sys_, last_gyro_, last_accel_, last_mag_);
  Serial.printf("Data Valid: %s, Age: %.2fs\n", data_valid_ ? "YES" : "NO", getDataAge());
  Serial.printf("Comm Errors: %lu, Last: %lums ago\n", comm_error_count_, 
               last_comm_error_ms_ > 0 ? millis() - last_comm_error_ms_ : 0);
}

void IMU55::forceRecalibration() {
  Serial.println("IMU: Forcing recalibration...");
  stored_ = false;
  clearCalNVS();
  bno_.setMode(OPERATION_MODE_CONFIG);
  delay(25);
  bno_.setExtCrystalUse(true);
  delay(10);
  bno_.setMode(OPERATION_MODE_NDOF);
  delay(25);
  for (int i = 0; i < 3; ++i) {
    bno_.getEvent(&e_, Adafruit_BNO055::VECTOR_EULER);
    delay(10);
  }
  status_ = IMU_NOT_CALIBRATED;
}