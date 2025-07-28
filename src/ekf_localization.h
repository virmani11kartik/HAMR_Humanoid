#ifndef EKF_LOCALIZATION_H
#define EKF_LOCALIZATION_H
#include <Arduino.h>
#include "odometry.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_BNO08x.h>
#include <Adafruit_ICM20948.h>
#include <utility/imumaths.h>
#include <cmath> 

// EKF state vector: [x, y, theta, vx, vy, omega]
const int EKF_STATE_SIZE = 6; // x, y, theta, vx, vy, omega
const int EKF_MEASUREMENT_SIZE = 3; // x, y, theta measurements

class EKFLocalization {
private:
    float state[EKF_STATE_SIZE]; // State vector
    float covariance[EKF_STATE_SIZE * EKF_STATE_SIZE]; // Covariance matrix
    float process_noise[EKF_STATE_SIZE]; // Process noise matrix
    float measurement_noise[EKF_MEASUREMENT_SIZE]; // Measurement noise matrix

    Adafruit_BNO055 bno; // BNO055 IMU sensor
    Adafruit_BNO08x bno08x; // BNO08x IMU sensor
    Adafruit_ICM20948 icm20948; // ICM20948 IMU sensor

    unsigned long last_update_time; // Last update time for EKF
    bool first_update; // Flag for first update

    // IMU Calibration
    bool imu_calibrated; // Flag to check if IMU is calibrated
    float imu_theta_offset; // Offset for IMU theta

    // Odometry values
    float prev_odo_x, prev_odo_y, prev_odo_theta; // Previous odometry values

    // Matrix operations
    void setMatrixElement(float* matrix, int size, int row, int col, float value);
    float getMatrixElement(const float* matrix, int size, int row, int col) const;
    void matrixMultiply(const float* A, const float* B, float* result, int rowsA, int colsA, int colsB);
    void matrixTranspose(const float* A, float* result, int rows, int cols);
    void matrixAdd(const float* A, const float* B, float* result, int rows, int cols);
    void matrixInvert3x3(const float* A, float* result);

    // EKF functions
    void predictState(float dt);
    void predictCovariance(float dt);
    void updateWithMeasurement(const float* measurement, const float* measurement_covariance);
    void computeJacobians(float* F, float* H, float dt);

public:
    EKFLocalization();
    
    //Initialize EKF
    bool initIMU(int sda_pin = 38, int scl_pin = 39);
    void initEKF();
    void calibrateIMU();

    void updateEKF();

    float getX() const { return state[0]; }
    float getY() const { return state[1]; }
    float getTheta() const { return state[2]; }
    float getVelX() const { return state[3]; }
    float getVelY() const { return state[4]; }
    float getOmega() const { return state[5]; }

    // Get uncertainties
    float getUncertaintyX() const;
    float getUncertaintyY() const;
    float getUncertaintyTheta() const;

    void resetEKF();
    void printState();
    void printCovariance();
    void setProcessNoise(float pos_noise, float vel_noise, float ang_noise);
    void setMeasurementNoise(float odo_pos_noise, float imu_ang_noise);

    // Get IMU data
    bool getIMUData(float& roll, float& pitch, float& yaw, float& angular_velocity);
    bool isIMUCalibrated();
};

EKFLocalization::EKFLocalization() : bno(55, 0x28), last_update_time(0), first_update(true), 
                                     imu_theta_offset(0.0), imu_calibrated(false),
                                     prev_odo_x(0), prev_odo_y(0), prev_odo_theta(0){
    // Initialize state vector and covariance matrix
    for (int i = 0; i < EKF_STATE_SIZE; i++) {
        state[i] = 0.0f;
    }
    for (int i = 0; i < EKF_STATE_SIZE * EKF_STATE_SIZE; i++) {
        covariance[i] = 0.0f;
    }
    // Set initial covariance (diagonal elements)
    setMatrixElement(covariance, EKF_STATE_SIZE, 0, 0, 0.01); // x
    setMatrixElement(covariance, EKF_STATE_SIZE, 1, 1, 0.01); // y  
    setMatrixElement(covariance, EKF_STATE_SIZE, 2, 2, 0.01); // theta
    setMatrixElement(covariance, EKF_STATE_SIZE, 3, 3, 0.1);  // vel_x
    setMatrixElement(covariance, EKF_STATE_SIZE, 4, 4, 0.1);  // vel_y
    setMatrixElement(covariance, EKF_STATE_SIZE, 5, 5, 0.1);  // omega

    // default process noise
    process_noise[0] = 0.01;  // x position
    process_noise[1] = 0.01;  // y position  
    process_noise[2] = 0.005; // theta
    process_noise[3] = 0.1;   // vel_x
    process_noise[4] = 0.1;   // vel_y
    process_noise[5] = 0.05;  // omega

    // default measurement noise
    measurement_noise[0] = 0.02; // odometry x
    measurement_noise[1] = 0.02; // odometry y
    measurement_noise[2] = 0.01; // IMU theta
}

bool EKFLocalization::initIMU(int sda_pin, int scl_pin) {
    Wire.begin(sda_pin, scl_pin);
    if(!bno.begin()) {
        Serial.println("Failed to initialize BNO055!");
        return false;
    }
    delay(1000);
    bno.setExtCrystalUse(true);
    Serial.println("BNO055 initialized successfully");
    return true;
}

void EKFLocalization::initEKF() {
    last_update_time = millis();
    first_update = true;
    Serial.println("EKF Localization initialized");
}

void EKFLocalization::calibrateIMU() {
    if(!imu_calibrated) {
        sensors_event_t event;
        bno.getEvent(&event);
        imu_theta_offset = event.orientation.x * PI / 180.0;
        imu_calibrated = true;
        Serial.printf("IMU calibrated with offset: %.3f rad\n", imu_theta_offset);
    }
}

bool EKFLocalization::getIMUData(float& roll, float& pitch, float& yaw, float& angular_velocity) {
    sensors_event_t orientation_event, gyro_event;
    bno.getEvent(&orientation_event, Adafruit_BNO055::VECTOR_EULER);
    bno.getEvent(&gyro_event, Adafruit_BNO055::VECTOR_GYROSCOPE);
    if(orientation_event.orientation.x == 0 && orientation_event.orientation.y == 0 && orientation_event.orientation.z == 0) {
        return false; // Invalid reading
    }
    roll = orientation_event.orientation.y * PI / 180.0;
    pitch = orientation_event.orientation.z * PI / 180.0;
    yaw = orientation_event.orientation.x * PI / 180.0;
    angular_velocity = gyro_event.gyro.z; // Z-axis gyro for yaw rate
    // Apply calibration offset
    yaw -= imu_theta_offset;
    // Normalize yaw to [-PI, PI]
    while(yaw > PI) yaw -= 2.0 * PI;
    while(yaw < -PI) yaw += 2.0 * PI;
    return true;
}

bool EKFLocalization::isIMUCalibrated() {
    uint8_t system, gyro, accel, mag;
    bno.getCalibration(&system, &gyro, &accel, &mag);
    return (gyro >=2 && mag >= 2);
}

void EKFLocalization::updateEKF(){
    unsigned long current_time = millis();
    float dt = (current_time - last_update_time) / 1000.0; // Convert to seconds

    if(first_update) {
        // Initialize state with odometry values
        first_update = false;
        last_update_time = current_time;
        state[0] = getRobotX(); // Initial x position
        state[1] = getRobotY(); // Initial y position
        state[2] = getRobotTheta(); // Initial theta

        prev_odo_x = state[0];
        prev_odo_y = state[1];
        prev_odo_theta = state[2];

        if(!imu_calibrated) {
            calibrateIMU(); // Calibrate IMU if not done
        }
        return; // Skip prediction step on first update
    }

    if(dt < 0.01) return; // Avoid too small dt

    // Predict step
    predictState(dt);
    predictCovariance(dt);

    float measurement[EKF_MEASUREMENT_SIZE];
    float meas_covariance[EKF_MEASUREMENT_SIZE];

    measurement[0] = getRobotX(); // Odometry x
    measurement[1] = getRobotY(); // Odometry y
    meas_covariance[0] = measurement_noise[0]; // Odometry x noise
    meas_covariance[1] = measurement_noise[1]; // Odometry y noise

    // Get IMU data
    float roll, pitch, yaw, omega;
    if(getIMUData(roll, pitch, yaw, omega)) {
        measurement[2] = yaw; // IMU theta
        meas_covariance[2] = measurement_noise[2]; // IMU theta noise
        state[5] = omega; // Update omega from IMU
        updateWithMeasurement(measurement, meas_covariance);
    } else {
        float odo_measurement[2] = {measurement[0], measurement[1]};
        float odo_covariance[2] = {meas_covariance[0], meas_covariance[1]};
    }
    last_update_time = current_time;
}

void EKFLocalization::predictState(float dt) {
    // State transition model: 
    // x' = x + vel_x * dt
    // y' = y + vel_y * dt  
    // theta' = theta + omega * dt
    // vel_x' = vel_x (constant velocity model)
    // vel_y' = vel_y
    // omega' = omega

    state[0] += state[3] * dt; // x
    state[1] += state[4] * dt; // y
    state[2] += state[5] * dt; // theta

    // Normalize theta to [-PI, PI]
    while(state[2] > PI) state[2] -= 2.0 * PI;
    while(state[2] < -PI) state[2] += 2.0 * PI;

    // Updating velocities based on odometry
    float current_odo_x = getRobotX();
    float current_odo_y = getRobotY();

    if(dt>0.001){
        state[3] = (current_odo_x - prev_odo_x) / dt; // vel_x
        state[4] = (current_odo_y - prev_odo_y) / dt; // vel_y
    }
    prev_odo_x = current_odo_x;
    prev_odo_y = current_odo_y;
}

void EKFLocalization::predictCovariance(float dt) {
    float dt2 = dt * dt;
    float current_cov[EKF_STATE_SIZE];
    for(int i = 0; i < EKF_STATE_SIZE; i++) {
        current_cov[i] = getMatrixElement(covariance, EKF_STATE_SIZE, i, i);
        current_cov[i] += process_noise[i] * dt2; // Add process noise
        setMatrixElement(covariance, EKF_STATE_SIZE, i, i, current_cov[i]);
    }
}

void EKFLocalization::updateWithMeasurement(const float* measurement, const float* measurement_covariance) {
    // Measurement model: H maps state to measurements
    // z = [x_odo, y_odo, theta_imu] = [x, y, theta] from state
    
    float innovation[EKF_MEASUREMENT_SIZE];
    innovation[0] = measurement[0] - state[0]; // x_odo - x
    innovation[1] = measurement[1] - state[1]; // y_odo
    innovation[2] = measurement[2] - state[2]; // theta_imu - theta

    // Normalize innovation for theta
    while(innovation[2] > PI) innovation[2] -= 2.0 * PI;
    while(innovation[2] < -PI) innovation[2] += 2.0 * PI;

    // Innovation Covariance
    float S[EKF_MEASUREMENT_SIZE];
    S[0] = getMatrixElement(covariance, EKF_STATE_SIZE, 0, 0) + measurement_covariance[0]; // σxx + odo_x_noise
    S[1] = getMatrixElement(covariance, EKF_STATE_SIZE, 1, 1) + measurement_covariance[1]; // σyy + odo_y_noise
    S[2] = getMatrixElement(covariance, EKF_STATE_SIZE, 2, 2) + measurement_covariance[2]; // σθθ + imu_theta_noise

    // Kalman Gain
    float K[EKF_STATE_SIZE * EKF_MEASUREMENT_SIZE];
    for(int i = 0; i < EKF_STATE_SIZE; i++) {
        for(int j = 0; j < EKF_MEASUREMENT_SIZE; j++) {
            K[i * EKF_MEASUREMENT_SIZE + j] = 0.0f;
        }
    }

    // Set Kalman Gain
    if(S[0] > 1e-6) K[0*EKF_MEASUREMENT_SIZE + 0] = getMatrixElement(covariance, EKF_STATE_SIZE, 0, 0) / S[0]; // K_x
    if(S[1] > 1e-6) K[1*EKF_MEASUREMENT_SIZE + 1] = getMatrixElement(covariance, EKF_STATE_SIZE, 1, 1) / S[1]; // K_y
    if(S[2] > 1e-6) K[2*EKF_MEASUREMENT_SIZE + 2] = getMatrixElement(covariance, EKF_STATE_SIZE, 2, 2) / S[2]; // K_theta

    // Update state with innovation
    for(int i = 0; i < EKF_STATE_SIZE; i++) {
        for(int j = 0; j < EKF_MEASUREMENT_SIZE; j++) {
            state[i] += K[i * EKF_MEASUREMENT_SIZE + j] * innovation[j];
        }
    }

    // Normalize theta to [-PI, PI]
    while(state[2] > PI) state[2] -= 2.0 * PI;
    while(state[2] < -PI) state[2] += 2.0 * PI;

    // Update covariance
    for(int i=0; i<min(EKF_STATE_SIZE, EKF_MEASUREMENT_SIZE); i++) {
        float current_var = getMatrixElement(covariance, EKF_STATE_SIZE, i, i);
        float updated_var = current_var * (1 - K[i * EKF_MEASUREMENT_SIZE + i]);
        setMatrixElement(covariance, EKF_STATE_SIZE, i, i, max(updated_var, 1e-6f)); // Ensure positive definiteness
    }
}

void EKFLocalization::setMatrixElement(float* matrix, int size, int row, int col, float value) {
    matrix[row * size + col] = value;
}

float EKFLocalization::getMatrixElement(const float* matrix, int size, int row, int col) const {
    return matrix[row * size + col];
}

float EKFLocalization::getUncertaintyX() const {
    return sqrt(getMatrixElement(covariance, EKF_STATE_SIZE, 0, 0));
}

float EKFLocalization::getUncertaintyY() const {
    return sqrt(getMatrixElement(covariance, EKF_STATE_SIZE, 1, 1));
}

float EKFLocalization::getUncertaintyTheta() const {
    return sqrt(getMatrixElement(covariance, EKF_STATE_SIZE, 2, 2));
}

void EKFLocalization::resetEKF(){
    for(int i = 0; i < EKF_STATE_SIZE; i++) {
        state[i] = 0.0f; // Reset state
    }
    for (int i = 0; i < EKF_STATE_SIZE * (EKF_STATE_SIZE + 1) / 2; i++) {
        covariance[i] = 0.0f; // Reset covariance
    }

    // Set initial covariance (diagonal elements)
    setMatrixElement(covariance, EKF_STATE_SIZE, 0, 0, 0.01); // x
    setMatrixElement(covariance, EKF_STATE_SIZE, 1, 1, 0.01); // y
    setMatrixElement(covariance, EKF_STATE_SIZE, 2, 2, 0.01); // theta
    setMatrixElement(covariance, EKF_STATE_SIZE, 3, 3, 0.1);  // vel_x
    setMatrixElement(covariance, EKF_STATE_SIZE, 4, 4, 0.1);  // vel_y
    setMatrixElement(covariance, EKF_STATE_SIZE, 5, 5, 0.1);  // omega

    first_update = true; // Reset first update flag
    imu_calibrated = false; // Reset IMU calibration flag
}

void EKFLocalization::printState() {
    Serial.printf("EKF State - X: %.4f±%.4f, Y: %.4f±%.4f, Theta: %.2f±%.2f°, Vel: %.3f,%.3f, Omega: %.3f\n",
                  state[0], getUncertaintyX(),
                  state[1], getUncertaintyY(),
                  state[2] * 180.0 / PI, getUncertaintyTheta() * 180.0 / PI,
                  state[3], state[4], state[5]);
}

void EKFLocalization::setProcessNoise(float pos_noise, float vel_noise, float ang_noise) {
    process_noise[0] = pos_noise;  // x position
    process_noise[1] = pos_noise;  // y position  
    process_noise[2] = ang_noise;   // theta
    process_noise[3] = vel_noise;   // vel_x
    process_noise[4] = vel_noise;   // vel_y
    process_noise[5] = ang_noise;   // omega
}

void EKFLocalization::setMeasurementNoise(float odo_pos_noise, float imu_ang_noise) {
    measurement_noise[0] = odo_pos_noise; // odometry x
    measurement_noise[1] = odo_pos_noise; // odometry y
    measurement_noise[2] = imu_ang_noise;  // IMU theta
}

#endif