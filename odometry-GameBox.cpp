#ifndef ODOMETRY_H
#define ODOMETRY_H
#include <Arduino.h>
#include <esp32-hal.h>
#include <esp32-hal-gpio.h>
#include <esp32-hal-ledc.h>
#include <math.h>

// Configurable parameters - matched to your main code
const float WHEEL_RADIUS = 0.05;             // meters
const float WHEEL_BASE = 0.30;               // meters between wheels
const int CPR = 64;                          // encoder counts per revolution
const int GEAR_RATIO = 150;                  // gearbox ratio
const int TICKS_PER_REV = CPR * GEAR_RATIO;  // 9600 ticks per wheel revolution

// Encoder pins - matched to your main code
const int ENC_LEFT_A = 14;                   // GPIOs
const int ENC_LEFT_B = 13;
const int ENC_RIGHT_A = 1;
const int ENC_RIGHT_B = 2;

// Probabilistic Motion Model Parameters
// These represent the noise characteristics of your robot
const float ALPHA1 = 0.1;    // Rotational error due to rotation (rad²/rad²)
const float ALPHA2 = 0.01;   // Rotational error due to translation (rad²/m²)
const float ALPHA3 = 0.01;   // Translational error due to translation (m²/m²)
const float ALPHA4 = 0.1;    // Translational error due to rotation (m²/rad²)

// Additional encoder-specific noise parameters
const float ENCODER_NOISE_PER_TICK = 0.001;  // meters of uncertainty per encoder tick
const float BIAS_CORRECTION_FACTOR = 1.0;    // Systematic bias correction

// State variables - using same names as main code
extern volatile long ticksL;                 // encoder ticks (defined in main)
extern volatile long ticksR;
long prevTicksL = 0;                        // previous ticks for delta calculation
long prevTicksR = 0;

// Deterministic pose (mean of distribution)
float robot_x = 0.0;                        // robot position in meters
float robot_y = 0.0;
float robot_theta = 0.0;                    // robot orientation in radians

// Covariance matrix (3x3 for x, y, theta)
// Stored as upper triangle: [σxx, σxy, σxθ, σyy, σyθ, σθθ]
float covariance[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

// Motion command history for probabilistic updates
float last_delta_rot1 = 0.0;
float last_delta_trans = 0.0;
float last_delta_rot2 = 0.0;

// Timing for velocity estimation
unsigned long last_update_time = 0;
bool first_update = true;

// Random number generation for sampling
float gaussianRandom(float mean, float std_dev) {
    static bool has_spare = false;
    static float spare;
    
    if (has_spare) {
        has_spare = false;
        return spare * std_dev + mean;
    }
    
    has_spare = true;
    
    static float u, v, mag;
    do {
        u = random(-1000, 1000) / 1000.0f;
        v = random(-1000, 1000) / 1000.0f;
        mag = u * u + v * v;
    } while (mag >= 1.0f || mag == 0.0f);
    
    mag = sqrt(-2.0f * log(mag) / mag);
    spare = v * mag;
    return mean + std_dev * u * mag;
}

// Matrix operations for covariance propagation
void matrixMultiply3x3(const float A[9], const float B[9], float result[9]) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            result[i*3 + j] = 0;
            for (int k = 0; k < 3; k++) {
                result[i*3 + j] += A[i*3 + k] * B[k*3 + j];
            }
        }
    }
}

void transposeMatrix3x3(const float A[9], float result[9]) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            result[j*3 + i] = A[i*3 + j];
        }
    }
}

// Convert covariance array to full 3x3 matrix
void covarianceToMatrix(const float cov[6], float matrix[9]) {
    matrix[0] = cov[0];  // σxx
    matrix[1] = cov[1];  // σxy
    matrix[2] = cov[2];  // σxθ
    matrix[3] = cov[1];  // σyx = σxy
    matrix[4] = cov[3];  // σyy
    matrix[5] = cov[4];  // σyθ
    matrix[6] = cov[2];  // σθx = σxθ
    matrix[7] = cov[4];  // σθy = σyθ
    matrix[8] = cov[5];  // σθθ
}

// Convert full 3x3 matrix to covariance array
void matrixToCovariance(const float matrix[9], float cov[6]) {
    cov[0] = matrix[0];  // σxx
    cov[1] = matrix[1];  // σxy
    cov[2] = matrix[2];  // σxθ
    cov[3] = matrix[4];  // σyy
    cov[4] = matrix[5];  // σyθ
    cov[5] = matrix[8];  // σθθ
}

// Encoder Interrupt Service Routines - consistent with main code
void IRAM_ATTR handleEncL() {
    bool A = digitalRead(ENC_LEFT_A);
    bool B = digitalRead(ENC_LEFT_B);
    ticksL += (A == B) ? 1 : -1;
}

void IRAM_ATTR handleEncR() {
    bool A = digitalRead(ENC_RIGHT_A);
    bool B = digitalRead(ENC_RIGHT_B);
    ticksR += (A == B) ? -1 : 1;  // Inverted to match main code
}

void initOdometry() {
    pinMode(ENC_LEFT_A, INPUT_PULLUP);
    pinMode(ENC_LEFT_B, INPUT_PULLUP);
    pinMode(ENC_RIGHT_A, INPUT_PULLUP);
    pinMode(ENC_RIGHT_B, INPUT_PULLUP);

    // Attach interrupts only to A channels for quadrature decoding
    attachInterrupt(digitalPinToInterrupt(ENC_LEFT_A), handleEncL, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_A), handleEncR, CHANGE);
    
    // Initialize covariance matrix (small initial uncertainty)
    covariance[0] = 0.01;  // σxx
    covariance[1] = 0.0;   // σxy
    covariance[2] = 0.0;   // σxθ
    covariance[3] = 0.01;  // σyy
    covariance[4] = 0.0;   // σyθ
    covariance[5] = 0.01;  // σθθ
    
    last_update_time = millis();
    
    Serial.println("Probabilistic odometry initialized");
}

void updateOdometry() {
    unsigned long current_time = millis();
    float dt = (current_time - last_update_time) / 1000.0;
    
    if (first_update) {
        first_update = false;
        last_update_time = current_time;
        return;
    }
    
    // Read encoder counts atomically
    noInterrupts();
    long currentTicksL = ticksL;
    long currentTicksR = ticksR;
    interrupts();
    
    long deltaLeft = currentTicksL - prevTicksL;
    long deltaRight = currentTicksR - prevTicksR;

    // Update previous ticks
    prevTicksL = currentTicksL;
    prevTicksR = currentTicksR;
    last_update_time = current_time;

    // Convert ticks to wheel rotations in radians
    float dThetaL = (2.0 * PI * deltaLeft) / TICKS_PER_REV;
    float dThetaR = (2.0 * PI * deltaRight) / TICKS_PER_REV;

    // Calculate distances traveled by each wheel
    float dSLeft = dThetaL * WHEEL_RADIUS * BIAS_CORRECTION_FACTOR;
    float dSRight = dThetaR * WHEEL_RADIUS * BIAS_CORRECTION_FACTOR;

    // Calculate motion command in odometry frame
    float delta_trans = (dSLeft + dSRight) / 2.0;
    float delta_rot = (dSRight - dSLeft) / WHEEL_BASE;
    
    // For differential drive, we can split rotation into two components
    float delta_rot1 = 0.0;  // Initial rotation (for non-holonomic motion)
    float delta_rot2 = delta_rot;  // Final rotation
    
    // If there's significant translation, calculate initial rotation
    if (fabs(delta_trans) > 0.001) {
        delta_rot1 = atan2(delta_trans * sin(delta_rot), delta_trans * cos(delta_rot)) - robot_theta;
        delta_rot2 = delta_rot - delta_rot1;
    }
    
    // Store motion commands for noise model
    last_delta_rot1 = delta_rot1;
    last_delta_trans = delta_trans;
    last_delta_rot2 = delta_rot2;
    
    // === DETERMINISTIC UPDATE (mean) ===
    float old_x = robot_x;
    float old_y = robot_y;
    float old_theta = robot_theta;
    
    // Update pose using standard odometry equations
    robot_x += delta_trans * cos(robot_theta + delta_rot1);
    robot_y += delta_trans * sin(robot_theta + delta_rot1);
    robot_theta += delta_rot;
    
    // Normalize theta to [-PI, PI]
    while (robot_theta > PI) robot_theta -= 2.0 * PI;
    while (robot_theta < -PI) robot_theta += 2.0 * PI;
    
    // === PROBABILISTIC UPDATE (covariance) ===
    
    // Motion noise based on motion commands
    float rot1_var = ALPHA1 * fabs(delta_rot1) + ALPHA2 * fabs(delta_trans);
    float trans_var = ALPHA3 * fabs(delta_trans) + ALPHA4 * (fabs(delta_rot1) + fabs(delta_rot2));
    float rot2_var = ALPHA1 * fabs(delta_rot2) + ALPHA2 * fabs(delta_trans);
    
    // Add encoder-specific noise
    float encoder_noise = ENCODER_NOISE_PER_TICK * (fabs(deltaLeft) + fabs(deltaRight));
    trans_var += encoder_noise;
    
    // Jacobian of motion model with respect to pose
    float G[9] = {
        1, 0, -delta_trans * sin(old_theta + delta_rot1),
        0, 1,  delta_trans * cos(old_theta + delta_rot1),
        0, 0,  1
    };
    
    // Jacobian of motion model with respect to motion commands
    float V[9] = {
        -delta_trans * sin(old_theta + delta_rot1), cos(old_theta + delta_rot1), -delta_trans * sin(old_theta + delta_rot1),
         delta_trans * cos(old_theta + delta_rot1), sin(old_theta + delta_rot1),  delta_trans * cos(old_theta + delta_rot1),
         1, 0, 1
    };
    
    // Motion noise covariance matrix
    float M[9] = {
        rot1_var, 0, 0,
        0, trans_var, 0,
        0, 0, rot2_var
    };
    
    // Convert covariance array to matrix
    float Sigma[9];
    covarianceToMatrix(covariance, Sigma);
    
    // Compute new covariance: Sigma' = G * Sigma * G^T + V * M * V^T
    float GT[9], VT[9];
    transposeMatrix3x3(G, GT);
    transposeMatrix3x3(V, VT);
    
    float temp1[9], temp2[9], temp3[9], temp4[9];
    
    // G * Sigma * G^T
    matrixMultiply3x3(G, Sigma, temp1);
    matrixMultiply3x3(temp1, GT, temp2);
    
    // V * M * V^T
    matrixMultiply3x3(V, M, temp3);
    matrixMultiply3x3(temp3, VT, temp4);
    
    // Add the two terms
    for (int i = 0; i < 9; i++) {
        temp2[i] += temp4[i];
    }
    
    // Convert back to covariance array
    matrixToCovariance(temp2, covariance);
    
    // Ensure covariance remains positive definite (add small diagonal terms)
    covariance[0] = max(covariance[0], 1e-6f);  // σxx
    covariance[3] = max(covariance[3], 1e-6f);  // σyy
    covariance[5] = max(covariance[5], 1e-6f);  // σθθ
}

// Sample from the current pose distribution
void samplePose(float &sample_x, float &sample_y, float &sample_theta) {
    // For simplicity, assume independence and sample from marginal distributions
    sample_x = gaussianRandom(robot_x, sqrt(covariance[0]));
    sample_y = gaussianRandom(robot_y, sqrt(covariance[3]));
    sample_theta = gaussianRandom(robot_theta, sqrt(covariance[5]));
    
    // Normalize sampled theta
    while (sample_theta > PI) sample_theta -= 2.0 * PI;
    while (sample_theta < -PI) sample_theta += 2.0 * PI;
}

void resetOdometry() {
    robot_x = 0.0;
    robot_y = 0.0;
    robot_theta = 0.0;
    prevTicksL = ticksL;
    prevTicksR = ticksR;
    
    // Reset covariance to small initial values
    covariance[0] = 0.01;  // σxx
    covariance[1] = 0.0;   // σxy
    covariance[2] = 0.0;   // σxθ
    covariance[3] = 0.01;  // σyy
    covariance[4] = 0.0;   // σyθ
    covariance[5] = 0.01;  // σθθ
    
    first_update = true;
}

void printPose() {
    Serial.print("Robot Pose - X: "); 
    Serial.print(robot_x, 4);
    Serial.print(" ± "); 
    Serial.print(sqrt(covariance[0]), 4);
    Serial.print(" m, Y: "); 
    Serial.print(robot_y, 4);
    Serial.print(" ± "); 
    Serial.print(sqrt(covariance[3]), 4);
    Serial.print(" m, Theta: "); 
    Serial.print(robot_theta * 180.0 / PI, 2);
    Serial.print(" ± "); 
    Serial.print(sqrt(covariance[5]) * 180.0 / PI, 2);
    Serial.println(" degrees");
}

void printCovariance() {
    Serial.println("Covariance Matrix:");
    Serial.printf("  [%8.6f %8.6f %8.6f]\n", covariance[0], covariance[1], covariance[2]);
    Serial.printf("  [%8.6f %8.6f %8.6f]\n", covariance[1], covariance[3], covariance[4]);
    Serial.printf("  [%8.6f %8.6f %8.6f]\n", covariance[2], covariance[4], covariance[5]);
}

void printMotionModel() {
    Serial.printf("Last Motion: rot1=%.4f, trans=%.4f, rot2=%.4f\n", 
                  last_delta_rot1, last_delta_trans, last_delta_rot2);
}

// Getter functions for accessing pose data
float getRobotX() { return robot_x; }
float getRobotY() { return robot_y; }
float getRobotTheta() { return robot_theta; }

// Getter functions for accessing uncertainty
float getUncertaintyX() { return sqrt(covariance[0]); }
float getUncertaintyY() { return sqrt(covariance[3]); }
float getUncertaintyTheta() { return sqrt(covariance[5]); }

// Get full covariance matrix
void getCovariance(float cov[6]) {
    for (int i = 0; i < 6; i++) {
        cov[i] = covariance[i];
    }
}

// Set noise parameters (for tuning)
void setNoiseParameters(float alpha1, float alpha2, float alpha3, float alpha4) {
    // These would need to be made non-const and stored as variables
    Serial.printf("Noise parameters: α1=%.4f, α2=%.4f, α3=%.4f, α4=%.4f\n", 
                  alpha1, alpha2, alpha3, alpha4);
}

#endif // ODOMETRY_H