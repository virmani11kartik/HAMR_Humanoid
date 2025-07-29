#ifndef ODOMETRY_H
#define ODOMETRY_H
#include <Arduino.h>
#include <esp32-hal.h>
#include <esp32-hal-gpio.h>
#include <esp32-hal-ledc.h>
#include <WiFi.h>   
#include <WiFiUdp.h>

extern const int CPR;
extern const int GEAR_RATIO;
extern const int TICKS_PER_WHEEL_REV;
extern volatile long ticksL;
extern volatile long ticksR;
extern WiFiUDP udp;
extern IPAddress remoteIP;
extern unsigned int remotePort;

// Configurable parameters - matched to your main code
const float WHEEL_RADIUS = 0.0762;             // meters
const float WHEEL_BASE = 0.297;               // meters between wheels

// Probabilistic Motion Model parameters 
// TUNABLE FOR ROBOT
// Noise Characteristics
const float ALPHA1 = 0.1;    // Rotational error due to rotation (rad²/rad²)
const float ALPHA2 = 0.01;   // Rotational error due to translation (rad²/m²)
const float ALPHA3 = 0.01;   // Translational error due to translation (m²/m²)
const float ALPHA4 = 0.1;    // Translational error due to rotation (m²/rad²)

// // Conservative settings (more uncertainty)
// const float ALPHA1 = 0.2;    // If your robot has wheel slip
// const float ALPHA2 = 0.05;   // If encoders have systematic drift  
// const float ALPHA3 = 0.05;   // If wheels have different diameters
// const float ALPHA4 = 0.2;    // If robot has play in drivetrain

// // Aggressive settings (less uncertainty) 
// const float ALPHA1 = 0.05;   // For precise, rigid robots
// const float ALPHA2 = 0.001;  // For high-quality encoders
// const float ALPHA3 = 0.001;  // For well-calibrated wheels
// const float ALPHA4 = 0.05;   // For stiff mechanical systems

// Encoder Noise Characteristics
const float ENCODER_NOISE_PER_TICK = 0.001; // meters per tick
const float BIAS_CORRCTION_FACTOR = 1.0; // Bias correction factor for encoder noise

// State variables - using same names as main code
long prevTicksL = 0;                        // previous ticks for delta calculation
long prevTicksR = 0;

float robot_x = 0.0;                        // robot position in meters (renamed to avoid conflicts)
float robot_y = 0.0;
float robot_theta = 0.0;                    // robot orientation in radians

// Covariance matrix for the robot pose
// Stored as upper triangle: [σxx, σxy, σxθ, σyy, σyθ, σθθ]
float covariance[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // initial covariance values

// Motion command history to update the robot pose
float last_delta_rot1= 0.0; // last rotation before translation
float last_delta_trans = 0.0; // last translation
float last_delta_rot2 = 0.0; // last rotation after translation

// Timing for velocity calculations
unsigned long last_update_time = 0; // last time the pose was updated
bool first_update = true; // flag to handle first update

// Random number generator for sampling
float gaussianRandom(float mean, float std_dev){
    static bool has_spare = false;
    static float spare;
    if (has_spare) {
        has_spare = false;
        return spare * std_dev + mean;
    }
    has_spare = true;

    static float u, v, mag;
    do{
        u = random(-1000,1000) / 1000.0f; // random float in range [-1, 1]
        v = random(-1000,1000) / 1000.0f;
        mag = u * u + v * v;
    } while(mag >= 1.0f || mag == 0.0f); // ensure u, v are within unit circle
    mag = sqrt(-2.0f * log(mag) / mag); // Box-Muller transform
    spare = v * mag; // store for next call
    return mean + std_dev * u * mag; // return Gaussian random variable
}

// COvariance Propagation function
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

// covariance array to matrix
void covarianceToMatrix(const float cov[6], float matrix[9]) {
    matrix[0] = cov[0]; // σxx
    matrix[1] = cov[1]; // σxy
    matrix[2] = cov[2]; // σxθ
    matrix[3] = cov[1]; // σxy
    matrix[4] = cov[3]; // σyy
    matrix[5] = cov[4]; // σyθ
    matrix[6] = cov[2]; // σxθ
    matrix[7] = cov[4]; // σyθ
    matrix[8] = cov[5]; // σθθ
}

void matrixToCovariance(const float matrix[9], float cov[6]) {
    cov[0] = matrix[0]; // σxx
    cov[1] = matrix[1]; // σxy
    cov[2] = matrix[2]; // σxθ
    cov[3] = matrix[4]; // σyy
    cov[4] = matrix[5]; // σyθ
    cov[5] = matrix[8]; // σθθ
}

void initOdometry() {

    // Intialize covariance matrix to zero
    // for (int i = 0; i < 6; i++) {
    //     covariance[i] = 0.0;
    // }

    covariance[0] = 0.01; // σxx
    covariance[1] = 0.0;  // σxy
    covariance[2] = 0.0;  // σxθ    
    covariance[3] = 0.01; // σyy
    covariance[4] = 0.0;  // σyθ
    covariance[5] = 0.01; // σθθ

    last_update_time = millis();
    
    Serial.println("Odometry initialized : Probabilistic Motion Model with Encoder Noise");
}

void updateOdometry() {
    unsigned long current_time = millis();
    float dt = (current_time - last_update_time) / 1000.0; // convert ms to seconds
    if(first_update){
        first_update = false;
        last_update_time = current_time; // skip first update
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
    float dThetaL = (2.0 * PI * deltaLeft) / TICKS_PER_WHEEL_REV;
    float dThetaR = (2.0 * PI * deltaRight) / TICKS_PER_WHEEL_REV;

     // Calculate distances traveled by each wheel
    float dSLeft = dThetaL * WHEEL_RADIUS * BIAS_CORRCTION_FACTOR;
    float dSRight = dThetaR * WHEEL_RADIUS * BIAS_CORRCTION_FACTOR;

    // Calculate change in robot orientation and position
    float delta_trans = (dSLeft + dSRight) / 2.0;
    float delta_rot = (dSRight - dSLeft) / WHEEL_BASE;

    // for differential drive kinematics
    float delta_rot1 = 0.0;  // Initial rotation (for non-holonomic motion)
    float delta_rot2 = delta_rot;  // Final rotation

    if (fabs(delta_trans) > 0.001) {
        delta_rot1 = atan2(delta_trans * sin(delta_rot), delta_trans * cos(delta_rot)) - robot_theta;
        delta_rot2 = delta_rot - delta_rot1;
    }

    // store last deltas for next update
    last_delta_rot1 = delta_rot1;
    last_delta_trans = delta_trans;
    last_delta_rot2 = delta_rot2;

    // Deterministic Motion Model update (mean)
    float old_x = robot_x;
    float old_y = robot_y;
    float old_theta = robot_theta;

    // Update robot pose using differential drive kinematics
    robot_x += delta_trans * cos(robot_theta + delta_rot1);
    robot_y += delta_trans * sin(robot_theta + delta_rot1);
    robot_theta += delta_rot;
    
    // Normalize theta to [-PI, PI]
    while (robot_theta > PI) robot_theta -= 2.0 * PI;
    while (robot_theta < -PI) robot_theta += 2.0 * PI;

    // Probabilistic Motion Model update (covariance)
    // Motion model noise
    float rot1_var = ALPHA1 * fabs(delta_rot1) + ALPHA2 * fabs(delta_trans);
    float trans_var = ALPHA3 * fabs(delta_trans) + ALPHA4 * (fabs(delta_rot1) + fabs(delta_rot2));
    float rot2_var = ALPHA1 * fabs(delta_rot2) + ALPHA2 * fabs(delta_trans);

    // Encoder Noise
    float encoder_noise = ENCODER_NOISE_PER_TICK * (fabs(deltaLeft) + fabs(deltaRight));
    trans_var += encoder_noise;

    // jacobian of the motion model wrt pose
    float G[9] = {
        1, 0, -delta_trans * sin(old_theta + delta_rot1),
        0, 1, delta_trans * cos(old_theta + delta_rot1),
        0, 0, 1
    };

    // jacobian of the motion model wrt control
    float V[9] = {
        -delta_trans * sin(old_theta + delta_rot1), cos(old_theta + delta_rot1), -delta_trans * sin(old_theta + delta_rot1),
         delta_trans * cos(old_theta + delta_rot1), sin(old_theta + delta_rot1),  delta_trans * cos(old_theta + delta_rot1),
         1, 0, 1
    };

    // Motion model covariance matrix
    float M[9] = {
        rot1_var, 0, 0,
        0, trans_var, 0,
        0, 0, rot2_var
    };

    // covariance array to matrix
    float Sigma[9];
    covarianceToMatrix(covariance, Sigma);

    // Update covariance using the motion model
    float GT[9], VT[9];
    transposeMatrix3x3(G, GT);
    transposeMatrix3x3(V, VT);

    float temp1[9], temp2[9], temp3[9], temp4[9];
    matrixMultiply3x3(GT, Sigma, temp1); // GT * Sigma
    matrixMultiply3x3(temp1, G, temp2); // GT * Sigma * G
    matrixMultiply3x3(V, M, temp3); // V * M
    matrixMultiply3x3(temp3, VT, temp4); // GT * Sigma * G * V * M

    for (int i = 0; i < 9; i++) {
        temp2[i] += temp4[i]; // Sigma' = GT * Sigma * G + V * M * VT
    }
    matrixToCovariance(temp2, covariance); // convert back to covariance array

    // Sanity check for Positive Definiteness
    covariance[0] = max(covariance[0], 1e-6f); // σxx
    covariance[3] = max(covariance[3], 1e-6f); // σyy
    covariance[5] = max(covariance[5], 1e-6f); // σθθ
}

// sample from current pose with noise
void samplePose(float &sample_x, float &sample_y, float &sample_theta) {
    sample_x = gaussianRandom(robot_x, sqrt(covariance[0]));
    sample_y = gaussianRandom(robot_y, sqrt(covariance[3]));
    sample_theta = gaussianRandom(robot_theta, sqrt(covariance[5]));
    // Normalize theta to [-PI, PI]
    while (sample_theta > PI) sample_theta -= 2.0 * PI;
    while (sample_theta < -PI) sample_theta += 2.0 * PI;
}

void resetOdometry() {
    robot_x = 0.0;
    robot_y = 0.0;
    robot_theta = 0.0;
    prevTicksL = ticksL;
    prevTicksR = ticksR;

    // Reset covariance to initial values
    covariance[0] = 0.01; // σxx
    covariance[1] = 0.0;  // σxy
    covariance[2] = 0.0;  // σxθ    
    covariance[3] = 0.01; // σyy
    covariance[4] = 0.0;  // σyθ    
    covariance[5] = 0.01; // σθθ

    first_update = true; // reset first update flag
}

void printPose() {
    // Serial.print("Robot Pose - X: "); 
    // Serial.print(robot_x, 4);
    // Serial.print(" m, Y: "); 
    // Serial.print(robot_y, 4);
    // Serial.print(" m, Theta: "); 
    // Serial.print(robot_theta * 180.0 / PI, 2);
    // Serial.println(" degrees");

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

    // UDP output
    char buffer[150];
    snprintf(buffer, sizeof(buffer),
        "Robot Pose - X: %.4f ± %.4f m, Y: %.4f ± %.4f m, Theta: %.2f ± %.2f degrees",
        robot_x, sqrt(covariance[0]),
        robot_y, sqrt(covariance[3]),
        robot_theta * 180.0 / PI, sqrt(covariance[5]) * 180.0 / PI
    );

    udp.beginPacket(remoteIP, remotePort);
    udp.print(buffer);
    udp.endPacket();
}

void printCovariance() {
    Serial.println("Covariance Matrix:");
    Serial.printf("  [%8.6f %8.6f %8.6f]\n", covariance[0], covariance[1], covariance[2]);
    Serial.printf("  [%8.6f %8.6f %8.6f]\n", covariance[1], covariance[3], covariance[4]);
    Serial.printf("  [%8.6f %8.6f %8.6f]\n", covariance[2], covariance[4], covariance[5]);

    // UDP output
    // char buffer[100];
    // snprintf(buffer, sizeof(buffer),
    //     "Covariance Matrix:\n"
    //     "  [%.6f %.6f %.6f]\n"
    //     "  [%.6f %.6f %.6f]\n"
    //     "  [%.6f %.6f %.6f]",
    //     covariance[0], covariance[1], covariance[2],
    //     covariance[1], covariance[3], covariance[4],
    //     covariance[2], covariance[4], covariance[5]
    // );
    // udp.beginPacket(remoteIP, remotePort);
    // udp.print(buffer);
    // udp.endPacket();
}

void printMotionModel() {
    Serial.printf("Last Motion: rot1=%.4f, trans=%.4f, rot2=%.4f\n", 
                  last_delta_rot1, last_delta_trans, last_delta_rot2);
    // UDP output
    // char buffer[100];
    // snprintf(buffer, sizeof(buffer),
    //          "Last Motion: rot1=%.4f, trans=%.4f, rot2=%.4f",
    //          last_delta_rot1, last_delta_trans, last_delta_rot2);

    // udp.beginPacket(remoteIP, remotePort);
    // udp.print(buffer);
    // udp.endPacket();
}

// functions for accessing pose data
float getRobotX() { return robot_x; }
float getRobotY() { return robot_y; }
float getRobotTheta() { return robot_theta; }

// functiond for accessing covariance data
float getUncertaintyX() { return sqrt(covariance[0]); }
float getUncertaintyY() { return sqrt(covariance[3]); }
float getUncertaintyTheta() { return sqrt(covariance[5]); }

void getCovariance(float cov[6]){
    for(int i = 0; i < 6; i++) {
        cov[i] = covariance[i];
    }
}

void setNoiseParameters(float alpha1, float alpha2, float alpha3, float alpha4) {
    Serial.printf("Setting noise parameters: ALPHA1=%.4f, ALPHA2=%.4f, ALPHA3=%.4f, ALPHA4=%.4f\n", 
                  alpha1, alpha2, alpha3, alpha4);
    
                  // UDP output
    // char buffer[100];
    // snprintf(buffer, sizeof(buffer),
    //          "Setting noise parameters: ALPHA1=%.4f, ALPHA2=%.4f, ALPHA3=%.4f, ALPHA4=%.4f",
    //          alpha1, alpha2, alpha3, alpha4);
    // udp.beginPacket(remoteIP, remotePort);
    // udp.print(buffer);
    // udp.endPacket();
}

#endif // ODOMETRY_H