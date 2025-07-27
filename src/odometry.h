#ifndef ODOMETRY_H
#define ODOMETRY_H
#include <Arduino.h>
#include <esp32-hal.h>
#include <esp32-hal-gpio.h>
#include <esp32-hal-ledc.h>

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

// State variables - using same names as main code
extern volatile long ticksL;                 // encoder ticks (defined in main)
extern volatile long ticksR;
long prevTicksL = 0;                        // previous ticks for delta calculation
long prevTicksR = 0;

float robot_x = 0.0;                        // robot position in meters (renamed to avoid conflicts)
float robot_y = 0.0;
float robot_theta = 0.0;                    // robot orientation in radians

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
    
    Serial.println("Odometry initialized");
}

void updateOdometry() {
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

    // Convert ticks to wheel rotations in radians
    float dThetaL = (2.0 * PI * deltaLeft) / TICKS_PER_REV;
    float dThetaR = (2.0 * PI * deltaRight) / TICKS_PER_REV;

    // Calculate distances traveled by each wheel
    float dSLeft = dThetaL * WHEEL_RADIUS;
    float dSRight = dThetaR * WHEEL_RADIUS;

    // Calculate change in robot orientation and position
    float dTheta = (dSRight - dSLeft) / WHEEL_BASE;
    float dS = (dSLeft + dSRight) / 2.0;

    // Update robot pose using differential drive kinematics
    robot_x += dS * cos(robot_theta + dTheta / 2.0);
    robot_y += dS * sin(robot_theta + dTheta / 2.0);
    robot_theta += dTheta;
    
    // Normalize theta to [-PI, PI]
    while (robot_theta > PI) robot_theta -= 2.0 * PI;
    while (robot_theta < -PI) robot_theta += 2.0 * PI;
}

void resetOdometry() {
    robot_x = 0.0;
    robot_y = 0.0;
    robot_theta = 0.0;
    prevTicksL = ticksL;
    prevTicksR = ticksR;
}

void printPose() {
    Serial.print("Robot Pose - X: "); 
    Serial.print(robot_x, 4);
    Serial.print(" m, Y: "); 
    Serial.print(robot_y, 4);
    Serial.print(" m, Theta: "); 
    Serial.print(robot_theta * 180.0 / PI, 2);
    Serial.println(" degrees");
}

// Getter functions for accessing pose data
float getRobotX() { return robot_x; }
float getRobotY() { return robot_y; }
float getRobotTheta() { return robot_theta; }

#endif // ODOMETRY_H