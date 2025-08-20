# LINK TO HIGH LEVEL PLANNER REPO IN ROS2
https://github.com/cedrichld/hamr_holonomic_robot/tree/kartik

# LOW LEVEL PLANNER SOURCE CODE IN THIS REPO
# HOLONOMIC AFFORDABLE MOBILE ROBOT (HAMR) at University of Pennsylvania MODLAB
Overview:
HAMR is a modular holonomic mobile robot designed for agile, off-road locomotion with a self-balancing torso. This project focuses on mechanical robustness, precise control, and real-time localization.


🛠️ Key Features
Modular Mechatronics Design: Engineered for omnidirectional movement and terrain adaptability.
Drive Control & Stability: Custom algorithms for holonomic drive coordination and torso balance.
Localization: Implemented odometry and sensor fusion using Extended and Unscented Kalman Filters (EKF/UKF).
Electronics Integration: End-to-end selection, mounting, and wiring of sensors, motor drivers, and microcontrollers.
In-House Fabrication: 3D-printed and CNC-machined components for a lightweight yet durable structure.
ROS Compatibility: Full software stack developed with ROS for communication, control, and future autonomy modules.


📦 Technologies Used
ROS (Robot Operating System)
C++ / Python
EKF / UKF for sensor fusion
Additive Manufacturing + CNC Machining
ESP32, Motor Drivers, IMU, Encoders

## PID TUNING OF DRIVE MOTORS

## ELECTROMECHANICAL BOM
https://docs.google.com/spreadsheets/d/1NsM1mKOt5aUw6rH_zVvXBUCNo1m5PK5jaZNAQDpiI4U/edit?usp=sharing

## ODOMETRY MOTION MODEL (USING ENCODER FEEDBACK)

## 🚀 What It Does

Given:
- Robot pose at time `t-1`: \( (x, y, \theta) \)
- Odometry readings (wheel encoder values) at time `t` and `t-1`

It:
1. Computes deterministic motion components:
   - Initial rotation (`rot1`)
   - Translation (`trans`)
   - Final rotation (`rot2`)
2. Adds Gaussian noise to each motion component using a parameterized noise model
3. Applies the noisy motion to compute a probabilistic estimate of the new pose \( (x_t, y_t, \theta_t) \)

---
## 📦 Motion Model Details

### 1. Deterministic Motion Decomposition

From encoder data:
## 🚀 What It Does

Given:
- Robot pose at time `t-1`: \( (x, y, \theta) \)
- Odometry readings (wheel encoder values) at time `t` and `t-1`

It:
1. Computes deterministic motion components:
   - Initial rotation (`rot1`)
   - Translation (`trans`)
   - Final rotation (`rot2`)
2. Adds Gaussian noise to each motion component using a parameterized noise model
3. Applies the noisy motion to compute a probabilistic estimate of the new pose \( (x_t, y_t, \theta_t) \)

---

## 📦 Motion Model Details

### 1. Deterministic Motion Decomposition

From encoder data:
delta_rot1 = atan2(y' - y, x' - x) - theta
delta_trans = sqrt((x' - x)^2 + (y' - y)^2)
delta_rot2 = theta' - theta - delta_rot1


### 2. Noise Model

Motion component variances:

σ_rot1² = α1 * delta_rot1² + α2 * delta_trans²

σ_trans² = α3 * delta_trans² + α4 * (delta_rot1² + delta_rot2²)

σ_rot2² = α1 * delta_rot2² + α2 * delta_trans²


You sample noise from:

rot1_hat = rot1 + N(0, σ_rot1²)

trans_hat = trans + N(0, σ_trans²)

rot2_hat = rot2 + N(0, σ_rot2²)


### 3. Apply Noisy Motion

Final pose update:

x' = x + trans_hat * cos(theta + rot1_hat)

y' = y + trans_hat * sin(theta + rot1_hat)

theta' = theta + rot1_hat + rot2_hat

# EKF-Based Sensor Fusion for Mobile Robot Localization (Odometry + IMU)

This repository's branch also implements a probabilistic motion model and Extended Kalman Filter (EKF) to fuse encoder odometry with IMU-based heading estimation for robust 2D localization of a mobile robot.

## 🚗 Project Overview

The robot uses:
- **Wheel Encoders** for translational motion (x, y)
- **IMU (BNO055/MPU6050/etc.)** for heading (yaw/θ)
- A **probabilistic motion model** to introduce realistic motion uncertainty
- An **EKF** to fuse the encoder and IMU data into a better pose estimate

---

## 📈 State Estimation

State vector:

x = [x_position, y_position, heading_angle (theta)]


Sensor inputs:
- Encoder-derived pose delta (`Δx`, `Δy`, `Δθ`)
- IMU absolute or delta yaw (`θ_IMU`)

---

## 🧠 Algorithm Workflow

### 1. **Probabilistic Odometry Motion Model**

Implements noisy motion update from encoder odometry:
- Estimate `rot1`, `trans`, `rot2` from t-1 to t
- Sample motion noise from tuned Gaussian distributions
- Apply noisy motion model to predict new pose

### 2. **EKF Prediction Step**

Predict pose using noisy odometry model:

x̄ₜ = f(xₜ₋₁, uₜ) # uₜ from encoders
P̄ₜ = Fₜ * Pₜ₋₁ * Fₜᵀ + Qₜ


### 3. **EKF Update Step (with IMU heading)**

Update belief with IMU:

zₜ = θ_IMU
yₜ = zₜ - h(x̄ₜ)
Kₜ = P̄ₜ * Hᵀ * (H * P̄ₜ * Hᵀ + R)⁻¹
xₜ = x̄ₜ + Kₜ * yₜ
Pₜ = (I - Kₜ * H) * P̄ₜ

