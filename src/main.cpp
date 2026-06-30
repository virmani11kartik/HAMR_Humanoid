#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include "imu_85.h"
#include "driver/twai.h"

// ---------------------- LED (M5Stamp C3 onboard) ----------------------
#define RGB_PIN 2
static Adafruit_NeoPixel led(1, RGB_PIN, NEO_GRB + NEO_KHZ800);

// ---------------------- IMU (I2C) -------------------------------------
static IMU_85 sense(1, 0); // SDA=GPIO1, SCL=GPIO0

// ---------------------- CAN (TWAI) pins & rate ------------------------
static const gpio_num_t CAN_TX_PIN = GPIO_NUM_19;
static const gpio_num_t CAN_RX_PIN = GPIO_NUM_18;
static const int        CAN_HZ     = 1000000; // 1 Mbit/s

// ---------------------- Motor protocol --------------------------------
static const uint8_t  MOTOR_ROLL  = 1;
static const uint8_t  MOTOR_PITCH = 2;
static const uint32_t TX_BASE     = 0x140;
static const uint32_t RX_BASE     = 0x240;

static const uint8_t CMD_SPEED_CONTROL = 0xA2;
static const uint8_t CMD_MOTOR_STOP    = 0x81;

// ---------------------- Gimbal speed limit ----------------------------
static const float GIMBAL_MAX_DPS = 200.0f;

// ---------------------- PID gains -------------------------------------
// Tuning order:
//   1. Raise ROLL_KP until fast response, just before oscillation starts
//   2. Add small ROLL_KI to eliminate steady-state offset
//   3. Add ROLL_KD last to damp overshoot — re-enable below when ready
//
// Roll
static const float ROLL_KP = 8.0f;
static const float ROLL_KI = 0.0f;
static const float ROLL_KD = 2.0f;

// Pitch
static const float PITCH_KP = 0.8f;
static const float PITCH_KI = 0.0f;
static const float PITCH_KD = 0.1f;

// ---------------------- Controller tuning constants -------------------
// EMA smoothing on the derivative term
static const float DERIV_ALPHA        = 0.30f;

// Deadzone — tight so small corrections aren't suppressed.
static const float ERROR_DEADZONE_DEG = 0.05f;

// Integrator clamp — small so KI can't wind too far while tuning.
static const float MAX_INTEGRATOR     =  10.0f;
static const float MIN_INTEGRATOR     = 10.0f;

// Per-axis speed clamps
static const float ROLL_MAX_DPS  = GIMBAL_MAX_DPS;
static const float PITCH_MAX_DPS = GIMBAL_MAX_DPS;

// Flip to -1.0f if an axis responds in the wrong direction
static const float ROLL_CMD_SIGN  = -1.0f;
static const float PITCH_CMD_SIGN =  1.0f;

// ---------------------- Loop timing -----------------------------------
// 100 Hz — consumes every BNO085 sample (IMU reports at 100 Hz).
// PRINT_EVERY=2 → 50 Hz log rate to keep serial from overwhelming the plotter.
static const uint16_t SAMPLE_MS   = 10;
static const uint8_t  PRINT_EVERY =  2;

// ---------------------- State flags -----------------------------------
static bool can_ok = false;
static bool imu_ok = false;
static bool target_ok = false;

// ---------------------- Data structures -------------------------------
struct AxisController {
    float kp;
    float ki;
    float kd;
    float max_dps;
    float cmd_sign;
    // Runtime state
    float integrator;
    float prev_error_deg;  // previous error for derivative
    float deriv_filtered;
    bool  seeded;
};

struct MotorStatus {
    float torque_A  = 0.0f;
    float speed_dps = 0.0f;
    float angle_deg = 0.0f;
    bool  valid     = false;
};

// ---------------------- Quaternion helpers ----------------------------

struct Quat { float w, x, y, z; };

static Quat q_target = { 1.0f, 0.0f, 0.0f, 0.0f };

static Quat quatConj(const Quat& q) {
    return { q.w, -q.x, -q.y, -q.z };
}

static Quat quatNormalize(const Quat& q) {
    float mag = sqrtf(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
    if (mag < 1e-6f) return { 1.0f, 0.0f, 0.0f, 0.0f };
    return { q.w / mag, q.x / mag, q.y / mag, q.z / mag };
}

static Quat quatMul(const Quat& p, const Quat& q) {
    return {
        p.w*q.w - p.x*q.x - p.y*q.y - p.z*q.z,
        p.w*q.x + p.x*q.w + p.y*q.z - p.z*q.y,
        p.w*q.y - p.x*q.z + p.y*q.w + p.z*q.x,
        p.w*q.z + p.x*q.y - p.y*q.x + p.z*q.w
    };
}

static float wrapDeg180(float deg) {
    while (deg >  180.0f) deg -= 360.0f;
    while (deg < -180.0f) deg += 360.0f;
    return deg;
}

// Extract roll and pitch error (deg) from quaternion error.
// q_err = conj(q_target) ⊗ q_current
// For identity target (level): q_err = q_current.
// BNO085 axis mapping: i → X = roll,  j → Y = pitch.
// If IMU is rotated, swap nx/ny or negate as needed.
static void quatErrorToRollPitch(const Quat& q_current,
                                 const Quat& q_target,
                                 float& roll_err_deg,
                                 float& pitch_err_deg)
{
    Quat q_err = quatMul(quatConj(q_target), q_current);

    // q and -q represent the same attitude. Keep the positive hemisphere so
    // axis-angle error always uses the shortest rotation near 180 deg.
    if (q_err.w < 0.0f) {
        q_err.w = -q_err.w;
        q_err.x = -q_err.x;
        q_err.y = -q_err.y;
        q_err.z = -q_err.z;
    }

    float w = q_err.w;
    if (w >  1.0f) w =  1.0f;
    if (w < -1.0f) w = -1.0f;

    float half_angle = acosf(w);
    float sin_ha     = sinf(half_angle);

    const float RAD2DEG = 57.2957795f;

    if (sin_ha < 1e-6f) {
        roll_err_deg  = 0.0f;
        pitch_err_deg = 0.0f;
        return;
    }

    float nx        = q_err.x / sin_ha;
    float ny        = q_err.y / sin_ha;
    float angle_deg = 2.0f * half_angle * RAD2DEG;

    roll_err_deg  = wrapDeg180(nx * angle_deg);
    pitch_err_deg = wrapDeg180(ny * angle_deg);
}

// ---------------------- TWAI helpers ----------------------------------
static bool twai_send_frame(uint32_t id, const uint8_t data[8]) {
    twai_message_t msg = {};
    msg.identifier       = id;
    msg.flags            = 0;
    msg.data_length_code = 8;
    memcpy(msg.data, data, 8);
    return twai_transmit(&msg, pdMS_TO_TICKS(5)) == ESP_OK;
}

static bool twai_recv_frame(uint32_t wantID, twai_message_t &out, uint32_t timeout_ms = 20) {
    uint32_t deadline = millis() + timeout_ms;
    while (millis() < deadline) {
        uint32_t rem = deadline - millis();
        twai_message_t m;
        if (twai_receive(&m, pdMS_TO_TICKS(rem > 0 ? rem : 1)) == ESP_OK) {
            if (m.identifier == wantID) { out = m; return true; }
        } else {
            break;
        }
    }
    return false;
}

// ---------------------- Motor commands --------------------------------
static MotorStatus sendSpeedCommand(uint8_t id, float speed_dps) {
    int32_t sc = (int32_t)lroundf(speed_dps * 100.0f);
    uint8_t tx[8] = {
        CMD_SPEED_CONTROL, 0x00, 0x00, 0x00,
        (uint8_t)(sc),
        (uint8_t)(sc >>  8),
        (uint8_t)(sc >> 16),
        (uint8_t)(sc >> 24)
    };
    twai_send_frame(TX_BASE + id, tx);

    twai_message_t rx;
    // 35 ms timeout — more headroom than original 15 ms to reduce dropouts
    if (!twai_recv_frame(RX_BASE + id, rx, 35) || rx.data[0] != CMD_SPEED_CONTROL) {
        return {};
    }

    MotorStatus s;
    s.torque_A  = (int16_t)(rx.data[2] | (rx.data[3] << 8)) * 0.01f;
    s.speed_dps = (float)  (int16_t)(rx.data[4] | (rx.data[5] << 8));
    s.angle_deg = (float)  (int16_t)(rx.data[6] | (rx.data[7] << 8));
    s.valid     = true;
    return s;
}

static void motorStop(uint8_t id) {
    uint8_t tx[8] = { CMD_MOTOR_STOP, 0, 0, 0, 0, 0, 0, 0 };
    twai_send_frame(TX_BASE + id, tx);
}

// ---------------------- CAN / TWAI init -------------------------------
static twai_timing_config_t timing_from_hz(int hz) {
    switch (hz) {
        case 1000000: return TWAI_TIMING_CONFIG_1MBITS();
        case  500000: return TWAI_TIMING_CONFIG_500KBITS();
        case  250000: return TWAI_TIMING_CONFIG_250KBITS();
        case  125000: return TWAI_TIMING_CONFIG_125KBITS();
        default:      return TWAI_TIMING_CONFIG_500KBITS();
    }
}

static void can_init() {
    twai_general_config_t g_config =
        TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
    twai_timing_config_t  t_config = timing_from_hz(CAN_HZ);
    twai_filter_config_t  f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
        Serial0.println("[CAN] driver_install failed");
        return;
    }
    if (twai_start() != ESP_OK) {
        Serial0.println("[CAN] start failed");
        return;
    }
    can_ok = true;
    Serial0.printf("[CAN] started at %d bit/s\n", CAN_HZ);
}

static bool captureLevelTarget(uint32_t timeout_ms = 1000) {
    uint32_t deadline = millis() + timeout_ms;
    while (millis() < deadline) {
        float qr = 0.0f, qi = 0.0f, qj = 0.0f, qk = 0.0f, acc = 0.0f;
        if (sense.getQuaternion(qr, qi, qj, qk, acc)) {
            q_target = quatNormalize({ qr, qi, qj, qk });
            target_ok = true;
            Serial0.printf(
                "[OK] Captured level target q=(%.4f, %.4f, %.4f, %.4f)\n",
                q_target.w, q_target.x, q_target.y, q_target.z
            );
            return true;
        }
        delay(10);
    }
    Serial0.println("[ERROR] Could not capture level target from IMU");
    return false;
}

// ---------------------- PID controller --------------------------------

static void seedAxis(AxisController &axis) {
    axis.integrator     = 0.0f;
    axis.prev_error_deg = 0.0f;
    axis.deriv_filtered = 0.0f;
    axis.seeded         = true;
}

// One PID step.
//
// Error pipeline:
//   raw quat error → EMA smooth (ERROR_ALPHA) → deadzone → P + I + D → clamp
//
// Derivative is computed on the smoothed error and then EMA-smoothed
// again (DERIV_ALPHA) before adding to output. This double-smoothing
// makes D usable at 100 Hz without amplifying IMU noise.
static float updateAxisSpeed(AxisController &axis, float error_deg, float dt) {
    if (!axis.seeded) seedAxis(axis);

    // Deadzone
    if (fabsf(error_deg) < ERROR_DEADZONE_DEG) error_deg = 0.0f;

    // Integrator with anti-windup
    axis.integrator += error_deg * dt;
    if (axis.integrator >  MAX_INTEGRATOR) axis.integrator =  MAX_INTEGRATOR;
    if (axis.integrator <  MIN_INTEGRATOR) axis.integrator =  MIN_INTEGRATOR;

    // Derivative on error, EMA-smoothed
    float safe_dt   = dt > 0.001f ? dt : 0.001f;
    float raw_deriv = (error_deg - axis.prev_error_deg) / safe_dt;
    axis.deriv_filtered =
        DERIV_ALPHA * raw_deriv + (1.0f - DERIV_ALPHA) * axis.deriv_filtered;
    axis.prev_error_deg = error_deg;

    float speed_dps =
        axis.kp * error_deg +
        axis.ki * axis.integrator +
        axis.kd * axis.deriv_filtered;

    speed_dps *= axis.cmd_sign;
    if (speed_dps >  axis.max_dps) speed_dps =  axis.max_dps;
    if (speed_dps < -axis.max_dps) speed_dps = -axis.max_dps;

    return speed_dps;
}

// ---------------------- Arduino setup ---------------------------------
void setup() {
    Serial0.begin(115200);
    delay(100);
    Serial0.println("\n[BOOT] ESP32-C3 Gimbal Controller (quaternion error, roll+pitch)");

    led.begin();
    led.setBrightness(128);
    led.clear();
    led.show();

    if (!sense.begin()) {
        Serial0.println("[ERROR] IMU init failed");
    } else {
        imu_ok = true;
        Serial0.println("[OK] IMU ready");
    }

    can_init();

    if (imu_ok && !captureLevelTarget()) {
        imu_ok = false;
    }

    if (imu_ok && can_ok && target_ok) {
        // Block until both motors reply — prevents control loop from running
        // before the motor is powered and ready, which would seed the
        // integrator with stale error and cause a lurch on first response.
        led.setPixelColor(0, led.Color(255, 180, 0));
        led.show();
        Serial0.println("[LED] Yellow: waiting for roll/pitch motors");

        Serial0.println("[BOOT] Waiting for roll/pitch motors...");
        while (true) {
            MotorStatus roll = sendSpeedCommand(MOTOR_ROLL, 0.0f);
            MotorStatus pitch = sendSpeedCommand(MOTOR_PITCH, 0.0f);
            if (roll.valid && pitch.valid) break;
            delay(100);
        }
        Serial0.println("[OK] Roll/pitch motors ready");

        led.setPixelColor(0, led.Color(20, 255, 20));
        led.show();
        Serial0.println("[LED] Green: system OK");
    } else {
        led.setPixelColor(0, led.Color(255, 0, 0));
        led.show();
        Serial0.println("[LED] Red: init error — check IMU/CAN wiring");
    }

    Serial0.println(
        "TUNE,time_ms"
        ",roll_approx_deg,roll_err_deg,roll_cmd_dps"
        ",pitch_approx_deg,pitch_err_deg,pitch_cmd_dps"
        ",roll_mot_speed,roll_mot_angle"
        ",pitch_mot_speed,pitch_mot_angle"
    );
}

// ---------------------- Arduino loop ----------------------------------
void loop() {
    static uint32_t t0       = millis();
    static uint32_t prev_ms  = millis();
    static uint32_t loop_seq = 0;

    static AxisController roll_axis = {
        ROLL_KP, ROLL_KI, ROLL_KD,
        ROLL_MAX_DPS, ROLL_CMD_SIGN,
        /*integrator*/     0.0f,
        /*prev_error_deg*/ 0.0f,
        /*deriv_filtered*/ 0.0f,
        /*seeded*/         false
    };

    static AxisController pitch_axis = {
        PITCH_KP, PITCH_KI, PITCH_KD,
        PITCH_MAX_DPS, PITCH_CMD_SIGN,
        0.0f, 0.0f, 0.0f, false
    };

    float qr = 0.0f, qi = 0.0f, qj = 0.0f, qk = 0.0f, acc = 0.0f;

    if (sense.getQuaternion(qr, qi, qj, qk, acc)) {
        uint32_t now_ms = millis();
        float dt = (now_ms - prev_ms) / 1000.0f;
        if (dt < 0.001f) dt = 0.001f;
        prev_ms = now_ms;

        Quat q_current = { qr, qi, qj, qk };

        float roll_err_deg  = 0.0f;
        float pitch_err_deg = 0.0f;
        quatErrorToRollPitch(q_current, q_target, roll_err_deg, pitch_err_deg);

        // Roll and pitch PID active
        float roll_dps  = updateAxisSpeed(roll_axis, roll_err_deg, dt);
        float pitch_dps = updateAxisSpeed(pitch_axis, pitch_err_deg, dt);

        MotorStatus roll_fb  = sendSpeedCommand(MOTOR_ROLL,  roll_dps);
        MotorStatus pitch_fb = sendSpeedCommand(MOTOR_PITCH, pitch_dps);

        if ((loop_seq++ % PRINT_EVERY) == 0) {
            const float RAD2DEG = 57.2957795f;
            float roll_approx  = 2.0f * qi * RAD2DEG;
            float pitch_approx = 2.0f * qj * RAD2DEG;

            Serial0.printf(
                "TUNE,%lu"
                ",%.3f,%.3f,%.3f"
                ",%.3f,%.3f,%.3f"
                ",%.1f,%.1f"
                ",%.1f,%.1f\n",
                (unsigned long)now_ms,
                roll_approx,  roll_err_deg,  roll_dps,
                pitch_approx, pitch_err_deg, pitch_dps,
                roll_fb.valid  ? roll_fb.speed_dps  : 0.0f,
                roll_fb.valid  ? roll_fb.angle_deg  : 0.0f,
                pitch_fb.valid ? pitch_fb.speed_dps : 0.0f,
                pitch_fb.valid ? pitch_fb.angle_deg : 0.0f
            );
        }
    } else {
        Serial0.println("[WARN] No IMU data — stopping motors");
        motorStop(MOTOR_ROLL);
        motorStop(MOTOR_PITCH);
    }

    // Fixed-rate loop pacing at 100 Hz
    uint32_t now  = millis();
    uint32_t next = t0 + SAMPLE_MS;
    if (now < next) delay(next - now);
    t0 = next;
}
