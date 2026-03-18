#pragma once
#include <Adafruit_ICM20948.h>
#include <Adafruit_AHRS_Mahony.h>

// ============================================================================
// BOARD ORIENTATION — READ BEFORE CHANGING AXIS MAPPINGS
// ============================================================================
//
// Physical mounting (Adafruit ICM-20948 breakout, viewed from above on drone):
//
//                        FRONT of drone
//                            ^^^
//             [FS AD AC G SDO CS]   ← these pins face FRONT
//             ┌─────────────────┐
//             │  ICM20948       │
//             │    ↑Y           │
//             │    •→X          │   chip X → drone RIGHT
//             │                 │   chip Y → drone FRONT
//             │                 │   chip Z → UP (out of board)
//             └─────────────────┘
//             [VIN GND SDA SCL INT] ← these pins face BACK
//                            vvv
//                        BACK of drone
//
// Drone body-frame convention (NED — North/Forward-East/Right-Down):
//   body X = forward  (pitch axis, nose-up positive)
//   body Y = right    (roll axis,  right-wing-down positive)
//   body Z = down     (yaw axis,   CW-from-above positive)
//
// Chip → body mapping:
//   chip X (right)  → body Y  (roll axis)
//   chip Y (front)  → body X  (pitch axis)
//   chip Z (up)     → body Z via negation  (-chipZ = down = body Z)
//
// AK09916 magnetometer sits inside the ICM die with its own axis convention.
// The standard PX4 remapping aligns it to the accel/gyro frame:
//   mag_body_X =  mag.y
//   mag_body_Y =  mag.x
//   mag_body_Z = -mag.z
// Then applying our chip→body swap on top of that.
//
// ============================================================================

// ============================================================================
// MAHONY FILTER TUNING
// ============================================================================
//
// FLIGHT Kp (0.5):
//   Low proportional gain during flight. Accel is corrupted by vibration and
//   centrifugal forces — a low Kp means the filter mostly trusts the gyro
//   during dynamic motion, only gently correcting toward the accel reference.
//   Betaflight uses 0.25 when armed. 0.5 is a conservative starting point.
//
// CONVERGENCE Kp (10.0):
//   High gain used for the first N seconds after init so the filter locks onto
//   gravity quickly while the drone is still. Drops to FLIGHT_KP after warmup.
//
// Ki (0.005):
//   Integral gain estimates and corrects the gyro's DC bias (zero-rate offset).
//   Even a small Ki eliminates slow heading drift caused by gyro bias. Keep it
//   very small — too large causes slow oscillation of the attitude estimate.
//   Betaflight uses Ki only while spin rate is low to avoid windup in flight.
//
// ============================================================================
#define MAHONY_KP_FLIGHT 0.5f
#define MAHONY_KP_CONVERGENCE 10.0f
#define MAHONY_KI 0.005f

// How many updateIMU() calls to run at high Kp before dropping to flight Kp.
// At 250 Hz: 500 calls = 2 seconds of fast convergence.
#define MAHONY_CONVERGENCE_STEPS 500

// ============================================================================
// ACCEL GATING — Betaflight / PX4 technique
// ============================================================================
// During flight the accelerometer measures gravity PLUS centrifugal forces and
// vibration. If the total acceleration magnitude deviates significantly from 1g,
// the reading is unreliable for attitude correction and is discarded.
//
// Only feed accel into the Mahony correction when magnitude is within this band.
// Values outside the band: the filter runs on gyro only for that iteration.
//
// Betaflight uses 0.9–1.1g. PX4 uses the same. We match that.
// ============================================================================
#define ACCEL_GATE_LOW_G 0.9f  // Below this: free-fall or hard vibration
#define ACCEL_GATE_HIGH_G 1.1f // Above this: thrust / centrifugal load

// ============================================================================
// GYRO BIAS CALIBRATION
// ============================================================================
// On startup (during IMU warmup, while the drone is stationary), we collect
// GYRO_CAL_SAMPLES raw gyro readings and average them. This mean is the
// zero-rate offset (bias) and is subtracted from every subsequent reading.
//
// This is the single most effective way to eliminate yaw/roll/pitch drift
// when stationary. It does not help with temperature-dependent drift in flight,
// but eliminates the constant-offset component entirely.
// ============================================================================
#define GYRO_CAL_SAMPLES 500

// ============================================================================
// FUNCTION DECLARATIONS
// ============================================================================
bool initICM();   // Init sensor + run gyro bias calibration
void updateIMU(); // Call at 250 Hz — runs Mahony filter

float getRoll();
float getPitch();
float getYaw();
float getGyroRateX(); // Bias-corrected, deg/s
float getGyroRateY();
float getGyroRateZ();

// ── Exported state ──────────────────────────────────────────────────────────
extern float currentRoll;
extern float currentPitch;
extern float currentYaw;

// Gyro rates in deg/s, bias-corrected, body frame (used by PID D-term)
extern float gyroRateX; // roll rate  (right-wing-down positive)
extern float gyroRateY; // pitch rate (nose-up positive)
extern float gyroRateZ; // yaw rate   (CW-from-above positive)

// Calibrated gyro bias offsets (exported so drone_main can log them)
extern float gyroBiasX;
extern float gyroBiasY;
extern float gyroBiasZ;

extern Adafruit_ICM20948 imu;
extern Adafruit_Mahony filter;