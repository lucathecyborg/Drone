#pragma once
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>
#include <math.h>

// ============================================================================
// I2C ADDRESS
// ============================================================================
// Adafruit ICM-20948 board: AD0 pad floating/high → 0x69
// Bridge the AD0 pad to GND for 0x68.
#define ICM_I2C_ADDR 0x69

// ============================================================================
// HARDWARE DLPF
// ============================================================================
// On-chip filters. Run before data leaves the sensor — zero CPU cost.
// Gyro:  119.5 Hz bandwidth  (ICM20X_GYRO_FREQ_119_5_HZ)
// Accel: 111.4 Hz bandwidth  (ICM20X_ACCEL_FREQ_111_4_HZ)
// Both stay well below the 125 Hz Nyquist limit of the 250 Hz loop.
#define GYRO_DLPF_CUTOFF ICM20X_GYRO_FREQ_119_5_HZ
#define ACCEL_DLPF_CUTOFF ICM20X_ACCEL_FREQ_111_4_HZ

// ============================================================================
// MAHONY FILTER GAINS
// ============================================================================
// MAHONY_KP_INIT: high gain for the first MAHONY_INIT_STEPS iterations.
//   Pulls the quaternion from identity (flat) to the real attitude quickly.
//   Source: madflight AHRS, Betaflight imuCalculateEstimatedAttitude().
//   Both use a high initial Kp for ~2 s to avoid a slow startup convergence.
//
// MAHONY_KP: steady-state flight gain.
//   0.5 is the x-io Technologies reference default (Mahony 2008 paper).
//   Too high → motor vibration contaminates roll/pitch.
//   Too low  → gyro drift is not corrected.
//
// MAHONY_KI: integral gain. Slowly removes residual gyro bias not caught by
//   startup calibration (temperature drift etc.). 0.005 is safe.
//
// MAHONY_INIT_STEPS: how many update() calls to run at MAHONY_KP_INIT.
//   250 Hz × 2 s = 500 steps.
#define MAHONY_KP_INIT 10.0f
#define MAHONY_KP 0.5f
#define MAHONY_KI 0.005f
#define MAHONY_INIT_STEPS 500

// ============================================================================
// ACCEL GATING
// ============================================================================
// Only apply the accelerometer correction step when the measured acceleration
// magnitude is between ACCEL_GATE_MIN and ACCEL_GATE_MAX m/s².
//
// Source: madflight MAHONY_BF (Betaflight-flavoured Mahony), which gates
//   accel correction to "rest only". The comment in madflight reads:
//   "only use accel when close to 1g, otherwise centrifugal force etc."
//
// 0.9g = 8.83 m/s²   (lower bound)
// 1.1g = 10.79 m/s²  (upper bound)
//
// During aggressive flight: motors vibrate, body accelerates laterally —
//   the accel vector is far from 1g and should NOT be trusted.
// At rest or gentle hover: accel ≈ 1g and is a reliable gravity reference.
//
// This is the single most impactful change for reducing attitude drift
// during motor-on testing and flight.
#define ACCEL_GATE_MIN 8.83f  // 0.9 × 9.81
#define ACCEL_GATE_MAX 10.79f // 1.1 × 9.81

// ============================================================================
// GYRO BIAS CALIBRATION
// ============================================================================
#define GYRO_CALIB_SAMPLES 512 // ~2 s at 250 Hz

// ============================================================================
// ACCELEROMETER CALIBRATION
// ============================================================================
// Re-measure with ACCEL_CALIBRATION.ino after confirming axes are correct.
// Until then: offsets=0 / scales=1 = safe uncorrected state.
#define ACCEL_OFFSET_X 0.0f
#define ACCEL_OFFSET_Y 0.0f
#define ACCEL_OFFSET_Z 0.0f
#define ACCEL_SCALE_X 1.0f
#define ACCEL_SCALE_Y 1.0f
#define ACCEL_SCALE_Z 1.0f

// ============================================================================
// MAGNETOMETER HARD-IRON CALIBRATION
// ============================================================================
// Computed from full 360° flat yaw rotation using MAG_CHECK.ino.
// Offsets = midpoint of (min, max) observed on each axis.
// Re-run MAG_CHECK after confirming axes are correct.
#define MAG_OFFSET_X -18.74f
#define MAG_OFFSET_Y 17.48f
#define MAG_OFFSET_Z 2.17f

#define MAG_SCALE_XX +1.037f
#define MAG_SCALE_XY +0.002f
#define MAG_SCALE_XZ +0.001f
#define MAG_SCALE_YX +0.002f
#define MAG_SCALE_YY +0.942f
#define MAG_SCALE_YZ -0.009f
#define MAG_SCALE_ZX +0.001f
#define MAG_SCALE_ZY -0.009f
#define MAG_SCALE_ZZ +1.023f

// ============================================================================
// LOOP RATE
// ============================================================================
#define IMU_UPDATE_HZ 250.0f
#define IMU_DT (1.0f / IMU_UPDATE_HZ) // 0.004 s

// ============================================================================
// FUNCTION DECLARATIONS
// ============================================================================
bool initICM();
void updateIMU();

// ---- Euler angles (degrees) ----
// Sign conventions — VERIFY with visualizer before first flight:
//   getRoll()  > 0 → right wing DOWN  (rolling clockwise from front)
//   getPitch() > 0 → nose UP
//   getYaw()   > 0 → nose turning RIGHT (CW from above), range 0–360
float getRoll();
float getPitch();
float getYaw();

// ---- Gyro rates (degrees/second) ----
// Sign conventions match the angle sign conventions above:
//   getGyroRateX() > 0 → rolling right
//   getGyroRateY() > 0 → pitching nose up
//   getGyroRateZ() > 0 → turning right (CW from above)
float getGyroRateX();
float getGyroRateY();
float getGyroRateZ();

// ============================================================================
// SHARED STATE
// ============================================================================
extern float currentRoll;
extern float currentPitch;
extern float currentYaw;

extern float gyroRateX;
extern float gyroRateY;
extern float gyroRateZ;

// Gyro bias in rad/s, raw sensor frame, measured at startup
extern float gyroBiasX;
extern float gyroBiasY;
extern float gyroBiasZ;

extern Adafruit_ICM20948 icm;