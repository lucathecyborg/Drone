#pragma once
#include <ICM_20948.h>
#include <Adafruit_AHRS_Mahony.h>

// ============================================================================
// I2C ADDRESS
// ============================================================================
// AD0_VAL = 1 → 0x69 (AD0 high/floating, Adafruit default)
// AD0_VAL = 0 → 0x68 (AD0 pulled to GND)
#define AD0_VAL 1

// ============================================================================
// HARDWARE DLPF CONFIGURATION
// ============================================================================
// The ICM-20948 contains on-chip digital low-pass filters that run before
// data is sent over I2C — before the ESP32, before any software filter.
// Enabling them is free noise rejection with zero CPU cost.
//
// Gyro DLPF:  119.5 Hz 3dB bandwidth
//   - Chosen to stay well below the Nyquist limit of our 250 Hz loop (125 Hz)
//   - Attenuates motor vibration noise (typically 150–400 Hz) by > –20 dB
//   - At 119.5 Hz cutoff, a PT1's phase lag at 80 Hz is ~34°, which at
//     250 Hz is only ~0.38 ms of delay — completely negligible for control
//   - SparkFun ICM-20948 enum: gyr_d119bw5_n154bw3
//
// Accel DLPF: 111.4 Hz 3dB bandwidth
//   - Same rationale as gyro. Accelerometer is noisier than gyro during flight.
//   - SparkFun ICM-20948 enum: acc_d111bw4_n136bw
//
// Both of these are datasheet-documented values from the ICM-20948 product
// specification, Table 17 (Gyro DLPF) and Table 19 (Accel DLPF).
// ============================================================================
#define GYRO_DLPF_SETTING gyr_d119bw5_n154bw3 // 119.5 Hz bandwidth
#define ACCEL_DLPF_SETTING acc_d111bw4_n136bw // 111.4 Hz bandwidth

// ============================================================================
// MAHONY FILTER TUNING
// ============================================================================
// The Adafruit AHRS Mahony filter uses two gains:
//
// Kp (proportional):
//   Controls how aggressively the filter corrects the gyro integration using
//   the accelerometer and magnetometer error vector each step.
//   - Too high (e.g. 10.0): filter over-trusts accel → motor vibrations
//     directly contaminate the angle estimate. This was the previous value
//     and is the primary cause of noisy angle output during flight.
//   - Too low (e.g. 0.1): filter barely uses accel → angles drift slowly
//     like a pure gyro integration.
//   - Adafruit default: 0.5f. This is the correct value for most IMUs.
//   - Reference: Mahony et al. 2008, "Nonlinear Complementary Filters on the
//     Special Orthogonal Group", IEEE Transactions on Automatic Control.
//
// Ki (integral):
//   Slowly estimates and corrects the gyro bias (the small constant offset
//   every gyro has). With Ki = 0 and no bias calibration, this offset
//   accumulates as yaw drift over time.
//   A small Ki (0.005) provides gentle long-term drift correction without
//   becoming unstable. The startup bias calibration (see initICM()) handles
//   the coarse offset; Ki handles the residual.
//   Adafruit default: 0.0f (we increase slightly).
// ============================================================================
#define MAHONY_KP 0.5f
#define MAHONY_KI 0.005f

// ============================================================================
// GYRO BIAS CALIBRATION
// ============================================================================
// Number of samples averaged at startup to estimate the gyro's resting offset.
// At 250 Hz the drone is still for ~2 seconds, giving 512 samples.
// This must be called while the drone is perfectly still on a flat surface.
// The average is subtracted from every subsequent gyro reading.
//
// Why this matters with Ki = 0.0:
//   Without bias calibration, a 1 °/s gyro offset (typical) accumulates to
//   60° of yaw drift per minute. Even with Ki = 0.005, convergence is slow.
//   Subtracting the measured bias at startup eliminates the bulk of the offset
//   immediately, leaving only small temperature-dependent residual drift for
//   Ki to handle.
// ============================================================================
#define GYRO_CALIB_SAMPLES 512

#define ACCEL_OFFSET_X -0.04868f
#define ACCEL_OFFSET_Y 0.10216f
#define ACCEL_OFFSET_Z 0.14659f
#define ACCEL_SCALE_X -0.03678f
#define ACCEL_SCALE_Y -0.00168f
#define ACCEL_SCALE_Z 1.00454f

// ============================================================================
// FUNCTION DECLARATIONS
// ============================================================================
bool initICM();
void updateIMU();

float getRoll();
float getPitch();
float getYaw();
float getGyroRateX(); // Bias-corrected, deg/s
float getGyroRateY();
float getGyroRateZ();

// ============================================================================
// EXTERNALLY ACCESSIBLE STATE
// ============================================================================
extern float currentRoll;
extern float currentPitch;
extern float currentYaw;

// Bias-corrected gyro rates in deg/s (fed into PID D term)
extern float gyroRateX;
extern float gyroRateY;
extern float gyroRateZ;

// Gyro bias offsets (computed at startup, subtracted from all readings)
extern float gyroBiasX;
extern float gyroBiasY;
extern float gyroBiasZ;

extern ICM_20948_I2C imu;
extern Adafruit_Mahony filter;