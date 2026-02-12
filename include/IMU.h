#pragma once
#include <ICM_20948.h>            // SparkFun ICM-20948 library
#include <Adafruit_AHRS_Mahony.h> // Adafruit Mahony filter

// ============================================================================
// I2C ADDRESS CONFIGURATION
// ============================================================================
// AD0_VAL = 1 → address 0x69 (Adafruit default, AD0 pin floating/high)
// AD0_VAL = 0 → address 0x68 (AD0 pin pulled to GND)
#define AD0_VAL 1

// ============================================================================
// MAHONY FILTER TUNING
// ============================================================================
// Kp: Proportional gain — higher = faster convergence, more noise
// Ki: Integral gain    — corrects steady-state drift, keep small
#define MAHONY_KP 10.0f
#define MAHONY_KI 0.0f

// ============================================================================
// FUNCTION DECLARATIONS
// ============================================================================

bool initICM();
void updateIMU();

float getRoll();
float getPitch();
float getYaw();
float getGyroRateX();
float getGyroRateY();
float getGyroRateZ();

// Current orientation (updated by updateIMU)
extern float currentRoll;
extern float currentPitch;
extern float currentYaw;

// Gyro rates in deg/s (useful for PID derivative term)
extern float gyroRateX;
extern float gyroRateY;
extern float gyroRateZ;

extern ICM_20948_I2C imu;
extern Adafruit_Mahony filter;