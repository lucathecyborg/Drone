#pragma once

#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_AHRS_Mahony.h>

bool initICM();

void updateIMU();

float getRoll();
float getPitch();
float getYaw();
float getGyroRateX();
float getGyroRateY();
float getGyroRateZ();

// Current orientation (updated by function)
extern float currentRoll;
extern float currentPitch;
extern float currentYaw;

// Gyro rates (useful for PID derivative term)
extern float gyroRateX; // deg/s
extern float gyroRateY;
extern float gyroRateZ;

extern Adafruit_ICM20948 icm;
extern Adafruit_Mahony filter;
