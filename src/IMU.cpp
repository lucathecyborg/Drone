#include <Arduino.h>
#include <Wire.h>
#include "IMU.h"

// Global variables declared in IMU.h
float currentRoll = 0;
float currentPitch = 0;
float currentYaw = 0;
float gyroRateX = 0;
float gyroRateY = 0;
float gyroRateZ = 0;
ICM_20948_I2C imu;
Adafruit_Mahony filter;

bool initICM()
{
    imu.begin(Wire, AD0_VAL);

    if (imu.status != ICM_20948_Stat_Ok)
    {
        Serial.print("ICM-20948 init failed: ");
        Serial.println(imu.statusString());
        return false;
    }

    // Configure full scale ranges
    ICM_20948_fss_t fss;
    fss.a = gpm8;    // ±8g — good for drones
    fss.g = dps2000; // ±2000°/s — for aggressive flying
    imu.setFullScale(ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr, fss);

    // Set sample mode to continuous
    imu.setSampleMode(ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr, ICM_20948_Sample_Mode_Continuous);

    // Initialize Mahony filter at 250 Hz with tuned gains
    filter.begin(250);
    filter.setKp(MAHONY_KP);
    filter.setKi(MAHONY_KI);

    Serial.println("IMU and filter initialised (SparkFun library)");
    Serial.printf("  Mahony Kp=%.1f  Ki=%.2f\n", MAHONY_KP, MAHONY_KI);
    return true;
}

void updateIMU()
{
    if (!imu.dataReady())
    {
        return; // No new data yet
    }

    imu.getAGMT(); // Read all sensors: accel, gyro, mag, temp

    // --- Gyro: SparkFun returns deg/s directly ---
    gyroRateX = imu.gyrX();
    gyroRateY = imu.gyrY();
    gyroRateZ = imu.gyrZ();

    // --- Accel: SparkFun returns mg, convert to m/s² ---
    float ax = imu.accX() * 0.001f * 9.81f;
    float ay = imu.accY() * 0.001f * 9.81f;
    float az = imu.accZ() * 0.001f * 9.81f;

    // --- Mag: AK09916 axis alignment to ICM-20948 accel/gyro frame ---
    // The AK09916 has a different axis convention than the accel/gyro.
    // PX4 uses: (mag.y, mag.x, -mag.z) to align them.
    float mx = imu.magY();
    float my = imu.magX();
    float mz = -imu.magZ();

    // Update Mahony filter
    filter.update(
        gyroRateX, gyroRateY, gyroRateZ,
        ax, ay, az,
        mx, my, mz);

    // Get orientation angles
    currentRoll = filter.getRoll();
    currentPitch = filter.getPitch();
    currentYaw = filter.getYaw();
}

float getRoll() { return currentRoll; }
float getPitch() { return currentPitch; }
float getYaw() { return currentYaw; }

float getGyroRateX() { return gyroRateX; }
float getGyroRateY() { return gyroRateY; }
float getGyroRateZ() { return gyroRateZ; }