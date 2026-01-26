#include "IMU.h"

// Define global variables declared in IMU.h
float currentRoll = 0;
float currentPitch = 0;
float currentYaw = 0;
float gyroRateX = 0;
float gyroRateY = 0;
float gyroRateZ = 0;
Adafruit_ICM20948 icm;
Adafruit_Mahony filter;

bool initICM()
{
    if (!icm.begin_I2C())
    {
        return false;
    }

    icm.setAccelRange(ICM20948_ACCEL_RANGE_8_G);    // ±8g is good for drones
    icm.setGyroRange(ICM20948_GYRO_RANGE_2000_DPS); // ±2000°/s for aggressive flying

    // Configure sample rates
    icm.setAccelRateDivisor(3);
    icm.setGyroRateDivisor(3);
    icm.setMagDataRate(AK09916_MAG_DATARATE_100_HZ);

    // Initialize Mahony filter (250 Hz update rate)
    filter.begin(250);
    Serial.println("IMU and filter initialised");
    return true;
}

void updateIMU()
{

    sensors_event_t accel, gyro, mag, temp;
    icm.getEvent(&accel, &gyro, &mag, &temp);

    // Store raw gyro rates (in deg/s) - useful for rate mode or PID derivative
    gyroRateX = gyro.gyro.x * 57.2958; // Convert rad/s to deg/s
    gyroRateY = gyro.gyro.y * 57.2958;
    gyroRateZ = gyro.gyro.z * 57.2958;

    // Update Mahony filter with sensor data
    filter.update(
        gyroRateX, gyroRateY, gyroRateZ, // Gyro in deg/s
        accel.acceleration.x,            // Accel in m/s²
        accel.acceleration.y,
        accel.acceleration.z,
        mag.magnetic.x, // Mag in µT
        mag.magnetic.y,
        mag.magnetic.z);

    // Get calculated orientation angles
    currentRoll = filter.getRoll();   // degrees
    currentPitch = filter.getPitch(); // degrees
    currentYaw = filter.getYaw();     // degrees
}

float getRoll()
{
    return currentRoll;
}

float getPitch()
{
    return currentPitch;
}

float getYaw()
{
    return currentYaw;
}

float getGyroRateX()
{
    return gyroRateX;
}

float getGyroRateY()
{
    return gyroRateY;
}

float getGyroRateZ()
{
    return gyroRateZ;
}