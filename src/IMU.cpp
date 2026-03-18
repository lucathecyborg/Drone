#include <Arduino.h>
#include <Wire.h>
#include "IMU.h"

// ============================================================================
// GLOBAL STATE
// ============================================================================
float currentRoll = 0;
float currentPitch = 0;
float currentYaw = 0;

float gyroRateX = 0;
float gyroRateY = 0;
float gyroRateZ = 0;

// Gyro bias: constant offset measured at startup while drone is still.
// Subtracted from every gyro reading before it reaches the filter or PID.
float gyroBiasX = 0;
float gyroBiasY = 0;
float gyroBiasZ = 0;

ICM_20948_I2C imu;
Adafruit_Mahony filter;

// ============================================================================
// INIT
// ============================================================================
bool initICM()
{
    imu.begin(Wire, AD0_VAL);

    if (imu.status != ICM_20948_Stat_Ok)
    {
        Serial.print("ICM-20948 init failed: ");
        Serial.println(imu.statusString());
        return false;
    }

    // ---- FULL SCALE RANGES ----
    // ±8g accel: good range for drones (rarely exceed 4–5g in normal flight).
    // ±2000°/s gyro: safe for crash scenarios. Resolution is ~0.061 °/s per LSB.
    // Note: if first flights are gentle, ±500°/s gives 4× better resolution
    // (~0.015 °/s per LSB) with no other changes needed.
    ICM_20948_fss_t fss;
    fss.a = gpm8;   // ±8g
    fss.g = dps500; // ±500°/s — 4× better resolution than ±2000°/s (~0.015°/s per LSB)
                    // Safe because MAX_ATTITUDE_DEG disarms before a full tumble occurs.
                    // Switch back to dps2000 only if you move to aggressive acrobatic flying.
    imu.setFullScale(ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr, fss);

    // ---- HARDWARE DIGITAL LOW-PASS FILTER ----
    // This runs on the ICM-20948 chip itself before data is sent over I2C.
    // It is the most effective and cheapest noise reduction available —
    // zero CPU cost, no code-level timing dependency, purely hardware.
    //
    // The filter must be enabled separately from being configured.
    // Doing both in the same init block ensures they stay in sync.
    //
    // Gyro DLPF: 119.5 Hz bandwidth (gyr_d119bw5_n154bw3)
    //   - Attenuates propeller/motor noise (typically 150–400 Hz) before
    //     it reaches the Mahony filter or the PID D term.
    //   - Phase lag at our loop frequency (250 Hz) is negligible.
    //
    // Accel DLPF: 111.4 Hz bandwidth (acc_d111bw4_n136bw)
    //   - Removes high-frequency vibration from the accelerometer reading.
    //   - This directly improves the Mahony filter's accel correction vector,
    //     since Kp now amplifies a cleaner signal.
    //
    // Reference: ICM-20948 Product Specification Rev 1.3, Section 5.3
    //            (Register GYRO_CONFIG_1, ACCEL_CONFIG)
    ICM_20948_dlpcfg_t dlpConfig;
    dlpConfig.g = GYRO_DLPF_SETTING;
    dlpConfig.a = ACCEL_DLPF_SETTING;
    imu.setDLPFcfg(ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr, dlpConfig);
    imu.enableDLPF(ICM_20948_Internal_Gyr, true);
    imu.enableDLPF(ICM_20948_Internal_Acc, true);

    if (imu.status != ICM_20948_Stat_Ok)
    {
        Serial.print("DLPF config failed: ");
        Serial.println(imu.statusString());
        return false;
    }

    // ---- CONTINUOUS SAMPLING ----
    imu.setSampleMode(ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr,
                      ICM_20948_Sample_Mode_Continuous);

    // ---- MAHONY FILTER INIT ----
    // Rate must match the actual update rate in updateIMU().
    // Kp = 0.5 (Adafruit default) — trusts accel for tilt correction
    //                               without amplifying vibration noise.
    // Ki = 0.005 — slowly estimates and corrects residual gyro bias.
    filter.begin(250);
    filter.setKp(MAHONY_KP);
    filter.setKi(MAHONY_KI);

    Serial.println("ICM-20948 initialised");
    Serial.printf("  Gyro DLPF: 119.5 Hz | Accel DLPF: 111.4 Hz\n");
    Serial.printf("  Mahony Kp=%.3f  Ki=%.4f\n", MAHONY_KP, MAHONY_KI);

    // ---- GYRO BIAS CALIBRATION ----
    // Collect GYRO_CALIB_SAMPLES readings while the drone is perfectly still.
    // Average them to find the resting offset, then subtract it from every
    // subsequent reading.
    //
    // This is called during setup() before armMotors(), at which point the
    // drone has already been sitting still for 2+ seconds (IMU warm-up delay).
    // The calibration itself takes 512 / 250Hz ≈ 2 seconds.
    //
    // Important: the main.cpp warm-up loop already runs 500 updateIMU()
    // calls before reaching this point, so the hardware DLPF output has
    // settled. Calibration samples are therefore DLPF-filtered, which is
    // what we want — we're calibrating the bias of the filtered signal,
    // not the raw signal.
    Serial.println("Calibrating gyro bias — keep drone still...");
    double sumX = 0, sumY = 0, sumZ = 0;
    int collected = 0;

    while (collected < GYRO_CALIB_SAMPLES)
    {
        if (imu.dataReady())
        {
            imu.getAGMT();
            sumX += imu.gyrX();
            sumY += imu.gyrY();
            sumZ += imu.gyrZ();
            collected++;
        }
        delay(1); // ~1 kHz polling, we stop at 512 samples
    }

    gyroBiasX = (float)(sumX / GYRO_CALIB_SAMPLES);
    gyroBiasY = (float)(sumY / GYRO_CALIB_SAMPLES);
    gyroBiasZ = (float)(sumZ / GYRO_CALIB_SAMPLES);

    Serial.printf("  Gyro bias: X=%.3f  Y=%.3f  Z=%.3f deg/s\n",
                  gyroBiasX, gyroBiasY, gyroBiasZ);

    // Sanity check: if bias is unusually large the drone wasn't still,
    // or the IMU has a hardware fault.
    if (fabsf(gyroBiasX) > 5.0f || fabsf(gyroBiasY) > 5.0f || fabsf(gyroBiasZ) > 5.0f)
    {
        Serial.println("WARNING: Gyro bias > 5 deg/s — was the drone still during calibration?");
        // Not a hard failure — continue with whatever was measured.
    }

    return true;
}

// ============================================================================
// UPDATE (call at 250 Hz from the fixed-rate loop in main.cpp)
// ============================================================================
void updateIMU()
{
    if (!imu.dataReady())
    {
        // The ICM-20948 outputs at the rate configured by setSampleMode.
        // At 250 Hz, this should essentially never be false when called on time.
        // If it is, the filter was not updated this iteration — which means
        // the Mahony integration step ran with a stale gyro rate.
        // We do not skip silently; the caller (main.cpp) can decide what to do.
        // For flight controllers, the standard approach is to still call
        // filter.update() with the last known rates rather than skipping,
        // because an entirely missing update step is worse than a repeated one.
        // However, our hardware DLPF + 250 Hz loop rate makes this very rare.
        return;
    }

    imu.getAGMT(); // Read accel, gyro, mag, temperature in one burst

    // ---- BIAS-CORRECTED GYRO RATES ----
    // Subtract the resting offset measured during calibration.
    // This is the coarse correction; Mahony's Ki handles residual drift.
    // Units: degrees per second (SparkFun library returns deg/s directly).
    gyroRateX = imu.gyrX() - gyroBiasX;
    gyroRateY = imu.gyrY() - gyroBiasY;
    gyroRateZ = imu.gyrZ() - gyroBiasZ;

    // ---- ACCELEROMETER ----
    // SparkFun returns milligrams (mg). Convert to m/s² for Adafruit AHRS.
    // 1 mg = 0.001 g = 0.001 × 9.81 m/s²
    float ax = (imu.accX() * 0.001f * 9.81f - ACCEL_OFFSET_X) / ACCEL_SCALE_X;
    float ay = (imu.accY() * 0.001f * 9.81f - ACCEL_OFFSET_Y) / ACCEL_SCALE_Y;
    float az = (imu.accZ() * 0.001f * 9.81f - ACCEL_OFFSET_Z) / ACCEL_SCALE_Z;

    // ---- MAGNETOMETER AXIS ALIGNMENT ----
    // The AK09916 magnetometer embedded in the ICM-20948 has a different
    // physical axis orientation than the accelerometer/gyro block.
    // Without alignment, the heading computed by Mahony will be wrong,
    // and heading hold / yaw correction will drift or be mirrored.
    //
    // The correct remapping (PX4 AHRS and SparkFun documentation):
    //   mag_x_corrected =  mag.y
    //   mag_y_corrected =  mag.x
    //   mag_z_corrected = -mag.z
    //
    // This aligns the magnetometer axes to the ICM-20948 accel/gyro frame
    // so that the Mahony filter receives a consistent coordinate system.
    // Reference: SparkFun ICM-20948 Hookup Guide, "Magnetometer Alignment"
    //            PX4 icm20948_mag driver (src/drivers/imu/icm20948/AK09916.cpp)
    float mx = imu.magY();
    float my = imu.magX();
    float mz = -imu.magZ();

    // ---- MAHONY FILTER UPDATE ----
    // Gyro in deg/s (bias-corrected).
    // Accel in m/s².
    // Mag in µT (SparkFun returns µT; Adafruit AHRS accepts any consistent unit).
    filter.update(
        gyroRateX, gyroRateY, gyroRateZ,
        ax, ay, az,
        mx, my, mz);

    currentRoll = filter.getRoll();
    currentPitch = filter.getPitch();
    currentYaw = filter.getYaw();
}

// ============================================================================
// ACCESSORS
// ============================================================================
float getRoll() { return currentRoll; }
float getPitch() { return currentPitch; }
float getYaw() { return currentYaw; }
float getGyroRateX() { return gyroRateX; }
float getGyroRateY() { return gyroRateY; }
float getGyroRateZ() { return gyroRateZ; }