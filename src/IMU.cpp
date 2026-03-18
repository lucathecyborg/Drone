#include <Arduino.h>
#include <Wire.h>
#include "IMU.h"

// ============================================================================
// GLOBALS
// ============================================================================
float currentRoll = 0.0f;
float currentPitch = 0.0f;
float currentYaw = 0.0f;

// Body-frame gyro rates (deg/s), bias-corrected
float gyroRateX = 0.0f; // roll rate
float gyroRateY = 0.0f; // pitch rate
float gyroRateZ = 0.0f; // yaw rate

// Gyro bias (zero-rate offset), computed during calibration in initICM()
float gyroBiasX = 0.0f;
float gyroBiasY = 0.0f;
float gyroBiasZ = 0.0f;

Adafruit_ICM20948 imu;
Adafruit_Mahony filter;

// Counts how many updateIMU() calls have happened — used to switch Kp from
// convergence value down to flight value after MAHONY_CONVERGENCE_STEPS.
static uint32_t imuCallCount = 0;
static bool convergenceDone = false;

// ============================================================================
// initICM()
// ============================================================================
// Initialises the sensor, configures ranges and DLPF, calibrates gyro bias,
// and starts the Mahony filter at the high convergence gain.
// Call once from setup(). Drone must be stationary during this call.
// ============================================================================
bool initICM()
{
    // ── I2C init ─────────────────────────────────────────────────────────────
    // Default address is 0x69 (AD pin floating/high on Adafruit board).
    // If AD pin is pulled low, use: imu.begin_I2C(0x68, &Wire)
    if (!imu.begin_I2C())
    {
        Serial.println("[IMU] ICM-20948 not found — check wiring / I2C address.");
        return false;
    }

    // ── Sensor ranges ────────────────────────────────────────────────────────
    // ±8 g  — enough headroom for drone manoeuvres without saturating
    imu.setAccelRange(ICM20948_ACCEL_RANGE_8_G);

    // ±2000 °/s — full agility for aerobatic flying
    imu.setGyroRange(ICM20948_GYRO_RANGE_2000_DPS);

    // ── On-chip DLPF (hardware low-pass filter) ───────────────────────────────
    // The rate divisor controls the ODR of the sensor.
    // A divisor of 0 gives maximum ODR (1.1 kHz for gyro, 4.5 kHz for accel
    // before the DLPF). We run at 250 Hz in software so we want the hardware
    // DLPF to be set narrower than 125 Hz (Nyquist) to avoid aliasing.
    //
    // Divisor formula: ODR = base_rate / (1 + divisor)
    // Gyro base rate (DLPF on): ~1125 Hz. Divisor=3 → ~281 Hz internal rate.
    // At 281 Hz internal rate with the chip's DLPF engaged, the 3dB corner is
    // around 111 Hz — which sits just above our 250 Hz loop Nyquist (125 Hz).
    // This is intentional: we do our own PT1 filtering at 80 Hz in drone_main.
    //
    // Note: setGyroRateDivisor and setAccelRateDivisor also enable the DLPF
    // automatically on the ICM-20948.
    imu.setGyroRateDivisor(3);
    imu.setAccelRateDivisor(3);

    // ── Gyro bias calibration ─────────────────────────────────────────────────
    // Collect GYRO_CAL_SAMPLES readings while the drone is stationary.
    // The average is the zero-rate offset and is subtracted from all future reads.
    Serial.println("[IMU] Calibrating gyro bias — keep drone still...");
    {
        double sumX = 0, sumY = 0, sumZ = 0;
        sensors_event_t a, g, m, t;
        constexpr float kRadToDeg = 57.29577951f;

        for (int i = 0; i < GYRO_CAL_SAMPLES; i++)
        {
            // Wait for fresh data — poll until getEvent returns true
            while (!imu.getEvent(&a, &g, &t, &m))
            {
                delayMicroseconds(100);
            }

            // Accumulate in chip frame (we remap after computing bias)
            sumX += g.gyro.x;
            sumY += g.gyro.y;
            sumZ += g.gyro.z;

            delayMicroseconds(3000); // ~333 Hz — faster than loop, good sample density
        }

        // Average, convert to deg/s, then apply the same chip→body remap
        // that updateIMU() will use (chip Y → body X, chip X → body Y)
        // so the bias is directly subtractable in body frame.
        float rawBiasChipX = (float)(sumX / GYRO_CAL_SAMPLES) * kRadToDeg;
        float rawBiasChipY = (float)(sumY / GYRO_CAL_SAMPLES) * kRadToDeg;
        float rawBiasChipZ = (float)(sumZ / GYRO_CAL_SAMPLES) * kRadToDeg;

        // body X = chip Y, body Y = chip X, body Z = -chip Z
        gyroBiasX = rawBiasChipY;  // roll  bias
        gyroBiasY = rawBiasChipX;  // pitch bias
        gyroBiasZ = -rawBiasChipZ; // yaw   bias

        Serial.printf("[IMU] Gyro bias (body frame, deg/s): X=%.4f  Y=%.4f  Z=%.4f\n",
                      gyroBiasX, gyroBiasY, gyroBiasZ);
    }

    // ── Mahony filter ─────────────────────────────────────────────────────────
    // Start at high Kp for fast convergence, Ki for gyro bias integration.
    // Kp will be lowered to MAHONY_KP_FLIGHT after MAHONY_CONVERGENCE_STEPS.
    filter.begin(250); // sample rate must match loop rate
    filter.setKp(MAHONY_KP_CONVERGENCE);
    filter.setKi(MAHONY_KI);

    imuCallCount = 0;
    convergenceDone = false;

    Serial.printf("[IMU] Mahony started: Kp=%.1f (convergence)  Ki=%.3f\n",
                  MAHONY_KP_CONVERGENCE, MAHONY_KI);
    Serial.printf("[IMU] Will drop to Kp=%.2f after %d calls (~%.1f s)\n",
                  MAHONY_KP_FLIGHT,
                  MAHONY_CONVERGENCE_STEPS,
                  MAHONY_CONVERGENCE_STEPS / 250.0f);

    return true;
}

// ============================================================================
// updateIMU()   — call at exactly 250 Hz
// ============================================================================
//
// Pipeline per call:
//   1. Read sensor data
//   2. Remap chip axes → drone body frame
//   3. Subtract gyro bias (removes zero-rate offset)
//   4. Gate accelerometer (only trust accel within 0.9–1.1 g)
//   5. AK09916 → body frame magnetometer remap
//   6. Run Mahony filter (gyro always; accel/mag only if gated in)
//   7. Step-down Kp from convergence → flight value after warmup
//   8. Export roll/pitch/yaw
//
// ============================================================================
void updateIMU()
{
    sensors_event_t accel_evt, gyro_evt, mag_evt, temp_evt;

    if (!imu.getEvent(&accel_evt, &gyro_evt, &temp_evt, &mag_evt))
    {
        return; // No fresh data — skip this call
    }

    constexpr float kRadToDeg = 57.29577951f; // 180 / π
    constexpr float kG = 9.80665f;            // m/s² per g

    // ── STEP 1: Chip → body frame remap ─────────────────────────────────────
    //
    // Chip silkscreen (see IMU.h header for full diagram):
    //   chip X → drone RIGHT  = body Y (roll axis)
    //   chip Y → drone FRONT  = body X (pitch axis)
    //   chip Z → UP           = -body Z (body Z is down in NED)
    //
    // Gyro: Adafruit ICM20X returns rad/s — convert to deg/s here.
    float rawRoll = gyro_evt.gyro.y * kRadToDeg;  // chip Y → body X (roll)
    float rawPitch = gyro_evt.gyro.x * kRadToDeg; // chip X → body Y (pitch)
    float rawYaw = -gyro_evt.gyro.z * kRadToDeg;  // chip Z negated → body Z (yaw)

    // ── STEP 2: Subtract gyro bias ───────────────────────────────────────────
    // Bias was computed in initICM() in the same body frame, so subtraction is direct.
    gyroRateX = rawRoll - gyroBiasX;
    gyroRateY = rawPitch - gyroBiasY;
    gyroRateZ = rawYaw - gyroBiasZ;

    // ── STEP 3: Accel remap (chip → body frame, same rule as gyro) ───────────
    // Adafruit ICM20X returns m/s² directly — no unit conversion needed.
    float ax = accel_evt.acceleration.y;  // chip Y → body X
    float ay = accel_evt.acceleration.x;  // chip X → body Y
    float az = -accel_evt.acceleration.z; // chip Z negated → body Z (down)

    // ── STEP 4: Accel gating (Betaflight / PX4 technique) ───────────────────
    //
    // The Mahony filter uses the accelerometer to correct roll and pitch by
    // comparing the measured gravity vector with the estimated one. This only
    // works when the accelerometer is actually measuring gravity.
    //
    // During flight, centrifugal forces, motor vibration, and aerodynamic
    // loads push the total acceleration away from 1 g. Feeding corrupted
    // accel data into the filter degrades attitude accuracy.
    //
    // Solution: only trust the accel when the magnitude is within ±10% of 1 g.
    // Outside that band the filter runs on gyro integration only for that step.
    //
    // Reference: betaflight/src/main/flight/imu.c — imuMahonyAHRSupdate()
    //            PX4: src/lib/ecl/attitude_fw/ecl_fw_pos_controller.cpp
    //
    float accelMagSq = ax * ax + ay * ay + az * az;
    float accelMagG = sqrtf(accelMagSq) / kG; // in units of g

    bool accelValid = (accelMagG > ACCEL_GATE_LOW_G) &&
                      (accelMagG < ACCEL_GATE_HIGH_G);

    // ── STEP 5: Magnetometer remap ───────────────────────────────────────────
    //
    // The AK09916 embedded magnetometer has a different axis convention from
    // the ICM-20948 accel/gyro. The standard PX4 alignment is:
    //   mag_icm_frame = (mag.y, mag.x, -mag.z)
    //
    // Then applying our chip→body swap (chip Y → body X, chip X → body Y):
    //   body frame mag = (mag_icm.y, mag_icm.x, -mag_icm.z)
    //                  = (mag.x, mag.y, mag.z) after double-swap cancels XY
    //
    // Wait — let's be explicit. PX4 gives us ICM-frame mag as:
    //   icm_mx = mag.y,  icm_my = mag.x,  icm_mz = -mag.z
    // Then chip→body: body_X = icm_Y = mag.x,  body_Y = icm_X = mag.y
    //   body_mx =  mag.x
    //   body_my =  mag.y
    //   body_mz =  mag.z   (the two negations cancel: -mag.z from PX4, then
    //                        -body_Z from our Z flip → double negative = positive)
    //
    // Note: if yaw is still noisy after flashing, try swapping back to
    //   (mag.y, mag.x, -mag.z) — the exact alignment can vary by board revision.
    float mx = mag_evt.magnetic.x;
    float my = mag_evt.magnetic.y;
    float mz = mag_evt.magnetic.z;

    // ── STEP 6: Mahony filter update ─────────────────────────────────────────
    //
    // The Adafruit Mahony filter takes:
    //   gyro   in deg/s
    //   accel  in m/s² (any consistent unit — it normalises internally)
    //   mag    in any consistent unit (µT here)
    //
    // When accel is not valid we still pass it but set all components to zero
    // so the filter's accel cross-product correction computes to zero — this
    // is equivalent to gyro-only integration for this step.
    //
    if (accelValid)
    {
        filter.update(gyroRateX, gyroRateY, gyroRateZ,
                      ax, ay, az,
                      mx, my, mz);
    }
    else
    {
        // Gyro-only update — pass zero accel and mag so correction = 0.
        // The filter still integrates the quaternion from gyro alone.
        filter.update(gyroRateX, gyroRateY, gyroRateZ,
                      0.0f, 0.0f, 0.0f,
                      0.0f, 0.0f, 0.0f);
    }

    // ── STEP 7: Kp step-down after convergence warmup ────────────────────────
    //
    // High Kp during the first MAHONY_CONVERGENCE_STEPS calls snaps the filter
    // to the correct attitude quickly. Then we drop to the low flight Kp so the
    // filter trusts the gyro more during dynamic motion.
    //
    imuCallCount++;
    if (!convergenceDone && imuCallCount >= MAHONY_CONVERGENCE_STEPS)
    {
        filter.setKp(MAHONY_KP_FLIGHT);
        convergenceDone = true;
        Serial.printf("[IMU] Convergence done. Kp → %.2f (flight mode)\n",
                      MAHONY_KP_FLIGHT);
    }

    // ── STEP 8: Export angles ─────────────────────────────────────────────────
    currentRoll = filter.getRoll();
    currentPitch = filter.getPitch();
    currentYaw = filter.getYaw();
}

// ============================================================================
// Accessors
// ============================================================================
float getRoll() { return currentRoll; }
float getPitch() { return currentPitch; }
float getYaw() { return currentYaw; }

float getGyroRateX() { return gyroRateX; }
float getGyroRateY() { return gyroRateY; }
float getGyroRateZ() { return gyroRateZ; }