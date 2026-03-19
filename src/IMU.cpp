#include <Arduino.h>
#include <Wire.h>
#include "IMU.h"

// ============================================================================
// GLOBAL STATE
// ============================================================================
float currentRoll = 0.0f;
float currentPitch = 0.0f;
float currentYaw = 0.0f;

float gyroRateX = 0.0f;
float gyroRateY = 0.0f;
float gyroRateZ = 0.0f;

float gyroBiasX = 0.0f;
float gyroBiasY = 0.0f;
float gyroBiasZ = 0.0f;

Adafruit_ICM20948 icm;

// ============================================================================
// MAHONY FILTER STATE
// ============================================================================
// Unit quaternion representing current orientation.
// Initialised to identity = level, nose pointing forward.
static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;

// Ki integral error accumulators (one per body axis)
static float intErrX = 0.0f, intErrY = 0.0f, intErrZ = 0.0f;

// Startup convergence counter (for dynamic Kp)
static int mahonyInitCount = 0;

// ============================================================================
// MAHONY AHRS  — 9-DOF implementation
// ============================================================================
// Based on the reference implementation by S.O.H. Madgwick and x-io Technologies.
// Original C source: www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/
//
// Coordinate convention: NWU body frame
//   X = forward (nose direction)
//   Y = left    (port side)
//   Z = up
//
// This matches the sensor → filter mapping derived from AXIS_CHECK.ino:
//   Sensor X = LEFT,  Sensor Y = BACKWARD,  Sensor Z = UP
//   → filter_x = −sensorY   (forward  = −backward)
//   → filter_y = +sensorX   (left     = +sensorX)
//   → filter_z = +sensorZ   (up       = +sensorZ)
//
// Inputs:
//   gx/gy/gz : body-frame gyro rates, rad/s
//   ax/ay/az : body-frame accel,      m/s² (any consistent unit)
//   mx/my/mz : body-frame mag,        µT   (any consistent unit)
//
// ---- ACCEL GATING (madflight/Betaflight feature) ----
// The accel correction is only applied when |accel| is between 0.9g and 1.1g.
// When a motor spins up, lateral acceleration or vibration pushes |accel|
// outside this window and the filter ignores it for that iteration.
// Source: madflight AHRS documentation, Betaflight imuIsAccelerometerHealthy().
//
// ---- DYNAMIC Kp (madflight/Betaflight feature) ----
// For the first MAHONY_INIT_STEPS iterations Kp is set to MAHONY_KP_INIT (10.0).
// This rapidly converges the quaternion from identity to the real attitude.
// Afterward Kp drops to MAHONY_KP (0.5) for quiet, noise-resistant operation.
// Source: madflight AHRS: "use a high gain initially to converge fast."
// ============================================================================

static inline float invSqrt(float x)
{
    return 1.0f / sqrtf(x);
}

static void mahonyUpdate(float gx, float gy, float gz,
                         float ax, float ay, float az,
                         float mx, float my, float mz)
{
    // ---- DYNAMIC Kp ----
    float kp = (mahonyInitCount < MAHONY_INIT_STEPS) ? MAHONY_KP_INIT : MAHONY_KP;
    if (mahonyInitCount < MAHONY_INIT_STEPS)
        mahonyInitCount++;

    float recipNorm;
    float hx, hy, bx, bz;
    float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
    float halfex = 0.0f, halfey = 0.0f, halfez = 0.0f;
    float qa, qb, qc;

    // ---- ACCEL GATING ----
    // Only apply accel correction when |accel| is within ±10% of 1g.
    float accelMag = sqrtf(ax * ax + ay * ay + az * az);
    bool useAccel = (accelMag > ACCEL_GATE_MIN && accelMag < ACCEL_GATE_MAX);

    // ---- MAG: use only if non-zero ----
    bool useMag = (mx != 0.0f || my != 0.0f || mz != 0.0f);

    if (useAccel || useMag)
    {
        if (useAccel)
        {
            // Normalise accelerometer
            recipNorm = invSqrt(accelMag * accelMag); // already have mag
            // Re-normalise with the same value
            recipNorm = 1.0f / accelMag;
            ax *= recipNorm;
            ay *= recipNorm;
            az *= recipNorm;
        }

        if (useMag)
        {
            // Normalise magnetometer
            recipNorm = invSqrt(mx * mx + my * my + mz * mz);
            mx *= recipNorm;
            my *= recipNorm;
            mz *= recipNorm;

            // Reference direction of Earth's magnetic field (in body frame)
            hx = 2.0f * (mx * (0.5f - q2 * q2 - q3 * q3) + my * (q1 * q2 - q0 * q3) + mz * (q1 * q3 + q0 * q2));
            hy = 2.0f * (mx * (q1 * q2 + q0 * q3) + my * (0.5f - q1 * q1 - q3 * q3) + mz * (q2 * q3 - q0 * q1));
            bx = sqrtf(hx * hx + hy * hy); // horizontal component only (removes dip angle)
            bz = 2.0f * (mx * (q1 * q3 - q0 * q2) + my * (q2 * q3 + q0 * q1) + mz * (0.5f - q1 * q1 - q2 * q2));

            // Estimated mag direction (reference) in body frame
            halfwx = bx * (0.5f - q2 * q2 - q3 * q3) + bz * (q1 * q3 - q0 * q2);
            halfwy = bx * (q1 * q2 - q0 * q3) + bz * (q0 * q1 + q2 * q3);
            halfwz = bx * (q1 * q3 + q0 * q2) + bz * (0.5f - q1 * q1 - q2 * q2);
        }

        // Estimated gravity direction (reference) in body frame
        halfvx = q1 * q3 - q0 * q2;
        halfvy = q0 * q1 + q2 * q3;
        halfvz = q0 * q0 - 0.5f + q3 * q3;

        // Error = cross product of estimated vs measured direction
        if (useAccel)
        {
            halfex += (ay * halfvz - az * halfvy);
            halfey += (az * halfvx - ax * halfvz);
            halfez += (ax * halfvy - ay * halfvx);
        }
        if (useMag)
        {
            halfex += (my * halfwz - mz * halfwy);
            halfey += (mz * halfwx - mx * halfwz);
            halfez += (mx * halfwy - my * halfwx);
        }

        // Ki integral term
        intErrX += MAHONY_KI * halfex * IMU_DT;
        intErrY += MAHONY_KI * halfey * IMU_DT;
        intErrZ += MAHONY_KI * halfez * IMU_DT;

        // Apply proportional + integral correction to gyro
        gx += kp * halfex + intErrX;
        gy += kp * halfey + intErrY;
        gz += kp * halfez + intErrZ;
    }

    // ---- QUATERNION INTEGRATION ----
    gx *= 0.5f * IMU_DT;
    gy *= 0.5f * IMU_DT;
    gz *= 0.5f * IMU_DT;
    qa = q0;
    qb = q1;
    qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);

    // Normalise quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}

// ============================================================================
// INIT
// ============================================================================
bool initICM()
{
    // The Adafruit ICM20X library initialises the AK09916 magnetometer
    // automatically during begin_I2C(). No separate startupMagnetometer()
    // call is needed — this was the main reason for switching libraries.
    if (!icm.begin_I2C(ICM_I2C_ADDR, &Wire))
    {
        Serial.println("ICM-20948 init failed — check wiring and I2C address.");
        return false;
    }

    // ---- FULL SCALE RANGES ----
    icm.setAccelRange(ICM20948_ACCEL_RANGE_8_G);   // ±8g
    icm.setGyroRange(ICM20948_GYRO_RANGE_500_DPS); // ±500 deg/s

    // ---- SAMPLE RATE ----
    // Set divisors to 0 = highest sample rate from the sensor.
    // Our 250 Hz control loop will read whenever data is ready.
    icm.setAccelRateDivisor(0);
    icm.setGyroRateDivisor(0);

    // ---- MAGNETOMETER RATE ----
    icm.setMagDataRate(AK09916_MAG_DATARATE_100_HZ);

    // ---- HARDWARE DLPF ----
    // On-chip filters — free noise rejection before data leaves the sensor.
    icm.enableAccelDLPF(true, ACCEL_DLPF_CUTOFF);
    icm.enableGyrolDLPF(true, GYRO_DLPF_CUTOFF);

    Serial.println("ICM-20948 initialised (Adafruit ICM20X library).");
    Serial.println("  Gyro DLPF: 119.5 Hz | Accel DLPF: 111.4 Hz | Gyro FSR: ±500 deg/s");
    Serial.printf("  Mahony Kp=%.1f (init) → %.2f (flight) | Ki=%.4f\n",
                  MAHONY_KP_INIT, MAHONY_KP, MAHONY_KI);
    Serial.printf("  Accel gate: %.2f – %.2f m/s²\n", ACCEL_GATE_MIN, ACCEL_GATE_MAX);

    // ---- GYRO BIAS CALIBRATION ----
    // Samples taken in raw sensor frame (rad/s from Adafruit library).
    // The bias is subtracted before axis remapping, which is correct —
    // bias is a property of the physical sensor axis, not the flight frame.
    Serial.println("Calibrating gyro bias — keep drone still...");
    double sumX = 0.0, sumY = 0.0, sumZ = 0.0;
    int collected = 0;

    sensors_event_t a_ev, g_ev, t_ev, m_ev;

    while (collected < GYRO_CALIB_SAMPLES)
    {
        icm.getEvent(&a_ev, &g_ev, &t_ev, &m_ev);
        sumX += g_ev.gyro.x;
        sumY += g_ev.gyro.y;
        sumZ += g_ev.gyro.z;
        collected++;
        delay(1);
    }

    gyroBiasX = (float)(sumX / GYRO_CALIB_SAMPLES);
    gyroBiasY = (float)(sumY / GYRO_CALIB_SAMPLES);
    gyroBiasZ = (float)(sumZ / GYRO_CALIB_SAMPLES);

    Serial.printf("  Gyro bias (rad/s, raw frame): X=%.4f  Y=%.4f  Z=%.4f\n",
                  gyroBiasX, gyroBiasY, gyroBiasZ);

    // Sanity check (5 deg/s = 0.087 rad/s)
    if (fabsf(gyroBiasX) > 0.087f || fabsf(gyroBiasY) > 0.087f || fabsf(gyroBiasZ) > 0.087f)
        Serial.println("  WARNING: bias > 5 deg/s — was the drone perfectly still?");

    return true;
}

// ============================================================================
// UPDATE  (call at exactly IMU_UPDATE_HZ from the fixed-rate loop in main.cpp)
// ============================================================================
void updateIMU()
{
    sensors_event_t a_ev, g_ev, t_ev, m_ev;
    icm.getEvent(&a_ev, &g_ev, &t_ev, &m_ev);

    // ========================================================================
    // SENSOR → FILTER AXIS MAPPING
    // ========================================================================
    // VERIFIED from AXIS_CHECK.ino with drone flat on table (SparkFun library,
    // axes are identical between libraries — both read the same registers):
    //
    //   Position       sensorX    sensorY    sensorZ
    //   FLAT            +0.01      +0.01     +9.81    → Z = UP
    //   NOSE UP         -0.16      -9.65     +0.36    → Y = BACKWARD
    //   LEFT DOWN       -9.82      -0.34     +0.62    → X = LEFT
    //   RIGHT DOWN      +9.76      +0.66     +0.68    → X = LEFT confirmed
    //
    // Physical sensor frame (Adafruit ICM-20948 board, I2C pins facing back):
    //   Sensor +X  = drone's LEFT  side
    //   Sensor +Y  = drone's BACK  (tail direction)
    //   Sensor +Z  = drone's UP    (away from ground)
    //
    // NOTE: The silkscreen Y arrow points toward the top-row connector (facing
    // forward on the drone), but the actual sensor die Y axis is inverted on
    // this board. The AXIS_CHECK data is ground truth — trust it, not the silk.
    //
    // NWU filter frame (X=forward, Y=left, Z=up):
    //   filter_X (forward) = −sensorY    ← sensorY points backward → negate
    //   filter_Y (left)    = +sensorX    ← sensorX already points left
    //   filter_Z (up)      = +sensorZ    ← sensorZ already points up
    //
    // If axes are still wrong after flashing, re-run AXIS_CHECK.ino to
    // re-verify, then adjust the three lines marked [REMAP] below.
    // ========================================================================

    // Raw sensor values (Adafruit: accel in m/s², gyro in rad/s, mag in µT)
    float rawAx = a_ev.acceleration.x;
    float rawAy = a_ev.acceleration.y;
    float rawAz = a_ev.acceleration.z;

    float rawGyrX = g_ev.gyro.x; // rad/s
    float rawGyrY = g_ev.gyro.y;
    float rawGyrZ = g_ev.gyro.z;

    // Bias correction in raw sensor frame (bias is in rad/s from calibration)
    float gyrX = rawGyrX - gyroBiasX;
    float gyrY = rawGyrY - gyroBiasY;
    float gyrZ = rawGyrZ - gyroBiasZ;

    // Accel calibration (subtract offset then divide by scale)
    rawAx = (rawAx - ACCEL_OFFSET_X) / ACCEL_SCALE_X;
    rawAy = (rawAy - ACCEL_OFFSET_Y) / ACCEL_SCALE_Y;
    rawAz = (rawAz - ACCEL_OFFSET_Z) / ACCEL_SCALE_Z;

    // [REMAP] Sensor → NWU filter frame
    float ax = -rawAy; // forward = −backward
    float ay = +rawAx; // left    = +sensorX
    float az = +rawAz; // up      = +sensorZ

    float gx = -gyrY; // rotation around forward axis (rad/s)
    float gy = +gyrX; // rotation around left axis
    float gz = +gyrZ; // rotation around up axis

    // Magnetometer (hard-iron corrected, raw sensor frame)
    // The AK09916 axes were verified to produce clean sine waves on MX and MY
    // during a flat 360° yaw rotation. No additional remapping applied.
    float mx = m_ev.magnetic.x - MAG_OFFSET_X;
    float my = m_ev.magnetic.y - MAG_OFFSET_Y;
    float mz = m_ev.magnetic.z - MAG_OFFSET_Z;

    // ---- RUN MAHONY FILTER ----
    // Accel gating and dynamic Kp are handled inside mahonyUpdate().
    mahonyUpdate(gx, gy, gz, ax, ay, az, mx, my, mz);

    // ========================================================================
    // EULER ANGLE EXTRACTION  (Tait-Bryan ZYX from unit quaternion)
    // ========================================================================
    // Standard formulas for NWU frame, ZYX rotation order.
    //
    // In NWU (X=forward, Y=left, Z=up), Tait-Bryan ZYX gives:
    //
    //   ROLL  = atan2(2(q0q1+q2q3), 1−2(q1²+q2²))
    //     Positive roll = right-hand rotation around X (forward) = left wing UP
    //     = right wing DOWN = ROLLING RIGHT ✓ (aviation convention)
    //     No sign flip needed.
    //
    //   PITCH = asin(clamp(2(q0q2−q3q1), −1, +1))
    //     Positive pitch = right-hand rotation around Y (left axis) = nose UP ✓
    //     No sign flip needed.
    //
    //   YAW   = atan2(2(q0q3+q1q2), 1−2(q2²+q3²))
    //     Positive yaw = right-hand rotation around Z (up) = turning LEFT (CCW).
    //     NEGATE for aviation convention (positive = turning RIGHT/CW).
    //     Remapped to 0–360.
    //
    // SIGN VERIFICATION — check these with the visualizer before flying:
    //   Tilt drone RIGHT wing down → getRoll()  shows POSITIVE value
    //   Tilt drone NOSE up         → getPitch() shows POSITIVE value
    //   Rotate drone CW from above → getYaw()   increases
    // ========================================================================

    currentRoll = -atan2f(2.0f * (q0 * q1 + q2 * q3),
                          1.0f - 2.0f * (q1 * q1 + q2 * q2)) *
                  (180.0f / M_PI);

    // Clamp input to asin to avoid NaN from floating-point rounding
    float sinPitch = 2.0f * (q0 * q2 - q3 * q1);
    sinPitch = constrain(sinPitch, -1.0f, 1.0f);
    currentPitch = asinf(sinPitch) * (180.0f / M_PI);

    float yawRaw = -atan2f(2.0f * (q0 * q3 + q1 * q2),
                           1.0f - 2.0f * (q2 * q2 + q3 * q3)) *
                   (180.0f / M_PI);
    if (yawRaw < 0.0f)
        yawRaw += 360.0f;
    currentYaw = yawRaw;

    // ========================================================================
    // GYRO RATE EXPORTS FOR PID D-TERM  (degrees/second)
    // ========================================================================
    // Must have the same sign convention as the angle outputs above.
    //
    //   gyroRateX (roll rate,  positive = rolling right):
    //     Roll right = clockwise around forward = negative NWU gx rotation.
    //     (Positive NWU gx = left wing DOWN = rolling LEFT.)
    //     gyroRateX = −gx = +gyrY (in rad/s, converted to deg/s)
    //     In terms of raw sensor: gyrY = rawGyrY − biasY (sensorY = backward)
    //
    //   gyroRateY (pitch rate, positive = nose up):
    //     Nose up = positive NWU gy rotation.
    //     gyroRateY = +gy = +gyrX
    //
    //   gyroRateZ (yaw rate,   positive = turning right/CW):
    //     CW from above = negative NWU gz rotation.
    //     gyroRateZ = −gz = −gyrZ
    //
    // SIGN VERIFICATION — check with visualizer before flying:
    //   Roll drone right  → getGyroRateX() positive while moving
    //   Pitch nose up     → getGyroRateY() positive while moving
    //   Yaw drone CW      → getGyroRateZ() positive while moving
    // ========================================================================
    const float RAD2DEG = 180.0f / M_PI;

    gyroRateX = -gyrY * RAD2DEG; // roll  rate: positive = rolling right
    gyroRateY = +gyrX * RAD2DEG; // pitch rate: positive = nose up
    gyroRateZ = -gyrZ * RAD2DEG; // yaw   rate: positive = turning right
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