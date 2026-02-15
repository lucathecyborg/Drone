#include <Arduino.h>

#include "Communication.h"
#include "IMU.h"
#include "BMP.h"
#include "ADS1115.h"

// ============================================================================
// MOTOR CONFIGURATION
// ============================================================================
// Motor layout (X configuration, viewed from above):
//
//       FRONT
//    TL   ^   TR        TL = Top Left (CCW)
//      \  |  /          TR = Top Right (CW)
//       \ | /           BL = Bottom Left (CW)
//        \|/            BR = Bottom Right (CCW)
//    -----+-----
//        /|\
//       / | \
//      /  |  \
//    BL   v   BR
//       BACK
//
// CCW motors spin counter-clockwise (produce CW reaction torque on frame)
// CW motors spin clockwise (produce CCW reaction torque on frame)
//
// To yaw CW (positive): speed up CW motors (TR, BL), slow CCW motors (TL, BR)
// To yaw CCW (negative): speed up CCW motors (TL, BR), slow CW motors (TR, BL)
// ============================================================================

#define TOPL_PIN 14
#define TOPR_PIN 27
#define BOTTOML_PIN 26
#define BOTTOMR_PIN 25

#define TOPL_CHANNEL 0
#define TOPR_CHANNEL 1
#define BOTTOML_CHANNEL 2
#define BOTTOMR_CHANNEL 3

#define PWM_FREQ 50       // 50Hz for ESC
#define PWM_RESOLUTION 16 // 16-bit resolution

// ESC pulse width range (microseconds)
#define ESC_MIN_US 1000
#define ESC_MAX_US 2000

// flags
#define FLAG_ARMED (1 << 0)          // bit 0: motors armed
#define FLAG_ALT_HOLD (1 << 1)       // bit 1: altitude hold enabled
#define FLAG_RETURN_TO_HOME (1 << 2) // enable return to home
#define FLAG_SAFE_LANDING (1 << 3)   // enable safe landing
#define FLAG_SET_HOME (1 << 4)       // set GPS home location
#define FLAG_FREEZE (1 << 5)         // freeze input from controller

bool holdingAltitude = false;
int heldPower = 0;

// 16-bit PWM duty cycle values for 50Hz (20ms period)
// duty = (pulse_us / 20000) * 65535
#define ESC_MIN_DUTY 3277 // 1000us / 20000 * 65535
#define ESC_MAX_DUTY 6554 // 2000us / 20000 * 65535

// Motor speed limits (0-1000 scale for easier math)
#define MOTOR_MIN 0
#define MOTOR_MAX 1000
#define MOTOR_IDLE 50 // Minimum spin when armed with throttle

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

/**
 * Float version of map() for smooth analog control.
 * Arduino's map() uses integer math which quantizes small ranges badly.
 */
float fmap(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// ============================================================================
// PID CONTROLLER
// ============================================================================

struct PIDController
{
  // Gains
  float kp;
  float ki;
  float kd;

  // State
  float integral;
  float prevError;
  float prevMeasurement; // For derivative-on-measurement
  unsigned long prevTime;
  bool initialized;

  // Limits
  float integralLimit; // Anti-windup limit
  float outputLimit;   // Maximum output magnitude
};

// PID controllers for each axis
// Roll/Pitch: angle mode (setpoint is target angle in degrees)
// Yaw: rate mode (setpoint is target rotation rate in deg/s)
//
// Tuning notes for 1.5kg quad:
//   - Start with ki=0 for first flights, tune P and D first
//   - D term uses gyro rate directly (not differentiated angle) for less noise
//   - Increase kp if drone feels sluggish, decrease if it oscillates
//   - Add ki slowly only after P and D are stable to fix steady-state drift
PIDController rollPID = {
    .kp = 3.0f,
    .ki = 0.0f, // Start at 0, add slowly after P/D are tuned
    .kd = 0.5f,
    .integral = 0,
    .prevError = 0,
    .prevMeasurement = 0,
    .prevTime = 0,
    .initialized = false,
    .integralLimit = 200.0f,
    .outputLimit = 400.0f};

PIDController pitchPID = {
    .kp = 3.0f,
    .ki = 0.0f, // Start at 0, add slowly after P/D are tuned
    .kd = 0.5f,
    .integral = 0,
    .prevError = 0,
    .prevMeasurement = 0,
    .prevTime = 0,
    .initialized = false,
    .integralLimit = 200.0f,
    .outputLimit = 400.0f};

PIDController yawPID = {
    .kp = 3.0f,
    .ki = 0.0f,
    .kd = 0.0f, // Usually 0 for yaw rate mode
    .integral = 0,
    .prevError = 0,
    .prevMeasurement = 0,
    .prevTime = 0,
    .initialized = false,
    .integralLimit = 150.0f,
    .outputLimit = 300.0f};

// ============================================================================
// CONTROL VARIABLES
// ============================================================================

// Target setpoints from joystick
float targetRoll = 0;    // Target roll angle (degrees)
float targetPitch = 0;   // Target pitch angle (degrees)
float targetYawRate = 0; // Target yaw rate (degrees/second)

// Base throttle from controller (0-1000 scale)
int baseThrottle = 0;

// Individual motor outputs (0-1000 scale)
int motorTL = 0;
int motorTR = 0;
int motorBL = 0;
int motorBR = 0;

// Control loop timing
#define CONTROL_LOOP_HZ 250
#define CONTROL_LOOP_PERIOD_US (1000000 / CONTROL_LOOP_HZ)
unsigned long lastControlTime = 0;

// Debug timing
unsigned long lastDebugTime = 0;

// Failsafe
bool armed = false;
unsigned long lastRxTime = 0;
#define FAILSAFE_TIMEOUT_MS 500

// Heading hold
float targetHeading = 0;
bool headingHoldActive = false;

// ============================================================================
// MAGNETOMETER HEADING (inverted due to board layout)
// ============================================================================

/**
 * Get the corrected heading from the IMU magnetometer.
 * The magnetometer is mounted inverted on the board, so raw 0° = 180° real
 * and raw 180° = 0° real. This corrects that by adding 180° and wrapping.
 *
 * @return Corrected heading in degrees (0-360, 0=North, 90=East)
 */
float getCorrectedHeading()
{
  float rawYaw = getYaw(); // From Mahony filter (0-360)

  // Invert: add 180 and wrap to 0-360
  float corrected = rawYaw + 180.0f;
  if (corrected >= 360.0f)
  {
    corrected -= 360.0f;
  }

  return corrected;
}

/**
 * Calculate the shortest angular difference between two headings.
 * Returns value in range -180 to +180.
 * Positive = target is clockwise from current.
 */
float headingError(float target, float current)
{
  float error = target - current;

  if (error > 180.0f)
    error -= 360.0f;
  if (error < -180.0f)
    error += 360.0f;

  return error;
}

// ============================================================================
// PID COMPUTATION
// ============================================================================

/**
 * Compute PID output using proper discrete-time implementation.
 *
 * Key features:
 * 1. Derivative uses gyro rate directly (passed as gyroRate parameter)
 *    instead of differentiating the angle measurement. This is far less
 *    noisy and is standard practice in flight controllers (Betaflight, etc.)
 * 2. Anti-windup: Clamps integral term to prevent saturation
 * 3. Proper time handling: Uses actual elapsed time for accurate integration
 * 4. Output limiting: Prevents excessive corrections
 *
 * @param pid      Pointer to PID controller structure
 * @param setpoint Desired value (angle for roll/pitch, rate for yaw)
 * @param measured Current measured value from sensors
 * @param gyroRate Gyro rate in deg/s for this axis (used for D term)
 * @return         PID correction output
 */
float computePID(PIDController *pid, float setpoint, float measured, float gyroRate)
{
  unsigned long now = micros();

  // First call initialization
  if (!pid->initialized)
  {
    pid->prevTime = now;
    pid->prevError = 0;
    pid->prevMeasurement = measured;
    pid->integral = 0;
    pid->initialized = true;
    return 0;
  }

  // Calculate time delta in seconds
  unsigned long dtMicros = now - pid->prevTime;
  float dt = dtMicros / 1000000.0f;

  // Handle timer overflow or invalid dt
  // micros() overflows every ~70 minutes
  if (dt <= 0 || dt > 1.0f || dtMicros > 1000000)
  {
    pid->prevTime = now;
    pid->prevMeasurement = measured;
    return 0;
  }

  // Skip if dt is too small (prevents division issues)
  if (dt < 0.0001f)
  {
    return 0;
  }

  // Calculate error
  float error = setpoint - measured;

  // ---- PROPORTIONAL TERM ----
  float P = pid->kp * error;

  // ---- INTEGRAL TERM ----
  pid->integral += error * dt;

  // Anti-windup: Clamp integral
  if (pid->integral > pid->integralLimit)
  {
    pid->integral = pid->integralLimit;
  }
  else if (pid->integral < -pid->integralLimit)
  {
    pid->integral = -pid->integralLimit;
  }

  float I = pid->ki * pid->integral;

  // ---- DERIVATIVE TERM ----
  // Use gyro rate directly instead of differentiating the angle.
  // Gyro gives a clean rate signal; differentiating the Mahony angle
  // amplifies noise. The negative sign is because if the measurement
  // is increasing (positive gyro rate), we want to resist that change.
  float D = -pid->kd * gyroRate;

  // Store state for next iteration
  pid->prevTime = now;
  pid->prevError = error;
  pid->prevMeasurement = measured;

  // Calculate total output
  float output = P + I + D;

  // Clamp output
  if (output > pid->outputLimit)
  {
    output = pid->outputLimit;
  }
  else if (output < -pid->outputLimit)
  {
    output = -pid->outputLimit;
  }

  return output;
}

/**
 * Reset PID controller state.
 * Call this when:
 * - Disarming motors
 * - Transitioning from idle to flight
 * - After a failsafe event
 */
void resetPID(PIDController *pid)
{
  pid->integral = 0;
  pid->prevError = 0;
  pid->prevMeasurement = 0;
  pid->prevTime = micros();
  pid->initialized = false;
}

/**
 * Reset all PID controllers.
 */
void resetAllPIDs()
{
  resetPID(&rollPID);
  resetPID(&pitchPID);
  resetPID(&yawPID);
}

// ============================================================================
// MOTOR CONTROL
// ============================================================================

int initMotors()
{
  Serial.println("Initializing motors...");

  if (ledcSetup(TOPL_CHANNEL, PWM_FREQ, PWM_RESOLUTION) == 0)
  {
    Serial.println("Failed to setup TOPL channel");
    return 0;
  }
  ledcAttachPin(TOPL_PIN, TOPL_CHANNEL);

  if (ledcSetup(TOPR_CHANNEL, PWM_FREQ, PWM_RESOLUTION) == 0)
  {
    Serial.println("Failed to setup TOPR channel");
    return 0;
  }
  ledcAttachPin(TOPR_PIN, TOPR_CHANNEL);

  if (ledcSetup(BOTTOML_CHANNEL, PWM_FREQ, PWM_RESOLUTION) == 0)
  {
    Serial.println("Failed to setup BOTTOML channel");
    return 0;
  }
  ledcAttachPin(BOTTOML_PIN, BOTTOML_CHANNEL);

  if (ledcSetup(BOTTOMR_CHANNEL, PWM_FREQ, PWM_RESOLUTION) == 0)
  {
    Serial.println("Failed to setup BOTTOMR channel");
    return 0;
  }
  ledcAttachPin(BOTTOMR_PIN, BOTTOMR_CHANNEL);

  Serial.println("All motor channels initialized");
  return 1;
}

void armMotors()
{
  // Send minimum throttle signal to arm ESCs
  ledcWrite(TOPL_CHANNEL, ESC_MIN_DUTY);
  ledcWrite(TOPR_CHANNEL, ESC_MIN_DUTY);
  ledcWrite(BOTTOML_CHANNEL, ESC_MIN_DUTY);
  ledcWrite(BOTTOMR_CHANNEL, ESC_MIN_DUTY);

  Serial.println("Sending arming signal to ESCs...");
  delay(3000);
  Serial.println("ESCs armed.");
  armed = true;
}

/**
 * Convert motor speed (0-1000) to PWM duty cycle.
 */
int speedToDuty(int speed)
{
  speed = constrain(speed, MOTOR_MIN, MOTOR_MAX);
  return map(speed, MOTOR_MIN, MOTOR_MAX, ESC_MIN_DUTY, ESC_MAX_DUTY);
}

/**
 * Write speeds to all motors.
 */
void writeMotors()
{
  ledcWrite(TOPL_CHANNEL, speedToDuty(motorTL));
  ledcWrite(TOPR_CHANNEL, speedToDuty(motorTR));
  ledcWrite(BOTTOML_CHANNEL, speedToDuty(motorBL));
  ledcWrite(BOTTOMR_CHANNEL, speedToDuty(motorBR));
}

/**
 * Stop all motors immediately.
 */
void stopMotors()
{
  motorTL = 0;
  motorTR = 0;
  motorBL = 0;
  motorBR = 0;
  writeMotors();
}

// ============================================================================
// MOTOR MIXING
// ============================================================================

/**
 * Apply motor mixing for X-configuration quadcopter.
 *
 * Physics of yaw mixing:
 *   A CCW-spinning prop produces a CW reaction torque on the frame.
 *   A CW-spinning prop produces a CCW reaction torque on the frame.
 *   To yaw CW (positive yaw command): speed up CW motors, slow CCW motors.
 *
 * Mixing equations:
 * - Throttle: All motors increase equally
 * - Roll (right = positive): Left motors increase, right motors decrease
 * - Pitch (forward = positive): Back motors increase, front motors decrease
 * - Yaw (CW = positive): CW motors (TR,BL) increase, CCW motors (TL,BR) decrease
 */
void mixMotors(int throttle, float roll, float pitch, float yaw)
{
  // Top Left (front-left, CCW): -pitch +roll -yaw
  motorTL = throttle - pitch + roll - yaw;

  // Top Right (front-right, CW): -pitch -roll +yaw
  motorTR = throttle - pitch - roll + yaw;

  // Bottom Left (back-left, CW): +pitch +roll +yaw
  motorBL = throttle + pitch + roll + yaw;

  // Bottom Right (back-right, CCW): +pitch -roll -yaw
  motorBR = throttle + pitch - roll - yaw;

  // Constrain all motors to valid range
  motorTL = constrain(motorTL, MOTOR_MIN, MOTOR_MAX);
  motorTR = constrain(motorTR, MOTOR_MIN, MOTOR_MAX);
  motorBL = constrain(motorBL, MOTOR_MIN, MOTOR_MAX);
  motorBR = constrain(motorBR, MOTOR_MIN, MOTOR_MAX);
}

// ============================================================================
// INPUT PROCESSING
// ============================================================================

#define MAX_ANGLE 30.0f
#define JOYSTICK_CENTER 512
#define JOYSTICK_DEADBAND 30
#define MAX_YAW_RATE 180.0f
#define YAW_DEADBAND 50

/**
 * Process joystick inputs and convert to control setpoints.
 *
 * Left joystick:
 * - X axis: Roll angle setpoint (±30 degrees)
 * - Y axis: Pitch angle setpoint (±30 degrees)
 *
 * Right joystick:
 * - X axis: Yaw rate setpoint (±180 deg/s)
 *
 * Throttle:
 * - Potentiometer: Base motor speed
 *
 * When the yaw stick is centered, heading hold engages automatically
 * using the magnetometer to prevent gyro drift.
 */
void processInputs()
{
  // ---- THROTTLE ----
  uint16_t rawThrottle = rxData.throttle;
  if (rawThrottle < 20)
  {
    rawThrottle = 0;
  }

  if (holdingAltitude)
  {
    rawThrottle = heldPower;
  }

  baseThrottle = map(rawThrottle, 0, 1023, 0, MOTOR_MAX);

  // ---- ROLL ----
  // Uses fmap() for smooth float output instead of integer map()
  int rollInput = rxData.leftX - JOYSTICK_CENTER;
  if (abs(rollInput) < JOYSTICK_DEADBAND)
  {
    targetRoll = 0;
  }
  else
  {
    targetRoll = fmap((float)rxData.leftX, 0.0f, 1023.0f, -MAX_ANGLE, MAX_ANGLE);
  }

  // ---- PITCH ----
  int pitchInput = rxData.leftY - JOYSTICK_CENTER;
  if (abs(pitchInput) < JOYSTICK_DEADBAND)
  {
    targetPitch = 0;
  }
  else
  {
    targetPitch = fmap((float)rxData.leftY, 0.0f, 1023.0f, -MAX_ANGLE, MAX_ANGLE);
  }

  // ---- YAW (with heading hold) ----
  int yawInput = rxData.rightX - JOYSTICK_CENTER;
  if (abs(yawInput) < YAW_DEADBAND)
  {
    // Stick centered: engage heading hold
    if (!headingHoldActive)
    {
      // Lock the current heading as the target
      targetHeading = getCorrectedHeading();
      headingHoldActive = true;
    }

    // Compute yaw rate from heading error to maintain heading
    float hError = headingError(targetHeading, getCorrectedHeading());
    targetYawRate = hError * 2.0f; // P-gain for heading hold
    targetYawRate = constrain(targetYawRate, -MAX_YAW_RATE, MAX_YAW_RATE);
  }
  else
  {
    // Stick deflected: manual yaw rate mode
    headingHoldActive = false;
    targetYawRate = fmap((float)rxData.rightX, 0.0f, 1023.0f, -MAX_YAW_RATE, MAX_YAW_RATE);
  }
}

/**
 * Update PID gains from controller if received.
 * Only updates when pidAxis is valid (0-2).
 */
void updatePIDGains()
{
  if (rxData.pidAxis > 2)
  {
    return; // No PID update requested
  }

  switch (rxData.pidAxis)
  {
  case 0: // Pitch
    pitchPID.kp = rxData.kp;
    pitchPID.ki = rxData.ki;
    pitchPID.kd = rxData.kd;
    Serial.printf("Pitch PID updated: P=%.3f I=%.3f D=%.3f\n",
                  rxData.kp, rxData.ki, rxData.kd);
    break;

  case 1: // Roll
    rollPID.kp = rxData.kp;
    rollPID.ki = rxData.ki;
    rollPID.kd = rxData.kd;
    Serial.printf("Roll PID updated: P=%.3f I=%.3f D=%.3f\n",
                  rxData.kp, rxData.ki, rxData.kd);
    break;

  case 2: // Yaw
    yawPID.kp = rxData.kp;
    yawPID.ki = rxData.ki;
    yawPID.kd = rxData.kd;
    Serial.printf("Yaw PID updated: P=%.3f I=%.3f D=%.3f\n",
                  rxData.kp, rxData.ki, rxData.kd);
    break;
  }
}

// ============================================================================
// MAIN CONTROL LOOP
// ============================================================================

/**
 * Execute one iteration of the flight control loop.
 * This should run at a fixed rate (250Hz recommended).
 */
void controlLoop()
{
  // Get current attitude from IMU (using IMU.h functions)
  float currentRollAngle = getRoll();    // Mahony filter output (degrees)
  float currentPitchAngle = getPitch();  // Mahony filter output (degrees)
  float currentYawRate = getGyroRateZ(); // Raw gyro Z for yaw rate mode (deg/s)

  // Check if we have enough throttle to fly
  if (baseThrottle < MOTOR_IDLE)
  {
    // Throttle too low - disarm/idle state
    stopMotors();
    resetAllPIDs();
    headingHoldActive = false;
    return;
  }

  // Compute PID corrections
  // Roll/Pitch: pass gyro rate for the D term (cleaner than differentiating angle)
  // Yaw: gyro rate IS the measurement (rate mode PID)
  float rollCorrection = computePID(&rollPID, targetRoll, currentRollAngle, getGyroRateX());
  float pitchCorrection = computePID(&pitchPID, targetPitch, currentPitchAngle, getGyroRateY());
  float yawCorrection = computePID(&yawPID, targetYawRate, currentYawRate, currentYawRate);

  // Apply motor mixing
  mixMotors(baseThrottle, rollCorrection, pitchCorrection, yawCorrection);

  // Write to motors
  writeMotors();
}

/**
 * Handle failsafe condition (no signal from controller).
 */
void failsafe()
{
  Serial.println("FAILSAFE: No signal from controller!");
  stopMotors();
  resetAllPIDs();
  armed = false;
}

// ============================================================================
// DEBUG OUTPUT
// ============================================================================

void printDebug()
{
  Serial.printf("Thr:%4d | R:%6.1f P:%6.1f H:%5.1f | tR:%5.1f tP:%5.1f tYR:%5.1f | M: %4d %4d %4d %4d\n",
                baseThrottle,
                getRoll(), getPitch(), getCorrectedHeading(),
                targetRoll, targetPitch, targetYawRate,
                motorTL, motorTR, motorBL, motorBR);
}

// ============================================================================
// ARDUINO SETUP & LOOP
// ============================================================================

void infiniteLoop()
{
  while (1)
  {
    delay(100);
  }
}

void setup()
{
  Serial.begin(115200);
  delay(100);

  Wire.begin();

  Serial.println("Initializing SPI...");
  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, CSN_PIN);
  delay(100);

  pinMode(CSN_PIN, OUTPUT);
  digitalWrite(CSN_PIN, HIGH);
  delay(10);

  if (!initRadio())
  {
    Serial.println("Radio failed to init, aborting.");
    infiniteLoop();
  }

  if (!initICM())
  {
    Serial.println("IMU failed to init, aborting.");
    infiniteLoop();
  }

  if (!bmp.initBMP())
  {
    Serial.println("BMP failed to init, aborting.");
    infiniteLoop();
  }

  if (!ads.initADS())
  {
    Serial.println("ADS failed to init, aborting.");
    infiniteLoop();
  }

  if (!initMotors())
  {
    Serial.println("Motors failed to init, aborting.");
    infiniteLoop();
  }

  Serial.println("Keep drone LEVEL and STILL for filter convergence...");
  delay(2000);

  // Pre-run the IMU update a few times to let the filter settle
  for (int i = 0; i < 500; i++)
  {
    updateIMU();
    delay(4); // 250Hz
  }
  Serial.println("IMU filter converged.");

  // Wait for ARM command from controller
  // IMPORTANT: Keep updating IMU so the Mahony filter doesn't go stale
  Serial.println("Waiting for ARM command...");
  while (true)
  {
    updateIMU(); // Keep filter running while waiting

    if (radio.available())
    {
      bool success = recieveData();
      if (success && (rxData.flags & FLAG_ARMED))
      {
        armMotors();
        break;
      }
    }
    delay(4); // ~250Hz to match filter rate
  }

  // Initialize control timing
  lastControlTime = micros();
  lastRxTime = millis();
  targetHeading = getCorrectedHeading(); // Initialize heading hold

  Serial.println("Ready for flight!");
}

void loop()
{
  unsigned long now = millis();
  unsigned long nowMicros = micros();

  // ---- RECEIVE DATA FROM CONTROLLER ----
  if (radio.available())
  {
    bool success = recieveData();

    if (success)
    {
      lastRxTime = now;

      // If throttle drops below 20, disarm and require re-arming
      if (rxData.throttle < 20)
      {
        armed = false;
      }

      // Only allow operation if armed flag is received AND we're armed
      if (!armed && (rxData.flags & FLAG_ARMED))
      {
        armed = true;
        resetAllPIDs();
        headingHoldActive = false;
        targetHeading = getCorrectedHeading();
        Serial.println("Motors re-armed");
      }

      // Handle altitude hold flag
      if (rxData.flags & FLAG_ALT_HOLD)
      {
        if (!holdingAltitude)
        {
          heldPower = rxData.throttle;
          holdingAltitude = true;
          Serial.printf("Altitude hold engaged at throttle: %d\n", heldPower);
        }
      }
      else
      {
        holdingAltitude = false;
      }

      if (armed)
      {
        // Process inputs from controller
        processInputs();

        // Update PID gains if controller sent new values
        updatePIDGains();

        // Update communication stats
        updateCommStats(success);
      }
      else
      {
        Serial.println("Motors need to be rearmed");
      }
    }
  }

  // ---- FAILSAFE CHECK ----
  if (armed && (now - lastRxTime > FAILSAFE_TIMEOUT_MS))
  {
    failsafe();
  }

  // ---- FIXED-RATE CONTROL LOOP ----
  // Run at 250Hz for smooth control
  // Use increment-by-period to prevent timing drift
  if (nowMicros - lastControlTime >= CONTROL_LOOP_PERIOD_US)
  {
    lastControlTime += CONTROL_LOOP_PERIOD_US;

    // If we've fallen behind by more than one period, reset instead of
    // trying to catch up (prevents burst of rapid iterations)
    if (nowMicros - lastControlTime >= CONTROL_LOOP_PERIOD_US)
    {
      lastControlTime = nowMicros;
    }

    // Update IMU data
    updateIMU();

    // Run control loop
    if (armed)
    {
      controlLoop();
    }
  }

  // ---- DEBUG OUTPUT ----
  if (now - lastDebugTime >= 200)
  {
    lastDebugTime = now;
    printDebug();
  }
}