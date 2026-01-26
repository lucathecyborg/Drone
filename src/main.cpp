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
// CCW motors spin counter-clockwise (produce positive yaw torque)
// CW motors spin clockwise (produce negative yaw torque)
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

// 16-bit PWM duty cycle values for 50Hz (20ms period)
// duty = (pulse_us / 20000) * 65535
#define ESC_MIN_DUTY 3277 // 1000us / 20000 * 65535
#define ESC_MAX_DUTY 6554 // 2000us / 20000 * 65535

// Motor speed limits (0-1000 scale for easier math)
#define MOTOR_MIN 0
#define MOTOR_MAX 1000
#define MOTOR_IDLE 50 // Minimum spin when armed with throttle

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
PIDController rollPID = {
    .kp = 1.5f,
    .ki = 0.08f,
    .kd = 0.9f,
    .integral = 0,
    .prevError = 0,
    .prevMeasurement = 0,
    .prevTime = 0,
    .initialized = false,
    .integralLimit = 200.0f,
    .outputLimit = 400.0f};

PIDController pitchPID = {
    .kp = 1.5f,
    .ki = 0.08f,
    .kd = 0.9f,
    .integral = 0,
    .prevError = 0,
    .prevMeasurement = 0,
    .prevTime = 0,
    .initialized = false,
    .integralLimit = 200.0f,
    .outputLimit = 400.0f};

PIDController yawPID = {
    .kp = 2.0f,
    .ki = 0.05f,
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

// ============================================================================
// PID COMPUTATION
// ============================================================================

/**
 * Compute PID output using proper discrete-time implementation.
 *
 * Key features:
 * 1. Derivative-on-measurement: Prevents derivative kick when setpoint changes
 * 2. Anti-windup: Clamps integral term to prevent saturation
 * 3. Proper time handling: Uses actual elapsed time for accurate integration
 * 4. Output limiting: Prevents excessive corrections
 *
 * @param pid      Pointer to PID controller structure
 * @param setpoint Desired value (angle for roll/pitch, rate for yaw)
 * @param measured Current measured value from sensors
 * @return         PID correction output
 */
float computePID(PIDController *pid, float setpoint, float measured)
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
  // P = Kp * error
  float P = pid->kp * error;

  // ---- INTEGRAL TERM ----
  // Accumulate error over time: I += Ki * error * dt
  // With anti-windup clamping
  pid->integral += error * dt;

  // Anti-windup: Clamp integral to prevent excessive accumulation
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
  // Using derivative-on-measurement to prevent derivative kick
  // D = -Kd * d(measurement)/dt
  // Negative because we want to oppose changes in measurement
  float dMeasurement = (measured - pid->prevMeasurement) / dt;
  float D = -pid->kd * dMeasurement;

  // Alternative: derivative-on-error (causes kick on setpoint change)
  // float dError = (error - pid->prevError) / dt;
  // float D = pid->kd * dError;

  // Store state for next iteration
  pid->prevTime = now;
  pid->prevError = error;
  pid->prevMeasurement = measured;

  // Calculate total output
  float output = P + I + D;

  // Clamp output to limits
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
 * Motor layout and rotation directions:
 *       FRONT
 *    TL(CCW)  TR(CW)
 *         \/
 *         /\
 *    BL(CW)   BR(CCW)
 *       BACK
 *
 * Mixing equations:
 * - Throttle: All motors increase equally
 * - Roll (right = positive): Left motors increase, right motors decrease
 * - Pitch (forward = positive): Back motors increase, front motors decrease
 * - Yaw (CW = positive): CCW motors increase, CW motors decrease
 *
 * @param throttle  Base throttle (0-1000)
 * @param roll      Roll correction from PID
 * @param pitch     Pitch correction from PID
 * @param yaw       Yaw correction from PID
 */
void mixMotors(int throttle, float roll, float pitch, float yaw)
{
  // Motor mixing equations for X configuration
  // Signs determined by motor positions and rotation directions

  // Top Left (front-left, CCW rotation)
  // - Positive pitch (nose up): decrease front motors
  // - Positive roll (right): increase left motors
  // - Positive yaw (CW): increase CCW motors
  motorTL = throttle - pitch + roll + yaw;

  // Top Right (front-right, CW rotation)
  // - Positive pitch (nose up): decrease front motors
  // - Positive roll (right): decrease right motors
  // - Positive yaw (CW): decrease CW motors
  motorTR = throttle - pitch - roll - yaw;

  // Bottom Left (back-left, CW rotation)
  // - Positive pitch (nose up): increase back motors
  // - Positive roll (right): increase left motors
  // - Positive yaw (CW): decrease CW motors
  motorBL = throttle + pitch + roll - yaw;

  // Bottom Right (back-right, CCW rotation)
  // - Positive pitch (nose up): increase back motors
  // - Positive roll (right): decrease right motors
  // - Positive yaw (CW): increase CCW motors
  motorBR = throttle + pitch - roll + yaw;

  // Constrain all motors to valid range
  motorTL = constrain(motorTL, MOTOR_MIN, MOTOR_MAX);
  motorTR = constrain(motorTR, MOTOR_MIN, MOTOR_MAX);
  motorBL = constrain(motorBL, MOTOR_MIN, MOTOR_MAX);
  motorBR = constrain(motorBR, MOTOR_MIN, MOTOR_MAX);
}

// ============================================================================
// INPUT PROCESSING
// ============================================================================

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
 */
void processInputs()
{
  // Map throttle from controller (0-1023) to motor scale (0-1000)
  // Apply deadband at low end to eliminate noise
  uint16_t rawThrottle = rxData.throttle;
  if (rawThrottle < 20)
  {
    rawThrottle = 0;
  }
  baseThrottle = map(rawThrottle, 0, 1023, 0, MOTOR_MAX);

// Map left joystick to roll/pitch angles
// Joystick center is ~512, range 0-1023
// Map to ±30 degrees (adjustable)
#define MAX_ANGLE 30.0f
#define JOYSTICK_CENTER 512
#define JOYSTICK_DEADBAND 30

  // Roll: Left joystick X axis
  // Positive X = stick right = roll right
  int rollInput = rxData.leftX - JOYSTICK_CENTER;
  if (abs(rollInput) < JOYSTICK_DEADBAND)
  {
    targetRoll = 0;
  }
  else
  {
    targetRoll = map(rxData.leftX, 0, 1023, -MAX_ANGLE, MAX_ANGLE);
  }

  // Pitch: Left joystick Y axis
  // Positive Y = stick forward = pitch forward
  int pitchInput = rxData.leftY - JOYSTICK_CENTER;
  if (abs(pitchInput) < JOYSTICK_DEADBAND)
  {
    targetPitch = 0;
  }
  else
  {
    targetPitch = map(rxData.leftY, 0, 1023, -MAX_ANGLE, MAX_ANGLE);
  }

// Yaw rate: Right joystick X axis
// Map to ±180 degrees per second
#define MAX_YAW_RATE 180.0f
#define YAW_DEADBAND 50

  int yawInput = rxData.rightX - JOYSTICK_CENTER;
  if (abs(yawInput) < YAW_DEADBAND)
  {
    targetYawRate = 0;
  }
  else
  {
    targetYawRate = map(rxData.rightX, 0, 1023, -MAX_YAW_RATE, MAX_YAW_RATE);
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
  // Get current attitude from IMU (using your IMU.h functions)
  float currentRollAngle = getRoll();    // Mahony filter output (degrees)
  float currentPitchAngle = getPitch();  // Mahony filter output (degrees)
  float currentYawRate = getGyroRateZ(); // Raw gyro Z for yaw rate mode (deg/s)

  // Check if we have enough throttle to fly
  if (baseThrottle < MOTOR_IDLE)
  {
    // Throttle too low - disarm/idle state
    stopMotors();
    resetAllPIDs();
    return;
  }

  // Compute PID corrections
  float rollCorrection = computePID(&rollPID, targetRoll, currentRollAngle);
  float pitchCorrection = computePID(&pitchPID, targetPitch, currentPitchAngle);
  float yawCorrection = computePID(&yawPID, targetYawRate, currentYawRate);

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
  Serial.printf("Thr:%4d | R:%6.1f P:%6.1f YR:%6.1f | tR:%5.1f tP:%5.1f tYR:%5.1f | M: %4d %4d %4d %4d\n",
                baseThrottle,
                getRoll(), getPitch(), getGyroRateZ(),
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

  // Arm ESCs
  armMotors();

  // Initialize control timing
  lastControlTime = micros();
  lastRxTime = millis();

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

      // Process inputs from controller
      processInputs();

      // Update PID gains if controller sent new values
      updatePIDGains();

      // Update communication stats
      updateCommStats(success);
    }
  }

  // ---- FAILSAFE CHECK ----
  if (armed && (now - lastRxTime > FAILSAFE_TIMEOUT_MS))
  {
    failsafe();
  }

  // ---- FIXED-RATE CONTROL LOOP ----
  // Run at 250Hz for smooth control
  unsigned long elapsed = nowMicros - lastControlTime;
  if (elapsed >= CONTROL_LOOP_PERIOD_US)
  {
    lastControlTime = nowMicros;

    // Update IMU data
    updateIMU(); // Assumes IMU.h provides this

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
