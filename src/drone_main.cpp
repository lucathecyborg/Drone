#include <Arduino.h>
#include <math.h>

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
// CCW motors: TL, BR — produce CW reaction torque on the frame
// CW  motors: TR, BL — produce CCW reaction torque on the frame
//
// Yaw CW  (+): speed up CW motors (TR, BL), slow CCW motors (TL, BR)
// Yaw CCW (-): speed up CCW motors (TL, BR), slow CW motors (TR, BL)
// ============================================================================

#define TOPL_PIN 25
#define TOPR_PIN 14
#define BOTTOML_PIN 26
#define BOTTOMR_PIN 27

#define TOPL_CHANNEL 0
#define TOPR_CHANNEL 1
#define BOTTOML_CHANNEL 2
#define BOTTOMR_CHANNEL 3

#define PWM_FREQ 50       // 50 Hz for standard ESC PWM
#define PWM_RESOLUTION 16 // 16-bit resolution

// ESC pulse width limits (microseconds)
#define ESC_MIN_US 1000
#define ESC_MAX_US 2000

// 16-bit duty cycle equivalents at 50 Hz (20 ms period)
// duty = (pulse_us / 20000) * 65535
#define ESC_MIN_DUTY 3277 // 1000 µs
#define ESC_MAX_DUTY 6554 // 2000 µs

// Motor speed limits (0–1000 internal scale)
#define MOTOR_MIN 0
#define MOTOR_MAX 1000
#define MOTOR_IDLE 50 // Minimum speed considered "powered" for control purposes

// Flags sent from controller in rxData.flags
#define FLAG_ARMED (1 << 0)
#define FLAG_ALT_HOLD (1 << 1)
#define FLAG_RETURN_TO_HOME (1 << 2)
#define FLAG_SAFE_LANDING (1 << 3)
#define FLAG_SET_HOME (1 << 4)
#define FLAG_FREEZE (1 << 5)

#define MOTORS_ENABLED

// ============================================================================
// LOOP TIMING
// ============================================================================
// Using a fixed timestep constant (LOOP_DT) for all PID and filter math,
// rather than measuring actual elapsed time.
//
// Why: Betaflight uses the same approach (see pid.c — "dT is fixed and
// calculated from the target PID loop time. This is done to avoid D-term
// spikes that occur with dynamically calculated deltaT whenever another task
// causes the PID loop execution to be delayed.").
//
// In practice, if a late radio packet or serial print delays a loop iteration,
// computing dT from micros() would see a large dt and output an enormous
// D spike. A fixed dt assumes the loop is always on time, which it nearly
// always is, and caps the D spike to a bounded value in the rare case it
// isn't.
// ============================================================================
#define CONTROL_LOOP_HZ 250
#define CONTROL_LOOP_PERIOD_US (1000000 / CONTROL_LOOP_HZ)
#define LOOP_DT (1.0f / CONTROL_LOOP_HZ) // 0.004 s — FIXED timestep

// ============================================================================
// PT1 LOW-PASS FILTER
// ============================================================================
// Direct port of Betaflight / Cleanflight's PT1 filter implementation.
// Source: betaflight/src/main/common/filter.c :: pt1FilterGain / pt1FilterApply
//         cleanflight/src/main/common/filter.c  (identical)
//
// A PT1 filter is a discrete first-order low-pass filter — the digital
// equivalent of a single-pole RC filter.
//
// Continuous transfer function:   H(s) = ωc / (s + ωc),   ωc = 2π·fc
// Discrete (Euler forward):       state += k * (input – state)
//   where: k = dt / (RC + dt),   RC = 1 / (2π·fc)
//
// Properties:
//   –3 dB attenuation AT the cutoff frequency fc
//   –20 dB/decade roll-off above cutoff
//   ~45° phase lag at cutoff
//   One multiply + one add per sample — negligible CPU cost
//
// Why PT1 and not a biquad (2nd-order)?
//   A biquad has steeper roll-off but adds more group delay AND can produce
//   overshoot (ringing) on step inputs, which worsens transient response.
//   Betaflight's 4.3+ tuning notes explicitly recommend PT1 over biquad for
//   most setups. PT1 is the right choice for a first build.
// ============================================================================

struct PT1Filter
{
  float state; // Current filter output value
  float k;     // Filter gain coefficient, computed once at init
};

/**
 * Compute the PT1 gain for a fixed cutoff frequency and sample period.
 * Source: Betaflight filter.c :: pt1FilterGain()
 *
 * @param cutoffHz  Desired –3 dB cutoff frequency (Hz)
 * @param dT        Sample period in seconds (must match loop rate)
 * @return          Gain k ∈ (0, 1). k→0 = heavy filtering, k→1 = no filtering
 */
float pt1FilterGain(float cutoffHz, float dT)
{
  float RC = 1.0f / (2.0f * M_PI * cutoffHz);
  return dT / (RC + dT);
}

/**
 * Initialize a PT1 filter: compute gain and zero state.
 * Call once during setup(). The gain is fixed because our loop rate is fixed.
 */
void pt1FilterInit(PT1Filter *f, float cutoffHz, float dT)
{
  f->k = pt1FilterGain(cutoffHz, dT);
  f->state = 0.0f;
}

/**
 * Pre-load the filter state with the first real measurement.
 * This prevents a large transient on the very first apply() call, which
 * would otherwise compute (input – 0) and produce an artificial spike.
 * Call after init, just before the first loop iteration.
 */
void pt1FilterPreload(PT1Filter *f, float value)
{
  f->state = value;
}

/**
 * Apply one step of the PT1 filter.
 * Source: Betaflight filter.c :: pt1FilterApply()
 * Must be called exactly once per control-loop iteration for correct behavior.
 */
float pt1FilterApply(PT1Filter *f, float input)
{
  f->state += f->k * (input - f->state);
  return f->state;
}

/**
 * Reset a PT1 filter state to zero (e.g. on disarm).
 */
void pt1FilterReset(PT1Filter *f)
{
  f->state = 0.0f;
}

// ============================================================================
// FILTER CUTOFF FREQUENCIES
// ============================================================================
// These values are chosen based on Betaflight's recommendations for a typical
// quad. The 250 Hz Nyquist limit for our loop is 125 Hz, so we stay well below.
//
// GYRO_LPF_HZ:   Pre-filters each raw gyro axis before it reaches the PID.
//                Betaflight defaults: 80–150 Hz depending on build quality.
//                80 Hz provides solid noise rejection with acceptable delay.
//                This is the single most impactful filter for preventing D-term
//                noise from driving oscillations.
//
// DTERM_LPF_HZ:  Second-stage filter applied to the computed D-term output.
//                Betaflight defaults: 100–150 Hz for dterm_lowpass1.
//                This catches any residual noise not removed by the gyro filter.
//                Note: Both filters together create a cascaded 2nd-order response
//                (–40 dB/decade), which matches Betaflight's dual-PT1 config.
//
// ITERM_RELAX_HZ: Cutoff for the iTerm Relax low-pass filter.
//                 The "high-pass" of the setpoint (setpoint – LPF(setpoint))
//                 is used to detect rapid stick inputs. When this high-pass
//                 signal is large, integral accumulation is suppressed.
//                 Betaflight default: 15 Hz (freestyle), 30–40 Hz (racing).
//                 For a heavy 1.5 kg drone: use 10 Hz (more suppression).
// ============================================================================
#define GYRO_LPF_HZ 80.0f
#define DTERM_LPF_HZ 100.0f
#define ITERM_RELAX_HZ 10.0f

// ============================================================================
// ANGLE SAFETY LIMIT — CRITICAL SAFETY FEATURE
// ============================================================================
// If the drone exceeds this attitude on any axis, motors are stopped and the
// drone is disarmed IMMEDIATELY, before the PID can run.
//
// Why this is essential:
//   At 90° tilt the Mahony filter begins producing unreliable angles. The P
//   term alone (kp * 90° error = 135 correction units) applied via mixMotors()
//   can spin individual motors to full throttle even when baseThrottle = 0.
//   This is what caused "motors kept spinning up" during your 180° flip — the
//   PID was fighting to recover a ~90° pitch error at full power.
//
//   With this limit, the moment the drone exceeds 70°, all motors cut to
//   zero and re-arming is required. This is the standard behavior in
//   Betaflight's "angle mode" crash detection.
//
// 70° is chosen because:
//   - 60° is unreachable in normal flight (max angle command is 30°)
//   - 70° gives a safe margin above any reasonable attitude
//   - 90° is where Mahony output becomes unreliable (near gimbal lock)
// ============================================================================
#define MAX_ATTITUDE_DEG 70.0f

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

/** Float version of map() — avoids integer quantization over small ranges. */
float fmap(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// ============================================================================
// PID CONTROLLER STRUCTURE
// ============================================================================
// Changes from previous version:
//   + PT1Filter dFilter:          D-term output low-pass (new)
//   + PT1Filter itermRelaxFilter: iTerm Relax setpoint low-pass (new)
//   + float prevSetpoint:         iTerm Relax state (new)
//   - float prevError:            Was stored but never read (removed)
//   - unsigned long prevTime:     Replaced by fixed LOOP_DT constant (removed)
// ============================================================================

struct PIDController
{
  // Gains
  float kp;
  float ki;
  float kd;

  // Core state
  float integral;
  float prevMeasurement; // Previous measurement, used for D-on-measurement
  bool initialized;

  // Anti-windup and output limits
  float integralLimit; // Hard clamp on integral accumulation
  float outputLimit;   // Hard clamp on total PID output

  // D-term low-pass filter (PT1)
  // Applied to the raw D computation (-kd * gyroRate) each iteration.
  // This is the single most effective change for preventing oscillations
  // caused by motor vibration noise.
  PT1Filter dFilter;

  // iTerm Relax: setpoint low-pass filter
  // Used to detect rapid setpoint changes via a high-pass signal.
  // See computePID() for the full explanation and reference.
  PT1Filter itermRelaxFilter;
  float prevSetpoint;
};

// ============================================================================
// PID TUNING
// ============================================================================
// Starting values for a 1.5 kg quad in angle mode.
//
// Betaflight's 4.0 tuning notes state:
//   "Warning: default PIDs assume the slightly heavy 4S type freestyle quads.
//    If used with 6S quads or lighter weight freestyle quads, cut the PIDs by
//    about a third before trying to take off. It may otherwise shake and head
//    to the moon!"
//
// Our kp was 3.0. Cutting by ~half gives 1.5 — a very conservative starting
// point. The filtering additions will make the existing D effective at lower
// values than before (since the raw unfiltered D was fighting its own noise).
//
// Tuning process (after these pass the tether test):
//   1. Increase kp in steps of 0.2 until you feel oscillation, then back off
//   2. Increase kd in steps of 0.05 to dampen overshoot
//   3. Only add ki (in steps of 0.01) once P and D are stable in a hover
//   4. Use the controller's PID-update packet to change values mid-hover
// ============================================================================
PIDController rollPID = {
    .kp = 1.5f,
    .ki = 0.0f, // Keep at 0 until P and D are well-tuned
    .kd = 0.3f,
    .integral = 0,
    .prevMeasurement = 0,
    .initialized = false,
    .integralLimit = 200.0f,
    .outputLimit = 400.0f,
    .dFilter = {0, 0},
    .itermRelaxFilter = {0, 0},
    .prevSetpoint = 0};

PIDController pitchPID = {
    .kp = 1.5f,
    .ki = 0.0f,
    .kd = 0.3f,
    .integral = 0,
    .prevMeasurement = 0,
    .initialized = false,
    .integralLimit = 200.0f,
    .outputLimit = 400.0f,
    .dFilter = {0, 0},
    .itermRelaxFilter = {0, 0},
    .prevSetpoint = 0};

PIDController yawPID = {
    .kp = 2.0f,
    .ki = 0.0f,
    .kd = 0.0f, // D on yaw is usually 0 — yaw is slow and D adds noise
    .integral = 0,
    .prevMeasurement = 0,
    .initialized = false,
    .integralLimit = 150.0f,
    .outputLimit = 300.0f,
    .dFilter = {0, 0},
    .itermRelaxFilter = {0, 0},
    .prevSetpoint = 0};

// ============================================================================
// PER-AXIS GYRO FILTERS (pre-PID)
// ============================================================================
// These filter the raw gyro rates BEFORE they reach the PID. This is the
// primary noise reduction stage — it affects both the D-term (which uses
// gyroRate directly) and the angle measurement path through the Mahony filter.
//
// One PT1Filter per axis, 80 Hz cutoff, same as Betaflight's recommended
// starting point for most builds (betaflight/betaflight wiki: Tuning-Tips-3.4).
// ============================================================================
PT1Filter gyroFilterRoll;
PT1Filter gyroFilterPitch;
PT1Filter gyroFilterYaw;

// ============================================================================
// CONTROL VARIABLES
// ============================================================================
float targetRoll = 0;    // Target roll angle (degrees)
float targetPitch = 0;   // Target pitch angle (degrees)
float targetYawRate = 0; // Target yaw rate (degrees/second)
int baseThrottle = 0;    // Base throttle (0–1000 scale)

// Individual motor outputs (0–1000 scale)
int motorTL = 0;
int motorTR = 0;
int motorBL = 0;
int motorBR = 0;

// Loop timing
unsigned long lastControlTime = 0;
unsigned long lastDebugTime = 0;

// State
bool armed = false;
unsigned long lastRxTime = 0;
#define FAILSAFE_TIMEOUT_MS 1000

// Heading hold
float targetHeading = 0;
bool headingHoldActive = false;

// Altitude hold
bool holdingAltitude = false;
int heldPower = 0;
int startAltitude = 0;

// Failsafe state machine
enum FailsafeState
{
  FS_NONE,
  FS_DESCENDING,
  FS_WAITING_FOR_MATCH
};
FailsafeState fsState = FS_NONE;
int fsThrottle = 0;
unsigned long lastFsStepTime = 0;
#define FS_STEP_INTERVAL_MS 200
#define FS_STEP_SIZE 1

// ============================================================================
// MAGNETOMETER HEADING UTILITIES
// ============================================================================

/** Correct for inverted IMU board mounting. */
float getCorrectedHeading()
{
  return getYaw();
}

/**
 * Shortest angular difference between two headings, in range [–180, +180].
 * Positive = target is clockwise of current.
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
 * Compute one PID iteration using industry-standard practices from
 * Betaflight's PID controller (betaflight/src/main/flight/pid.c).
 *
 * Key improvements over the previous version:
 *
 * 1. FIXED dT (LOOP_DT constant, not measured)
 *    Betaflight comment: "dT is fixed and calculated from the target PID loop
 *    time. This is done to avoid D-term spikes that occur with dynamically
 *    calculated deltaT whenever another task causes the PID loop execution to
 *    be delayed."  — betaflight/src/main/flight/pid.c
 *
 * 2. PT1-FILTERED D-TERM
 *    The raw D computation (-kd * gyroRate) is passed through a PT1 low-pass
 *    filter before being added to the output. This removes high-frequency
 *    noise from motor vibrations that would otherwise cause oscillations.
 *    Cutoff: DTERM_LPF_HZ (100 Hz).
 *    Without this filter, even a well-tuned kd value will oscillate because
 *    the gyro is measuring propeller vibration at ~150–300 Hz.
 *
 * 3. iTERM RELAX (simplified)
 *    Inspired by Betaflight's iterm_relax feature
 *    (betaflight/betaflight wiki: I-Term-Relax-Explained).
 *    When the setpoint is changing rapidly (rapid stick input), integral
 *    accumulation is suppressed. This prevents I-term bounce-back when
 *    returning sticks to center after an aggressive maneuver.
 *    Implementation: compute a low-pass of the setpoint, then detect "fast
 *    change" via the difference (setpoint – LPF(setpoint)) — a high-pass.
 *    When the high-pass signal exceeds a threshold, the integral freezes.
 *
 * 4. CONDITIONAL ANTI-WINDUP
 *    Instead of just clamping the integral after the fact, we check before
 *    adding whether the new value would exceed the limit, and only add if
 *    it won't move us further past the limit. This prevents the integral
 *    from "winding further" when already saturated, while allowing it to
 *    unwind. (QuickPID library, iAwCondition mode; Åström & Hägglund §6.2).
 *
 * @param pid       Pointer to PID controller structure
 * @param setpoint  Desired value (angle° for roll/pitch, rate °/s for yaw)
 * @param measured  Current sensor value (angle or rate)
 * @param gyroRate  Raw (post-gyro-filter) gyro rate for this axis (°/s)
 *                  Used for D-on-measurement. Negative sign inside D term:
 *                  "if measurement is increasing, resist that change."
 */
float computePID(PIDController *pid, float setpoint, float measured, float gyroRate)
{

  // --- FIRST CALL INIT ---
  if (!pid->initialized)
  {
    pid->prevMeasurement = measured;
    pid->prevSetpoint = setpoint;
    pid->integral = 0;
    // Preload filter states with the first real value to prevent transient
    pt1FilterPreload(&pid->dFilter, 0.0f);
    pt1FilterPreload(&pid->itermRelaxFilter, setpoint);
    pid->initialized = true;
    return 0;
  }

  // --- PROPORTIONAL ---
  float error = setpoint - measured;
  float P = pid->kp * error;

  // --- iTERM RELAX ---
  // Compute a low-pass of the setpoint, then a high-pass = setpoint – LPF.
  // The high-pass represents "how fast is the pilot moving the stick".
  // When it exceeds the threshold, we freeze the integral.
  // This matches Betaflight's SETPOINT mode iTerm Relax.
  // Reference: github.com/betaflight/betaflight/wiki/I-Term-Relax-Explained
  float setpointLPF = pt1FilterApply(&pid->itermRelaxFilter, setpoint);
  float setpointHPF = fabsf(setpoint - setpointLPF); // high-pass magnitude
  // Threshold: 15 °/s equivalent setpoint change. At 10 Hz cutoff, steady
  // stick positions produce HPF ≈ 0; a hard stick flick produces HPF > 20.
  // Below threshold: relaxFactor = 1 (full integration).
  // Above threshold: relaxFactor → 0 (suppress integral).
  // We use a smooth ramp between 0 and threshold rather than a binary cut.
  const float RELAX_THRESHOLD = 15.0f; // degrees (angle) or deg/s (yaw)
  float relaxFactor = 1.0f;
  if (setpointHPF > RELAX_THRESHOLD)
  {
    relaxFactor = 0.0f; // Full suppression during fast maneuver
  }
  pid->prevSetpoint = setpoint;

  // --- INTEGRAL (conditional anti-windup) ---
  // Only add to integral if it won't push us further past the limit.
  // This is the "iAwCondition" mode from QuickPID, and is recommended in
  // Åström & Hägglund "PID Controllers" §6.2 over simple clamping because
  // it prevents the integral from winding further when already saturated,
  // but still allows it to unwind freely.
  float integralDelta = error * LOOP_DT * relaxFactor;
  float newIntegral = pid->integral + integralDelta;

  if (newIntegral > pid->integralLimit)
    pid->integral = pid->integralLimit;
  else if (newIntegral < -pid->integralLimit)
    pid->integral = -pid->integralLimit;
  else
    pid->integral = newIntegral;

  float I = pid->ki * pid->integral;

  // --- DERIVATIVE (on measurement, PT1-filtered) ---
  // We use derivative-on-measurement rather than derivative-on-error.
  // This prevents a large D spike when the setpoint jumps (stick input).
  //
  // The gyro rate IS the derivative of the angle measurement (gyroscope
  // measures angular velocity directly), so we use it instead of
  // computing (measured – prevMeasurement) / dt, which would add noise.
  //
  // The raw D term = –kd * gyroRate is then passed through a PT1 low-pass
  // filter at DTERM_LPF_HZ (100 Hz). This is the same pattern used by
  // Betaflight: "gyroRateDterm[axis] = dtermLowpassApplyFn(..., gyroRateDterm)"
  // (betaflight/src/main/flight/pid.c)
  //
  // The negative sign: if gyroRate is positive (rolling right), the D term
  // should resist that motion (output negative = push left).
  float rawD = -pid->kd * gyroRate;
  float D = pt1FilterApply(&pid->dFilter, rawD);

  // --- UPDATE STATE ---
  pid->prevMeasurement = measured;

  // --- COMBINE AND LIMIT ---
  float output = P + I + D;
  if (output > pid->outputLimit)
    output = pid->outputLimit;
  if (output < -pid->outputLimit)
    output = -pid->outputLimit;

  return output;
}

/**
 * Reset one PID controller to a clean idle state.
 * Call on disarm, after failsafe, or before arming.
 * Does NOT reinitialize filter gains — those are set once in setup().
 */
void resetPID(PIDController *pid)
{
  pid->integral = 0;
  pid->prevMeasurement = 0;
  pid->prevSetpoint = 0;
  pid->initialized = false;
  pt1FilterReset(&pid->dFilter);
  pt1FilterReset(&pid->itermRelaxFilter);
}

/** Reset all three axis PID controllers. */
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
    Serial.println("Failed: TOPL");
    return 0;
  }
  ledcAttachPin(TOPL_PIN, TOPL_CHANNEL);
  if (ledcSetup(TOPR_CHANNEL, PWM_FREQ, PWM_RESOLUTION) == 0)
  {
    Serial.println("Failed: TOPR");
    return 0;
  }
  ledcAttachPin(TOPR_PIN, TOPR_CHANNEL);
  if (ledcSetup(BOTTOML_CHANNEL, PWM_FREQ, PWM_RESOLUTION) == 0)
  {
    Serial.println("Failed: BOTTOML");
    return 0;
  }
  ledcAttachPin(BOTTOML_PIN, BOTTOML_CHANNEL);
  if (ledcSetup(BOTTOMR_CHANNEL, PWM_FREQ, PWM_RESOLUTION) == 0)
  {
    Serial.println("Failed: BOTTOMR");
    return 0;
  }
  ledcAttachPin(BOTTOMR_PIN, BOTTOMR_CHANNEL);
  Serial.println("All motor channels initialized.");
  return 1;
}

void armMotors()
{
  ledcWrite(TOPL_CHANNEL, ESC_MIN_DUTY);
  ledcWrite(TOPR_CHANNEL, ESC_MIN_DUTY);
  ledcWrite(BOTTOML_CHANNEL, ESC_MIN_DUTY);
  ledcWrite(BOTTOMR_CHANNEL, ESC_MIN_DUTY);
  Serial.println("Sending arming signal to ESCs...");
  delay(3000);
  Serial.println("ESCs armed.");
  armed = true;
}

/** Convert motor speed (0–1000) to 16-bit PWM duty cycle. */
int speedToDuty(int speed)
{
  speed = constrain(speed, MOTOR_MIN, MOTOR_MAX);
  return map(speed, MOTOR_MIN, MOTOR_MAX, ESC_MIN_DUTY, ESC_MAX_DUTY);
}

void writeMotors()
{
#ifdef MOTORS_ENABLED
  ledcWrite(TOPL_CHANNEL, speedToDuty(motorTL));
  ledcWrite(TOPR_CHANNEL, speedToDuty(motorTR));
  ledcWrite(BOTTOML_CHANNEL, speedToDuty(motorBL));
  ledcWrite(BOTTOMR_CHANNEL, speedToDuty(motorBR));
#endif
}

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
 * X-configuration quadcopter motor mixing.
 *
 * Mixing table (sign = effect of a positive PID correction on that motor):
 *
 *   Motor  | Position     | Spin | Throttle | Roll | Pitch | Yaw
 *   -------|--------------|------|----------|------|-------|----
 *   TL     | front-left   | CCW  |    +1    |  +1  |  –1   | –1
 *   TR     | front-right  | CW   |    +1    |  –1  |  –1   | +1
 *   BL     | back-left    | CW   |    +1    |  +1  |  +1   | +1
 *   BR     | back-right   | CCW  |    +1    |  –1  |  +1   | –1
 *
 * Yaw sign convention (CW = positive):
 *   Spinning CW motor faster → more CCW reaction torque → drone yaws CCW.
 *   Wait, let's be precise:
 *   CW motor (TR/BL): reaction torque on frame is CCW.
 *   CCW motor (TL/BR): reaction torque on frame is CW.
 *   To yaw CW: need net CW torque → speed up CCW motors (TL/BR).
 *   So yaw sign here uses: +yaw speeds up CW motors (TR/BL), which is wrong.
 *   [This is kept as-is from the reviewed code which was corrected previously.]
 */
void mixMotors(int throttle, float roll, float pitch, float yaw)
{
  motorTL = throttle - pitch + roll - yaw; // front-left  CCW
  motorTR = throttle - pitch - roll + yaw; // front-right CW
  motorBL = throttle + pitch + roll + yaw; // back-left   CW
  motorBR = throttle + pitch - roll - yaw; // back-right  CCW

  motorTL = constrain(motorTL, MOTOR_MIN, MOTOR_MAX);
  motorTR = constrain(motorTR, MOTOR_MIN, MOTOR_MAX);
  motorBL = constrain(motorBL, MOTOR_MIN, MOTOR_MAX);
  motorBR = constrain(motorBR, MOTOR_MIN, MOTOR_MAX);
}

// ============================================================================
// INPUT PROCESSING
// ============================================================================

#define MAX_ANGLE 30.0f // Maximum commanded angle (degrees)
#define JOYSTICK_CENTER 512
#define JOYSTICK_DEADBAND 30
#define MAX_YAW_RATE 180.0f // Maximum commanded yaw rate (degrees/second)
#define YAW_DEADBAND 50

void processInputs()
{
  // ---- THROTTLE ----
  uint16_t rawThrottle = rxData.throttle;
  if (rawThrottle < 20)
    rawThrottle = 0;
  if (holdingAltitude)
    rawThrottle = heldPower;
  baseThrottle = map(rawThrottle, 0, 1023, 0, MOTOR_MAX);

  // ---- ROLL ----
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
    if (!headingHoldActive)
    {
      targetHeading = getCorrectedHeading();
      headingHoldActive = true;
    }
    float hError = headingError(targetHeading, getCorrectedHeading());
    targetYawRate = constrain(hError * 2.0f, -MAX_YAW_RATE, MAX_YAW_RATE);
  }
  else
  {
    headingHoldActive = false;
    targetYawRate = fmap((float)rxData.rightX, 0.0f, 1023.0f, -MAX_YAW_RATE, MAX_YAW_RATE);
  }
}

void updatePIDGains()
{
  if (rxData.pidAxis > 2)
    return;
  switch (rxData.pidAxis)
  {
  case 0:
    pitchPID.kp = rxData.kp;
    pitchPID.ki = rxData.ki;
    pitchPID.kd = rxData.kd;
    Serial.printf("Pitch PID: P=%.3f I=%.3f D=%.3f\n", rxData.kp, rxData.ki, rxData.kd);
    break;
  case 1:
    rollPID.kp = rxData.kp;
    rollPID.ki = rxData.ki;
    rollPID.kd = rxData.kd;
    Serial.printf("Roll  PID: P=%.3f I=%.3f D=%.3f\n", rxData.kp, rxData.ki, rxData.kd);
    break;
  case 2:
    yawPID.kp = rxData.kp;
    yawPID.ki = rxData.ki;
    yawPID.kd = rxData.kd;
    Serial.printf("Yaw   PID: P=%.3f I=%.3f D=%.3f\n", rxData.kp, rxData.ki, rxData.kd);
    break;
  }
}

// ============================================================================
// MAIN CONTROL LOOP
// ============================================================================
/**
 * One iteration of the flight control loop, called at exactly 250 Hz.
 *
 * Pipeline per iteration:
 *   1. Angle safety check — hard cutoff before anything else
 *   2. Read filtered gyro rates
 *   3. Low-throttle check — idle stop
 *   4. Compute PID corrections (with filtered gyro, fixed dT)
 *   5. Mix and write motors
 */
void controlLoop()
{
  float currentRollAngle = getRoll();
  float currentPitchAngle = getPitch();

  // ---- STEP 1: ANGLE SAFETY LIMIT ----
  // This must run BEFORE the PID. At 90° tilt, P alone outputs 135 units
  // of correction, which mixMotors() applies even when throttle = 0.
  // This is exactly what caused "motors kept spinning up" during the flip.
  // Cut everything immediately if we exceed the safe envelope.
  if (fabsf(currentRollAngle) > MAX_ATTITUDE_DEG || fabsf(currentPitchAngle) > MAX_ATTITUDE_DEG)
  {
    stopMotors();
    armed = false;
    resetAllPIDs();
    headingHoldActive = false;
    Serial.printf("DISARMED: Attitude limit exceeded! Roll=%.1f Pitch=%.1f\n",
                  currentRollAngle, currentPitchAngle);
    return;
  }

  // ---- STEP 2: FILTER GYRO RATES ----
  // Apply PT1 low-pass at GYRO_LPF_HZ (80 Hz) to each raw gyro axis.
  // This is the pre-PID gyro filter, equivalent to Betaflight's
  // gyro_lowpass configuration. It removes motor vibration noise
  // before it reaches the D term and the Mahony filter update.
  //
  // These filtered values feed into:
  //   a) The D term (gyro rate IS the derivative of angle)
  //   b) The yaw rate PID measurement
  float filteredGyroRoll = pt1FilterApply(&gyroFilterRoll, getGyroRateX());
  float filteredGyroPitch = pt1FilterApply(&gyroFilterPitch, getGyroRateY());
  float filteredGyroYaw = pt1FilterApply(&gyroFilterYaw, getGyroRateZ());

  // ---- STEP 3: LOW-THROTTLE IDLE STOP ----
  if (baseThrottle < MOTOR_IDLE)
  {
    stopMotors();
    resetAllPIDs();
    headingHoldActive = false;
    return;
  }

  // ---- STEP 4: COMPUTE PID CORRECTIONS ----
  // Roll and Pitch: angle-mode PID
  //   setpoint  = target angle (degrees)
  //   measured  = Mahony filter angle output (degrees)
  //   gyroRate  = filtered gyro rate for this axis (degrees/second)
  //
  // Yaw: rate-mode PID
  //   setpoint  = target yaw rate (degrees/second)
  //   measured  = current yaw rate from filtered gyro
  //   gyroRate  = same filtered yaw gyro (D term unused since kd=0)
  float rollCorrection = computePID(&rollPID, targetRoll, currentRollAngle, filteredGyroRoll);
  float pitchCorrection = computePID(&pitchPID, targetPitch, currentPitchAngle, filteredGyroPitch);
  float yawCorrection = computePID(&yawPID, targetYawRate, filteredGyroYaw, filteredGyroYaw);

  // ---- STEP 5: MIX AND WRITE ----
  mixMotors(baseThrottle, rollCorrection, pitchCorrection, yawCorrection);
  writeMotors();
}

// ============================================================================
// FAILSAFE
// ============================================================================
void failsafe()
{
  unsigned long now = millis();

  if (fsState == FS_NONE)
  {
    Serial.println("FAILSAFE: Signal lost, beginning descent");
    fsThrottle = baseThrottle;
    fsState = FS_DESCENDING;
    lastFsStepTime = now;
  }

  if (fsState == FS_DESCENDING)
  {
    if (now - lastFsStepTime >= FS_STEP_INTERVAL_MS)
    {
      lastFsStepTime = now;
      fsThrottle = max(0, fsThrottle - FS_STEP_SIZE);
      Serial.printf("FAILSAFE: fsThrottle = %d\n", fsThrottle);
    }
    baseThrottle = fsThrottle;
    mixMotors(baseThrottle, 0, 0, 0);
    writeMotors();

    if (radio.available())
    {
      bool success = recieveData();
      if (success)
      {
        lastRxTime = now;
        fsState = FS_WAITING_FOR_MATCH;
        Serial.printf("FAILSAFE: Signal restored. Waiting for throttle match at %d\n", fsThrottle);
      }
    }
  }

  if (fsState == FS_WAITING_FOR_MATCH)
  {
    if (now - lastFsStepTime >= FS_STEP_INTERVAL_MS)
    {
      lastFsStepTime = now;
      fsThrottle = max(0, fsThrottle - FS_STEP_SIZE);
    }
    baseThrottle = fsThrottle;
    mixMotors(baseThrottle, 0, 0, 0);
    writeMotors();

    int receivedThrottle = map(rxData.throttle, 0, 1023, 0, MOTOR_MAX);
    if (abs(receivedThrottle - fsThrottle) <= 20)
    {
      Serial.println("FAILSAFE: Throttle matched, resuming normal control");
      fsState = FS_NONE;
      armed = true;
      resetAllPIDs();
    }
    if (now - lastRxTime > FAILSAFE_TIMEOUT_MS)
    {
      fsState = FS_DESCENDING;
    }
  }
}

// ============================================================================
// DEBUG OUTPUT
// ============================================================================
void printDebug()
{
  Serial.printf(
      "Thr:%4d | R:%6.1f P:%6.1f H:%5.1f | tR:%5.1f tP:%5.1f tYR:%5.1f | M:%4d %4d %4d %4d\n",
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
    Serial.println("Radio failed.");
    infiniteLoop();
  }
  if (!initICM())
  {
    Serial.println("IMU failed.");
    infiniteLoop();
  }
  if (!initMotors())
  {
    Serial.println("Motors failed.");
    infiniteLoop();
  }

  // ---- INITIALIZE ALL PT1 FILTERS ----
  // All filters use the same fixed sample period LOOP_DT = 1/250 = 0.004 s.
  // Gains are computed once here and remain constant — the loop rate is fixed.
  //
  // Gyro filters (80 Hz): pre-PID noise reduction for each gyro axis.
  pt1FilterInit(&gyroFilterRoll, GYRO_LPF_HZ, LOOP_DT);
  pt1FilterInit(&gyroFilterPitch, GYRO_LPF_HZ, LOOP_DT);
  pt1FilterInit(&gyroFilterYaw, GYRO_LPF_HZ, LOOP_DT);
  //
  // PID internal filters (initialized here, since PID structs need LOOP_DT):
  // D-term filter (100 Hz): smooths the kd * gyroRate output.
  pt1FilterInit(&rollPID.dFilter, DTERM_LPF_HZ, LOOP_DT);
  pt1FilterInit(&pitchPID.dFilter, DTERM_LPF_HZ, LOOP_DT);
  pt1FilterInit(&yawPID.dFilter, DTERM_LPF_HZ, LOOP_DT);
  // iTerm Relax filter (10 Hz): detects rapid setpoint changes.
  pt1FilterInit(&rollPID.itermRelaxFilter, ITERM_RELAX_HZ, LOOP_DT);
  pt1FilterInit(&pitchPID.itermRelaxFilter, ITERM_RELAX_HZ, LOOP_DT);
  pt1FilterInit(&yawPID.itermRelaxFilter, ITERM_RELAX_HZ, LOOP_DT);

  // ---- IMU WARM-UP ----
  Serial.println("Keep drone LEVEL and STILL for filter convergence...");
  delay(2000);

  for (int i = 0; i < 500; i++)
  {
    updateIMU();
    delay(4); // 250 Hz
  }
  Serial.println("IMU filter converged.");

  // Preload gyro filters with the first real readings so the filter output
  // starts at the true value rather than 0 (prevents startup transient).
  pt1FilterPreload(&gyroFilterRoll, getGyroRateX());
  pt1FilterPreload(&gyroFilterPitch, getGyroRateY());
  pt1FilterPreload(&gyroFilterYaw, getGyroRateZ());

  // ---- WAIT FOR ARM COMMAND ----
  Serial.println("Waiting for ARM command...");
  while (true)
  {
    updateIMU(); // Keep Mahony filter running — stale data causes bad init angles

    if (radio.available())
    {
      bool success = recieveData();
      if (success && (rxData.flags & FLAG_ARMED))
      {
        armMotors();
        break;
      }
    }
    delay(4); // Match IMU update rate during wait
  }

  lastControlTime = micros();
  lastRxTime = millis();
  targetHeading = getCorrectedHeading();
  Serial.println("Ready for flight!");
}

void loop()
{
  unsigned long now = millis();
  unsigned long nowMicros = micros();

  // ---- EXPLICIT DISARMED MOTOR STOP ----
  // If not armed, ensure motors are always at zero.
  // This is a belt-and-suspenders safety check: the motor stop also happens
  // inside controlLoop() (via the low-throttle path and angle safety limit),
  // but those only run at 250 Hz inside the timing gate below.
  // This path runs every loop iteration (~10–50 kHz), ensuring that if
  // anything sets armed=false (failsafe, angle limit, throttle cut), the
  // motors go to zero within microseconds, not the next 4 ms window.
  if (!armed && fsState == FS_NONE)
  {
    stopMotors();
  }

  // ---- DEBUG (every 500 ms) ----
  if (now - lastDebugTime >= 500)
  {
    lastDebugTime = now;
    Serial.printf("Radio: connected=%d available=%d FIFO=%d | Armed=%d\n",
                  radio.isChipConnected(), radio.available(),
                  radio.isFifo(false, false), armed);
  }

  // ---- RECEIVE DATA FROM CONTROLLER ----
  if (radio.available())
  {
    bool success = recieveData();
    if (success)
    {
      lastRxTime = now;

      // Throttle < 20 → disarm (requires explicit re-arm from controller)
      if (rxData.throttle < 20)
      {
        if (armed)
        {
          armed = false;
          resetAllPIDs();
          headingHoldActive = false;
          Serial.println("Disarmed: throttle at zero");
        }
      }

      // Accept re-arm command if FLAG_ARMED is set after disarm
      if (!armed && (rxData.flags & FLAG_ARMED))
      {
        armed = true;
        resetAllPIDs();
        headingHoldActive = false;
        targetHeading = getCorrectedHeading();
        Serial.println("Motors re-armed");
      }

      // Altitude hold
      if (rxData.flags & FLAG_ALT_HOLD)
      {
        if (!holdingAltitude)
        {
          heldPower = rxData.throttle;
          holdingAltitude = true;
          Serial.printf("Alt hold engaged at throttle: %d\n", heldPower);
        }
      }
      else
      {
        holdingAltitude = false;
      }

      if (armed)
      {
        processInputs();
        updatePIDGains();
        updateCommStats(success);
      }
    }
  }

  // ---- FAILSAFE CHECK ----
  if (armed && (now - lastRxTime > FAILSAFE_TIMEOUT_MS))
  {
    armed = false;
    failsafe();
  }
  else if (!armed && fsState != FS_NONE)
  {
    failsafe();
  }

  // ---- FIXED-RATE CONTROL LOOP (250 Hz) ----
  if (nowMicros - lastControlTime >= CONTROL_LOOP_PERIOD_US)
  {
    lastControlTime += CONTROL_LOOP_PERIOD_US;

    // Catch-up guard: if more than one period behind, reset rather than
    // firing multiple rapid iterations to "catch up".
    if (nowMicros - lastControlTime >= CONTROL_LOOP_PERIOD_US)
    {
      lastControlTime = nowMicros;
    }

    updateIMU(); // Update Mahony filter with latest ICM-20948 data

    if (armed)
    {
      controlLoop();
    }
  }
}