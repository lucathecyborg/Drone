#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Arduino.h>
#include <MPU6050_light.h>

RF24 radio(2, 4); // CE, CSN
MPU6050 mpu(Wire);

const byte address[6] = "NODE1";

struct joystickValues
{
  uint16_t x;
  uint16_t y;
  bool button;
};

struct message
{
  uint16_t pot1;
  joystickValues joystickL;
  joystickValues joystickR;

  float roll_kp, roll_ki, roll_kd;
  float pitch_kp, pitch_ki, pitch_kd;
  float yaw_kp, yaw_ki, yaw_kd;
};

message Data;

// PWM config for motors
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

// PID structure
struct PID {
  float kp, ki, kd;
  float prevError;
  float integral;
  float integralLimit;
  unsigned long lastTime;
};

// PID controllers for roll, pitch, yaw
PID rollPID = {1.5, 0.05, 0.8, 0, 0, 400, 0};
PID pitchPID = {1.5, 0.05, 0.8, 0, 0, 400, 0};
PID yawPID = {2.0, 0.02, 0.5, 0, 0, 400, 0};

// Target angles from joystick
float targetRoll = 0;
float targetPitch = 0;
float targetYaw = 0;

int base_motor_speed;
uint16_t power = 69;

int topL_speed;
int topR_speed;
int bottomL_speed;
int bottomR_speed;

unsigned long lastPrint = 0;
unsigned long lastIMURead = 0;

float computePID(PID *pid, float setpoint, float measured) {
  unsigned long now = micros();
  float dt = (now - pid->lastTime) / 1000000.0;
  pid->lastTime = now;
  
  if (dt > 0.5) return 0; // Safety check
  
  float error = setpoint - measured;
  
  // Proportional
  float P = pid->kp * error;
  
  // Integral with anti-windup
  pid->integral += error * dt;
  pid->integral = constrain(pid->integral, -pid->integralLimit, pid->integralLimit);
  float I = pid->ki * pid->integral;
  
  // Derivative
  float D = pid->kd * (error - pid->prevError) / dt;
  pid->prevError = error;
  
  return P + I + D;
}

void resetPID(PID *pid) {
  pid->integral = 0;
  pid->prevError = 0;
  pid->lastTime = micros();
}

void setup()
{
  Serial.begin(115200);
  Wire.begin();

  // Initialize MPU6050
  byte status = mpu.begin();
  Serial.print("MPU6050 status: ");
  Serial.println(status);
  
  if (status != 0) {
    Serial.println("MPU6050 connection failed!");
    while (1);
  }
  
  Serial.println("Calibrating gyro... Keep drone still!");
  delay(1000);
  mpu.calcOffsets();
  Serial.println("Calibration complete!");

  if (!radio.begin())
  {
    Serial.println("NRF24L01 not responding");
    while (1);
  }

  radio.setPALevel(RF24_PA_LOW);
  radio.openReadingPipe(0, address);
  radio.enableAckPayload();
  radio.startListening();

  // Setup PWM for all ESCs
  ledcSetup(TOPL_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(TOPL_PIN, TOPL_CHANNEL);
  ledcSetup(TOPR_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(TOPR_PIN, TOPR_CHANNEL);
  ledcSetup(BOTTOML_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(BOTTOML_PIN, BOTTOML_CHANNEL);
  ledcSetup(BOTTOMR_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(BOTTOMR_PIN, BOTTOMR_CHANNEL);

  // Arm all ESCs
  int armDuty = map(1000, 1000, 2000, 3276, 6553);
  ledcWrite(TOPL_CHANNEL, armDuty);
  ledcWrite(TOPR_CHANNEL, armDuty);
  ledcWrite(BOTTOML_CHANNEL, armDuty);
  ledcWrite(BOTTOMR_CHANNEL, armDuty);
  Serial.println("Sending arming signal to ESCs...");
  delay(3000);
  Serial.println("ESCs armed.");

  // Initialize PID timers
  resetPID(&rollPID);
  resetPID(&pitchPID);
  resetPID(&yawPID);

  while (!radio.available()) {
    // Wait for radio
  }
}

void loop()
{
  unsigned long now = millis();
  
  // Update MPU6050 data
  mpu.update();
  
  if (radio.available())
  {
    radio.read(&Data, sizeof(Data));

    // Update PID parameters from received data
      rollPID.kp = Data.roll_kp;
      rollPID.ki = Data.roll_ki;
      rollPID.kd = Data.roll_kd;
      
      pitchPID.kp = Data.pitch_kp;
      pitchPID.ki = Data.pitch_ki;
      pitchPID.kd = Data.pitch_kd;
      
      yawPID.kp = Data.yaw_kp;
      yawPID.ki = Data.yaw_ki;
      yawPID.kd = Data.yaw_kd;

    base_motor_speed = map(Data.pot1, 0, 1023, 0, 150);

    // Map joystick to target angles (±30 degrees)
    targetRoll = map(Data.joystickL.x, 0, 1023, -30, 30);
    targetPitch = map(Data.joystickL.y, 0, 1023, -30, 30);
    
    // Use right joystick for yaw rate
    float yawRate = map(Data.joystickR.x, 0, 1023, -100, 100);
    targetYaw += yawRate * 0.01;

    // Only apply PID when throttle is above minimum
    if (base_motor_speed > 30)
    {
      // Get current angles from MPU6050
      float currentRoll = mpu.getAngleX();
      float currentPitch = mpu.getAngleY();
      float currentYaw = mpu.getAngleZ();
      
      // Compute PID corrections
      float rollCorrection = computePID(&rollPID, targetRoll, currentRoll);
      float pitchCorrection = computePID(&pitchPID, targetPitch, currentPitch);
      float yawCorrection = computePID(&yawPID, targetYaw, currentYaw);

      // Apply corrections to motors
      topL_speed = base_motor_speed - pitchCorrection - rollCorrection + yawCorrection;
      topR_speed = base_motor_speed - pitchCorrection + rollCorrection - yawCorrection;
      bottomL_speed = base_motor_speed + pitchCorrection - rollCorrection - yawCorrection;
      bottomR_speed = base_motor_speed + pitchCorrection + rollCorrection + yawCorrection;

      // Constrain motor speeds
      topL_speed = constrain(topL_speed, 0, 180);
      topR_speed = constrain(topR_speed, 0, 180);
      bottomL_speed = constrain(bottomL_speed, 0, 180);
      bottomR_speed = constrain(bottomR_speed, 0, 180);
    }
    else
    {
      // Reset PID when disarmed
      topL_speed = base_motor_speed;
      topR_speed = base_motor_speed;
      bottomL_speed = base_motor_speed;
      bottomR_speed = base_motor_speed;
      
      resetPID(&rollPID);
      resetPID(&pitchPID);
      resetPID(&yawPID);
    }

    // Write to all motors
    ledcWrite(TOPL_CHANNEL, map(topL_speed, 0, 180, 3276, 6553));
    ledcWrite(TOPR_CHANNEL, map(topR_speed, 0, 180, 3276, 6553));
    ledcWrite(BOTTOML_CHANNEL, map(bottomL_speed, 0, 180, 3276, 6553));
    ledcWrite(BOTTOMR_CHANNEL, map(bottomR_speed, 0, 180, 3276, 6553));

    if (now - lastPrint > 200)
    {
      lastPrint = now;
      Serial.println("R:" + String(mpu.getAngleX(), 1) + " P:" + String(mpu.getAngleY(), 1) + 
                     " Y:" + String(mpu.getAngleZ(), 1) + " | LT:" + String(topL_speed) + 
                     " RT:" + String(topR_speed) + " LB:" + String(bottomL_speed) + 
                     " RB:" + String(bottomR_speed));
    }
    
    radio.writeAckPayload(1, &power, sizeof(power));
  }
}