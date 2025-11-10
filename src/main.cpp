#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Arduino.h>
#include <MPU6050_light.h>

// Pin definitions for e01-mlodp5 antenna
#define CE_PIN 4
#define CSN_PIN 5

RF24 radio(CE_PIN, CSN_PIN); 
const byte address[6] = "NODE1";

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


//Init MPU6050
MPU6050 mpu(Wire);



//Data structure for joysticks
struct joystickValues
{
  uint16_t x;
  uint16_t y;
  bool button;
};

//Data structure for incomming radio communication (max 32 bytes)
struct message
{
  uint16_t pot1;
  joystickValues joystickL;
  joystickValues joystickR;
  uint8_t PidAxis=3; // 0 = pitch, 1 = roll, 2 = yaw
  float kp=1.5, ki=0.05, kd=0.8;

};

// Received data
message Data;



// PID structure
struct PID {
  float kp, ki, kd;
  float prevError;
  float integral;
  float integralLimit;
  unsigned long lastTime;
};

// PID controllers for roll, pitch (angle mode), yaw (rate mode)
PID rollPID = {0.8, 0.02, 0.4, 0, 0, 400, 0};
PID pitchPID = {0.8, 0.02, 0.4, 0, 0, 400, 0};
PID yawPID = {1.5, 0.01, 0.05, 0, 0, 400, 0}; // Rate mode gains

// Target angles/rates from joystick
float targetRoll = 0;
float targetPitch = 0;
float targetYawRate = 0; // Changed to rate instead of angle

int base_motor_speed;
uint16_t power = 69;

int topL_speed;
int topR_speed;
int bottomL_speed;
int bottomR_speed;

unsigned long lastPrint = 0;

// PID computation function
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

// Reset PID controller
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
  // NOTE: MPU6050 is mounted FLAT (chip and LED facing UP)
  // X-axis = Roll, Y-axis = Pitch, Z-axis = Yaw
  byte status = mpu.begin();
  Serial.print("MPU6050 status: ");
  Serial.println(status);
  
  if (status != 0) {
    Serial.println("MPU6050 connection failed!");
    while (1);
  }
  
  Serial.println("Calibrating gyro... Keep drone FLAT and STILL!");
  delay(1000);
  mpu.calcOffsets();
  Serial.println("Calibration complete!"); 
  
  Serial.println("\nInitializing SPI bus (VSPI)...");
  SPI.begin(18, 19, 23, 5); // SCK, MISO, MOSI, SS
  delay(100);
  
  // Force CSN high before radio.begin()
  digitalWrite(5, HIGH);
  delay(10);

  if (!radio.begin(&SPI, CE_PIN, CSN_PIN))
  {
    Serial.println("NRF24L01 not responding");
    while (1);
  }

  radio.setPALevel(RF24_PA_LOW);
  radio.setDataRate(RF24_1MBPS);
  radio.setChannel(108);
  delay(100);
  radio.openReadingPipe(1, address);
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

// Read incoming data and update PID gains if needed
void readData(){
  radio.read(&Data, sizeof(Data));
  if(Data.PidAxis < 3){
    switch(Data.PidAxis){
      case 0:
        pitchPID.kp = Data.kp;
        pitchPID.ki = Data.ki;
        pitchPID.kd = Data.kd;
        break;
      case 1:
        rollPID.kp = Data.kp;
        rollPID.ki = Data.ki;
        rollPID.kd = Data.kd;
        break;
      case 2:
        yawPID.kp = Data.kp;
        yawPID.ki = Data.ki;
        yawPID.kd = Data.kd;
        break;
    }
  }
}

// Process joystick input and set target angles/rates
void processJoystickInput() {
  // Increased throttle range and minimum for ESC response
  base_motor_speed = map(Data.pot1, 0, 1023, 40, 180);

  // Map joystick to target angles (±30 degrees for roll/pitch)
  targetRoll = map(Data.joystickL.x, 0, 1023, -30, 30);
  targetPitch = map(Data.joystickL.y, 0, 1023, -30, 30);
  
  // Apply deadband to prevent drift
  if (abs(targetRoll) < 3.0) targetRoll = 0;
  if (abs(targetPitch) < 3.0) targetPitch = 0;
  
  // Use right joystick for yaw RATE (degrees per second)
  targetYawRate = map(Data.joystickR.x, 0, 1023, -150, 150);
  if (abs(targetYawRate) < 10) targetYawRate = 0; // Deadband for yaw
}

// Compute all PID corrections based on current sensor data
void computePIDCorrections(float &rollCorrection, float &pitchCorrection, float &yawCorrection) {
  // Get current angles and rates from MPU6050 (mounted flat)
  float currentRoll = mpu.getAngleX();   // X-axis = Roll
  float currentPitch = mpu.getAngleY();  // Y-axis = Pitch
  float currentYawRate = mpu.getGyroZ(); // Z-axis gyro rate for yaw
  
  // Compute PID corrections
  rollCorrection = computePID(&rollPID, targetRoll, currentRoll);
  pitchCorrection = computePID(&pitchPID, targetPitch, currentPitch);
  yawCorrection = computePID(&yawPID, targetYawRate, currentYawRate);

  // Limit corrections to prevent motor saturation
  float maxCorrection = min(60.0f, (180.0f - base_motor_speed) / 2.0f);
  rollCorrection = constrain(rollCorrection, -maxCorrection, maxCorrection);
  pitchCorrection = constrain(pitchCorrection, -maxCorrection, maxCorrection);
  yawCorrection = constrain(yawCorrection, -40.0f, 40.0f);
}

// Calculate motor speeds based on corrections
void calculateMotorSpeeds(float rollCorrection, float pitchCorrection, float yawCorrection) {
  // Apply corrections to motors (X configuration, MPU flat)
  // Motor layout:
  //     FRONT
  //   topL  topR
  //      \ /
  //       X
  //      / \
  // bottomL bottomR
  //     BACK
  topL_speed = base_motor_speed + pitchCorrection - rollCorrection - yawCorrection;
  topR_speed = base_motor_speed + pitchCorrection + rollCorrection + yawCorrection;
  bottomL_speed = base_motor_speed - pitchCorrection - rollCorrection + yawCorrection;
  bottomR_speed = base_motor_speed - pitchCorrection + rollCorrection - yawCorrection;

  // Constrain motor speeds
  topL_speed = constrain(topL_speed, 0, 180);
  topR_speed = constrain(topR_speed, 0, 180);
  bottomL_speed = constrain(bottomL_speed, 0, 180);
  bottomR_speed = constrain(bottomR_speed, 0, 180);
}

// Reset all motor speeds and PID controllers
void disarmMotors() {
  topL_speed = 0;
  topR_speed = 0;
  bottomL_speed = 0;
  bottomR_speed = 0;
  
  resetPID(&rollPID);
  resetPID(&pitchPID);
  resetPID(&yawPID);
}

// Write calculated speeds to all ESCs
void writeMotorSpeeds() {
  ledcWrite(TOPL_CHANNEL, map(topL_speed, 0, 180, 3276, 6553));
  ledcWrite(TOPR_CHANNEL, map(topR_speed, 0, 180, 3276, 6553));
  ledcWrite(BOTTOML_CHANNEL, map(bottomL_speed, 0, 180, 3276, 6553));
  ledcWrite(BOTTOMR_CHANNEL, map(bottomR_speed, 0, 180, 3276, 6553));
}

// Print debug information periodically
void printDebugInfo(unsigned long now) {
  if (now - lastPrint > 200) {
    lastPrint = now;
    
    // Detailed diagnostics
    Serial.print("Base: "); Serial.print(base_motor_speed);
    Serial.print(" | R:"); Serial.print(mpu.getAngleX(), 1);
    Serial.print(" P:"); Serial.print(mpu.getAngleY(), 1);
    Serial.print(" YawRate:"); Serial.print(mpu.getGyroZ(), 1);
    Serial.print(" | tR:"); Serial.print(targetRoll, 1);
    Serial.print(" tP:"); Serial.print(targetPitch, 1);
    Serial.print(" tYR:"); Serial.print(targetYawRate, 1);
    Serial.print(" | Motors: ");
    Serial.print(topL_speed); Serial.print(",");
    Serial.print(topR_speed); Serial.print(",");
    Serial.print(bottomL_speed); Serial.print(",");
    Serial.println(bottomR_speed);
  }
}

// Print joystick values for diagnostics
void printJoystickDebug(unsigned long now) {
  static unsigned long lastJoyPrint = 0;
  if (now - lastJoyPrint > 1000) {
    lastJoyPrint = now;
    Serial.print("JoyL: "); Serial.print(Data.joystickL.x);
    Serial.print(","); Serial.print(Data.joystickL.y);
    Serial.print(" | JoyR: "); Serial.print(Data.joystickR.x);
    Serial.print(","); Serial.println(Data.joystickR.y);
  }
}



void loop()
{
  unsigned long now = millis();
  
  // Update MPU6050 data
  mpu.update();
  
  if (radio.available())
  {
    readData();
    processJoystickInput();
    printJoystickDebug(now);

    // Only apply PID when throttle is above minimum
    if (base_motor_speed > 40)
    {
      float rollCorrection, pitchCorrection, yawCorrection;
      computePIDCorrections(rollCorrection, pitchCorrection, yawCorrection);
      calculateMotorSpeeds(rollCorrection, pitchCorrection, yawCorrection);
    }
    else
    {
      disarmMotors();
    }

    writeMotorSpeeds();
    printDebugInfo(now);
    radio.writeAckPayload(1, &power, sizeof(power));
  }
}