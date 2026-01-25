
#include <Arduino.h>

#include "Communication.h"
#include "IMU.h"
#include "BMP.h"
#include "ADS1115.h"

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

int initMotors()
{
  Serial.println("initin motors...");
  // Setup PWM for all ESCs
  if (ledcSetup(TOPL_CHANNEL, PWM_FREQ, PWM_RESOLUTION) == 0)
  {
    Serial.println("Failed to setup TOPL channel");
    return -1;
  }
  ledcAttachPin(TOPL_PIN, TOPL_CHANNEL);

  if (ledcSetup(TOPR_CHANNEL, PWM_FREQ, PWM_RESOLUTION) == 0)
  {
    Serial.println("Failed to setup TOPR channel");
    return -1;
  }
  ledcAttachPin(TOPR_PIN, TOPR_CHANNEL);

  if (ledcSetup(BOTTOML_CHANNEL, PWM_FREQ, PWM_RESOLUTION) == 0)
  {
    Serial.println("Failed to setup BOTTOML channel");
    return -1;
  }
  ledcAttachPin(BOTTOML_PIN, BOTTOML_CHANNEL);

  if (ledcSetup(BOTTOMR_CHANNEL, PWM_FREQ, PWM_RESOLUTION) == 0)
  {
    Serial.println("Failed to setup BOTTOMR channel");
    return -1;
  }
  ledcAttachPin(BOTTOMR_PIN, BOTTOMR_CHANNEL);

  Serial.println("All motor channels initialized");
  return 0;
}

void armMotors()
{
  int armDuty = map(1000, 1000, 2000, 3276, 6553);

  ledcWrite(TOPL_CHANNEL, armDuty);
  ledcWrite(TOPR_CHANNEL, armDuty);
  ledcWrite(BOTTOML_CHANNEL, armDuty);
  ledcWrite(BOTTOMR_CHANNEL, armDuty);

  Serial.println("Sending arming signal to ESCs...");
  delay(3000);
  Serial.println("ESCs armed.");
}

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
  Serial.println("Initing SPI");
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
  if (initMotors())
  {
    armMotors();
  }
  else
  {
    infiniteLoop();
  }
}

void loop()
{

  if (radio.available())
  {
    bool success = recieveData();
    updateCommStats(success);

    unsigned long now = millis();
    unsigned long timeSinceLastRx = now - commStats.lastReceived;

    if (commStats.packetsReceived > 0 && timeSinceLastRx > 200)
    {
      // No data for 200ms
      // Add motor failsafe here
    }

    if (success)
    {
      if (now - lastPrintTime >= 200)
      {
        lastPrintTime = now;
        printRecievedData();
      }
    }
  }
}