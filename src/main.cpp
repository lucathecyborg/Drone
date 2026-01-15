
#include <Arduino.h>

#include "Communication.h"
#include "IMU.h"

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
    while (1)
    {
      delay(1000);
    }
  }

  if (!initICM())
  {
    Serial.println("IMU not found, aborting.");
    while (1)
    {
      delay(100);
    }
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