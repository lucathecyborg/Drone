#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Arduino.h>
#include <MPU6050_light.h>

// Pin definitions for e01-mlodp5 antenna
#define CE_PIN 4
#define CSN_PIN 5

#define MISO_PIN 19
#define MOSI_PIN 23
#define SCK_PIN 18

RF24 radio(CE_PIN, CSN_PIN);
const byte address[6] = "NODE1";

#pragma pack(push, 1)
struct message
{
  uint16_t leftX;
  uint16_t leftY;
  bool leftButton;

  uint16_t rightX;
  uint16_t rightY;
  bool rightButton;

  uint16_t throttle;

  uint8_t pidAxis;
  float kp, ki, kd;

  uint8_t flags;
  uint8_t checksum;
};
#pragma pack(pop)

// GLOBAL VARIABLES
message rxData;
uint16_t txBattery;
int lastPrintTime = 201;
char space = ' ';

// Communication stats
struct CommStats
{
  uint32_t packetsReceived = 0;
  uint32_t checksumErrors = 0;
  unsigned long lastReceived = 0;
  unsigned long maxGap = 0;
} commStats;

bool initRadio()
{

  if (!radio.begin(&SPI, CE_PIN, CSN_PIN))
  {
    return false;
  }

  radio.setPALevel(RF24_PA_LOW);
  radio.setDataRate(RF24_1MBPS);
  radio.setChannel(108);
  radio.setAutoAck(true);
  radio.setRetries(3, 5);
  radio.openReadingPipe(1, address);
  radio.startListening();

  return true;
}

void setup()
{
  Serial.begin(115200);
  delay(100);

  Serial.println("Initing SPI");
  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, CSN_PIN);
  delay(100);
  pinMode(CSN_PIN, OUTPUT);
  digitalWrite(CSN_PIN, HIGH);
  delay(10);

  if (!initRadio())
  {
    Serial.println("Radio failed to init");
    while (1)
    {
      delay(1000);
    }
  }
}

uint8_t calculateChecksum(const message *msg)
{
  uint8_t sum = 0;
  const uint8_t *data = (const uint8_t *)msg;

  for (size_t i = 0; i < sizeof(message) - 1; i++)
  {
    sum += data[i];
  }
  return sum;
}

bool validateChecksum(const message *msg)
{

  uint8_t calculated = calculateChecksum(msg);
  return (calculated == msg->checksum);
}

bool recieveData()
{

  radio.read(&rxData, sizeof(rxData));

  if (!validateChecksum(&rxData))
  {
    commStats.checksumErrors++;
    return false;
  }

  radio.writeAckPayload(1, &txBattery, sizeof(txBattery));
  return true;
}

void updateCommStats(bool recieved)
{
  if (recieved)
  {
    unsigned long now = millis();

    if (commStats.packetsReceived > 0)
    {
      unsigned long gap = now - commStats.lastReceived;
      if (gap > commStats.maxGap)
      {
        commStats.maxGap = gap;
      }
    }

    commStats.packetsReceived++;
    commStats.lastReceived = now;
  }
}

void printCommStats()
{
  unsigned long timeSinceLastRx = millis() - commStats.lastReceived;

  Serial.println("\n--- Communication Statistics ---");
  Serial.print("Packets Received: ");
  Serial.println(commStats.packetsReceived);
  Serial.print("Checksum Errors: ");
  Serial.println(commStats.checksumErrors);
  Serial.print("Max Gap: ");
  Serial.print(commStats.maxGap);
  Serial.println("ms");
  Serial.print("Last received: ");
  Serial.print(timeSinceLastRx);
  Serial.println("ms ago");

  if (commStats.packetsReceived > 0)
  {
    Serial.println("Status: CONNECTED");
  }
  else
  {
    Serial.println("Status: WAITING...");
  }
  Serial.println("--------------------------------\n");
}

void printRecievedData()
{
  Serial.print("throttle: ");
  Serial.println(rxData.throttle);
  Serial.print("leftX: ");
  Serial.println(rxData.leftX);
  Serial.print("leftY: ");
  Serial.println(rxData.leftY);
  Serial.print("leftButton: ");
  Serial.println(rxData.leftButton);
  Serial.print("rightX: ");
  Serial.println(rxData.rightX);
  Serial.print("rightY: ");
  Serial.println(rxData.rightY);
  Serial.print("rightButton: ");
  Serial.println(rxData.rightButton);
  Serial.print("Pid axis: ");
  Serial.println(rxData.pidAxis);
  Serial.print("Ki, Kp, Kd: ");
  Serial.println(rxData.ki + space + rxData.kp + space + rxData.kd);
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