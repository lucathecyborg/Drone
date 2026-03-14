#include "Communication.h"

// Define global variables declared in Communication.h
RF24 radio(CE_PIN, CSN_PIN);
const byte address[6] = "NODE1";
message rxData;
uint16_t txBattery;
int lastPrintTime = 201;
char space = ' ';
CommStats commStats = {0, 0, 0, 0};

bool initRadio()
{
    if (!radio.begin())
    {
        return false;
    }

    radio.setPALevel(RF24_PA_LOW);
    radio.setDataRate(RF24_250KBPS);
    radio.setChannel(125);
    radio.enableAckPayload();
    radio.openReadingPipe(1, address);
    radio.startListening();

    return true;
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
    Serial.print("rightX: ");
    Serial.println(rxData.rightX);
    Serial.print("rightY: ");
    Serial.println(rxData.rightY);
    Serial.print("Pid axis: ");
    Serial.println(rxData.pidAxis);
    Serial.print("Ki, Kp, Kd: ");
    Serial.println(rxData.ki + space + rxData.kp + space + rxData.kd);
}