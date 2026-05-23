#pragma once
#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <Arduino.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define CE_PIN 4
#define CSN_PIN 5

#define MISO_PIN 19
#define MOSI_PIN 23
#define SCK_PIN 18
extern RF24 radio;
extern const byte address[6];

// Struct declarations
#pragma pack(push, 1)
struct message
{
    uint16_t leftX;
    uint16_t leftY;

    uint16_t rightX;
    uint16_t rightY;

    uint16_t throttle;

    uint8_t pidAxis;
    float kp, ki, kd;

    uint8_t flags;
    uint8_t checksum;
};
#pragma pack(pop)

extern message rxData;
extern uint16_t txBattery;
extern int lastPrintTime;
extern char space;

struct CommStats
{
    uint32_t packetsReceived;
    uint32_t checksumErrors;
    unsigned long lastReceived;
    unsigned long maxGap;
};
extern CommStats commStats;

// Forward declarations
bool initRadio();
uint8_t calculateChecksum(const message *msg);
bool validateChecksum(const message *msg);
bool recieveData(float batteryPercent);
void updateCommStats(bool recieved);
void printCommStats();
void printRecievedData();

#endif
