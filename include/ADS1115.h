#pragma once
#include <Adafruit_ADS1X15.h>

struct BatteryData
{
    float cell1;
    float fullVoltage;
    float percentage;
    bool isCritical;
    bool isHealthy;
};

class ADS1115
{
private:
    Adafruit_ADS1115 ads;
    const float VOLTAGE_DIVIDER_RATIO = 4.03; // change to actual ratio later (acutal_ratio= multimeter_reading/ads_reading) (full voltage)
    const float CELL_VOLTAGE_FULL = 4.2;
    const float CELL_VOLTAGE_EMPTY = 3.3;
    const float CELL_VOLTAGE_CRITICAL = 3.5;
    BatteryData battery1;
    BatteryData battery2;

public:
    bool initADS();
    void updateBatteryVoltages();
    float calculateBatteryPercent(float voltage);
    bool checkBatteryHealth(BatteryData *bat);
    float getBattery1Voltage();
    float getBattery2Voltage();
    float getBattery1Percent();
    float getBattery2Percent();
    bool isBattery1Critical();
    bool isBattery2Critical();
};

ADS1115 ads;