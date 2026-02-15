#include "ADS1115.h"

ADS1115 ads;

bool ADS1115::initADS()
{
    if (!ads.begin())
    {
        return false;
    }

    ads.setGain(GAIN_ONE);
    return true;
}

void ADS1115::updateBatteryVoltages()
{

    int16_t adc0 = ads.readADC_SingleEnded(0); // Battery 1 - Cell 1
    int16_t adc1 = ads.readADC_SingleEnded(1); // Battery 1 - Full
    int16_t adc2 = ads.readADC_SingleEnded(2); // Battery 2 - Cell 1
    int16_t adc3 = ads.readADC_SingleEnded(3); // Battery 2 - Full

    // Convert to actual voltages
    battery1.cell1 = ads.computeVolts(adc0) * VOLTAGE_DIVIDER_RATIO;
    battery1.fullVoltage = ads.computeVolts(adc1) * VOLTAGE_DIVIDER_RATIO;
    battery2.cell1 = ads.computeVolts(adc2) * VOLTAGE_DIVIDER_RATIO;
    battery2.fullVoltage = ads.computeVolts(adc3) * VOLTAGE_DIVIDER_RATIO;

    // Battery %
    battery1.percentage = calculateBatteryPercent(battery1.fullVoltage);
    battery2.percentage = calculateBatteryPercent(battery2.fullVoltage);

    // Check health
    battery1.isHealthy = checkBatteryHealth(&battery1);
    battery1.isCritical = (battery1.cell1 < CELL_VOLTAGE_CRITICAL);

    battery2.isHealthy = checkBatteryHealth(&battery2);
    battery2.isCritical = (battery2.cell1 < CELL_VOLTAGE_CRITICAL);
}

float ADS1115::calculateBatteryPercent(float voltage)
{
    const float V_MIN = 9.9;
    const float V_MAX = 12.6;

    float percent = (voltage - V_MIN) / (V_MAX - V_MIN) * 100.0;
    return constrain(percent, 0, 100);
}

bool ADS1115::checkBatteryHealth(BatteryData *batt)
{
    // Over-voltage check
    if (batt->cell1 > 4.3)
        return false;

    // Under-voltage check
    if (batt->cell1 < 3.0)
        return false;

    return true;
}

float ADS1115::getBattery1Voltage() { return battery1.fullVoltage; }
float ADS1115::getBattery2Voltage() { return battery2.fullVoltage; }
float ADS1115::getBattery1Percent() { return battery1.percentage; }
float ADS1115::getBattery2Percent() { return battery2.percentage; }
bool ADS1115::isBattery1Critical() { return battery1.isCritical; }
bool ADS1115::isBattery2Critical() { return battery2.isCritical; }
