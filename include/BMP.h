#pragma once
#include <Adafruit_BMP280.h>

class BMP
{
private:
    float temp;
    float pressure;
    float altitude;
    Adafruit_BMP280 bmp;
    float groundAltitude;
    bool calibrated;

public:
    bool initBMP();
    void updateBmpData();
    float getTemp();
    float getPressure();
    float getAltitude();
    void calibrateAltitude();
    float getRelativeAltitude();
};
