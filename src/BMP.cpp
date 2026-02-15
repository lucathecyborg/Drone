#include <BMP.h>

BMP bmp;

bool BMP::initBMP()
{
    if (!bmp.begin(0x76))
    {
        return false;
    }

    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     // Operating mode
                    Adafruit_BMP280::SAMPLING_X2,     // Temp oversampling
                    Adafruit_BMP280::SAMPLING_X16,    // Pressure oversampling
                    Adafruit_BMP280::FILTER_X16,      // Filtering
                    Adafruit_BMP280::STANDBY_MS_500); // Standby time

    return true;
}

void BMP::updateBmpData()
{
    temp = bmp.readTemperature();
    pressure = bmp.readPressure();
    altitude = bmp.readAltitude(1013.25);
}

float BMP::getAltitude()
{
    return altitude;
}

float BMP::getPressure()
{
    return pressure;
}

float BMP::getTemp()
{
    return temp;
}

void BMP::calibrateAltitude()
{
    groundAltitude = bmp.readAltitude(1013.25);
    calibrated = true;
}

float BMP::getRelativeAltitude()
{
    if (!calibrated)
        return 0;
    return bmp.readAltitude(1013.25) - groundAltitude;
}