#pragma once
#include <TinyGPS++.h>

#define GPS_UPDATE_PIN 7

class GT_U7_GPS {
public:
    GT_U7_GPS();

    void startGPS();
    void getGPSData(float * dataBuffer);

private:
    TinyGPSPlus gps_;
};