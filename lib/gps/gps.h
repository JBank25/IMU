#pragma once
#include <TinyGPS++.h>

class GT_U7_GPS {
public:
    GT_U7_GPS();

    void startGPS();
    void getGPSData(float * dataBuffer);

private:
    TinyGPSPlus gps_;
};