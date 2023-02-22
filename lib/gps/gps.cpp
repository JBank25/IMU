#include "gps.h"
#include <TinyGPS++.h>

GT_U7_GPS :: GT_U7_GPS() : gps_(TinyGPSPlus())
{}

void GT_U7_GPS :: startGPS()
{
    Serial1.begin(9600);
}

void GT_U7_GPS :: getGPSData(float * dataBuffer)
{
    while(Serial1.available() > 0)
    {
        gps_.encode(Serial1.read());
    }
    dataBuffer[0] = gps_.location.lat();
    dataBuffer[1] = gps_.location.lng();
    dataBuffer[2] = gps_.speed.mph();
    dataBuffer[3] = gps_.altitude.feet();
}