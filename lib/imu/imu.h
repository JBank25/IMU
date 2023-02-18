#pragma once
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <cstring>

#define GRAVITY 9.80665

// Declare the class
class BNO055_IMU {
public:
    // Constructor
    BNO055_IMU();

    // Public member functions
    void startIMU();
    imu::Vector<3> getAcceleration();
    imu::Vector<3> getGyroscope();
    imu::Vector<3> getMagnetometer();
    void get9AxisReadings(float * dataBuffer);
    void ledHigh();       //delere this method, only for testing
    void ledLow();       //delere this method, only for testing
    float getPitch();
    float getRoll();

private:
    Adafruit_BNO055 imu_;
};