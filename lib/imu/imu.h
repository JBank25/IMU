#pragma once
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define GRAVITY 9.80665

// Declare the class
class BNO055_IMU {
public:
    // Constructor
    BNO055_IMU();

    // Public member functions
    imu::Vector<3> getAcceleration();
    imu::Vector<3> getGyroscope();
    imu::Vector<3> getMagnetometer();
    void startIMU();
    //delere this method, only for testing
    void toogleLed();
    float getPitch();
    float getRoll();

private:
    Adafruit_BNO055 imu_;
};

//need def for toogleLed()
void toogleLed();