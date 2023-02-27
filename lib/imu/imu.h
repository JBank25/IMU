#pragma once
#include <Adafruit_BNO055.h>
#include "MadgwickAHRS.h"

#define GRAVITY 9.80665
#define ACCEL_CALIBRATION_PIN 2
#define GYRO_CALIBRATION_PIN 3
#define MAG_CALIBRATION_PIN 4
#define SYSTEM_CALIBRATION_PIN 5
#define IMU_UPDATE_PIN 6


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
    void updateMadgwick();
    float getIMUHeading();
    float getIMURoll();
    float getIMUPitch();
    void getMadgwickOrientation(float * dataBuffer);
    void calibrate();
    float getPitch();
    float getRoll();

private:
    Adafruit_BNO055 imu_;
    Madgwick madgwick_;
};