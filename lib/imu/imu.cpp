#include "imu.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>



//Constructor: Initializes the Adafruit_BNO055 object and checks if it was successful
BNO055_IMU :: BNO055_IMU() : imu_(Adafruit_BNO055()) {
}

void BNO055_IMU :: startIMU() {
    imu_.begin();
    //set the external crystal
    imu_.setExtCrystalUse(true);
}

// Public member function that returns the accelerometer vector from the IMU object
imu::Vector<3> BNO055_IMU :: getAcceleration() {
    return imu_.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
}

// Public member function that returns the gyroscope vector from the IMU object
imu::Vector<3> BNO055_IMU :: getGyroscope() {
    return imu_.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
}

// Public member function that returns the magnetometer vector from the IMU object
imu::Vector<3> BNO055_IMU :: getMagnetometer() {
    return imu_.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
}

// Public member function that calculates and returns the pitch angle from the accelerometer vector
float BNO055_IMU :: getPitch() {
    imu::Vector<3> acc = getAcceleration();
    return atan2(acc.x()/GRAVITY, acc.z()/GRAVITY) * RAD_TO_DEG;
}

// Public member function that calculates and returns the roll angle from the accelerometer vector
float BNO055_IMU :: getRoll() {
    imu::Vector<3> acc = getAcceleration();
    return atan2(acc.y()/GRAVITY, acc.z()/GRAVITY) * RAD_TO_DEG;
}

void BNO055_IMU :: toogleLed() {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
}