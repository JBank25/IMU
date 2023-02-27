#include "imu.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <cstring>
#include <stdint.h>
#include "MadgwickAHRS.h"

/**
 * @brief Construct a new BNO055_IMU::BNO055_IMU object, initializes the imu_ and madgwick_ objects.
 * 
 * @return BNO055_IMU
 */
BNO055_IMU :: BNO055_IMU() : imu_(Adafruit_BNO055()), madgwick_(Madgwick()) {
}
/**
 * @brief Public member function that initializes the IMU object and sets the external crystal.
 * 
 */
void BNO055_IMU :: startIMU() {
    imu_.begin();
    imu_.setExtCrystalUse(true);        //set the external crystal
    madgwick_.begin(104.);      
}

/**
 * @brief Public member function that returns the accelerometer vector from the IMU object
 * 
 * @return imu::Vector<3> 
 */
imu::Vector<3> BNO055_IMU :: getAcceleration() {
    return imu_.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
}
/**
 * @brief Public member function that returns the gyroscope vector from the IMU object
 * 
 * @return imu::Vector<3>
 */
imu::Vector<3> BNO055_IMU :: getGyroscope() {
    return imu_.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
}

/**
 * @brief Public member function that returns the magnetometer vector from the IMU object
 * 
 * @return imu::Vector<3>
 */
imu::Vector<3> BNO055_IMU :: getMagnetometer() {
    return imu_.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
}

/**
 * @brief Public member function that gets the 9 axis readings from the IMU object and copies them into the dataBuffer
 * 
 * @param dataBuffer 
 */
void BNO055_IMU :: get9AxisReadings(float * dataBuffer){
    imu::Vector<3> acc = getAcceleration();
    imu::Vector<3> gyro = getGyroscope();
    imu::Vector<3> mag = getMagnetometer();
    for(int i = 0; i < 3; i++) 
    {
        dataBuffer[i] = acc[i];
        dataBuffer[i+3] = gyro[i];
        dataBuffer[i+6] = mag[i];
    }
}

/**
 * @brief 
 * 
 */
void BNO055_IMU :: calibrate(){
    uint8_t system_calibration = 0;
    uint8_t gyro_calibration = 0;
    uint8_t accel_calibration = 0;
    uint8_t mag_calibration = 0;
    while(system_calibration != 3)
    {
        imu_.getCalibration(&system_calibration, &gyro_calibration, &accel_calibration, &mag_calibration);
        digitalWrite(ACCEL_CALIBRATION_PIN, (accel_calibration == 3));
        digitalWrite(GYRO_CALIBRATION_PIN, (gyro_calibration == 3));
        digitalWrite(MAG_CALIBRATION_PIN, (mag_calibration == 3));
        digitalWrite(SYSTEM_CALIBRATION_PIN, (system_calibration == 3));
    }
}
/**
 * @brief Public member function that updates the Madgwick filter with the latest IMU readings
 * 
 */
void BNO055_IMU :: updateMadgwick(){
    imu::Vector<3> acc = getAcceleration();
    imu::Vector<3> gyro = getGyroscope();
    madgwick_.updateIMU(gyro[0], gyro[1], gyro[2], acc[0], acc[1], acc[2]);
}
/**
 * @brief Public member function that returns the heading from the Madgwick filter
 * heading is the angle aroud the z axis
 * 
 * @return float 
 */
float BNO055_IMU :: getIMUHeading(){
    updateMadgwick();
    return madgwick_.getYaw();
}
/**
 * @brief Public member function that returns the roll from the Madgwick filter
 * roll is the angle around the x axis
 * 
 * @return float 
 */
float BNO055_IMU :: getIMURoll(){
    updateMadgwick();
    return madgwick_.getRoll();
}
/**
 * @brief Public member function that returns the pitch from the Madgwick filter
 * pitch is the angle around the y axis
 * 
 * @return float 
 */
float BNO055_IMU :: getIMUPitch(){
    updateMadgwick();
    return madgwick_.getPitch();
}
/**
 * @brief Gets orientation from the Madgwick filter and copies it into the dataBuffer
 * 
 * @param dataBuffer 
 */
void BNO055_IMU :: getMadgwickOrientation(float * dataBuffer){
    updateMadgwick();
    dataBuffer[0] = madgwick_.getRoll();
    dataBuffer[1] = madgwick_.getPitch();
    dataBuffer[2] = madgwick_.getYaw();
}

//TWO DEPRECATED FUNCTIONS, DO NOT USE
float BNO055_IMU :: getPitch() {
    imu::Vector<3> acc = getAcceleration();
    return atan2(acc.x()/GRAVITY, acc.z()/GRAVITY) * RAD_TO_DEG;
}
float BNO055_IMU :: getRoll() {
    imu::Vector<3> acc = getAcceleration();
    return atan2(acc.y()/GRAVITY, acc.z()/GRAVITY) * RAD_TO_DEG;
}