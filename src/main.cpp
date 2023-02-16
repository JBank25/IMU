#include <Arduino.h>
#include "imu.h"

BNO055_IMU myIMU = BNO055_IMU();

// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT); 
  //set the external crystal
  myIMU.startIMU();
}

// the loop function runs over and over again forever
void loop() {
  //get accelerometer readings
  imu::Vector<3> acc = myIMU.getAcceleration();
  myIMU.toogleLed();
}