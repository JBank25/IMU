#include <Arduino.h>
#include "imu/imu.h"

BNO055_IMU myImu = BNO055_IMU();

void setup() 
{
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  Serial.println("Starting");
}
void loop() 
{
  //show what we are debugging
  Serial.println("Accelerometer");
  //get the accelerometer vector
  imu::Vector<3> acc = myImu.getAcceleration();
  //print the x, y, and z components of the vector
  Serial.print("X: ");
  Serial.print(acc.x());
  Serial.print(" Y: ");
  Serial.print(acc.y());
  Serial.print(" Z: ");
  Serial.println(acc.z());

}