#include <Arduino.h>
#include "imu.h"
#include "ble.h"
#include "gps.h"

BNO055_IMU myIMU = BNO055_IMU();
BLE_Device myBLE = BLE_Device();
GT_U7_GPS myGPS = GT_U7_GPS();

//Code for testing BLE rate at which it can change values
unsigned long currentMillis = 0;
unsigned long previousMillis = 0;
unsigned int variable = 0;
const int interval = 1000 / 10; // 60Hz = 1000ms / 60 = 16.67ms
int num_iterations = 0;

// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT); 
  //set LED_BUILTIN to HIGH
  digitalWrite(LED_BUILTIN, HIGH);
  //set the external crystal
  myIMU.startIMU();
  myGPS.startGPS();
  myBLE.startBLE();
}

// the loop function runs over and over again forever
void loop() {
  //create var for number of iterations imu has been read
  int imu_update_count = 0;
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);
  //get BLE connection
  myBLE.getConnection();
  delay(1000);

  digitalWrite(LED_BUILTIN, HIGH);
  int ledState = 0;
  bool connected = myBLE.isConnected();
  delay(2000);
  while(connected)
  {
    currentMillis = millis();
    if (currentMillis - previousMillis >= interval && num_iterations < 300) 
    {
      digitalWrite(LED_BUILTIN, ledState);
      ledState = !ledState;
      num_iterations += 1;
      previousMillis = currentMillis;
      //get 9 axis readings
      float dataBuffer[9] = {0.};
      myIMU.get9AxisReadings(dataBuffer);
      //send 9 axis readings
      myBLE.updateIMUData(dataBuffer);
      imu_update_count += 1;
    }
    if(imu_update_count == 10)
    {
      float gpsData[4] = {0.};
      myGPS.getGPSData(gpsData);
      myBLE.updateGPSData(gpsData);
      imu_update_count = 0;
    }
    connected = myBLE.isConnected();
  }
}