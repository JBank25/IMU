#include <Arduino.h>
#include "imu.h"
#include "ble.h"
#include "gps.h"

//All our global objects
BNO055_IMU myIMU = BNO055_IMU();  //IMU object, 9-axis readings, orientation calc., etc
BLE_Device myBLE = BLE_Device();  //BLE object, handles BLE connection, functions for IMU and GPS transfers
GT_U7_GPS myGPS = GT_U7_GPS();    //currently does nothing, but can give us readings

//Code for testing BLE rate at which it can change values
unsigned long currentMillis = 0;  //current time
unsigned long previousMillisIMU = 0; //previous time, update this
unsigned long previousMillisGPS = 0; //previous time, update this
const int imuInterval = 1000 / 100;  //100Hz
const int gpsInterval = 1000 / 1;  //1Hz
int numIterations = 0;

// the setup function runs once when you press reset or power the board
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);       // LED_BUILTIN used for debugging BLE connectino
  pinMode(ACCEL_CALIBRATION_PIN, OUTPUT); //calibration LED's used to show when full calibration is complete
  pinMode(GYRO_CALIBRATION_PIN, OUTPUT);
  pinMode(MAG_CALIBRATION_PIN, OUTPUT);
  pinMode(SYSTEM_CALIBRATION_PIN, OUTPUT);
  pinMode(IMU_UPDATE_PIN, OUTPUT);        //IMU and GPS update LED's used to show when data is being sent
  pinMode(GPS_UPDATE_PIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);      //LED initially high
  myIMU.startIMU();
  myGPS.startGPS();
  myBLE.startBLE();
}

void loop() {
  // myIMU.calibrate();   //calibrate IMU, not really needed for testing purposes
  //create var for number of iterations imu has been read
  delay(1000);      //probably unnecessary, think it can be deleted
  digitalWrite(LED_BUILTIN, LOW);
  myBLE.getConnection();      //get BLE connection
  digitalWrite(LED_BUILTIN, HIGH);    //BLE LED high when connected
  bool connected = myBLE.isConnected();
  delay(3000);          //delay for 3 seconds to allow for BLE connection to be established in client
  while(connected)
  {
    currentMillis = millis();
    //numIterations is being used to testing to be easier, not necessary. Limits number of packets we send
    if (currentMillis - previousMillisIMU >= imuInterval && numIterations < 3000) 
    {
      float dataBuffer[13] = {0.};  //buffer for 9 axis ordinal ID (1), IMU readings (9), orientation (3)
      myIMU.get9AxisReadings(dataBuffer);         //writes 9 axis readings to dataBuffer
      myIMU.getMadgwickOrientation(dataBuffer + 9); //writes orientation to dataBuffer
      myBLE.updateIMUData(dataBuffer);            //sends dataBuffer for IMU to BLE
      //toggle IMU state, must be slower than rate this loop runs at (100Hz) else would appear constantly high
      if(numIterations % 50 == 0)
      {
        digitalWrite(IMU_UPDATE_PIN, digitalRead(IMU_UPDATE_PIN) ^ 1);        //toggle IMU state
      }
      numIterations += 1;
      previousMillisIMU = currentMillis;
    }
    //update GPS data every 1 second (1Hz)
    if(currentMillis - previousMillisGPS >= gpsInterval)
    {
      float gpsData[4] = {0.};  //buffer for GPS 
      myGPS.getGPSData(gpsData);  //latitude (1), longitude (1), mph (1), altitude (1), NOT SURE IF THIS IS WHAT WE NEED YET
      myBLE.updateGPSData(gpsData); //sends gpsData to BLE
      previousMillisGPS = currentMillis;
      digitalWrite(GPS_UPDATE_PIN, digitalRead(GPS_UPDATE_PIN) ^ 1);        //toggle GPS state
    }
    connected = myBLE.isConnected();
  }
}