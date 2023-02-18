#include <Arduino.h>
#include "imu.h"
#include "ble.h"

BNO055_IMU myIMU = BNO055_IMU();
BLE_Device myBLE = BLE_Device();

//Code for testing BLE rate at which it can change values
unsigned long currentMillis = 0;
unsigned long previousMillis = 0;
unsigned int variable = 0;
const int interval = 1000 / 1; // 60Hz = 1000ms / 60 = 16.67ms
int num_iterations = 0;

// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT); 
  //set LED_BUILTIN to HIGH
  digitalWrite(LED_BUILTIN, HIGH);
  //set the external crystal
  myIMU.startIMU();
  myBLE.startBLE();
}

// the loop function runs over and over again forever
void loop() {
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);
  //get BLE connection
  myBLE.getConnection();
  delay(1000);

  digitalWrite(LED_BUILTIN, HIGH);
  int ledState = 0;
  bool connected = myBLE.isConnected();
  while(connected)
  {
    currentMillis = millis();
    if (currentMillis - previousMillis >= interval && num_iterations < 300) 
    {
      digitalWrite(LED_BUILTIN, ledState);
      ledState = !ledState;
      num_iterations += 1;
      previousMillis = currentMillis;
      float dataBuffer[] = {1., 2., 3., 4., 5., 6., 7., 8., 9.};
      myBLE.sendFloatValues(dataBuffer);
    }
    connected = myBLE.isConnected();
  }
}