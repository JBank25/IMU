#pragma once
#include <ArduinoBLE.h>


class BLE_Device {
public:
    BLE_Device();
    void startBLE();
    void getConnection();
    void updateIMUData(float dataBuffer[10]);
    void updateGPSData(float dataBuffer[4]);
    bool isConnected();
private:
    char deviceName[255];
    char localName[255];
    char serviceUUID[255];
    char imuCharacteristicUUID[255];
    char gpsCharacteristicUUID[255];
    BLEService service;
    BLECharacteristic imuFloatValueCharacteristic;
    BLECharacteristic gpsFloatValueCharacteristic;
    BLEDevice central;
    float imuPacketCount;
    float gpsPacketCount;
};