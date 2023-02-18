#pragma once
#include <ArduinoBLE.h>


class BLE_Device {
public:
    BLE_Device();
    void startBLE();
    void getConnection();
    void sendFloatValues(float dataBuffer[10]);
    bool isConnected();
private:
    char deviceName[255];
    char localName[255];
    char serviceUUID[255];
    char characteristicUUID[255];
    BLEService service;
    BLECharacteristic floatValueCharacteristic;
    BLEDevice central;
    float packetCount;
};