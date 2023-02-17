#pragma once
#include <ArduinoBLE.h>


class BLE_Device {
public:
    BLE_Device();
    void startBLE();
    void getConnection();
    void sendFloatValues(float* dataBuffer);
private:
    const char * deviceName;
    const char * localName;
    const char * serviceUUID;
    const char * characteristicUUID;
    BLEService service;
    BLECharacteristic floatValueCharacteristic;
    BLEDevice central;
    float packetCount;
};