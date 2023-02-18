#include "ble.h"
#include <ArduinoBLE.h>

//Contructor
BLE_Device::BLE_Device() : deviceName("Arduino Nano 33 BLE"), localName("Arduino Nano 33 BLE"),
    serviceUUID("9A48ECBA-2E92-082F-C079-9E75AAE428B1"), characteristicUUID("C8F88594-2217-0CA6-8F06-A4270B675D69"),
    service(BLEService(serviceUUID)), floatValueCharacteristic(BLECharacteristic(characteristicUUID, BLERead | BLENotify, 10*sizeof(float))),
    packetCount(0.)
    {}
void BLE_Device :: startBLE()
{
    BLE.begin();
    BLE.setLocalName(localName);
    BLE.setDeviceName(deviceName);
    BLE.setAdvertisedService(service);
    service.addCharacteristic(floatValueCharacteristic);
    BLE.addService(service);
    BLE.advertise();
}
//need a connection
void BLE_Device :: getConnection()
{
    while(true)
    {
        central = BLE.central();
        if(central)
        {
            return;
        }
    }
}
void BLE_Device :: sendFloatValues(float dataBuffer[9])
{
    static float count = 0.;
    float transmitBuf[10] = {0.};
    transmitBuf[0] = count;
    for (int i = 0; i < 9; i++)
    {
        transmitBuf[i+1] = dataBuffer[i];
    }
    floatValueCharacteristic.writeValue((byte*)transmitBuf, sizeof(transmitBuf));
    count += 1.;
}

bool BLE_Device :: isConnected()
{
    return central.connected();
}