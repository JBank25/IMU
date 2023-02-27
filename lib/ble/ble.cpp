#include "ble.h"
#include <ArduinoBLE.h>

#define IMU_PACKET_NUM_FLOATS 13
#define GPS_PACKET_NUM_FLOATS 5

//Contructor
BLE_Device::BLE_Device() : deviceName("Arduino Nano 33 BLE"), localName("Arduino Nano 33 BLE"),
    serviceUUID("9A48ECBA-2E92-082F-C079-9E75AAE428B1"), imuCharacteristicUUID("C8F88594-2217-0CA6-8F06-A4270B675D69"),
    gpsCharacteristicUUID("C8F88594-2217-0CA6-8F05-A4270B675D69"),service(BLEService(serviceUUID)), imuFloatValueCharacteristic(BLECharacteristic(imuCharacteristicUUID, BLERead | BLENotify, IMU_PACKET_NUM_FLOATS*sizeof(float))),
    gpsFloatValueCharacteristic(BLECharacteristic(gpsCharacteristicUUID, BLERead | BLENotify, GPS_PACKET_NUM_FLOATS*sizeof(float))), imuPacketCount(0.), gpsPacketCount(0.)
    {}
void BLE_Device :: startBLE()
{
    BLE.begin();
    BLE.setLocalName(localName);
    BLE.setDeviceName(deviceName);
    BLE.setAdvertisedService(service);
    service.addCharacteristic(imuFloatValueCharacteristic);
    service.addCharacteristic(gpsFloatValueCharacteristic);
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
void BLE_Device :: updateIMUData(float dataBuffer[12])
{
    float transmitBuf[13] = {0.};
    transmitBuf[0] = imuPacketCount;
    for (int i = 0; i < 12; i++)
    {
        transmitBuf[i+1] = dataBuffer[i];
    }
    imuFloatValueCharacteristic.writeValue((byte*)transmitBuf, sizeof(transmitBuf));
    imuPacketCount += 1.;
}

void BLE_Device :: updateGPSData(float dataBuffer[4])
{
    float transmitBuf[5] = {0.};
    transmitBuf[0] = gpsPacketCount;
    for (int i = 0; i < 4; i++)
    {
        transmitBuf[i+1] = dataBuffer[i];
    }
    gpsFloatValueCharacteristic.writeValue((byte*)transmitBuf, sizeof(transmitBuf));
    gpsPacketCount += 1.;
}

bool BLE_Device :: isConnected()
{
    return central.connected();
}