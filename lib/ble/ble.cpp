#include "ble.h"
#include <ArduinoBLE.h>

//Contructor
BLE_Device::BLE_Device() : deviceName("Arduino Nano 33 BLE"), localName("Arduino Nano 33 BLE"),
    serviceUUID("9A48ECBA-2E92-082F-C079-9E75AAE428B1"), characteristicUUID("C8F88594-2217-0CA6-8F06-A4270B675D69"),
    service(serviceUUID), floatValueCharacteristic(characteristicUUID, BLERead | BLENotify, 10),
    packetCount(0.)
    {}
    void BLE_Device :: startBLE()
    {
        BLE.setLocalName("Arduino Nano 33 BLE");
        BLE.setDeviceName("Arduino Nano 33 BLE");
        BLE.addService(service);
        service.addCharacteristic(floatValueCharacteristic);
        BLE.addService(service);
        BLE.advertise();
    }
    //need a connection
    void BLE_Device :: getConnection()
    {
        digitalWrite(LED_BUILTIN, HIGH);
        central = BLE.central();
        while(!central)
        {
            digitalWrite(LED_BUILTIN, LOW);
            central = BLE.central();
            delay(1000);
            digitalWrite(LED_BUILTIN, HIGH);
        }
        delay(1000);
        digitalWrite(LED_BUILTIN, LOW);
    }
    void BLE_Device :: sendFloatValues(float * dataBuffer)
    {
        //need to create a buffer that include packetCount first
        float * transmitBuf = new float[11];
        transmitBuf[0] = packetCount;
        for(int i = 0; i < 10; i++)
        {
            transmitBuf[i+1] = dataBuffer[i];
        }
        floatValueCharacteristic.writeValue(dataBuffer, 10);
        packetCount+=1;
    }