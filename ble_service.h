/*
#ifndef BLE_SERVICE_H
#define BLE_SERVICE_H

#include <NimBLEDevice.h> 
#include "config.h"
#include "terminal.h" // For processCommand

// Bluno/DFRobot Service UUIDs
#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E" // For receiving commands from the app (WRITE)
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E" // For sending telemetry to the app (NOTIFY)

class BLEManager; // Forward declaration

// Global instance
extern BLEManager bleManager;

class BLEManager {
public:
    BLEManager() : pServer(nullptr), pTxCharacteristic(nullptr), deviceConnected(false) {}

    void setup() {
        // 1. Generate a unique device name
        uint64_t chipid = ESP.getEfuseMac();
        char deviceName[20];
        snprintf(deviceName, sizeof(deviceName), "Nono-%04X", (uint16_t)(chipid >> 32));

        // 2. Initialize BLE
        NimBLEDevice::init(deviceName);
        pServer = NimBLEDevice::createServer();
        pServer->setCallbacks(new MyServerCallbacks());

        // 3. Create the Service
        NimBLEService *pService = pServer->createService(SERVICE_UUID);

        // 4. Create TX Characteristic (ESP32 -> App)
        pTxCharacteristic = pService->createCharacteristic(
                                CHARACTERISTIC_UUID_TX,
                                NIMBLE_PROPERTY::NOTIFY
                            );
        
        // 5. Create RX Characteristic (App -> ESP32)
        NimBLECharacteristic *pRxCharacteristic = pService->createCharacteristic(
                                                   CHARACTERISTIC_UUID_RX,
                                                   NIMBLE_PROPERTY::WRITE
                                               );
        pRxCharacteristic->setCallbacks(new MyRxCallbacks());

        // 6. Start service and advertising
        pService->start();
        pServer->getAdvertising()->addServiceUUID(SERVICE_UUID);
        pServer->getAdvertising()->start();

        Serial.printf("BLE Server ready. Advertising as %s\n", deviceName);
    }

    void sendData(const String& data) {
        if (deviceConnected && pTxCharacteristic) {
            pTxCharacteristic->setValue(data.c_str());
            pTxCharacteristic->notify();
        }
    }

    bool isConnected() {
        return deviceConnected;
    }

private:
    friend class MyServerCallbacks;
    friend class MyRxCallbacks;

    NimBLEServer *pServer;
    NimBLECharacteristic *pTxCharacteristic;
    volatile bool deviceConnected;

    // --- Server Connection Callbacks ---
    class MyServerCallbacks : public NimBLEServerCallbacks {
        void onConnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo) override {
            bleManager.deviceConnected = true;
            Serial.println("BLE device connected");
        }

        void onDisconnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo, int reason) override {
            bleManager.deviceConnected = false;
            Serial.println("BLE device disconnected");
            // Restart advertising to allow a new connection
            NimBLEDevice::getAdvertising()->start();
        }
    };

    // --- RX Characteristic Callbacks ---
    class MyRxCallbacks : public NimBLECharacteristicCallbacks {
        void onWrite(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo& connInfo) override {
            std::string rxValue = pCharacteristic->getValue();

            if (rxValue.length() > 0) {
                Serial.print("Received via BLE: ");
                Serial.println(rxValue.c_str());

                // Process the received command
                processCommand(String(rxValue.c_str()));
            }
        }
    };
};

// External declaration of the global instance
extern BLEManager bleManager;

#endif // BLE_SERVICE_H
*/
