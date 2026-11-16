#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEAdvertising.h>
#include <string.h>

// =========================================================
// 1. CONFIGURATION
// =========================================================

// Team UUID
#define BEACON_UUID "7A1B2C3D-4E5F-6071-8293-A4B5C6D7E8F9"

// Compartment number (0~4)
#define BEACON_MAJOR 1

// Minor value (fixed at 0)
#define BEACON_MINOR 0

// Advertising interval (ms)
#define TX_INTERVAL_MS 100

// Measured Power (-59 dBm)
#define TX_POWER_1M -59


// =========================================================
// 2. UUID CONVERSION
// =========================================================

bool parseUUID(const char *uuid, uint8_t *output) {

    int count = 0;
    int highNibble = -1;

    while (*uuid) {

        char c = *uuid++;

        if (c == '-') continue;

        int value = -1;

        if (c >= '0' && c <= '9')
            value = c - '0';

        else if (c >= 'A' && c <= 'F')
            value = c - 'A' + 10;

        else if (c >= 'a' && c <= 'f')
            value = c - 'a' + 10;

        else
            return false;

        if (highNibble == -1) {

            highNibble = value;

        } else {

            if (count >= 16)
                return false;

            output[count++] =
                (uint8_t)((highNibble << 4) | value);

            highNibble = -1;
        }
    }

    return count == 16 && highNibble == -1;
}


// =========================================================
// 3. BUILD IBEACON ADVERTISEMENT
// =========================================================

bool startBeacon() {

    uint8_t uuidBytes[16];

    if (!parseUUID(BEACON_UUID, uuidBytes)) {

        Serial.println("ERROR: Invalid UUID");
        return false;
    }

    // Complete manufacturer AD structure (27 bytes)
    uint8_t manufacturerAD[27] = {0};

    // Length and type
    manufacturerAD[0] = 0x1A;
    manufacturerAD[1] = 0xFF;

    // Apple Manufacturer ID
    manufacturerAD[2] = 0x4C;
    manufacturerAD[3] = 0x00;

    // iBeacon Prefix
    manufacturerAD[4] = 0x02;
    manufacturerAD[5] = 0x15;

    // UUID (16 bytes)
    memcpy(
        &manufacturerAD[6],
        uuidBytes,
        sizeof(uuidBytes)
    );

    // Major: Big Endian
    manufacturerAD[22] =
        (BEACON_MAJOR >> 8) & 0xFF;

    manufacturerAD[23] =
        BEACON_MAJOR & 0xFF;

    // Minor: Big Endian
    manufacturerAD[24] =
        (BEACON_MINOR >> 8) & 0xFF;

    manufacturerAD[25] =
        BEACON_MINOR & 0xFF;

    // Measured Power
    manufacturerAD[26] =
        (uint8_t)TX_POWER_1M;


    // =====================================================
    // 4. ADVERTISING SETTINGS
    // =====================================================

    BLEAdvertisementData advertisementData;

    advertisementData.setFlags(0x06);

    // Add complete manufacturer AD structure
    advertisementData.addData(
        (char *)manufacturerAD,
        sizeof(manufacturerAD)
    );

    BLEAdvertising *advertising =
        BLEDevice::getAdvertising();

    // Non-connectable advertising
    advertising->setAdvertisementType(
        ADV_TYPE_NONCONN_IND
    );

    // 1 BLE interval unit = 0.625 ms
    uint16_t intervalUnits =
        (TX_INTERVAL_MS * 1000UL) / 625UL;

    advertising->setMinInterval(intervalUnits);
    advertising->setMaxInterval(intervalUnits);

    // Apply advertisement data
    if (!advertising->setAdvertisementData(
            advertisementData)) {

        Serial.println(
            "ERROR: Failed to set advertising data"
        );

        return false;
    }

    // Start broadcasting
    if (!advertising->start()) {

        Serial.println(
            "ERROR: Failed to request advertising start"
        );

        return false;
    }

    return true;
}


// =========================================================
// 5. SETUP
// =========================================================

void setup() {

    Serial.begin(115200);
    delay(1000);

    // Validate configuration
    if (BEACON_MAJOR < 0 || BEACON_MAJOR > 4 ||
        BEACON_MINOR < 0 || BEACON_MINOR > 65535 ||
        TX_INTERVAL_MS < 20 || TX_INTERVAL_MS > 10240) {

        Serial.println("ERROR: Invalid beacon configuration");
        return;
    }

    // Initialize BLE
    BLEDevice::init("");

    // Start iBeacon
    if (startBeacon()) {

        Serial.println("================================");
        Serial.println("ESP32 iBeacon advertising requested");

        Serial.print("UUID: ");
        Serial.println(BEACON_UUID);

        Serial.print("Major (Slot ID): ");
        Serial.println(BEACON_MAJOR);

        Serial.print("Minor: ");
        Serial.println(BEACON_MINOR);

        Serial.print("Interval: ");
        Serial.print(TX_INTERVAL_MS);
        Serial.println(" ms");

        Serial.println("================================");
    }
}


// =========================================================
// 6. LOOP
// =========================================================

void loop() {

    // BLE advertising runs independently
    delay(1000);
}
