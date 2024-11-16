//
// Created by Daniel Coburn on 11/14/24.
//

#include "Magnetometer.h"

bool Magnetometer::init() {
    Serial.println("Magnetometer init start...");
    if (!this->mag.begin()) {
        Serial.println("Magnetometer init fail ---");
        return false;
    }

    this->mag.softReset();
    this->mag.setFilterBandwidth(800); // Filter Bandwidth - BITS 0 | 0 (100Hz - 0.4 mG RMS Noise)

    Serial.println("Magnetometer init good ---");
    initStatus = true;
    return true;
}

void* Magnetometer::poll() {
    uint32_t rawX = mag.getMeasurementX();
    uint32_t rawY = mag.getMeasurementY();
    uint32_t rawZ = mag.getMeasurementZ();

    // Convert raw 18 bit unsigned integer to +/- 1.0 approximate zero is 2^17 (131072)
    double scaledX = (double) rawX - 131072.0;
    scaledX /= 131072.0;
    double scaledY = (double) rawY - 131072.0;
    scaledY /= 131072.0;
    double scaledZ = (double) rawZ - 131072.0;
    scaledZ /= 131072.0;

    data.x = scaledX * 800000.0; // [nT]
    data.y = scaledY * 800000.0; // [nT]
    data.z = scaledZ * 800000.0; // [nT]

    return &data;
}

size_t Magnetometer::sensorDataBytes() const {
    return sizeof(MagnetometerData);
}

std::optional<MagnetometerData> Magnetometer::getData() {
    if(initStatus) {
        return data;
    } else {
        return std::nullopt;
    }
}

void Magnetometer::debugData() {
    if(initStatus) {
        Serial.print("time: ");
        Serial.print(this->data.id.timestamp);
        Serial.print(", x: ");
        Serial.print(this->data.x);
        Serial.print(", y: ");
        Serial.print(this->data.y);
        Serial.print(", z: ");
        Serial.print(this->data.x);
    } else {
        Serial.println("No Magnetometer");
    }
}