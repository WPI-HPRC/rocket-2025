#include "Rocket/Rocket.h"
#include <Arduino.h>


#include <cstring>

Rocket::Rocket(Time *time) : time(time) {

    sensors = Sensors {
      .accelerometer = new Accelerometer(time, 1000 / 100),
    };
}

void Rocket::init() {
    Sensor** sensorArray = (Sensor **) &sensors;
    for (size_t i = 0; i < sizeof(Sensors) / sizeof(Sensor *); i++) {
        sensorArray[i]->init();
    }
}

void Rocket::iterate() {

    Sensor** sensorArray = (Sensor**) &sensors;

    for (size_t i = 0; i < sizeof(Sensors) / sizeof(Sensor*); i++) {
        void* data = sensorArray[i]->update();
        // Serial.println(data != nullptr);

        size_t dataSize = sensorArray[i]->sensorDataBytes();
        if (data) {
          // Serial.println(sensors.accelerometer->data.accX);
            // Serial.print("accX: "); Serial.print(sensors.accelerometer->data.accX);
            // Serial.print(", accY: "); Serial.print(sensors.accelerometer->data.accY);
            // Serial.print(", accZ: "); Serial.print(sensors.accelerometer->data.accZ);
            // Serial.print(", gyroX: "); Serial.print(sensors.accelerometer->data.gyroX);
            // Serial.print(", gyroY: "); Serial.print(sensors.accelerometer->data.gyroY);
            // Serial.print(", gyroZ: "); Serial.println(sensors.accelerometer->data.gyroZ);
            char* buffer = new char[dataSize];
            memcpy(buffer, data, dataSize);

            // Serial.printf("Data %d: %s @ %d\n", i, buffer, sensorArray[i]->getLastTimeRead());

            delete[] buffer;
        }
    }

}
