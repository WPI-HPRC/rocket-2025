#include "Rocket/Rocket.h"


#include <iostream>
#include <cstring>

void Rocket::iterate() {

    Sensor** sensorArray = (Sensor**) &sensors;

    for (size_t i = 0; i < sizeof(Sensors) / sizeof(Sensor*); i++) {
        void* data = sensorArray[i]->update();

        size_t dataSize = sensorArray[i]->sensorDataBytes();
        char* buffer = new char[dataSize];
        memcpy(buffer, data, dataSize);

        if (data) std::cout << "Data" << i << ": " << buffer << std::endl;

        delete[] buffer;

    }
}