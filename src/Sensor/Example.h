//
// Created by Daniel Coburn on 9/27/24.
//

#ifndef NEW_BOARD_24_25_EXAMPLE_H
#define NEW_BOARD_24_25_EXAMPLE_H
#include "Sensor.h"

class Example: Sensor {

    float sensorData;
    static const int readTime = 100; // in ms

public:
    std::optional<Data> update() override;
};

#endif //NEW_BOARD_24_25_EXAMPLE_H
