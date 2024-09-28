#include "Time.h"

#include <Arduino.h>

class ArduinoTime: public Time {
public:
    long millis() const override {
        return ::millis();
    }
};