//
// Created by Daniel Coburn on 9/27/24.
//

#include "Example.h"
#include "Arduino.h"

std::optional<Data> Example::update() {
    int curTime = millis();

    if(curTime - lastTimeRead > readTime) {
        // read
        lastTimeRead = curTime;
        data = Data {
            .pointer = (void*)&data,
            .len = sizeof data,
        };

        return data;
    } else {
        // none
        return {};
    }
}
