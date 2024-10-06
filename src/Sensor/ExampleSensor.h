//
// Created by Daniel Coburn on 9/27/24.
//
#pragma once

#include <cstdlib>

namespace ExampleSensor {
    struct Data {
        int a;
        int b;
    };

    struct Context {
    };

    bool init(Context *ctx) { return true; }

    Data poll(Context *ctx) {
        return Data{
                .a = rand(),
                .b = rand(),
        };
    }
} // namespace ExampleSensor
