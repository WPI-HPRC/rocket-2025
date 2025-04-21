#pragma once

#include <Arduino.h>
#include <Servo.h>
#include <cstdint>

class AirbrakeController {
  public:
    AirbrakeController(uint8_t servo_pin, uint8_t feedback_pin)
        : servo_pin(servo_pin), feedback_pin(feedback_pin), servo() {}

    void init();

    void write(int val);

    int read();

  private:
    uint8_t servo_pin;
    uint8_t feedback_pin;
    Servo servo;
};
