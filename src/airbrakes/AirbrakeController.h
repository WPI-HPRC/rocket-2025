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

    int getCurrDeploy() { return current_break_deploy; }

    float deployAmount(float acc_z, float v_z, float alt);

    float constantDeploy();

  private:
    // float deployAmmount(float acc_z, float v_z, float alt, float curr_deploy);
    // acc_z = (acc_z - g)*-g
    // target_height = alt - (v_z)^2/2acc_z_gloal

    uint8_t servo_pin;
    uint8_t feedback_pin;
    Servo servo;
    float init_break_deploy;
    float current_break_deploy;
    float CONSTANT_DEPLOY = 0.8;
};
