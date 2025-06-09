#pragma once

#include "../boilerplate/Sensors/Impl/ASM330.h"
#include "../boilerplate/Sensors/Impl/ICM20948.h"
#include "../boilerplate/Sensors/Impl/LPS22.h"
#include "../boilerplate/TimedPointer/TimedPointer.h"
#include <Arduino.h>
#include <Servo.h>
#include <cstdint>

class AirbrakeController {
  public:
    AirbrakeController(uint8_t servo_pin, uint8_t feedback_pin,
                       const TimedPointer<ICMData> magData,
                       const TimedPointer<ASM330Data> accData,
                       const TimedPointer<LPS22Data> baroData)
        : servo_pin(servo_pin), feedback_pin(feedback_pin), servo(),
          magData(magData), accData(accData), baroData(baroData) {}

    void init();

    void write(int val);

    int read();

    float controllBreak();

  private:
    float deployAmmount(float acc_z, float v_z, float alt, float curr_deploy);
    // acc_z = (acc_z - g)*-g
    // target_height = alt - (v_z)^2/2acc_z_gloal

    uint8_t servo_pin;
    uint8_t feedback_pin;
    Servo servo;
    float init_break_deploy;
    float current_break_deploy;
    float target_height = 3073.0; // m. This is 25m over actual target to
                                  // account for most likley going under target?
                                  // think this over
    const TimedPointer<ICMData> magData;
    const TimedPointer<ASM330Data> accData;
    const TimedPointer<LPS22Data> baroData;
    long lastMagRead = 0; 
    long lastAccRead = 0;
    long lastBaroRead = 0;
};
