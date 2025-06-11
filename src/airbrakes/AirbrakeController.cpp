#include "AirbrakeController.h"
#include "boilerplate/StateEstimator/kfConsts.h"
#include "FlightParams.h"

void AirbrakeController::init() {
    servo.attach(servo_pin);
    pinMode(feedback_pin, INPUT);
}

void AirbrakeController::write(int val) { servo.writeMicroseconds(val); }

int AirbrakeController::read() { return analogRead(feedback_pin); }

float AirbrakeController::deployAmount(float acc_z, float v_z, float alt) {
    acc_z = (acc_z - 1) * g * -1;

    float curr_max_height = alt - (v_z * v_z) / (2 * acc_z);
    // 25m subtracted here since we are overestimating the current apogee due to not taking drag into account
    if (curr_max_height <= /* TARGET_APOGEE - 25 */ 5) {
        return 0;
    } else {
        float new_deploy = current_break_deploy * 1.05;
        return (new_deploy >= 1 ? 1 : new_deploy);
    }
}
