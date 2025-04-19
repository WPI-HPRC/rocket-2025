#pragma once

#include "Arduino.h"

#include "BasicLinearAlgebra.h"

/**
 * @author @frostydev99 @cvolf-exe
 * @brief Quaternion State Estimator
 */
class Att_EKF {
public:

    Att_EKF() = default;

    int init(BLA::Matrix<7>);

    void onLoop();

private:

};