#pragma once

#include <cmath>
#include <cstdint>

constexpr float LAUNCH_THRESHHOLD = 5; // [g] upwards acceleration greater than this value = launch detected

constexpr uint32_t BOOST_MAX_TIME = 3000; // [ms] FIXME: put actual value here
constexpr float BURNOUT_THRESHHOLD = 0.3; // [g] upwards accelerateion smaller than this value = motor burnout detected

constexpr uint32_t WAIT_AFTER_BURNOUT = 1000; // [ms] Time to wait after burnout before actuating airbrakes.
constexpr uint32_t COAST_EXPECTED_TIME = 5000; // [ms] Expected time for coast phase to last FIXME: put actual value here
constexpr uint32_t COAST_AIRBRAKE_TIME = COAST_EXPECTED_TIME / 2; // [ms] Time in coast while the airbrakes will be controlled
constexpr uint32_t COAST_MAX_TIME = COAST_EXPECTED_TIME * 1.5; // [ms] Time in coast after which the state times out and forcefully transitions
constexpr float APOGEE_VEL_THRESHHOLD = 0.3; // [m/s] magnitude of velocity (numerical derivative of baro alt) smaller than this value = apogee detected
constexpr float TARGET_APOGEE = 3048; // [m] 10,000 feet in meters

constexpr uint32_t DROGUE_DESCENT_MAX_TIME = 5000; // [ms] FIXME: put actual value here
constexpr float DROGUE_DESCENT_VELOCITY = 23; // [m/s] magnitude of expected velocity during drogue descent FIXME: put actual value here
constexpr float MAIN_DESCENT_VELOCITY = 10; // [m/s] magnitude of expected velocity during main descent FIXME: put actual value here
constexpr float DROGUE_DESCENT_VEL_THRESHHOLD = (std::abs(MAIN_DESCENT_VELOCITY) + std::abs(DROGUE_DESCENT_VELOCITY)) / 2;

constexpr uint32_t MAIN_DESCENT_MAX_TIME = 5000; // [ms] FIXME: put actual value here
constexpr float LANDED_VEL_THRESHHOLD = 0.1; // [m/s] magnitude of velocity (numerical derivative of baro alt) smaller than this value = landing detected
