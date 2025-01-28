#pragma once
#include "Arduino.h"
#include <TelemetryBoard/XBeeProSX.h>
#include <EKF/AttitudeEKF.h>

//! @brief Enum representing the id of the state, to be used in logging and communication with ground station
enum StateId
{
    ID_PreLaunch = 0,
    ID_Launch,
    ID_Coast,
    ID_DrogueDescent,
    ID_MainDescent,
    ID_Recovery,
    ID_Abort
};

struct Context {
  Sensors *sensors;
  AttitudeStateEstimator *attitudeStateEstimator;
};

/**
 * @brief Abstract class representing a rocket state.
 */
class State
{
public:
    /**
     * @brief Code to be run once when the state begins.
     */
    void initialize();
    /**
     * @brief Code to be run every iteration of the loop while the rocket is in this state.
     * @return The pointer to the next state or nullptr if the state has not changed.
     */
    State *loop();
    /**
     * @brief Get the ID of this state
     */
    virtual enum StateId getId() = 0;
    virtual ~State() {}

protected:
    //! @note Constructor to be called from subclasses to initialize the `sensors` object
    State(Context ctx);
    //! @brief number of milliseconds since the initialize call
    long long currentTime = 0;
    //! @brief number of milliseconds since the last loop call
    long long deltaTime = 0;
    //! @brief loop count since initialization
    long long loopCount = 0;
    //! @brief "global" context object
    Context ctx;

private:
    //! @brief number of milliseconds from boot to the initialize call
    long long startTime = 0;
    //! @brief number of milliseconds from boot to the previous loop call
    long long lastLoopTime = 0;
    virtual void initialize_impl() = 0;
    virtual State *loop_impl() = 0;
};
