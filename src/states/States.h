#pragma once

#include "../Context.h"
#include "../boilerplate/StateMachine/State.h"
#include "../boilerplate/StateMachine/StateMachine.h"
#include <Arduino.h>

enum StateId {
    ID_PreLaunch,
    ID_Boost,
    ID_Coast,
    ID_DrogueDescent,
    ID_MainDescent,
    ID_Recovery,
    ID_Abort
};

using State_t = State<Context, StateId, decltype(&millis)>;
using StateMachine_t = StateMachine<Context, StateId, decltype(&millis)>;

#define STATE(name)                                                            \
    class name : public State_t {                                                     \
      public:                                                                  \
        name(Context *ctx) : State_t(ID_##name, ::millis, ctx) {}              \
                                                                               \
      private:                                                                 \
        void initialize_impl() override;                                       \
        State_t *loop_impl() override;                                         \
    }

STATE(PreLaunch);

STATE(Boost);

STATE(Coast);

STATE(DrogueDescent);

STATE(MainDescent);

STATE(Recovery);

STATE(Abort);
