#include <Arduino.h>

#include "Context.h"
#include "states/States.h"
#include <boilerplate/StateMachine/StateMachine.h>

Context ctx = {

};

StateMachine_t stateMachine((State_t *)new PreLaunch(&ctx));

void setup() {
    Serial.begin(9600);
    stateMachine.initialize();
}

void loop() { stateMachine.loop(); }
