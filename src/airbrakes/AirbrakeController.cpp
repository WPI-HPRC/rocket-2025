#include "AirbrakeController.h"

void AirbrakeController::init() {
  servo.attach(servo_pin);
  pinMode(feedback_pin, INPUT);
}

void AirbrakeController::write(int val) {
  servo.writeMicroseconds(val);
}

int AirbrakeController::read() {
  return analogRead(feedback_pin);
}
