#include "PID.h"
#include <Arduino.h>

PID::PID(float kP, float kI, float kD, float minOut, float maxOut) 
    : _kP(kP), _kI(kI), _kD(kD), _min(minOut), _max(maxOut), _integral(0), _lastErr(0) {}

float PID::compute(float setpoint, float measured, float dt) {
    float error = setpoint - measured;
    _integral += error * dt;
    float derivative = (error - _lastErr) / dt;
    float output = (_kP * error) + (_kI * _integral) + (_kD * derivative);
    _lastErr = error;
    return constrain(output, _min, _max);
}

void PID::reset() { _integral = 0; _lastErr = 0; }