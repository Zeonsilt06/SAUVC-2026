#include "pid_control.h"
#include <Arduino.h>

PID::PID(float kp, float ki, float kd, float outLimit)
{
    _kp = kp;
    _ki = ki;
    _kd = kd;
    _outLimit = outLimit;

    _integral = 0;
    _prevError = 0;
}

void PID::reset()
{
    _integral = 0;
    _prevError = 0;
}

float PID::compute(float setpoint, float measurement)
{
    float error = setpoint - measurement;

    _integral += error;
    _integral = constrain(_integral, -_outLimit, _outLimit);

    float derivative = error - _prevError;
    _prevError = error;

    float output =
        (_kp * error) +
        (_ki * _integral) +
        (_kd * derivative);

    return constrain(output, -_outLimit, _outLimit);
}
