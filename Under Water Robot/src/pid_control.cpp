#include "pid_control.h"
#include <Arduino.h>

PID::PID(float kp, float ki, float kd, float limit) {
    _kp = kp;
    _ki = ki;
    _kd = kd;
    _limit = limit;
    _prevError = 0;
    _integral = 0;
    _lastTime = millis();
}

float PID::compute(float setpoint, float input) {
    unsigned long now = millis();
    float dt = (now - _lastTime) / 1000.0;
    _lastTime = now;

    float error = setpoint - input;
    _integral += error * dt;
    float derivative = (dt > 0) ? (error - _prevError) / dt : 0;

    float output = _kp * error + _ki * _integral + _kd * derivative;
    _prevError = error;

    if (output > _limit) output = _limit;
    if (output < -_limit) output = -_limit;
    return output;
}
