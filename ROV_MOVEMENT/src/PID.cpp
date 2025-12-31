#include "PID.h"

PID::PID(double kp, double ki, double kd,
         double minOutput, double maxOutput,
         bool angleMode)
{
    _kp = kp;
    _ki = ki;
    _kd = kd;

    _minOut = minOutput;
    _maxOut = maxOutput;

    _angleMode = angleMode;

    _setpoint = 0.0;
    reset();
}

void PID::reset()
{
    _integral = 0.0;
    _lastError = 0.0;
    _lastTime = millis();
}

void PID::setSetpoint(double setpoint)
{
    _setpoint = setpoint;
}

void PID::setGains(double kp, double ki, double kd)
{
    _kp = kp;
    _ki = ki;
    _kd = kd;
}

void PID::setOutputLimits(double minOut, double maxOut)
{
    _minOut = minOut;
    _maxOut = maxOut;
}

double PID::compute(double input)
{
    unsigned long now = millis();
    double dt = (double)(now - _lastTime) / 1000.0;

    if (dt <= 0.0) return 0.0;
    if (dt > 0.5) dt = 0.05;

    double error = _setpoint - input;

    if (_angleMode) {
        if (error > 180.0) error -= 360.0;
        else if (error < -180.0) error += 360.0;
    }

    double p = _kp * error;

    _integral += error * dt;
    double i = _ki * _integral;

    if (i > _maxOut) {
        i = _maxOut;
        _integral -= error * dt;
    } else if (i < _minOut) {
        i = _minOut;
        _integral -= error * dt;
    }

    double d = _kd * (error - _lastError) / dt;

    _lastError = error;
    _lastTime = now;

    return constrain(p + i + d, _minOut, _maxOut);
}