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

    if (dt <= 0.0)
        return 0.0;

    // Proteksi jika loop lama (misal delay / serial)
    if (dt > 0.5)
        dt = 0.05;

    double error = _setpoint - input;
    // DEADZONE (anti jitter)
    if (abs(error) < 0.5)
    error = 0;

    // Mode sudut (yaw)
    if (_angleMode)
    {
        if (error > 180.0)
            error -= 360.0;
        else if (error < -180.0)
            error += 360.0;
    }

    // Proportional
    double P = _kp * error;

    // Integral (anti wind-up)
    _integral += error * dt;
    double I = _ki * _integral;

    if (I > _maxOut) {
        I = _maxOut;
        _integral -= error * dt;
    }
    else if (I < _minOut) {
        I = _minOut;
        _integral -= error * dt;
    }

    // Derivative
    double D = _kd * (error - _lastError) / dt;

    _lastError = error;
    _lastTime = now;

    return constrain(P + I + D, _minOut, _maxOut);
}
