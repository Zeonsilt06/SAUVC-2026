#ifndef PID_H
#define PID_H

#include <Arduino.h>

class PID
{
public:
    PID(double kp, double ki, double kd,
        double minOutput, double maxOutput,
        bool angleMode = false);

    void reset();

    void setSetpoint(double setpoint);
    void setGains(double kp, double ki, double kd);
    void setOutputLimits(double minOut, double maxOut);

    double compute(double input);

private:
    double _kp;
    double _ki;
    double _kd;

    double _minOut;
    double _maxOut;

    double _setpoint;

    double _integral;
    double _lastError;
    unsigned long _lastTime;

    bool _angleMode;   // true untuk yaw (0–360 / -180–180)
};

#endif
