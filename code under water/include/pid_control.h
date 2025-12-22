#ifndef PID_CONTROL_H
#define PID_CONTROL_H

class PID {
public:
    PID(float kp, float ki, float kd, float outLimit);

    void reset();
    float compute(float setpoint, float measurement);

private:
    float _kp, _ki, _kd;
    float _outLimit;

    float _integral;
    float _prevError;
};

#endif
