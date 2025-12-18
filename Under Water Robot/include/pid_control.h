#ifndef PID_CONTROL_H
#define PID_CONTROL_H

class PID {
public:
    PID(float kp, float ki, float kd, float limit);
    float compute(float setpoint, float input);

private:
    float _kp, _ki, _kd;
    float _prevError;
    float _integral;
    float _limit;
    unsigned long _lastTime;
};

#endif
