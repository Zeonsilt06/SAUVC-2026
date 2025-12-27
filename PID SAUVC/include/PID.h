#ifndef PID_H
#define PID_H

class PID {
public:
    PID(float kP, float kI, float kD, float minOut, float maxOut);
    float compute(float setpoint, float measured, float dt);
    void reset();
private:
    float _kP, _kI, _kD, _min, _max, _integral, _lastErr;
};

#endif