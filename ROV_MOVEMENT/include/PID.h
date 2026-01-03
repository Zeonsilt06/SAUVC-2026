#ifndef PID_H
#define PID_H

class MyPID { // Ganti nama menjadi MyPID
public:
    MyPID(float kp = 0.0, float ki = 0.0, float kd = 0.0, float dt = 0.02); // dt diset 0.02 (sesuai loop 50Hz)

    void setGains(float kp, float ki, float kd);
    void reset();
    float compute(float setpoint, float measured);

private:
    float _kp, _ki, _kd;
    float _dt;
    float _integral;
    float _prev_error;
};

#endif