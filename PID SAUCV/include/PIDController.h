#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

class PIDController {
private:
    double kp, ki, kd;
    double integral;
    double lastError;
    double output;
    double minOutput, maxOutput;
    unsigned long lastTime;
    unsigned long sampleTime;

public:
    PIDController(double Kp, double Ki, double Kd, unsigned long sampleTime = 10);
    void setTunings(double Kp, double Ki, double Kd);
    void setOutputLimits(double min, double max);
    double compute(double setpoint, double input);
    void reset();
    double getKp() { return kp; }
    double getKi() { return ki; }
    double getKd() { return kd; }
};

#endif
