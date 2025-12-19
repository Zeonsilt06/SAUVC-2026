#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <Arduino.h>

class MotorController {
private:
    int pwmPin;
    int dirPin;
    int maxPWM;
    int minPWM;
    int motorID;

public:
    MotorController(int id, int pwmPin, int dirPin);
    void begin();
    void setPWMFrequency(int frequency);
    void drive(double output);
    void stop();
    void setLimits(int min, int max);
};

#endif
