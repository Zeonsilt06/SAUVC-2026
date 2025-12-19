#include "PIDController.h"
#include <Arduino.h>

PIDController::PIDController(double Kp, double Ki, double Kd, unsigned long sampleTime) {
    this->kp = Kp;
    this->ki = Ki;
    this->kd = Kd;
    this->sampleTime = sampleTime;
    this->integral = 0;
    this->lastError = 0;
    this->output = 0;
    this->minOutput = -255;
    this->maxOutput = 255;
    this->lastTime = 0;
}

void PIDController::setTunings(double Kp, double Ki, double Kd) {
    this->kp = Kp;
    this->ki = Ki;
    this->kd = Kd;
}

void PIDController::setOutputLimits(double min, double max) {
    this->minOutput = min;
    this->maxOutput = max;
}

double PIDController::compute(double setpoint, double input) {
    unsigned long currentTime = millis();
    
    if (currentTime - lastTime >= sampleTime) {
        double dt = (currentTime - lastTime) / 1000.0;
        lastTime = currentTime;
        
        double error = setpoint - input;
        double P = kp * error;
        
        integral += error * dt;
        double I = ki * integral;
        
        double derivative = (error - lastError) / dt;
        double D = kd * derivative;
        
        output = P + I + D;
        
        if (output > maxOutput) {
            integral -= error * dt;
            output = maxOutput;
        } else if (output < minOutput) {
            integral -= error * dt;
            output = minOutput;
        }
        
        lastError = error;
    }
    
    return output;
}

void PIDController::reset() {
    integral = 0;
    lastError = 0;
    output = 0;
}