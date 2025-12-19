#include "MotorController.h"

MotorController::MotorController(int id, int pwmPin, int dirPin) {
    this->motorID = id;
    this->pwmPin = pwmPin;
    this->dirPin = dirPin;
    this->maxPWM = 255;
    this->minPWM = 0;
}

void MotorController::begin() {
    pinMode(pwmPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
    analogWrite(pwmPin, 0);
    digitalWrite(dirPin, LOW);
}

void MotorController::setPWMFrequency(int frequency) {
    analogWriteFrequency(pwmPin, frequency);
    analogWriteResolution(8);
}

void MotorController::drive(double output) {
    int pwmValue;
    
    if (output >= 0) {
        digitalWrite(dirPin, HIGH);
        pwmValue = constrain(abs(output), minPWM, maxPWM);
    } else {
        digitalWrite(dirPin, LOW);
        pwmValue = constrain(abs(output), minPWM, maxPWM);
    }
    
    analogWrite(pwmPin, pwmValue);
}

void MotorController::stop() {
    analogWrite(pwmPin, 0);
}

void MotorController::setLimits(int min, int max) {
    this->minPWM = min;
    this->maxPWM = max;
}