#include "MixedMotorDriver.h"
#include "Config.h"
#include <Arduino.h>

// Constructor sekarang menerima pin
MixedMotorDriver::MixedMotorDriver(int h1, int h2, int v1, int v2) {
    _pinH1 = h1; _pinH2 = h2; _pinV1 = v1; _pinV2 = v2;
}

void MixedMotorDriver::begin() {
    mH1.attach(_pinH1);
    mH2.attach(_pinH2);
    mV1.attach(_pinV1);
    mV2.attach(_pinV2);
    stopAll();
}

void MixedMotorDriver::setHorizontal(int throttle, int turn) {
    int s1 = throttle + turn; // Motor Kiri
    int s2 = throttle - turn; // Motor Kanan
    
    // H1 normal, H2 reverse karena CCW
    mH1.writeMicroseconds(map(s1, -100, 100, PWM_MIN, PWM_MAX));
    mH2.writeMicroseconds(map(s2, -100, 100, PWM_MAX, PWM_MIN)); 
}

void MixedMotorDriver::setVertical(int throttle, int rollCorr) {
    int v1 = throttle + rollCorr;
    int v2 = throttle - rollCorr;
    mV1.writeMicroseconds(map(v1, -100, 100, PWM_MIN, PWM_MAX));
    mV2.writeMicroseconds(map(v2, -100, 100, PWM_MIN, PWM_MAX));
}

void MixedMotorDriver::stopAll() {
    mH1.writeMicroseconds(PWM_STOP);
    mH2.writeMicroseconds(PWM_STOP);
    mV1.writeMicroseconds(PWM_STOP);
    mV2.writeMicroseconds(PWM_STOP);
}