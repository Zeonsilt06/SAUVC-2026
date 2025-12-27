#ifndef MIXEDMOTORDRIVER_H
#define MIXEDMOTORDRIVER_H

#include <Servo.h>

class MixedMotorDriver {
public:
    MixedMotorDriver(int h1, int h2, int v1, int v2);
    void begin();
    void setHorizontal(int throttle, int turn);
    void setVertical(int throttle, int rollCorr);
    void stopAll();

private:
    Servo mH1, mH2, mV1, mV2;
    int _pinH1, _pinH2, _pinV1, _pinV2;
};

#endif