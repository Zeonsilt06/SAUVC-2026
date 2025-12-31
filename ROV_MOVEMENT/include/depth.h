#ifndef DEPTH_H
#define DEPTH_H

#include <Arduino.h>
#include <Servo.h>
#include "SparkFun_MS5803_I2C.h"

class DepthController
{
public:
    DepthController(uint8_t pinLeft, uint8_t pinRight);

    bool begin(TwoWire &wire);
    void update();
    void setTargetDepth(float depthCm);

    private:
    // Thruster
    Servo M3, M4;
    uint8_t pinM3, pinM4;

    // Sensor
    MS5803 pressureSensor;

    // Depth
    float surfacePressure;
    float targetDepthCm;
    float deadbandCm;

    // PID
    float Kp, Ki, Kd;
    float pidIntegral;
    float lastError;
    unsigned long lastPIDTime;

    void depthPID(float pressure);
};

#endif