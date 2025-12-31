#ifndef DEPTH_H
#define DEPTH_H

#include <Arduino.h>
#include "SparkFun_MS5803_I2C.h"

class DepthController
{
public:
    DepthController();

    bool begin(TwoWire &wire);
    void update();

    float getDepth();   // depth dalam cm

private:
    MS5803 pressureSensor;

    float surfacePressure;
    float currentDepth;

    void computeDepth(float pressure);
};

#endif
