#ifndef SENSORHELPER_H
#define SENSORHELPER_H
#include <Adafruit_LSM9DS0.h>
#include <SparkFun_MS5803_I2C.h>

extern float roll, pitch, yaw;
bool initSensors(MS5803 &press);
void updateSensors(Adafruit_LSM9DS0 &lsm, MS5803 &press, float surfaceP, float &dpt);
#endif