#ifndef SENSOR_HANDLER_H
#define SENSOR_HANDLER_H

#include <Adafruit_LSM9DS0.h>
#include <Adafruit_Sensor.h>

// Pastikan ada 9 argumen float di sini (ax sampai mz)
bool initSensor();
void getRawData(float &ax, float &ay, float &az, float &gx, float &gy, float &gz, float &mx, float &my, float &mz);

#endif