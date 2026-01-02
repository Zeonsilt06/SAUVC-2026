#ifndef IMU_H
#define IMU_H

#include <Arduino.h>
#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050.h>
#include <HMC5883L.h>
    
class IMU
{
public:
    IMU();
    bool begin();
    void update();

    float getYaw();
    float getPitch();
    float getRoll();

private:
    HMC5883L mag;
    MPU6050 accelgyro;

    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    int16_t mx, my, mz;

    float Ax, Ay, Az;
    float Gx, Gy, Gz;
    float Mx, My, Mz;

    float accelPitch, accelRoll;
    float compensatedYaw;
};

#endif
