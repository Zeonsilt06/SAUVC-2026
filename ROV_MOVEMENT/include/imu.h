#ifndef IMU_H
#define IMU_H

#include <Arduino.h>
#include <Wire.h>
#include <SFE_LSM9DS0.h>

class IMU
{
public:
    IMU();

    bool begin();
    void update();

    // Getter
    float getYaw();
    float getPitch();
    float getRoll();

private:
    // Sensor object
    LSM9DS0 imu;

    // Raw & processed data
    float ax, ay, az;
    float gx, gy, gz;
    float mx, my, mz;

    float accelPitch, accelRoll;
    float compensatedYaw;
};

#endif