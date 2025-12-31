#include "imu.h"
#include <math.h>

#define LSM9DS0_XM  0x1D
#define LSM9DS0_G   0x6B

IMU::IMU() :
    imu(MODE_I2C, LSM9DS0_G, LSM9DS0_XM),
    ax(0), ay(0), az(0),
    gx(0), gy(0), gz(0),
    mx(0), my(0), mz(0),
    accelPitch(0), accelRoll(0),
    compensatedYaw(0)
{
}

bool IMU::begin()
{
    return (imu.begin() == 0x49D4);
}

void IMU::update()
{
    imu.readGyro();
    imu.readAccel();
    imu.readMag();

    ax = imu.calcAccel(imu.ax);
    ay = imu.calcAccel(imu.ay);
    az = imu.calcAccel(imu.az);

    mx = imu.calcMag(imu.mx);
    my = imu.calcMag(imu.my);
    mz = imu.calcMag(imu.mz);

    accelPitch = atan2(ay, sqrt(ax*ax + az*az));
    accelRoll  = atan2(-ax, sqrt(ay*ay + az*az));

    float Yh = (my * cos(accelRoll)) - (mz * sin(accelRoll));
    float Xh = (mx * cos(accelPitch)) +
               (my * sin(accelRoll) * sin(accelPitch)) +
               (mz * cos(accelRoll) * sin(accelPitch));

    compensatedYaw = atan2(Yh, Xh) * 180.0 / M_PI;
    if (compensatedYaw < 0) compensatedYaw += 360;
}

float IMU::getYaw()   { return compensatedYaw; }
float IMU::getPitch() { return accelPitch * 180.0 / M_PI; }
float IMU::getRoll()  { return accelRoll  * 180.0 / M_PI; }
