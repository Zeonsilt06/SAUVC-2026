#include "imu.h"
#include <math.h>


IMU::IMU() :
    ax(0),ay(0),az(0),
    gx(0),gy(0),gz(0),
    mx(0),my(0),mz(0),
    Ax(0), Ay(0), Az(0),
    Gx(0), Gy(0), Gz(0),
    Mx(0), My(0), Mz(0),
    accelPitch(0), accelRoll(0),
    compensatedYaw(0)
{
}

bool IMU::begin()
{
    Wire1.begin();
    accelgyro.initialize();
    mag.initialize();
    delay(20);
    if((accelgyro.testConnection()) && (mag.testConnection())){
        return (true);
    }
    else{
        return(false);
    }
}

void IMU::update()
{
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    mag.getHeading(&mx, &my, &mz);

    Ax = (float) ax / 16384.0;
    Ay = (float) ay / 16384.0;
    Az = (float) az / 16384.0;
    Gx = (float) Gx / 131.0;
    Gy = (float) Gy / 131.0;
    Gz = (float) Gz / 131.0;
    Mx = (float) Mx / 1090.0;
    Mx = (float) Mx / 1090.0;
    Mx = (float) Mx / 1090.0;

    accelPitch = atan2(Ay, sqrt(Ax*Ax + Az*Az));
    accelRoll  = atan2(-Ax, sqrt(Ay*Ay + Az*Az));

    float Yh = (My * cos(accelRoll)) - (Mz * sin(accelRoll));
    float Xh = (Mx * cos(accelPitch)) +
               (My * sin(accelRoll) * sin(accelPitch)) +
               (Mz * cos(accelRoll) * sin(accelPitch));

    compensatedYaw = atan2(Yh, Xh) * 180.0 / M_PI;
    if (compensatedYaw < 0) compensatedYaw += 360;
}

float IMU::getYaw()   { return compensatedYaw; }
float IMU::getPitch() { return accelPitch * 180.0 / M_PI; }
float IMU::getRoll()  { return accelRoll  * 180.0 / M_PI; }
