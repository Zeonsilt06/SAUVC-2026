#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <SFE_LSM9DS0.h>

#include "ThrusterControl.h"

/* ================= IMU ================= */
#define LSM9DS0_XM  0x1D
#define LSM9DS0_G   0x6B

LSM9DS0 imu(MODE_I2C, LSM9DS0_G, LSM9DS0_XM);

/* ================= CONTROL ================= */
float targetYaw = 0.0;      
float currentYaw = 0.0;
float yawError   = 0.0;

const float YAW_DEADBAND = 5.0;   // deadband aman
const int   YAW_OFFSET   = 10;    // koreksi pelan

/* ================= SETUP ================= */
void setup()
{
    Serial.begin(115200);
    Wire.begin();

    initMotor();
    stopAll();

    uint16_t status = imu.begin();
    Serial.print("IMU status: 0x");
    Serial.println(status, HEX);

    delay(2000); // stabilisasi IMU

    targetYaw = readYaw(); // lock heading awal
}

/* ================= LOOP (AMAN & HALUS) ================= */
void loop()
{
    currentYaw = readYaw();
    yawError   = targetYaw - currentYaw;

    // Normalize ke -180 .. 180
    if (yawError > 180)  yawError -= 360;
    if (yawError < -180) yawError += 360;

    Serial.print("Yaw: ");
    Serial.print(currentYaw);
    Serial.print("  Error: ");
    Serial.println(yawError);

    if (abs(yawError) <= YAW_DEADBAND)
    {
        // HOLD HALUS (tidak stop total)
        rotateRight(0);
    }
    else if (yawError > 0)
    {
        rotateLeft(YAW_OFFSET);
    }
    else
    {
        rotateRight(YAW_OFFSET);
    }

    delay(50); // loop stabil (20 Hz)
}

/* ================= IMU YAW ================= */
float readYaw()
{
    imu.readAccel();
    imu.readMag();

    float ax = imu.calcAccel(imu.ax);
    float ay = imu.calcAccel(imu.ay);
    float az = imu.calcAccel(imu.az);

    float mx = imu.calcMag(imu.mx);
    float my = imu.calcMag(imu.my);
    float mz = imu.calcMag(imu.mz);

    float accelPitch = atan2(ay, sqrt(ax * ax + az * az));
    float accelRoll  = atan2(-ax, sqrt(ay * ay + az * az));

    float Yh = (my * cos(accelRoll)) - (mz * sin(accelRoll));
    float Xh = (mx * cos(accelPitch)) +
               (my * sin(accelRoll) * sin(accelPitch)) +
               (mz * cos(accelRoll) * sin(accelPitch));

    float yaw = atan2(Yh, Xh) * 180.0 / M_PI;

    if (yaw < 0) yaw += 360;
    return yaw;
}