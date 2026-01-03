#ifndef IMU_H
#define IMU_H

#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050.h>
#include <HMC5883L.h>
#include <MadgwickAHRS.h>

class IMU {
public:
    IMU();

    bool begin();  // Inisialisasi sensor

    void update(); // Baca sensor + update Madgwick filter

    // Dapatkan nilai yang sudah di-zero dan di-smooth (roll 0-360°)
    float getPitch();  // -180 ~ +180
    float getRoll();   // 0 ~ 360
    float getYaw();    // 0 ~ 360

    void calibrateZero(); // Auto-zero saat diam (panggil di setup setelah stabil)

private:
    MPU6050 accelgyro;
    HMC5883L mag;
    Madgwick filter;

    int16_t ax, ay, az, gx, gy, gz, mx, my, mz;

    // Raw converted
    float Ax, Ay, Az, Gx, Gy, Gz, Mx, My, Mz;

    // Offset auto-zero
    float pitchOffset = 0.0;
    float rollOffset = 0.0;
    float yawOffset = 0.0;

    // Smoothed values
    float smooth_pitch = 0.0;
    float smooth_roll = 0.0;
    float smooth_yaw = 0.0;

    // Smoothing parameters
    const float alpha_general = 0.15;
    const float alpha_roll = 0.08;
    const float maxChange_general = 4.0;
    const float maxChange_roll = 2.0;

    void readSensors();
    void applySmoothing();
};

#endif