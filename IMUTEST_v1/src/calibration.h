#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <Arduino.h>
#include "sensor.h"

// Variabel offset kalibrasi
extern float gyroOffsetX;
extern float gyroOffsetY;
extern float gyroOffsetZ;
extern float accelOffsetX;
extern float accelOffsetY;

// Konstanta dead zone
extern const float GYRO_DEAD_ZONE;
extern const float ACCEL_DEAD_ZONE;
extern const float ANGLE_DEAD_ZONE;

// Low-pass filter untuk accelerometer
extern float accelFilterX;
extern float accelFilterY;
extern float accelFilterZ;
extern const float ACCEL_FILTER_ALPHA;

// Fungsi kalibrasi
void calibrateSensors();

// Fungsi filter dan dead zone
void applyAccelFilter(float ax, float ay, float az);
void applyDeadZone(float &gx, float &gy, float &gz);

#endif