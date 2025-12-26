#ifndef ORIENTATION_H
#define ORIENTATION_H

#include <Arduino.h>

// Variabel orientasi
extern float roll;
extern float pitch;
extern float yaw;

// Filter settings
extern float alpha;
extern unsigned long lastTime;

// Deteksi gerakan
extern float lastRoll;
extern float lastPitch;
extern float lastYaw;
extern bool isMoving;

// Fungsi orientasi
void calculateOrientation(float ax, float ay, float az, 
                         float gx, float gy, float gz);
float normalizeAngle(float angle);

// Getter untuk orientasi dalam derajat
float getRollDegrees();
float getPitchDegrees();
float getYawDegrees();

#endif