#include "orientation.h"
#include "calibration.h"

// Variabel orientasi
float roll = 0;
float pitch = 0;
float yaw = 0;

// Filter settings
float alpha = 0.98;  // Complementary filter
unsigned long lastTime = 0;

// Deteksi gerakan
float lastRoll = 0;
float lastPitch = 0;
float lastYaw = 0;
bool isMoving = false;

void calculateOrientation(float ax, float ay, float az, 
                         float gx, float gy, float gz) {
    unsigned long currentTime = millis();
    float dt = (currentTime - lastTime) / 1000.0;
    lastTime = currentTime;
    
    // Validasi dt
    if (dt <= 0 || dt > 1.0) {
        dt = 0.05;
        return; // Skip frame ini
    }
    
    // Hitung sudut dari accelerometer
    float accelRoll = atan2(ay, az);
    float accelPitch = atan2(-ax, sqrt(ay*ay + az*az));
    
    // Deteksi gerakan
    isMoving = (abs(gx) > 0 || abs(gy) > 0 || abs(gz) > 0);
    
    // Integrasi gyroscope HANYA saat bergerak
    if (isMoving) {
        roll += gx * dt;
        pitch += gy * dt;
        yaw += gz * dt;
    } else {
        // Saat diam, decay perlahan ke 0 untuk yaw
        yaw *= 0.99;
    }
    
    // Complementary filter
    roll = alpha * roll + (1.0 - alpha) * accelRoll;
    pitch = alpha * pitch + (1.0 - alpha) * accelPitch;
    
    // Apply dead zone untuk sudut kecil
    float rollDeg = degrees(roll);
    float pitchDeg = degrees(pitch);
    float yawDeg = degrees(yaw);
    
    if (abs(rollDeg) < ANGLE_DEAD_ZONE) roll = 0;
    if (abs(pitchDeg) < ANGLE_DEAD_ZONE) pitch = 0;
    if (abs(yawDeg) < ANGLE_DEAD_ZONE) yaw = 0;
    
    // Normalize angles
    roll = normalizeAngle(roll);
    pitch = normalizeAngle(pitch);
    yaw = normalizeAngle(yaw);
    
    // Anti-drift: reset jika perubahan sangat kecil dalam waktu lama
    if (!isMoving) {
        float deltaRoll = abs(roll - lastRoll);
        float deltaPitch = abs(pitch - lastPitch);
        
        if (deltaRoll < 0.001 && abs(roll) < 0.01) roll = 0;
        if (deltaPitch < 0.001 && abs(pitch) < 0.01) pitch = 0;
    }
    
    lastRoll = roll;
    lastPitch = pitch;
    lastYaw = yaw;
}

float normalizeAngle(float angle) {
    while (angle > PI) angle -= TWO_PI;
    while (angle < -PI) angle += TWO_PI;
    return angle;
}

float getRollDegrees() {
    return degrees(roll);
}

float getPitchDegrees() {
    return degrees(pitch);
}

float getYawDegrees() {
    return degrees(yaw);
}