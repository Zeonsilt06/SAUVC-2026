#include "calibration.h"

// Variabel kalibrasi offset
float gyroOffsetX = 0;
float gyroOffsetY = 0;
float gyroOffsetZ = 0;
float accelOffsetX = 0;
float accelOffsetY = 0;

// Dead zone yang lebih ketat
const float GYRO_DEAD_ZONE = 0.015;      // rad/s
const float ACCEL_DEAD_ZONE = 0.05;      // m/s²
const float ANGLE_DEAD_ZONE = 0.5;       // degrees

// Low-pass filter untuk accelerometer
float accelFilterX = 0;
float accelFilterY = 0;
float accelFilterZ = 0;
const float ACCEL_FILTER_ALPHA = 0.3;

void calibrateSensors() {
    Serial.println("=================================");
    Serial.println("KALIBRASI SENSOR");
    Serial.println("Letakkan sensor di permukaan DATAR");
    Serial.println("JANGAN GERAKKAN selama 5 detik!");
    Serial.println("=================================");
    
    delay(3000);
    
    float sumGX = 0, sumGY = 0, sumGZ = 0;
    float sumAX = 0, sumAY = 0;
    int samples = 200;
    
    Serial.println("Mengambil sample...");
    
    for(int i = 0; i < samples; i++) {
        sensors_event_t accel, mag, gyro, temp;
        readSensorData(accel, mag, gyro, temp);
        
        sumGX += gyro.gyro.x;
        sumGY += gyro.gyro.y;
        sumGZ += gyro.gyro.z;
        sumAX += accel.acceleration.x;
        sumAY += accel.acceleration.y;
        
        if (i % 20 == 0) Serial.print(".");
        delay(10);
    }
    
    Serial.println();
    
    gyroOffsetX = sumGX / samples;
    gyroOffsetY = sumGY / samples;
    gyroOffsetZ = sumGZ / samples;
    accelOffsetX = sumAX / samples;
    accelOffsetY = sumAY / samples;
    
    Serial.println("Kalibrasi Selesai!");
    Serial.println("-------------------");
    Serial.print("Gyro Offset X: "); Serial.println(gyroOffsetX, 5);
    Serial.print("Gyro Offset Y: "); Serial.println(gyroOffsetY, 5);
    Serial.print("Gyro Offset Z: "); Serial.println(gyroOffsetZ, 5);
    Serial.print("Accel Offset X: "); Serial.println(accelOffsetX, 3);
    Serial.print("Accel Offset Y: "); Serial.println(accelOffsetY, 3);
    Serial.println("=================================");
    Serial.println();
    
    delay(2000);
}

void applyAccelFilter(float ax, float ay, float az) {
    // Low-pass filter untuk accelerometer (mengurangi noise)
    accelFilterX = accelFilterX * (1.0 - ACCEL_FILTER_ALPHA) + ax * ACCEL_FILTER_ALPHA;
    accelFilterY = accelFilterY * (1.0 - ACCEL_FILTER_ALPHA) + ay * ACCEL_FILTER_ALPHA;
    accelFilterZ = accelFilterZ * (1.0 - ACCEL_FILTER_ALPHA) + az * ACCEL_FILTER_ALPHA;
    
    // Dead zone untuk accelerometer
    if (abs(accelFilterX) < ACCEL_DEAD_ZONE) accelFilterX = 0;
    if (abs(accelFilterY) < ACCEL_DEAD_ZONE) accelFilterY = 0;
}

void applyDeadZone(float &gx, float &gy, float &gz) {
    // Dead zone untuk gyroscope
    if (abs(gx) < GYRO_DEAD_ZONE) gx = 0;
    if (abs(gy) < GYRO_DEAD_ZONE) gy = 0;
    if (abs(gz) < GYRO_DEAD_ZONE) gz = 0;
}