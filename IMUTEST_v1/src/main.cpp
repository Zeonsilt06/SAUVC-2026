#include <Arduino.h>
#include "sensor.h"
#include "calibration.h"
#include "orientation.h"

void setup() {
    Serial.begin(115200);
    
    while (!Serial && millis() < 3000) {
        delay(1);
    }
    
    // Inisialisasi sensor
    if (!initializeSensor()) {
        while(1) {
            delay(1000);
        }
    }
    
    setupSensor();
    delay(1000);
    
    // Kalibrasi
    calibrateSensors();
    
    lastTime = millis();
    
    Serial.println("Mulai streaming data...");
    Serial.println();
}

void loop() {
    sensors_event_t accel, mag, gyro, temp;
    readSensorData(accel, mag, gyro, temp);
    
    // Koreksi offset
    float ax = accel.acceleration.x - accelOffsetX;
    float ay = accel.acceleration.y - accelOffsetY;
    float az = accel.acceleration.z;
    
    // Apply low-pass filter
    applyAccelFilter(ax, ay, az);
    
    // Koreksi offset gyroscope
    float gx = gyro.gyro.x - gyroOffsetX;
    float gy = gyro.gyro.y - gyroOffsetY;
    float gz = gyro.gyro.z - gyroOffsetZ;
    
    // Apply dead zone
    applyDeadZone(gx, gy, gz);
    
    // Hitung orientasi
    calculateOrientation(accelFilterX, accelFilterY, accelFilterZ, gx, gy, gz);
    
    // Kirim data untuk SerialPlot (terpisah)
    Serial.print("Roll: ");
    Serial.print(getRollDegrees());
    Serial.print(" | Pitch: ");
    Serial.print(getPitchDegrees());
    Serial.print(" | Yaw: ");
    Serial.println(getYawDegrees());
    
    delay(50);
}