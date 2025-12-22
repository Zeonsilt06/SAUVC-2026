#include "LSM9DS0.h"
#include <Wire.h>
#include <SPI.h>

LSM9DS0::LSM9DS0() {}

bool LSM9DS0::begin() {
    // Adafruit_LSM9DS0::begin() doesn't accept addresses in the current library
    if (lsm.begin()) {
        Serial.println("LSM9DS0 found via default interface");
        return true;
    }

    Serial.println("LSM9DS0 not found");
    return false;
}

void LSM9DS0::setAccelRange(uint8_t range) {
    // Cast to the proper enum type
    lsm.setupAccel((Adafruit_LSM9DS0::lsm9ds0AccelRange_t)range);
}

void LSM9DS0::setGyroRange(uint8_t scale) {
    // Cast to the proper enum type
    lsm.setupGyro((Adafruit_LSM9DS0::lsm9ds0GyroScale_t)scale);
}

void LSM9DS0::setMagRange(uint8_t gain) {
    // Cast to the proper enum type
    lsm.setupMag((Adafruit_LSM9DS0::lsm9ds0MagGain_t)gain);
}

bool LSM9DS0::read() {
    lsm.read();
    
    // Get raw sensor events
    lsm.getEvent(&accel_event, &mag_event, &gyro_event, &temp_event);
    
    return true;
}

Vector3 LSM9DS0::getAccel() const {
    // Convert from m/s² to G, apply offset, then back to m/s²
    Vector3 accel(
        (accel_event.acceleration.x / G_TO_MS2) - accel_offset.x,
        (accel_event.acceleration.y / G_TO_MS2) - accel_offset.y,
        (accel_event.acceleration.z / G_TO_MS2) - accel_offset.z
    );
    
    // Convert back to m/s²
    return accel * G_TO_MS2;
}

Vector3 LSM9DS0::getGyro() const {
    Vector3 gyro(
        gyro_event.gyro.x * DEG_TO_RAD - gyro_offset.x,
        gyro_event.gyro.y * DEG_TO_RAD - gyro_offset.y,
        gyro_event.gyro.z * DEG_TO_RAD - gyro_offset.z
    );
    
    return gyro;
}

Vector3 LSM9DS0::getMag() const {
    // Apply calibration offsets and scale factors
    Vector3 mag(
        (mag_event.magnetic.x - mag_offset.x) * mag_scale.x,
        (mag_event.magnetic.y - mag_offset.y) * mag_scale.y,
        (mag_event.magnetic.z - mag_offset.z) * mag_scale.z
    );
    
    // Convert to uT
    return mag * GAUSS_TO_UT;
}

void LSM9DS0::calibrate(uint16_t samples) {
    Serial.println("Starting IMU calibration...");
    
    Vector3 accel_sum(0, 0, 0);
    Vector3 gyro_sum(0, 0, 0);
    Vector3 mag_min(1000, 1000, 1000);
    Vector3 mag_max(-1000, -1000, -1000);
    
    for (uint16_t i = 0; i < samples; i++) {
        if (read()) {
            Vector3 accel = getAccel();
            Vector3 gyro = getGyro();
            Vector3 mag = getMag();
            
            accel_sum = accel_sum + accel;
            gyro_sum = gyro_sum + gyro;
            
            // Find min/max for magnetometer
            mag_min.x = fmin(mag_min.x, mag.x);
            mag_min.y = fmin(mag_min.y, mag.y);
            mag_min.z = fmin(mag_min.z, mag.z);
            
            mag_max.x = fmax(mag_max.x, mag.x);
            mag_max.y = fmax(mag_max.y, mag.y);
            mag_max.z = fmax(mag_max.z, mag.z);
        }
        
        delay(10);
        
        if (i % 100 == 0) {
            Serial.print(".");
        }
    }
    
    // Calculate averages for accelerometer and gyroscope
    accel_offset = accel_sum / samples;
    gyro_offset = gyro_sum / samples;
    
    // Calculate magnetometer calibration
    mag_offset = (mag_min + mag_max) * 0.5;
    mag_scale.x = (mag_max.x - mag_min.x) * 0.5;
    mag_scale.y = (mag_max.y - mag_min.y) * 0.5;
    mag_scale.z = (mag_max.z - mag_min.z) * 0.5;
    
    // Normalize scale factors
    float avg_scale = (mag_scale.x + mag_scale.y + mag_scale.z) / 3.0;
    mag_scale = mag_scale / avg_scale;
    
    Serial.println("\nCalibration complete!");
    Serial.print("Accel offset: "); Serial.println(accel_offset.toString());
    Serial.print("Gyro offset: "); Serial.println(gyro_offset.toString());
    Serial.print("Mag offset: "); Serial.println(mag_offset.toString());
    Serial.print("Mag scale: "); Serial.println(mag_scale.toString());
}

void LSM9DS0::setAccelOffset(const Vector3& offset) {
    accel_offset = offset;
}

void LSM9DS0::setGyroOffset(const Vector3& offset) {
    gyro_offset = offset;
}

void LSM9DS0::setMagCalibration(const Vector3& offset, const Vector3& scale) {
    mag_offset = offset;
    mag_scale = scale;
}