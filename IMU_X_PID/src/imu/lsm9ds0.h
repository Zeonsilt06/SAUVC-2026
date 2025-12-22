#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM9DS0.h>

#include "../utils/Vector3.h"

// Define I2C addresses if not already defined
#ifndef LSM9DS0_XG_ADDR
    #define LSM9DS0_XG_ADDR 0x6B
#endif
#ifndef LSM9DS0_M_ADDR
    #define LSM9DS0_M_ADDR 0x1D
#endif

class LSM9DS0 {
private:
    Adafruit_LSM9DS0 lsm;
    sensors_event_t accel_event, gyro_event, mag_event, temp_event;
    
    // Calibration offsets
    Vector3 accel_offset = Vector3(0, 0, 0);
    Vector3 gyro_offset = Vector3(0, 0, 0);
    Vector3 mag_offset = Vector3(0, 0, 0);
    Vector3 mag_scale = Vector3(1, 1, 1);
    
    // Conversion factors
    const float G_TO_MS2 = 9.80665;
    const float DEG_TO_RAD_FACTOR = 0.01745329251;
    const float GAUSS_TO_UT = 100.0;
    
public:
    LSM9DS0();
    bool begin();
    
    // Configuration
    void setAccelRange(uint8_t accelRange);
    void setGyroRange(uint8_t gyroScale);
    void setMagRange(uint8_t magGain);
    
    // Data reading
    bool read();
    Vector3 getAccel() const;   // Returns in m/s²
    Vector3 getGyro() const;    // Returns in rad/s
    Vector3 getMag() const;     // Returns in uT
    
    // Calibration
    void calibrate(uint16_t samples = 500);
    void setAccelOffset(const Vector3& offset);
    void setGyroOffset(const Vector3& offset);
    void setMagCalibration(const Vector3& offset, const Vector3& scale);
    
private:
    void readRawData();
    Vector3 convertAccel(const sensors_event_t& event);
    Vector3 convertGyro(const sensors_event_t& event);
    Vector3 convertMag(const sensors_event_t& event);
};