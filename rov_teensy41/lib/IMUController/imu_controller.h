#ifndef IMU_CONTROLLER_H
#define IMU_CONTROLLER_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_LSM9DS0.h>
#include <Adafruit_Sensor.h>

struct IMUData {
    // Orientation (Euler angles)
    float roll;     // X-axis rotation (-180 to 180)
    float pitch;    // Y-axis rotation (-90 to 90)
    float yaw;      // Z-axis rotation (heading, -180 to 180)
    
    // Raw sensor data
    float accelX, accelY, accelZ;   // m/s²
    float gyroX, gyroY, gyroZ;      // rad/s
    float magX, magY, magZ;         // gauss
    
    // Status
    bool valid;
    unsigned long timestamp;
};

class IMUController {
public:
    IMUController();
    
    // Initialization
    bool begin();
    bool calibrate(uint16_t duration_ms = 3000);
    
    // Data acquisition
    bool update();
    IMUData getData();
    
    // Getters for individual values
    float getRoll();
    float getPitch();
    float getYaw();
    
    // Utility
    void resetOrientation();
    bool isHealthy();
    void printData();
    
private:
    Adafruit_LSM9DS0 lsm;
    IMUData currentData;
    
    // Calibration offsets
    float gyroOffsetX, gyroOffsetY, gyroOffsetZ;
    float magOffsetX, magOffsetY, magOffsetZ;
    float accelOffsetZ;
    
    // Complementary filter variables
    float filteredRoll, filteredPitch;
    unsigned long lastUpdateTime;
    
    // Private methods
    void readRawData();
    void calculateOrientation();
    float normalizeAngle(float angle);
    float calculateHeading(float magX, float magY, float roll, float pitch);
};

#endif // IMU_CONTROLLER_H