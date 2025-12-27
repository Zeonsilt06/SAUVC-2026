#include "imu_controller.h"

#define SENSOR_ID 9000
#define ALPHA 0.98  // Complementary filter coefficient

IMUController::IMUController() : 
    lsm(SENSOR_ID),
    gyroOffsetX(0), gyroOffsetY(0), gyroOffsetZ(0),
    magOffsetX(0), magOffsetY(0), magOffsetZ(0),
    accelOffsetZ(0),
    filteredRoll(0), filteredPitch(0),
    lastUpdateTime(0)
{
    currentData.valid = false;
}

bool IMUController::begin() {
    Serial.println("[IMU] Initializing LSM9DS0...");
    
    if (!lsm.begin()) {
        Serial.println("[IMU] ERROR: Failed to initialize!");
        Serial.println("[IMU] Check wiring:");
        Serial.println("  SDA -> Pin 18 (Teensy 4.1)");
        Serial.println("  SCL -> Pin 19 (Teensy 4.1)");
        Serial.println("  VIN -> 3.3V");
        Serial.println("  GND -> GND");
        return false;
    }
    
    // Configure sensor ranges
    lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G);
    lsm.setupMag(lsm.LSM9DS0_MAGGAIN_2GAUSS);
    lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS);
    
    Serial.println("[IMU] LSM9DS0 initialized successfully");
    
    lastUpdateTime = millis();
    return true;
}

bool IMUController::calibrate(uint16_t duration_ms) {
    Serial.println("[IMU] Calibration started...");
    Serial.println("[IMU] Keep ROV STILL and LEVEL!");
    
    float sumGyroX = 0, sumGyroY = 0, sumGyroZ = 0;
    float sumMagX = 0, sumMagY = 0, sumMagZ = 0;
    float sumAccelZ = 0;
    int samples = 0;
    
    unsigned long startTime = millis();
    
    while (millis() - startTime < duration_ms) {
        lsm.read();
        
        // Accumulate gyro data
        sumGyroX += lsm.gyroData.x;
        sumGyroY += lsm.gyroData.y;
        sumGyroZ += lsm.gyroData.z;
        
        // Accumulate magnetometer data
        sumMagX += lsm.magData.x;
        sumMagY += lsm.magData.y;
        sumMagZ += lsm.magData.z;
        
        // Accumulate accelerometer Z
        sumAccelZ += lsm.accelData.z;
        
        samples++;
        delay(10);
        
        // Progress indicator
        if (samples % 50 == 0) {
            Serial.print(".");
        }
    }
    Serial.println();
    
    if (samples > 0) {
        // Calculate offsets
        gyroOffsetX = sumGyroX / samples;
        gyroOffsetY = sumGyroY / samples;
        gyroOffsetZ = sumGyroZ / samples;
        
        magOffsetX = sumMagX / samples;
        magOffsetY = sumMagY / samples;
        // Don't offset magZ as we need it for tilt compensation
        
        accelOffsetZ = (sumAccelZ / samples) - 9.81;  // Should be ~9.81 m/s²
        
        Serial.println("[IMU] Calibration complete!");
        Serial.print("[IMU] Gyro offsets: ");
        Serial.print(gyroOffsetX); Serial.print(", ");
        Serial.print(gyroOffsetY); Serial.print(", ");
        Serial.println(gyroOffsetZ);
        
        return true;
    }
    
    Serial.println("[IMU] ERROR: Calibration failed!");
    return false;
}

bool IMUController::update() {
    unsigned long currentTime = millis();
    float dt = (currentTime - lastUpdateTime) / 1000.0;  // seconds
    lastUpdateTime = currentTime;
    
    if (dt > 1.0) {  // Sanity check
        dt = 0.01;
    }
    
    // Read raw sensor data
    readRawData();
    
    // Calculate orientation
    calculateOrientation();
    
    currentData.timestamp = currentTime;
    currentData.valid = true;
    
    return true;
}

void IMUController::readRawData() {
    lsm.read();
    
    // Accelerometer (m/s²)
    currentData.accelX = lsm.accelData.x;
    currentData.accelY = lsm.accelData.y;
    currentData.accelZ = lsm.accelData.z - accelOffsetZ;
    
    // Gyroscope (rad/s) with offset compensation
    currentData.gyroX = lsm.gyroData.x - gyroOffsetX;
    currentData.gyroY = lsm.gyroData.y - gyroOffsetY;
    currentData.gyroZ = lsm.gyroData.z - gyroOffsetZ;
    
    // Magnetometer (gauss) with offset compensation
    currentData.magX = lsm.magData.x - magOffsetX;
    currentData.magY = lsm.magData.y - magOffsetY;
    currentData.magZ = lsm.magData.z;  // No offset for Z
}

void IMUController::calculateOrientation() {
    // Calculate roll and pitch from accelerometer
    float accelRoll = atan2(currentData.accelY, currentData.accelZ) * 180.0 / PI;
    float accelPitch = atan2(-currentData.accelX, 
                             sqrt(currentData.accelY * currentData.accelY + 
                                  currentData.accelZ * currentData.accelZ)) * 180.0 / PI;
    
    // Integrate gyroscope data
    unsigned long currentTime = millis();
    float dt = (currentTime - lastUpdateTime) / 1000.0;
    
    float gyroRoll = filteredRoll + currentData.gyroX * dt * 180.0 / PI;
    float gyroPitch = filteredPitch + currentData.gyroY * dt * 180.0 / PI;
    
    // Complementary filter
    filteredRoll = ALPHA * gyroRoll + (1.0 - ALPHA) * accelRoll;
    filteredPitch = ALPHA * gyroPitch + (1.0 - ALPHA) * accelPitch;
    
    currentData.roll = normalizeAngle(filteredRoll);
    currentData.pitch = normalizeAngle(filteredPitch);
    
    // Calculate yaw (heading) from magnetometer with tilt compensation
    currentData.yaw = calculateHeading(currentData.magX, currentData.magY, 
                                       currentData.roll * PI / 180.0, 
                                       currentData.pitch * PI / 180.0);
}

float IMUController::calculateHeading(float magX, float magY, float roll, float pitch) {
    // Tilt compensation for magnetometer
    float magXComp = magX * cos(pitch) + magY * sin(roll) * sin(pitch) + 
                     currentData.magZ * cos(roll) * sin(pitch);
    float magYComp = magY * cos(roll) - currentData.magZ * sin(roll);
    
    // Calculate heading
    float heading = atan2(magYComp, magXComp) * 180.0 / PI;
    
    return normalizeAngle(heading);
}

float IMUController::normalizeAngle(float angle) {
    while (angle > 180.0) angle -= 360.0;
    while (angle < -180.0) angle += 360.0;
    return angle;
}

IMUData IMUController::getData() {
    return currentData;
}

float IMUController::getRoll() {
    return currentData.roll;
}

float IMUController::getPitch() {
    return currentData.pitch;
}

float IMUController::getYaw() {
    return currentData.yaw;
}

void IMUController::resetOrientation() {
    filteredRoll = 0;
    filteredPitch = 0;
    lastUpdateTime = millis();
}

bool IMUController::isHealthy() {
    if (!currentData.valid) return false;
    
    unsigned long age = millis() - currentData.timestamp;
    if (age > 1000) return false;  // Data too old
    
    // Check for reasonable values
    if (abs(currentData.roll) > 180 || abs(currentData.pitch) > 90) return false;
    
    return true;
}

void IMUController::printData() {
    Serial.print("[IMU] Roll: "); Serial.print(currentData.roll, 1);
    Serial.print("° | Pitch: "); Serial.print(currentData.pitch, 1);
    Serial.print("° | Yaw: "); Serial.print(currentData.yaw, 1);
    Serial.println("°");
}