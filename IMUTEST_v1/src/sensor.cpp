#include "sensor.h"

Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0();

bool initializeSensor() {
    Serial.println("LSM9DS0 9DOF - STABLE VERSION");
    Serial.println("");
    
    if(!lsm.begin()) {
        Serial.println("ERROR: Sensor tidak ditemukan!");
        Serial.println("Periksa koneksi I2C:");
        Serial.println("  VIN  -> 3.3V");
        Serial.println("  GND  -> GND");
        Serial.println("  SCL  -> Pin 19");
        Serial.println("  SDA  -> Pin 18");
        return false;
    }
    
    Serial.println("✓ Sensor LSM9DS0 ditemukan!");
    return true;
}

void setupSensor() {
    lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G);
    lsm.setupMag(lsm.LSM9DS0_MAGGAIN_2GAUSS);
    lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS);
}

void readSensorData(sensors_event_t &accel, sensors_event_t &mag, 
                    sensors_event_t &gyro, sensors_event_t &temp) {
    lsm.read();
    lsm.getEvent(&accel, &mag, &gyro, &temp);
}