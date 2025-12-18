#include "sensor.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM9DS0.h>
#include <SparkFun_MS5803_I2C.h>

// ==================
// LSM9DS0
// ==================
Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(1000); // ID bebas

// ==================
// PRESSURE SENSOR
// ==================
MS5803 pressure(ADDRESS_HIGH); // ganti ADDRESS_LOW jika perlu

// ==================
// INIT SENSOR
// ==================
void sensor_init() {
    Wire.begin();
    delay(100);

    // ===== LSM9DS0 INIT =====
    if (!lsm.begin()) {
        Serial.println("LSM9DS0 NOT detected");
        while (1); // berhenti → sensor wajib ada
    }
    Serial.println("LSM9DS0 detected");

    // Setup range (default aman)
    lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G);
    lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS);
    lsm.setupMag(lsm.LSM9DS0_MAGGAIN_2GAUSS);

    // ===== PRESSURE INIT =====
    if (pressure.begin()) {
        Serial.println("MS5803 detected");
    } else {
        Serial.println("MS5803 NOT detected");
    }
}

// ==================
// READ IMU (UNTUK PID)
// ==================
void read_imu(float &yaw, float &pitch, float &roll) {
    sensors_event_t accel, mag, gyro, temp;
    lsm.getEvent(&accel, &mag, &gyro, &temp);

    // Sementara:
    // PID pakai gyro (lebih stabil untuk heading)
    roll  = gyro.gyro.x;
    pitch = gyro.gyro.y;
    yaw   = gyro.gyro.z;
}

// ==================
// TAMPILKAN DATA LSM9DS0 KE SERIAL
// ==================
void print_imu_serial() {
    sensors_event_t accel, mag, gyro, temp;
    lsm.getEvent(&accel, &mag, &gyro, &temp);

    Serial.print("ACC (m/s^2): ");
    Serial.print(accel.acceleration.x); Serial.print(", ");
    Serial.print(accel.acceleration.y); Serial.print(", ");
    Serial.print(accel.acceleration.z);

    Serial.print(" | GYRO (rad/s): ");
    Serial.print(gyro.gyro.x); Serial.print(", ");
    Serial.print(gyro.gyro.y); Serial.print(", ");
    Serial.print(gyro.gyro.z);

    Serial.print(" | MAG (uT): ");
    Serial.print(mag.magnetic.x); Serial.print(", ");
    Serial.print(mag.magnetic.y); Serial.print(", ");
    Serial.print(mag.magnetic.z);

    Serial.println();
}

// ==================
// READ DEPTH
// ==================
float read_depth() {
    float pressure_mbar = pressure.getPressure(ADC_4096);

    float depth = (pressure_mbar - 1013.25f) / 100.0f;
    if (depth < 0) depth = 0;

    return depth;
}
