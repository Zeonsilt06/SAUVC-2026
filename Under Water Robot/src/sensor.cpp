#include "sensor.h"
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_LSM9DS0.h>
#include <Adafruit_Sensor.h>

// ============================
// LSM9DS0 OBJECT
// ============================
Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(1000); // ID bebas

// ============================
// SENSOR INIT
// ============================
void sensor_init() {
    Wire.begin();

    if (!lsm.begin()) {
        Serial.println("LSM9DS0 NOT detected");
        while (1);
    }
    Serial.println("LSM9DS0 detected");

    // Configure ranges (mirip default Adafruit Python)
    lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G);
    lsm.setupMag(lsm.LSM9DS0_MAGGAIN_2GAUSS);
    lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS);
}

// ============================
// READ RAW LSM9DS0 (DOSEN MODE)
// ============================
void read_lsm9ds0(
    float &ax, float &ay, float &az,
    float &mx, float &my, float &mz,
    float &gx, float &gy, float &gz,
    float &temp
) {
    lsm.read();

    // Acceleration (m/s^2)
    ax = lsm.accelData.x;
    ay = lsm.accelData.y;
    az = lsm.accelData.z;

    // Magnetometer (gauss)
    mx = lsm.magData.x;
    my = lsm.magData.y;
    mz = lsm.magData.z;

    // Gyroscope (deg/s)
    gx = lsm.gyroData.x;
    gy = lsm.gyroData.y;
    gz = lsm.gyroData.z;

    // Temperature (°C)
    temp = lsm.temperature;
}

// ============================
// SIMPLE IMU (PLACEHOLDER)
// ============================
void read_imu(float &yaw, float &pitch, float &roll) {
    float ax, ay, az;
    float mx, my, mz;
    float gx, gy, gz;
    float temp;

    read_lsm9ds0(ax, ay, az, mx, my, mz, gx, gy, gz, temp);

    // sementara: yaw/pitch/roll dummy
    yaw   = gx;
    pitch = ay;
    roll  = ax;
}

// ============================
// SERIAL OUTPUT (REF DOSEN)
// ============================
void print_imu_serial() {
    float ax, ay, az;
    float mx, my, mz;
    float gx, gy, gz;
    float temp;

    read_lsm9ds0(ax, ay, az, mx, my, mz, gx, gy, gz, temp);

    Serial.print("Acceleration (m/s^2): (");
    Serial.print(ax, 3); Serial.print(",");
    Serial.print(ay, 3); Serial.print(",");
    Serial.print(az, 3); Serial.println(")");

    Serial.print("Magnetometer (gauss): (");
    Serial.print(mx, 3); Serial.print(",");
    Serial.print(my, 3); Serial.print(",");
    Serial.print(mz, 3); Serial.println(")");

    Serial.print("Gyroscope (deg/s): (");
    Serial.print(gx, 3); Serial.print(",");
    Serial.print(gy, 3); Serial.print(",");
    Serial.print(gz, 3); Serial.println(")");

    Serial.print("Temperature: ");
    Serial.print(temp, 3);
    Serial.println(" C");

    Serial.println();
}
