#include "sensor.h"
#include <Wire.h>
#include <Adafruit_LSM9DS0.h>
#include <MadgwickAHRS.h>

Adafruit_LSM9DS0 lsm(1000);
Madgwick filter;

const float SAMPLE_FREQ = 100.0f;

void sensor_init() {
    Wire.begin();

    if (!lsm.begin()) {
        Serial.println("LSM9DS0 NOT detected");
        while (1);
    }

    lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G);
    lsm.setupMag(lsm.LSM9DS0_MAGGAIN_2GAUSS);
    lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS);

    filter.begin(SAMPLE_FREQ);

    Serial.println("LSM9DS0 + Madgwick READY");
}

void read_imu(float &yaw, float &pitch, float &roll) {
    sensors_event_t accel, mag, gyro, temp;
    lsm.getEvent(&accel, &mag, &gyro, &temp);

    float gx = gyro.gyro.x * RAD_TO_DEG;
    float gy = gyro.gyro.y * RAD_TO_DEG;
    float gz = gyro.gyro.z * RAD_TO_DEG;

    filter.update(
        gx, gy, gz,
        accel.acceleration.x,
        accel.acceleration.y,
        accel.acceleration.z,
        mag.magnetic.x,
        mag.magnetic.y,
        mag.magnetic.z
    );

    roll  = constrain(filter.getRoll()  + 90.0f, 0, 180);
    pitch = constrain(filter.getPitch() + 90.0f, 0, 180);
    yaw   = fmod(filter.getYaw() + 360.0f, 360.0f) / 2.0f;
}

void print_imu_serial() {
    float y, p, r;
    read_imu(y, p, r);

    Serial.print(y); Serial.print(",");
    Serial.print(p); Serial.print(",");
    Serial.println(r);
}
