#include "sensor_handler.h"

// Inisialisasi objek sensor (externally linkable)
Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0();

bool initSensor() {
    if (!lsm.begin()) return false;
    lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G);
    lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS);
    lsm.setupMag(lsm.LSM9DS0_MAGGAIN_2GAUSS);
    return true;
}

void getRawData(float &ax, float &ay, float &az, float &gx, float &gy, float &gz, float &mx, float &my, float &mz) {
    sensors_event_t accel, mag, gyro, temp;
    lsm.getEvent(&accel, &mag, &gyro, &temp);

    ax = accel.acceleration.x;
    ay = accel.acceleration.y;
    az = accel.acceleration.z;
    
    gx = gyro.gyro.x * 57.29578f;
    gy = gyro.gyro.y * 57.29578f;
    gz = gyro.gyro.z * 57.29578f;

    mx = mag.magnetic.x;
    my = mag.magnetic.y;
    mz = mag.magnetic.z;
}