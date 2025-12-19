#ifndef SENSOR_H
#define SENSOR_H

void sensor_init();

// ============================
// LSM9DS0 RAW DATA
// ============================
void read_lsm9ds0(
    float &ax, float &ay, float &az,
    float &mx, float &my, float &mz,
    float &gx, float &gy, float &gz,
    float &temp
);

// ============================
// IMU (YPR dari LSM9DS0)
// ============================
void read_imu(float &yaw, float &pitch, float &roll);

// ============================
// SERIAL OUTPUT
// ============================
void print_imu_serial();

// ============================
// PRESSURE
// ============================
float read_depth();

#endif
