#ifndef SENSOR_H
#define SENSOR_H

void sensor_init();

// IMU
void read_imu(float &yaw, float &pitch, float &roll);

// Pressure
float read_depth(); // meter

void print_imu_serial();

#endif
