#ifndef SENSOR_H
#define SENSOR_H

void sensor_init();
void read_imu(float &yaw, float &pitch, float &roll);
void print_imu_serial();
float read_depth();



#endif
