#ifndef MOTOR_H
#define MOTOR_H

void motor_init();
void stop_all();

// Gerak dasar
void forward(int speed);
void yaw_right(int speed);
void yaw_left(int speed);

// (nanti depth, sekarang dummy dulu)
void depth_up(int speed);
void depth_down(int speed);

#endif
