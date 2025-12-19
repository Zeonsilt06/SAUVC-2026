#ifndef MOTOR_H
#define MOTOR_H

#include <Servo.h>

void motor_init();
void stop_all();
void forward(int speed);
void yaw_right(int speed);
void set_vertical(int ka, int ki);

#endif
