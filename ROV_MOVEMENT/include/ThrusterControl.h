#ifndef THRUSTER_CONTROL_H
#define THRUSTER_CONTROL_H

#include <Arduino.h>
#include <Servo.h>

/* ================= CONSTANTS ================= */
#define STOP_PWM 1500

/* ================= API ================= */
void initMotor();
void stopAll();

/* Basic Movement */
void forward(int offset);
void reverse(int offset);

/* Rotation */
void rotateRight(int offset);
void rotateLeft(int offset);

/* Vertical */
void up(int offset);
void down(int offset);

/* Combined */
void rotateRightUp(int offsetH, int offsetV);
void rotateLeftUp(int offsetH, int offsetV);
void rotateRightDown(int offsetH, int offsetV);
void rotateLeftDown(int offsetH, int offsetV);

#endif