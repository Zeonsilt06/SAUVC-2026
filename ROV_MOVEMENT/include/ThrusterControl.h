#ifndef THRUSTER_CONTROL_H
#define THRUSTER_CONTROL_H

#include <Arduino.h>
#include <Servo.h>

/* ================= CONSTANTS ================= */
#define STOP_PWM 1500

/* ================= API ================= */
void initMotor();
void updateMotors();
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

#endif // <--- BARIS INI WAJIB ADA UNTUK MENUTUP #ifndef DI ATAS