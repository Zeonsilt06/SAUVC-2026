#ifndef THRUSTER_CONTROL_H
#define THRUSTER_CONTROL_H

#include <Arduino.h>
#include <Servo.h>

void initMotor();
void updateMotors(int yOut, int rOut, int dOut); // Tambahkan parameter untuk mixing
void stopAll();

/* Perintah Gerak */
void setForwardPower(int power); 
void setDepthTarget(double target);
void setYawTarget(double target);

#endif