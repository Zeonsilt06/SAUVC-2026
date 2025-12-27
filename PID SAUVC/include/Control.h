#ifndef CONTROL_H
#define CONTROL_H

#include "MixedMotorDriver.h"

// Memberitahu compiler bahwa variabel ini didefinisikan di file lain (Control.cpp)
extern int manThr;
extern int manTurn;
extern int manVert;

// Deklarasi fungsi agar bisa dipanggil di main.cpp
void updatePID1_Vertical(float dpt, float r, float dt, MixedMotorDriver &motors);
void updatePID2_Horizontal(float y, float dt, MixedMotorDriver &motors);

#endif