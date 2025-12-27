#include "Control.h"
#include "Config.h"
#include "PID.h"

// Definisi variabel global
int manThr = 0;
int manTurn = 0;
int manVert = 0;

// Objek PID
PID pid1_depth(15.0, 0.5, 2.0, -50, 50);

void updatePID1_Vertical(float dpt, float r, float dt, MixedMotorDriver &motors) {
    float dCorr = 0;
    float dCM = dpt * 100.0;
    
    if (dCM > 100.0) dCorr = pid1_depth.compute(100.0, dCM, dt);
    else if (dCM < 90.0) dCorr = pid1_depth.compute(90.0, dCM, dt);
    else { dCorr = 0; pid1_depth.reset(); }
    
    motors.setVertical(manVert + (int)dCorr, 0);
}

void updatePID2_Horizontal(float y, float dt, MixedMotorDriver &motors) {
    motors.setHorizontal(manThr, manTurn);
}