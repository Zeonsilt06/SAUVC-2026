#include "ThrusterControl.h"

static int currentPWM[4] = {1500, 1500, 1500, 1500};
static int targetPWM[4]  = {1500, 1500, 1500, 1500};
static int power_gerak = 0; // Positif = Maju, Negatif = Mundur

static Servo thrus_1, thrus_2, thrus_3, thrus_4;

void initMotor() {
    thrus_1.attach(2); thrus_2.attach(5); 
    thrus_3.attach(4); thrus_4.attach(3);
    stopAll();
}

void setForwardPower(int power) {
    power_gerak = power; // Menyimpan input maju/mundur
}

void stopAll() {
    power_gerak = 0;
    for(int i=0; i<4; i++) targetPWM[i] = 1500;
}

// Fungsi Update dengan MIXING LOGIC
void updateMotors(int yOut, int rOut, int dOut) {
    // 1. MIXING HORIZONTAL (Motor 1 & 2)
    // Maju/Mundur digabung dengan koreksi Belok (Yaw)
    targetPWM[0] = 1500 + power_gerak + yOut; // Motor Kiri
    targetPWM[2] = 1500 + power_gerak - yOut; // Motor Kanan

    // 2. MIXING VERTIKAL (Motor 3 & 4) - Posisi No. 2
    // Kedalaman (dOut) digabung dengan koreksi Miring (Roll)
    targetPWM[1] = 1500 + dOut + rOut; // Vertikal Kiri
    targetPWM[3] = 1500 - dOut + rOut; // Vertikal Kanan

    // Slew Rate (Pergerakan Halus)
    for(int i = 0; i < 4; i++) {
        if (currentPWM[i] < targetPWM[i]) currentPWM[i] += 10;
        else if (currentPWM[i] > targetPWM[i]) currentPWM[i] -= 10;
        if (abs(currentPWM[i] - targetPWM[i]) < 10) currentPWM[i] = targetPWM[i];
    }

    thrus_1.writeMicroseconds(constrain(currentPWM[0], 1100, 1900));
    thrus_2.writeMicroseconds(constrain(currentPWM[1], 1100, 1900));
    thrus_3.writeMicroseconds(constrain(currentPWM[2], 1100, 1900));
    thrus_4.writeMicroseconds(constrain(currentPWM[3], 1100, 1900));
}